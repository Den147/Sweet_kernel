/*
 * Copyright (C) 2001, Russell King.
 * Copyright (C) 2003, Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>
 * Copyright (C) 2003, Jun Nakajima <jun.nakajima@intel.com>
 * Copyright (C) 2009, Alexander Clouter <alex@digriz.org.uk>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include "cpufreq_governor.h"

#define DEF_FREQUENCY_UP_THRESHOLD		(80)
#define DEF_FREQUENCY_UP_THRESHOLD_BURST	(95)
#define DEF_FREQUENCY_DOWN_THRESHOLD		(20)
#define DEF_FREQUENCY_UP_STEP			(5)
#define DEF_FREQUENCY_DOWN_STEP			(10)
#define DEF_FREQUENCY_SAMPLING_DOWN_FACTOR	(1)

#define SCALE_UP				(0)
#define SCALE_DOWN				(1)

static DEFINE_PER_CPU(struct cs_cpu_dbs_info_s, cs_cpu_dbs_info);
define_get_cpu_dbs_routines(cs_cpu_dbs_info);

static struct cs_dbs_tuners cs_tuners = {
	.sampling_down_factor = DEF_FREQUENCY_SAMPLING_DOWN_FACTOR,
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.up_threshold_burst = DEF_FREQUENCY_UP_THRESHOLD_BURST,
	.down_threshold = DEF_FREQUENCY_DOWN_THRESHOLD,
	.freq_up_step = DEF_FREQUENCY_UP_STEP,
	.freq_down_step = DEF_FREQUENCY_DOWN_STEP,
};

/* Minimal sampling rate supported by hardware and aligned with software */
static unsigned int __read_mostly min_sampling_rate;

/* Number of cpus that currently use this governor */
static unsigned int gov_enable_cnt;

/* Workqueue used to run this governor on */
static struct workqueue_struct *cs_wq;

/* Mutex that protect governor start/stop routines */
static DEFINE_MUTEX(cs_mutex);

static inline u32 get_policy_max_load(struct cpufreq_policy *policy)
{
	struct cpu_dbs_common_info *j_cdbs;
	u32 max_load = 0, load, load_at_max_freq;
	int cpu;

	for_each_cpu(cpu, policy->cpus) {
		u64 cur_idle_time, cur_wall_time;
		u32 idle_time, wall_time;

		/* Some targets want iowait time to be subtracted from idle */
		cur_idle_time = get_cpu_idle_time(cpu, &cur_wall_time,
						  cs_tuners.io_is_busy);

		j_cdbs = get_cpu_cdbs(cpu);
		idle_time = (u32)(cur_idle_time - j_cdbs->prev_cpu_idle);
		j_cdbs->prev_cpu_idle = cur_idle_time;

		wall_time = (u32)(cur_wall_time - j_cdbs->prev_cpu_wall);
		j_cdbs->prev_cpu_wall = cur_wall_time;

		if (unlikely(!wall_time)) {
			/*
			 * That can only happen when this function is called
			 * twice in a row with a very short interval between
			 * the calls, so the previous load value can be used
			 * then.
			 */
			load = j_cdbs->prev_load;
		} else if (unlikely(wall_time < idle_time)) {
			/*
			 * That can happen if idle_time is returned by
			 * get_cpu_idle_time_jiffy(). In that case idle_time
			 * is roughly equal to the difference between wall_time
			 * and "busy time" obtained from CPU statistics. Then,
			 * the "busy time" can end up being greater than
			 * wall_time (for example, if jiffies_64 and the CPU
			 * statistics are updated by different CPUs), so
			 * idle_time may in fact be negative. That means,
			 * though, that the CPU was busy all the time (on the
			 * rough average) during the last sampling interval and
			 * 100 can be returned as the load.
			 */
			load = (int)idle_time < 0 ? 100 : 0;
		} else {
			/*
			 * All the required data is valid. We can calculate
			 * current load of a CPU in an ordinary way without
			 * hesitating about a theoretical failure.
			 */
			load = 100 * (wall_time - idle_time) / wall_time;
		}

		if (unlikely(wall_time > cs_tuners.sampling_rate * 2 &&
		    load < j_cdbs->prev_load)) {
			/*
			 * If the CPU had gone completely idle and a task has
			 * just woken up on this CPU now, it would be unfair to
			 * calculate 'load' the usual way for this elapsed
			 * time-window, because it would show near-zero load,
			 * irrespective of how CPU intensive that task actually
			 * was. This is undesirable for latency-sensitive bursty
			 * workloads.
			 *
			 * To avoid this, reuse the 'load' from the previous
			 * time-window and give this task a chance to start with
			 * a reasonably high CPU frequency.  However, that
			 * shouldn't be over-done, lest we get stuck at a high
			 * load (high frequency) for too long, even when the
			 * current system load has actually dropped down, so
			 * clear prev_load to guarantee that the load will be
			 * computed again next time.
			 *
			 * Detecting this situation is easy: the governor's
			 * utilization update handler would not have run during
			 * CPU-idle periods.  Hence, an unusually large
			 * 'wall_time' (as compared to the sampling rate)
			 * indicates this scenario.
			 */
			load = j_cdbs->prev_load;
			j_cdbs->prev_load = 0;
		} else {
			j_cdbs->prev_load = load;
		}

		max_load = max(max_load, load);
	}

	/* Report the policy utilization to userspace */
	load_at_max_freq = max_load * policy->cur / policy->max;
	cpufreq_notify_utilization(policy, load_at_max_freq);

	return max_load;
}

static inline void scale_freq(struct cs_cpu_dbs_info_s *dbs_info,
			      struct cpufreq_policy *policy, u32 decrease)
{
	u32 freq_diff;

	/* This function is called in non-burst scenarios only */
	dbs_info->rate_mult = 1;

	if (policy->cur == (decrease ? policy->min : policy->max))
		return;

	freq_diff = max_t(u32, (policy->max * (decrease ?
		cs_tuners.freq_down_step : cs_tuners.freq_up_step) / 100), 5);

	dbs_info->target_freq += (decrease ? -freq_diff : freq_diff);
	dbs_info->target_freq  = clamp_t(int, dbs_info->target_freq,
					 policy->min, policy->max);

	__cpufreq_driver_target(policy, dbs_info->target_freq,
				CPUFREQ_RELATION_C);
}

static inline void cs_check_cpu(struct cs_cpu_dbs_info_s *dbs_info)
{
	struct cpufreq_policy *policy = dbs_info->cdbs.cur_policy;
	u32 burst_threshold = cs_tuners.up_threshold_burst;
	u32 max_load = get_policy_max_load(policy);

	if (burst_threshold && max_load >= burst_threshold) {
		if (policy->cur < policy->max)
			dbs_info->rate_mult = cs_tuners.sampling_down_factor;

		dbs_info->target_freq = policy->max;
		switch_freq(policy, dbs_info->target_freq);
		return;
	}

	if (max_load >= cs_tuners.up_threshold)
		scale_freq(dbs_info, policy, SCALE_UP);
	else if (max_load <= cs_tuners.down_threshold)
		scale_freq(dbs_info, policy, SCALE_DOWN);
}

static void cs_dbs_timer(struct work_struct *work)
{
	struct cs_cpu_dbs_info_s *dbs_info =
		container_of(work, struct cs_cpu_dbs_info_s, cdbs.work.work);
	int delay, cpu = dbs_info->cdbs.cpu;

	mutex_lock(&dbs_info->cdbs.timer_mutex);
	cs_check_cpu(dbs_info);

	delay = align_delay(cs_tuners.sampling_rate, dbs_info->rate_mult);
	queue_delayed_work_on(cpu, cs_wq, &dbs_info->cdbs.work, delay);
	mutex_unlock(&dbs_info->cdbs.timer_mutex);
}

static inline void cs_timer_init(struct cpu_dbs_common_info *cdbs)
{
	int delay = align_delay(cs_tuners.sampling_rate, 1);

	INIT_DEFERRABLE_WORK(&cdbs->work, cs_dbs_timer);
	queue_delayed_work_on(cdbs->cpu, cs_wq, &cdbs->work, delay);
}

static inline void cs_timer_exit(struct cpu_dbs_common_info *cdbs)
{
	cancel_delayed_work_sync(&cdbs->work);
}

/**
 * update_sampling_rate() - update sampling rate effective immediately.
 * @new_rate: new sampling rate
 *
 * If new sampling rate is smaller than the old, simply updaing sampling_rate
 * might not be appropriate. For example, if the original sampling_rate was 1
 * second and the requested new sampling rate is 10 ms because the user needs
 * immediate reaction from ondemand governor, but not sure if higher frequency
 * will be required or not, then the governor may change the sampling rate too
 * late, up to 1 second later.  Thus, if we are reducing the sampling rate, we
 * need to make the new value effective immediately.
 */
static inline void update_sampling_rate(u32 new_rate)
{
	int cpu;

	cs_tuners.sampling_rate = new_rate;

	get_online_cpus();
	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy;
		struct cpu_dbs_common_info *cdbs;
		unsigned long next_sampling, appointed_at;

		policy = cpufreq_cpu_get(cpu);
		if (IS_ERR_OR_NULL(policy))
			continue;

		cdbs = get_cpu_cdbs(policy->cpu);
		cpufreq_cpu_put(policy);

		mutex_lock(&cdbs->timer_mutex);
		if (!delayed_work_pending(&cdbs->work)) {
			mutex_unlock(&cdbs->timer_mutex);
			continue;
		}

		next_sampling = jiffies + usecs_to_jiffies(new_rate);
		appointed_at  = cdbs->work.timer.expires;

		if (time_before(next_sampling, appointed_at)) {
			mutex_unlock(&cdbs->timer_mutex);

			cancel_delayed_work_sync(&cdbs->work);

			mutex_lock(&cdbs->timer_mutex);
			queue_delayed_work_on(cdbs->cpu, cs_wq, &cdbs->work,
					usecs_to_jiffies(new_rate));

		}
		mutex_unlock(&cdbs->timer_mutex);
	}
	put_online_cpus();
}

static ssize_t store_sampling_rate(struct kobject *kobj,
				   struct attribute *attr,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val < min_sampling_rate)
		return -EINVAL;

	update_sampling_rate(val);

	return count;
}

show_one_dbs(cs, sampling_rate);
define_one_global_rw(sampling_rate);

static ssize_t store_sampling_down_factor(struct kobject *kobj,
					  struct attribute *attr,
					  const char *buf, size_t count)
{
	struct cs_cpu_dbs_info_s *j_dbs_info;
	unsigned int val;
	int ret, cpu;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val < 1)
		return -EINVAL;

	cs_tuners.sampling_down_factor = val;

	for_each_online_cpu(cpu) {
		j_dbs_info = get_cpu_dbs_info_s(cpu);
		j_dbs_info->rate_mult = 1;
	}

	return count;
}

show_one_dbs(cs, sampling_down_factor);
define_one_global_rw(sampling_down_factor);

define_ratemin_node(cs);
define_one_dbs_node(cs, up_threshold, (cs_tuners.down_threshold + 1), 100);
define_one_dbs_node(cs, up_threshold_burst, 0, 100);
define_one_dbs_node(cs, down_threshold, 0, (cs_tuners.up_threshold - 1));
define_one_dbs_node(cs, freq_up_step, 1, 100);
define_one_dbs_node(cs, freq_down_step, 1, 100);
define_one_dbs_node(cs, io_is_busy, 0, 1);

static struct attribute *cs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&sampling_down_factor.attr,
	&up_threshold.attr,
	&up_threshold_burst.attr,
	&down_threshold.attr,
	&freq_up_step.attr,
	&freq_down_step.attr,
	&io_is_busy.attr,
	NULL,
};

static struct attribute_group cs_attr_group = {
	.name = "conservative",
	.attrs = cs_attributes,
};

static int cs_cpufreq_notifier(struct notifier_block *nb,
			       unsigned long val, void *data)
{
	struct cpufreq_policy *policy;
	struct cpufreq_freqs *freq = data;
	struct cs_cpu_dbs_info_s *dbs_info = get_cpu_dbs_info_s(freq->cpu);

	if (IS_ERR_OR_NULL(dbs_info->cdbs.cur_policy))
		return NOTIFY_OK;

	policy = dbs_info->cdbs.cur_policy;

	/*
	 * We only care if our internally tracked freq moves outside the 'valid'
	 * ranges of frequency available to us.  Otherwise, we do not change it.
	 */
	if (dbs_info->target_freq < policy->min ||
	    dbs_info->target_freq > policy->max)
		dbs_info->target_freq = freq->new;

	return NOTIFY_DONE;
}

static struct notifier_block cs_cpufreq_notifier_block = {
	.notifier_call = cs_cpufreq_notifier,
};

static inline int cs_init(struct cpufreq_policy *policy,
			  struct cpu_dbs_common_info *cdbs)
{
	u32 latency;
	int ret;

	/* Constants below should be initialized only once */
	if (++gov_enable_cnt != 1)
		return 0;

	/* Bring kernel and HW constraints together */
	latency = max_t(u32, policy->cpuinfo.transition_latency / 1000, 1);
	min_sampling_rate = max(min_sampling_rate,
				latency * MIN_LATENCY_MULTIPLIER);
	cs_tuners.sampling_rate = max(min_sampling_rate,
				latency * LATENCY_MULTIPLIER);

	if (!cs_tuners.io_is_busy)
		cs_tuners.io_is_busy = should_io_be_busy();

	ret = cpufreq_register_notifier(&cs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to register atomic notifier\n");
		goto fail_notifier;
	}

	ret = sysfs_create_group(cpufreq_global_kobject, &cs_attr_group);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to create sysfs group\n");
		goto fail_sysfs;
	}

	return 0;

fail_sysfs:
	cpufreq_unregister_notifier(&cs_cpufreq_notifier_block,
				    CPUFREQ_TRANSITION_NOTIFIER);
fail_notifier:
	gov_enable_cnt--;

	return ret;
}

static inline void cs_exit(struct cpu_dbs_common_info *cdbs)
{
	cdbs->cur_policy = NULL;

	/* Constants below should be voided only once */
	if (--gov_enable_cnt != 0)
		return;

	sysfs_remove_group(cpufreq_global_kobject, &cs_attr_group);
	cpufreq_unregister_notifier(&cs_cpufreq_notifier_block,
				    CPUFREQ_TRANSITION_NOTIFIER);
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy, u32 event)
{
	u32 cpu = policy->cpu;
	struct cs_cpu_dbs_info_s *dbs_info = get_cpu_dbs_info_s(cpu);
	struct cpu_dbs_common_info *cdbs = get_cpu_cdbs(cpu), *j_cdbs;
	int ret, j;

	switch (event) {
	case CPUFREQ_GOV_START:
		if (unlikely(!cpu_online(cpu) || !policy->cur))
			return -EINVAL;

		mutex_lock(&cs_mutex);
		for_each_cpu(j, policy->cpus) {
			j_cdbs = get_cpu_cdbs(j);
			j_cdbs->cpu = j;
			j_cdbs->prev_load = 0;
			j_cdbs->cur_policy = policy;
			j_cdbs->prev_cpu_idle = get_cpu_idle_time(j,
				&j_cdbs->prev_cpu_wall, should_io_be_busy());
		}

		dbs_info->rate_mult = 1;
		dbs_info->target_freq = policy->cur;

		ret = cs_init(policy, cdbs);
		if (IS_ERR_VALUE(ret)) {
			mutex_unlock(&cs_mutex);
			return ret;
		}
		mutex_unlock(&cs_mutex);

		cs_timer_init(cdbs);
		break;
	case CPUFREQ_GOV_STOP:
		cs_timer_exit(cdbs);

		mutex_lock(&cs_mutex);
		cs_exit(cdbs);
		mutex_unlock(&cs_mutex);
		break;
	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&cdbs->timer_mutex);
		if (cdbs->cur_policy->cur > policy->max)
			__cpufreq_driver_target(cdbs->cur_policy, policy->max,
						CPUFREQ_RELATION_H);
		else if (cdbs->cur_policy->cur < policy->min)
			__cpufreq_driver_target(cdbs->cur_policy, policy->min,
						CPUFREQ_RELATION_L);
		cs_check_cpu(dbs_info);
		mutex_unlock(&cdbs->timer_mutex);
	}

	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_CONSERVATIVE
static
#endif
struct cpufreq_governor cpufreq_gov_conservative = {
	.name			= "conservative",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	struct cpu_dbs_common_info *cdbs;
	int cpu;

	cs_wq = alloc_workqueue("conservative_cs_wq", WQ_HIGHPRI, 0);
	if (IS_ERR_OR_NULL(cs_wq)) {
		pr_err("Unable to allocate high-priority workqueue\n");
		return -EFAULT;
	}

	for_each_possible_cpu(cpu) {
		cdbs = get_cpu_cdbs(cpu);
		mutex_init(&cdbs->timer_mutex);
	}

	/*
	 * In NOHZ/micro accounting case we set the minimum frequency
	 * not depending on HZ, but fixed (very low).  The deferred
	 * timer might skip some samples if idle/sleeping as needed.
	 */
	if (nohz_idle_used())
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	else
		min_sampling_rate = jiffy_sampling_rate();

	return cpufreq_register_governor(&cpufreq_gov_conservative);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	struct cpu_dbs_common_info *cdbs;
	int cpu;

	cpufreq_unregister_governor(&cpufreq_gov_conservative);

	for_each_possible_cpu(cpu) {
		cdbs = get_cpu_cdbs(cpu);
		mutex_destroy(&cdbs->timer_mutex);
	}

	destroy_workqueue(cs_wq);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_CONSERVATIVE
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);

MODULE_AUTHOR("Alexander Clouter <alex@digriz.org.uk>");
MODULE_DESCRIPTION("'cpufreq_conservative' - A dynamic cpufreq governor for "
		   "Low Latency Frequency Transition capable processors "
		   "optimised for use in a battery environment");
MODULE_LICENSE("GPL v2");
