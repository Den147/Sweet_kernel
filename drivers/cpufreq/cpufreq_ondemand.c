/*
 * Copyright (C) 2001, Russell King.
 * Copyright (C) 2003, Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>
 * Copyright (C) 2003, Jun Nakajima <jun.nakajima@intel.com>
 * Copyright (C) 2013, The Linux Foundation. All rights reserved.
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
#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(10)
#define DEF_FREQUENCY_SAMPLING_DOWN_FACTOR	(1)
#define DEF_FREQUENCY_SYNCHRONIZATION		(1)
#define DEF_FREQUENCY_LOAD_DEPENDENT_SCALING	(0)

#define MICRO_FREQUENCY_UP_THRESHOLD		(95)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(3)

struct dbs_work_struct {
	struct work_struct work;
	u32 cpu;
};

static DEFINE_PER_CPU(struct dbs_work_struct, dbs_refresh_work);
static DEFINE_PER_CPU(struct od_cpu_dbs_info_s, od_cpu_dbs_info);
define_get_cpu_dbs_routines(od_cpu_dbs_info);

static struct od_dbs_tuners od_tuners = {
	.sampling_down_factor = DEF_FREQUENCY_SAMPLING_DOWN_FACTOR,
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.up_threshold_multi_core = DEF_FREQUENCY_UP_THRESHOLD,
	.up_threshold_any_cpu_load = DEF_FREQUENCY_UP_THRESHOLD,
	.down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
	.down_differential_multi_core = MICRO_FREQUENCY_DOWN_DIFFERENTIAL,
	.sync_on_migration = DEF_FREQUENCY_SYNCHRONIZATION,
	.load_scaling = DEF_FREQUENCY_LOAD_DEPENDENT_SCALING,
};

/* Minimal sampling rate supported by hardware and aligned with software */
static unsigned int __read_mostly min_sampling_rate;

/* Number of cpus that currently use this governor */
static unsigned int gov_enable_cnt;

/* Workqueue used to run this governor on */
static struct workqueue_struct *od_wq;

/* Mutex that protect governor start/stop routines */
static DEFINE_MUTEX(od_mutex);

static inline u32 get_policy_max_load_freq(struct cpufreq_policy *policy,
					   u32 *max_load_ptr)
{
	struct cpu_dbs_common_info *j_cdbs;
	struct od_cpu_dbs_info_s *j_dbs_info;
	u32 max_load_freq = 0, max_load = 0, load_freq, load, load_at_max_freq;
	int cpu, freq_avg;

	for_each_cpu(cpu, policy->cpus) {
		u64 cur_idle_time, cur_wall_time;
		u32 idle_time, wall_time;

		/* Some targets want iowait time to be subtracted from idle */
		cur_idle_time = get_cpu_idle_time(cpu, &cur_wall_time,
						  od_tuners.io_is_busy);

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

		if (unlikely(wall_time > od_tuners.sampling_rate * 2 &&
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

		/*
		 * Update dbs load as well.  These will be used in
		 * frequency synchronization on task migration event.
		 */
		j_dbs_info = get_cpu_dbs_info_s(cpu);
		j_dbs_info->max_load  = max(load, j_dbs_info->prev_load);
		j_dbs_info->prev_load = load;

		/*
		 * Try to get average CPU frequency. Fallback to current
		 * frequency upon failure.
		 */
		freq_avg = __cpufreq_driver_getavg(policy, cpu);
		if (freq_avg <= 0)
			freq_avg = policy->cur;

		load_freq = load * freq_avg;
		max_load_freq = max(max_load_freq, load_freq);
	}

	/* Report the policy utilization to userspace */
	load_at_max_freq = max_load_freq / policy->max;
	cpufreq_notify_utilization(policy, load_at_max_freq);

	if (max_load_ptr)
		*max_load_ptr = max_load;

	return max_load_freq;
}

static inline u32 get_policy_max_load_other_cpu(struct cpufreq_policy *policy)
{
	struct cpu_dbs_common_info *j_cdbs;
	struct od_cpu_dbs_info_s *j_dbs_info;
	u32 max_load_other_cpu = 0;
	int cpu;

	for_each_online_cpu(cpu) {
		if (cpu == policy->cpu)
			continue;

		j_dbs_info = get_cpu_dbs_info_s(cpu);
		max_load_other_cpu = max(max_load_other_cpu,
					 j_dbs_info->max_load);

		/*
		 * The other CPU could be running at higher frequency, but may
		 * not have completed its sampling_down_factor.  For that case
		 * consider other CPU is loaded so that frequency imbalance
		 * does not occur.
		 */
		j_cdbs = get_cpu_cdbs(cpu);
		if (j_cdbs->cur_policy &&
		    j_cdbs->cur_policy->cur == j_cdbs->cur_policy->max &&
		    policy->cur >= od_tuners.optimal_freq)
			max_load_other_cpu =
				od_tuners.up_threshold_any_cpu_load;
	}

	return max_load_other_cpu;
}

static inline void align_freq_next(struct cpufreq_policy *policy,
				   u32 *freq_next, u32 max_load_other_cpu,
				   u32 max_load_freq)
{
	u32 aligned_freq;

	if (num_online_cpus() < 2)
		return;

	aligned_freq = *freq_next;
	if (max_load_other_cpu >
	    od_tuners.up_threshold_multi_core -
	    od_tuners.down_differential &&
	    aligned_freq < od_tuners.sync_freq)
		aligned_freq = od_tuners.sync_freq;

	if (max_load_freq >
	   ((od_tuners.up_threshold_multi_core -
	    od_tuners.down_differential_multi_core) *
	    policy->cur) && aligned_freq < od_tuners.optimal_freq)
		aligned_freq = od_tuners.optimal_freq;

	*freq_next = aligned_freq;
}

static void od_check_cpu(struct od_cpu_dbs_info_s *dbs_info)
{
	struct cpufreq_policy *policy = dbs_info->cdbs.cur_policy;
	u32 max_load = 0, max_load_freq, max_load_other_cpu, freq_next;
	u32 min_f = policy->cpuinfo.min_freq, max_f = policy->cpuinfo.max_freq;

	max_load_freq = get_policy_max_load_freq(policy, &max_load);
	max_load_other_cpu = get_policy_max_load_other_cpu(policy);

	/* Switch to load dependent algorithm early if specified */
	if (od_tuners.load_scaling)
		goto load_dependent_algorithm;

	if (max_load_freq > od_tuners.up_threshold * policy->cur) {
		if (policy->cur < policy->max)
			dbs_info->rate_mult = od_tuners.sampling_down_factor;

		switch_freq(policy, policy->max);
		return;
	}

	if (num_online_cpus() > 1) {
		if (max_load_other_cpu > od_tuners.up_threshold_any_cpu_load) {
			if (policy->cur < od_tuners.sync_freq)
				switch_freq(policy, od_tuners.sync_freq);
			return;
		}

		if (max_load_freq >
		    od_tuners.up_threshold_multi_core * policy->cur) {
			if (policy->cur < od_tuners.optimal_freq)
				switch_freq(policy, od_tuners.optimal_freq);
			return;
		}
	}

	if (policy->cur == policy->min)
		return;

	if (max_load_freq <
	   (od_tuners.up_threshold - od_tuners.down_differential) *
	    policy->cur) {
		dbs_info->rate_mult = 1;

		freq_next = max((max_load_freq / (od_tuners.up_threshold -
				 od_tuners.down_differential)), policy->min);
		align_freq_next(policy, &freq_next,
				max_load_other_cpu, max_load_freq);

		__cpufreq_driver_target(policy, freq_next, CPUFREQ_RELATION_C);
	}

	return;

load_dependent_algorithm:
	if (max_load >= od_tuners.up_threshold) {
		if (policy->cur < policy->max)
			dbs_info->rate_mult = od_tuners.sampling_down_factor;

		switch_freq(policy, policy->max);
	} else {
		dbs_info->rate_mult = 1;

		freq_next = min_f + max_load * (max_f - min_f) / 100;
		align_freq_next(policy, &freq_next,
				max_load_other_cpu, max_load_freq);

		__cpufreq_driver_target(policy, freq_next, CPUFREQ_RELATION_C);
	}
}

static void od_dbs_timer(struct work_struct *work)
{
	struct od_cpu_dbs_info_s *dbs_info =
		container_of(work, struct od_cpu_dbs_info_s, cdbs.work.work);
	int delay, cpu = dbs_info->cdbs.cpu;

	mutex_lock(&dbs_info->cdbs.timer_mutex);
	od_check_cpu(dbs_info);

	delay = align_delay(od_tuners.sampling_rate, dbs_info->rate_mult);
	queue_delayed_work_on(cpu, od_wq, &dbs_info->cdbs.work, delay);
	mutex_unlock(&dbs_info->cdbs.timer_mutex);
}

static inline void od_timer_init(struct cpu_dbs_common_info *cdbs)
{
	int delay = align_delay(od_tuners.sampling_rate, 1);

	INIT_DEFERRABLE_WORK(&cdbs->work, od_dbs_timer);
	queue_delayed_work_on(cdbs->cpu, od_wq, &cdbs->work, delay);
}

static inline void od_timer_exit(struct cpu_dbs_common_info *cdbs)
{
	cancel_delayed_work_sync(&cdbs->work);
}

static int od_migration_notify(struct notifier_block *nb,
			       unsigned long target_cpu, void *arg)
{
	struct od_cpu_dbs_info_s *dbs_info = get_cpu_dbs_info_s(target_cpu);

	if (!od_tuners.sync_on_migration)
		return NOTIFY_OK;

	atomic_set(&dbs_info->src_sync_cpu, (int)arg);
	wake_up(&dbs_info->sync_wq);

	return NOTIFY_DONE;
}

static struct notifier_block od_migration_nb = {
	.notifier_call = od_migration_notify,
};

static inline int sync_pending(struct od_cpu_dbs_info_s *dbs_info)
{
	return atomic_read(&dbs_info->src_sync_cpu) >= 0;
}

static int od_sync_thread(void *data)
{
	int src_cpu, cpu = (int)data, delay;
	struct od_cpu_dbs_info_s *dest_dbs_info = get_cpu_dbs_info_s(cpu);
	struct od_cpu_dbs_info_s *src_dbs_info;
	struct cpufreq_policy *policy;
	u32 src_freq, src_max_load;

	while (1) {
		wait_event(dest_dbs_info->sync_wq,
			   sync_pending(dest_dbs_info) ||
			   kthread_should_stop());

		if (kthread_should_stop())
			break;

		get_online_cpus();
		if (!atomic_read(&dest_dbs_info->sync_enabled)) {
			atomic_set(&dest_dbs_info->src_sync_cpu, -1);
			put_online_cpus();
			continue;
		}

		src_cpu = atomic_read(&dest_dbs_info->src_sync_cpu);
		src_dbs_info = get_cpu_dbs_info_s(src_cpu);

		if (src_dbs_info && src_dbs_info->cdbs.cur_policy) {
			src_freq = src_dbs_info->cdbs.cur_policy->cur;
			src_max_load = src_dbs_info->max_load;
		} else {
			src_freq = od_tuners.sync_freq;
			src_max_load = 0;
		}

		if (IS_ERR_VALUE(lock_policy_rwsem_write(cpu)))
			goto bail_acq_sema_failed;

		policy = dest_dbs_info->cdbs.cur_policy;
		if (IS_ERR_OR_NULL(policy))
			goto bail_incorrect_governor;

		delay = usecs_to_jiffies(od_tuners.sampling_rate);

		if (policy->cur < src_freq) {
			cancel_delayed_work_sync(&dest_dbs_info->cdbs.work);

			/*
			 * Arch specific cpufreq driver may fail.
			 * Don't update governor frequency upon failure.
			 */
			if (__cpufreq_driver_target(policy, src_freq,
			    CPUFREQ_RELATION_L) >= 0) {
				policy->cur = src_freq;

				if (src_max_load > dest_dbs_info->max_load) {
					dest_dbs_info->max_load = src_max_load;
					dest_dbs_info->prev_load = src_max_load;
				}
			}

			mutex_lock(&dest_dbs_info->cdbs.timer_mutex);
			schedule_delayed_work_on(cpu,
				&dest_dbs_info->cdbs.work, delay);
			mutex_unlock(&dest_dbs_info->cdbs.timer_mutex);
		}

bail_incorrect_governor:
		unlock_policy_rwsem_write(cpu);
bail_acq_sema_failed:
		put_online_cpus();
		atomic_set(&dest_dbs_info->src_sync_cpu, -1);
	}

	return 0;
}

static void od_input_boost(struct work_struct *work)
{
	struct dbs_work_struct *dbs_work =
		container_of(work, struct dbs_work_struct, work);
	struct cpu_dbs_common_info *cdbs;
	struct cpufreq_policy *policy;
	u32 cpu = dbs_work->cpu, target_freq;

	get_online_cpus();
	if (IS_ERR_VALUE(lock_policy_rwsem_write(cpu)))
		goto bail_acq_sema_failed;

	cdbs = get_cpu_cdbs(cpu);

	policy = cdbs->cur_policy;
	if (IS_ERR_OR_NULL(policy))
		goto bail_incorrect_governor;

	target_freq = min(od_tuners.input_boost_freq, policy->max);
	if (policy->cur < target_freq) {
		/*
		 * Arch specific cpufreq driver may fail.
		 * Don't update governor frequency upon failure.
		 */
		if (__cpufreq_driver_target(policy, target_freq,
		    CPUFREQ_RELATION_L) >= 0)
			policy->cur = target_freq;

		cdbs->prev_cpu_idle = get_cpu_idle_time(cpu,
			&cdbs->prev_cpu_wall, od_tuners.io_is_busy);
	}

bail_incorrect_governor:
	unlock_policy_rwsem_write(cpu);
bail_acq_sema_failed:
	put_online_cpus();

	return;
}

static void od_input_event(struct input_handle *handle,
			   unsigned int type, unsigned int code,
			   int value)
{
	int i;

	for_each_online_cpu(i)
		queue_work_on(i, od_wq, &per_cpu(dbs_refresh_work, i).work);
}

static int od_input_connect(struct input_handler *handler,
			    struct input_dev *dev,
			    const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (IS_ERR_OR_NULL(handle))
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (IS_ERR_VALUE(error))
		goto err2;

	error = input_open_device(handle);
	if (IS_ERR_VALUE(error))
		goto err1;

	return 0;

err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);

	return error;
}

static void od_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id od_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler od_input_handler = {
	.event		= od_input_event,
	.connect	= od_input_connect,
	.disconnect	= od_input_disconnect,
	.name		= "cpufreq_ond",
	.id_table	= od_ids,
};

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

	od_tuners.sampling_rate = new_rate;

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
			queue_delayed_work_on(cdbs->cpu, od_wq, &cdbs->work,
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

show_one_dbs(od, sampling_rate);
define_one_global_rw(sampling_rate);

static ssize_t store_sampling_down_factor(struct kobject *kobj,
					  struct attribute *attr,
					  const char *buf, size_t count)
{
	struct od_cpu_dbs_info_s *j_dbs_info;
	unsigned int val;
	int ret, cpu;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val < 1)
		return -EINVAL;

	od_tuners.sampling_down_factor = val;

	for_each_online_cpu(cpu) {
		j_dbs_info = get_cpu_dbs_info_s(cpu);
		j_dbs_info->rate_mult = 1;
	}

	return count;
}

show_one_dbs(od, sampling_down_factor);
define_one_global_rw(sampling_down_factor);

define_ratemin_node(od);
define_one_dbs_node(od, up_threshold, (od_tuners.down_differential + 1), 100);
define_one_dbs_node(od, up_threshold_multi_core,
		   (od_tuners.down_differential_multi_core + 1), 100);
define_one_dbs_node(od, up_threshold_any_cpu_load, 1, 100);
define_one_dbs_node(od, down_differential, 0, (od_tuners.up_threshold - 1));
define_one_dbs_node(od, down_differential_multi_core, 0,
		   (od_tuners.up_threshold_multi_core - 1));
define_one_dbs_node(od, input_boost_freq, 0, UINT_MAX);
define_one_dbs_node(od, optimal_freq, 0, UINT_MAX);
define_one_dbs_node(od, sync_freq, 0, UINT_MAX);
define_one_dbs_node(od, sync_on_migration, 0, 1);
define_one_dbs_node(od, load_scaling, 0, 1);
define_one_dbs_node(od, io_is_busy, 0, 1);

static struct attribute *od_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&sampling_down_factor.attr,
	&up_threshold.attr,
	&up_threshold_multi_core.attr,
	&up_threshold_any_cpu_load.attr,
	&down_differential.attr,
	&down_differential_multi_core.attr,
	&input_boost_freq.attr,
	&optimal_freq.attr,
	&sync_freq.attr,
	&sync_on_migration.attr,
	&load_scaling.attr,
	&io_is_busy.attr,
	NULL,
};

static struct attribute_group od_attr_group = {
	.name = "ondemand",
	.attrs = od_attributes,
};

static inline int od_init(struct cpufreq_policy *policy,
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
	od_tuners.sampling_rate = max(min_sampling_rate,
				latency * LATENCY_MULTIPLIER);

	if (!od_tuners.io_is_busy)
		od_tuners.io_is_busy = should_io_be_busy();
	if (!od_tuners.input_boost_freq)
		od_tuners.input_boost_freq = policy->max;

	od_tuners.optimal_freq =
		clamp(od_tuners.optimal_freq, policy->min, policy->max);
	od_tuners.sync_freq =
		clamp(od_tuners.sync_freq, policy->min, policy->max);

	ret = atomic_notifier_chain_register(&migration_notifier_head,
					     &od_migration_nb);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to register atomic notifier\n");
		goto fail_notifier;
	}

	ret = input_register_handler(&od_input_handler);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to register input handler\n");
		goto fail_input;
	}

	ret = sysfs_create_group(cpufreq_global_kobject, &od_attr_group);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to create sysfs group\n");
		goto fail_sysfs;
	}

	return 0;

fail_sysfs:
	input_unregister_handler(&od_input_handler);
fail_input:
	atomic_notifier_chain_unregister(&migration_notifier_head,
					 &od_migration_nb);
fail_notifier:
	gov_enable_cnt--;

	return ret;
}

static inline void od_exit(struct cpufreq_policy *policy,
			   struct cpu_dbs_common_info *cdbs)
{
	struct od_cpu_dbs_info_s *j_dbs_info;
	int cpu;

	for_each_cpu(cpu, policy->cpus) {
		j_dbs_info = get_cpu_dbs_info_s(cpu);
		atomic_set(&j_dbs_info->sync_enabled, 0);
	}

	cdbs->cur_policy = NULL;

	/* Constants below should be voided only once */
	if (--gov_enable_cnt != 0)
		return;

	sysfs_remove_group(cpufreq_global_kobject, &od_attr_group);
	input_unregister_handler(&od_input_handler);
	atomic_notifier_chain_unregister(&migration_notifier_head,
					 &od_migration_nb);
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy, u32 event)
{
	u32 cpu = policy->cpu;
	struct od_cpu_dbs_info_s *j_dbs_info;
	struct od_cpu_dbs_info_s *dbs_info = get_cpu_dbs_info_s(cpu);
	struct cpu_dbs_common_info *cdbs = get_cpu_cdbs(cpu), *j_cdbs;
	int ret, j;

	switch (event) {
	case CPUFREQ_GOV_START:
		if (unlikely(!cpu_online(cpu) || !policy->cur))
			return -EINVAL;

		mutex_lock(&od_mutex);
		for_each_cpu(j, policy->cpus) {
			j_cdbs = get_cpu_cdbs(j);
			j_cdbs->cpu = j;
			j_cdbs->prev_load = 0;
			j_cdbs->cur_policy = policy;
			j_cdbs->prev_cpu_idle = get_cpu_idle_time(j,
				&j_cdbs->prev_cpu_wall, should_io_be_busy());

			j_dbs_info = get_cpu_dbs_info_s(j);
			set_cpus_allowed(j_dbs_info->sync_thread,
					 *cpumask_of(j));
			atomic_set(&j_dbs_info->sync_enabled, 1);
		}

		dbs_info->rate_mult = 1;

		ret = od_init(policy, cdbs);
		if (IS_ERR_VALUE(ret)) {
			mutex_unlock(&od_mutex);
			return ret;
		}
		mutex_unlock(&od_mutex);

		od_timer_init(cdbs);
		break;
	case CPUFREQ_GOV_STOP:
		od_timer_exit(cdbs);

		mutex_lock(&od_mutex);
		od_exit(policy, cdbs);
		mutex_unlock(&od_mutex);
		break;
	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&cdbs->timer_mutex);
		if (cdbs->cur_policy->cur > policy->max)
			__cpufreq_driver_target(cdbs->cur_policy, policy->max,
						CPUFREQ_RELATION_H);
		else if (cdbs->cur_policy->cur < policy->min)
			__cpufreq_driver_target(cdbs->cur_policy, policy->min,
						CPUFREQ_RELATION_L);
		od_check_cpu(dbs_info);
		mutex_unlock(&cdbs->timer_mutex);
	}

	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
static
#endif
struct cpufreq_governor cpufreq_gov_ondemand = {
	.name			= "ondemand",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	struct od_cpu_dbs_info_s *dbs_info;
	struct dbs_work_struct *dbs_work;
	int cpu;

	od_wq = alloc_workqueue("ondemand_dbs_wq", WQ_HIGHPRI, 0);
	if (IS_ERR_OR_NULL(od_wq)) {
		pr_err("Unable to allocate high-priority workqueue\n");
		return -EFAULT;
	}

	for_each_possible_cpu(cpu) {
		dbs_work = &per_cpu(dbs_refresh_work, cpu);
		dbs_work->cpu = cpu;

		INIT_WORK(&dbs_work->work, od_input_boost);

		dbs_info = get_cpu_dbs_info_s(cpu);
		mutex_init(&dbs_info->cdbs.timer_mutex);

		atomic_set(&dbs_info->src_sync_cpu, -1);
		init_waitqueue_head(&dbs_info->sync_wq);

		dbs_info->sync_thread = kthread_run(od_sync_thread,
					(void *)cpu, "dbs_sync/%d", cpu);
	}

	/*
	 * In NOHZ/micro accounting case we set the minimum frequency
	 * not depending on HZ, but fixed (very low).  The deferred
	 * timer might skip some samples if idle/sleeping as needed.
	 */
	if (nohz_idle_used()) {
		od_tuners.up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
		od_tuners.down_differential = MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
		od_tuners.down_differential_multi_core =
					      MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		min_sampling_rate = jiffy_sampling_rate();
	}

	return cpufreq_register_governor(&cpufreq_gov_ondemand);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	struct od_cpu_dbs_info_s *dbs_info;
	int cpu;

	cpufreq_unregister_governor(&cpufreq_gov_ondemand);

	for_each_possible_cpu(cpu) {
		dbs_info = get_cpu_dbs_info_s(cpu);
		mutex_destroy(&dbs_info->cdbs.timer_mutex);
		kthread_stop(dbs_info->sync_thread);
	}

	destroy_workqueue(od_wq);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);

MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_DESCRIPTION("'cpufreq_ondemand' - A dynamic cpufreq governor for "
		   "Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL v2");