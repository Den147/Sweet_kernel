/*
 * Copyright (C) 2002-2003, Dominik Brodowski <linux@brodo.de>
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

#include <linux/module.h>
#include <linux/cpufreq.h>

static int cpufreq_governor_limits(struct cpufreq_policy *policy, u32 event)
{
	switch (event) {
	case CPUFREQ_GOV_START:
	case CPUFREQ_GOV_LIMITS:
		pr_debug("%s: Setting to %ukHz because of event %u\n",
			 __func__, policy->min, event);
		__cpufreq_driver_target(policy, policy->min,
					CPUFREQ_RELATION_L);
	}

	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_POWERSAVE
static
#endif
struct cpufreq_governor cpufreq_gov_powersave = {
	.name		= "powersave",
	.governor	= cpufreq_governor_limits,
	.owner		= THIS_MODULE,
};

static int __init cpufreq_gov_powersave_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_powersave);
}

static void __exit cpufreq_gov_powersave_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_powersave);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_POWERSAVE
fs_initcall(cpufreq_gov_powersave_init);
#else
module_init(cpufreq_gov_powersave_init);
#endif
module_exit(cpufreq_gov_powersave_exit);

MODULE_AUTHOR("Dominik Brodowski <linux@brodo.de>");
MODULE_DESCRIPTION("CPUfreq policy governor 'powersave'");
MODULE_LICENSE("GPL v2");
