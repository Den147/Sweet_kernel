/*
 * Copyright (C) 2017-2018, Alex Saiko <solcmdr@gmail.com>
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

#define pr_fmt(fmt)	KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_data/wcd9xxx-snd-ctrl.h>

#define CTRL_NAME_LEN	32

/* Read/write flags for snd_ctrl_data_ready() call */
#define __WL		(0)
#define __RL		(1)

/* Helpers */
#define line_present(name)	\
	__builtin_expect(!!(ctrl_data->lines.name##_line), 1)
#define read_line(name)		\
	ctrl_data->read(ctrl_data->codec, ctrl_data->lines.name##_line)
#define write_line(name, val)	\
	ctrl_data->write(ctrl_data->codec, ctrl_data->lines.name##_line, (val))

/*
 * Local control stats that are likely to be used onto target control data.
 *
 * The idea is to fill one of the expected control data with an appropriate
 * information including target sound lines and default sound gains. This way
 * it is possible to setup the default sound gain of the whole most-used dev
 * map via either Open Firmware or platform data, omitting the setting by
 * initramfs or other user-space service.
 */
struct ctrl_stats {
	struct snd_ctrl_pdata data;

	/* Default sound gains */
	u32 mic_gain;
	u32 cam_mic_gain;
	u32 speaker_l_gain;
	u32 speaker_r_gain;
	u32 headphone_l_gain;
	u32 headphone_r_gain;
};

/*
 * Local sound control data which will be used as a source of default
 * values as soon as expected sound control data will be registered.
 */
static struct ctrl_stats *stats;

/* Global sound control data which is used in all sysfs nodes */
static struct snd_ctrl_data *ctrl_data;

/* Kernel object where sysfs groups are proceed */
static struct kobject *snd_ctrl_kobj;

/* Mutex that protects access to linked list below */
static DEFINE_MUTEX(list_mutex);

/* List of conjuncted control data */
static LIST_HEAD(ctrl_list);

/* Just prototypes. See the information below. */
static inline bool snd_ctrl_data_global(struct snd_ctrl_data *snd_data);
static inline bool snd_ctrl_data_global_rw(struct snd_ctrl_data *snd_data);
static inline bool snd_ctrl_data_expected(struct snd_ctrl_data *snd_data);
static inline int snd_ctrl_data_fill(struct snd_ctrl_data *snd_data);
static struct snd_ctrl_data *find_ctrl_data(const char *ctrl_name);

/**
 * snd_ctrl_register() - register new sound control data.
 * @snd_data: pointer to sound control data.
 *
 * Tries to register passed control data. If one is incomplete or is already
 * registered, an appropriate negative will be returned. If this is the first
 * control data in a global ctrl list, it will become a global one.
 *
 * In case Open Firmware or platform data is used, hence one of the ctrl data
 * is expected, this function will fill the target one with the values from
 * OF/pdata source and immediately make it global, bypassing the queue.
 *
 * Returns zero on success or -EINVAL[-EEXIST] on error.
 */
int snd_ctrl_register(struct snd_ctrl_data *snd_data)
{
	/* Incomplete control data cannot be registered */
	if (IS_ERR_OR_NULL(snd_data))
		return -EINVAL;
	if (IS_ERR_OR_NULL(snd_data->name))
		return -EINVAL;
	if (IS_ERR_OR_NULL(snd_data->codec))
		return -EINVAL;

	/* Control data MUST have read function. Otherwise it is useless. */
	if (IS_ERR_OR_NULL(snd_data->read))
		return -EINVAL;

	/*
	 * Mutexes are used here to avoid the race between concurrent calls.
	 * Since it is unwanted to have more than one thread while manipulating
	 * with lists, we should mute the access for every entry. These entries
	 * will not cooperate with each other, so the deadlock is unlikely to
	 * happen.
	 */
	mutex_lock(&list_mutex);
	/* Ensure that control data has not been already registered */
	if (unlikely(find_ctrl_data(snd_data->name))) {
		mutex_unlock(&list_mutex);
		return -EEXIST;
	}

	/* Add new control data to a global ctrl data list */
	list_add(&snd_data->member, &ctrl_list);
	pr_debug("%s ctrl data is registered\n", snd_data->name);

	/* Apply default values if this data is expected by OF */
	if (snd_ctrl_data_expected(snd_data) && snd_ctrl_data_fill(snd_data))
		pr_err("Unable to fill %s ctrl data\n", snd_data->name);

	/*
	 * Make the first passed control data global.  It will become the
	 * default controlled codec, and its sound gains will be exported
	 * first. A controlled codec can be changed later via a corresponding
	 * sysfs node. Also take expected codec into account.
	 */
	if (list_is_singular(&ctrl_list) || snd_ctrl_data_expected(snd_data)) {
		ctrl_data = snd_data;
		pr_info("New global ctrl data: %s\n", ctrl_data->name);
	}
	mutex_unlock(&list_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_ctrl_register);

/**
 * snd_ctrl_unregister() - unregister sound control data.
 * @snd_data: pointer to sound control data.
 *
 * Tries to unregister passed control data. If one is incomplete or even has
 * not been registered yet, this function will return early. If that ctrl data
 * is a global one right now and is going to be unregistered, the first codec
 * in the list will pretend to be a replacement. In case there are no available
 * ctrl data left, global control data will be nulled, hence controlling will
 * be disabled.
 */
void snd_ctrl_unregister(struct snd_ctrl_data *snd_data)
{
	/* Incomplete control data cannot be unregistered */
	if (IS_ERR_OR_NULL(snd_data) ||
	    IS_ERR_OR_NULL(snd_data->name))
		return;

	/*
	 * If the codec's control data somehow was not registered,
	 * there is no point in continuing.
	 */
	mutex_lock(&list_mutex);
	if (unlikely(!find_ctrl_data(snd_data->name))) {
		mutex_unlock(&list_mutex);
		return;
	}

	/*
	 * Remove control data from a global ctrl data list.
	 * Also reinitialize the list to take advantage of list_empty() call.
	 */
	list_del_init(&snd_data->member);
	pr_debug("%s ctrl data is unregistered\n", snd_data->name);

	if (!list_empty(&ctrl_list) && snd_ctrl_data_global(snd_data)) {
		/*
		 * Replace a global control data with the first in the list.
		 * This way we will keep something under control unless all
		 * control data are unregistered.
		 */
		ctrl_data = list_first_entry(&ctrl_list,
				struct snd_ctrl_data, member);
		pr_info("New global ctrl data: %s\n", ctrl_data->name);
	} else if (list_empty(&ctrl_list)) {
		/*
		 * When there are no any registered control data, the only
		 * option is to completely remove any controlling by nulling
		 * the global control data.
		 */
		ctrl_data = NULL;
		pr_debug("No available ctrl data yet\n");
	}
	mutex_unlock(&list_mutex);
}
EXPORT_SYMBOL_GPL(snd_ctrl_unregister);

/**
 * snd_ctrl_data_handled() - check whether passed control data is handled now.
 * @snd_data: pointer to sound control data.
 *
 * Checks the presence of controlled lines in passed control data. Ensures that
 * passed control data and a global one are the same and have an ability to
 * write to.
 */
bool snd_ctrl_data_handled(struct snd_ctrl_data *snd_data)
{
	u32 ret = 0;

	/*
	 * Codec is considered as handled only if there is an ability to
	 * write to it.
	 */
	if (!snd_ctrl_data_global_rw(snd_data))
		return false;

	/* At least one sound line must be filled */
	ret |= snd_data->lines.mic_line;
	ret |= snd_data->lines.cam_mic_line;
	ret |= snd_data->lines.speaker_l_line;
	ret |= snd_data->lines.speaker_r_line;
	ret |= snd_data->lines.headphone_l_line;
	ret |= snd_data->lines.headphone_r_line;

	return ret;
}
EXPORT_SYMBOL_GPL(snd_ctrl_data_handled);

/**
 * find_ctrl_data() - search for control data in a global ctrl data list.
 * @ctrl_name: name of a target member.
 *
 * Returns target member structure if found or NULL to the contrary.
 * ! This function MUST be called with list_mutex taken.
 */
static struct snd_ctrl_data *find_ctrl_data(const char *ctrl_name)
{
	struct snd_ctrl_data *entry;

	/* Return early if there is nothing to search */
	if (list_empty(&ctrl_list) || IS_ERR_OR_NULL(ctrl_name))
		return NULL;

	/*
	 * It is pretended that there will not be a huge list of codecs, so
	 * a linear search here will not be a problem. Conversely, it is an
	 * efficient searching algorithm within a little list of options.
	 * However, there will be a mess if a giant number of codecs is
	 * registered.
	 */
	list_for_each_entry(entry, &ctrl_list, member)
		if (!strnicmp(ctrl_name, entry->name, CTRL_NAME_LEN))
			return entry;

	return NULL;
}

/**
 * snd_ctrl_data_expected() - check if passed control data is set in OF/pdata.
 * @snd_data: pointer to sound control data.
 *
 * Compares the names of local (expected) control data from OF/pdata and a
 * passed one. Returns true if they are the same, false otherwise. In case
 * local data is not initialized, it returns false too.
 */
static inline bool snd_ctrl_data_expected(struct snd_ctrl_data *snd_data)
{
	/* Empty stats mean the absence of OF/pdata intercalation */
	if (IS_ERR_OR_NULL(stats))
		return false;

	return !strnicmp(stats->data.name, snd_data->name, CTRL_NAME_LEN);
}

/**
 * snd_ctrl_data_ready() - check whether there is global control data.
 * @read_only: ignore the absence of write call.
 *
 * Returns false if some of the critical parts of the global sound control
 * data are NULL. Otherwise, returns true.
 */
static inline bool snd_ctrl_data_ready(int read_only)
{
	/* Implement it this way to make it more or less readable */
	if (IS_ERR_OR_NULL(ctrl_data))
		return false;
	if (IS_ERR_OR_NULL(ctrl_data->name))
		return false;
	if (IS_ERR_OR_NULL(ctrl_data->codec))
		return false;
	if (IS_ERR_OR_NULL(ctrl_data->read))
		return false;
	if (IS_ERR_OR_NULL(ctrl_data->write) && !read_only)
		return false;

	return true;
}

/**
 * For internal usage only. See the information below.
 */
static inline bool __snd_ctrl_data_global(struct snd_ctrl_data *snd_data,
					  int read_only)
{
	if (!snd_ctrl_data_ready(read_only))
		return false;

	return !strnicmp(ctrl_data->name, snd_data->name, CTRL_NAME_LEN);
}

/**
 * snd_ctrl_data_global() - check whether passed control data is global.
 * @snd_data: pointer to sound control data.
 *
 * Compares a passed control data's name and a global's one.
 * Returns true if they are the same, false otherwise.
 */
static inline bool snd_ctrl_data_global(struct snd_ctrl_data *snd_data)
{
	return __snd_ctrl_data_global(snd_data, __RL);
}

/**
 * snd_ctrl_data_global_rw() - check whether passed control data is global and
 * has an ability to write to.
 * @snd_data: pointer to sound control data.
 *
 * Compares a passed control data's name and a global's one and checks the
 * presence of write call in it. Returns true if both statements are true,
 * false otherwise.
 */
static inline bool snd_ctrl_data_global_rw(struct snd_ctrl_data *snd_data)
{
	return __snd_ctrl_data_global(snd_data, __WL);
}

/**
 * apply_gains() - write default gains from local data to a passed ctrl data.
 * @snd_data: pointer to sound control data.
 *
 * Uses internal calls to setup a passed control data.
 */
static inline void apply_gains(struct snd_ctrl_data *snd_data)
{
#define apply_gain(name)					\
	if (likely(snd_data->lines.name##_line))		\
		snd_data->write(snd_data->codec,		\
				snd_data->lines.name##_line,	\
			       (stats->name##_gain))
	apply_gain(mic);
	apply_gain(cam_mic);
	apply_gain(speaker_l);
	apply_gain(speaker_r);
	apply_gain(headphone_l);
	apply_gain(headphone_r);
}

/**
 * snd_ctrl_data_fill() - fill passed control data with values from OF/pdata.
 * @snd_data: pointer to sound control data.
 *
 * Assigns the values from local data which are gained from OF/pdata to passed
 * data. Returns -EINVAL on failure or zero on success.
 */
static inline int snd_ctrl_data_fill(struct snd_ctrl_data *snd_data)
{
	struct snd_ctrl_pdata *pdata;

	if (IS_ERR_OR_NULL(stats))
		return -EINVAL;

	pdata = &(stats->data);

	/* Setup default sound lines */
	snd_data->lines.mic_line = pdata->lines.mic_line;
	snd_data->lines.cam_mic_line = pdata->lines.cam_mic_line;
	snd_data->lines.speaker_l_line = pdata->lines.speaker_l_line;
	snd_data->lines.speaker_r_line = pdata->lines.speaker_r_line;
	snd_data->lines.headphone_l_line = pdata->lines.headphone_l_line;
	snd_data->lines.headphone_r_line = pdata->lines.headphone_r_line;

	/* Apply default values for each acquired sound line */
	apply_gains(snd_data);

	return 0;
}

/**
 * parse_ctrl_data() - try to switch to another control data.
 * @snd_data: pointer to pointer to sound control data to be switched.
 * @ctrl_name: name of a desired codec whose control data will be used instead.
 *
 * Returns zero on success or -EINVAL to the contrary.
 */
static inline int
parse_ctrl_data(struct snd_ctrl_data **snd_data, const char *ctrl_name)
{
	struct snd_ctrl_data *tmp = NULL, named = { .name = ctrl_name };

	mutex_lock(&list_mutex);
	/* There is no sense in reassigning the same control data */
	if (snd_ctrl_data_global(&named)) {
		mutex_unlock(&list_mutex);
		return 0;
	}

	/* This is used for debugging only */
	if (unlikely(!strnicmp(ctrl_name, "none", 4)))
		goto empty;

	/*
	 * A temporary structure is used here just to make sure,
	 * that there is a desired codec in a codec list.
	 */
	tmp = find_ctrl_data(ctrl_name);
	if (IS_ERR_OR_NULL(tmp)) {
		mutex_unlock(&list_mutex);
		return -EINVAL;
	}
empty:
	*snd_data = tmp;
	mutex_unlock(&list_mutex);

	return 0;
}

#define create_rw_kobj_attr(name)					\
static struct kobj_attribute name =					\
	__ATTR(gpl_##name, 0644, show_##name, store_##name);

#define create_one_single(name)						\
static ssize_t show_##name##_gain(struct kobject *kobj,			\
				  struct kobj_attribute *attr,		\
				  char *buf)				\
{									\
	if (!snd_ctrl_data_ready(__RL) || !line_present(name))		\
		return scnprintf(buf, 15, "<unsupported>\n");		\
									\
	return scnprintf(buf, 5, "%u\n", read_line(name));		\
}									\
									\
static ssize_t store_##name##_gain(struct kobject *kobj,		\
				   struct kobj_attribute *attr,		\
				   const char *buf, size_t count)	\
{									\
	int ret, val;							\
									\
	if (!snd_ctrl_data_ready(__WL) || !line_present(name))		\
		return -EINVAL;						\
									\
	ret = sscanf(buf, "%d", &val);					\
	if (ret != 1 || val < 0 || val > 256)				\
		return -EINVAL;						\
									\
	ret = write_line(name, val);					\
									\
	return IS_ERR_VALUE(ret) ? ret : count;				\
}									\
									\
create_rw_kobj_attr(name##_gain);

#define create_one_double(name)						\
static ssize_t show_##name##_gain(struct kobject *kobj,			\
				  struct kobj_attribute *attr,		\
				  char *buf)				\
{									\
	if (!snd_ctrl_data_ready(__RL) ||				\
	    !line_present(name##_l) || !line_present(name##_r))		\
		return scnprintf(buf, 15, "<unsupported>\n");		\
									\
	return scnprintf(buf, 9, "%u %u\n",				\
		read_line(name##_l), read_line(name##_r));		\
}									\
									\
static ssize_t store_##name##_gain(struct kobject *kobj,		\
				   struct kobj_attribute *attr,		\
				   const char *buf, size_t count)	\
{									\
	int ret, lval, rval;						\
									\
	if (!snd_ctrl_data_ready(__WL) ||				\
	    !line_present(name##_l) || !line_present(name##_r))		\
		return -EINVAL;						\
									\
	ret = sscanf(buf, "%d %d", &lval, &rval);			\
	if (ret != 2 ||							\
	    lval < 0 || lval > 256 ||					\
	    rval < 0 || rval > 256)					\
		return -EINVAL;						\
									\
	ret  = write_line(name##_l, lval);				\
	ret |= write_line(name##_r, rval);				\
									\
	return IS_ERR_VALUE(ret) ? ret : count;				\
}									\
									\
create_rw_kobj_attr(name##_gain);

#define create_line_control(name)					\
static ssize_t show_##name##_line(struct kobject *kobj,			\
				  struct kobj_attribute *attr,		\
				  char *buf)				\
{									\
	if (!snd_ctrl_data_ready(__RL) || !line_present(name))		\
		return scnprintf(buf, 8, "<none>\n");			\
									\
	return scnprintf(buf, 7, "0x%x\n",				\
			 ctrl_data->lines.name##_line);			\
}									\
									\
static ssize_t store_##name##_line(struct kobject *kobj,		\
				   struct kobj_attribute *attr,		\
				   const char *buf, size_t count)	\
{									\
	int ret, reg;							\
									\
	if (!snd_ctrl_data_ready(__RL))					\
		return -EINVAL;						\
									\
	ret = sscanf(buf, "%x", &reg);					\
	if (ret != 1 || reg < 0 || reg > 0x3FF)				\
		return -EINVAL;						\
									\
	ctrl_data->lines.name##_line = reg;				\
									\
	return count;							\
}									\
									\
create_rw_kobj_attr(name##_line);

static ssize_t show_active_codec(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	struct snd_ctrl_data *entry;
	ssize_t len = 0;

	/* Continuing is suboptimal in case of an emptiness */
	if (list_empty(&ctrl_list))
		return scnprintf(buf, 8, "<none>\n");

	/*
	 * Iterate over the codec list and print all registered ctrl data.
	 * The data that is in use right now is put into square brackets.
	 */
	list_for_each_entry(entry, &ctrl_list, member)
		len += scnprintf(buf + len, CTRL_NAME_LEN + 4,
				 snd_ctrl_data_global(entry) ?
				 "[%s] " : "%s ", entry->name);

	/* Remove whitespace and put a new line in a right position */
	scnprintf(buf + len - 1, 2, "\n");

	return len;
}

static ssize_t store_active_codec(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	char name[CTRL_NAME_LEN];
	int ret;

	/* Codec name should fit into CTRL_NAME_LEN (32 characters) */
	ret = sscanf(buf, "%32s", name);
	if (ret != 1 || IS_ERR_OR_NULL(name))
		return -EINVAL;

	/* Try to parse ctrl data and set it if it was found */
	ret = parse_ctrl_data(&ctrl_data, name);

	return IS_ERR_VALUE(ret) ? ret : count;
}

static ssize_t show_ioctl_bypass(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	if (!snd_ctrl_data_ready(__WL))
		return scnprintf(buf, 15, "<unsupported>\n");
	if (!(ctrl_data->flags & SND_CTRL_BYPASS_IOCTL))
		return scnprintf(buf, 13, "Hybrid mode\n");

	/* SND_CTRL_BYPASS_IOCTL indicates that IOCTL ->write() is bypassed */
	return scnprintf(buf, 17, "Restricted mode\n");
}

static ssize_t store_ioctl_bypass(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	int ret;
	u8 val;

	if (!snd_ctrl_data_ready(__WL))
		return -EINVAL;

	ret = kstrtou8(buf, 2, &val);
	if (IS_ERR_VALUE(ret))
		return -EINVAL;

	if (val)
		ctrl_data->flags |= SND_CTRL_BYPASS_IOCTL;
	else
		ctrl_data->flags &= ~(SND_CTRL_BYPASS_IOCTL);

	return count;
}

create_one_single(mic);
create_one_single(cam_mic);
create_one_double(speaker);
create_one_double(headphone);

create_line_control(mic);
create_line_control(cam_mic);
create_line_control(speaker_l);
create_line_control(speaker_r);
create_line_control(headphone_l);
create_line_control(headphone_r);

create_rw_kobj_attr(active_codec);
create_rw_kobj_attr(ioctl_bypass);

static struct attribute *snd_ctrl_attrs[] = {
	&mic_gain.attr,
	&cam_mic_gain.attr,
	&speaker_gain.attr,
	&headphone_gain.attr,
	&active_codec.attr,
	&ioctl_bypass.attr,
	NULL,
};

static struct attribute *snd_ctrl_lines[] = {
	&mic_line.attr,
	&cam_mic_line.attr,
	&speaker_l_line.attr,
	&speaker_r_line.attr,
	&headphone_l_line.attr,
	&headphone_r_line.attr,
	NULL,
};

static struct attribute_group snd_ctrl_attr_group = {
	.attrs = snd_ctrl_attrs,
};

static struct attribute_group snd_ctrl_lines_group = {
	.name = "snd_lines",
	.attrs = snd_ctrl_lines,
};

static inline bool __devinit is_enabled(struct device_node *node)
{
	if (IS_ERR_OR_NULL(node))
		return false;

	return of_property_match_string(node, "status", "disabled") < 0;
}

static int __devinit snd_ctrl_parse_dt(struct device_node *node)
{
	u32 data[6] = { 0 };
	int ret, fail;
	char *key;

	/*
	 * Codec name is compulsory. There is no sense in continuing in case of
	 * an absence as it is required for snd_ctrl_data_expected() function.
	 */
	key = "qcom,codec_name";
	ret = of_property_read_string(node, key, &stats->data.name);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to get codec name from device tree\n");
		return -EINVAL;
	}

	/**
	 * get_line() - try to get a register value from Device Tree.
	 * @fail: indicator of a failure. An integer that is defined above.
	 * @node: Device Tree node which will be used as a source of values.
	 * @key: name of a Device Tree property.
	 * @name: name of a sound line to store the value from property into.
	 */
#define get_line(fail, node, key, name) ({				   \
	int rc = of_property_read_u32(node, key, &stats->data.lines.name); \
	if (IS_ERR_VALUE(rc))						   \
		pr_err("Unable to get %s from device tree\n", key);	   \
	fail &= rc; })

	/* Fail only if all the keys are unstated */
	fail = -EINVAL | -ENODATA | -EOVERFLOW;
	get_line(fail, node, "qcom,mic_line", mic_line);
	get_line(fail, node, "qcom,cam_mic_line", cam_mic_line);
	get_line(fail, node, "qcom,speaker_l_line", speaker_l_line);
	get_line(fail, node, "qcom,speaker_r_line", speaker_r_line);
	get_line(fail, node, "qcom,headphone_l_line", headphone_l_line);
	get_line(fail, node, "qcom,headphone_r_line", headphone_r_line);
	if (IS_ERR_VALUE(fail))
		return -EINVAL;

	/* Default gains are recommended but not required */
	key = "qcom,default_gain";
	ret = of_property_read_u32_array(node, key, data, ARRAY_SIZE(data));
	if (IS_ERR_VALUE(ret))
		pr_err("Unable to get default sound gains from device tree\n");

	/* Setup default sound gains */
	stats->mic_gain = (!ret ? data[0] : 0);
	stats->cam_mic_gain = (!ret ? data[1] : 0);
	stats->speaker_l_gain = (!ret ? data[2] : 0);
	stats->speaker_r_gain = (!ret ? data[3] : 0);
	stats->headphone_l_gain = (!ret ? data[4] : 0);
	stats->headphone_r_gain = (!ret ? data[5] : 0);

	return 0;
}

static int __devinit snd_ctrl_parse_pdata(struct snd_ctrl_pdata *pdata)
{
	/*
	 * Codec name is compulsory. There is no sense in continuing in case of
	 * an absence as it is required for snd_ctrl_data_expected() function.
	 */
	stats->data.name = pdata->name;
	if (IS_ERR_OR_NULL(stats->data.name)) {
		pr_err("Unable to get codec name from platform data\n");
		return -EINVAL;
	}

	/* We cannot fail in this context */
	stats->data.lines.mic_line = pdata->lines.mic_line;
	stats->data.lines.cam_mic_line = pdata->lines.cam_mic_line;
	stats->data.lines.speaker_l_line = pdata->lines.speaker_l_line;
	stats->data.lines.speaker_r_line = pdata->lines.speaker_r_line;
	stats->data.lines.headphone_l_line = pdata->lines.headphone_l_line;
	stats->data.lines.headphone_r_line = pdata->lines.headphone_r_line;

	/* Setup default sound gains */
	if (likely(pdata->default_gain)) {
		stats->mic_gain = pdata->default_gain[0];
		stats->cam_mic_gain = pdata->default_gain[1];
		stats->speaker_l_gain = pdata->default_gain[2];
		stats->speaker_r_gain = pdata->default_gain[3];
		stats->headphone_l_gain = pdata->default_gain[4];
		stats->headphone_r_gain = pdata->default_gain[5];
	}

	return 0;
}

static int __devinit snd_ctrl_probe(struct platform_device *pdev)
{
	int ret = 0;

	stats = devm_kzalloc(&pdev->dev, sizeof(*stats), GFP_KERNEL);
	if (IS_ERR_OR_NULL(stats)) {
		pr_err("Unable to allocate memory for control stats\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, stats);

	/* Try to get default values either from OF or platform data */
	if (is_enabled(pdev->dev.of_node)) {
		ret = snd_ctrl_parse_dt(pdev->dev.of_node);
		if (IS_ERR_VALUE(ret)) {
			pr_err("Unable to parse device tree\n");
			goto fail_parse;
		}
	} else if (pdev->dev.platform_data) {
		ret = snd_ctrl_parse_pdata(pdev->dev.platform_data);
		if (IS_ERR_VALUE(ret)) {
			pr_err("Unable to parse platform data\n");
			goto fail_parse;
		}
	} else {
		/* Get rid of stats if they are not going to be used */
		goto fail_parse;
	}

	return ret;

fail_parse:
	platform_set_drvdata(pdev, NULL);
	/* Nullify statistics to avoid data filling at all */
	devm_kfree(&pdev->dev, stats);

	return ret;
}

static int __devexit snd_ctrl_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id snd_ctrl_match_table[] = {
	{ .compatible = "qcom,wcd9xxx-snd-ctrl" },
	{ },
};

static struct platform_driver snd_ctrl_driver = {
	.probe = snd_ctrl_probe,
	.remove = __devexit_p(snd_ctrl_remove),
	.driver = {
		.name = "wcd9xxx-snd-ctrl",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(snd_ctrl_match_table),
	},
};

static int __init snd_ctrl_init(void)
{
	int ret;

	ret = platform_driver_register(&snd_ctrl_driver);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to register platform driver\n");
		goto fail_pdrv;
	}

	snd_ctrl_kobj = kobject_create_and_add("sound_control_3", kernel_kobj);
	if (IS_ERR_OR_NULL(snd_ctrl_kobj)) {
		pr_err("Unable to create sysfs kernel object\n");
		ret = -ENOMEM;
		goto fail_kobj;
	}

	ret = sysfs_create_group(snd_ctrl_kobj, &snd_ctrl_attr_group);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to create sound attributes group\n");
		goto fail_attrs;
	}

	ret = sysfs_create_group(snd_ctrl_kobj, &snd_ctrl_lines_group);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to create sound lines group\n");
		goto fail_lines;
	}

	return 0;

fail_lines:
	sysfs_remove_group(snd_ctrl_kobj, &snd_ctrl_attr_group);
fail_attrs:
	kobject_del(snd_ctrl_kobj);
fail_kobj:
	platform_driver_unregister(&snd_ctrl_driver);
fail_pdrv:
	return ret;
}

static void __exit snd_ctrl_exit(void)
{
	sysfs_remove_group(snd_ctrl_kobj, &snd_ctrl_lines_group);
	sysfs_remove_group(snd_ctrl_kobj, &snd_ctrl_attr_group);
	kobject_del(snd_ctrl_kobj);

	platform_driver_unregister(&snd_ctrl_driver);
}

module_init(snd_ctrl_init);
module_exit(snd_ctrl_exit);

MODULE_AUTHOR("Alex Saiko <solcmdr@gmail.com>");
MODULE_DESCRIPTION("WCD9xxx Sound Register SysFS Controller");
MODULE_LICENSE("GPL v2");
