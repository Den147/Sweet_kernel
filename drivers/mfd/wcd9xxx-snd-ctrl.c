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

/* Read/write flags for __snd_ctrl_data_global() */
#define __WL		(0)
#define __RL		(1)

/* Helpers */
#define line_present(name)	\
	__builtin_expect(!!(ctrl_data->lines.name##_line), 1)
#define read_line(name)		\
	ctrl_data->read(ctrl_data->codec, ctrl_data->lines.name##_line)
#define write_line(name, val)	\
	ctrl_data->write(ctrl_data->codec, ctrl_data->lines.name##_line, (val))

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
 * Local sound control data which will be used as a source of
 * default values as soon as expected codec data is registered.
 */
static struct ctrl_stats *stats;

/* Global sound control data which is used in all sysfs nodes */
static struct snd_ctrl_data *ctrl_data;

/* Kernel object where sysfs groups are handled */
static struct kobject *snd_ctrl_kobj;

/* Mutex that protects control data state switch */
static DEFINE_MUTEX(snd_ctrl_mutex);

/* List of conjuncted codecs */
static LIST_HEAD(codec_list);

/* Just prototypes. See the information below. */
static inline bool snd_ctrl_data_global(struct snd_ctrl_data *snd_data);
static inline bool snd_ctrl_data_global_rw(struct snd_ctrl_data *snd_data);
static inline bool snd_ctrl_data_expected(struct snd_ctrl_data *snd_data);
static inline int snd_ctrl_data_fill_of(struct snd_ctrl_data *snd_data);
static struct snd_ctrl_data *find_ctrl_data(const char *ctrl_name);

/**
 * snd_ctrl_register() - register a new sound control data.
 * @snd_data: pointer to sound control data.
 *
 * Tries to register a passed control data. If the passed control data is
 * incomplete or is already registered, an appropriate negative (-EINVAL)
 * will be returned.  If this is the first control data in a global codec
 * list, it will become a global one.
 *
 * Returns zero on success or -EINVAL on error.
 */
int snd_ctrl_register(struct snd_ctrl_data *snd_data)
{
	int err = -EEXIST;

	/* Incomplete control data can not be registered */
	if (IS_ERR_OR_NULL(snd_data) ||
	    IS_ERR_OR_NULL(snd_data->name) ||
	    IS_ERR_OR_NULL(snd_data->codec))
		return -EINVAL;

	/* Control data must have read function. Otherwise, it is useless */
	if (IS_ERR_OR_NULL(snd_data->read))
		return -EINVAL;

	/*
	 * Mutexes are used here to avoid the race between concurrent calls.
	 * Since it is unwanted to have more than one thread while manipulating
	 * with lists, we should mute the access for every entry. These entries
	 * will not cooperate with each other, so the deadlock is unlikely to
	 * happen.
	 */
	mutex_lock(&snd_ctrl_mutex);
	/* Ensure that control data has not been already registered */
	if (likely(!find_ctrl_data(snd_data->name))) {
		err = 0;

		list_add(&snd_data->member, &codec_list);
		pr_debug("%s ctrl data was registered\n", snd_data->name);

		/* Apply default values if this data is expected by OF */
		if (likely(snd_ctrl_data_expected(snd_data))) {
			err = snd_ctrl_data_fill_of(snd_data);
			if (IS_ERR_VALUE(err))
				pr_err("Unable to fulfill %s control data\n",
					snd_data->name);
		}

		/*
		 * Make the first passed control data global. It will become
		 * the default controlled codec, and its sound gains will be
		 * exported first.  A controlled codec can be changed later
		 * via a corresponding sysfs node.
		 */
		if (list_is_singular(&codec_list)) {
			ctrl_data = snd_data;
			pr_info("New global ctrl data --> %s\n",
				 ctrl_data->name);
		}
	}
	mutex_unlock(&snd_ctrl_mutex);

	return err;
}
EXPORT_SYMBOL_GPL(snd_ctrl_register);

/**
 * snd_ctrl_unregister() - unregister a sound control data.
 * @snd_data: pointer to sound control data.
 *
 * Tries to unregister a sound control data.  If the passed data is incomplete
 * or even has not been registered yet, the function will return early. If the
 * unregistered control data was a global one, the first codec in the list will
 * pretend to be a replacement.  In case there are no available codecs left,
 * global control data will be nulled.
 */
void snd_ctrl_unregister(struct snd_ctrl_data *snd_data)
{
	/* Incomplete control data can not be unregistered */
	if (IS_ERR_OR_NULL(snd_data) ||
	    IS_ERR_OR_NULL(snd_data->name))
		return;

	/*
	 * If the codec's control data somehow was not registered,
	 * there is no point in continuing.
	 */
	mutex_lock(&snd_ctrl_mutex);
	if (unlikely(!find_ctrl_data(snd_data->name))) {
		mutex_unlock(&snd_ctrl_mutex);
		return;
	}

	list_del(&snd_data->member);
	pr_debug("%s ctrl data was unregistered\n", snd_data->name);

	if (!list_empty(&codec_list)) {
		/*
		 * Replace a global control data with the first in the list.
		 * This way we will keep something under control unless all
		 * control datas are unregistered.
		 */
		if (snd_ctrl_data_global(snd_data)) {
			ctrl_data = list_first_entry(&codec_list,
					struct snd_ctrl_data, member);
			pr_info("New global ctrl data --> %s\n",
				 ctrl_data->name);
		}
	} else {
		/*
		 * When there is no any registered codec's control data, the
		 * only option here is to completely remove any controlling
		 * by nulling the global control data.
		 */
		ctrl_data = NULL;
		pr_debug("No available ctrl datas yet\n");
	}
	mutex_unlock(&snd_ctrl_mutex);
}
EXPORT_SYMBOL_GPL(snd_ctrl_unregister);

/**
 * snd_ctrl_data_handled() - check whether a passed control data is handled
 * right now.
 * @snd_data: pointer to sound control data.
 *
 * Checks the presence of controlled lines in a passed control data.  Ensures
 * that a passed control data and a global one are the same and have an ability
 * to write to.
 */
bool snd_ctrl_data_handled(struct snd_ctrl_data *snd_data)
{
	/*
	 * Codec is considered as handled only if there is an ability to
	 * write to it.
	 */
	if (!snd_ctrl_data_global_rw(snd_data))
		return false;

	/* At least one sound line must be fullfilled */
	return (snd_data->lines.mic_line ||
		snd_data->lines.cam_mic_line ||
		snd_data->lines.speaker_l_line ||
		snd_data->lines.speaker_r_line ||
		snd_data->lines.headphone_l_line ||
		snd_data->lines.headphone_r_line);
}
EXPORT_SYMBOL_GPL(snd_ctrl_data_handled);

/**
 * find_ctrl_data() - search for a control data in a global codec list.
 * @ctrl_name: name of a target member.
 *
 * Returns target member structure if found or NULL to the contrary.
 */
static struct snd_ctrl_data *find_ctrl_data(const char *ctrl_name)
{
	struct snd_ctrl_data *entry;

	/* Return early if there is nothing to search */
	if (IS_ERR_OR_NULL(ctrl_name) || list_empty(&codec_list))
		return NULL;

	/*
	 * It is pretended that there will not be a huge list of codecs, so
	 * a linear search here will not be a problem. Conversely, it is an
	 * efficient searching algorithm within a little list of options.
	 * However, there will be a mess if a giant number of codecs is
	 * registered.
	 */
	list_for_each_entry(entry, &codec_list, member)
		if (!strnicmp(ctrl_name, entry->name, CTRL_NAME_LEN))
			return entry;

	return NULL;
}

/**
 * snd_ctrl_data_expected() - check whether the passed control data is set
 * in open firmware.
 * @snd_data: pointer to sound control data.
 *
 * Compares the names of local (expected) control data from open firmware and
 * a passed one. Returns true if they are the same, false otherwise.  In case
 * local data is not initialized, it returns false too.
 */
static inline bool snd_ctrl_data_expected(struct snd_ctrl_data *snd_data)
{
	/* Empty stats mean the absence of DT intercalation */
	if (IS_ERR_OR_NULL(stats))
		return false;

	return !strnicmp(stats->data.name, snd_data->name, CTRL_NAME_LEN);
}

/**
 * snd_ctrl_data_ready() - check whether there is a global control data.
 * @read_only: ignore the absence of write call.
 *
 * Returns false if some of the critical parts of control data are empty.
 * Otherwise, returns true.
 */
static inline bool snd_ctrl_data_ready(int read_only)
{
	return (ctrl_data && ctrl_data->name && ctrl_data->codec &&
	        ctrl_data->read && (ctrl_data->write || read_only));
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
 * snd_ctrl_data_global() - check whether a passed control data is global.
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
 * snd_ctrl_data_global_rw() - check whether a passed control data is global
 * and has an ability to write to. Used exclusively by snd_ctrl_data_handled().
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
#define apply_gain(line)					\
	if (snd_data->lines.line##_line)			\
		snd_data->write(snd_data->codec,		\
				snd_data->lines.line##_line,	\
			       (stats->line##_gain))
	apply_gain(mic);
	apply_gain(cam_mic);
	apply_gain(speaker_l);
	apply_gain(speaker_r);
	apply_gain(headphone_l);
	apply_gain(headphone_r);
}

/**
 * snd_ctrl_data_fill_of() - fill a passed control data with the values
 * from open firmware.
 * @snd_data: pointer to sound control data.
 *
 * Assigns the values from local data which are gained from DT to a passed
 * data. Returns -EINVAL on failure or zero on success.
 */
static inline int snd_ctrl_data_fill_of(struct snd_ctrl_data *snd_data)
{
	if (IS_ERR_OR_NULL(stats))
		return -EINVAL;

	snd_data->lines.mic_line = stats->data.lines.mic_line;
	snd_data->lines.cam_mic_line = stats->data.lines.cam_mic_line;
	snd_data->lines.speaker_l_line = stats->data.lines.speaker_l_line;
	snd_data->lines.speaker_r_line = stats->data.lines.speaker_r_line;
	snd_data->lines.headphone_l_line = stats->data.lines.headphone_l_line;
	snd_data->lines.headphone_r_line = stats->data.lines.headphone_r_line;

	apply_gains(snd_data);

	return 0;
}

/**
 * parse_ctrl_data() - try to switch to another control data.
 * @snd_data: pointer to pointer to a sound control data to be switched.
 * @ctrl_name: name of a desired codec whose control data will be used instead.
 *
 * Returns zero on success or -EINVAL to the contrary.
 */
static int parse_ctrl_data(struct snd_ctrl_data **snd_data,
			   const char *ctrl_name)
{
	struct snd_ctrl_data *tmp = NULL, named = { .name = ctrl_name };

	mutex_lock(&snd_ctrl_mutex);
	/* There is no sense in reassigning the same control data */
	if (snd_ctrl_data_global(&named)) {
		mutex_unlock(&snd_ctrl_mutex);
		return 0;
	}

	/* This is used for debugging only */
	if (unlikely(!strnicmp(ctrl_name, "none", 4)))
		goto empty;

	/*
	 * A temporary structure is used here just to make sure,
	 * that there is a desired codec in the codec list.
	 */
	tmp = find_ctrl_data(ctrl_name);
	if (IS_ERR_OR_NULL(tmp)) {
		mutex_unlock(&snd_ctrl_mutex);
		return -EINVAL;
	}

empty:
	*snd_data = tmp;
	mutex_unlock(&snd_ctrl_mutex);

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
	int ret;							\
	unsigned int val;						\
									\
	if (!snd_ctrl_data_ready(__WL) || !line_present(name))		\
		return -EINVAL;						\
									\
	ret = sscanf(buf, "%u", &val);					\
	if (ret != 1 || (int)val < 0 || val > 256)			\
		return -EINVAL;						\
									\
	ret = write_line(name, val);					\
	if (IS_ERR_VALUE(ret))						\
		return -EINVAL;						\
									\
	return count;							\
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
	int ret;							\
	unsigned int lval, rval;					\
									\
	if (!snd_ctrl_data_ready(__WL) ||				\
	    !line_present(name##_l) || !line_present(name##_r))		\
		return -EINVAL;						\
									\
	ret = sscanf(buf, "%u %u", &lval, &rval);			\
	if (ret != 2 ||							\
	   (int)lval < 0 || lval > 256 ||				\
	   (int)rval < 0 || rval > 256)					\
		return -EINVAL;						\
									\
	ret  = write_line(name##_l, lval);				\
	ret |= write_line(name##_r, rval);				\
	if (IS_ERR_VALUE(ret))						\
		return -EINVAL;						\
									\
	return count;							\
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
	int ret;							\
	unsigned int reg;						\
									\
	if (!snd_ctrl_data_ready(__RL))					\
		return -EINVAL;						\
									\
	ret = sscanf(buf, "%x", &reg);					\
	if (ret != 1 || (int)reg < 0 || reg > 0x3FF)			\
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

	if (list_empty(&codec_list))
		return scnprintf(buf, 8, "<none>\n");

	list_for_each_entry(entry, &codec_list, member)
		len += scnprintf(buf + len, CTRL_NAME_LEN + 4,
				 list_is_last(&entry->member, &codec_list) ?
				(snd_ctrl_data_global(entry) ? "[%s]"  : "%s") :
				(snd_ctrl_data_global(entry) ? "[%s] " : "%s "),
				 entry->name);

	len += scnprintf(buf + len, 2, "\n");

	return len;
}

static ssize_t store_active_codec(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	int ret;
	char name[CTRL_NAME_LEN];

	ret = sscanf(buf, "%31s", name);
	if (ret != 1 || IS_ERR_OR_NULL(name))
		return -EINVAL;

	ret = parse_ctrl_data(&ctrl_data, name);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to set %s\n", name);
		return ret;
	}

	return count;
}

static ssize_t show_ioctl_bypass(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 char *buf)
{
	if (!snd_ctrl_data_ready(__WL))
		return scnprintf(buf, 15, "<unsupported>\n");
	if (!(ctrl_data->flags & SND_CTRL_BYPASS_IOCTL))
		return scnprintf(buf, 13, "Hybrid mode\n");

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

static int __devinit snd_ctrl_parse_dt(struct device_node *node)
{
	u32 default_gain[6] = { 0 };
	int ret, fail = -EINVAL;
	char *key;

	/* Stop parsing if driver is disabled in DT */
	key = "status";
	ret = of_property_match_string(node, key, "disabled");
	if (ret > 0) {
		pr_debug("Sound controller is disabled\n");
		return 0;
	}

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

	/* Fail only if all the keys are unstated */
#define get_line(fail, node, key, line) ({				    \
	int ret = of_property_read_u32(node, key, &stats->data.lines.line); \
	if (IS_ERR_VALUE(ret))						    \
		pr_err("Unable to get %s\n", key);			    \
	fail &= ret; })

	get_line(fail, node, "qcom,mic_line", mic_line);
	get_line(fail, node, "qcom,cam_mic_line", cam_mic_line);
	get_line(fail, node, "qcom,speaker_l_line", speaker_l_line);
	get_line(fail, node, "qcom,speaker_r_line", speaker_r_line);
	get_line(fail, node, "qcom,headphone_l_line", headphone_l_line);
	get_line(fail, node, "qcom,headphone_r_line", headphone_r_line);

	key = "qcom,default_gain";
	ret = of_property_read_u32_array(node, key, default_gain, 6);
	if (IS_ERR_VALUE(ret))
		pr_err("Unable to get default sound gains from DT\n");
	fail &= ret;

	/* Setup default sound gains */
	stats->mic_gain = (!ret ? default_gain[0] : 0);
	stats->cam_mic_gain = (!ret ? default_gain[1] : 0);
	stats->speaker_l_gain = (!ret ? default_gain[2] : 0);
	stats->speaker_r_gain = (!ret ? default_gain[3] : 0);
	stats->headphone_l_gain = (!ret ? default_gain[4] : 0);
	stats->headphone_r_gain = (!ret ? default_gain[5] : 0);

	return fail;
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
	if (pdata->default_gain) {
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
	struct snd_ctrl_pdata *pdata = pdev->dev.platform_data;
	struct device_node *node = pdev->dev.of_node;
	int ret;

	stats = devm_kzalloc(&pdev->dev, sizeof(*stats), GFP_KERNEL);
	if (IS_ERR_OR_NULL(stats)) {
		pr_err("Unable to allocate memory for control stats\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, stats);

	/* Try to get default values either from OF or platform data */
	if (node) {
		ret = snd_ctrl_parse_dt(node);
		if (IS_ERR_VALUE(ret)) {
			pr_err("Unable to parse device tree\n");
			goto fail_parse;
		}
	} else if (pdata) {
		ret = snd_ctrl_parse_pdata(pdata);
		if (IS_ERR_VALUE(ret)) {
			pr_err("Unable to parse platform data\n");
			goto fail_parse;
		}
	} else {
		/* Get rid of stats if they are not going to be used */
		platform_set_drvdata(pdev, NULL);
		kfree(stats);
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
fail_parse:
	platform_set_drvdata(pdev, NULL);
	kfree(stats); /* Nullify statistics to avoid data filling at all */

	return ret;
}

static int __devexit snd_ctrl_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);

	sysfs_remove_group(snd_ctrl_kobj, &snd_ctrl_lines_group);
	sysfs_remove_group(snd_ctrl_kobj, &snd_ctrl_attr_group);
	kobject_del(snd_ctrl_kobj);

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
	return platform_driver_register(&snd_ctrl_driver);
}

static void __exit snd_ctrl_exit(void)
{
	platform_driver_unregister(&snd_ctrl_driver);
}

module_init(snd_ctrl_init);
module_exit(snd_ctrl_exit);
