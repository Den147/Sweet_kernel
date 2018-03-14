/*
 * Copyright (C) 2017, Alex Saiko <solcmdr@gmail.com>
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

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/mfd/wcd9xxx/wcd9xxx-snd-ctrl.h>

#define CTRL_NAME_LEN	32
#define __WL		(0)
#define __RL		(1)

/* Global sound control data which is used in all sysfs nodes */
static struct snd_ctrl_data *ctrl_data;

/* Kernel object where sysfs groups are handled */
static struct kobject *snd_ctrl_kobj;

/* Mutex that protects control data switch */
static DEFINE_MUTEX(snd_ctrl_mutex);

/* List of conjuncted codecs */
static LIST_HEAD(codec_list);

/* Just prototypes. See the information below. */
static inline bool snd_ctrl_data_global(struct snd_ctrl_data *snd_data);
static inline bool snd_ctrl_data_global_rw(struct snd_ctrl_data *snd_data);
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
	int err = -EINVAL;

	/* Incomplete control data can not be registered */
	if (IS_ERR_OR_NULL(snd_data) || IS_ERR_OR_NULL(snd_data->name))
		return err;

	/* Control data must have a read function. Otherwise, it is useless. */
	if (IS_ERR_OR_NULL(snd_data->codec) ||
	    IS_ERR_OR_NULL(snd_data->codec_read))
		return err;

	/*
	 * Mutexes are used here to avoid the race between concurrent calls.
	 * Since the first transmitted codec will likely become a global one,
	 * there is a chance of a race condition between different candidates.
	 * However, this is unlikely to happen in the kernel space, since all
	 * codecs are expected to be initialized one by one.
	 */
	mutex_lock(&snd_ctrl_mutex);
	/* Ensure that control data has not been already registered */
	if (likely(!find_ctrl_data(snd_data->name))) {
		err = 0;

		list_add(&snd_data->member, &codec_list);
		pr_debug("%s was registered\n", snd_data->name);

		/*
		 * Make the first passed control data global. It will become
		 * the default controlled codec, and its sound gains will be
		 * exported first.  A controlled codec can be changed later
		 * via a corresponding sysfs node.
		 */
		if (list_is_singular(&codec_list)) {
			ctrl_data = snd_data;
			pr_debug("%s became global\n", ctrl_data->name);
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
 * Tries to unregister a sound control data. If the passed data is incomplete
 * or even has not been registered yet, the function will return early. If the
 * unregistered control data was a global one, the first codec in the list will
 * pretend to be a replacement.  In case there are no available codecs left,
 * global control data will be nulled.
 */
void snd_ctrl_unregister(struct snd_ctrl_data *snd_data)
{
	/* Incomplete control data can not be unregistered */
	if (IS_ERR_OR_NULL(snd_data) || IS_ERR_OR_NULL(snd_data->name))
		return;

	/*
	 * If the codec's control data somehow was not registered,
	 * there is no point in continuing.
	 */
	if (unlikely(!find_ctrl_data(snd_data->name)))
		return;

	list_del(&snd_data->member);
	pr_debug("%s was unregistered\n", snd_data->name);

	mutex_lock(&snd_ctrl_mutex);
	if (!list_empty(&codec_list)) {
		/*
		 * Replace a global control data with the first in the list.
		 * This way we will keep something under control unless all
		 * control datas are unregistered.
		 */
		if (snd_ctrl_data_global(snd_data)) {
			ctrl_data = list_first_entry(&codec_list,
					struct snd_ctrl_data, member);
			pr_debug("Switching to %s\n", ctrl_data->name);
		}
	} else {
		/*
		 * When there is no any registered codec's control data, the
		 * only option is to completely remove any controlling by
		 * nulling the global control data.
		 */
		ctrl_data = NULL;
		pr_debug("No available codecs yet\n");
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
	if (!snd_ctrl_data_global_rw(snd_data))
		return false;

	if (!snd_data->mic_line && !snd_data->cam_mic_line &&
	    !snd_data->headphone_l_line && !snd_data->headphone_r_line &&
	    !snd_data->speaker_l_line && !snd_data->speaker_r_line)
		return false;

	return true;
}
EXPORT_SYMBOL_GPL(snd_ctrl_data_handled);

/**
 * snd_ctrl_data_ready() - check whether there is a global control data.
 * @read_only: ignore the absence of codec_write call.
 *
 * Returns false if some of the critical parts of control data are empty.
 * Otherwise, returns true.
 */
static inline bool snd_ctrl_data_ready(int read_only)
{
	if (IS_ERR_OR_NULL(ctrl_data))
		return false;
	if (IS_ERR_OR_NULL(ctrl_data->name))
		return false;
	if (IS_ERR_OR_NULL(ctrl_data->codec))
		return false;
	if (IS_ERR_OR_NULL(ctrl_data->codec_read))
		return false;
	if (IS_ERR_OR_NULL(ctrl_data->codec_write) && !read_only)
		return false;

	return true;
}

/**
 * Internal usage only. See the information below.
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
 * snd_ctrl_data_global_rw() - check whether a passed control data is global and
 * has an ability to write to. Used exclusively by snd_ctrl_data_handled().
 * @snd_data: pointer to sound control data.
 *
 * Compares a passed control data's name and a global's one and checks the
 * presence of codec_write call in it. Returns true if both statements are
 * true, false otherwise.
 */
static inline bool snd_ctrl_data_global_rw(struct snd_ctrl_data *snd_data)
{
	return __snd_ctrl_data_global(snd_data, __WL);
}

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

#define create_rw_kobj_attr(_name)					\
static struct kobj_attribute _name =					\
	__ATTR(gpl_##_name, 0644, show_##_name, store_##_name);

#define create_one_single(name)						\
static ssize_t show_##name##_gain(struct kobject *kobj,			\
				  struct kobj_attribute *attr,		\
				  char *buf)				\
{									\
	if (!snd_ctrl_data_ready(__RL) || !ctrl_data->name##_line)	\
		return scnprintf(buf, 15, "<unsupported>\n");		\
									\
	return scnprintf(buf, 5, "%u\n",				\
		ctrl_data->codec_read(ctrl_data->codec,			\
				      ctrl_data->name##_line));		\
}									\
									\
static ssize_t store_##name##_gain(struct kobject *kobj,		\
				   struct kobj_attribute *attr,		\
				   const char *buf, size_t count)	\
{									\
	int ret;							\
	unsigned int val;						\
									\
	if (!snd_ctrl_data_ready(__WL) || !ctrl_data->name##_line)	\
		return -EINVAL;						\
									\
	ret = sscanf(buf, "%u", &val);					\
	if (ret != 1 || (int)val < 0 || val > 256)			\
		return -EINVAL;						\
									\
	ctrl_data->codec_write(ctrl_data->codec,			\
			       ctrl_data->name##_line, val);		\
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
	if (!snd_ctrl_data_ready(__RL))					\
		return scnprintf(buf, 15, "<unsupported>\n");		\
	if (!ctrl_data->name##_l_line || !ctrl_data->name##_r_line)	\
		return scnprintf(buf, 15, "<unsupported>\n");		\
									\
	return scnprintf(buf, 9, "%u %u\n",				\
		ctrl_data->codec_read(ctrl_data->codec,			\
				      ctrl_data->name##_l_line),	\
		ctrl_data->codec_read(ctrl_data->codec,			\
				      ctrl_data->name##_r_line));	\
}									\
									\
static ssize_t store_##name##_gain(struct kobject *kobj,		\
				   struct kobj_attribute *attr,		\
				   const char *buf, size_t count)	\
{									\
	int ret;							\
	unsigned int lval, rval;					\
									\
	if (!snd_ctrl_data_ready(__WL))					\
		return -EINVAL;						\
	if (!ctrl_data->name##_l_line || !ctrl_data->name##_r_line)	\
		return -EINVAL;						\
									\
	ret = sscanf(buf, "%u %u", &lval, &rval);			\
	if (ret != 2 ||							\
	   (int)lval < 0 || lval > 256 ||				\
	   (int)rval < 0 || rval > 256)					\
		return -EINVAL;						\
									\
	ctrl_data->codec_write(ctrl_data->codec,			\
			       ctrl_data->name##_l_line, lval);		\
	ctrl_data->codec_write(ctrl_data->codec,			\
			       ctrl_data->name##_r_line, rval);		\
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
	if (!snd_ctrl_data_ready(__RL) || !ctrl_data->name##_line)	\
		return scnprintf(buf, 8, "<none>\n");			\
									\
	return scnprintf(buf, 7, "0x%x\n", ctrl_data->name##_line);	\
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
	ctrl_data->name##_line = reg;					\
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
		return -EINVAL;
	}

	return count;
}

create_one_single(mic);
create_one_single(cam_mic);
create_one_double(headphone);
create_one_double(speaker);

create_line_control(mic);
create_line_control(cam_mic);
create_line_control(headphone_l);
create_line_control(headphone_r);
create_line_control(speaker_l);
create_line_control(speaker_r);

create_rw_kobj_attr(active_codec);

static struct attribute *snd_ctrl_attrs[] = {
	&mic_gain.attr,
	&cam_mic_gain.attr,
	&headphone_gain.attr,
	&speaker_gain.attr,
	&active_codec.attr,
	NULL,
};

static struct attribute_group snd_ctrl_attr_group = {
	.attrs = snd_ctrl_attrs,
};

static struct attribute *snd_ctrl_lines[] = {
	&mic_line.attr,
	&cam_mic_line.attr,
	&headphone_l_line.attr,
	&headphone_r_line.attr,
	&speaker_l_line.attr,
	&speaker_r_line.attr,
	NULL,
};

static struct attribute_group snd_ctrl_lines_group = {
	.name = "snd_lines",
	.attrs = snd_ctrl_lines,
};

static int __init snd_ctrl_init(void)
{
	int ret;

	snd_ctrl_kobj = kobject_create_and_add("sound_control_3", kernel_kobj);
	if (IS_ERR_OR_NULL(snd_ctrl_kobj)) {
		pr_err("Unable to create sound_control kernel object\n");
		return -ENOMEM;
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

	return ret;
}

static void __exit snd_ctrl_exit(void)
{
	sysfs_remove_group(snd_ctrl_kobj, &snd_ctrl_lines_group);
	sysfs_remove_group(snd_ctrl_kobj, &snd_ctrl_attr_group);
	kobject_del(snd_ctrl_kobj);
}

module_init(snd_ctrl_init);
module_exit(snd_ctrl_exit);
