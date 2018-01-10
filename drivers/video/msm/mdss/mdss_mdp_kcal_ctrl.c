/*
 * Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 * Copyright (c) 2013, LGE Inc. All rights reserved.
 * Copyright (c) 2014, Savoca <adeddo27@gmail.com>
 * Copyright (c) 2017, Alex Saiko <solcmdr@gmail.com>
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

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "mdss_fb.h"
#include "mdss_mdp.h"

#define PCC_ADJ		(128)
#define DEF_ENABLE	(1)
#define DEF_INVERT	(0)
#define DEF_MIN		(35)
#define DEF_PCC		(256)
#define DEF_HUE		(0)
#define DEF_PA		(255)

struct kcal_lut_data {
	uint32_t enable:1;
	uint32_t invert:1;
	uint32_t min;
	uint32_t red;
	uint32_t green;
	uint32_t blue;
	uint32_t hue;
	uint32_t sat;
	uint32_t val;
	uint32_t cont;
};

static inline void mdss_mdp_kcal_display_setup(void)
{
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	struct mdss_mdp_ctl *ctl;
	int ret, i;

	for (i = 0; i < mdata->nctl; i++) {
		ctl = mdata->ctl_off + i;
		if (ctl->power_on && ctl->mfd && ctl->mfd->index == 0) {
			ret = mdss_mdp_pp_setup(ctl);
			if (IS_ERR_VALUE(ret))
				pr_err("Unable to setup panel\n");
			break;
		}
	}
}

static inline void mdss_mdp_kcal_read_pcc(struct kcal_lut_data *lut_data)
{
	u32 copyback = 0;
	struct mdp_pcc_cfg_data pcc_config = {
		.block = MDP_LOGICAL_BLOCK_DISP_0,
		.ops = MDP_PP_OPS_READ,
	};

	mdss_mdp_pcc_config(&pcc_config, &copyback);
	if (!copyback || !pcc_config.r.r || !pcc_config.g.g || !pcc_config.b.b)
		return;

	lut_data->red = (pcc_config.r.r & 0xFFFF) / PCC_ADJ;
	lut_data->green = (pcc_config.g.g & 0xFFFF) / PCC_ADJ;
	lut_data->blue = (pcc_config.b.b & 0xFFFF) / PCC_ADJ;
}

static inline void mdss_mdp_kcal_update_pcc(struct kcal_lut_data *lut_data)
{
	struct mdp_pcc_cfg_data pcc_config = {
		.block = MDP_LOGICAL_BLOCK_DISP_0,
		.ops = MDP_PP_OPS_WRITE,
	};

	lut_data->red = max(lut_data->red, lut_data->min);
	lut_data->green = max(lut_data->green, lut_data->min);
	lut_data->blue = max(lut_data->blue, lut_data->min);

	pcc_config.r.r = lut_data->red * PCC_ADJ;
	pcc_config.g.g = lut_data->green * PCC_ADJ;
	pcc_config.b.b = lut_data->blue * PCC_ADJ;

	if (unlikely(lut_data->invert)) {
		pcc_config.r.c  = pcc_config.g.c = pcc_config.b.c = 0x7FF8;
		pcc_config.r.r |= (0xFFFF << 16);
		pcc_config.g.g |= (0xFFFF << 16);
		pcc_config.b.b |= (0xFFFF << 16);
	}

	pcc_config.ops |= lut_data->enable ?
		MDP_PP_OPS_ENABLE : MDP_PP_OPS_DISABLE;

	mdss_mdp_pcc_config(&pcc_config, NULL);
	mdss_mdp_kcal_display_setup();
}

static inline void mdss_mdp_kcal_update_pa(struct kcal_lut_data *lut_data)
{
	struct mdp_pa_cfg_data pa_config = {
		.block = MDP_LOGICAL_BLOCK_DISP_0,
		.pa_data = {
			.flags    = MDP_PP_OPS_WRITE,
			.hue_adj  = lut_data->hue,
			.sat_adj  = lut_data->sat,
			.val_adj  = lut_data->val,
			.cont_adj = lut_data->cont,
		},
	};

	pa_config.pa_data.flags |= lut_data->enable ?
		MDP_PP_OPS_ENABLE : MDP_PP_OPS_DISABLE;

	mdss_mdp_pa_config(&pa_config, NULL);
	mdss_mdp_kcal_display_setup();
}

#define create_one_rw_node(_name)					\
static DEVICE_ATTR(_name, 0644, show_##_name, store_##_name);

#define define_one_kcal_node(_name, object, min, max, update_pa)	\
static ssize_t show_##_name(struct device *dev,				\
			    struct device_attribute *attr,		\
			    char *buf)					\
{									\
	struct kcal_lut_data *lut_data = dev_get_drvdata(dev);		\
									\
	return scnprintf(buf, 6, "%u\n", lut_data->object);		\
}									\
									\
static ssize_t store_##_name(struct device *dev,			\
			     struct device_attribute *attr,		\
			     const char *buf, size_t count)		\
{									\
	struct kcal_lut_data *lut_data = dev_get_drvdata(dev);		\
	uint32_t val;							\
	int ret;							\
									\
	ret = kstrtouint(buf, 10, &val);				\
	if (ret || val < min || val > max || lut_data->object == val)	\
		return -EINVAL;						\
									\
	lut_data->object = val;						\
									\
	mdss_mdp_kcal_update_pcc(lut_data);				\
	if (update_pa)							\
		mdss_mdp_kcal_update_pa(lut_data);			\
									\
	return count;							\
}									\
									\
create_one_rw_node(_name);

static ssize_t show_kcal(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct kcal_lut_data *lut_data = dev_get_drvdata(dev);

	mdss_mdp_kcal_read_pcc(lut_data);

	return scnprintf(buf, 13, "%u %u %u\n",
			 lut_data->red, lut_data->green, lut_data->blue);
}

static ssize_t store_kcal(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct kcal_lut_data *lut_data = dev_get_drvdata(dev);
	uint32_t kcal_r, kcal_g, kcal_b;
	int ret;

	ret = sscanf(buf, "%u %u %u", &kcal_r, &kcal_g, &kcal_b);
	if (ret != 3 ||
	   (int32_t)kcal_r < 1 || kcal_r > 256 ||
	   (int32_t)kcal_g < 1 || kcal_g > 256 ||
	   (int32_t)kcal_b < 1 || kcal_b > 256)
		return -EINVAL;

	lut_data->red = kcal_r;
	lut_data->green = kcal_g;
	lut_data->blue = kcal_b;

	mdss_mdp_kcal_update_pcc(lut_data);

	return count;
}

create_one_rw_node(kcal);
define_one_kcal_node(kcal_enable, enable, 0, 1, true);
define_one_kcal_node(kcal_invert, invert, 0, 1, false);
define_one_kcal_node(kcal_min, min, 1, 256, false);
define_one_kcal_node(kcal_hue, hue, 0, 1536, true);
define_one_kcal_node(kcal_sat, sat, 224, 383, true);
define_one_kcal_node(kcal_val, val, 128, 383, true);
define_one_kcal_node(kcal_cont, cont, 128, 383, true);

static int __devinit kcal_ctrl_probe(struct platform_device *pdev)
{
	struct kcal_lut_data *lut_data;
	int ret;

	lut_data = devm_kzalloc(&pdev->dev, sizeof(*lut_data), GFP_KERNEL);
	if (IS_ERR_OR_NULL(lut_data)) {
		pr_err("Unable to allocate memory for LUT data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, lut_data);

	lut_data->enable = DEF_ENABLE;
	lut_data->invert = DEF_INVERT;
	lut_data->min = DEF_MIN;
	lut_data->red = DEF_PCC;
	lut_data->green = DEF_PCC;
	lut_data->blue = DEF_PCC;
	lut_data->hue = DEF_HUE;
	lut_data->sat = DEF_PA;
	lut_data->val = DEF_PA;
	lut_data->cont = DEF_PA;

	mdss_mdp_kcal_update_pcc(lut_data);
	mdss_mdp_kcal_update_pa(lut_data);

	ret  = device_create_file(&pdev->dev, &dev_attr_kcal);
	ret |= device_create_file(&pdev->dev, &dev_attr_kcal_enable);
	ret |= device_create_file(&pdev->dev, &dev_attr_kcal_invert);
	ret |= device_create_file(&pdev->dev, &dev_attr_kcal_min);
	ret |= device_create_file(&pdev->dev, &dev_attr_kcal_hue);
	ret |= device_create_file(&pdev->dev, &dev_attr_kcal_sat);
	ret |= device_create_file(&pdev->dev, &dev_attr_kcal_val);
	ret |= device_create_file(&pdev->dev, &dev_attr_kcal_cont);
	if (IS_ERR_VALUE(ret))
		pr_err("Unable to create sysfs nodes\n");

	return ret;
}

static int __devexit kcal_ctrl_remove(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_kcal_cont);
	device_remove_file(&pdev->dev, &dev_attr_kcal_val);
	device_remove_file(&pdev->dev, &dev_attr_kcal_sat);
	device_remove_file(&pdev->dev, &dev_attr_kcal_hue);
	device_remove_file(&pdev->dev, &dev_attr_kcal_min);
	device_remove_file(&pdev->dev, &dev_attr_kcal_invert);
	device_remove_file(&pdev->dev, &dev_attr_kcal_enable);
	device_remove_file(&pdev->dev, &dev_attr_kcal);

	return 0;
}

static struct platform_driver kcal_ctrl_driver = {
	.probe = kcal_ctrl_probe,
	.remove = kcal_ctrl_remove,
	.driver = {
		.name = "kcal_ctrl",
		.owner = THIS_MODULE,
	},
};

static struct platform_device kcal_ctrl_device = {
	.name = "kcal_ctrl",
};

static int __init kcal_ctrl_init(void)
{
	int ret;

	ret = platform_driver_register(&kcal_ctrl_driver);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to register platform driver\n");
		return ret;
	}

	ret = platform_device_register(&kcal_ctrl_device);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to register platform device\n");
		platform_driver_unregister(&kcal_ctrl_driver);
		return ret;
	}

	pr_info("Registered\n");

	return ret;
}

static void __exit kcal_ctrl_exit(void)
{
	platform_device_unregister(&kcal_ctrl_device);
	platform_driver_unregister(&kcal_ctrl_driver);
}

module_init(kcal_ctrl_init);
module_exit(kcal_ctrl_exit);