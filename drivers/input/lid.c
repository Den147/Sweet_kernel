/*
 * Copyright (C) 2013, ASUSTek Computer Inc.
 * Copyright (C) 2018, The Linux Foundation. All rights reserved.
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

#define ASUS_LID	"asustek_lid"
#define pr_fmt(fmt)	ASUS_LID ": %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>

/* LID input device configuration */
#define LID_NAME	"lid_input"
#define LID_PHYS	"/dev/input/lid_indev"

/* LID device GPIO number */
#define LID_GPIO	(36)

/* Time in milliseconds to sleep before getting the state of the GPIO above */
#define LID_DELAY	(50)

/* Internal LID device data */
struct lid_data {
	/* LID input device structure */
	struct input_dev *idev;

	/* A work used to report GPIO state to the LID device */
	struct delayed_work report;

	/* Current state of a LID device */
	unsigned int status:1;

	/* GPIO identificator assosiated with LID device */
	unsigned int gpio;

	/* Interrupt request assosiated with LID device */
	unsigned int irq;

	/* Delay in milliseconds to sleep before the report of a LID input */
	unsigned int delay;
};

/* Global pointer to LID device data */
static struct lid_data *data;

/* A switch to enable the handling of interrupts for LID device */
static unsigned int __read_mostly enable_lid = 1;
module_param(enable_lid, uint, 0644);

/**
 * lid_report() - main function of a LID driver.
 * @work: pointer to a work structure.
 *
 * This work reports the changed GPIO state to the LID device.
 * Following actions will be taken by the LID device's firmware itself.
 */
static void lid_report(struct work_struct *work)
{
	/* Save new GPIO state */
	data->status = !gpio_get_value(data->gpio);

	/* Report the changed state to the LID device */
	input_report_switch(data->idev, SW_LID, data->status);
	input_sync(data->idev);
}

/**
 * lid_interrupt_handler() - handle a LID interrupt.
 * @irq: number of a waiting-to-be-handled interrupt.
 * @dev_id: cookie assosiated with @irq. Unused as we have global data.
 *
 * Queues lid_report() after "delay" milliseconds if a specified IRQ is the
 * one assigned to the LID device.  lid_report() will report the GPIO state
 * change due to an interrupt to the LID driver.
 */
static irqreturn_t lid_interrupt_handler(int irq, void *dev_id)
{
	if (enable_lid && irq == data->irq)
		queue_delayed_work(&data->report,
				      msecs_to_jiffies(data->delay));

	return IRQ_HANDLED;
}

/**
 * lid_create_device() - create a LID input device.
 * @idev: pointer to a LID input device.
 * @name: name of a LID input device.
 * @phys: phandle to a LID input device.
 *
 * Tries to allocate memory for input device and set up its configuration
 * according to the requirements for LID input device. This function also
 * tries to register a freshly created device, thus it creates a completely
 * work-ready input device as a result.
 *
 * If an input device was allocated by this function, lid_destroy_device()
 * should be used to remove it and free the memory occupied by it.
 */
static inline int
lid_create_device(struct input_dev *idev, const char *name, const char *phys)
{
	int ret;

	/* Allocate general purpose input device */
	idev = input_allocate_device();
	if (IS_ERR_OR_NULL(idev))
		return -ENOMEM;

	/* Assign provided data to that device */
	idev->name = name;
	idev->phys = phys;

	/* Set bits appropriate for LID device */
	set_bit(EV_SW, idev->evbit);
	set_bit(SW_LID, idev->swbit);

	ret = input_register_device(idev);
	if (IS_ERR_VALUE(ret)) {
		input_free_device(idev);
		idev = NULL;
	}

	return ret;
}

/**
 * lid_destroy_device() - destroy a LID input device.
 * @dev: pointer to a LID input device.
 *
 * Tries to destroy a passed input device. Note that input device must be
 * unused at the moment of the destruction.  Otherwise, unexpected things
 * might happen.
 */
static inline void lid_destroy_device(struct input_dev *dev)
{
	/* Return early if an input device does not exist */
	if (IS_ERR_OR_NULL(dev))
		return;

	input_unregister_device(dev);
	input_free_device(dev);
}

/**
 * lid_request_irq() - request an interrupt for the GPIO.
 * @gpio: number of GPIO.
 *
 * Tries to convert a passed @gpio to an interrupt and request it from any
 * context.
 */
static inline int lid_request_irq(unsigned int gpio)
{
	unsigned long irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	int irq;

	/* Convert a passed GPIO to interrupt request */
	irq = gpio_to_irq(gpio);
	if (IS_ERR_VALUE(irq))
		return -EINVAL;

	/* Save the converted value in the driver */
	data->irq = irq;

	return request_any_context_irq(irq, lid_interrupt_handler,
				       irqflags, "hall_sensor", NULL);
}

#define show_one(name)						\
static ssize_t show_##name(struct device *dev,			\
			   struct device_attribute *attr,	\
			   char *buf)				\
{								\
	return scnprintf(buf, 12, "%u\n", data->name);		\
}

#define store_one(name, min, max)				\
static ssize_t store_##name(struct device *dev,			\
			    struct device_attribute *attr,	\
			    const char *buf, size_t count)	\
{								\
	int ret;						\
	unsigned int val;					\
								\
	ret = kstrtouint(buf, 10, &val);			\
	if (ret || val < min || val > max)			\
		return -EINVAL;					\
								\
	data->name = val;					\
								\
	return count;						\
}

#define create_one_rw(name)					\
static DEVICE_ATTR(lid_##name, 0644, show_##name, store_##name)

#define create_one_ro(name)					\
static DEVICE_ATTR(lid_##name, 0444, show_##name, NULL)

#define define_one_rw(name, min, max)				\
show_one(name);							\
store_one(name, min, max);					\
create_one_rw(name)

#define define_one_ro(name)					\
show_one(name);							\
create_one_ro(name)

define_one_rw(delay, 0, 1000);
define_one_ro(status);
define_one_ro(gpio);
define_one_ro(irq);

static struct attribute *lid_attrs[] = {
	&dev_attr_lid_status.attr,
	&dev_attr_lid_delay.attr,
	&dev_attr_lid_gpio.attr,
	&dev_attr_lid_irq.attr,
	NULL
};

static struct attribute_group lid_attr_group = {
	.attrs = lid_attrs,
};

static int __devinit lid_probe(struct platform_device *pdev)
{
	int ret;

	/* Allocate LID device data and set up default values */
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (IS_ERR_OR_NULL(data)) {
		pr_err("Unable to allocate memory for LID device data\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, data);

	data->gpio = LID_GPIO;
	data->delay = LID_DELAY;

	/* Try to create LID input device */
	ret = lid_create_device(data->idev, LID_NAME, LID_PHYS);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to allocate LID input device\n");
		goto fail_create;
	}

	/* Try to request a GPIO assosiated with the LID device */
	ret = gpio_request_one(data->gpio, GPIOF_DIR_IN, "LID");
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to request a GPIO number %u\n", data->gpio);
		goto fail_gpio;
	}

	/* All the required data is initialized; we can finally init the work */
	INIT_DELAYED_WORK_DEFERRABLE(&data->report, lid_report);

	/* Try to request an IRQ for the LID device */
	ret = lid_request_irq(data->gpio);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to request an IRQ for the LID device\n");
		goto fail_irq;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &lid_attr_group);
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to create sysfs group\n");
		goto fail_sysfs;
	}

	device_init_wakeup(&pdev->dev, 1);
	enable_irq_wake(data->irq);

	return 0;

fail_sysfs:
	free_irq(data->irq, NULL);
fail_irq:
	gpio_free(data->gpio);
fail_gpio:
	lid_destroy_device(data->idev);
fail_create:
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, data);

	return ret;
}

static int __devexit lid_remove(struct platform_device *pdev)
{
	disable_irq_wake(data->irq);
	device_init_wakeup(&pdev->dev, 0);
	sysfs_remove_group(&pdev->dev.kobj, &lid_attr_group);

	free_irq(data->irq, NULL);
	gpio_free(data->gpio);

	lid_destroy_device(data->idev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver lid_driver = {
	.probe = lid_probe,
	.remove = __devexit_p(lid_remove),
	.driver = {
		.name = ASUS_LID,
		.owner = THIS_MODULE,
	},
};

module_platform_driver(lid_driver);

MODULE_AUTHOR("ASUSTek Computer Inc.");
MODULE_DESCRIPTION("Hall Sensor Driver");
MODULE_LICENSE("GPL v2");
