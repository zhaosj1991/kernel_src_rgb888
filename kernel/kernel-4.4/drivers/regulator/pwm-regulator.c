/*
 * Regulator driver for PWM Regulators
 *
 * Copyright (C) 2014 - STMicroelectronics Inc.
 *
 * Author: Lee Jones <lee.jones@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pwm.h>
#include <linux/gpio/consumer.h>

struct pwm_regulator_data {
	/*  Shared */
	struct pwm_device *pwm;

	/* Voltage table */
	struct pwm_voltages *duty_cycle_table;

	/* regulator descriptor */
	struct regulator_desc desc;

	/* regulator device */
	struct regulator_dev    *rdev;

	/* Regulator ops */
	struct regulator_ops ops;

	int state;

	/* Continuous voltage */
	int volt_uV;

	/* Enable GPIO */
	struct gpio_desc *enb_gpio;

	/* Voltage ramp time */
	u32 voltage_ramp_time;
};

struct pwm_voltages {
	unsigned int uV;
	unsigned int dutycycle;
};

/**
 * Voltage table call-backs
 */
static int pwm_regulator_get_voltage_sel(struct regulator_dev *rdev)
{
	struct pwm_regulator_data *drvdata = rdev_get_drvdata(rdev);

	return drvdata->state;
}

static int pwm_regulator_set_voltage_sel(struct regulator_dev *rdev,
					 unsigned selector)
{
	struct pwm_regulator_data *drvdata = rdev_get_drvdata(rdev);
	unsigned int pwm_reg_period;
	int dutycycle;
	int ret;

	pwm_reg_period = pwm_get_period(drvdata->pwm);

	dutycycle = (pwm_reg_period *
		    drvdata->duty_cycle_table[selector].dutycycle) / 100;

	ret = pwm_config(drvdata->pwm, dutycycle, pwm_reg_period);
	if (ret) {
		dev_err(&rdev->dev, "Failed to configure PWM: %d\n", ret);
		return ret;
	}

	drvdata->state = selector;

	return 0;
}

static int pwm_regulator_list_voltage(struct regulator_dev *rdev,
				      unsigned selector)
{
	struct pwm_regulator_data *drvdata = rdev_get_drvdata(rdev);

	if (selector >= rdev->desc->n_voltages)
		return -EINVAL;

	return drvdata->duty_cycle_table[selector].uV;
}

static int pwm_regulator_enable(struct regulator_dev *dev)
{
	struct pwm_regulator_data *drvdata = rdev_get_drvdata(dev);

	if (drvdata->enb_gpio)
		gpiod_set_value_cansleep(drvdata->enb_gpio, 1);

	return pwm_enable(drvdata->pwm);
}

static int pwm_regulator_disable(struct regulator_dev *dev)
{
	struct pwm_regulator_data *drvdata = rdev_get_drvdata(dev);

	pwm_disable(drvdata->pwm);

	if (drvdata->enb_gpio)
		gpiod_set_value_cansleep(drvdata->enb_gpio, 0);

	return 0;
}

static int pwm_regulator_is_enabled(struct regulator_dev *dev)
{
	struct pwm_regulator_data *drvdata = rdev_get_drvdata(dev);

	if (drvdata->enb_gpio && !gpiod_get_value_cansleep(drvdata->enb_gpio))
		return false;

	return pwm_is_enabled(drvdata->pwm);
}

static int pwm_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct pwm_regulator_data *drvdata = rdev_get_drvdata(rdev);

	return drvdata->volt_uV;
}

static int pwm_regulator_set_voltage(struct regulator_dev *rdev,
					int min_uV, int max_uV,
					unsigned *selector)
{
	struct pwm_regulator_data *drvdata = rdev_get_drvdata(rdev);
	unsigned int ramp_delay = rdev->constraints->ramp_delay;
	unsigned int period = pwm_get_period(drvdata->pwm);
	unsigned int req_diff = min_uV - rdev->constraints->min_uV;
	unsigned int diff;
	unsigned int duty_pulse;
	u64 req_period;
	u32 rem;
	int ret;

	dev_dbg(&rdev->dev, "%s() is called with %d:%d\n",
				__func__, min_uV, max_uV);

	diff = rdev->constraints->max_uV - rdev->constraints->min_uV;

	/* First try to find out if we get the iduty cycle time which is
	 * factor of PWM period time. If (request_diff_to_min * pwm_period)
	 * is perfect divided by voltage_range_diff then it is possible to
	 * get duty cycle time which is factor of PWM period. This will help
	 * to get output voltage nearer to requested value as there is no
	 * calculation loss.
	 */
	req_period = req_diff;
	req_period *= period;
	div_u64_rem(req_period, diff, &rem);
	if (!rem) {
		do_div(req_period, diff);
		duty_pulse = (unsigned int)req_period;
	} else {
		duty_pulse = (period / 100) * ((req_diff * 100) / diff);
	}

	ret = pwm_config(drvdata->pwm, duty_pulse, period);
	if (ret) {
		dev_err(&rdev->dev, "Failed to configure PWM: %d\n", ret);
		return ret;
	}

	ret = pwm_enable(drvdata->pwm);
	if (ret) {
		dev_err(&rdev->dev, "Failed to enable PWM: %d\n", ret);
		return ret;
	}
	drvdata->volt_uV = min_uV;

	/* Delay required by PWM regulator to settle to the new voltage */
	usleep_range(ramp_delay, ramp_delay + 1000);

	return 0;
}

static int pwm_regulator_set_voltage_time_sel(struct regulator_dev *rdev,
				unsigned int old_uV, unsigned int new_uV)
{
	struct pwm_regulator_data *drvdata = rdev_get_drvdata(rdev);

	return drvdata->voltage_ramp_time;
}

static struct regulator_ops pwm_regulator_voltage_table_ops = {
	.set_voltage_sel = pwm_regulator_set_voltage_sel,
	.get_voltage_sel = pwm_regulator_get_voltage_sel,
	.list_voltage    = pwm_regulator_list_voltage,
	.map_voltage     = regulator_map_voltage_iterate,
	.enable          = pwm_regulator_enable,
	.disable         = pwm_regulator_disable,
	.is_enabled      = pwm_regulator_is_enabled,
};

static struct regulator_ops pwm_regulator_voltage_continuous_ops = {
	.get_voltage = pwm_regulator_get_voltage,
	.set_voltage = pwm_regulator_set_voltage,
	.enable          = pwm_regulator_enable,
	.disable         = pwm_regulator_disable,
	.is_enabled      = pwm_regulator_is_enabled,
};

static struct regulator_desc pwm_regulator_desc = {
	.name		= "pwm-regulator",
	.type		= REGULATOR_VOLTAGE,
	.owner		= THIS_MODULE,
	.supply_name    = "pwm",
};

static int pwm_regulator_init_table(struct platform_device *pdev,
				    struct pwm_regulator_data *drvdata,
				    struct regulator_desc **desc)
{
	struct device_node *np = pdev->dev.of_node;
	struct pwm_voltages *duty_cycle_table;
	unsigned int length = 0;
	int ret;

	of_find_property(np, "voltage-table", &length);

	if ((length < sizeof(*duty_cycle_table)) ||
	    (length % sizeof(*duty_cycle_table))) {
		dev_err(&pdev->dev, "voltage-table length(%d) is invalid\n",
			length);
		return -EINVAL;
	}

	duty_cycle_table = devm_kzalloc(&pdev->dev, length, GFP_KERNEL);
	if (!duty_cycle_table)
		return -ENOMEM;

	*desc = devm_kzalloc(&pdev->dev, sizeof(**desc), GFP_KERNEL);
	if (!*desc)
		return -ENOMEM;
	**desc = pwm_regulator_desc;

	ret = of_property_read_u32_array(np, "voltage-table",
					 (u32 *)duty_cycle_table,
					 length / sizeof(u32));
	if (ret) {
		dev_err(&pdev->dev, "Failed to read voltage-table: %d\n", ret);
		return ret;
	}

	drvdata->duty_cycle_table	= duty_cycle_table;
	memcpy(&drvdata->ops, &pwm_regulator_voltage_table_ops,
	       sizeof(drvdata->ops));
	drvdata->desc.ops = &drvdata->ops;
	drvdata->desc.n_voltages	= length / sizeof(*duty_cycle_table);

	return 0;
}

static int pwm_regulator_init_continuous(struct platform_device *pdev,
					 struct pwm_regulator_data *drvdata,
					 struct regulator_desc **desc)
{
	memcpy(&drvdata->ops, &pwm_regulator_voltage_continuous_ops,
	       sizeof(drvdata->ops));

	if (!of_property_read_u32(pdev->dev.of_node, "voltage-time-sel",
				&drvdata->voltage_ramp_time))
		drvdata->ops.set_voltage_time_sel = pwm_regulator_set_voltage_time_sel;

	drvdata->desc.ops = &drvdata->ops;
	drvdata->desc.continuous_voltage_range = true;

	return 0;
}

static int pwm_regulator_probe(struct platform_device *pdev)
{
	const struct regulator_init_data *init_data;
	struct pwm_regulator_data *drvdata;
	struct regulator_dev *regulator;
	struct regulator_config config = { };
	struct device_node *np = pdev->dev.of_node;
	struct regulator_desc *desc;
	enum gpiod_flags gpio_flags;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "Device Tree node missing\n");
		return -EINVAL;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	memcpy(&drvdata->desc, &pwm_regulator_desc, sizeof(drvdata->desc));

	if (of_find_property(np, "voltage-table", NULL))
		ret = pwm_regulator_init_table(pdev, drvdata, &desc);
	else
		ret = pwm_regulator_init_continuous(pdev, drvdata, &desc);

	if (ret)
		return ret;

	init_data = of_get_regulator_init_data(&pdev->dev, np,
					       &drvdata->desc);
	if (!init_data)
		return -ENOMEM;

	if (init_data->constraints.boot_on || init_data->constraints.always_on)
		gpio_flags = GPIOD_OUT_HIGH;
	else
		gpio_flags = GPIOD_OUT_LOW;

	drvdata->enb_gpio = devm_gpiod_get_optional(&pdev->dev, "enable",
				gpio_flags);
	if (IS_ERR(drvdata->enb_gpio)) {
		ret = PTR_ERR(drvdata->enb_gpio);
		dev_err(&pdev->dev, "Failed to get enable GPIO: %d\n", ret);
		return ret;
	}

	config.of_node = np;
	config.dev = &pdev->dev;
	config.driver_data = drvdata;
	config.init_data = init_data;

	drvdata->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(drvdata->pwm)) {
		ret = PTR_ERR(drvdata->pwm);
		dev_err(&pdev->dev, "Failed to get PWM: %d\n", ret);
		return ret;
	}

	regulator = devm_regulator_register(&pdev->dev,
					    &drvdata->desc, &config);
	if (IS_ERR(regulator)) {
		ret = PTR_ERR(regulator);
		dev_err(&pdev->dev, "Failed to register regulator %s: %d\n",
			drvdata->desc.name, ret);
		return ret;
	}
	drvdata->rdev = regulator;
	platform_set_drvdata(pdev, drvdata);
	dev_info(&pdev->dev, "PWM regulator registration passed\n");
	return 0;
}
#ifdef CONFIG_PM_SLEEP
static int pwm_regulator_resume(struct device *dev)
{
	struct pwm_regulator_data *drvdata = dev_get_drvdata(dev);

	if (drvdata->desc.continuous_voltage_range)
		pwm_regulator_set_voltage(drvdata->rdev, drvdata->volt_uV,
				drvdata->volt_uV, &drvdata->state);
	else
		pwm_regulator_set_voltage_sel(drvdata->rdev, drvdata->state);

	return 0;
}
#endif
static const struct dev_pm_ops pwm_regulator_pm_ops = {
		SET_SYSTEM_SLEEP_PM_OPS(NULL, pwm_regulator_resume)
};

static const struct of_device_id pwm_of_match[] = {
	{ .compatible = "pwm-regulator" },
	{ },
};
MODULE_DEVICE_TABLE(of, pwm_of_match);

static struct platform_driver pwm_regulator_driver = {
	.driver = {
		.name		= "pwm-regulator",
		.of_match_table = of_match_ptr(pwm_of_match),
		.pm = &pwm_regulator_pm_ops,
	},
	.probe = pwm_regulator_probe,
};

module_platform_driver(pwm_regulator_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lee Jones <lee.jones@linaro.org>");
MODULE_DESCRIPTION("PWM Regulator Driver");
MODULE_ALIAS("platform:pwm-regulator");
