/*
 * arch/arm/mach-tegra/board-n710-sensors.c
 *
 * Copyright (c) 2013, Yaroslav Levandovskiy <leyarx@gmail.com>
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/mpu.h>
#include <linux/regulator/consumer.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>
#ifdef CONFIG_SOC_CAMERA_HM2057
#include <media/hm2057.h>
#endif
#ifdef CONFIG_SENSORS_AK8975
#include <linux/akm8975.h>
#endif
#ifdef CONFIG_INPUT_KXTIK
#include <linux/input/kxtik.h>
#endif
#include "board.h"
#include "board-n710.h"
#include "cpu-tegra.h"
#include <linux/nct1008.h>
#include <mach/thermal.h>
#include <linux/slab.h>

#ifdef CONFIG_SOC_CAMERA_HM2057

static struct regulator *n710_1v8_cam1;
static struct regulator *n710_vdd_cam1;

#define CAM_HM2057_RST_GPIO			TEGRA_GPIO_PBB0
#define CAM_HM2057_POWER_DWN_GPIO	TEGRA_GPIO_PBB5

static int n710_hm2057_power_on(void)
{
	gpio_set_value(CAM_HM2057_POWER_DWN_GPIO, 0);

	if (n710_1v8_cam1 == NULL) {
		n710_1v8_cam1 = regulator_get(NULL, "vdd_1v8_cam1");
		if (WARN_ON(IS_ERR(n710_1v8_cam1))) {
			pr_err("%s: couldn't get regulator vdd_1v8_cam1: %ld\n",
				__func__, PTR_ERR(n710_1v8_cam1));
			goto reg_get_vdd_1v8_cam1_fail;
		}
	}

	regulator_enable(n710_1v8_cam1);

	if (n710_vdd_cam1 == NULL) {
		n710_vdd_cam1 = regulator_get(NULL, "vdd_cam1");
		if (WARN_ON(IS_ERR(n710_vdd_cam1))) {
			pr_err("%s: couldn't get regulator vdd_cam1: %ld\n",
				__func__, PTR_ERR(n710_vdd_cam1));
			goto reg_get_vdd_cam1_fail;
		}
	}

	regulator_enable(n710_vdd_cam1);
	
	gpio_set_value(CAM_HM2057_RST_GPIO, 1);

	mdelay(100);
	
	return 0;

reg_get_vdd_cam1_fail:
	n710_vdd_cam1 = NULL;	
	regulator_put(n710_1v8_cam1);
	
reg_get_vdd_1v8_cam1_fail:
	n710_1v8_cam1 = NULL;
	
	return -ENODEV;
}

static int n710_hm2057_power_off(void)
{
	gpio_direction_output(CAM_HM2057_POWER_DWN_GPIO, 1);

	gpio_direction_output(CAM_HM2057_RST_GPIO, 0);

	if (n710_1v8_cam1)
		regulator_disable(n710_1v8_cam1);
	if (n710_vdd_cam1)
		regulator_disable(n710_vdd_cam1);

	return 0;
}

static int n710_hm2057_init(void)
{
	int ret;

	tegra_gpio_enable(CAM_HM2057_POWER_DWN_GPIO);
	ret = gpio_request(CAM_HM2057_POWER_DWN_GPIO, "hm2057_pwdn");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM_HM2057_POWER_DWN_GPIO");
	}

	gpio_direction_output(CAM_HM2057_POWER_DWN_GPIO, 1);
	mdelay(10);

	tegra_gpio_enable(CAM_HM2057_RST_GPIO);
	ret = gpio_request(CAM_HM2057_RST_GPIO, "hm2057_reset");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM_HM2057_RST_GPIO");
	}

	gpio_direction_output(CAM_HM2057_RST_GPIO, 0);
	mdelay(5);	
	
	return 0;
}

struct hm2057_platform_data n710_hm2057_data = {
	.power_on = n710_hm2057_power_on,
	.power_off = n710_hm2057_power_off,
	.init = n710_hm2057_init,
};

static struct i2c_board_info n710_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("hm2057", 0x24),
		.platform_data = &n710_hm2057_data,
	},
};
#endif

#ifndef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data, shutdown_temp);
}

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *thermal_device;

	thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!thermal_device) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	thermal_device->name = "nct1008";
	thermal_device->data = data;
	thermal_device->offset = TDIODE_OFFSET;
	thermal_device->get_temp = nct_get_temp;
	thermal_device->get_temp_low = nct_get_temp_low;
	thermal_device->set_limits = nct_set_limits;
	thermal_device->set_alert = nct_set_alert;
	thermal_device->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_set_device(thermal_device);
}
#endif

static struct nct1008_platform_data n710_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
#ifndef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
	.probe_callback = nct1008_probe_callback,
#endif
};

static struct i2c_board_info n710_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &n710_nct1008_pdata,
		.irq = -1,
	}
};

static int n710_nct1008_init(void)
{
	int nct1008_port = -1;
	int ret = 0;

	nct1008_port = TEGRA_GPIO_PS3;
	if (nct1008_port >= 0) {
		/* FIXME: enable irq when throttling is supported */
		n710_i2c4_nct1008_board_info[0].irq = TEGRA_GPIO_TO_IRQ(nct1008_port);

		ret = gpio_request(nct1008_port, "temp_alert");
		if (ret < 0)
			return ret;

		ret = gpio_direction_input(nct1008_port);
		if (ret < 0)
			gpio_free(nct1008_port);
		else
			tegra_gpio_enable(nct1008_port);
	}

	return ret;
}
#ifdef CONFIG_INPUT_KXTIK
static struct kxtik_platform_data n710_kxtik_pdata = {
	.min_interval = 66,
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
//	.negate_x = 1,
//	.negate_z = 1,
	.res_12bit = RES_12BIT,
	.g_range = KXTIK_G_2G,
};
#endif
#ifdef CONFIG_SENSORS_AK8975
static struct akm8975_platform_data n710_akm8975_pdata = {
	.intr = 0,
	.init = NULL,
	.exit = NULL,
	.power_on = NULL,
	.power_off = NULL,
};

static void n710_akm8975_init(void)
{
	tegra_gpio_enable(AKM8975_IRQ_GPIO);
	gpio_request(AKM8975_IRQ_GPIO, "akm8975");
	gpio_direction_input(AKM8975_IRQ_GPIO);
}
#endif

static struct i2c_board_info n710_i2c0_board_info[] = {
#ifdef CONFIG_INPUT_KXTIK //SENSORS_KXTIK
	{
		I2C_BOARD_INFO("kxtik", 0x0F), //kxtik
		.platform_data = &n710_kxtik_pdata,
//		.irq = TEGRA_GPIO_TO_IRQ(0), // Disable ACCELIRQ: TEGRA_GPIO_PN4
	},
#endif
#ifdef CONFIG_SENSORS_AK8975
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.platform_data = &n710_akm8975_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(AKM8975_IRQ_GPIO),
	},
#endif
};

int __init n710_sensors_init(void)
{
	int err;

#ifdef CONFIG_SOC_CAMERA_HM2057
	i2c_register_board_info(2, n710_i2c2_board_info,
		ARRAY_SIZE(n710_i2c2_board_info));
#endif

#ifdef CONFIG_SENSORS_AK8975
	n710_akm8975_init();
#endif

	err = n710_nct1008_init();
	if (err)
		printk("[Error] Thermal: Configure GPIO_PCC2 as an irq fail!");
		
	i2c_register_board_info(0, n710_i2c0_board_info,
		ARRAY_SIZE(n710_i2c0_board_info));	
		
	i2c_register_board_info(4, n710_i2c4_nct1008_board_info,
		ARRAY_SIZE(n710_i2c4_nct1008_board_info));

	return 0;
}
