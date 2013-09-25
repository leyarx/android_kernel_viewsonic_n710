/*
 * arch/arm/mach-tegra/board-n710-kbc.c
 * Keys configuration for Nvidia tegra3 n710 platform.
 *
 * Copyright (C) 2012 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>

#include <mach/irqs.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include "board.h"
#include "board-n710.h"

#include "gpio-names.h"
#include "devices.h"

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}
	
#define GPIO_IKEY(_id, _irq, _iswake, _deb)	\
	{					\
		.code = _id,			\
		.gpio = -1,			\
		.irq = _irq,			\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = _deb,	\
	}
	
static struct gpio_keys_button n710_keys[] = {
	[0] = GPIO_KEY(KEY_VOLUMEUP, PR0, 0),
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, PR1, 0),
	[2] = GPIO_IKEY(KEY_POWER, MAX77663_IRQ_BASE + MAX77663_IRQ_ONOFF_EN0_FALLING, 1, 100),
	[3] = GPIO_IKEY(KEY_POWER, MAX77663_IRQ_BASE + MAX77663_IRQ_ONOFF_EN0_1SEC, 1, 3000),
};

static struct gpio_keys_platform_data n710_keys_platform_data = {
	.buttons	= n710_keys,
	.nbuttons	= ARRAY_SIZE(n710_keys),
};

static struct platform_device n710_keys_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &n710_keys_platform_data,
	},
};

int __init n710_keys_init(void)
{
	int i;
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	BUG_ON(board_info.board_id != BOARD_E1565);

	pr_info("Registering gpio keys\n");

	/* Enable gpio mode for other pins */
	for (i = 0; i < n710_keys_platform_data.nbuttons; i++)
		tegra_gpio_enable(n710_keys_platform_data.
					buttons[i].gpio);

	platform_device_register(&n710_keys_device);

	return 0;
}
