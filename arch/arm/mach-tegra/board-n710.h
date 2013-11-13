/*
 * arch/arm/mach-tegra/board-n720.h
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

#ifndef _MACH_TEGRA_BOARD_N710_H
#define _MACH_TEGRA_BOARD_N710_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/max77663-core.h>
#include "gpio-names.h"

/* Processor Board  ID */
#define BOARD_E1565	0xF41

/* Board Fab version */
#define BOARD_FAB_A00			0x0
#define BOARD_FAB_A01			0x1
#define BOARD_FAB_A02			0x2
#define BOARD_FAB_A03			0x3
#define BOARD_FAB_A04			0x4
#define BOARD_FAB_A05			0x5

/* External peripheral act as gpio */
/* MAX77663 GPIO */
#define MAX77663_GPIO_BASE	TEGRA_NR_GPIOS
#define MAX77663_GPIO_END	(MAX77663_GPIO_BASE + MAX77663_GPIO_NR)

/* CAMERA RELATED GPIOs on N710 */
#define CAM2_RST_GPIO		TEGRA_GPIO_PBB4
#define CAM2_POWER_DWN_GPIO	TEGRA_GPIO_PBB6
/* Audio-related GPIOs */
#define TEGRA_GPIO_SPKR_EN			TEGRA_GPIO_PB1
#define TEGRA_GPIO_HP_DET			TEGRA_GPIO_PW2
/* Tegra Modem related GPIOs */
#define TEGRA_GPIO_W_DISABLE		TEGRA_GPIO_PDD7
#define TEGRA_GPIO_MODEM_RSVD1		TEGRA_GPIO_PV0
#define TEGRA_GPIO_MODEM_RSVD2		TEGRA_GPIO_PH7

/* Stat LED GPIO */
#define TEGRA_GPIO_STAT_LED		(MAX77663_GPIO_BASE + MAX77663_GPIO7)

/* Power GPIO's */
#define BQ24160_IRQ_GPIO			TEGRA_GPIO_PK2
#define BQ24160_OTG_VBUS_GPIO		TEGRA_GPIO_PH1

/*****************Interrupt tables ******************/
/* External peripheral act as interrupt controller */
/* MAX77663 IRQs */
#define MAX77663_IRQ_BASE	TEGRA_NR_IRQS
#define MAX77663_IRQ_END	(MAX77663_IRQ_BASE + MAX77663_IRQ_NR)
#define MAX77663_IRQ_ACOK_RISING MAX77663_IRQ_ONOFF_ACOK_RISING
#define MAX77663_IRQ_ACOK_FALLING MAX77663_IRQ_ONOFF_ACOK_FALLING

/* UART port which is used by bluetooth*/
#define BLUETOOTH_UART_DEV_NAME "/dev/ttyHS2"

/*
UART2- GPS
UART3- BT (uartc)
UART4- TP54 TP65

USB1- HOST
USB2- 3G
USB3- HOST

SDMMC3- WI-FI
*/

//int n710_charge_init(void);
int n710_regulator_init(void);
int n710_suspend_init(void);
int n710_sdhci_init(void);
int n710_pinmux_init(void);
int n710_panel_init(void);
int n710_sensors_init(void);
int n710_keys_init(void);
int n710_pins_state_init(void);
int n710_emc_init(void);
int n710_edp_init(void);

void __init n710_tsensor_init(void);

#define AKM8975_IRQ_GPIO	TEGRA_GPIO_PW0

#define TDIODE_OFFSET	(10000) /* in millicelsius */

#endif /* _MACH_TEGRA_BOARD_N710_H */
