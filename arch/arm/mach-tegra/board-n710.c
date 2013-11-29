/*
 * arch/arm/mach-tegra/board-n710.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/regulator/consumer.h>
#include <linux/bq24160_charger.h>
#include <linux/leds.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/tegra_aic325x_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/thermal.h>
#include <mach/tegra_fiq_debugger.h>

#include "board.h"
#include "clock.h"
#include "board-n710.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "wdt-recovery.h"

/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.temp_throttle = 85000,
	.temp_shutdown = 90000,
	.temp_offset = TDIODE_OFFSET, /* temps based on tdiode */
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	.tc1 = 0,
	.tc2 = 1,
	.passive_delay = 2000,
#else
	.hysteresis_throttle = 10000,
#endif
};

/* !!!TODO: Change for N710 (Taken from Ventana & Grouper) */
static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[1] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[2] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};

/* wl128x BT, FM, GPS connectivity chip */
/**/
struct ti_st_plat_data kai_wilink_pdata = {
	.nshutdown_gpio = TEGRA_GPIO_PU0,
	.dev_name = BLUETOOTH_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3000000, //3686400,
};

static struct platform_device wl128x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &kai_wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static noinline void __init kai_bt_st(void)
{
	pr_info("kai_bt_st");

	platform_device_register(&wl128x_device);
	platform_device_register(&btwilink_device);
	tegra_gpio_enable(TEGRA_GPIO_PU0);
}

static struct resource kai_bluesleep_resources[] = {
	[0] = {
		.name = "host_wake", // "bt_host_wake"
			.start	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS6), //TEGRA_GPIO_PU6
			.end	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS6), //TEGRA_GPIO_PU6
			.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device kai_bluesleep_device = {
	.name		= "tibluesleep",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(kai_bluesleep_resources),
	.resource	= kai_bluesleep_resources,
};

static noinline void __init kai_tegra_setup_tibluesleep(void)
{
	platform_device_register(&kai_bluesleep_device);
	tegra_gpio_enable(TEGRA_GPIO_PS6); //TEGRA_GPIO_PU6
}

static __initdata struct tegra_clk_init_table n710_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	5100000,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_i2c_platform_data n710_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data n710_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data n710_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data n710_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data n710_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct i2c_board_info n710_i2c1_bq27x00_board_info[] = {
	{
		I2C_BOARD_INFO("bq27541-battery", 0x55)
	}
};

static struct bq24160_charger_platform_data n710_bq24160_pdata = {
	.vbus_gpio					= BQ24160_OTG_VBUS_GPIO,
	.bq24160_reg3 				= 0x8e, /* Battery Regulation Voltage: (3500mV) + 700 mV / Input Limit for IN input 2.5A */
	.bq24160_reg5 				= 0xee, /* Charge current: (550mA) + 2175mA / Termination current sense voltage: (50mA) + 300mA */
	.bq24160_reg5_susp 			= 0x6a, /* Charge current: (550mA) + 975mA / Termination current sense voltage: (50mA) + 100mA */
};

static void n710_bq24160_init(void)
{
	tegra_gpio_enable(BQ24160_IRQ_GPIO);
	gpio_request(BQ24160_IRQ_GPIO, "bq24160-charger");
	gpio_direction_input(BQ24160_IRQ_GPIO);	
}

static struct i2c_board_info n710_i2c4_bq24160_board_info[] = {
	{
		I2C_BOARD_INFO("bq24160", 0x6b),
		.platform_data = &n710_bq24160_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(BQ24160_IRQ_GPIO),
	},
};

static struct i2c_board_info __initdata n710_codec_aic325x_info = {
	I2C_BOARD_INFO("tlv320aic325x", 0x18),
};

static void n710_i2c_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	tegra_i2c_device1.dev.platform_data = &n710_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &n710_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &n710_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &n710_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &n710_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);

	n710_bq24160_init();
	
	i2c_register_board_info(4, n710_i2c4_bq24160_board_info,
		ARRAY_SIZE(n710_i2c4_bq24160_board_info));
		
	i2c_register_board_info(4, &n710_codec_aic325x_info, 1);

	i2c_register_board_info(1, n710_i2c1_bq27x00_board_info,
		ARRAY_SIZE(n710_i2c1_bq27x00_board_info));
}

static struct platform_device *n710_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data n710_uart_pdata;
static struct tegra_uart_platform_data n710_loopback_uart_pdata;

static unsigned int debug_uart_port_irq;

static char *uart_names[] = {
	"uarta",
	"uartb",
	"uartc",
	"uartd",
	"uarte",
};

static struct platform_device *debug_uarts[] = {
	&debug_uarta_device,
	&debug_uartb_device,
	&debug_uartc_device,
	//&debug_uartd_device,
	&debug_uarte_device,
};

static void __init uart_debug_init(void)
{
	int debug_port_id;
	struct platform_device *debug_uart;

	debug_port_id = get_tegra_uart_debug_port_id();
	if (debug_port_id < 0) {
		debug_port_id = 3;
	} else if (debug_port_id >= ARRAY_SIZE(debug_uarts)) {
		pr_info("The debug console id %d is invalid, Assuming UARTA",
			debug_port_id);
		debug_port_id = 0;
	}

	pr_info("Selecting %s as the debug port\n",
		uart_names[debug_port_id]);
	debug_uart_clk = clk_get_sys("serial8250.0",
				     uart_names[debug_port_id]);
	debug_uart = debug_uarts[debug_port_id];
	debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uart->dev.platform_data))->mapbase;
	debug_uart_port_irq = ((struct plat_serial8250_port *)(
			debug_uart->dev.platform_data))->irq;
	return;
}

static void __init n710_uart_init(void)
{
	struct clk *c;
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	n710_uart_pdata.parent_clk_list = uart_parent_clk;
	n710_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	n710_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	n710_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	n710_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &n710_uart_pdata;
	tegra_uartb_device.dev.platform_data = &n710_uart_pdata;
	tegra_uartc_device.dev.platform_data = &n710_uart_pdata;
	//tegra_uartd_device.dev.platform_data = &n710_uart_pdata;
	/* UARTE is used for loopback test purpose */
	tegra_uarte_device.dev.platform_data = &n710_loopback_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, clk_get_rate(c));
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	tegra_serial_debug_init(debug_uart_port_base, debug_uart_port_irq,
				debug_uart_clk, -1, -1, false);

	platform_add_devices(n710_uart_devices,
				ARRAY_SIZE(n710_uart_devices));
}

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device *n710_spi_devices[] __initdata = {
	&tegra_spi_device4,
	&tegra_spi_device1,
};

static struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data n710_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};

static void __init n710_spi_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	n710_spi_pdata.parent_clk_list = spi_parent_clk;
	n710_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device4.dev.platform_data = &n710_spi_pdata;
	platform_add_devices(n710_spi_devices,
				ARRAY_SIZE(n710_spi_devices));

}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct tegra_aic325x_platform_data n710_audio_aic325x_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_ext_mic_en	= -1,
};

static struct platform_device n710_audio_device = {
	.name	= "tegra-snd-aic325x",
	.id	= 0,
	.dev	= {
		.platform_data  = &n710_audio_aic325x_pdata,
	},
};

static struct gpio_led n710_led_info[] = {
	{
		.name			= "statled",
		.default_trigger	= "default-on",
		.gpio			= TEGRA_GPIO_STAT_LED,
		.active_low		= 1,
		.retain_state_suspended	= 0,
		.default_state		= LEDS_GPIO_DEFSTATE_OFF,
	},
};

static struct gpio_led_platform_data n710_leds_pdata = {
	.leds		= n710_led_info,
	.num_leds	= ARRAY_SIZE(n710_led_info),
};

static struct platform_device n710_leds_gpio_device = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &n710_leds_pdata,
	},
};

static struct platform_device *n710_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU) || defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
//	&n710_bcm4330_rfkill_device,
	&tegra_pcm_device,
	&n710_audio_device,
	&n710_leds_gpio_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
};
#if 0 //remove 
static __initdata struct tegra_clk_init_table spi_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "sbc1",       "pll_p",        52000000,       true},
	{ NULL,         NULL,           0,              0},
};

static __initdata struct tegra_clk_init_table touch_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "extern3",    "pll_p",        41000000,       true},
	{ "clk_out_3",  "extern3",      40800000,       true},
	{ NULL,         NULL,           0,              0},
};
#endif
#if defined(CONFIG_TOUCHSCREEN_NOVATEK)

#define TOUCH_GPIO_INT      TEGRA_GPIO_PJ0
#define TOUCH_GPIO_RST      TEGRA_GPIO_PK7
// Power pin:	
// Interrupt pin: TEGRA_GPIO_PJ0
// Reset pin: TEGRA_GPIO_PK7

#include <linux/i2c/novatek_nt11003.h>

static struct novatek_i2c_platform_data ts_novatek_nt11003_data[] = {
        {
			.version = 11003,               /* Use this entry for panels with */		
			.gpio_rst = TOUCH_GPIO_RST,
			.gpio_irq = TOUCH_GPIO_INT,
			//.gpio_pwn = TEGRA_GPIO_PH3,
			.irq_edge = 1,          /* 0:rising edge, 1:falling edge */
			.touch_max_x = 30 * 64, /* 1920 */
			.touch_max_y = 15 * 64, /* 960 */
			.screen_max_x = 1280,
			.screen_max_y = 800,
			.swap_xy = 1,
			.xpol = 1,
        },
};
static struct i2c_board_info novatek_i2c_devices[] = {
        {
                I2C_BOARD_INFO(NOVATEK_I2C_NAME, 0x01),//0x10
                .platform_data = &ts_novatek_nt11003_data,
                //.irq = (INT_GPIO_BASE + TOUCH_GPIO_INT),
        },

};
#endif

static int novatek_touch_init(void)
{
#if defined(CONFIG_TOUCHSCREEN_NOVATEK)
	tegra_gpio_enable(TOUCH_GPIO_INT); //TEGRA_GPIO_PH4
	tegra_gpio_enable(TOUCH_GPIO_RST); //TEGRA_GPIO_PH6

	gpio_request(TOUCH_GPIO_INT, "tp_int");
	gpio_direction_input(TOUCH_GPIO_INT); //TEGRA_GPIO_PG4

	gpio_request(TOUCH_GPIO_RST, "tp_rst");
	gpio_direction_output(TOUCH_GPIO_RST, 0); //TEGRA_GPIO_PG5

	i2c_register_board_info(1, novatek_i2c_devices, 1);
#endif
	return 0;
}

static int __init n710_touch_init(void)
{
//	int touch_id;

	tegra_gpio_enable(TEGRA_GPIO_PH4); //60
	tegra_gpio_enable(TEGRA_GPIO_PH5); //61
	tegra_gpio_enable(TEGRA_GPIO_PH6); //62
	tegra_gpio_enable(TEGRA_GPIO_PH7); //63
	
	gpio_request(TEGRA_GPIO_PH4, "tp_detect");
	gpio_direction_input(TEGRA_GPIO_PH4);
		
	gpio_request(TEGRA_GPIO_PH5, "tp_detect0");
	gpio_direction_input(TEGRA_GPIO_PH5);
		
	gpio_request(TEGRA_GPIO_PH6, "tp_detect1");
	gpio_direction_input(TEGRA_GPIO_PH6);
		
	gpio_request(TEGRA_GPIO_PH7, "tp_detect2");
	gpio_direction_input(TEGRA_GPIO_PH7);
	
	printk("touch detect %d %d %d %d\n",	gpio_get_value(TEGRA_GPIO_PH4),
											gpio_get_value(TEGRA_GPIO_PH5),
											gpio_get_value(TEGRA_GPIO_PH6),
											gpio_get_value(TEGRA_GPIO_PH7));
	
    return novatek_touch_init();	
}

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST, 
			.power_down_on_bus_suspend = 0,
			.default_enable = true,
	},
	[1] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.default_enable = false,
	},
};

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
};

#ifdef CONFIG_USB_SUPPORT
static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_gpio = -1,
			.vbus_irq = MAX77663_IRQ_BASE +
							MAX77663_IRQ_ACOK_FALLING,
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
};

static void n710_usb_init(void)
{	
	tegra_usb_phy_init(tegra_usb_phy_pdata,
			ARRAY_SIZE(tegra_usb_phy_pdata));

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
	platform_device_register(&tegra_ehci2_device);
}
/*
static void n710_modem_init(void)
{
	int ret;

	tegra_gpio_enable(TEGRA_GPIO_W_DISABLE);
	tegra_gpio_enable(TEGRA_GPIO_MODEM_RSVD1);
	tegra_gpio_enable(TEGRA_GPIO_MODEM_RSVD2);

	ret = gpio_request(TEGRA_GPIO_W_DISABLE, "w_disable_gpio");
	if (ret < 0)
		pr_err("%s: gpio_request failed for gpio %d\n",
			__func__, TEGRA_GPIO_W_DISABLE);
	else
		gpio_direction_output(TEGRA_GPIO_W_DISABLE, 1);


	ret = gpio_request(TEGRA_GPIO_MODEM_RSVD1, "Port_V_PIN_0");
	if (ret < 0)
		pr_err("%s: gpio_request failed for gpio %d\n",
			__func__, TEGRA_GPIO_MODEM_RSVD1);
	else
		gpio_direction_input(TEGRA_GPIO_MODEM_RSVD1);


	ret = gpio_request(TEGRA_GPIO_MODEM_RSVD2, "Port_H_PIN_7");
	if (ret < 0)
		pr_err("%s: gpio_request failed for gpio %d\n",
			__func__, TEGRA_GPIO_MODEM_RSVD2);
	else
		gpio_direction_output(TEGRA_GPIO_MODEM_RSVD2, 1);

}
*/
#else
static void n710_usb_init(void) { }
//static void n710_modem_init(void) { }
#endif

unsigned int boot_reason=0;
void n710_booting_info(void )
{
	static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	unsigned int reg;
	#define PMC_RST_STATUS_WDT (1)
	#define PMC_RST_STATUS_SW   (3)

	reg = readl(pmc +0x1b4);

	if (reg ==PMC_RST_STATUS_SW){
		boot_reason=PMC_RST_STATUS_SW;
		printk("n710_booting_info-SW reboot\n");
	} else if (reg ==PMC_RST_STATUS_WDT){
		boot_reason=PMC_RST_STATUS_WDT;
		printk("n710_booting_info-watchdog reboot\n");
	} else{
		boot_reason=0;
		printk("n710_booting_info-normal\n");
	}
}

static void __init tegra_n710_init(void)
{
	tegra_thermal_init(&thermal_data);
	tegra_clk_init_from_table(n710_clk_init_table);
	n710_pinmux_init();
	
	n710_booting_info();
	n710_i2c_init();	
	n710_spi_init();	
	n710_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	n710_edp_init();
#endif
	
	n710_uart_init();

printk("*** platform_add_devices\n");	
	platform_add_devices(n710_devices, ARRAY_SIZE(n710_devices));
printk("*** tegra_ram_console_debug_init\n");
	tegra_ram_console_debug_init();
printk("*** n710_sdhci_init\n");	
	n710_sdhci_init();
printk("*** regulator_init\n");	
	n710_regulator_init();
	n710_suspend_init();
printk("*** n710_touch_init\n");	
	n710_touch_init();
printk("*** n710_keys_init\n");	
	n710_keys_init();	
printk("*** n710_panel_init\n");	
	n710_panel_init();
printk("*** kai_bt_st\n");		
	kai_bt_st();
	kai_tegra_setup_tibluesleep();

printk("*** n710_sensors_init\n");		
	n710_sensors_init();
	
/*	
printk("*** n710_pins_state_init\n");		
	n710_pins_state_init();
*/	
	n710_emc_init();
//	tegra_release_bootloader_fb(); 
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
}

static void __init n710_ramconsole_reserve(unsigned long size)
{
	tegra_ram_console_debug_reserve(SZ_1M);
}

static void __init tegra_n710_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* support 1920X1200 with 24bpp */
	tegra_reserve(0, SZ_8M + SZ_1M, SZ_8M + SZ_1M);
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
	n710_ramconsole_reserve(SZ_1M);
}

MACHINE_START(N710, "N710")
	.boot_params	= 0x80000100,
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_n710_reserve,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.timer		= &tegra_timer,
	.init_machine	= tegra_n710_init,
MACHINE_END

#ifdef CONFIG_MACH_QC750
MACHINE_START(QC750, "QC750")
	.boot_params	= 0x80000100,
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_n710_reserve,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.timer		= &tegra_timer,
	.init_machine	= tegra_n710_init,
MACHINE_END
#endif