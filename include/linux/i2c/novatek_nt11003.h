/*
 * include/linux/i2c/novatek_nt11003.h
 *
 * Copyright (C) 2010 - 2011 Novatek, Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef 	_LINUX_NOVATEK_TOUCH_H 
#define		_LINUX_NOVATEK_TOUCH_H

#define NOVATEK_I2C_NAME "novatek-ts"

struct novatek_i2c_platform_data {
        uint32_t version;               /* Use this entry for panels with */

        int (*ts_init_platform_hw)(void);
        int (*ts_exit_platform_hw)(void);
        int (*get_probe_state)(void);	//-remove
        void (*set_probe_state)(int);  	//-remove
        int gpio_rst;
        int gpio_irq;
		int gpio_pwn;
        bool irq_edge;          /* 0:rising edge, 1:falling edge */
        uint16_t touch_max_x;		//-remove
        uint16_t touch_max_y;		//-remove
        uint16_t screen_max_x;
        uint16_t screen_max_y;
        u8 swap_xy :1;
        u8 xpol :1;
        u8 ypol :1;
};

#endif /* _LINUX_NOVATEK_TOUCH_H */
