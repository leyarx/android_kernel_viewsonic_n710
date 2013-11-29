/*
 * Copyright (c) 2013, Yaroslav Levandovskiy <leyarx@gmail.com>
 * Based on "ov2710.h"
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
 
#ifndef __HM2057_H__
#define __HM2057_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define HM2057_IOCTL_SET_MODE           _IOW('o', 1, struct hm2057_mode)
//#define HM2057_IOCTL_GET_STATUS         _IOR('o', 2, __u8)
#define HM2057_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, __u8)
#define HM2057_IOCTL_SET_EXPOSURE     	_IOW('o', 4, __u8)
#define HM2057_IOCTL_SET_WHITE_BALANCE  _IOW('o', 5, __u8)

struct hm2057_mode {
    int xres;
    int yres;
};

#ifdef __KERNEL__
struct hm2057_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
	int (*init)(void);
};
#endif /* __KERNEL__ */

#endif  /* __HM2057_H__ */