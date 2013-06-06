/*
 * Touch Screen I2C driver
 *
 * Copyright (c) 2011 Touchplus Inc.
 *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU  General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __TOUCHPLUS_I2C_TS_H__
#define __TOUCHPLUS_I2C_TS_H__ 


//#define MAX_FINGER  5

#define X_COORDINATE_INVERT
#define Y_COORDINATE_INVERT

#if defined (CONFIG_TOUCHSCREEN_1024X768)
#define TOUCHSCREEN_MAX_X 1024
#define TOUCHSCREEN_MAX_Y 768
#elif defined (CONFIG_TOUCHSCREEN_1024X600)
#define TOUCHSCREEN_MAX_X 1024
#define TOUCHSCREEN_MAX_Y 600
#elif defined (CONFIG_TOUCHSCREEN_800X600)
#define TOUCHSCREEN_MAX_X 800
#define TOUCHSCREEN_MAX_Y 600
#elif defined (CONFIG_TOUCHSCREEN_800X480)
#define TOUCHSCREEN_MAX_X 800
#define TOUCHSCREEN_MAX_Y 480
#define TPF_FORMAT                // defined for YF7, undefined for YF8
#endif

#define IOCTL_MAGIC_NUMBER   0x55

enum{
    IOCTL_AUTOTUNE_SET_ID = 1,
    IOCTL_AUTOTUNE_GET_ID,
    IOCTL_ENABLE_IRQ_ID,
    IOCTL_DISABLE_IRQ_ID,
    IOCTL_CMDMODE_SET_ID,
    IOCTL_MSIREG_SET_ID,
    IOCTL_MSIREG_GET_ID,
    IOCTL_BOOTLOADER_ID,
    IOCTL_RESET_TSP_ID,
	IOCTL_INT_PIN_ID,

    IOCTL_ID_INVALID,
};

#define IOCTL_CMD_AUTOTUNE_SET    _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_AUTOTUNE_SET_ID,1)
#define IOCTL_CMD_AUTOTUNE_GET    _IOC(_IOC_READ,   IOCTL_MAGIC_NUMBER, IOCTL_AUTOTUNE_GET_ID,1)
#define IOCTL_CMD_ENABLE_IRQ      _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_ENABLE_IRQ_ID,1)
#define IOCTL_CMD_DISABLE_IRQ     _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_DISABLE_IRQ_ID,1)
#define IOCTL_CMD_CMDMODE_SET     _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_CMDMODE_SET_ID,1)
#define IOCTL_CMD_MSIREG_SET      _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_MSIREG_SET_ID,1)
#define IOCTL_CMD_MSIREG_GET      _IOC(_IOC_READ,   IOCTL_MAGIC_NUMBER, IOCTL_MSIREG_GET_ID,1)
#define IOCTL_CMD_BOOTLOADER_SET  _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_BOOTLOADER_ID,1)
#define IOCTL_CMD_RESET_TSP_SET   _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_RESET_TSP_ID,1)
#define IOCTL_CMD_INT_PIN_GET     _IOC(_IOC_WRITE,  IOCTL_MAGIC_NUMBER, IOCTL_INT_PIN_ID,1)

#define TOUCHPLUS_BOOTLOADER_I2C_ADDRESS  0x5D    // 7-bit addressing
#define TOUCHPLUS_NORMAL_I2C_ADDRESS      0x5C    // 7-bit addressing


#endif

