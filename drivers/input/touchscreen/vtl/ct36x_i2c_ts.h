#ifndef CT36X_TS_H
#define CT36X_TS_H

#include <linux/gpio.h>

#define CT360_CHIP_VER				0x02	// CT360
#define CT36X_CHIP_VER				0x01	// CT363, CT365

#define CT360_FW_VER_OFFSET			16372	//
#define CT360_FW_SIZE				16384	//

#define CT36X_FW_VER_OFFSET			32756	//
#define CT36X_FW_SIZE				32768	//
#define CT36X_TS_DEBUG				0

#if defined(CONFIG_CT360_CHIP_VER)
#define CT36X_TS_CHIP_SEL			CT360_CHIP_VER
#else
#define CT36X_TS_CHIP_SEL			CT36X_CHIP_VER
#endif

#define CT36X_TS_POINT_NUM			10		//max touch points supported

#if defined(CONFIG_TOUCH_MAX_X)
#define CT36X_TS_ABS_X_MAX			CONFIG_TOUCH_MAX_X
#else
#define CT36X_TS_ABS_X_MAX			2048	//1280
#endif

#if defined(CONFIG_TOUCH_MAX_Y)
#define CT36X_TS_ABS_Y_MAX			CONFIG_TOUCH_MAX_Y
#else
#define CT36X_TS_ABS_Y_MAX			1536	//800
#endif

#define CT36X_TS_X_REVERSE			0
#define CT36X_TS_Y_REVERSE			0
#define CT36X_TS_XY_SWAP			0
#define CT36X_TS_PTS_VER			1	// Touch Point protocol 0:old 1:new

#if (CT36X_TS_CHIP_SEL == CT360_CHIP_VER)
#define CT36X_TS_FW_VER_OFFSET			CT360_FW_VER_OFFSET	//
#define CT36X_TS_FW_SIZE				CT360_FW_SIZE	//

#elif (CT36X_TS_CHIP_SEL == CT36X_CHIP_VER)
#define CT36X_TS_FW_VER_OFFSET			CT36X_FW_VER_OFFSET	//
#define CT36X_TS_FW_SIZE				CT36X_FW_SIZE	//
#endif

#define CT36X_TS_I2C_BUS			2				// I2C Bus
#define CT36X_TS_I2C_ADDRESS		0x01			// I2C Chip Address
#define CT36X_TS_I2C_SPEED			(300*1024)		// I2C Bus Speed 300KHz
#define CT36X_TS_IRQ_PIN			RK30_PIN4_PC2	// RK29_PIN0_PA2
#define CT36X_TS_RST_PIN			RK30_PIN4_PD0	// RK29_PIN6_PC3

#define CT36X_TS_HAS_EARLYSUSPEND			1		/* ct36x_zx */

// #define CT36X_TS_ESD_TIMER_INTERVAL			0//5	// Sec
#define CT36X_TS_ESD_TIMER_INTERVAL			5	// Sec Reenabled by Astralix

#if (CT36X_TS_CHIP_SEL == CT360_CHIP_VER)
struct ct36x_finger_info {
#if (!CT36X_TS_PTS_VER)
	unsigned char	status : 4; 	// Action information, 1: Down; 2: Move; 3: Up
	unsigned char	id : 4; 		// ID information, from 1 to CFG_MAX_POINT_NUM
#endif
	unsigned char	xhi;			// X coordinate Hi
	unsigned char	yhi;			// Y coordinate Hi
	unsigned char	ylo : 4;		// Y coordinate Lo
	unsigned char	xlo : 4;		// X coordinate Lo
#if (CT36X_TS_PTS_VER)
	unsigned char	status : 4;		// Action information, 1: Down; 2: Move; 3: Up
	unsigned char	id : 4;			// ID information, from 1 to CFG_MAX_POINT_NUM
#endif
};
#elif (CT36X_TS_CHIP_SEL == CT36X_CHIP_VER)
struct ct36x_finger_info {
#if (!CT36X_TS_PTS_VER)
	unsigned char	status : 3;		// Action information, 1: Down; 2: Move; 3: Up
	unsigned char	id : 5;			// ID information, from 1 to CFG_MAX_POINT_NUM
#endif
	unsigned char	xhi;			// X coordinate Hi
	unsigned char	yhi;			// Y coordinate Hi
	unsigned char	ylo : 4;		// Y coordinate Lo
	unsigned char	xlo : 4;		// X coordinate Lo
#if (CT36X_TS_PTS_VER)
	unsigned char	status : 3;		// Action information, 1: Down; 2: Move; 3: Up
	unsigned char	id : 5;			// ID information, from 1 to CFG_MAX_POINT_NUM
#endif
	unsigned char	area;			// Touch area
	unsigned char	pressure;		// Touch Pressure
};
#endif

union ct36x_i2c_data {
	struct ct36x_finger_info	pts[CT36X_TS_POINT_NUM];
	unsigned char				buf[CT36X_TS_POINT_NUM * sizeof(struct ct36x_finger_info)];
};

struct ct36x_ts_info {
	// Chip
	int				chip_id;
	
	// Communication settings
	int				spi_bus;
	int				i2c_bus;
	unsigned short			i2c_address;
	struct i2c_client		*client;

	// Devices
	struct input_dev		*input;
	int 				irq;
	int 				rst;
	int 				ss;
	int					ready;
	
	// Early suspend
#ifdef CONFIG_HAS_EARLYSUSPEND
#if	(CT36X_TS_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
#endif

	// Work thread settings
	struct work_struct		event_work;
	struct workqueue_struct *ts_workqueue;


	// ESD 
#if (CT36X_TS_ESD_TIMER_INTERVAL)
	struct timer_list	timer;
	int					timer_on;
#endif

	// touch event data
	union ct36x_i2c_data	data;

	int				press;
	int				release;

	int x_max;
	int y_max;
};

#endif
