/*
 * drivers/input/touchscreen/ct36x_ts.c
 *
 * VTL ct36x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * George Chen, 2012-06-15
 */

// ****************************************************************************
// Includes
// ****************************************************************************

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/timer.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
#include <linux/input/mt.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "ct36x_i2c_ts.h"


/* ==================  ct36x_zx 2012.09.13  ==================*/
#if 0	/* ct36x_zx */
#define ct36x_debug(...) \
	do { \
		printk(KERN_DEBUG __VA_ARGS__); \
	} while (0)
#else
#define ct36x_debug(...)
#endif

#if 0	/* ct36x_zx */
#define ct36x_debug_2(...) \
	do { \
		printk(KERN_DEBUG __VA_ARGS__); \
	} while (0)
#else
#define ct36x_debug_2(...)
#endif

#if 0	/* ct36x_zx */
#define ct36x_debug_pt(...) \
	do { \
		printk(KERN_DEBUG __VA_ARGS__); \
	} while (0)
#else
#define ct36x_debug_pt(...)
#endif


#define CT363_TS_WQ		"ct363_ts_wq"
#define CT36X_TS_WQ		CT363_TS_WQ

#define CT36X_TS_SETUPTIME		(600)		/* ct36x_zx: ct363 tp setup time 450 ms. */

#if defined(CONFIG_CT360_CHIP_UPDATE_SUPPORT)
#define CT36X_FOR_HW_VER_1		(1)
#define CT36X_FOR_HW_VER_2		(2)
#define CT36X_FOR_HW_VER_3		(3)
#define CT36X_FOR_HW_VER_4		(4)
#define CT36X_FOR_HW_VER_5		(5)

#define CT36X_FOR_HW_VER		CT36X_FOR_HW_VER_5
#endif

/* ==================  ct36x_zx 2012.09.13  ==================*/


// ****************************************************************************
// Defines
// ****************************************************************************

//#define DRIVER_NAME		"ct36x_ts"

#define DRIVER_NAME		"ct3610_ts"

// ****************************************************************************
// Globel or static variables
// ****************************************************************************
static struct ct36x_ts_info	ct36x_ts;

#if defined(CONFIG_CT360_CHIP_UPDATE_SUPPORT)
static char const Binary_Data[]=
{
#include CHIP_DRIVER_FILE
#if 0
//#include "CT365Five3020D_V42120523A.dat"
//#include "CT365_V1_9_LR101_V08120712A.dat"
#if (CT36X_FOR_HW_VER == CT36X_FOR_HW_VER_5)
	#include "CT360_ODYS_GENIO.DAT"

#elif (CT36X_FOR_HW_VER == CT36X_FOR_HW_VER_4)
	//#include "CT363_JXX_2030D_2048x1536_V0A121026W.dat"
	#include "CT363_JXX_2030D_2048x1536_V01_130117.dat"
	//#include "CT363LR2030D_2048x1536_V08120915.dat"

#elif (CT36X_FOR_HW_VER == CT36X_FOR_HW_VER_3)
	#include "CT363LR2030D_2048x1536_V06120915.dat"

#else
	#include "CT363LR2030D_2048x1536_V01120910.dat"
#endif
#endif
};
#endif

// ****************************************************************************
// Function declaration
// ****************************************************************************
static void ct36x_ts_reg_read(struct i2c_client *client, unsigned short addr, char *buf, int len, int rate)
{
	struct i2c_msg msgs;

	msgs.addr = addr;
	msgs.flags = 0x01;  // 0x00: write 0x01:read
	msgs.len = len;
	msgs.buf = buf;
	msgs.scl_rate = rate;
	i2c_transfer(client->adapter, &msgs, 1);
}

static void ct36x_ts_reg_write(struct i2c_client *client, unsigned short addr, char *buf, int len, int rate)
{
	struct i2c_msg msgs;

	msgs.addr = addr;
	msgs.flags = 0x00;  // 0x00: write 0x01:read
	msgs.len = len;
	msgs.buf = buf;
	msgs.scl_rate = rate;
	i2c_transfer(client->adapter, &msgs, 1);
}

static int ct36x_ts_get_chip(struct i2c_client *client)
{
	struct ct36x_ts_info *ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	// Write read chip id command
	ts->data.buf[0] = 0xFF;
	ts->data.buf[1] = 0xF0;
	ts->data.buf[2] = 0x00;
	ct36x_ts_reg_write(client, client->addr, (char *) ts->data.buf, 3, CT36X_TS_I2C_SPEED);

	// Reset i2c offsets
	ts->data.buf[0] = 0x00;
	ct36x_ts_reg_write(client, client->addr, (char *) ts->data.buf, 1, CT36X_TS_I2C_SPEED);

	// read chip id
	ct36x_ts_reg_read(client, client->addr, (char *) ts->data.buf, 1, CT36X_TS_I2C_SPEED);

	return ts->data.buf[0];
}

static int ct36x_ts_hw_init(struct ct36x_ts_info *ct36x_ts)
{
	int err = -1;

	// Init Reset
	ct36x_ts->rst = CT36X_TS_RST_PIN;
	err = gpio_request(ct36x_ts->rst, "ct36x_ts_rst");
	if ( err ) {
		return -EIO;
	}
	gpio_direction_output(ct36x_ts->rst, GPIO_HIGH);
	gpio_set_value(ct36x_ts->rst, GPIO_HIGH);

	// Init Int
	ct36x_ts->ss = CT36X_TS_IRQ_PIN;
	err = gpio_request(ct36x_ts->ss, "ct36x_ts_int");
	if ( err ) {
		return -EIO;
	}
	gpio_direction_input(ct36x_ts->ss);

	return 0;
}

static void ct36x_ts_hw_reset(struct ct36x_ts_info *ct36x_ts)
{
	printk("\nct36x_ts_hw_reset test !!!!!!!!!!!!!!!!!!!!!!!\n");
	mdelay(100);
	gpio_set_value(ct36x_ts->rst, GPIO_LOW);
	mdelay(10);
	gpio_set_value(ct36x_ts->rst, GPIO_HIGH);
	mdelay(CT36X_TS_SETUPTIME);
}

static void ct36x_ts_hw_exit(struct ct36x_ts_info *ct36x_ts)
{
	gpio_free(ct36x_ts->rst);
	gpio_free(ct36x_ts->ss);
}

#if defined(CONFIG_CT360_CHIP_UPDATE_SUPPORT)
#if (CT36X_TS_CHIP_SEL == CT360_CHIP_VER)
int ct36x_ts_bootloader(struct i2c_client *client)
{
	int i = 0, j = 0;
	unsigned int ver_chk_cnt = 0;
	unsigned int flash_addr = 0;
	unsigned char CheckSum[16];

	struct ct36x_ts_info *ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	//------------------------------
	// Step1 --> initial BootLoader
	// Note. 0x7F -> 0x00 -> 0xA5 ;
	// MCU goto idle
	//------------------------------
	printk("%s() Set mcu to idle \n", __FUNCTION__);
	ts->data.buf[0] = 0x00;
	ts->data.buf[1] = 0xA5;
	ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 2, CT36X_TS_I2C_SPEED);
	mdelay(10);

	//------------------------------
	// Reset I2C Offset address
	// Note. 0x7F -> 0x00
	//------------------------------
	printk(&"%s() Reset i2c offset address \n", __FUNCTION__);
	ts->data.buf[0] = 0x00;
	ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 1, CT36X_TS_I2C_SPEED);
	mdelay(10);

	//------------------------------
	// Read I2C Bus status
	//------------------------------
	printk("%s() Read i2c bus status \n", __FUNCTION__);
	ct36x_ts_reg_read(client, 0x7F, ts->data.buf, 1, CT36X_TS_I2C_SPEED);
	mdelay(10); 									// Delay 1 ms

	// if return "AAH" then going next step
	if (ts->data.buf[0] != 0xAA)
	{
		printk("%s() i2c bus status: 0x%x \n", __FUNCTION__, ts->data.buf[0]);
		return -1;
	}

	//------------------------------
	// Check incomplete flash erase
	//------------------------------
	printk("%s() Flash erase verify \n", __FUNCTION__);
	ts->data.buf[0] = 0x00;
	ts->data.buf[1] = 0x99;			// Generate check sum command  -->read flash, set addr
	ts->data.buf[2] = 0x00;			// define a flash address for CT36x to generate check sum
	ts->data.buf[3] = 0x00;			//
	ts->data.buf[4] = 0x08;			// Define a data length for CT36x to generate check sum

	// Write Genertate check sum command to CT36x
	ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 5, CT36X_TS_I2C_SPEED);
	mdelay(10); 								// Delay 10 ms

	ct36x_ts_reg_read(client, 0x7F, ts->data.buf, 13, CT36X_TS_I2C_SPEED);
	mdelay(10); 								// Delay 10 ms

	CheckSum[0] = ts->data.buf[5];
	CheckSum[1] = ts->data.buf[6];

	ts->data.buf[0] = 0x00;
	ts->data.buf[1] = 0x99;			// Generate check sum command  -->read flash, set addr
	ts->data.buf[2] = 0x3F;			// define a flash address for CT36x to generate check sum
	ts->data.buf[3] = 0xE0;			//
	ts->data.buf[4] = 0x08;			// Define a data length for CT36x to generate check sum
	// Write Genertate check sum command to CT36x
	ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 5, CT36X_TS_I2C_SPEED);
	mdelay(10); 								// Delay 10 ms

	ct36x_ts_reg_read(client, 0x7F, ts->data.buf, 13, CT36X_TS_I2C_SPEED);
	mdelay(10);

	CheckSum[2] = ts->data.buf[5];
	CheckSum[3] = ts->data.buf[6];

	if ( (CheckSum[0] ^ CheckSum[2]) == 0xFF && (CheckSum[1] ^ CheckSum[3]) == 0xFF )
		goto FLASH_ERASE;

	//------------------------------
	// check valid Vendor ID
	//------------------------------
	printk("%s() Vendor ID Check \n", __FUNCTION__);
	ts->data.buf[0] = 0x00;
	ts->data.buf[1] = 0x99;			// Generate check sum command  -->read flash, set addr
	ts->data.buf[2] = 0x00;			// define a flash address for CT365 to generate check sum
	ts->data.buf[3] = 0x44;			//
	ts->data.buf[4] = 0x08;			// Define a data length for CT365 to generate check sum

	// Write Genertate check sum command to CT36x
	ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 5, CT36X_TS_I2C_SPEED);
	mdelay(10); 								// Delay 10 ms

	ct36x_ts_reg_read(client, 0x7F, ts->data.buf, 13, CT36X_TS_I2C_SPEED);
	mdelay(10); 								// Delay 10 ms

	// Read check sum and flash data from CT36x
	if ( (ts->data.buf[5] != 'V') || (ts->data.buf[9] != 'T') )
		ver_chk_cnt++;

	ts->data.buf[0] = 0x00;
	ts->data.buf[1] = 0x99;			// Generate check sum command  -->read flash,set addr
	ts->data.buf[2] = 0x00;			// define a flash address for CT365 to generate check sum
	ts->data.buf[3] = 0xA4;			//
	ts->data.buf[4] = 0x08;			// Define a data length for CT365 to generate check sum

	// Write Genertate check sum command to CT365
	ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 5, CT36X_TS_I2C_SPEED);
	mdelay(10); 								// Delay 10 ms

	ct36x_ts_reg_read(client, 0x7F, ts->data.buf, 13, CT36X_TS_I2C_SPEED);
	mdelay(10); 								// Delay 10 ms

	if ((ts->data.buf[5] != 'V') || (ts->data.buf[9] != 'T'))
		ver_chk_cnt++;

	if ( ver_chk_cnt >= 2 ) {
		printk("%s() Invalid FW Version \n", __FUNCTION__);
		return -1;
	}

FLASH_ERASE:
	//-----------------------------------------------------
	// Step 2 : Erase 32K flash memory via Mass Erase (33H)
	// 0x7F --> 0x00 --> 0x33 --> 0x00;
	//-----------------------------------------------------
	printk("%s() Erase flash \n", __FUNCTION__);
	for(i = 0; i < 8; i++ ) {
		ts->data.buf[0] = 0x00;			// Offset address
		ts->data.buf[1] = 0x33;			// Mass Erase command
		ts->data.buf[2] = 0x00 + (i * 8);
		ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 3, CT36X_TS_I2C_SPEED);
		mdelay(120); 				// Delay 10 mS

		//------------------------------
		// Reset I2C Offset address
		// Note. 0x7F -> 0x00
		//------------------------------
		ts->data.buf[0] = 0x00;
		ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 1, CT36X_TS_I2C_SPEED);
		mdelay(120); 				// Delay 10 mS

		//------------------------------
		// Read I2C Bus status
		//------------------------------
		ct36x_ts_reg_read(client, 0x7F, ts->data.buf, 1, CT36X_TS_I2C_SPEED);
		mdelay(10); 							// Delay 1 ms

		// if return "AAH" then going next step
		if( ts->data.buf[0] != 0xAA )
			return -1;
	}

	//----------------------------------------
	// Step3. Host write 128 bytes to CT36x
	// Step4. Host read checksum to verify ;
	// Write/Read for 256 times ( 32k Bytes )
	//----------------------------------------
	printk("%s() flash FW \n", __FUNCTION__);
	for ( flash_addr = 0; flash_addr < 0x3FFF; flash_addr+=8 ) {
		// Step 3 : write binary data to CT36x
		ts->data.buf[0] = 0x00;								// Offset address
		ts->data.buf[1] = 0x55;								// Flash write command
		ts->data.buf[2] = (char)(flash_addr  >> 8);			// Flash address [15:8]
		ts->data.buf[3] = (char)(flash_addr & 0xFF);			// Flash address [7:0]
		ts->data.buf[4] = 0x08;								// Data Length

			ts->data.buf[6] = Binary_Data[flash_addr + 0];			// Binary data 1
			ts->data.buf[7] = Binary_Data[flash_addr + 1];			// Binary data 2
			ts->data.buf[8] = Binary_Data[flash_addr + 2];			// Binary data 3
			ts->data.buf[9] = Binary_Data[flash_addr + 3];			// Binary data 4
			ts->data.buf[10] = Binary_Data[flash_addr + 4];			// Binary data 5
			ts->data.buf[11] = Binary_Data[flash_addr + 5];			// Binary data 6
			ts->data.buf[12] = Binary_Data[flash_addr + 6];			// Binary data 7
			ts->data.buf[13] = Binary_Data[flash_addr + 7];			// Binary data 8

		// Calculate a check sum by Host controller.
		// Checksum = / (FLASH_ADRH+FLASH_ADRL+LENGTH+
		// Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
		// Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1
		CheckSum[0] = ~(ts->data.buf[2] + ts->data.buf[3] + ts->data.buf[4] + ts->data.buf[6] + ts->data.buf[7] +
			ts->data.buf[8] + ts->data.buf[9] + ts->data.buf[10] + ts->data.buf[11] + ts->data.buf[12] +
			ts->data.buf[13]) + 1;

		ts->data.buf[5] = CheckSum[0];						// Load check sum to I2C Buffer

		ct36x_ts_reg_write(0x7F, ts->data.buf, 14, CT36X_TS_I2C_SPEED);									// Host write I2C_Buf[0?K12] to CT365.
		mdelay(1);													// 8 Bytes program --> Need 1 ms delay time

		// Step4. Verify process
		printk("%s() Verify FW \n", __FUNCTION__);
		//Step 4 : Force CT365 generate check sum for host to compare data.
		//Prepare get check sum from CT36x
		ts->data.buf[0] = 0x00;
		ts->data.buf[1] = 0x99;								// Generate check sum command
		ts->data.buf[2] = (char)(flash_addr >> 8);			// define a flash address for NT1100x to generate check sum
		ts->data.buf[3] = (char)(flash_addr & 0xFF);		//
		ts->data.buf[4] = 0x08;								// Define a data length for CT36x to generate check sum

		ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 5, CT36X_TS_I2C_SPEED);									// Write Genertate check sum command to CT365
		mdelay(1);													// Delay 1 ms

		ct36x_ts_reg_read(client, 0x7F, ts->data.buf, 13, CT36X_TS_I2C_SPEED);	// Read check sum and flash data from CT365

		// Compare host check sum with CT365 check sum(I2C_Buf[4])
		if ( ts->data.buf[4] != CheckSum[0] ) {
			return -1;
		}
	}

	return	0;
}

#elif (CT36X_TS_CHIP_SEL == CT36X_CHIP_VER)

int ct36x_ts_bootloader(struct i2c_client *client)
{
	int i = 0, j = 0;
	unsigned int ver_chk_cnt = 0;
	unsigned int flash_addr = 0;
	unsigned char CheckSum[16];

	struct ct36x_ts_info *ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	//------------------------------
	// Step1 --> initial BootLoader
	// Note. 0x7F -> 0x00 -> 0xA5 ;
	// MCU goto idle
	//------------------------------
	printk("%s() Set mcu to idle \n", __FUNCTION__);
	ts->data.buf[0] = 0x00;
	ts->data.buf[1] = 0xA5;
	ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 2, CT36X_TS_I2C_SPEED);
	mdelay(10);

	//------------------------------
	// Reset I2C Offset address
	// Note. 0x7F -> 0x00
	//------------------------------
	printk("%s() Reset i2c offset address \n", __FUNCTION__);
	ts->data.buf[0] = 0x00;
	ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 1, CT36X_TS_I2C_SPEED);
	mdelay(10);

	//------------------------------
	// Read I2C Bus status
	//------------------------------
	printk("%s() Read i2c bus status \n", __FUNCTION__);
	ct36x_ts_reg_read(client, 0x7F, ts->data.buf, 1, CT36X_TS_I2C_SPEED);
	mdelay(10);										// Delay 1 ms

	// if return "AAH" then going next step
	if (ts->data.buf[0] != 0xAA)
	{
		printk("%s() i2c bus status: 0x%x \n", __FUNCTION__, ts->data.buf[0]);
		return -1;
	}

  //------------------------------
	printk("%s() Vendor ID check \n", __FUNCTION__);
	ts->data.buf[0] = 0x00;
	ts->data.buf[1] = 0x99;			// Generate check sum command  -->read flash, set addr
	ts->data.buf[2] = 0x00;			// define a flash address for CT365 to generate check sum
	ts->data.buf[3] = 0x44;			//
	ts->data.buf[4] = 0x08;			// Define a data length for CT365 to generate check sum

	// Write Genertate check sum command to CT36x
	ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 5, CT36X_TS_I2C_SPEED);
	mdelay(10);									// Delay 10 ms

	ct36x_ts_reg_read(client, 0x7F, ts->data.buf, 13, CT36X_TS_I2C_SPEED);
	mdelay(10);									// Delay 10 ms

	// Read check sum and flash data from CT36x
	if ( (ts->data.buf[5] != 'V') || (ts->data.buf[9] != 'T') )
		ver_chk_cnt++;

	ts->data.buf[0] = 0x00;
	ts->data.buf[1] = 0x99;			// Generate check sum command  -->read flash,set addr
	ts->data.buf[2] = 0x00;			// define a flash address for CT365 to generate check sum
	ts->data.buf[3] = 0xA4;		    //
	ts->data.buf[4] = 0x08;			// Define a data length for CT365 to generate check sum

	// Write Genertate check sum command to CT365
	ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 5, CT36X_TS_I2C_SPEED);
	mdelay(10);									// Delay 10 ms

	ct36x_ts_reg_read(client, 0x7F, ts->data.buf, 13, CT36X_TS_I2C_SPEED);
	mdelay(10);									// Delay 10 ms

	if ((ts->data.buf[5] != 'V') || (ts->data.buf[9] != 'T'))
		ver_chk_cnt++;

	if ( ver_chk_cnt >= 2 ) {
		printk("%s() Invalid Vendor ID \n", __FUNCTION__);
		return -1;
	}

	//-----------------------------------------------------
	// Step 2 : Erase 32K flash memory via Mass Erase (33H)
	// 0x7F --> 0x00 --> 0x33 --> 0x00;
	//-----------------------------------------------------
	printk("%s() Erase flash \n", __FUNCTION__);
	for(i = 0; i < 100; i++ ) {
		ts->data.buf[0] = 0x00;			// Offset address
		ts->data.buf[1] = 0x33;			// Mass Erase command
		ts->data.buf[2] = 0x00;
		ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 3, CT36X_TS_I2C_SPEED);
		mdelay(10 + i);					// Delay 10 mS

		//------------------------------
		// Reset I2C Offset address
		// Note. 0x7F -> 0x00
		//------------------------------
		ts->data.buf[0] = 0x00;
		ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 1, CT36X_TS_I2C_SPEED);
		mdelay(10 + i);					// Delay 10 mS

		//------------------------------
		// Read I2C Bus status
		//------------------------------
		ct36x_ts_reg_read(client, 0x7F, ts->data.buf, 1, CT36X_TS_I2C_SPEED);
		mdelay(10);								// Delay 1 ms

		// if return "AAH" then going next step
		if( ts->data.buf[0] == 0xAA )
			break;
	}
	if ( i >= 100 )
		return -1;

	//----------------------------------------
	// Step3. Host write 128 bytes to CT36x
	// Step4. Host read checksum to verify ;
	// Write/Read for 256 times ( 32k Bytes )
	//----------------------------------------
	printk("%s() flash FW \n", __FUNCTION__);
	for ( j = 0; j < 256; j++ ) {						// 32k/128 = 256 times
		flash_addr = 128 * j; 							// 0 ~ 127; 128 ~ 255;

		for ( i = 0; i < 16; i++ ) {					// 128/8 = 16 times for One Row program
			// Step 3 : write binary data to CT36x
			ts->data.buf[0] = 0x00;								// Offset address
			ts->data.buf[1] = 0x55;								// Flash write command
			ts->data.buf[2] = (char)(flash_addr  >> 8);			// Flash address [15:8]
			ts->data.buf[3] = (char)(flash_addr & 0xFF);			// Flash address [7:0]
			ts->data.buf[4] = 0x08;		// Data Length
			ts->data.buf[6] = Binary_Data[flash_addr + 0];            // Binary data 1
			ts->data.buf[7] = Binary_Data[flash_addr + 1];            // Binary data 2
			ts->data.buf[8] = Binary_Data[flash_addr + 2];            // Binary data 3
			ts->data.buf[9] = Binary_Data[flash_addr + 3];            // Binary data 4
			ts->data.buf[10] = Binary_Data[flash_addr + 4];           // Binary data 5
			ts->data.buf[11] = Binary_Data[flash_addr + 5];           // Binary data 6
			ts->data.buf[12] = Binary_Data[flash_addr + 6];           // Binary data 7
			ts->data.buf[13] = Binary_Data[flash_addr + 7];           // Binary data 8

			// Calculate a check sum by Host controller.
			// Checksum = / (FLASH_ADRH+FLASH_ADRL+LENGTH+
			// Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
			// Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1
			CheckSum[i] = ~(ts->data.buf[2] + ts->data.buf[3] + ts->data.buf[4] + ts->data.buf[6] + ts->data.buf[7] +
							ts->data.buf[8] + ts->data.buf[9] + ts->data.buf[10] + ts->data.buf[11] + ts->data.buf[12] +
							ts->data.buf[13]) + 1;

			ts->data.buf[5] = CheckSum[i];						// Load check sum to I2C Buffer

			ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 14, CT36X_TS_I2C_SPEED);									// Host write I2C_Buf[0?K12] to CT365.
			mdelay(1);													// 8 Bytes program --> Need 1 ms delay time

			flash_addr += 8;											// Increase Flash Address. 8 bytes for 1 time
		}
		mdelay(20);														// Each Row command --> Need 20 ms delay time

		flash_addr = 128 * j; 											// 0 ~ 127 ; 128 ~ 255 ;

		// Step4. Verify process
		printk("%s() Verify FW \n", __FUNCTION__);
		for ( i = 0; i < 16; i++ ) {									// 128/8 = 16 times for One Row program
			//Step 4 : Force CT365 generate check sum for host to compare data.
			//Prepare get check sum from CT36x
			ts->data.buf[0] = 0x00;
			ts->data.buf[1] = 0x99;								// Generate check sum command
			ts->data.buf[2] = (char)(flash_addr >> 8);			// define a flash address for NT1100x to generate check sum
			ts->data.buf[3] = (char)(flash_addr & 0xFF);		    //
			ts->data.buf[4] = 0x08;								// Define a data length for CT36x to generate check sum

			ct36x_ts_reg_write(client, 0x7F, ts->data.buf, 5, CT36X_TS_I2C_SPEED);									// Write Genertate check sum command to CT365
			mdelay(1);													// Delay 1 ms

			ct36x_ts_reg_read(client, 0x7F, ts->data.buf, 13, CT36X_TS_I2C_SPEED);	// Read check sum and flash data from CT365

			// Compare host check sum with CT365 check sum(I2C_Buf[4])
			if ( ts->data.buf[4] != CheckSum[i] ) {
				return -1;
			}
			flash_addr += 8;													// Increase Flash Address.
		}
	}

	return  0;
}
#endif
#endif /* CONFIG_CT360_CHIP_UPDATE_SUPPORT */

#ifdef CONFIG_HAS_EARLYSUSPEND
#if	(CT36X_TS_HAS_EARLYSUSPEND)
static void ct36x_early_suspend(struct early_suspend *handler);
static void ct36x_early_resume(struct early_suspend *handler);
#endif
#endif

extern int lcd_state ;
extern void rk29_backlight_set(bool on);
void set_input_resolution(int max_x,int max_y,int rotation)
{
#if 0
	struct ct36x_ts_info *ts = &ct36x_ts;
	if(!ts->input)
		return 0;

	ts->x_max = max_x;
	ts->y_max = max_y;
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, ts->x_max, 0, 0);
    	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, ts->y_max, 0, 0);
#endif
}

static irqreturn_t ct36x_ts_irq(int irq, void *dev)
{
	struct ct36x_ts_info *ts;

	ts = (struct ct36x_ts_info *)dev;

	if ( /* CT36X_TS_DEBUG */ 0)
		ct36x_debug(">>>>> %s() called <<<<< \n", __FUNCTION__);

	// touch device is ready??
	if ( ts->ready ) {
		// Disable ts interrupt
		disable_irq_nosync(ts->irq);

	#if (CT36X_TS_ESD_TIMER_INTERVAL)
		if ( ts->timer_on ) {
			// Disable ESD timer
			del_timer(&ts->timer);
			ts->timer_on = 0;
		}
	#endif

		//schedule_work(&ts->event_work);
		queue_work(ts->ts_workqueue, &ts->event_work);	/* ct36x_zx */
	} else {
		printk("---> %s: didn't already!!!!!!\n", __FUNCTION__);
	}

	return IRQ_HANDLED;
}

static void ct36x_ts_timer(unsigned long data)
{
	struct ct36x_ts_info *ts = (struct ct36x_ts_info *) data;

	if ( /*CT36X_TS_DEBUG*/ 0 )
		ct36x_debug(">>>>> %s() called <<<<< \n", __FUNCTION__);

	schedule_work(&ts->event_work);
}


static void ct36x_ts_workfunc(struct work_struct *work)
{
	int iter;
	int sync;
	int x, y;
	struct ct36x_ts_info *ts;

	if ( /*CT36X_TS_DEBUG*/ 0 )
		ct36x_debug_2(">>>>> %s() called <<<<< \n", __FUNCTION__);

	ts = container_of(work, struct ct36x_ts_info, event_work);

	// esd timer check
#if (CT36X_TS_ESD_TIMER_INTERVAL)
	if ( ts->timer_on ) {
		ts->chip_id = ct36x_ts_get_chip(ts->client);

		if ( ts->chip_id != CT360_CHIP_VER && ts->chip_id != CT36X_CHIP_VER ) {
			if ( /*CT36X_TS_DEBUG*/ 0 )
				ct36x_debug("Read Chip ID Error (%x), Chip Reset! \n", ts->chip_id);
			// reset chip
			ct36x_ts_hw_reset(ts);
		}
		// reset esd timer
		mod_timer(&ts->timer, jiffies + HZ * CT36X_TS_ESD_TIMER_INTERVAL);
		return;
	}
#endif

	// read touch points
	ct36x_ts_reg_read(ts->client,
		ts->i2c_address,
		(char *) ts->data.pts,
		sizeof(struct ct36x_finger_info) * CT36X_TS_POINT_NUM,
		CT36X_TS_I2C_SPEED);

	// report points
	sync = 0; ts->press = 0;
	for ( iter = 0; iter < CT36X_TS_POINT_NUM; iter++ ) {
		if ( ts->data.pts[iter].xhi != 0xFF && ts->data.pts[iter].yhi != 0xFF &&
		     (ts->data.pts[iter].status == 1 || ts->data.pts[iter].status == 2) ) {
		#if CT36X_TS_XY_SWAP
			x = (ts->data.pts[iter].yhi<<4)|(ts->data.pts[iter].ylo&0xF);
			y = (ts->data.pts[iter].xhi<<4)|(ts->data.pts[iter].xlo&0xF);
		#else
			x = (ts->data.pts[iter].xhi<<4)|(ts->data.pts[iter].xlo&0xF);
		 	y = (ts->data.pts[iter].yhi<<4)|(ts->data.pts[iter].ylo&0xF);
		#endif
		#if CT36X_TS_X_REVERSE
			x = CT36X_TS_ABS_X_MAX - x;
		#endif
		#if CT36X_TS_Y_REVERSE
			y = CT36X_TS_ABS_Y_MAX - y;
		#endif

			if (CT36X_TS_DEBUG) {
				ct36x_debug_pt("ID:       %d\n", ts->data.pts[iter].id);
				ct36x_debug_pt("status:   %d\n", ts->data.pts[iter].status);
				ct36x_debug_pt("X Lo:     %d\n", ts->data.pts[iter].xlo);
				ct36x_debug_pt("Y Lo:     %d\n", ts->data.pts[iter].ylo);
				ct36x_debug_pt("X Hi:     %d\n", ts->data.pts[iter].xhi);
				ct36x_debug_pt("Y Hi:     %d\n", ts->data.pts[iter].yhi);
				ct36x_debug_pt("X:        %d\n", (ts->data.pts[iter].xhi<<4)|(ts->data.pts[iter].xlo&0xF));
				ct36x_debug_pt("Y:        %d\n", (ts->data.pts[iter].yhi<<4)|(ts->data.pts[iter].ylo&0xF));
			#if (CT36X_TS_CHIP_SEL == CT36X_CHIP_VER)
				ct36x_debug_pt("Area:     %d\n", ts->data.pts[iter].area);
				ct36x_debug_pt("Pressure: %d\n", ts->data.pts[iter].pressure);
			#endif
			}

		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))	// for android 4.0.x
			input_mt_slot(ts->input, ts->data.pts[iter].id - 1);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(ts->input, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
			input_report_abs(ts->input, ABS_MT_PRESSURE,
			#if (CT36X_TS_CHIP_SEL == CT360_CHIP_VER)
				255
			#elif (CT36X_TS_CHIP_SEL == CT36X_CHIP_VER)
				ts->data.pts[iter].pressure
			#endif
			);
		#else
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, ts->data.pts[iter].id - 1);
			input_report_abs(ts->input, ABS_MT_POSITION_X,  x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,  y);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
			#if (CT36X_TS_CHIP_SEL == CT360_CHIP_VER)
				255
			#elif (CT36X_TS_CHIP_SEL == CT36X_CHIP_VER)
				ts->data.pts[iter].pressure
			#endif
			);
			input_mt_sync(ts->input);
		#endif

	#if 0
		if(lcd_state==0)
		{
			lcd_state = 1;
			rk29_backlight_set(1);
			printk("TP huanxing beiguang================\n");
		}
	#endif
			sync = 1;
			ts->press |= 0x01 << (ts->data.pts[iter].id - 1);
		}
	}

	ts->release &= ts->release ^ ts->press;
	for ( iter = 0; iter < CT36X_TS_POINT_NUM; iter++ ) {
		if ( ts->release & (0x01<<iter) ) {
		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))	// for android 4.0.x
			input_mt_slot(ts->input, iter);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
		#else
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, ts->data.pts[iter].id - 1);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input, ABS_MT_POSITION_X,  x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,  y);
			input_mt_sync(ts->input);
		#endif
			sync = 1;
		}
	}
	ts->release = ts->press;

	if ( sync ) input_sync(ts->input);

	// Enable esd timer
#if (CT36X_TS_ESD_TIMER_INTERVAL)
	if ( !ts->timer_on ) {
		ts->timer.expires = jiffies + HZ * CT36X_TS_ESD_TIMER_INTERVAL;
		add_timer(&ts->timer);
		ts->timer_on = 1;
	}
#endif
	// Enable ts interrupt
	enable_irq(ts->irq);
}

static int ct36x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ct36x_ts_info *ts;
	struct device *dev;
	int err = -1;

	if ( CT36X_TS_DEBUG )
		ct36x_debug(">>>>> %s() called <<<<< \n", __FUNCTION__);

	dev = &client->dev;
	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	ts->ready = 0;	// Device is not ready

	// HW Init
	err = ct36x_ts_hw_init(ts);
	if ( err ) {
		dev_err(dev, "Platform HW Init Failed.\n");
		goto ERR_HW_INIT;
	}


	// HW Reset when Firmware update.
	ct36x_ts_hw_reset(ts);

#if (CT36X_TS_CHIP_SEL == CT360_CHIP_VER)
	// Set I2C scon to 0x0f2f --> read version
	ts->data.buf[0] = 0xFF;
	ts->data.buf[1] = 0x0F;
	ts->data.buf[2] = 0x2A;
#elif (CT36X_TS_CHIP_SEL == CT36X_CHIP_VER)
	// Set I2C scon to 0x3fff --> read version
	ts->data.buf[0] = 0xFF;
	ts->data.buf[1] = 0x3F;
	ts->data.buf[2] = 0xFF;
#endif

	ct36x_ts_reg_write(client, client->addr, (char *) ts->data.buf, 3, CT36X_TS_I2C_SPEED);
	mdelay(10);

	ts->data.buf[0] = 0x00;
	ct36x_ts_reg_write(client, client->addr, (char *) ts->data.buf, 1, CT36X_TS_I2C_SPEED);
	mdelay(10);

	// do read version
	ct36x_ts_reg_read(client, client->addr, (char *) ts->data.buf, 1, CT36X_TS_I2C_SPEED);
	mdelay(10);

	dev_info(dev, "FW Version read: 0x%x \n", ts->data.buf[0]);
#if defined(CONFIG_CT360_CHIP_UPDATE_SUPPORT)
	{
		char fw_ver_upd = Binary_Data[CT36X_TS_FW_VER_OFFSET];
		dev_info(dev, "FW Version write: 0x%x \n", fw_ver_upd);

		if ( fw_ver_upd != ts->data.buf[0] )
		{
			dev_info(dev, "Running bootloader ... \n");
			err = ct36x_ts_bootloader(client);

			dev_info(dev, "Bootloader done, %s \n", err ? "Failed" : "OK");
		}
	}
#else
	dev_info(dev, "Bootloader support disabled, trying without...\n");
#endif

	// register early suspend
#ifdef CONFIG_HAS_EARLYSUSPEND
#if	(CT36X_TS_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = ct36x_early_suspend;
	ts->early_suspend.resume = ct36x_early_resume;
	register_early_suspend(&ts->early_suspend);
#endif
#endif

	// Check I2C Functionality
	err = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if ( !err ) {
		dev_err(dev, "Check I2C Functionality Failed.\n");
		goto ERR_I2C_CHK;
	}

	// allocate input device
	ts->input = input_allocate_device();
	if ( !ts->input ) {
		dev_err(dev, "Unable to allocate input device for device %s.\n", DRIVER_NAME);
		err = -ENOMEM;
		goto ERR_INPUT_ALLOC;
	}

	__set_bit(EV_ABS, ts->input->evbit);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
	__set_bit(INPUT_PROP_DIRECT, ts->input->propbit);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
	input_mt_init_slots(ts->input, CT36X_TS_POINT_NUM);
#endif
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, CT36X_TS_ABS_X_MAX, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, CT36X_TS_ABS_Y_MAX, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	ts->input->name =		DRIVER_NAME;
	ts->input->id.bustype =	BUS_I2C;

	// register input device
	err = input_register_device(ts->input);
	if ( err ) {
		dev_err(dev, "Unable to register input device for device %s.\n", DRIVER_NAME);
		goto ERR_INPUT_REGIS;
	}

	/* ct36x_zx */
	ts->ts_workqueue = create_singlethread_workqueue(CT36X_TS_WQ);
	if (!ts->ts_workqueue) {
		err = -ESRCH;
		printk(KERN_ERR "create_workqueue falled!\n");
		goto ERR_TS_WQ;
	}
	// work
	INIT_WORK(&ts->event_work, ct36x_ts_workfunc);

	// Init irq
	ts->irq = gpio_to_irq(CT36X_TS_IRQ_PIN);
	//err = request_irq(ts->irq, ct36x_ts_irq, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, DRIVER_NAME, ts);
	err = request_irq(ts->irq, ct36x_ts_irq, IRQF_TRIGGER_FALLING, DRIVER_NAME, ts);		/* ct36x_zx */
	if ( err ) {
		dev_err(dev, "Unable to request irq.\n");
		goto ERR_IRQ_REQEST;
	}

	// ESD timer
#if (CT36X_TS_ESD_TIMER_INTERVAL)
	setup_timer(&ts->timer, ct36x_ts_timer, ts);
	ts->timer.expires = jiffies + HZ * CT36X_TS_ESD_TIMER_INTERVAL;
	add_timer(&ts->timer);
	ts->timer_on = 1;
	dev_info(dev, "ESD timer, %s \n", ts->timer_on ? "On" : "Off");
#endif

	ts->ready = 1;	// Device is ready

	// HW Reset
	ct36x_ts_hw_reset(ts);

	return 0;

ERR_TS_WQ:
ERR_IRQ_REQEST:
ERR_INPUT_REGIS:
	input_free_device(ts->input);
ERR_INPUT_ALLOC:
ERR_I2C_CHK:
#ifdef CONFIG_HAS_EARLYSUSPEND
#if	(CT36X_TS_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif
#endif
ERR_HW_INIT:
	ct36x_ts_hw_exit(ts);

	return err;
}

static int ct36x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ct36x_ts_info *ts;

	ct36x_debug_2("%s: enter\n", __FUNCTION__);		/* ct36x_zx */

	if (CT36X_TS_DEBUG)
		ct36x_debug("ct36x_ts_suspend\n");
	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);
	disable_irq_nosync(ts->irq);
#if (CT36X_TS_ESD_TIMER_INTERVAL)
	if ( ts->timer_on ) {
		// Disable ESD timer
		del_timer(&ts->timer);
		ts->timer_on = 0;
	}
#endif
	//cancel_work_sync(&ts->event_work);		/* ct36x_zx */

#if (CT36X_TS_CHIP_SEL == CT360_CHIP_VER)
	// step 01 W FF 0F 2B
	ts->data.buf[0] = 0xFF;
	ts->data.buf[1] = 0x0F;
	ts->data.buf[2] = 0x2B;
	ct36x_ts_reg_write(client, client->addr, ts->data.buf, 3, CT36X_TS_I2C_SPEED);
	mdelay(3);

	// step 01 W 00 00
	ts->data.buf[0] = 0x00;
	ts->data.buf[1] = 0x00;
	ct36x_ts_reg_write(client, client->addr, ts->data.buf, 2, CT36X_TS_I2C_SPEED);
	mdelay(3);
#elif (CT36X_TS_CHIP_SEL == CT36X_CHIP_VER)
	// step 01 W FF 8F FF
	ts->data.buf[0] = 0xFF;
	ts->data.buf[1] = 0x8F;
	ts->data.buf[2] = 0xFF;
	ct36x_ts_reg_write(client, client->addr, ts->data.buf, 3, CT36X_TS_I2C_SPEED);
	mdelay(3);

	// step 01 W 00 AF
	ts->data.buf[0] = 0x00;
	ts->data.buf[1] = 0xAF;
	ct36x_ts_reg_write(client, client->addr, ts->data.buf, 2, CT36X_TS_I2C_SPEED);
	mdelay(3);
#endif

  mdelay(600);

	return 0;
}

static int ct36x_ts_resume(struct i2c_client *client)
{
	struct ct36x_ts_info *ts;

	ct36x_debug_2("%s: enter\n", __FUNCTION__);		/* ct36x_zx */

	if (CT36X_TS_DEBUG)
		ct36x_debug("ct36x_ts_resume\n");
	ts = (struct ct36x_ts_info *)i2c_get_clientdata(client);

	ct36x_ts_hw_reset(ts);

#if (CT36X_TS_ESD_TIMER_INTERVAL)
	ts->timer.expires = jiffies + HZ * CT36X_TS_ESD_TIMER_INTERVAL;
	add_timer(&ts->timer);
	ts->timer_on = 1;
#endif
	enable_irq(ts->irq);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
#if	(CT36X_TS_HAS_EARLYSUSPEND)
static void ct36x_early_suspend(struct early_suspend *handler)
{
	//ct36x_ts_suspend(ct36x_ts->client, PMSG_SUSPEND);
	ct36x_debug("%s: enter\n", __FUNCTION__);

	ct36x_ts_suspend(ct36x_ts.client, PMSG_SUSPEND);
}

static void ct36x_early_resume(struct early_suspend *handler)
{
	//ct36x_ts_resume(ct36x_ts->client);

	ct36x_debug("%s: enter\n", __FUNCTION__);

	ct36x_ts_resume(ct36x_ts.client);
}
#endif
#endif


static int __devexit ct36x_ts_remove(struct i2c_client *client)
{
	struct ct36x_ts_info *pdata;

	if (CT36X_TS_DEBUG)
		ct36x_debug("ct36x_ts_remove\n");

	pdata = (struct ct36x_ts_info *)client->dev.platform_data;

	i2c_unregister_device(client);

	return 0;
}

static struct i2c_device_id ct36x_ts_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};

static struct i2c_board_info i2c_board_info[] = {
	{
		I2C_BOARD_INFO(DRIVER_NAME, 0x01),
		.platform_data = NULL,
	},
};

static struct i2c_driver ct36x_ts_driver  = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME
	},
	.id_table	= ct36x_ts_id,
	.probe      = ct36x_ts_probe,
	.remove 	= __devexit_p(ct36x_ts_remove),
};


static int __init ct36x_ts_init(void)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;

	printk("VTL ct36x TouchScreen driver, <george.chen@vtl.com>.\n");

	// Init Platform data
	ct36x_ts.i2c_bus =		CT36X_TS_I2C_BUS;
	ct36x_ts.i2c_address =	CT36X_TS_I2C_ADDRESS;

	adapter = i2c_get_adapter(ct36x_ts.i2c_bus);
	if ( !adapter ) {
		printk("Unable to get i2c adapter on bus %d.\n", ct36x_ts.i2c_bus);
		return -ENODEV;
	}

	client = i2c_new_device(adapter, i2c_board_info);
	i2c_put_adapter(adapter);
	if (!client) {
		printk("Unable to create i2c device on bus %d.\n", ct36x_ts.i2c_bus);
		return -ENODEV;
	}

	ct36x_ts.client = client;
	i2c_set_clientdata(client, &ct36x_ts);

	return i2c_add_driver(&ct36x_ts_driver);
}

static void __exit ct36x_ts_exit(void)
{
	i2c_del_driver(&ct36x_ts_driver);
}

module_init(ct36x_ts_init);
module_exit(ct36x_ts_exit);

MODULE_AUTHOR("<george.chen@vtl.com>");
MODULE_DESCRIPTION("VTL ct36x TouchScreen driver");
MODULE_LICENSE("GPL");
