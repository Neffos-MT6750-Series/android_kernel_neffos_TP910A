/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#include <linux/sysfs.h>

/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE 1
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

#define GPIO_FLASH_EN		GPIO8 | 0x80000000
#define GPIO_FLASH_STROBE	GPIO9 | 0x80000000
#define GPIO_TORCH_EN  		GPIO76 | 0x80000000
#define GPIO_AMP_SYNC  		GPIO88 | 0x80000000

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int gDuty = 0;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);




static struct work_struct workTimeOut;

/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */


#define e_DutyNum  26
#define TORCHDUTYNUM 4
int LED1CloseFlag = 1;
int m_duty1 = 0;

int g_LED1_Torch_Mode = 0;
extern int g_LED2_Torch_Mode;
int gIsTorch[e_DutyNum] = { 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // [lihehe] Change array size
//static int isMovieMode[e_DutyNum] = {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#if defined(CONFIG_TPLINK_PRODUCT_TP913)
static int torchLEDReg[e_DutyNum] = {33, 67, 101, 121, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// Torch(mA) = (Brightness Code + 1) X 1.465mA => 50, 100, 150, 179mA
static int flashLEDReg[e_DutyNum] = {3, 8, 12, 14, 16, 20, 25, 29, 33, 37, 42, 46, 50, 55, 59, 63, 67, 72, 76, 80, 84, 93, 101, 110, 118, 127};
// Flash(mA) = (Brightness Code +1 ) X 11.72mA
// 50,100,150,179,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1100,1200,1300,1400,1500mA
#else
static int torchLEDReg[e_DutyNum] = {35, 71, 106, 127, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// Torch(mA) = (Brightness Code X 1.4mA) + 0.977 mA => 50, 100, 150, 179mA
static int flashLEDReg[e_DutyNum] = {3, 8, 12, 14, 16, 20, 25, 29, 33, 37, 42, 46, 50, 55, 59, 63, 67, 72, 76, 80, 84, 93, 101, 110, 118, 127};
// Flash(mA) = (Brightness Code X 11.725mA) + 10.9 mA
// 50,100,150,179,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1100,1200,1300,1400,1500mA
#endif

/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

static struct i2c_client *LM3643_i2c_client;




struct LM3643_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct LM3643_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct LM3643_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

static int LM3643_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);
	char write_data[2] = {0};
	write_data[0] = reg;
	write_data[1] = val;

	mutex_lock(&chip->lock);
	ret = i2c_master_send(client, write_data, 2);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writing at 0x%02x\n", reg);
	return ret;
}

static int LM3643_read_reg(struct i2c_client *client, u8 reg)
{
	u8 val = 0;
	int ret = 0;
	u8 read_reg = reg;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_master_send(client, &read_reg, 1);
	ret = i2c_master_recv(client, &val, 1);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed read at 0x%02x\n", reg);

	return (int)val;
}


#if defined(CONFIG_TPLINK_PRODUCT_TP913)
static int last_power_status = 0;
#endif

static void KTD2687_chip_poweron(void)
{
#if defined(CONFIG_TPLINK_PRODUCT_TP913)
	if (last_power_status == 0) {
		mt_set_gpio_out(GPIO_FLASH_EN,GPIO_OUT_ONE);
		last_power_status = 1;
		udelay(200);
	}
#else
	return;
#endif
}

static void KTD2687_chip_poweroff(void)
{
#if defined(CONFIG_TPLINK_PRODUCT_TP913)
	if (last_power_status == 1) {
		mt_set_gpio_out(GPIO_FLASH_EN,GPIO_OUT_ZERO);
		last_power_status = 0;
	}
#else
	return;
#endif
}


static int LM3643_chip_init(struct LM3643_chip_data *chip)
{

	if(mt_set_gpio_mode(GPIO_FLASH_EN,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!!\n");}
	if(mt_set_gpio_dir(GPIO_FLASH_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!!\n");}
#if defined(CONFIG_TPLINK_PRODUCT_TP913)
	if(mt_set_gpio_out(GPIO_FLASH_EN,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!!\n");}
	last_power_status = 0;
#else
	if(mt_set_gpio_out(GPIO_FLASH_EN,GPIO_OUT_ONE)){PK_DBG("[constant_flashlight] set gpio failed!!\n");}
#endif

	return 0;
}

static unsigned int torch_led_status = 0;
static ssize_t show_torch_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", torch_led_status);
}

static ssize_t set_torch_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int val = 0;

	if (buf != NULL && size != 0) {
		pvalue = (char *)buf;
		ret = kstrtou32(pvalue, 16, (unsigned int *)&val);
		if (val == 1) {
			/* Torch LED1 ON */
			KTD2687_chip_poweron();
			LM3643_write_reg(LM3643_i2c_client, 0x05, 0x7F);
			LM3643_write_reg(LM3643_i2c_client, 0x01, 0x09);
			torch_led_status = 1;
		} else if (val == 2) {
			/* Torch LED2 ON */
			KTD2687_chip_poweron();
			LM3643_write_reg(LM3643_i2c_client, 0x06, 0x7F);
			LM3643_write_reg(LM3643_i2c_client, 0x01, 0x0A);
			torch_led_status = 1;
		} else {
			KTD2687_chip_poweroff();
			LM3643_write_reg(LM3643_i2c_client, 0x01, 0x00);
			torch_led_status = 0;
		}
	}
	return size;
}

static DEVICE_ATTR(torch_status, 0664, show_torch_status, set_torch_status);

// [zhuqiupei start] Add torch level support
static unsigned int torch_led_level_support = 0;

static ssize_t show_torch_level_support(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", torch_led_level_support);
}

static DEVICE_ATTR(torch_level_support, 0444, show_torch_level_support, NULL);
// [zhuqiupei end]

static int LM3643_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct LM3643_chip_data *chip;
	struct LM3643_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("LM3643_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG("LM3643 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3643_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_DBG("LM3643 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3643_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (LM3643_chip_init(chip) < 0)
		goto err_chip_init;

	LM3643_i2c_client = client;
	LM3643_i2c_client->timing = 400;

	if (sysfs_create_file(kernel_kobj, &dev_attr_torch_status.attr)) {
		PK_DBG("sysfs_create_file torch status fail\n");
		goto err_chip_init;
	}

	// [zhuqiupei start] Add torch level support
	if (sysfs_create_file(kernel_kobj, &dev_attr_torch_level_support.attr)) {
		PK_DBG("sysfs_create_file torch level support fail\n");
		goto err_chip_init;
	}
	// [zhuqiupei end]

	PK_DBG("LM3643 Initializing is done\n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_DBG("LM3643 probe is failed\n");
	return -ENODEV;
}

static int LM3643_remove(struct i2c_client *client)
{
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	sysfs_remove_file(kernel_kobj, &dev_attr_torch_status.attr);
	sysfs_remove_file(kernel_kobj, &dev_attr_torch_level_support.attr);  //[zhuqiupei add] Add torch level support

	return 0;
}


#define LM3643_NAME "leds-LM3643"
static const struct i2c_device_id LM3643_id[] = {
	{LM3643_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id LM3643_of_match[] = {
#if defined(CONFIG_TPLINK_PRODUCT_TP910)
	{},
#else
	{.compatible = "mediatek,strobe_main"},
	{},
#endif
};
#endif

static struct i2c_driver LM3643_i2c_driver = {
	.driver = {
		   .name = LM3643_NAME,
#ifdef CONFIG_OF
		   .of_match_table = LM3643_of_match,
#endif
		   },
	.probe = LM3643_probe,
	.remove = LM3643_remove,
	.id_table = LM3643_id,
};
static int __init LM3643_init(void)
{
	PK_DBG("LM3643_init\n");
	return i2c_add_driver(&LM3643_i2c_driver);
}

static void __exit LM3643_exit(void)
{
	i2c_del_driver(&LM3643_i2c_driver);
}


module_init(LM3643_init);
module_exit(LM3643_exit);

MODULE_DESCRIPTION("Flash driver for LM3643");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

int readReg(int reg)
{

	int val;

	val = LM3643_read_reg(LM3643_i2c_client, reg);
	return (int)val;
}

int FL_Enable(void)
{
	int buf[2];

	if (gIsTorch[gDuty] == 1) {
		//buf[0] = 9;
		//buf[1] = val << 4;
		/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
		//LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);

		/* Torch Mode */
		buf[0] = 0x01;
		buf[1] = 0x0B;
		/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
		LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
	} else {
		//buf[0] = 9;
		//buf[1] = val;
		/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
		//LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);

		/* Strobe Mode */
		buf[0] = 0x01;
		buf[1] = 0x0F;
		/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
		LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
	}

	PK_DBG(" FL_Enable line=%d\n", __LINE__);

	//readReg(0);
	readReg(1);
	readReg(6);
	readReg(8);
	readReg(9);
	readReg(0xa);
	readReg(0xb);

	return 0;
}



int FL_Disable(void)
{
	int buf[2];

	buf[0] = 0x01;
	buf[1] = 0x00;

	/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
	LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
	torch_led_status = 0;
	PK_DBG(" FL_Disable line=%d\n", __LINE__);
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
    /* [lihehe start] Change max duty to 22 */
	if (duty > 22)
		duty = 22;
    /* [lihehe end] Change max duty to 22 */
	if (duty < 0)
		duty = 0;

	/* setDuty_sky81296_1(duty); */
	if (gIsTorch[duty] == 1)
	{
		g_LED1_Torch_Mode = 1;
	}
	else
	{
		g_LED1_Torch_Mode = 0;
	}


	m_duty1 = duty;
	PK_DBG(" FL_dim_duty duty=%d\n", duty);
	return 0;
}




int FL_Init(void)
{
	int buf[2];

#if 0
        if(mt_set_gpio_mode(GPIO_FLASH_EN,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!!\n");}
	if(mt_set_gpio_dir(GPIO_FLASH_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!!\n");}
	if(mt_set_gpio_out(GPIO_FLASH_EN,GPIO_OUT_ONE)){PK_DBG("[constant_flashlight] set gpio failed!!\n");}
#endif

	KTD2687_chip_poweron();

	buf[0] = 0x01;
	buf[1] = 0x00;
	/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
	LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);

	buf[0] = 0x08;
	buf[1] = 0x1F;
	/* iWriteRegI2C(buf , 2, STROBE_DEVICE_ID); */
	LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);

/*	PK_DBG(" FL_Init line=%d\n", __LINE__); */
	return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
	KTD2687_chip_poweroff();
	return 0;
}

int flashEnable_LM3643(void)
{
	int buf[2];
	PK_DBG("flashEnable_LM3643\n");
	PK_DBG("LED1CloseFlag = %d, LED2CloseFlag = %d \n", LED1CloseFlag, LED2CloseFlag);

	if((LED1CloseFlag == 1) && (LED2CloseFlag == 1)) // Both LED1 & LED2 are OFF
	{
		FL_Disable();
		return 0;
	}
	else if (LED1CloseFlag == 1)  //only LED2 is ON
	{
		buf[0] = 0x01;

		if(g_LED2_Torch_Mode)  // LED2 torch mode
		{
			buf[1] = 0x0A;
		}
		else  // LED2 Flash Mode
		{
			buf[1] = 0x0E;
		}

		LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
	}
	else if (LED2CloseFlag == 1) // only LED1 is ON
	{
		buf[0] = 0x01;

		if(g_LED1_Torch_Mode)  // LED1 torch mode
		{
	        buf[1] = 0x09;
		}
		else   // LED1 Flash Mode
		{
	        buf[1] = 0x0D;
		}

		LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
	}
	else  // Both LED1 & LED2 are ON
	{

		buf[0] = 0x01;

		/*
                 Add by Zhangliangliang @ 2016-08-10
                 We should avoid the case that LED1 is torch, while LED2 is Flash.
                 This may cause wrong calculation for luminance and white balance.

                 Currently, we will specify that LED1 is on and LED2 is off for torch mode.
                 We may also specify that LED1 is on and LED2 is off for Preflash.

                 For more detail, please refer to flash_tuning_custom_cct.cpp in HAL
		*/
		if(g_LED1_Torch_Mode && g_LED2_Torch_Mode)  // LED1 & LED2 torch mode
		{
	        buf[1] = 0x0B;
		}
		else   // LED1 & LED2 Flash Mode
		{
	        buf[1] = 0x0F;
		}

		LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
	}

	//readReg(0);
	readReg(1);
	readReg(6);
	readReg(8);
	readReg(9);
	readReg(0xa);
	readReg(0xb);

	return 0;
}

int flashDisable_LM3643(void)
{
	LM3643_write_reg(LM3643_i2c_client, 0x01, 0x00);
	torch_led_status = 0;
	return 0;
}

int setDuty_LM3643(void)
{
	int buf[2];
	PK_DBG("setDuty_LM3643 m_duty1 = %d, m_duty2 = %d\n", m_duty1, m_duty2);
	PK_DBG("LED1CloseFlag = %d, LED2CloseFlag = %d \n", LED1CloseFlag, LED2CloseFlag);

	if((LED1CloseFlag == 1) && (LED2CloseFlag == 1))
	{
		return 0;
	}
	else if (LED1CloseFlag == 1)	/* Set LED2 */
	{
		if(g_LED2_Torch_Mode)
		{  // LED2 torch mode
			buf[0] = 0x06;
			buf[1] = torchLEDReg[m_duty2];
		}
		else
		{
			buf[0] = 0x04;
			buf[1] = flashLEDReg[m_duty2];
		}

		LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
	}
	else if (LED2CloseFlag == 1)	/* Set LED1 */
	{
		if(g_LED1_Torch_Mode)
		{  // LED2 torch mode
			buf[0] = 0x05;
			buf[1] = torchLEDReg[m_duty1];
		}
		else
		{
			buf[0] = 0x03;
			buf[1] = flashLEDReg[m_duty1];
		}

		LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
	}
	else
	{
		if(g_LED1_Torch_Mode && g_LED2_Torch_Mode)  // LED1 & LED2 torch mode
		{
			buf[0] = 0x05;
			buf[1] = torchLEDReg[m_duty1];
			LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);

			buf[0] = 0x06;
			buf[1] = torchLEDReg[m_duty2];
			LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
		}
		else   // LED1 & LED2 Flash Mode
		{
			buf[0] = 0x03;
			buf[1] = flashLEDReg[m_duty1];
			LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);

			buf[0] = 0x04;
			buf[1] = flashLEDReg[m_duty2];
			LM3643_write_reg(LM3643_i2c_client, buf[0], buf[1]);
		}
	}

	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}

enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	static int init_flag;

	if (init_flag == 0) {
		init_flag = 1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("LM3643 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
	switch (cmd) {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
		if (arg == 1) {
			if (g_timeOutTimeMs != 0) {
				ktime_t ktime;

				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			LED1CloseFlag = 0;
			//FL_Enable();
		} else {
			LED1CloseFlag = 1;
			//FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
