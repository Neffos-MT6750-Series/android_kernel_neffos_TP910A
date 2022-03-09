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
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>

#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#include <mt-plat/mt_pwm.h>
#include <linux/delay.h>

#include <linux/i2c.h>

#include "kd_flashlight.h"
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
#define TAG_NAME "[strobe_main_sid1_part2.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif

static int FL_Disable(void);

#define e_DutyNum  26

static int gIsTorch[e_DutyNum] = { 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int torchLEDReg[e_DutyNum] = {23, 47, 71, 95, 119, 142, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// 50, 100, 150, 200, 250, 300mA
static int flashLEDReg[e_DutyNum] = {4, 9, 14, 19, 24, 29, 34, 39, 44, 49, 54, 59, 64,69,74,79,84,89,94,99, 104, 109,119,129,139,149};
// 50,100,150,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000 ,1050, 1100,1200,1300,1400,1500mA

static int g_timeOutTimeMs;
int g_LED_Torch_Mode = 0;
int m_duty = 0;
static struct hrtimer g_timeOutTimer;
static struct work_struct workTimeOut;

static unsigned int torch_led_status = 0;

static unsigned int torch_led_level_support = 1;  // [zhuqiupei add] Add torch level support

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}

enum hrtimer_restart ledTimeOutCallback2(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static void timerInit(void)
{
	static int init_flag;

	if (init_flag == 0) {
		init_flag = 1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback2;
	}
}


static int FL_dim_duty(kal_uint32 duty)
{
    /* [lihehe start] Change max flash duty to 23 */
	if (duty > 23)
		duty = 23;
    /* [lihehe end] Change max flash duty to 23 */
	if (duty < 0)
		duty = 0;

	if (gIsTorch[duty] == 1)
	{
		g_LED_Torch_Mode = 1;
	}
	else
	{
		g_LED_Torch_Mode = 0;
	}

	m_duty = duty;
	PK_DBG(" FL_dim_duty duty=%d\n", duty);
	return 0;
}

#define  GPIO_FLASH_ENABLE (GPIO8 | 0x80000000)
#define  GPIO_FLASH_STROBE (GPIO9 | 0x80000000)
#define PWM_NO PWM2

static int Set_PWM(int thresh)
{
	struct pwm_spec_config pwm_setting = { .pwm_no = PWM_NO};

	PK_DBG("PWM_OldMode:\n");

	if (mt_set_gpio_mode(GPIO_FLASH_STROBE,GPIO_MODE_01)) {//pwm mode
		PK_DBG("set pwm mode failed!!\n");
		return -1;
	}

	pwm_setting.pwm_no = PWM_NO;
	pwm_setting.mode = PWM_MODE_OLD;
	pwm_setting.pmic_pad = false;

	pwm_setting.clk_div = CLK_DIV4;// div 4
	pwm_setting.clk_src = PWM_CLK_OLD_MODE_BLOCK;//26MHz

	pwm_setting.PWM_MODE_OLD_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_OLD_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_OLD_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_OLD_REGS.WAVE_NUM = 0;
	pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 142;	/* 143 level, because max current 1426mA */

	pwm_setting.PWM_MODE_OLD_REGS.THRESH = thresh;

	pwm_set_spec_config(&pwm_setting);

	return 0;
}

static int FL_TorchEnable(int pwm_duty)
{
		PK_DBG(" FL_Enable torch mode\n");

		/* Open torch led */
		if (mt_set_gpio_out(GPIO_FLASH_ENABLE, 0)) {
			return -1;
		}

		/* keep 5ms output high*/
		if (mt_set_gpio_mode(GPIO_FLASH_STROBE,GPIO_MODE_00)) {
			return -1;
		}
		if (mt_set_gpio_dir(GPIO_FLASH_STROBE, GPIO_DIR_OUT)) {
			return -1;
		}
		if (mt_set_gpio_out(GPIO_FLASH_STROBE, 1)) {
			return -1;
		}

		udelay(6000);// For torch mode stable, set to 6ms

		Set_PWM(pwm_duty);

		return 0;
}

static int FL_Enable(void)
{
	PK_DBG(" FL_Enable \n");
	if(g_LED_Torch_Mode)
	{
		return FL_TorchEnable(torchLEDReg[m_duty]);
	}
	else
	{
		PK_DBG(" FL_Enable flash mode\n");

		/* Open flash led */
		if (mt_set_gpio_out(GPIO_FLASH_ENABLE, 1)) {
			return -1;
		}

		Set_PWM(flashLEDReg[m_duty]);
	}

	return 0;
}

static int FL_Disable(void)
{
	/* Close Flash light */
	PK_DBG(" FL_Disable \n");

	if (mt_set_gpio_out(GPIO_FLASH_ENABLE, 0)) {
		return -1;
	}

	if (mt_set_gpio_mode(GPIO_FLASH_STROBE,GPIO_MODE_00)) {
		return -1;
	}
	if (mt_set_gpio_dir(GPIO_FLASH_STROBE, GPIO_DIR_OUT)) {
		return -1;
	}
	if (mt_set_gpio_out(GPIO_FLASH_STROBE, 0)) {
		return -1;
	}

	torch_led_status = 0;

	return 0;
}


static int strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;

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
			FL_Enable();
		} else {
			FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;
	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
	return 0;
}

static int strobe_open(void *pArg)
{
	PK_DBG("strobe_open\n");
	timerInit();

	if (mt_set_gpio_mode(GPIO_FLASH_ENABLE,GPIO_MODE_00)) {
		return -1;
	}

	if (mt_set_gpio_dir(GPIO_FLASH_ENABLE, GPIO_DIR_OUT)) {
		return -1;
	}

	return 0;
}

static int strobe_release(void *pArg)
{
	PK_DBG("strobe_release\n");
	return 0;
}

static FLASHLIGHT_FUNCTION_STRUCT strobeFunc = {
	strobe_open,
	strobe_release,
	strobe_ioctl
};

MUINT32 strobeInit_main_sid1_part2(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &strobeFunc;
	return 0;
}

// [zhuqiupei start] Add torch level support
static ssize_t show_torch_level_support(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", torch_led_level_support);
}

static DEVICE_ATTR(torch_level_support, 0444, show_torch_level_support, NULL);
// [zhuqiupei end]

static ssize_t show_torch_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", torch_led_status);
}

static ssize_t store_torch_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL;
	unsigned int val = 0;

	if (buf != NULL && size != 0) {
		pvalue = (char *)buf;
		ret = kstrtou32(pvalue, 16, (unsigned int *)&val);
		if (val) {
			/* Torch LED ON */
			switch (val) {
			case 1:
				FL_TorchEnable(torchLEDReg[2]);// 150mA
				break;
			case 3:
				FL_TorchEnable(35);// 75mA
				break;
			case 4:
				FL_TorchEnable(torchLEDReg[2]);// 150mA
				break;
			case 5:
				FL_TorchEnable(torchLEDReg[3]);// 200mA
				break;
			default:
				FL_TorchEnable(torchLEDReg[2]);// 150mA
				break;
			}
			torch_led_status = val;
		}else {
			FL_Disable();
			torch_led_status = 0;
		}
	}
	return size;
}

static DEVICE_ATTR(torch_status, 0664, show_torch_status, store_torch_status);

static int SGM3785_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	PK_DBG("SGM3785_probe start--->.\n");

	if (mt_set_gpio_mode(GPIO_FLASH_ENABLE,GPIO_MODE_00)) {
		return -1;
	}

	if (mt_set_gpio_dir(GPIO_FLASH_ENABLE, GPIO_DIR_OUT)) {
		return -1;
	}

	if (sysfs_create_file(kernel_kobj, &dev_attr_torch_status.attr)) {
		PK_DBG("sysfs_create_file torch status fail\n");
		goto err_chip_init;
	}

	// [zhuqiupei start] Add torch level support
	if (sysfs_create_file(kernel_kobj, &dev_attr_torch_level_support.attr)) {
		PK_DBG("sysfs_create_file torch support fail\n");
		goto err_chip_init;
	}
	// [zhuqiupei end]

	torch_led_status = 0;

	PK_DBG("SGM3785 Initializing is done\n");

	return 0;

err_chip_init:
	PK_DBG("SGM3785 probe is failed\n");
	return -ENODEV;
}

static int SGM3785_remove(struct i2c_client *client)
{
	PK_DBG("SGM3785_remove\n");
	sysfs_remove_file(kernel_kobj, &dev_attr_torch_status.attr);
	sysfs_remove_file(kernel_kobj, &dev_attr_torch_level_support.attr); // [zhuqiupei add] Add torch level support
	return 0;
}


#define SGM3785_NAME "leds-SGM3785"
static const struct i2c_device_id SGM3785_id[] = {
	{SGM3785_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id SGM3785_of_match[] = {
#if defined(CONFIG_TPLINK_PRODUCT_TP910)
	{.compatible = "mediatek,strobe_main"},
	{},
#else
	{},
#endif
};
#endif

static struct i2c_driver SGM3785_i2c_driver = {
	.driver = {
		   .name = SGM3785_NAME,
#ifdef CONFIG_OF
		   .of_match_table = SGM3785_of_match,
#endif
		   },
	.probe = SGM3785_probe,
	.remove = SGM3785_remove,
	.id_table = SGM3785_id,
};
static int __init SGM3785_init(void)
{
	PK_DBG("SGM3785_init\n");
	return i2c_add_driver(&SGM3785_i2c_driver);
}

static void __exit SGM3785_exit(void)
{
	i2c_del_driver(&SGM3785_i2c_driver);
}


module_init(SGM3785_init);
module_exit(SGM3785_exit);

MODULE_DESCRIPTION("Flash driver for SGM3785");
MODULE_AUTHOR("linyimin@tp-link.com.cn");
MODULE_LICENSE("GPL");
