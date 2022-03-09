/******************************************************************************
 *
 * Copyright (C), 2015, TP-LINK TECHNOLOGIES CO., LTD.
 *
 * Filename: hx8394f_720p_dsi_vdo_fp7720_booyi.c
 *
 * Author: lichenggang
 *
 * Mail : lichenggang@tp-link.com.cn
 *
 * Description:
 *
 * Last modified: 2016-06-22 17:07
 *
******************************************************************************/
#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
//#include <platform/mtk_auxadc_sw.h>
//#include <platform/mtk_auxadc_hw.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "mt-plat/upmu_common.h"
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>

#ifndef CONFIG_FPGA_EARLY_PORTING
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#endif

#endif
#endif

#define CONFIG_HX8394F_DEBUG

#ifdef CONFIG_HX8394F_DEBUG
#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(ALWAYS, "[LK/"LOG_TAG"]"" HX8394F: "string, ##args)
#define LCM_LOGD(string, args...)  dprintf(ALWAYS, "[LK/"LOG_TAG"]"" HX8394F: "string, ##args)
#else
#define LCM_LOGI(fmt, args...)  printk("[KERNEL/"LOG_TAG"]"" HX8394F: "fmt, ##args)
#define LCM_LOGD(fmt, args...)  printk("[KERNEL/"LOG_TAG"]"" HX8394F: "fmt, ##args)
#endif
#else
#ifdef BUILD_LK
#define LCM_LOGI(string, args...)
#define LCM_LOGD(string, args...)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"" HX8394F: "fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"" HX8394F: "fmt, ##args)
#endif
#endif

/* FP7720 */
#ifdef BUILD_LK
#define GPIO_65132_VSN_EN GPIO_LCD_BIAS_ENN_PIN
#define GPIO_65132_VSP_EN GPIO_LCD_BIAS_ENP_PIN
//#define GPIO_LCD_ID       GPIO_LCD_IO_9_PIN
#endif

/* LCDs */
#define R61350_VOL      (0)
#define HX8394F_VOL      (1)
/* [lichenggang] Add DJN_VOL */
//#define DJN_VOL         (2)
#define UNKNOWN_VOL     (999)
#define VOL_THRESHOLD_MIN1 (0)          //1st supply
#define VOL_THRESHOLD_MAX1 (300)
#define VOL_THRESHOLD_MIN2 (700)       //2nd supply
#define VOL_THRESHOLD_MAX2 (1100)
#define VOL_THRESHOLD_MIN3 (1200)      //3nd supply
#define VOL_THRESHOLD_MAX3 (1500)

#define AUX_IN2_NTC  (12) /* to choose different LCDs */
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata);
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define MDELAY(n)           (lcm_util.mdelay(n))
#define UDELAY(n)           (lcm_util.udelay(n))

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
        lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define set_gpio_lcd_enp(cmd) \
        lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_enn(cmd) \
        lcm_util.set_gpio_lcd_enn_bias(cmd)
#define get_gpio_lcd_id() \
        lcm_util.get_gpio_lcd_id()
#define set_gpio_lcd_tp_rst(cmd) \
        lcm_util.set_gpio_lcd_tp_rst(cmd)
#define set_tp_regulator(cmd) \
        lcm_util.set_tp_regulator(cmd)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
extern atomic_t double_enable;

/* [yanlin] Use to recovery lcm when in double-click wakeup mode */
extern atomic_t esd_check_failed;

/*****************************************************************************
 * Define
 *****************************************************************************/

extern int mt_set_gpio_out(unsigned long pin, unsigned long output);
extern int mt_set_gpio_mode(unsigned long pin, unsigned long mode);
extern int mt_set_gpio_dir(unsigned long pin, unsigned long dir);

#define I2C_I2C_LCD_BIAS_CHANNEL 0
#define FP7720_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL    /* for I2C channel 0 */
#define I2C_ID_NAME "fp7720"
#define FP7720_ADDR 0x3E
/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
static struct i2c_client *fp7720_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int fp7720_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int fp7720_remove(struct i2c_client *client);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/
struct fp7720_dev {
    struct i2c_client *client;
};

static int fp7720_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    LCM_LOGI("fp7720_iic_probe\n");
    LCM_LOGI("FP7720: info==>name=%s addr=0x%x\n", client->name, client->addr);
    fp7720_i2c_client = client;
    return 0;
}

static int fp7720_remove(struct i2c_client *client)
{
    LCM_LOGI("fp7720_remove\n");
    fp7720_i2c_client = NULL;
    i2c_unregister_device(client);
    return 0;
}

/* for LCDs, FAQ 12444 */
extern int fp7720_write_bytes(unsigned char addr, unsigned char value);
#if 0
int fp7720_write_bytes(unsigned char addr, unsigned char value)
{
    int ret = 0;
    char write_data[2] = { 0 };
    struct i2c_client *client = fp7720_i2c_client;
    if (fp7720_i2c_client == NULL) {
        printk("client is null %s \n",__func__);
        return -1;
    }

    write_data[0] = addr;
    write_data[1] = value;
    ret = i2c_master_send(client, write_data, 2);
    if (ret < 0)
        LCM_LOGI("fp7720 write data fail !!\n");
    return ret;
}
EXPORT_SYMBOL_GPL(fp7720_write_bytes);
#endif

static const struct i2c_device_id fp7720_id[] = {
    {I2C_ID_NAME, 0},
    {}
};

static const struct of_device_id lcm_of_match[] = {
    { .compatible = "mediatek,i2c_lcd_bias" },
    {},
};

static struct i2c_driver fp7720_iic_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "fp7720",
        .of_match_table = lcm_of_match,
        },
    .id_table = fp7720_id,
    .probe = fp7720_probe,
    .remove = fp7720_remove,
};

static int __init fp7720_iic_init(void)
{
    LCM_LOGI("fp7720_iic_init\n");
    i2c_add_driver(&fp7720_iic_driver);
    LCM_LOGI("fp7720_iic_init success\n");
    return 0;
}

static void __exit fp7720_iic_exit(void)
{
    LCM_LOGI("fp7720_iic_exit\n");
    i2c_del_driver(&fp7720_iic_driver);
}

module_init(fp7720_iic_init);
module_exit(fp7720_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK FP7720 I2C Driver");
MODULE_LICENSE("GPL");
#endif

#define LCM_DSI_CMD_MODE    (0)
#define FRAME_WIDTH         (720)
#define FRAME_HEIGHT        (1280)

#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY      0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW       0xFFFE
#define REGFLAG_RESET_HIGH      0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
    {0x28, 0, {} },
    {0x10, 0, {} },
    {REGFLAG_DELAY, 120, {} },
};

/* [yanlin start] */
static struct LCM_setting_table init_setting_tp910[] = {
	{0xB9,3,{0xFF,0x83,0x94} },
	{0xBA,6,{0x63,0x03,0x68,0x6B,0xB2,0xC0} },
	{0xB1,10,{0x48,0x12,0x72,0x09,0x32,0x44,0x71,0x31,0x4D,0x2F} },
	{0xB2,5,{0x00,0x80,0x64,0x05,0x07} },
	{0xB4,21,{0x26,0x76,0x26,0x76,0x26,0x26,
				0x05,0x10,0x86,0x75,0x00,0x3F,
				0x26,0x76,0x26,0x76,0x26,0x26,
				0x05,0x10,0x86} },
	{0xD3,33,{0x00,0x00,0x06,0x06,0x01,0x01,
				0x10,0x10,0x32,0x10,0x00,0x00,
				0x00,0x32,0x15,0x04,0x05,0x04,
				0x32,0x15,0x14,0x05,0x14,0x37,
				0x33,0x04,0x04,0x37,0x00,0x00,
				0x47,0x05,0x40} },
	{REGFLAG_DELAY, 5, {} },
	{0xD5,44,{0x18,0x18,0x25,0x24,0x27,0x26,
				0x11,0x10,0x15,0x14,0x13,0x12,
				0x17,0x16,0x01,0x00,0x18,0x18,
				0x18,0x18,0x18,0x18,0x18,0x18,
				0x18,0x18,0x05,0x04,0x03,0x02,
				0x07,0x06,0x18,0x18,0x18,0x18,
				0x21,0x20,0x23,0x22,0x18,0x18,
				0x18,0x18} },
	{REGFLAG_DELAY, 5, {} },
	{0xD6,44,{0x18,0x18,0x22,0x23,0x20,0x21,
				0x12,0x13,0x16,0x17,0x10,0x11,
				0x14,0x15,0x06,0x07,0x18,0x18,
				0x18,0x18,0x18,0x18,0x18,0x18,
				0x18,0x18,0x02,0x03,0x04,0x05,
				0x00,0x01,0x18,0x18,0x18,0x18,
				0x26,0x27,0x24,0x25,0x18,0x18,
				0x18,0x18} },
	{REGFLAG_DELAY, 5, {} },
	{0xE0,58,{0x00,0x05,0x0E,0x14,0x17,0x1A,
				0x1D,0x1C,0x36,0x48,0x5F,0x64,
				0x6F,0x88,0x90,0x99,0xA8,0xAE,
				0xAA,0xB6,0xBA,0x5F,0x5F,0x5A,
				0x60,0x64,0x6A,0x72,0x7F,0x00,
				0x05,0x0E,0x14,0x17,0x1A,0x1D,
				0x1C,0x36,0x48,0x5F,0x64,0x6F,
				0x88,0x90,0x99,0xA8,0xAE,0xAA,
				0xB6,0xBA,0x5F,0x5F,0x5A,0x60,
				0x64,0x6A,0x72,0x7F} },
	{REGFLAG_DELAY, 5, {}},
	{0xB6,2,{0x3F,0x3F} },
	/* [yanlin] Changed from 0x0B */
	{0xCC,1,{0x07} },
	{0xC0,2,{0x1F,0x31} },
	{0xD4,1,{0x02} },
	{0xBD,1,{0x01} },
	{0xB1,1,{0x00} },
	{0xBD,1,{0x00} },
	{0xBF,7,{0x40,0x81,0x50,0x00,0x1A,0xFC,0x01} },
	{0xC6,1,{0xEF} },
	{0x11,0,{} },
	{REGFLAG_DELAY, 120, {} },
	{0x29,0,{} },
	{REGFLAG_DELAY, 20, {} },
};
/* [yanlin end] */

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;
		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->physical_width = 68;
	params->physical_height = 121;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
#endif

#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 350;	/* this value must be in MTK suggested table */
#else
	/* [yanlin] Change MIPI clock */
	params->dsi.PLL_CLOCK = 217;	/* this value must be in MTK suggested table, for MT6750 */
#endif
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif

	params->dsi.switch_mode_enable = 0;
	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 4;
	params->dsi.vertical_backporch = 12;
	params->dsi.vertical_frontporch = 15;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 30;
	params->dsi.horizontal_backporch = 60;
	params->dsi.horizontal_frontporch = 60;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	/* [lichenggang start] ESD check: change 0x0A to 0x0D*/
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0D;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x00;
	/* [lichenggang end] */
}

#ifdef BUILD_LK
#define FP7720_SLAVE_ADDR_WRITE  0x7C
#define I2C_I2C_LCD_BIAS_CHANNEL 0
#define I2C_ID_NAME "fp7720"
static struct mt_i2c_t fp7720_i2c;
extern int fp7720_write_byte(kal_uint8 addr, kal_uint8 value);
#endif

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret = 0;
	//config fp7720 output voltage
	cmd = 0x00;
	data = 0x0F;
#ifdef BUILD_LK
	ret = fp7720_write_byte(cmd, data);
#else
	ret = fp7720_write_bytes(cmd, data);
#endif
	MDELAY(2);

	if (ret < 0)
		LCM_LOGI("HX8394F----fp7720----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("HX8394F----fp7720----cmd=%0x--i2c write success----\n", cmd);
	cmd = 0x01;
	data = 0x0F;

#ifdef BUILD_LK
	ret = fp7720_write_byte(cmd, data);
#else
	ret = fp7720_write_bytes(cmd, data);
#endif

	if (ret < 0)
		LCM_LOGI("HX8394F----fp7720----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("HX8394F----fp7720----cmd=%0x--i2c write success----\n", cmd);
	MDELAY(2);

	SET_RESET_PIN(0);
	MDELAY(2);
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO_65132_VSP_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_VSP_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_VSP_EN, GPIO_OUT_ONE);
	MDELAY(2);
	mt_set_gpio_mode(GPIO_65132_VSN_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_VSN_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_VSN_EN, GPIO_OUT_ONE);
#else
	set_gpio_lcd_enp(1);
	MDELAY(2);
	set_gpio_lcd_enn(1);
#endif

	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(2);
	SET_RESET_PIN(0);
	MDELAY(2);
	SET_RESET_PIN(1);
	MDELAY(25);

	push_table(init_setting_tp910, sizeof(init_setting_tp910) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	pr_err("tp910/912 suspend start\n");
	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);

	/* [yanlin] Use to recovery lcm when in double-click wakeup mode */
	if ((atomic_read(&double_enable) == 1) && (atomic_read(&esd_check_failed) == 0)) {
		LCM_LOGI("s61350----double_click enabled, not power off\n");
		MDELAY(2);
		SET_RESET_PIN(1);
		MDELAY(3);
	//do nothing
	} else {
		//set_gpio_lcd_tp_rst(0);
		/* MDELAY(2);
		SET_RESET_PIN(0);
		MDELAY(6); */
#ifdef BUILD_LK
		mt_set_gpio_mode(GPIO_65132_VSN_EN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_65132_VSN_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_65132_VSN_EN, GPIO_OUT_ZERO);
		mt_set_gpio_mode(GPIO_65132_VSP_EN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_65132_VSP_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_65132_VSP_EN, GPIO_OUT_ZERO);
#else
		set_gpio_lcd_enn(0);
		MDELAY(6);
		set_gpio_lcd_enp(0);
#endif
		MDELAY(2);
		SET_RESET_PIN(0);
		MDELAY(6);
	}
	pr_err("tp910/912 suspend end\n");
}

static void lcm_resume(void)
{
	pr_err("tp910/912 resume start\n");
	lcm_init();
	pr_err("tp910/912 resume end\n");
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}

extern int get_lcd_id_by_aux(void);
static unsigned int lcm_compare_id(void)
{
	int ret = 0;
	ret = get_lcd_id_by_aux();
	/* [lichenggang start] lcm compare tianma and djn */
	if (ret == HX8394F_VOL) {
		return 1;  /* ft LCD */
	} else{
		return 0;
	}
	/* [lichenggang end] */
}

/* return TRUE: need recovery */
/* return FALSE: No need recovery */
/* this function is no use, ddp_dsi.c use other way to do esd-check */
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char buffer[3];
	int array[4];
	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x53, buffer, 1);
	if (buffer[0] != 0x24) {
		LCM_LOGI("[LCM ERROR] [0x53]=0x%02x\n", buffer[0]);
		return TRUE;
	}
	LCM_LOGI("[LCM NORMAL] [0x53]=0x%02x\n", buffer[0]);
	return FALSE;
#else
	return FALSE;
#endif
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
	unsigned int ret = 0;
	unsigned int x0 = FRAME_WIDTH / 4;
	unsigned int x1 = FRAME_WIDTH * 3 / 4;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);

	unsigned int data_array[3];
	unsigned char read_buf[4];

	LCM_LOGI("ATA check size = 0x%x,0x%x,0x%x,0x%x\n", x0_MSB, x0_LSB, x1_MSB, x1_LSB);
	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	data_array[0] = 0x00043700;	/* read id return two byte,version and id */
	dsi_set_cmdq(data_array, 1, 1);
	read_reg_v2(0x2A, read_buf, 4);
	if ((read_buf[0] == x0_MSB) && (read_buf[1] == x0_LSB)
		&& (read_buf[2] == x1_MSB) && (read_buf[3] == x1_LSB))
		ret = 1;
	else
		ret = 0;

	x0 = 0;
	x1 = FRAME_WIDTH - 1;

	x0_MSB = ((x0 >> 8) & 0xFF);
	x0_LSB = (x0 & 0xFF);
	x1_MSB = ((x1 >> 8) & 0xFF);
	x1_LSB = (x1 & 0xFF);

	data_array[0] = 0x0005390A;	/* HS packet */
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	return ret;
#else
	return 0;
#endif
}

LCM_DRIVER hx8394f_720p_dsi_vdo_fp7720_booyi_lcm_drv = {
	.name = "hx8394f_720p_dsi_vdo_fp7720_booyi_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
	.esd_check = lcm_esd_check,
	.ata_check = lcm_ata_check,
	.update = lcm_update,
};
