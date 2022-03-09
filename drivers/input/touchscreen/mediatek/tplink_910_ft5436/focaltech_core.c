/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, FocalTech Systems, Ltd., all rights reserved.
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
 */
/*****************************************************************************
*
* File Name: focaltech_core.c

* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/rtpm_prio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/wakelock.h>
#include "focaltech_core.h"
#include "tpd.h"
/* [liliwen start] Add for TP INT */
#include <mt_gpio.h>
#include <mach/gpio_const.h>
#include <upmu_common.h>

#define GPIO_CTP_RST_PIN         (GPIO10 | 0x80000000)
/* [liliwen end] */

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#define INTERVAL_READ_REG                   20  //interval time per read reg unit:ms
#define TIMEOUT_READ_REG                    300 //timeout of read reg unit:ms
#define FTS_I2C_SLAVE_ADDR                  0x38
#define FTS_READ_TOUCH_BUFFER_DIVIDED       0

/* [liliwen start] Add for button reverse */
#define FTS_BUTTON_LEFT_X                   180
#define FTS_BUTTON_CENTER_X                 360
#define FTS_BUTTON_RIGHT_X                  540
/* [liliwen end] */

/*****************************************************************************
* Static variables
*****************************************************************************/
struct i2c_client *fts_i2c_client;
struct input_dev *fts_input_dev;
struct task_struct *thread_tpd;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag;

#if FTS_DEBUG_EN
int g_show_log = 1;
#else
int g_show_log = 0;
#endif

unsigned int tpd_rst_gpio_number = 0;
unsigned int tpd_int_gpio_number = 1;
static unsigned int ft_touch_irq = 0;
static unsigned int ft_irq_disable = 0;
/* [liliwen] Add for usb check */
static int TP910_FT5436_PROBE_OK = 0;
static spinlock_t irq_lock;
struct mutex report_mutex;

/* [liliwen start] Add for ex mode */
static int key_back_button = 0;
static int key_recent_button = 0;
static int key_home_button = 0;
static int tp_suspend = 0;
extern int button_reverse;
int usb_check_state = 0;
DEFINE_MUTEX(ft_suspend_lock);
static struct proc_dir_entry *prEntry_tp = NULL;
static struct proc_dir_entry *prEntry_dtap = NULL;
int TP_IS_FTS = 0;
atomic_t double_enable;
static atomic_t close_incall;
/* [liliwen end] */

/* [liliwen] Add for glove_mode */
#if FTS_GLOVE_EN
extern atomic_t fts_glove_mode;
#endif

atomic_t interrupt_counter = ATOMIC_INIT(0); /* [jiangtingyu] add tp report rate node */

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
static int tpd_def_calmat_local_normal[8] = TPD_CALIBRATION_MATRIX_ROTATION_NORMAL;
static int tpd_def_calmat_local_factory[8] = TPD_CALIBRATION_MATRIX_ROTATION_FACTORY;
#endif

/* [liliwen] Add for usb check */
extern int fts_enter_charger_mode(struct i2c_client *client, int mode);

/* [liliwen] Add for glove mode */
#if FTS_GLOVE_EN
extern int fts_enter_glove_mode(struct i2c_client *client, int mode);
#endif

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static void tpd_resume(struct device *h);
static void tpd_suspend(struct device *h);
static void fts_release_all_finger(void);

/*****************************************************************************
* Focaltech ts i2c driver configuration
*****************************************************************************/
static const struct i2c_device_id fts_tpd_id[] = { {FTS_DRIVER_NAME, 0}, {} };

static const struct of_device_id fts_dt_match[] = {
    {.compatible = "mediatek,cap_touch"},
    {},
};

MODULE_DEVICE_TABLE(of, fts_dt_match);

static struct i2c_driver tpd_i2c_driver = {
    .driver = {
               .name = FTS_DRIVER_NAME,
               .of_match_table = of_match_ptr(fts_dt_match),
               },
    .probe = tpd_probe,
    .remove = tpd_remove,
    .id_table = fts_tpd_id,
    .detect = tpd_i2c_detect,
};

/* [liliwen start] Add for sysfs proc */
// sysfs proc start
#if FTS_GESTURE_EN
static ssize_t tp_double_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int ret = 0;
    int state = 0;
    char buf[10] = {0};

    if (count > 10)
        return count;
    if (copy_from_user(buf,buffer,count)) {
        printk("%s,read proc input error.\n",__func__);
        return count;
    }
    sscanf(buf,"%d",&ret);
    printk("tp_double_read_func:buf=%d,ret=%d\n",*buf,ret);
    mutex_lock(&ft_suspend_lock);
    if ((ret == 0) || (ret == 1)) {
        atomic_set(&double_enable, ret);
    }
    if (tp_suspend == 1) {
        if (ret == 1) {
            printk(".....gesture will enable.....\n");
            fts_i2c_read_reg(fts_i2c_client,0xa5,(u8 *)&state);
            fts_i2c_write_reg(fts_i2c_client,0xa5,0x00);
            fts_i2c_write_reg(fts_i2c_client,0xd0,0x01);
            fts_i2c_write_reg(fts_i2c_client,0xd1,0x3f);
            fts_i2c_write_reg(fts_i2c_client,0xd2,0x07);
            fts_i2c_write_reg(fts_i2c_client,0xd6,0xff);
            enable_irq(ft_touch_irq);
        }
        if (ret == 0) {
            printk(".....gesture will disable.....\n");
            fts_i2c_read_reg(fts_i2c_client,0xa5,(u8 *)&state);
            msleep(50);
            fts_i2c_write_reg(fts_i2c_client,0xa5,0x02);
        }
    }
    mutex_unlock(&ft_suspend_lock);
    return count;
}

static ssize_t tp_double_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int len = 0;
    int ret = 0;
    char page[512];
    printk("double_tap enable is:%d\n",atomic_read(&double_enable));
    len = sprintf(page,"%d\n",atomic_read(&double_enable));
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

static ssize_t support_gesture_id_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int ret = 0;
    char page[512];
    /* [liliwen] Add double tap support id */
    ret = sprintf(page, "0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",0x01, 0x02,
                    0x04, 0x05, 0x06, 0x0C, 0x0D);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}
#endif

static ssize_t close_incall_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
 {
    int ret = 0;
    char buf[10] = {0};

    if (count > 10)
        return count;
    if (copy_from_user(buf, buffer, count)) {
        printk("%s:read from proc error!\n", __func__);
        return count;
    }
    sscanf(buf, "%d\n", &ret);
    printk("%s:buf=%d,ret=%d\n", __func__, *buf, ret);
    mutex_lock(&ft_suspend_lock);
    if ((ret == 1) || (ret == 0)) {
        atomic_set(&close_incall, ret);
    }
    mutex_unlock(&ft_suspend_lock);
    return count;
}

static ssize_t tp_report_rate_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int len = 0;
    int ret = 0;
    char page[512];
    int count1, count2, rate;
    count1 = atomic_read(&interrupt_counter);
    mdelay(1000);
    count2 = atomic_read(&interrupt_counter);
    rate = (count2-count1) * 1;
    atomic_set(&interrupt_counter, 0);

    printk("tplink touchpanel report rate is:%d\n",rate);
    len = sprintf(page,"%d\n", rate);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}


#if FTS_GESTURE_EN
static const struct file_operations gesture_coordinate_fops = {
    .read = gesture_coordinate_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};

static const struct file_operations double_tap_enable_fops = {

    .write = tp_double_write,
    .read = tp_double_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};

static const struct file_operations support_gesture_id_fops = {
    .read = support_gesture_id_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};
#endif

#if FTS_GLOVE_EN
static const struct file_operations fts_glove_enable_fops = {
    .write = fts_glove_enable_write,
    .read = fts_glove_enable_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};
#endif

#if TPLINK_CHANGE_TP_THRESHOLD_ENABLE
static const struct file_operations tp_thresh_value_fops = {
    .write = tplink_thresh_value_write,
    .read = tplink_thresh_value_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};
#endif

#if TPLINK_READ_DIFFDATA_ENABLE
static const struct file_operations diffdata_read_fops = {
    .read = tplink_diffdata_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};
#endif

static const struct file_operations fts_button_reverse_fops = {
    .write = fts_button_reverse_write,
    .read = fts_button_reverse_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};

static const struct file_operations firmware_update_fops = {
    .write = firmware_update_write,
    .read = firmware_update_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};

static const struct file_operations close_incall_fops = {
    .write = close_incall_write,
    .open = simple_open,
    .owner = THIS_MODULE,
};

static const struct file_operations report_rate_fops = {
    .read = tp_report_rate_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};
//sysfs proc end
/* [liliwen end] */

/*****************************************************************************
*  Name: fts_wait_tp_to_valid
*  Brief:   Read chip id until TP FW become valid, need call when reset/power on/resume...
*           1. Read Chip ID per INTERVAL_READ_REG(20ms)
*           2. Timeout: TIMEOUT_READ_REG(400ms)
*  Input:
*  Output:
*  Return: 0 - Get correct Device ID
*****************************************************************************/
int fts_wait_tp_to_valid(struct i2c_client *client)
{
    int ret = 0;
    int cnt = 0;
    u8 reg_value = 0;

    do {
        ret = fts_i2c_read_reg(client, FTS_REG_CHIP_ID, &reg_value);
        if ((ret < 0) || (reg_value != chip_types.chip_idh)) {
            FTS_INFO("TP Not Ready, ReadData = 0x%x", reg_value);
        }
        else if (reg_value == chip_types.chip_idh) {
            FTS_INFO("TP Ready, Device ID = 0x%x", reg_value);
            return 0;
        }
        cnt++;
        msleep(INTERVAL_READ_REG);
    }
    while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

    /* error: not get correct reg data */
    return -1;
}

/*****************************************************************************
*  Name: fts_recover_state
*  Brief: Need execute this function when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_tp_state_recovery(struct i2c_client *client)
{
    /* wait tp stable */
    fts_wait_tp_to_valid(client);
    /* recover TP charger state 0x8B */
    /* recover TP glove state 0xC0 */
    /* recover TP cover state 0xC1 */
    /* [liliwen] remove fts_ex_mode_recovery */
    //fts_ex_mode_recovery(client);
}

/*****************************************************************************
*  Name: fts_reset_proc
*  Brief: Execute reset operation
*  Input: hdelayms - delay time unit:ms
*  Output:
*  Return: 0 - Get correct Device ID
*****************************************************************************/
int fts_reset_proc(int hdelayms)
{
    tpd_gpio_output(tpd_rst_gpio_number, 0);
    msleep(20);
    tpd_gpio_output(tpd_rst_gpio_number, 1);
    msleep(hdelayms);

    return 0;
}

/*****************************************************************************
*  Name: fts_irq_disable
*  Brief: disable irq
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_irq_disable(void)
{
    unsigned long irqflags;

    spin_lock_irqsave(&irq_lock, irqflags);

    if (!ft_irq_disable) {
        disable_irq_nosync(ft_touch_irq);
        ft_irq_disable = 1;
    }

    spin_unlock_irqrestore(&irq_lock, irqflags);
}

/*****************************************************************************
*  Name: fts_irq_enable
*  Brief: enable irq
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_irq_enable(void)
{
    unsigned long irqflags = 0;

    spin_lock_irqsave(&irq_lock, irqflags);

    if (ft_irq_disable) {
        enable_irq(ft_touch_irq);
        ft_irq_disable = 0;
    }

    spin_unlock_irqrestore(&irq_lock, irqflags);
}

#if FTS_POWER_SOURCE_CUST_EN
/*****************************************************************************
* Power Control
*****************************************************************************/
int fts_power_init(void)
{
    int retval;
    /*set TP volt */
    tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
    retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
    if (retval != 0) {
        FTS_ERROR("[POWER]Failed to set voltage of regulator,ret=%d!", retval);
        return retval;
    }

    retval = regulator_enable(tpd->reg);
    if (retval != 0) {
        FTS_ERROR("[POWER]Fail to enable regulator when init,ret=%d!", retval);
        return retval;
    }

    return 0;
}

void fts_power_suspend(void)
{
    int retval;

    retval = regulator_disable(tpd->reg);
    if (retval != 0)
        FTS_ERROR("[POWER]Failed to disable regulator when suspend ret=%d!", retval);
}

int fts_power_resume(void)
{
    int retval = 0;

    retval = regulator_enable(tpd->reg);
    if (retval != 0)
        FTS_ERROR("[POWER]Failed to enable regulator when resume,ret=%d!", retval);

    return retval;
}
#endif

/*****************************************************************************
*  Reprot related
*****************************************************************************/
/*****************************************************************************
*  Name: fts_release_all_finger
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void fts_release_all_finger(void)
{
#if FTS_MT_PROTOCOL_B_EN
    unsigned int finger_count = 0;
#endif

    FTS_FUNC_ENTER();

    mutex_lock(&report_mutex);

#if FTS_MT_PROTOCOL_B_EN
    for (finger_count = 0; finger_count < tpd_dts_data.touch_max_num; finger_count++) {
        input_mt_slot(fts_input_dev, finger_count);
        input_mt_report_slot_state(fts_input_dev, MT_TOOL_FINGER, false);
    }
#else
    input_mt_sync(fts_input_dev);
#endif

    input_report_key(fts_input_dev, BTN_TOUCH, 0);
    input_sync(fts_input_dev);
    input_report_key(fts_input_dev, KEY_BACK, 0);
    input_sync(fts_input_dev);
    input_report_key(fts_input_dev, KEY_HOMEPAGE, 0);
    input_sync(fts_input_dev);
    input_report_key(fts_input_dev, KEY_BACK, 0);
    input_sync(fts_input_dev);

    mutex_unlock(&report_mutex);

    FTS_FUNC_EXIT();
}

#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
char g_sz_debug[1024] = { 0 };

#define FTS_ONE_TCH_LEN     FTS_TOUCH_STEP
static void fts_show_touch_buffer(u8 * buf, int point_num)
{
    int len = point_num * FTS_ONE_TCH_LEN;
    int count = 0;
    int i;

    memset(g_sz_debug, 0, 1024);
    if (len > (POINT_READ_BUF - 3)) {
        len = POINT_READ_BUF - 3;
    }
    else if (len == 0) {
        len += FTS_ONE_TCH_LEN;
    }
    count += sprintf(g_sz_debug, "%02X,%02X,%02X", buf[0], buf[1], buf[2]);
    for (i = 0; i < len; i++) {
        count += sprintf(g_sz_debug + count, ",%02X", buf[i + 3]);
    }
    FTS_DEBUG("buffer: %s", g_sz_debug);
}
#endif

#if (!FTS_MT_PROTOCOL_B_EN)
static void tpd_down(int x, int y, int p, int id)
{
    if ((id < 0) || (id > 9))
        return;
    input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);
    FTS_DEBUG("tpd_down::finger_id(%d);X_Y_location(%4d,%4d) DOWN!", id, x, y);

}

static void tpd_up(int x, int y)
{
    FTS_DEBUG("[A]All Up!");
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_mt_sync(tpd->dev);
}

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
    int i = 0;
    int ret;
    u8 data[POINT_READ_BUF] = { 0 };
    u16 high_byte, low_byte;
    char writebuf[10] = { 0 };

#if FTS_READ_TOUCH_BUFFER_DIVIDED
    u8 pointnum;
    memset(data, 0xff, POINT_READ_BUF);
    ret = fts_i2c_read(fts_i2c_client, writebuf, 1, data, 3 + FTS_TOUCH_STEP);
    if (ret < 0) {
        FTS_ERROR("[A]Read touchdata failed, ret: %d", ret);
        return ret;
    }

    pointnum = data[2] & 0x0f;
    if (pointnum > 1) {
        writebuf[0] = 9;
        ret = fts_i2c_read(fts_i2c_client, writebuf, 1, data + 9, (pointnum - 1) * FTS_TOUCH_STEP);
        if (ret < 0) {
            FTS_ERROR("[A]Read touchdata failed, ret: %d", ret);
            return ret;
        }
    }
#else
    ret = fts_i2c_read(fts_i2c_client, writebuf, 1, data, POINT_READ_BUF);
    if (ret < 0) {
        FTS_ERROR("[B]Read touchdata failed, ret: %d", ret);
        FTS_FUNC_EXIT();
        return ret;
    }
#endif

    if ((data[0] & 0x70) != 0)
        return false;

    memcpy(pinfo, cinfo, sizeof(struct touch_info));
    memset(cinfo, 0, sizeof(struct touch_info));
    for (i = 0; i < tpd_dts_data.touch_max_num; i++)
        cinfo->p[i] = 1;        /* Put up */

    /*get the number of the touch points */
    cinfo->count = data[2] & 0x0f;
    FTS_DEBUG("Number of touch points = %d", cinfo->count);

#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
    fts_show_touch_buffer(data, cinfo->count);
#endif
    for (i = 0; i < cinfo->count; i++) {
        cinfo->p[i] = (data[3 + 6 * i] >> 6) & 0x0003;  /* event flag */
        cinfo->id[i] = data[3 + 6 * i + 2] >> 4;    // touch id

        /*get the X coordinate, 2 bytes */
        high_byte = data[3 + 6 * i];
        high_byte <<= 8;
        high_byte &= 0x0F00;

        low_byte = data[3 + 6 * i + 1];
        low_byte &= 0x00FF;
        cinfo->x[i] = high_byte | low_byte;

        /*get the Y coordinate, 2 bytes */
        high_byte = data[3 + 6 * i + 2];
        high_byte <<= 8;
        high_byte &= 0x0F00;

        low_byte = data[3 + 6 * i + 3];
        low_byte &= 0x00FF;
        cinfo->y[i] = high_byte | low_byte;

        FTS_DEBUG(" cinfo->x[%d] = %d, cinfo->y[%d] = %d, cinfo->p[%d] = %d", i, cinfo->x[i], i, cinfo->y[i], i, cinfo->p[i]);
    }

    return true;
};
#else
/************************************************************************
* Name: fts_read_touchdata
* Brief: report the point information
* Input: event info
* Output: get touch data in pinfo
* Return: success is zero
***********************************************************************/
static int fts_read_touchdata(struct ts_event *data)
{
    u8 buf[POINT_READ_BUF] = { 0 };
    int ret = -1;
    int i = 0;
    u8 pointid = FTS_MAX_ID;

    FTS_FUNC_ENTER();
#if FTS_READ_TOUCH_BUFFER_DIVIDED
    memset(buf, 0xff, POINT_READ_BUF);
    buf[0] = 0x0;
    ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, 3 + FTS_TOUCH_STEP);
    if (ret < 0) {
        FTS_ERROR("[B]Read touchdata failed, ret: %d", ret);
        return ret;
    }
    ret = data->touchs;
    memset(data, 0, sizeof(struct ts_event));
    data->touchs = ret;
    data->touch_point_num = buf[FT_TOUCH_POINT_NUM] & 0x0F;

    if (data->touch_point_num > 1) {
        buf[9] = 9;
        ret = fts_i2c_read(fts_i2c_client, buf + 9, 1, buf + 9, (data->touch_point_num - 1) * FTS_TOUCH_STEP);
        if (ret < 0) {
            FTS_ERROR("[B]Read touchdata failed, ret: %d", ret);
            return ret;
        }
    }
#else
    ret = fts_i2c_read(fts_i2c_client, buf, 1, buf, POINT_READ_BUF);
    if (ret < 0) {
        FTS_ERROR("[B]Read touchdata failed, ret: %d", ret);
        FTS_FUNC_EXIT();
        return ret;
    }
    ret = data->touchs;
    memset(data, 0, sizeof(struct ts_event));
    data->touchs = ret;
    data->touch_point_num = buf[FT_TOUCH_POINT_NUM] & 0x0F;
#endif
    data->touch_point = 0;

#if (FTS_DEBUG_EN && (FTS_DEBUG_LEVEL == 2))
    fts_show_touch_buffer(buf, data->touch_point_num);
#endif

    for (i = 0; i < tpd_dts_data.touch_max_num; i++) {
        pointid = (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;
        if (pointid >= FTS_MAX_ID)
            break;
        else
            data->touch_point++;
        data->au16_x[i] = (s16) (buf[FTS_TOUCH_X_H_POS + FTS_TOUCH_STEP * i] & 0x0F) << 8 | (s16) buf[FTS_TOUCH_X_L_POS + FTS_TOUCH_STEP * i];
        data->au16_y[i] = (s16) (buf[FTS_TOUCH_Y_H_POS + FTS_TOUCH_STEP * i] & 0x0F) << 8 | (s16) buf[FTS_TOUCH_Y_L_POS + FTS_TOUCH_STEP * i];
        data->au8_touch_event[i] = buf[FTS_TOUCH_EVENT_POS + FTS_TOUCH_STEP * i] >> 6;
        data->au8_finger_id[i] = (buf[FTS_TOUCH_ID_POS + FTS_TOUCH_STEP * i]) >> 4;

        data->pressure[i] = (buf[FTS_TOUCH_XY_POS + FTS_TOUCH_STEP * i]);   //cannot constant value
        data->area[i] = (buf[FTS_TOUCH_MISC + FTS_TOUCH_STEP * i]) >> 4;
        if ((data->au8_touch_event[i] == 0 || data->au8_touch_event[i] == 2) && (data->touch_point_num == 0)) {
            FTS_DEBUG("abnormal touch data from fw");
            return -1;
        }
    }
    if (data->touch_point == 0) {
        return -1;
    }
    FTS_FUNC_EXIT();
    return 0;
}

/************************************************************************
* Name: fts_report_key
* Brief: report key event
* Input: event info
* Output: no
* Return: 0: is key event, -1: isn't key event
***********************************************************************/
static int fts_report_key(struct ts_event *data)
{
    int i = 0;

    if (1 != data->touch_point)
        return -1;

    for (i = 0; i < tpd_dts_data.touch_max_num; i++) {
        if (data->au16_y[i] <= TPD_RES_Y) {
            return -1;
        }
        else {
            break;
        }
    }
    /* [liliwen start] Modify for button reverse */
    if (data->au8_touch_event[i] == 0 || data->au8_touch_event[i] == 2) {
        if (button_reverse == 1) {
            if ((data->au16_x[i] == FTS_BUTTON_LEFT_X) && (key_back_button == 0)) {
                input_report_key(tpd->dev, KEY_BACK, 1);
                key_back_button = 1;
                input_sync(tpd->dev);
            } else if ((data->au16_x[i] == FTS_BUTTON_CENTER_X) && (key_home_button == 0)) {
                input_report_key(tpd->dev, KEY_HOMEPAGE, 1);
                key_home_button = 1;
                input_sync(tpd->dev);
            } else if ((data->au16_x[i] == FTS_BUTTON_RIGHT_X) && (key_recent_button == 0)) {
                input_report_key(tpd->dev, KEY_RECENT, 1);
                key_recent_button = 1;
                input_sync(tpd->dev);
            }
        } else {
            if (data->au16_x[i] == FTS_BUTTON_LEFT_X && (key_recent_button == 0)) {
                input_report_key(tpd->dev, KEY_RECENT, 1);
                key_recent_button = 1;
                input_sync(tpd->dev);
            } else if (data->au16_x[i] == FTS_BUTTON_CENTER_X && (key_home_button == 0)) {
                input_report_key(tpd->dev, KEY_HOMEPAGE, 1);
                key_home_button = 1;
                input_sync(tpd->dev);
            } else if (data->au16_x[i] == FTS_BUTTON_RIGHT_X && (key_back_button == 0)) {
                input_report_key(tpd->dev, KEY_BACK, 1);
                key_back_button = 1;
                input_sync(tpd->dev);
            }
        }
        //tpd_button(data->au16_x[i], data->au16_y[i], 1);
        FTS_DEBUG("[B]Key(%d, %d) DOWN!", data->au16_x[i], data->au16_y[i]);
    } else {
        if (key_back_button == 1) {
            input_report_key(tpd->dev, KEY_BACK, 0);
            input_sync(tpd->dev);
            key_back_button = 0;
        } else if (key_home_button == 1) {
            input_report_key(tpd->dev, KEY_HOMEPAGE, 0);
            input_sync(tpd->dev);
            key_home_button = 0;
        } else if (key_recent_button == 1) {
            input_report_key(tpd->dev, KEY_RECENT, 0);
            input_sync(tpd->dev);
            key_recent_button = 0;
        }
        //tpd_button(data->au16_x[i], data->au16_y[i], 0);
        FTS_DEBUG("[B]Key(%d, %d) UP!", data->au16_x[i], data->au16_y[i]);
    }

    //input_sync(tpd->dev);
    /* [liliwen end] */

    return 0;
}

/************************************************************************
* Name: fts_report_touch
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
static int fts_report_touch(struct ts_event *data)
{
    int i = 0;
    int up_point = 0;
    int touchs = 0;

    for (i = 0; i < data->touch_point; i++) {
        if (data->au8_finger_id[i] >= tpd_dts_data.touch_max_num) {
            break;
        }

        input_mt_slot(tpd->dev, data->au8_finger_id[i]);

        if (data->au8_touch_event[i] == 0 || data->au8_touch_event[i] == 2) {
            input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, true);
            if (data->area[i] <= 0) {
                FTS_ERROR("[B]Illegal touch-major: %d", data->area[i]);
                data->area[i] = 1;
            }
            input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, data->area[i]);
            input_report_abs(tpd->dev, ABS_MT_POSITION_X, data->au16_x[i]);
            input_report_abs(tpd->dev, ABS_MT_POSITION_Y, data->au16_y[i]);
            touchs |= BIT(data->au8_finger_id[i]);
            data->touchs |= BIT(data->au8_finger_id[i]);
            FTS_DEBUG("[B]tpd_down::finger_id(%d);X_Y_locartion(%4d,%4d)DOWN!", data->au8_finger_id[i], data->au16_x[i], data->au16_y[i]);
        }
        else {
            up_point++;
            input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
            data->touchs &= ~BIT(data->au8_finger_id[i]);
            FTS_DEBUG("[B]P%d UP fw!", data->au8_finger_id[i]);
        }

    }
    for (i = 0; i < tpd_dts_data.touch_max_num; i++) {
        if (BIT(i) & (data->touchs ^ touchs)) {
            FTS_DEBUG("[B]P%d UP driver!", i);
            data->touchs &= ~BIT(i);
            input_mt_slot(tpd->dev, i);
            input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
        }
    }
    data->touchs = touchs;

    if ((data->touch_point == up_point) || !data->touch_point_num) {
        FTS_DEBUG("[B]Points All UP!");
        input_report_key(tpd->dev, BTN_TOUCH, 0);
    }
    else {
        input_report_key(tpd->dev, BTN_TOUCH, 1);
    }

    input_sync(tpd->dev);
    return 0;
}
#endif

/*****************************************************************************
*  Name: tpd_eint_interrupt_handler
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
{
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}

/*****************************************************************************
*  Name: tpd_irq_registration
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int tpd_irq_registration(struct i2c_client *client)
{
    struct device_node *node = NULL;
    int ret = 0;
    u32 ints[2] = { 0, 0 };

    FTS_FUNC_ENTER();
    node = of_find_matching_node(node, touch_of_match);
    if (node) {
        of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
        gpio_set_debounce(ints[0], ints[1]);

        ft_touch_irq = irq_of_parse_and_map(node, 0);
        ret = request_irq(ft_touch_irq, tpd_eint_interrupt_handler, IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);
        if (ret == 0) {
            FTS_INFO("IRQ request succussfully, irq=%d", ft_touch_irq);
            client->irq = ft_touch_irq;
        }
        else
            FTS_ERROR("tpd request_irq IRQ LINE NOT AVAILABLE!.");

    }
    else {
        FTS_ERROR("Can not find touch eint device node!");
    }
    FTS_FUNC_EXIT();
    return 0;
}

#if !(FTS_MT_PROTOCOL_B_EN)
static int fts_report_key_a(struct touch_info *cinfo, struct touch_info *pinfo, struct touch_info *finfo)
{
    if (tpd_dts_data.use_tpd_button) {
        if (cinfo->p[0] == 0)
            memcpy(finfo, cinfo, sizeof(struct touch_info));
    }

    if ((cinfo->y[0] >= TPD_RES_Y) && (pinfo->y[0] < TPD_RES_Y)
        && ((pinfo->p[0] == 0) || (pinfo->p[0] == 2))) {
        FTS_DEBUG("All up");
        tpd_up(pinfo->x[0], pinfo->y[0]);
        input_sync(tpd->dev);
        return 0;
    }

    if (tpd_dts_data.use_tpd_button) {
        if ((cinfo->y[0] <= TPD_RES_Y && cinfo->y[0] != 0) && (pinfo->y[0] > TPD_RES_Y)
            && ((pinfo->p[0] == 0) || (pinfo->p[0] == 2))) {
            FTS_DEBUG("All key up");
            tpd_button(pinfo->x[0], pinfo->y[0], 0);
            input_sync(tpd->dev);
            return 0;
        }

        if ((cinfo->y[0] > TPD_RES_Y) || (pinfo->y[0] > TPD_RES_Y)) {
            if (finfo->y[0] > TPD_RES_Y) {
                if ((cinfo->p[0] == 0) || (cinfo->p[0] == 2)) {
                    FTS_DEBUG("Key(%d,%d) Down", pinfo->x[0], pinfo->y[0]);
                    tpd_button(pinfo->x[0], pinfo->y[0], 1);
                }
                else if ((cinfo->p[0] == 1) && ((pinfo->p[0] == 0) || (pinfo->p[0] == 2))) {
                    FTS_DEBUG("Key(%d,%d) Up!", pinfo->x[0], pinfo->y[0]);
                    tpd_button(pinfo->x[0], pinfo->y[0], 0);
                }
                input_sync(tpd->dev);
            }
            return 0;
        }
    }
    return -1;
}

static void fts_report_touch_a(struct touch_info *cinfo)
{
    int i;
    if (cinfo->count > 0) {
        for (i = 0; i < cinfo->count; i++) {
            tpd_down(cinfo->x[i], cinfo->y[i], i + 1, cinfo->id[i]);
        }
    }
    else {
        tpd_up(cinfo->x[0], cinfo->y[0]);
    }
    input_sync(tpd->dev);
}
#endif

/*****************************************************************************
*  Name: touch_event_handler
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int touch_event_handler(void *unused)
{
    int i = 0;
    int ret;

#if FTS_GESTURE_EN
    u8 state = 0;
#endif
#if FTS_MT_PROTOCOL_B_EN
    struct ts_event pevent;
#else
    struct touch_info cinfo, pinfo;
#endif

    struct touch_info finfo;
    struct sched_param param = {.sched_priority = RTPM_PRIO_TPD };

    if (tpd_dts_data.use_tpd_button) {
        memset(&finfo, 0, sizeof(struct touch_info));
        for (i = 0; i < tpd_dts_data.touch_max_num; i++)
            finfo.p[i] = 1;
    }
#if !FTS_MT_PROTOCOL_B_EN
    memset(&cinfo, 0, sizeof(struct touch_info));
    memset(&pinfo, 0, sizeof(struct touch_info));
#endif
    sched_setscheduler(current, SCHED_RR, &param);

    do {
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, tpd_flag != 0);

        tpd_flag = 0;

        set_current_state(TASK_RUNNING);

        atomic_add(1, &interrupt_counter); /* [jiangtingyu] add tp report rate node */

#if FTS_GESTURE_EN
        ret = fts_i2c_read_reg(fts_i2c_client, FTS_REG_GESTURE_EN, &state);
        if (ret < 0) {
            FTS_ERROR("[Focal][Touch] read value fail");
        }
        if (state == 1) {
            /* [liliwen] Change to fts_read_Gesturedata */
            fts_read_Gesturedata(fts_i2c_client);
            continue;
        }
#endif

        /* [liliwen] Add for close incall */
        if (atomic_read(&close_incall) == 1)
            continue;

#if FTS_POINT_REPORT_CHECK_EN
        fts_point_report_check_queue_work();
#endif

        FTS_DEBUG("touch_event_handler start");
#if FTS_ESDCHECK_EN
        fts_esdcheck_set_intr(1);
#endif
#if FTS_MT_PROTOCOL_B_EN
        {
            ret = fts_read_touchdata(&pevent);
            if (ret == 0) {
                mutex_lock(&report_mutex);
                if (tpd_dts_data.use_tpd_button) {
                    ret = !fts_report_key(&pevent);
                }
                if (ret == 0) {
                    /* [yanlin start] Handle virtual key (1/2) */
                    if (key_back_button) {
                        input_report_key(tpd->dev, KEY_BACK, 0);
                        input_sync(tpd->dev);
                        key_back_button = 0;
                    }
                    if (key_recent_button) {
                        input_report_key(tpd->dev, KEY_RECENT, 0);
                        input_sync(tpd->dev);
                        key_recent_button = 0;
                    }
                    if (key_home_button) {
                        input_report_key(tpd->dev, KEY_HOMEPAGE, 0);
                        input_sync(tpd->dev);
                        key_home_button = 0;
                    }
                    /* [yanlin end] */
                    fts_report_touch(&pevent);
                }
                mutex_unlock(&report_mutex);
            }
        }
#else //FTS_MT_PROTOCOL_A_EN
        {
            ret = tpd_touchinfo(&cinfo, &pinfo);
            if (ret) {
                mutex_lock(&report_mutex);
                ret = fts_report_key_a(&cinfo, &pinfo, &finfo);
                if (ret) {
                    fts_report_touch_a(&cinfo);
                }
                mutex_unlock(&report_mutex);

            }
        }
#endif
#if FTS_ESDCHECK_EN
        fts_esdcheck_set_intr(0);
#endif
    }
    while (!kthread_should_stop());

    return 0;
}

/************************************************************************
* Name: tpd_i2c_detect
* Brief:
* Input:
* Output:
* Return:
***********************************************************************/
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE);

    return 0;
}

/* [liliwen start] Add tp_info */
#define  TP_INFO_PAGE_SIZE        512
ssize_t tp_info_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned char panel_type[] = "ft5436";
    unsigned char panel_ic[3][10] = {"truly", "booyi", "unknown"};
    unsigned char  driver_version[] = "1.0";
    u32 nLength = 0;
    int vendor = 0;
    u8 fts_firmware_version = 0;
    u8 vendor_id = 0;

    mutex_lock(&fts_input_dev->mutex);
    fts_i2c_read_reg(fts_i2c_client, FTS_REG_FW_VER, &fts_firmware_version);
    fts_i2c_read_reg(fts_i2c_client, FTS_REG_VENDOR_ID, &vendor_id);
    if (vendor_id == FTS_VENDOR_1_ID) { // 0x05
        vendor = 0; // truly TP
    } else if (vendor_id == FTS_VENDOR_2_ID) { // 0x55
        vendor = 1; // booyi TP
    } else {
        vendor = 2; // unknown
    }
    nLength = snprintf(buf, TP_INFO_PAGE_SIZE,
        "type:\t%s\n"
        "vendor:\t%s\n"
        "firmware_version:\t0x%x\n"
        "driver_version:\t%s\n",
        panel_type,
        &panel_ic[vendor][0],
        fts_firmware_version,
        driver_version);
    mutex_unlock(&fts_input_dev->mutex);

    return nLength;
}
static DEVICE_ATTR(tp_info, S_IRUGO, tp_info_read, NULL);
/* [liliwen end] */

/* [liliwen start] Add fts_init_proc */
static int fts_init_proc(void)
{
    int ret = 0;

    prEntry_tp = proc_mkdir("touchpanel", NULL);
    if (prEntry_tp == NULL) {
        ret = -ENOMEM;
        printk("touchpanle:can not create proc entry!\n");
    }

#if FTS_GESTURE_EN
    prEntry_dtap = proc_create("gesture_coordinate", 0444, prEntry_tp, &gesture_coordinate_fops);
    if (prEntry_dtap == NULL) {
        ret = -ENOMEM;
        printk("gesture_coordinate can not create\n");
    }

    prEntry_dtap = proc_create("double_tap_enable", 0666, prEntry_tp, &double_tap_enable_fops);
    if (prEntry_dtap == NULL) {
        ret = -ENOMEM;
        printk("double_tap_enable can not create\n");
    }

    prEntry_dtap = proc_create("support_gesture_id", 0666, prEntry_tp, &support_gesture_id_fops);
    if (prEntry_dtap == NULL) {
        ret = -ENOMEM;
        printk("support_gesture_id can not create\n");
    }
#endif

#if FTS_GLOVE_EN
    prEntry_dtap = proc_create("glove_mode_enable", 0666, prEntry_tp, &fts_glove_enable_fops);
    if (prEntry_dtap == NULL) {
        ret = -ENOMEM;
        printk("glove_enable can not create\n");
    }
#endif

    prEntry_dtap = proc_create("button_reverse_enable", 0666, prEntry_tp, &fts_button_reverse_fops);
    if (prEntry_dtap == NULL) {
        ret = -ENOMEM;
        printk("button_reverse_enable can not create\n");
    }

    prEntry_dtap = proc_create("firmware_update_enable", 0666, prEntry_tp, &firmware_update_fops);
    if(prEntry_dtap == NULL)
    {
        ret = -ENOMEM;
        printk("firmware_update_enable can not create\n");
    }

#if TPLINK_CHANGE_TP_THRESHOLD_ENABLE
    prEntry_dtap = proc_create("tp_thresh_value", 0666, prEntry_tp, &tp_thresh_value_fops);
    if(prEntry_dtap == NULL)
    {
        ret = -ENOMEM;
        printk("tp_thresh_value can not create\n");
    }
#endif //TPLINK_CHANGE_TP_THRESHOLD_ENABLE

#if TPLINK_READ_DIFFDATA_ENABLE
    prEntry_dtap = proc_create("diffdata_read_enable", 0666, prEntry_tp, &diffdata_read_fops);
    if(prEntry_dtap == NULL)
    {
        ret = -ENOMEM;
        printk("diffdata_read_enable can not create\n");
    }
#endif //TPLINK_READ_DIFFDATA_ENABLE

    prEntry_dtap = proc_create("psensor_incall_disable_tp", 0666, prEntry_tp, &close_incall_fops);
    if(prEntry_dtap == NULL)
    {
        ret = -ENOMEM;
        printk("psensor_incall_disable_tp can not create\n");
    }

    prEntry_dtap = proc_create("report_rate", 0666, prEntry_tp, &report_rate_fops);
    if(prEntry_dtap == NULL)
    {
        ret = -ENOMEM;
        printk("tplink touchpanel report rate can not create\n");
    }
    return ret;
}
/* [liliwen end] */

/************************************************************************
* Name: fts_probe
* Brief:
* Input:
* Output:
* Return:
***********************************************************************/
extern void pmic_ldo_vldo28_sw_en(int en_value);/* [lichenggang start]modify power squeence*/

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int retval = 0;

    FTS_FUNC_ENTER();

    fts_i2c_client = client;
    fts_input_dev = tpd->dev;
    /* [liliwen] Add for close incall */
    atomic_set(&close_incall, 0);

    if (fts_i2c_client->addr != FTS_I2C_SLAVE_ADDR) {
        FTS_INFO("[TPD]Change i2c addr 0x%02x to 0x38", fts_i2c_client->addr);
        fts_i2c_client->addr = FTS_I2C_SLAVE_ADDR;
        FTS_DEBUG("[TPD]fts_i2c_client->addr=0x%x\n", fts_i2c_client->addr);
    }

    /* [liliwen] Change TP i2c speed to 400kHz */
    fts_i2c_client->timing = 400;

    spin_lock_init(&irq_lock);
    mutex_init(&report_mutex);

    /* Configure gpio to irq and request irq */
    tpd_gpio_as_int(tpd_int_gpio_number);
    tpd_irq_registration(client);
    fts_irq_disable();

    /* Init I2C */
    fts_i2c_init();

    fts_ctpm_get_upgrade_array();


#if FTS_MT_PROTOCOL_B_EN
    input_mt_init_slots(tpd->dev, tpd_dts_data.touch_max_num, INPUT_MT_DIRECT);
#endif

/* [liliwen start] gesture key event */
#if FTS_GESTURE_EN
    set_bit(KEY_F4 , tpd->dev->keybit); //gesture key event
#endif
/* [liliwen end] */

    // TP probe power on sequence start
    /* [liliwen] Change for TP sequence */
#if 1
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(2);
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(1);
#if FTS_POWER_SOURCE_CUST_EN
    if (fts_power_init() != 0)
        return -1;
#endif
    /* [lichenggang start]modify power squeence*/
    pmic_ldo_vldo28_sw_en(1);
    /* [lichenggang end] */

    msleep(20);
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(10);
#else
    tpd_gpio_output(tpd_rst_gpio_number, 1);
    msleep(5);
    tpd_gpio_output(tpd_rst_gpio_number, 0);
    msleep(20);
    tpd_gpio_output(tpd_rst_gpio_number, 1);
    msleep(10);
#endif

    mt_set_gpio_mode(tpd_int_gpio_number,GPIO_MODE_01);
    mt_set_gpio_dir(tpd_int_gpio_number, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(tpd_int_gpio_number, GPIO_PULL_DISABLE);
    /* [liliwen end] */
    // TP probe power on sequence end

    fts_wait_tp_to_valid(client);

    tpd_load_status = 1;

#if FTS_SYSFS_NODE_EN
    fts_create_sysfs(client);
#endif

    /* [liliwen] Add for sysfs proc */
    fts_init_proc();

/* [liliwen start] Add tp_info */
    if (sysfs_create_file(kernel_kobj, &dev_attr_tp_info.attr)) {
        printk("tp910::sysfs_create_file tp_info fail\n");
        return -ENODEV;
    }
/* [liliwen end] */

#if FTS_POINT_REPORT_CHECK_EN
    fts_point_report_check_init();
#endif

#if FTS_APK_NODE_EN
    fts_create_apk_debug_channel(client);
#endif

    thread_tpd = kthread_run(touch_event_handler, 0, TPD_DEVICE);
    if (IS_ERR(thread_tpd)) {
        retval = PTR_ERR(thread_tpd);
        FTS_ERROR("[TPD]Failed to create kernel thread_tpd,ret=%d!", retval);
    }

    FTS_DEBUG("[TPD]Touch Panel Device Probe %s!", (retval < 0) ? "FAIL" : "PASS");

/* [liliwen start] Add for TP threshold change */
#if TPLINK_CHANGE_TP_THRESHOLD_ENABLE
    tplink_change_thresh_value(client);
#endif
/* [liliwen end] */


#if FTS_ESDCHECK_EN
    fts_esdcheck_init();
#endif

    fts_irq_enable();           /* need execute before upgrade */

#if FTS_AUTO_UPGRADE_EN
    fts_ctpm_upgrade_init();
#endif

#if FTS_TEST_EN
    fts_test_init(client);
#endif

    FTS_FUNC_EXIT();

    /* [liliwen start] TP probe state */
    TP910_FT5436_PROBE_OK = 1;
    TP_IS_FTS = 1;
    /* [liliwen end] */

    return 0;
}

/*****************************************************************************
*  Name: tpd_remove
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int tpd_remove(struct i2c_client *client)
{
    FTS_FUNC_ENTER();

#if FTS_TEST_EN
    fts_test_exit(client);
#endif

#if FTS_POINT_REPORT_CHECK_EN
    fts_point_report_check_exit();
#endif

#if FTS_SYSFS_NODE_EN
    fts_remove_sysfs(client);
#endif


#if FTS_APK_NODE_EN
    fts_release_apk_debug_channel();
#endif

#if FTS_AUTO_UPGRADE_EN
    fts_ctpm_upgrade_exit();
#endif

#if FTS_ESDCHECK_EN
    fts_esdcheck_exit();
#endif

    fts_i2c_exit();

    FTS_FUNC_EXIT();

    return 0;
}

/*****************************************************************************
*  Name: tpd_local_init
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int tpd_local_init(void)
{
    FTS_FUNC_ENTER();
    if (i2c_add_driver(&tpd_i2c_driver) != 0) {
        FTS_ERROR("[TPD]: Unable to add fts i2c driver!!");
        FTS_FUNC_EXIT();
        return -1;
    }

    if (tpd_dts_data.use_tpd_button) {
        tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local, tpd_dts_data.tpd_key_dim_local);
    }

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(CONFIG_TPD_HAVE_CALIBRATION) && !defined(CONFIG_TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local_factory, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local_factory, 8 * 4);

    memcpy(tpd_calmat, tpd_def_calmat_local_normal, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local_normal, 8 * 4);
#endif

    tpd_type_cap = 1;

    FTS_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
*  Name: tpd_suspend
*  Brief: When suspend, will call this function
*           1. Set gesture if EN
*           2. Disable ESD if EN
*           3. Process MTK sensor hub if configure, default n, if n, execute 4/5/6
*           4. disable irq
*           5. Set TP to sleep mode
*           6. Disable power(regulator) if EN
*           7. fts_release_all_finger
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void tpd_suspend(struct device *h)
{
    int retval = 0;

    FTS_FUNC_ENTER();

    /* [liliwen] Add tp_suspend */
    tp_suspend = 1;
    fts_release_all_finger();

#if FTS_ESDCHECK_EN
    fts_esdcheck_suspend();
#endif

#if FTS_GESTURE_EN
    /* [liliwen start] Change for gesture */
    if(1 == atomic_read(&double_enable)) {
        retval = fts_gesture_suspend(fts_i2c_client);
        if (retval == 0) {
            /* Enter into gesture mode(suspend) */
            return;
        }
    }
    /* [liliwen end] */
#endif

    fts_irq_disable();
    /* [liliwen] Add for close incall */
    atomic_set(&close_incall, 0);
    /* TP enter sleep mode */
    retval = fts_i2c_write_reg(fts_i2c_client, FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP_VALUE);
    if (retval < 0) {
        FTS_ERROR("Set TP to sleep mode fail, ret=%d!", retval);
    }

    // TP suspend power off sequence start
    /* [liliwen start] Add for TP sequence */
    tpd_gpio_output(tpd_rst_gpio_number, 0); //llw
    msleep(10);
    /* [liliwen end] */

    /* [lichenggang start]power off when IC suspend and no need gesture*/
    pmic_ldo_vldo28_sw_en(0);
    /* [lichenggang end] */

#if FTS_POWER_SOURCE_CUST_EN
    fts_power_suspend();
#endif
    // TP suspend power off sequence end
    FTS_FUNC_EXIT();
}

/*****************************************************************************
*  Name: tpd_resume
*  Brief: When suspend, will call this function
*           1. Clear gesture if EN
*           2. Enable power(regulator) if EN
*           3. Execute reset if no IDC to wake up
*           4. Confirm TP in valid app by read chip ip register:0xA3
*           5. fts_release_all_finger
*           6. Enable ESD if EN
*           7. tpd_usb_plugin if EN
*           8. Process MTK sensor hub if configure, default n, if n, execute 9
*           9. disable irq
*  Input:
*  Output:
*  Return:
*****************************************************************************/

static void tpd_resume(struct device *h)
{
    FTS_FUNC_ENTER();
    /* [liliwen] Add tp_suspend */
    tp_suspend = 0;
    /* [liliwen] Add for close incall */
    atomic_set(&close_incall, 0);

    /* [lichenggang start]power on when IC resume*/
    pmic_ldo_vldo28_sw_en(1);
    /* [lichenggang end] */

    fts_release_all_finger();

    // TP resume power on sequence start
    /* [liliwen start] Add for TP sequence */
    mt_set_gpio_mode(tpd_int_gpio_number, GPIO_MODE_01);
    mt_set_gpio_dir(tpd_int_gpio_number, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(tpd_int_gpio_number, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(tpd_int_gpio_number,GPIO_PULL_UP);
    /* [liliwen end] */

#if FTS_POWER_SOURCE_CUST_EN
    fts_power_resume();
#endif

#if (!FTS_CHIP_IDC)
    /* [liliwen start] Add for TP rst */
    tpd_gpio_output(tpd_rst_gpio_number, 1); //llw
    msleep(2);
    fts_reset_proc(10);
    /* [liliwen end] */
#endif

    /* [liliwen start] Add for TP sequence */
    mt_set_gpio_mode(tpd_int_gpio_number, GPIO_MODE_01);
    mt_set_gpio_dir(tpd_int_gpio_number, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(tpd_int_gpio_number, GPIO_PULL_DISABLE);
    /* [liliwen end] */
    // TP resume power on sequence end

/* [liliwen start] Add for gesture */
#if FTS_GESTURE_EN
    if (1 == atomic_read(&double_enable))
        fts_i2c_write_reg(fts_i2c_client, 0xD0, 0x00);
#endif
/* [liliwen end] */

    /* Before read/write TP register, need wait TP to valid */
    fts_tp_state_recovery(fts_i2c_client);

#if FTS_ESDCHECK_EN
    fts_esdcheck_resume();
#endif

/* [liliwen start] Add glove mode */
#if FTS_GLOVE_EN
    if ((atomic_read(&fts_glove_mode)) == 1) {
        fts_enter_glove_mode(fts_i2c_client, true);
    }
#endif
/* [liliwen end] */

/* [liliwen start] Add for TP threshold change */
#if TPLINK_CHANGE_TP_THRESHOLD_ENABLE
    tplink_change_thresh_value(fts_i2c_client);
#endif
/* [liliwen end] */
	fts_irq_enable();

}

int usb_check(int usb_state)
{
    /* [liliwen start] Change usb_check */
    int ret = 0;
    printk("usb_check =%d\n", usb_state);
    if(TP910_FT5436_PROBE_OK == 0) {
        pr_err("910 probe fail and return\n");
        return -1;
    }

    ret = fts_enter_charger_mode(fts_i2c_client, usb_state);
    /* [liliwen] fix usb_check bug */
    if (ret >= 0) {
        usb_check_state = usb_state;
        return usb_state;
    } else {
        pr_err("[TPD]fts_enter_charger_mode fail\n");
        return ret;
    }
    /* [liliwen end] */
}

/*****************************************************************************
*  TPD Device Driver
*****************************************************************************/

struct device_attribute *fts_attrs[] = {
};

static struct tpd_driver_t tpd_device_driver = {
    .tpd_device_name = FTS_DRIVER_NAME,
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
    .attrs = {
              .attr = fts_attrs,
              .num = ARRAY_SIZE(fts_attrs),
              },
};

/*****************************************************************************
*  Name: tpd_driver_init
*  Brief: 1. Get dts information
*         2. call tpd_driver_add to add tpd_device_driver
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static int __init tpd_driver_init(void)
{
    FTS_FUNC_ENTER();
    FTS_INFO("Driver version: %s", FTS_DRIVER_VERSION);
    tpd_get_dts_info();
    if (tpd_driver_add(&tpd_device_driver) < 0) {
        FTS_ERROR("[TPD]: Add FTS Touch driver failed!!");
    }

    FTS_FUNC_EXIT();
    return 0;
}

/*****************************************************************************
*  Name: tpd_driver_exit
*  Brief:
*  Input:
*  Output:
*  Return:
*****************************************************************************/
static void __exit tpd_driver_exit(void)
{
    FTS_FUNC_ENTER();
    tpd_driver_remove(&tpd_device_driver);
    FTS_FUNC_EXIT();
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver for Mediatek");
MODULE_LICENSE("GPL v2");
