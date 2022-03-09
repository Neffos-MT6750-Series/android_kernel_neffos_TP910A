/*
 *
 * FocalTech ftxxxx TouchScreen driver.
 *
 * Copyright (c) 2010-2017, Focaltech Ltd. All rights reserved.
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
* File Name: focaltech_ex_mode.c
*
*    Author: Liu WeiGuang
*
*   Created: 2016-08-31
*
*  Abstract:
*
* Reference:
*
*****************************************************************************/

/*****************************************************************************
* 1.Included header files
*****************************************************************************/
#include "focaltech_core.h"

/*****************************************************************************
* 2.Private constant and macro definitions using #define
*****************************************************************************/

/*****************************************************************************
* 3.Private enumerations, structures and unions using typedef
*****************************************************************************/

/* [liliwen] Add for glove_mode */
#if FTS_GLOVE_EN
atomic_t fts_glove_mode;
#endif

int button_reverse;

extern int usb_check_state;

#if TPLINK_CHANGE_TP_THRESHOLD_ENABLE
static int tp_thresh_value = 1;
#endif

#if TPLINK_READ_DIFFDATA_ENABLE
#define TX_NUM_MAX 14
#define RX_NUM_MAX 25
static int m_iTempRawData[TX_NUM_MAX * RX_NUM_MAX] = {0};
#define ERROR_CODE_COMM_ERROR                       0x0c
#define ERROR_CODE_OK                               0x00
#define ERROR_CODE_INVALID_COMMAND                  0x02
static int ic_mode_register = 0x00;
#endif

DEFINE_MUTEX(ft_ex_mode_lock);
/*[liliwen end] */

/*****************************************************************************
* 4.Static variables
*****************************************************************************/

/*****************************************************************************
* 5.Global variable or extern global variabls/functions
*****************************************************************************/
int fts_enter_glove_mode(struct i2c_client *client, int mode);
int fts_glove_init(struct i2c_client *client);
int fts_glove_exit(struct i2c_client *client);

int fts_enter_cover_mode(struct i2c_client *client, int mode);
int fts_cover_init(struct i2c_client *client);
int fts_cover_exit(struct i2c_client *client);

int fts_enter_charger_mode(struct i2c_client *client, int mode);
int fts_charger_init(struct i2c_client *client);
int fts_charger_exit(struct i2c_client *client);

/*****************************************************************************
* 6.Static function prototypes
*******************************************************************************/

#if FTS_GLOVE_EN

/* [liliwen start] Add for glove mode */
//sysfs proc
ssize_t fts_glove_enable_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
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
    mutex_lock(&ft_ex_mode_lock);

    if((ret == 1) || (ret == 0)) {
        if (ret != atomic_read(&fts_glove_mode)) {
            if (fts_enter_glove_mode(fts_i2c_client, ret) >= 0) {
                atomic_set(&fts_glove_mode, ret);
            }
        }
    }
    mutex_unlock(&ft_ex_mode_lock);
    return count;
}

ssize_t fts_glove_enable_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int len = 0;
    int ret = 0 ;
    char page[512];
    printk("glove_enable is: %d\n", atomic_read(&fts_glove_mode));
    len = sprintf(page, "%d\n", atomic_read(&fts_glove_mode));
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}
/* [liliwen end] */

/************************************************************************
* Name: fts_enter_glove_mode
* Brief:  change glove mode
* Input:  glove mode
* Output: no
* Return: success >=0, otherwise failed
***********************************************************************/
int fts_enter_glove_mode(struct i2c_client *client, int mode)
{
    int ret = 0;
    static u8 buf_addr[2] = { 0 };
    static u8 buf_value[2] = { 0 };
    buf_addr[0] = FTS_REG_GLOVE_MODE_EN;    //glove control

    if (mode)
        buf_value[0] = 0x01;
    else
        buf_value[0] = 0x00;

    ret = fts_i2c_write_reg(client, buf_addr[0], buf_value[0]);
    if (ret < 0) {
        FTS_ERROR("[Mode]fts_enter_glove_mode write value fail");
    }

    return ret;

}

#endif

#if FTS_COVER_EN

/************************************************************************
* Name: fts_enter_cover_mode
* Brief:  change cover mode
* Input:  cover mode
* Output: no
* Return: success >=0, otherwise failed
***********************************************************************/
int fts_enter_cover_mode(struct i2c_client *client, int mode)
{
    int ret = 0;
    static u8 buf_addr[2] = { 0 };
    static u8 buf_value[2] = { 0 };
    buf_addr[0] = FTS_REG_COVER_MODE_EN;    //cover control

    if (mode)
        buf_value[0] = 0x01;
    else
        buf_value[0] = 0x00;

    ret = fts_i2c_write_reg(client, buf_addr[0], buf_value[0]);
    if (ret < 0) {
        FTS_ERROR("[Mode] fts_enter_cover_mode write value fail \n");
    }

    return ret;

}

#endif

#if FTS_CHARGER_EN

/************************************************************************
* Name: fts_enter_charger_mode
* Brief:  change charger mode
* Input:  charger mode
* Output: no
* Return: success >=0, otherwise failed
***********************************************************************/
int fts_enter_charger_mode(struct i2c_client *client, int mode)
{
    int ret = 0;
    static u8 buf_addr[2] = { 0 };
    static u8 buf_value[2] = { 0 };
    buf_addr[0] = FTS_REG_CHARGER_MODE_EN;  //charger control

    if (mode)
        buf_value[0] = 0x01;
    else
        buf_value[0] = 0x00;

    ret = fts_i2c_write_reg(client, buf_addr[0], buf_value[0]);
    if (ret < 0) {
        FTS_DEBUG("[Mode]fts_enter_charger_mode write value fail");
    }

    return ret;

}

#endif

/* [liliwen start] Add for button_reverse */
//sysfs proc button_reverse_enable
ssize_t fts_button_reverse_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
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
    mutex_lock(&ft_ex_mode_lock);
    button_reverse = ret;
    mutex_unlock(&ft_ex_mode_lock);
    return count;
}

ssize_t fts_button_reverse_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int len = 0;
    int ret = 0;
    char page[512];
    //TPD_ERR("button_reverse is:%d\n", button_reverse);
    len = sprintf(page, "%d\n", button_reverse);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}
/* [liliwen end] */

/* [liliwen start] Add for tp_thresh_value */
//sysfs proc tp_thresh_value
#if TPLINK_CHANGE_TP_THRESHOLD_ENABLE
ssize_t tplink_thresh_value_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
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

    mutex_lock(&ft_ex_mode_lock);
    tp_thresh_value = ret;

    if (tplink_change_thresh_value(fts_i2c_client) == 0)
        printk("tplink_change_thresh_value OK\n");
    else
        printk("tplink_change_thresh_value failed\n");

    mutex_unlock(&ft_ex_mode_lock);
    return count;
}

ssize_t tplink_thresh_value_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    int len = 0;
    int ret = 0;
    char page[512];
    //TPD_ERR("tp_thresh_value=%d\n",tp_thresh_value);
    len = sprintf(page, "%d\n", tp_thresh_value);
    ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
    return ret;
}

int tplink_change_thresh_value(struct i2c_client *client)
{
    u8 read_data;
    u8 vendor_id = 0;

    fts_i2c_read_reg(client, FTS_REG_CHARGER_MODE_EN, &read_data);
    printk("usb_check_state=%d 0x8B=%d\n", usb_check_state, read_data);
    if(read_data != 1){
        fts_i2c_write_reg(client, FTS_REG_CHARGER_MODE_EN, 1); // 0x8B register
        msleep(50);
        fts_i2c_read_reg(client, FTS_REG_CHARGER_MODE_EN, &read_data);
        if(read_data != 1) {
            //TPD_ERR("!!!!!!write error!!!!!::0x8B=%d\n",read_data);
            return -1;
        }
    }

    // Truly and Booyi thresh
    fts_i2c_read_reg(client, FTS_REG_VENDOR_ID, &vendor_id);
    /* [liliwen start] Change TP threshold, +-32 -> +-16 */
    if (vendor_id == FTS_VENDOR_1_ID) { // truly
        if (tp_thresh_value == 0) {
            fts_i2c_write_reg(client, 0x80, 46);
        } else if (tp_thresh_value == 1) {
            fts_i2c_write_reg(client, 0x80, 42);
        } else if (tp_thresh_value == 2) {
            fts_i2c_write_reg(client, 0x80, 38);
        }
    } else if (vendor_id == FTS_VENDOR_2_ID) { // booyi
        if (tp_thresh_value == 0) {
            fts_i2c_write_reg(client, 0x80, 44);
        } else if (tp_thresh_value == 1) {
            fts_i2c_write_reg(client, 0x80, 40);
        } else if (tp_thresh_value == 2) {
            fts_i2c_write_reg(client, 0x80, 36);
        }
    } else {
        printk("unknown vendor\n");
        return -1;
    }
    /* [liliwen end] */

    msleep(50);
    fts_i2c_read_reg(client, 0x80, &read_data);
    //TPD_ERR("0x80=%d\n", read_data);

    return 0;
}
#endif //TPLINK_CHANGE_TP_THRESHOLD_ENABLE
/* [liliwen end] */

/* [liliwen start] Add for diffdata_read_enable */
//sysfs proc diffdata_read_enable
#if TPLINK_READ_DIFFDATA_ENABLE
unsigned char tplink_Comm_Base_IIC_IO(unsigned char *pWriteBuffer, int  iBytesToWrite, unsigned char *pReadBuffer, int iBytesToRead)
{
    int iRet;

    pr_err("enter TPLINK_Comm_Base_IIC_IO\n");
    iRet = fts_i2c_read(fts_i2c_client, pWriteBuffer, iBytesToWrite, pReadBuffer, iBytesToRead);

    if(iRet >= 0)
        return (ERROR_CODE_OK);
    else
        return (ERROR_CODE_COMM_ERROR);
}

static unsigned char tplink_ReadRawData(unsigned char Freq, unsigned char LineNum, int ByteNum, int *pRevBuffer)
{
    unsigned char ReCode = ERROR_CODE_COMM_ERROR;
    unsigned char I2C_wBuffer[3] = {0};
    unsigned char pReadData[ByteNum];
    int i, iReadNum, j = 0;
    unsigned short BytesNumInTestMode1 = 0;

    pr_err("Enter ReadRawData\n");
    iReadNum = ByteNum / 342;

    if (0 != (ByteNum % 342)) iReadNum++;

    if (ByteNum <= 342)
    {
        BytesNumInTestMode1 = ByteNum;
    }
    else
    {
        BytesNumInTestMode1 = 342;
    }
    //***********************************************************Read raw data in test mode1
    I2C_wBuffer[0] = 0x36;
    msleep(10);
    ReCode = tplink_Comm_Base_IIC_IO(I2C_wBuffer, 1, pReadData, BytesNumInTestMode1);
    for (i=1; i < iReadNum; i++)
    {
        if (ReCode != ERROR_CODE_OK) break;
        if (i == iReadNum - 1)//last packet
        {
            msleep(10);
            ReCode = tplink_Comm_Base_IIC_IO(NULL, 0, pReadData+342*i, ByteNum-342*i);
        }
        else
        {
            msleep(10);
            ReCode = tplink_Comm_Base_IIC_IO(NULL, 0, pReadData+342*i, 342);
        }
    }
    pr_err("show diffdata");
    if (ReCode == ERROR_CODE_OK)
    {
        //for(i=0; i<(ByteNum>>1); i++)
        for (i = 0; i < ByteNum; i = i + 2)
        {
            pRevBuffer[j] = (pReadData[i]<<8) + (pReadData[(i)+1]);
            if (pRevBuffer[j] & 0x8000)
            {
                pRevBuffer[j] -= 0xffff + 1;
            }
            pr_err("... pRevBuffer[%d]=%d....\n", j, pRevBuffer[j]);
            j++;
        }
    }
    return ReCode;
}

ssize_t tplink_diffdata_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{

    int ret = 0;
    int x, y;
    u8 mode;
    u8 tx_num, rx_num;
    char *kernel_buf;
    ssize_t num_read_chars = 0;
    int len;

    kernel_buf = kmalloc(4096, GFP_KERNEL);
    if (kernel_buf == NULL) {
        printk("[tpd]kmalloc error\n");
        return 0;
    }
    fts_i2c_read_reg(fts_i2c_client, ic_mode_register, &mode);
    printk("[tpd]one::IC is in mode = 0x%x\n",mode);
    while(1) {
        fts_i2c_write_reg(fts_i2c_client, ic_mode_register, 0x40);
        msleep(400);
        fts_i2c_read_reg(fts_i2c_client, ic_mode_register, &mode);
        printk("[tpd]two::IC is in mode = 0x%x\n",mode);
        if(mode == 0x40)
            break;
    }
    fts_i2c_write_reg(fts_i2c_client, 0x06, 0x01);//read diff data
    fts_i2c_write_reg(fts_i2c_client, 0x01, 0xAA);
    msleep(100);
    fts_i2c_read_reg(fts_i2c_client, 0x02, &tx_num);
    fts_i2c_read_reg(fts_i2c_client, 0x03, &rx_num);
    printk("[tpd]tx_num=%d,rx_num=%d\n", tx_num, rx_num);
    if((tx_num != TX_NUM_MAX) || (rx_num != RX_NUM_MAX))
    {
        fts_i2c_write_reg(fts_i2c_client, ic_mode_register, 0x00);
        num_read_chars += sprintf( &( kernel_buf[num_read_chars] ), "%s" , "tx or rx number is error\n" );
        ret = simple_read_from_buffer(user_buf, count, ppos, kernel_buf, strlen(kernel_buf));
        return ret;
    }
    fts_i2c_read_reg(fts_i2c_client, ic_mode_register, &mode);
    mode |= 0x80;
    fts_i2c_write_reg(fts_i2c_client, ic_mode_register, 0xC0);
    while(1) {
        fts_i2c_read_reg(fts_i2c_client, ic_mode_register, &mode);
        msleep(200);
        if ((mode>>7) == 0)
            break;
    }

    len = tx_num * rx_num * 2;
    pr_err("...DiffDATA...\n");
    memset(m_iTempRawData, 0, sizeof(m_iTempRawData));
    tplink_ReadRawData(0, 0xAD, tx_num * rx_num * 2, m_iTempRawData);

    len = 0;
    for (x = 0; x < TX_NUM_MAX; x++) {
        num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "\n[%3d]", x);
        for (y = 0; y < RX_NUM_MAX; y++) {
            num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "%3d ", m_iTempRawData[len]);
            len++;
        }
    }
    printk("[tpd]num_read_chars = %zd , count = %zd\n", num_read_chars, count);
    num_read_chars += sprintf( &( kernel_buf[num_read_chars] ), "%s" , "\r\n" );
    ret = simple_read_from_buffer(user_buf, count, ppos, kernel_buf, strlen(kernel_buf));
    fts_i2c_write_reg(fts_i2c_client, ic_mode_register, 0x00);
    return ret;
}
#endif
/* [liliwen end] */
