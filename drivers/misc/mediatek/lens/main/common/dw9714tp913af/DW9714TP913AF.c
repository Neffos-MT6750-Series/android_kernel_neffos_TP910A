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

/*
 * DW9714TP913AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
// [linyimin] for async power down
#include <linux/kthread.h>

#include "lens_info.h"

#if defined(CONFIG_MTK_PMIC_CHIP_MT6353) && defined(CONFIG_ARCH_MT6755)
#include "pmic_api_buck.h"
#include "pmic_api_ldo.h"
#endif


#define AF_DRVNAME "DW9714TP913AF_DRV"
#define AF_I2C_SLAVE_ADDR        0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif


static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;
static int g_advance_mode = 0;
// [linyimin] for async power down
static struct task_struct *release_af_task;

static int s4AF_ReadReg(unsigned short *a_pu2Result)
{
	int i4RetValue = 0;
	char pBuff[2];

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, pBuff, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C read failed!!\n");
		return -1;
	}

	*a_pu2Result = (((u16) pBuff[0]) << 4) + (pBuff[1] >> 4);

	return 0;
}

static int s4AF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[2] = { (char)(a_u2Data >> 4), (char)((a_u2Data & 0xF) << 4) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

	return 0;
}

static int s4AF_Adv_ReadReg(unsigned short *a_pu2Result)
{

	int i4RetValue = 0;
	char pBuff[2];
	char VCMMSB[1] = { (char)(0x03) };
	char VCMLSB[1] = { (char)(0x04) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	/* Read MSB */
	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, VCMMSB, 1);

	if (i4RetValue < 0) {
		LOG_INF("I2C read MSB send failed!!\n");
		return -1;
	}

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, &pBuff[1], 1);

	if (i4RetValue < 0) {
		LOG_INF("I2C read MSB recv failed!!\n");
		return -1;
	}

	/* Read LSB */
	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, VCMLSB, 1);

	if (i4RetValue < 0) {
		LOG_INF("I2C read LSB send failed!!\n");
		return -1;
	}

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, &pBuff[0], 1);

	if (i4RetValue < 0) {
		LOG_INF("I2C read LSB recv failed!!\n");
		return -1;
	}

	*a_pu2Result = ((u16) pBuff[0] + (u16) (pBuff[1] << 8));

	return 0;
}

static int s4AF_Adv_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	/* 0x03[1:0] VCM MSB data */
	/* 0x04[7:0] VCM LSB data */
	char VCMMSB[2] = { (char)(0x03), (char)((a_u2Data >> 8) & 0x03) };
	char VCMLSB[2] = { (char)(0x04), (char)(a_u2Data & 0xFF) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, VCMMSB, 2);
	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, VCMLSB, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}

	return 0;
}

static int s4AF_Write_Reg(u16 addr, u16 para)
{
	int i4RetValue = 0;

	char puSendCmd[2] = { (char)(addr & 0xFF), (char)(para & 0xFF) };

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);

	if (i4RetValue < 0) {
		LOG_INF("s4AF_Write_Reg: I2C send failed!!\n");
		return -1;
	}

	return 0;
}

static void DW9714TP913AF_Adv_init(void)
{
	int i4RetValue = 0;
	i4RetValue = s4AF_Write_Reg(0xED, 0xAB);
	if (i4RetValue < 0) {
		g_advance_mode = 0;
		return;
	}
	g_advance_mode = 1;

	s4AF_Write_Reg(0x02, 0x01);
	s4AF_Write_Reg(0x02, 0x00);
	udelay(1000);
	s4AF_Write_Reg(0x06, 0x84);
	s4AF_Write_Reg(0x07, 0x01);
	s4AF_Write_Reg(0x08, 0x50);

}

static void DW9714TP913AF_Adv_powerOff(void)
{
	#if 0
	int i4RetValue = 0;
	i4RetValue = s4AF_Write_Reg(0xDF, 0x5B);
	if (i4RetValue < 0) {
		s4AF_Write_Reg(0x02, 0x01);
		return;
	}
	s4AF_Write_Reg(0x80, 0x00);
	#endif
	s4AF_Write_Reg(0x02, 0x01);
}

static int DW9714TP913AF_powerOff(void)
{

        int i4RetValue = 0;

        char puSendCmd[2] = { (char)(0x80), (char)(0x00) };

        g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

        g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

        i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);

        if (i4RetValue < 0) {
                LOG_INF("I2C send failed!!\n");
                return -1;
        }

        return 0;
}

static inline int getAFInfo(__user struct stAF_MotorInfo *pstMotorInfo)
{
	struct stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(struct stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;

	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

	if (*g_pAF_Opened == 1) {
		unsigned short InitPos;
		pmic_ldo_vldo28_sw_en(1);
		mdelay(3);
		DW9714TP913AF_Adv_init();
		if (g_advance_mode == 1)
		{
			ret = s4AF_Adv_ReadReg(&InitPos);
		} else {
			ret = s4AF_ReadReg(&InitPos);
		}

		if (ret == 0) {
			LOG_INF("Init Pos %6d\n", InitPos);

			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);

		} else {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}

	if (g_u4CurrPosition == a_u4Position)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	/* LOG_INF("move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); */

	if (g_advance_mode == 1) {
		if (s4AF_Adv_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
			spin_unlock(g_pAF_SpinLock);
		} else {
			LOG_INF("set I2C failed when moving the motor\n");
			ret = -1;
		}
	} else {
		if (s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
			spin_unlock(g_pAF_SpinLock);
		} else {
			LOG_INF("set I2C failed when moving the motor\n");
			ret = -1;
		}
	}

	return ret;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long DW9714TP913AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user struct stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}


static inline int releaseAF(void)
{
	long a_u4Position;

	for(a_u4Position = g_u4TargetPosition; a_u4Position > 0; a_u4Position -= 20)
	{
		LOG_INF("Move to %ld", a_u4Position);
		moveAF(a_u4Position);
		mdelay(5);
	}

	moveAF(0);
	mdelay(5);

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = 0;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* [linyimin start] power off thread */
static int dw9714tp913_release_af_kthread(void* data)
{
	LOG_INF("release_af_kthread + \n");
	releaseAF();
	if (g_advance_mode == 1)
	{
		DW9714TP913AF_Adv_powerOff();
	} else {
		DW9714TP913AF_powerOff();
	}
	pmic_ldo_vldo28_sw_en(0);
	LOG_INF("release_af_kthread - \n");
	return 0;
}
/* [linyimin end] power off thread */

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int DW9714TP913AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");

	if (*g_pAF_Opened == 2) {
		LOG_INF("Wait\n");
		// [linyimin] power off thread
		release_af_task = kthread_create(dw9714tp913_release_af_kthread, NULL,"dw9714tp913_release_af");
		wake_up_process(release_af_task);

		LOG_INF("Wait -\n");
	}

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

	LOG_INF("End\n");

	return 0;
}

int DW9714TP913AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pstAF_I2Cclient->timing = 400;	
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
	g_advance_mode = 0;

	return 1;
}
