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

#ifndef _EMMC_RPMB_H
#define _EMMC_RPMB_H

#include <linux/mmc/ioctl.h>
#include <linux/mmc/card.h>

extern struct msdc_host *mtk_msdc_host[];

/************************************************************************
 *
 * RPMB IOCTL interface.
 *
 ***********************************************************************/
#define RPMB_IOCTL_PROGRAM_KEY  1
#define RPMB_IOCTL_WRITE_DATA   3
#define RPMB_IOCTL_READ_DATA    4

#if (defined(CONFIG_MICROTRUST_TEE_SUPPORT))

#define RPMB_MULTI_BLOCK_ACCESS 1

#if RPMB_MULTI_BLOCK_ACCESS
#define MAX_RPMB_TRANSFER_BLK 16
/* 8KB(16blks) per requests */
#define MAX_RPMB_REQUEST_SIZE (512*MAX_RPMB_TRANSFER_BLK)
#else
#define MAX_RPMB_TRANSFER_BLK 1
/* 512B(1blks) per requests */
#define MAX_RPMB_REQUEST_SIZE (512*MAX_RPMB_TRANSFER_BLK)
#endif

#define RPMB_IOCTL_SOTER_WRITE_DATA   5
#define RPMB_IOCTL_SOTER_READ_DATA    6
#define RPMB_IOCTL_SOTER_GET_CNT      7
#define RPMB_IOCTL_SOTER_GET_WR_SIZE      8

struct rpmb_infor {
	unsigned int size;
	unsigned char *data_frame;
};
#endif

/* [liguanxiong start] Add for rongcard tee setup */
#ifdef CONFIG_RSEE
#define 	RSEE_RPMB_IOCTL_BASE		200
#define 	RSEE_RPMB_IOCTL(cmd)		(RSEE_RPMB_IOCTL_BASE + cmd)
#define 	RSEE_RPMB_IOCTL_GET_WC	  RSEE_RPMB_IOCTL(0)
#define 	RSEE_RPMB_IOCTL_WRITE_DATA  RSEE_RPMB_IOCTL(1)
#define 	RSEE_RPMB_IOCTL_READ_DATA   RSEE_RPMB_IOCTL(2)
#endif
/* [liguanxiong end] */

struct rpmb_ioc_param {
	unsigned char *key;
	unsigned char *data;
	unsigned int  data_len;
	unsigned short addr;
	unsigned char *hmac;
	unsigned int hmac_len;
};
/***********************************************************************/


#define RPMB_SZ_STUFF 196
#define RPMB_SZ_MAC   32
#define RPMB_SZ_DATA  256
#define RPMB_SZ_NONCE 16

struct s_rpmb {
	unsigned char stuff[RPMB_SZ_STUFF];
	unsigned char mac[RPMB_SZ_MAC];
	unsigned char data[RPMB_SZ_DATA];
	unsigned char nonce[RPMB_SZ_NONCE];
	unsigned int write_counter;
	unsigned short address;
	unsigned short block_count;
	unsigned short result;
	unsigned short request;
};

enum {
	RPMB_SUCCESS = 0,
	RPMB_HMAC_ERROR,
	RPMB_RESULT_ERROR,
	RPMB_WC_ERROR,
	RPMB_NONCE_ERROR,
	RPMB_ALLOC_ERROR,
	RPMB_TRANSFER_NOT_COMPLETE,
};

#define RPMB_PROGRAM_KEY       1       /* Program RPMB Authentication Key */
#define RPMB_GET_WRITE_COUNTER 2       /* Read RPMB write counter */
#define RPMB_WRITE_DATA		   3	   /* Write data to RPMB partition */
#define RPMB_READ_DATA         4       /* Read data from RPMB partition */
#define RPMB_RESULT_READ       5       /* Read result request */
#define RPMB_REQ               1       /* RPMB request mark */
#define RPMB_RESP              (1 << 1)/* RPMB response mark */
#define RPMB_AVAILABLE_SECTORS 8       /* 4K page size */

#define RPMB_TYPE_BEG          510
#define RPMB_RES_BEG           508
#define RPMB_BLKS_BEG          506
#define RPMB_ADDR_BEG          504
#define RPMB_WCOUNTER_BEG      500

#define RPMB_NONCE_BEG         484
#define RPMB_DATA_BEG          228
#define RPMB_MAC_BEG           196

struct emmc_rpmb_req {
	__u16 type;                     /* RPMB request type */
	__u16 *result;                  /* response or request result */
	__u16 blk_cnt;                  /* Number of blocks(half sector 256B) */
	__u16 addr;                     /* data address */
	__u32 *wc;                      /* write counter */
	__u8 *nonce;                    /* Ramdom number */
	__u8 *data;                     /* Buffer of the user data */
	__u8 *mac;                      /* Message Authentication Code */
	__u8 *data_frame;
};


int mmc_rpmb_set_key(struct mmc_card *card, void *key);
int mmc_rpmb_read(struct mmc_card *card, u8 *buf, u16 blk, u16 cnt, void *key);
int mmc_rpmb_write(struct mmc_card *card, u8 *buf, u16 blk, u16 cnt, void *key);

extern void emmc_rpmb_set_host(void *mmc_host);


#endif
