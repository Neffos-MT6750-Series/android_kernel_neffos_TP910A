/* Himax Android Driver Sample Code for MTK kernel 3.18 platform
*
* Copyright (C) 2017 Himax Corporation.
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

#include "himax_platform.h"
#include "himax_common.h"
#include "himax_ic.h"

int i2c_error_count = 0;
int irq_enable_count = 0;
extern struct i2c_client *i2c_client_point;
static int irq_flag_true = 0;//lichenggang add for irq judge
DEFINE_MUTEX(hx_wr_access);
extern atomic_t close_incall;//[lichenggang start] modify for psensor contorl tp when call

MODULE_DEVICE_TABLE(of, himax_match_table);
struct of_device_id himax_match_table[] =
{
    {.compatible = "mediatek,cap_touch" },
    {},
};

static int himax_tpd_int_gpio = 5;
unsigned int himax_touch_irq = 0;
unsigned int himax_tpd_rst_gpio_number = 10;
unsigned int himax_tpd_int_gpio_number = 1;

u8 *gpDMABuf_va = NULL;
u8 *gpDMABuf_pa = NULL;

//Custom set some config
static int hx_panel_coords[4] = {0,720,0,1440};
static int hx_display_coords[4] = {0,720,0,1440};
static int report_type = PROTOCOL_TYPE_B;

static struct task_struct *touch_thread = NULL;
struct i2c_client *i2c_client_point = NULL;
static int tpd_flag = 0;

extern struct himax_ic_data* ic_data;
extern struct himax_ts_data *private_ts;
extern void himax_ts_work(struct himax_ts_data *ts);
extern enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer);

#ifdef HX_TP_PROC_DIAG
extern uint8_t getDiagCommand(void);
#endif

int himax_dev_set(struct himax_ts_data *ts)
{
    ts->input_dev = tpd->dev;
    return NO_ERR;
}

int himax_input_register_device(struct input_dev *input_dev)
{
    return 0;
}

extern void himax_log_touch_int_devation(int touched);
int himax_parse_dt(struct himax_ts_data *ts,struct himax_i2c_platform_data *pdata)
{
    struct device_node *dt = ts->client->dev.of_node;
    struct i2c_client *client = ts->client;

    if (dt)
    {
        const struct of_device_id *match;
        match = of_match_device(of_match_ptr(himax_match_table), &client->dev);
        if (!match)
        {
            TPD_DMESG("[Himax]Error: No device match found\n");
            return -ENODEV;
        }
    }

    /* It will be a non-zero and non-one value for MTK PINCTRL API */
    himax_tpd_rst_gpio_number = 10;
    pdata->gpio_reset	= himax_tpd_rst_gpio_number;
    pdata->gpio_irq		= himax_tpd_int_gpio_number;
    I("%s: int : %2.2x\n",__func__,pdata->gpio_irq);
    I("%s: rst : %2.2x\n",__func__,pdata->gpio_reset);

    //Set device tree data
    //Set panel coordinates
    pdata->abs_x_min = hx_panel_coords[0], pdata->abs_x_max = hx_panel_coords[1];
    pdata->abs_y_min = hx_panel_coords[2], pdata->abs_y_max = hx_panel_coords[3];
    I(" %s:panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
      pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);

    //Set display coordinates
    pdata->screenWidth  = hx_display_coords[1];
    pdata->screenHeight = hx_display_coords[3];
    I(" %s:display-coords = (%d, %d)", __func__, pdata->screenWidth,
      pdata->screenHeight);
    //report type
    pdata->protocol_type = report_type;
    return 0;
}

#ifdef MTK_I2C_DMA
int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
    int ret=0;
    s32 retry = 0;
    u8 buffer[1];

    struct i2c_msg msg[] =
    {
        {
            .addr = (client->addr & I2C_MASK_FLAG),
            .flags = 0,
            .buf = buffer,
            .len = 1,
            .timing = 400
        },
        {
            .addr = (client->addr & I2C_MASK_FLAG),
            .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = I2C_M_RD,
            .buf = gpDMABuf_pa,
            .len = length,
            .timing = 400
        },
    };
    mutex_lock(&hx_wr_access);
    buffer[0] = command;

    if (data == NULL)
    {
        mutex_unlock(&hx_wr_access);
        return -1;
    }
    for (retry = 0; retry < toRetry; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg[0], 2);
        if (ret < 0)
        {
            continue;
        }
        memcpy(data, gpDMABuf_va, length);
        mutex_unlock(&hx_wr_access);
        return 0;
    }
    E("Dma I2C Read Error: %d byte(s), err-code: %d", length, ret);
    i2c_error_count = toRetry;
    mutex_unlock(&hx_wr_access);
    return ret;
}

int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *buf, uint8_t len, uint8_t toRetry)
{
    int rc=0,retry=0;
    u8 *pWriteData = gpDMABuf_va;

    struct i2c_msg msg[]=
    {
        {
            .addr = (client->addr & I2C_MASK_FLAG),
            .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = 0,
            .buf = gpDMABuf_pa,
            .len = len+1,
            .timing = 400
        },
    };

    mutex_lock(&hx_wr_access);
    if(!pWriteData)
    {
        E("dma_alloc_coherent failed!\n");
        mutex_unlock(&hx_wr_access);
        return -1;
    }

    gpDMABuf_va[0] = command;

    memcpy(gpDMABuf_va+1, buf, len);

    for (retry = 0; retry < toRetry; ++retry)
    {
        rc = i2c_transfer(client->adapter, &msg[0], 1);
        if (rc < 0)
        {
            continue;
        }
        mutex_unlock(&hx_wr_access);
        return 0;
    }

    E("Dma I2C master write Error: %d byte(s), err-code: %d", len, rc);
    i2c_error_count = toRetry;
    mutex_unlock(&hx_wr_access);
    return rc;
}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
    return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *buf, uint8_t len, uint8_t toRetry)
{
    int rc=0, retry=0;
    u8 *pWriteData = gpDMABuf_va;

    struct i2c_msg msg[] =
    {
        {
            .addr = (client->addr & I2C_MASK_FLAG),
            .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = 0,
            .buf = gpDMABuf_pa,
            .len = len,
            .timing = 400
        },
    };

    mutex_lock(&hx_wr_access);
    if(!pWriteData)
    {
        E("dma_alloc_coherent failed!\n");
        mutex_unlock(&hx_wr_access);
        return -1;
    }

    memcpy(gpDMABuf_va, buf, len);
    for (retry = 0; retry < toRetry; ++retry)
    {
        rc = i2c_transfer(client->adapter, &msg[0], 1);
        if (rc < 0)
        {
            continue;
        }
        mutex_unlock(&hx_wr_access);
        return 0;
    }
    E("Dma I2C master write Error: %d byte(s), err-code: %d", len, rc);
    i2c_error_count = toRetry;
    mutex_unlock(&hx_wr_access);
    return rc;
}

#else
int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
    int retry;
    struct i2c_msg msg[] =
    {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &command,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = data,
        }
    };

    for (retry = 0; retry < toRetry; retry++)
    {
        if (i2c_transfer(client->adapter, msg, 2) == 2)
            break;
        msleep(10);
    }
    if (retry == toRetry)
    {
        E("%s: i2c_read_block retry over %d\n",
          __func__, toRetry);
        i2c_error_count = toRetry;
        return -EIO;
    }
    return 0;

}

int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
    int retry/*, loop_i*/;
    uint8_t buf[length + 1];

    struct i2c_msg msg[] =
    {
        {
            .addr = client->addr,
            .flags = 0,
            .len = length + 1,
            .buf = buf,
        }
    };

    buf[0] = command;
    memcpy(buf+1, data, length);

    for (retry = 0; retry < toRetry; retry++)
    {
        if (i2c_transfer(client->adapter, msg, 1) == 1)
            break;
        msleep(10);
    }

    if (retry == toRetry)
    {
        E("%s: i2c_write_block retry over %d\n",
          __func__, toRetry);
        i2c_error_count = toRetry;
        return -EIO;
    }
    return 0;

}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
    return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry)
{
    int retry/*, loop_i*/;
    uint8_t buf[length];

    struct i2c_msg msg[] =
    {
        {
            .addr = client->addr,
            .flags = 0,
            .len = length,
            .buf = buf,
        }
    };

    memcpy(buf, data, length);

    for (retry = 0; retry < toRetry; retry++)
    {
        if (i2c_transfer(client->adapter, msg, 1) == 1)
            break;
        msleep(10);
    }

    if (retry == toRetry)
    {
        E("%s: i2c_write_block retry over %d\n",
          __func__, toRetry);
        i2c_error_count = toRetry;
        return -EIO;
    }
    return 0;
}
#endif

uint8_t himax_int_gpio_read(int pinnum)
{
    return  gpio_get_value(himax_tpd_int_gpio);
}

void himax_int_enable(int irqnum, int enable)
{
    if (enable == 1 && irq_enable_count == 0)
    {
        irq_enable_count=1;
#ifdef CONFIG_OF_TOUCH
        enable_irq(irqnum);
#else
        mt_eint_unmask(irqnum);
#endif
        private_ts->irq_enabled = 1;
    }
    else if (enable == 0 && irq_enable_count == 1)
    {
        irq_enable_count=0;
#ifdef CONFIG_OF_TOUCH
        disable_irq_nosync(irqnum);
#else
        mt_eint_mask(irqnum);
#endif
        private_ts->irq_enabled = 0;
    }
    I("irq_enable_count = %d\n", irq_enable_count);
}

#ifdef HX_RST_PIN_FUNC
void himax_rst_gpio_set(int pinnum, uint8_t value)
{
    if(value)
        tpd_gpio_output(himax_tpd_rst_gpio_number, 1);
    else
        tpd_gpio_output(himax_tpd_rst_gpio_number, 0);
}
#endif


int himax_gpio_power_config(struct i2c_client *client,struct himax_i2c_platform_data *pdata)
{
    int error=0;

    error = regulator_enable(tpd->reg);
    if (error != 0)
        TPD_DMESG("Failed to enable reg-vgp6: %d\n", error);
    msleep(100);

#ifdef HX_RST_PIN_FUNC
    tpd_gpio_output(himax_tpd_rst_gpio_number, 1);
    mdelay(15);//msleep(20)
    tpd_gpio_output(himax_tpd_rst_gpio_number, 0);
    mdelay(15);//msleep(20)
    tpd_gpio_output(himax_tpd_rst_gpio_number, 1);
#endif

    TPD_DMESG("mtk_tpd: himax reset over \n");

    /* set INT mode */

    tpd_gpio_as_int(himax_tpd_int_gpio_number);
    return 0;
}

static void himax_ts_work_func(struct work_struct *work)
{
    struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work);

    himax_ts_work(ts);
}

#ifdef CONFIG_OF_TOUCH
irqreturn_t tpd_eint_interrupt_handler(int irq, void *desc)
{
    tpd_flag = 1;
    irq_flag_true = 1;//lichenggang add for irq judge
    /* enter EINT handler disable INT, make sure INT is disable when handle touch event including top/bottom half */
    /* use _nosync to avoid deadlock */
    wake_up_interruptible(&waiter);
    return IRQ_HANDLED;
}
#else
void tpd_eint_interrupt_handler(void)
{
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
}
#endif

static void himax_ts_isr_func(void)
{
	struct himax_ts_data *ts = private_ts;
	/* [lichenggang start]add for irq judge*/
	if(irq_flag_true){
		himax_ts_work(ts);
		irq_flag_true = 0;
	}else{
		return;
	}
	/* [lichenggang end] */

}
extern atomic_t interrupt_counter;/* [lichenggang] add for tp report rate node */

static int touch_event_handler(void *ptr)
{
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);

    do
    {
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter,tpd_flag!=0);

        tpd_flag = 0;
        set_current_state(TASK_RUNNING);
        atomic_add(1, &interrupt_counter); /* [lichenggang] add tp report rate node */
        if (private_ts->debug_log_level & BIT(2))
        {
            himax_log_touch_int_devation(HX_FINGER_ON);
        }
        if (atomic_read(&close_incall) == 1)//[lichenggang] do not report touch data when incall and blank screen
            continue;

        himax_ts_isr_func();

        if(private_ts->debug_log_level & BIT(2))
        {
            himax_log_touch_int_devation(HX_FINGER_LEAVE);
        }
    }
    while(!kthread_should_stop());
    return 0;
}

int himax_int_register_trigger(struct i2c_client *client)
{
    int ret = 0;

    if(ic_data->HX_INT_IS_EDGE)
    {
        I("%s edge triiger falling\n ",__func__);
        ret = request_irq(client->irq, tpd_eint_interrupt_handler, IRQF_TRIGGER_FALLING, "TOUCH_PANEL-eint", NULL);/* [lichenggang] modify Interruption application */
        if(ret > 0)
        {
            ret = -1;
            E("tpd request_irq IRQ LINE NOT AVAILABLE\n");
        }
    }
    else
    {
        I("%s level trigger low\n ",__func__);
        ret = request_irq(client->irq, tpd_eint_interrupt_handler, IRQF_TRIGGER_LOW, "TOUCH_PANEL-eint", NULL);
        if(ret > 0)
        {
            ret = -1;
            E("tpd request_irq IRQ LINE NOT AVAILABLE\n");
        }
    }

    return ret;
}

int himax_int_en_set(struct i2c_client *client)
{
    int ret = NO_ERR;
    himax_int_enable(client->irq,1);
    return ret;
}

int himax_ts_register_interrupt(struct i2c_client *client)
{
    struct himax_ts_data *ts = i2c_get_clientdata(client);
    int ret = 0;
    struct device_node *node = NULL;
    u32 ints[2] = {0,0};

    node = of_find_matching_node(node, touch_of_match);
    if (node)
    {
        of_property_read_u32_array(node,"debounce", ints, ARRAY_SIZE(ints));
        gpio_set_debounce(ints[0], ints[1]);
        himax_touch_irq = irq_of_parse_and_map(node, 0);
        I("himax_touch_irq=%ud \n",himax_touch_irq);
        client->irq = himax_touch_irq;
        ts->client->irq = himax_touch_irq;
    }
    else
    {
        I("[%s] tpd request_irq can not find touch eint device node!.", __func__);
    }

    ts->irq_enabled = 0;
    ts->use_irq = 0;
    //Work functon
    if (client->irq)  /*INT mode*/
    {
        ts->use_irq = 1;
        ret = himax_int_register_trigger(client);
        ts->irq_enabled = 1;
        irq_enable_count = 1;
#ifdef HX_SMART_WAKEUP
        irq_set_irq_wake(client->irq, 1);
#endif
        touch_thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
        if (IS_ERR(touch_thread))
        {
            ret = PTR_ERR(touch_thread);
            E(" Failed to create kernel thread: %d\n", ret);
            return ret;
        }
    }
    else
    {
        I("%s: client->irq is empty, use polling mode.\n", __func__);
    }

    if (!ts->use_irq)  /*if use polling mode need to disable HX_ESD_RECOVERY function*/
    {
        ts->himax_wq = create_singlethread_workqueue("himax_touch");
        INIT_WORK(&ts->work, himax_ts_work_func);
        hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        ts->timer.function = himax_ts_timer_func;
        hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        I("%s: polling mode enabled\n", __func__);
    }
    tpd_load_status = 1;
	enable_irq(himax_touch_irq);
    return 0;
}

int himax_common_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
#if defined(MTK_I2C_DMA)
    client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
    gpDMABuf_va = (u8 *)dma_alloc_coherent(&client->dev, 4096, (dma_addr_t *)&gpDMABuf_pa, GFP_KERNEL);
    if(!gpDMABuf_va)
    {
        E("Allocate DMA I2C Buffer failed\n");
        ret = -ENODEV;
        goto err_alloc_MTK_DMA_failed;
    }
    memset(gpDMABuf_va, 0, 4096);
#endif

    i2c_client_point = client;

    ret = himax_chip_common_probe(client,id);

#if defined(MTK_I2C_DMA)
    if(ret)
    {
        if(gpDMABuf_va)
        {
            dma_free_coherent(&client->dev, 4096, gpDMABuf_va, (dma_addr_t)gpDMABuf_pa);
            gpDMABuf_va = NULL;
            gpDMABuf_pa = NULL;
        }
    }
err_alloc_MTK_DMA_failed:
#endif
    return ret;
}


int himax_common_remove(struct i2c_client *client)
{
    int ret = 0;
    himax_chip_common_remove(client);
    if(gpDMABuf_va)
    {
        dma_free_coherent(&client->dev, 4096, gpDMABuf_va, (dma_addr_t)gpDMABuf_pa);
        gpDMABuf_va = NULL;
        gpDMABuf_pa = NULL;
    }
    return ret;
}

static void himax_common_suspend(struct device *dev)
{
    struct himax_ts_data *ts = dev_get_drvdata(&i2c_client_point->dev);
    I("%s: enter \n", __func__);
    himax_chip_common_suspend(ts);
    I("%s: END \n", __func__);
    return ;
}
static void himax_common_resume(struct device *dev)
{
    struct himax_ts_data *ts = dev_get_drvdata(&i2c_client_point->dev);

    I("%s: enter \n", __func__);
    irq_flag_true = 0;
#if 0 /* [lichenggang] Cut tp reset */
    tpd_gpio_output(himax_tpd_rst_gpio_number, 1);
    mdelay(15);//msleep(20)
    tpd_gpio_output(himax_tpd_rst_gpio_number, 0);
    mdelay(15);//msleep(20)
    tpd_gpio_output(himax_tpd_rst_gpio_number, 1);
#endif
    himax_chip_common_resume(ts);

    I("%s: END \n", __func__);
    return ;
}


#if defined(CONFIG_FB)
int fb_notifier_callback(struct notifier_block *self,
                         unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    struct himax_ts_data *ts=
        container_of(self, struct himax_ts_data, fb_notif);

    I(" %s\n", __func__);
    if (evdata && evdata->data && event == FB_EVENT_BLANK && ts &&
            ts->client)
    {
        blank = evdata->data;
        switch (*blank)
        {
        case FB_BLANK_UNBLANK:
            himax_common_resume(&ts->client->dev);
            break;

        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
            himax_common_suspend(&ts->client->dev);
            break;
        }
    }

    return 0;
}
#endif

static int himax_common_detect (struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE);
    return 0;
}

static const struct i2c_device_id himax_common_ts_id[] =
{
    {HIMAX_common_NAME, 0 },
    {}
};

static struct i2c_driver tpd_i2c_driver =
{
    .probe = himax_common_probe,
    .remove = himax_common_remove,
    .detect = himax_common_detect,
    .driver	= {
        .name = HIMAX_common_NAME,
        .of_match_table = of_match_ptr(himax_match_table),
    },
    .id_table = himax_common_ts_id,
    .address_list = (const unsigned short *) forces,
};

static int himax_common_local_init(void)
{
    int retval;

    I("[Himax] Himax_ts I2C Touchscreen Driver local init\n");

    tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
    retval = regulator_set_voltage(tpd->reg, 2800000, 2800000);
    if (retval != 0)
    {
        TPD_DMESG("Failed to set reg-vgp6 voltage: %d\n", retval);
        return -1;
    }

    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        I("unable to add i2c driver.\n");
        return -1;
    }

    I("end %s, %d\n", __FUNCTION__, __LINE__);
    tpd_type_cap = 1;

    return 0;
}

static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = HIMAX_common_NAME,
    .tpd_local_init = himax_common_local_init,
    .suspend = himax_common_suspend,
    .resume = himax_common_resume,
    .tpd_have_button = 0,
};

static void __init himax_common_init_async(void *unused, async_cookie_t cookie)
{
    I("%s:Enter \n", __func__);
    tpd_get_dts_info();
    if(tpd_driver_add(&tpd_device_driver)<0)
        I("Failed to add Driver!\n");
}

static int __init himax_common_init(void)
{
    I("Himax_common touch panel driver init\n");
    async_schedule(himax_common_init_async, NULL);
    return 0;
}

static void __exit himax_common_exit(void)
{
    tpd_driver_remove(&tpd_device_driver);
}

module_init(himax_common_init);
module_exit(himax_common_exit);

MODULE_DESCRIPTION("Himax_common driver");
MODULE_LICENSE("GPL");

