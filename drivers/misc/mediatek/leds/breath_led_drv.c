/*
 * drivers/leds/leds-mt65xx.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 * mt65xx leds driver
 *
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_hw.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <asm/setup.h>
#include <linux/version.h>

#define ISINK_BREATH_MODE 1
#define ISINK_PWM_MODE 0
#define ISINK_3   3

//[wanglei start] Adjust default breathing speed
#define DEFAULT_BREATH1_TR1_SEL		0x01
#define DEFAULT_BREATH1_TR2_SEL		0x03
#define DEFAULT_BREATH1_TF1_SEL		0x01
#define DEFAULT_BREATH1_TF2_SEL		0x03
#define DEFAULT_BREATH1_TON_SEL		0x01
#define DEFAULT_BREATH1_TOFF_SEL	0x02
//[wanglei end]
#define DEFAULT_DIM1_DUTY			31  //default max duty, (31+1)/32
#define DEFAULT_DIM1_FSEL			1

#define GREEN_EN           PMIC_ISINK_CH0_EN
#define GREEN_MODE         PMIC_ISINK_CH0_MODE
#define GREEN_STEP         PMIC_ISINK_CH0_STEP
#define GREEN_TR1_SEL      PMIC_ISINK_BREATH0_TR1_SEL
#define GREEN_TR2_SEL      PMIC_ISINK_BREATH0_TR2_SEL
#define GREEN_TF1_SEL      PMIC_ISINK_BREATH0_TF1_SEL
#define GREEN_TF2_SEL      PMIC_ISINK_BREATH0_TF2_SEL
#define GREEN_TON_SEL      PMIC_ISINK_BREATH0_TON_SEL
#define GREEN_TOFF_SEL     PMIC_ISINK_BREATH0_TOFF_SEL
#define GREEN_DUTY         PMIC_ISINK_DIM0_DUTY
#define GREEN_FSEL         PMIC_ISINK_DIM0_FSEL

static int  breath_leds_setup(void)
{
	printk("breath_leds_start\n");
	printk("PMIC_ISINK_CH1_MODE = 0x%x\n" ,PMIC_ISINK_CH1_MODE);
	printk("PMIC_ISINK_CH1_STEP = 0x%x\n" ,PMIC_ISINK_CH1_STEP);
	printk("PMIC_ISINK_BREATH1_TR1_SEL = 0x%x\n" ,PMIC_ISINK_BREATH1_TR1_SEL);
	printk("PMIC_ISINK_BREATH1_TR2_SEL = 0x%x\n" ,PMIC_ISINK_BREATH1_TR2_SEL);
	printk("PMIC_ISINK_BREATH1_TF1_SEL = 0x%x\n" ,PMIC_ISINK_BREATH1_TF1_SEL);
	printk("PMIC_ISINK_BREATH1_TF2_SEL = 0x%x\n" ,PMIC_ISINK_BREATH1_TF2_SEL);
	printk("PMIC_ISINK_BREATH1_TON_SEL = 0x%x\n" ,PMIC_ISINK_BREATH1_TON_SEL);
	printk("PMIC_ISINK_BREATH1_TOFF_SEL = 0x%x\n" ,PMIC_ISINK_BREATH1_TOFF_SEL);
	printk("PMIC_ISINK_DIM1_DUTY = 0x%x\n" ,PMIC_ISINK_DIM1_DUTY);
	printk("PMIC_ISINK_DIM1_FSEL = 0x%x\n" ,PMIC_ISINK_DIM1_FSEL);

	pmic_set_register_value(PMIC_ISINK_CH1_MODE, ISINK_BREATH_MODE);

	pmic_set_register_value(PMIC_ISINK_CH1_STEP, ISINK_3);

	pmic_set_register_value(PMIC_ISINK_BREATH1_TR1_SEL, DEFAULT_BREATH1_TR1_SEL);

	pmic_set_register_value(PMIC_ISINK_BREATH1_TR2_SEL, DEFAULT_BREATH1_TR2_SEL);

	pmic_set_register_value(PMIC_ISINK_BREATH1_TF1_SEL, DEFAULT_BREATH1_TF1_SEL);

	pmic_set_register_value(PMIC_ISINK_BREATH1_TF2_SEL, DEFAULT_BREATH1_TF2_SEL);

	pmic_set_register_value(PMIC_ISINK_BREATH1_TON_SEL, DEFAULT_BREATH1_TON_SEL);

	pmic_set_register_value(PMIC_ISINK_BREATH1_TOFF_SEL, DEFAULT_BREATH1_TOFF_SEL);

	pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, DEFAULT_DIM1_DUTY);

	pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, DEFAULT_DIM1_FSEL);

	pmic_set_register_value(PMIC_CLK_DRV_ISINK0_CK_PDN, 0);

	pmic_set_register_value(PMIC_CLK_DRV_ISINK1_CK_PDN, 0);

	pmic_set_register_value(GREEN_MODE, ISINK_BREATH_MODE);

	pmic_set_register_value(GREEN_STEP, ISINK_3);

	pmic_set_register_value(GREEN_TR1_SEL, DEFAULT_BREATH1_TR1_SEL);

	pmic_set_register_value(GREEN_TR2_SEL, DEFAULT_BREATH1_TR2_SEL);

	pmic_set_register_value(GREEN_TF1_SEL, DEFAULT_BREATH1_TF1_SEL);

	pmic_set_register_value(GREEN_TF2_SEL, DEFAULT_BREATH1_TF2_SEL);

	pmic_set_register_value(GREEN_TON_SEL, DEFAULT_BREATH1_TON_SEL);

	pmic_set_register_value(GREEN_TOFF_SEL, DEFAULT_BREATH1_TOFF_SEL);

	pmic_set_register_value(GREEN_DUTY, DEFAULT_DIM1_DUTY);

	pmic_set_register_value(GREEN_FSEL, DEFAULT_DIM1_FSEL);
	return 0;
}

static  void breath_leds_enable(int val){
	pmic_set_register_value(PMIC_ISINK_CH1_EN, val);
}


/*-------------------------------MISC device related------------------------------------------*/
static int breath_leds_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int breath_leds_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long breath_leds_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	return 0;
}

static struct file_operations breath_leds_fops = {
	.owner = THIS_MODULE,
	.open = breath_leds_open,
	.release = breath_leds_release,
	.unlocked_ioctl = breath_leds_unlocked_ioctl,
};

static struct miscdevice breath_leds_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "breath_led",
	.fops = &breath_leds_fops,
};
/*----------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------*/

static ssize_t breath_leds_write_step(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = ISINK_3;
	int ret = 0;
	char sbuf[20];
	memset(sbuf,0,20);
	strncpy(sbuf,buf,count);
	ret = kstrtoint((const char *)sbuf, 10,&val);
	printk("SET PMIC_ISINK_CH1_STEP = 0x%x\n", val);
	pmic_set_register_value(PMIC_ISINK_CH1_STEP, (short)val);
	pmic_set_register_value(GREEN_STEP, (short)val);
	return count;
}
static ssize_t breath_leds_read_step(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", pmic_get_register_value(PMIC_ISINK_CH1_STEP));
}
static DRIVER_ATTR(step, S_IWUSR | S_IRUGO, breath_leds_read_step,	breath_leds_write_step);
/*----------------------------------------------------------------------------------------------*/

static ssize_t breath_leds_write_tr1(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = DEFAULT_BREATH1_TR1_SEL;
	int ret = 0;
	char sbuf[20];
	memset(sbuf,0,20);
	strncpy(sbuf,buf,count);
	ret = kstrtoint((const char *)sbuf, 10,&val);
	printk("SET PMIC_ISINK_BREATH1_TR1_SEL = 0x%x\n", val);
	pmic_set_register_value(PMIC_ISINK_BREATH1_TR1_SEL, (short)val);
	pmic_set_register_value(GREEN_TR1_SEL, (short)val);
	return count;
}
static ssize_t breath_leds_read_tr1(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", pmic_get_register_value(PMIC_ISINK_BREATH1_TR1_SEL));
}
static DRIVER_ATTR(tr1, S_IWUSR | S_IRUGO, breath_leds_read_tr1,	breath_leds_write_tr1);
/*----------------------------------------------------------------------------------------------*/

static ssize_t breath_leds_write_tr2(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = DEFAULT_BREATH1_TR2_SEL;
	int ret = 0;
	char sbuf[20];
	memset(sbuf,0,20);
	strncpy(sbuf,buf,count);
	ret = kstrtoint((const char *)sbuf, 10,&val);
	printk("SET PMIC_ISINK_BREATH1_TR2_SEL = 0x%x\n", val);
	pmic_set_register_value(PMIC_ISINK_BREATH1_TR2_SEL, (short)val);
	pmic_set_register_value(GREEN_TR2_SEL, (short)val);
	return count;
}
static ssize_t breath_leds_read_tr2(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", pmic_get_register_value(PMIC_ISINK_BREATH1_TR2_SEL));
}
static DRIVER_ATTR(tr2, S_IWUSR | S_IRUGO, breath_leds_read_tr2,	breath_leds_write_tr2);
/*----------------------------------------------------------------------------------------------*/

static ssize_t breath_leds_write_tf1(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = DEFAULT_BREATH1_TF1_SEL;
	int ret = 0;
	char sbuf[20];
	memset(sbuf,0,20);
	strncpy(sbuf,buf,count);
	ret = kstrtoint((const char *)sbuf, 10,&val);
	printk("SET PMIC_ISINK_BREATH1_TR1_SEL = 0x%x\n", val);
	pmic_set_register_value(PMIC_ISINK_BREATH1_TF1_SEL, (short)val);
	pmic_set_register_value(GREEN_TF1_SEL, (short)val);
	return count;
}
static ssize_t breath_leds_read_tf1(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", pmic_get_register_value(PMIC_ISINK_BREATH1_TF1_SEL));
}
static DRIVER_ATTR(tf1, S_IWUSR | S_IRUGO, breath_leds_read_tf1,	breath_leds_write_tf1);
/*----------------------------------------------------------------------------------------------*/

static ssize_t breath_leds_write_tf2(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = DEFAULT_BREATH1_TF2_SEL;
	int ret = 0;
	char sbuf[20];
	memset(sbuf,0,20);
	strncpy(sbuf,buf,count);
	ret = kstrtoint((const char *)sbuf, 10,&val);
	printk("SET PMIC_ISINK_CH1_STEP = 0x%x\n", val);
	pmic_set_register_value(PMIC_ISINK_BREATH1_TF2_SEL, (short)val);
	pmic_set_register_value(GREEN_TF2_SEL, (short)val);
	return count;
}
static ssize_t breath_leds_read_tf2(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", pmic_get_register_value(PMIC_ISINK_BREATH1_TF2_SEL));
}
static DRIVER_ATTR(tf2, S_IWUSR | S_IRUGO, breath_leds_read_tf2,	breath_leds_write_tf2);
/*----------------------------------------------------------------------------------------------*/

static ssize_t breath_leds_write_ton(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = DEFAULT_BREATH1_TON_SEL;
	int ret = 0;
	char sbuf[20];
	memset(sbuf,0,20);
	strncpy(sbuf,buf,count);
	ret = kstrtoint((const char *)sbuf, 10,&val);
	printk("SET PMIC_ISINK_BREATH1_TON_SEL = 0x%x\n", val);
	pmic_set_register_value(PMIC_ISINK_BREATH1_TON_SEL, (short)val);
	pmic_set_register_value(GREEN_TON_SEL, (short)val);
	return count;
}
static ssize_t breath_leds_read_ton(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", pmic_get_register_value(PMIC_ISINK_BREATH1_TON_SEL));
}
static DRIVER_ATTR(ton, S_IWUSR | S_IRUGO, breath_leds_read_ton,	breath_leds_write_ton);
/*----------------------------------------------------------------------------------------------*/

static ssize_t breath_leds_write_toff(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = DEFAULT_BREATH1_TOFF_SEL;
	int ret = 0;
	char sbuf[20];
	memset(sbuf,0,20);
	strncpy(sbuf,buf,count);
	ret = kstrtoint((const char *)sbuf, 10,&val);
	printk("SET PMIC_ISINK_CH1_STEP = 0x%x\n", val);
	pmic_set_register_value(PMIC_ISINK_BREATH1_TOFF_SEL, (short)val);
	pmic_set_register_value(GREEN_TOFF_SEL, (short)val);
	return count;
}
static ssize_t breath_leds_read_toff(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", pmic_get_register_value(PMIC_ISINK_BREATH1_TOFF_SEL));
}
static DRIVER_ATTR(toff, S_IWUSR | S_IRUGO, breath_leds_read_toff,	breath_leds_write_toff);
/*----------------------------------------------------------------------------------------------*/

static ssize_t breath_leds_write_dimDutyRed(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = DEFAULT_DIM1_DUTY;
	int ret = 0;
	char sbuf[20];
	memset(sbuf,0,20);
	strncpy(sbuf,buf,count);
	ret = kstrtoint((const char *)sbuf, 10,&val);
	printk("SET PMIC_ISINK_CH1_STEP = 0x%x\n", val);
	pmic_set_register_value(PMIC_ISINK_DIM1_DUTY, (short)val);
	return count;
}
static ssize_t breath_leds_read_dimDutyRed(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", pmic_get_register_value(PMIC_ISINK_DIM1_DUTY));
}
static DRIVER_ATTR(dimDutyRed, S_IWUSR | S_IRUGO, breath_leds_read_dimDutyRed,	breath_leds_write_dimDutyRed);
/*----------------------------------------------------------------------------------------------*/

static ssize_t breath_leds_write_dimDutyGreen(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = DEFAULT_DIM1_DUTY;
	int ret = 0;
	char sbuf[20];
	memset(sbuf,0,20);
	strncpy(sbuf,buf,count);
	ret = kstrtoint((const char *)sbuf, 10,&val);
	printk("SET GREEN_STEP = 0x%x\n", val);
	pmic_set_register_value(GREEN_DUTY, (short)val);
	return count;
}
static ssize_t breath_leds_read_dimDutyGreen(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", pmic_get_register_value(GREEN_DUTY));
}
static DRIVER_ATTR(dimDutyGreen, S_IWUSR | S_IRUGO, breath_leds_read_dimDutyGreen,	breath_leds_write_dimDutyGreen);
/*----------------------------------------------------------------------------------------------*/
static ssize_t breath_leds_write_dimFset(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = DEFAULT_DIM1_FSEL;
	int ret = 0;
	char sbuf[20];
	memset(sbuf,0,20);
	strncpy(sbuf,buf,count);
	ret = kstrtoint((const char *)sbuf, 10,&val);
	printk("SET PMIC_ISINK_CH1_STEP = 0x%x\n", val);
	pmic_set_register_value(PMIC_ISINK_DIM1_FSEL, (short)val);
	pmic_set_register_value(GREEN_FSEL, (short)val);
	return count;
}
static ssize_t breath_leds_read_dimFset(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", pmic_get_register_value(PMIC_ISINK_DIM1_FSEL));
}
static DRIVER_ATTR(dimFset, S_IWUSR | S_IRUGO, breath_leds_read_dimFset,	breath_leds_write_dimFset);
/*----------------------------------------------------------------------------------------------*/

static ssize_t breath_leds_write_enable(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = 0;
	int ret = 0;
	char sbuf[20];
	memset(sbuf,0,20);
	strncpy(sbuf,buf,count);
	ret = kstrtoint((const char *)sbuf, 10,&val);
	printk("SET PMIC_ISINK_CH1_EN = 0x%x\n", val);

	pmic_set_register_value(PMIC_ISINK_CH1_EN, 0x0);
	pmic_set_register_value(GREEN_EN, 0x0);

	if(val == 0x1)
	{
		pmic_set_register_value(PMIC_ISINK_CH1_EN, 0x1);
	}
	else if(val == 0x2)
	{
		pmic_set_register_value(GREEN_EN, 0x1);
	}
	else if(val == 0x3)
	{
		pmic_set_register_value(PMIC_ISINK_CH1_EN, 0x1);
		pmic_set_register_value(GREEN_EN, 0x1);
	}

	return count;
}
static ssize_t breath_leds_read_enable(struct device_driver *ddri, char *buf)
{
	int ret = 0;

	ret = pmic_get_register_value(PMIC_ISINK_CH1_EN) | (pmic_get_register_value(GREEN_EN) << 1);
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", ret);
}
static DRIVER_ATTR(enable, S_IWUSR | S_IRUGO, breath_leds_read_enable,	breath_leds_write_enable);

/*----------------------------------------------------------------------------------------------*/

static ssize_t breath_leds_write_mode(struct device_driver *ddri, const char *buf, size_t count)
{
	int val = ISINK_BREATH_MODE;
	int ret = 0;
	char sbuf[20];
	memset(sbuf,0,20);
	strncpy(sbuf,buf,count);
	ret = kstrtoint((const char *)sbuf, 10,&val);
	printk("SET PMIC_ISINK_CH1_MODE = 0x%x\n", val);
	pmic_set_register_value(PMIC_ISINK_CH1_MODE, (short)val);
	pmic_set_register_value(GREEN_MODE, (short)val);
	return count;
}
static ssize_t breath_leds_read_mode(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", pmic_get_register_value(PMIC_ISINK_CH1_MODE));
}
static DRIVER_ATTR(mode, S_IWUSR | S_IRUGO, breath_leds_read_mode,	breath_leds_write_mode);
/*----------------------------------------------------------------------------------------------*/

static struct driver_attribute *breath_leds_attr_list[] = {
	&driver_attr_step,
    &driver_attr_tr1,
    &driver_attr_tr2,
    &driver_attr_tf1,
    &driver_attr_tf2,
    &driver_attr_ton,
    &driver_attr_toff,
    &driver_attr_dimDutyRed,
    &driver_attr_dimDutyGreen,
    &driver_attr_dimFset,
    &driver_attr_enable,
    &driver_attr_mode,
};

static int breath_leds_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(breath_leds_attr_list)/sizeof(breath_leds_attr_list[0]));

	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, breath_leds_attr_list[idx])))
		{
			printk("driver_create_file (%s) = %d\n", breath_leds_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int breath_leds_probe(struct platform_device *pdev);
static int breath_leds_remove(struct platform_device *pdev);


#ifdef CONFIG_OF
static const struct of_device_id breath_leds_of_match[] = {
	{.compatible = "mediatek,breath-leds",},
	{},
};
#endif

static struct platform_driver breath_leds_driver = {
	.probe	  = breath_leds_probe,
	.remove	 = breath_leds_remove,
	.driver = {

		.name  = "breath-leds",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = breath_leds_of_match,
#endif
	}
};

static int breath_leds_probe(struct platform_device *pdev)
{
	int ret = -1;

	printk("breath_leds_probe\n");
	breath_leds_setup();
	breath_leds_enable(0);

	ret = misc_register(&breath_leds_device);
	if(ret)
	{
		printk("%s: breath_leds misc_register failed\n", __func__);
	}

	ret = breath_leds_attr(&(breath_leds_driver.driver));
	if (ret) {
		printk("%s: failed to sysfs_create_file\n", __func__);
	}

	return 0;
}

static int breath_leds_remove(struct platform_device *pdev)
{
	printk("breath_leds_remove\n");
	return 0;
}

static int __init breath_leds_init(void)
{
	int ret = -1;
	ret = platform_driver_register(&breath_leds_driver);
	if (ret)
	{
		printk("%s: failed to register breath_leds driver\n", __func__);
	}
	else
	{
		printk("%s: register breath_leds driver success\n", __func__);
	}
	return ret;
}

static void __exit breath_leds_exit(void)
{
	platform_driver_unregister(&breath_leds_driver);
}

module_init(breath_leds_init);

MODULE_AUTHOR("Li Guanxiong");
MODULE_DESCRIPTION("BREATH LEDS DRIVER");
MODULE_LICENSE("GPL");
