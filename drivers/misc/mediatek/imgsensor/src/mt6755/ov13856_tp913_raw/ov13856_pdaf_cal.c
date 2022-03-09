#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov13856tp913raw_Sensor.h"
#define PFX "OV13856_eeprom"
#define LOG_INF(format, args...)	pr_err(PFX "[%s] " format, __FUNCTION__, ##args)

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);

#define USHORT             unsigned short
//#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define OV13856_EEPROM_READ_ID  0xA1
#define OV13856_EEPROM_WRITE_ID   0xA2//eeprom id 0xA0

#define OV13856_I2C_SPEED        100
#define OV13856_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048
unsigned char OV13856_eeprom_data[DATA_SIZE]= {0};
BYTE ov13856_eeprom_awb_data[32] = {0};/* [linyimin] Add awb cali */
unsigned char ov13856_eeprom_oqc_data[DATA_SIZE+1]= {0};

static bool awb_get_done = false;/* [linyimin] Add awb cali */
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;
static int oqc_get_done = 0;

static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	if(addr > OV13856_MAX_OFFSET)
		return false;
	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, OV13856_EEPROM_READ_ID)<0) // [linyimin] Fix read id err
		return false;
    return true;
}

static bool _read_ov13856_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0;
	int offset = addr;
	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			return false;
		}

//		LOG_INF("read_eeprom proc[%d] %d\n",offset, data[i]);
//		if (i >= 1360)
//		LOG_INF("read_eeprom proc[%d] %d\n",offset, data[i]);

		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

bool read_ov13856_eeprom( kal_uint16 addr, unsigned char* data, kal_uint32 size){
	addr = 0x801;//from the first valid data on
	size = 1404;//0x57c//the total valid data size
//	unsigned char header[9]= {0};
//	_read_ov13856_eeprom(0x801, header, 9);

	LOG_INF("read_ov13856_eeprom, size = %d\n", size);

	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_ov13856_eeprom(addr, OV13856_eeprom_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}

	memcpy(data, OV13856_eeprom_data, size);
    return true;
}

/* [linyimin start] Add awb cali */
bool read_ov13856_eeprom_AWB(kal_uint16 addr, BYTE* data, kal_uint32 size){
	BYTE flag;
	int i, checksum=0, sum=0;
	if (!awb_get_done) {
		if (!selective_read_eeprom(0x0020, &flag)){
			pr_err("Read awb flag error\n");
			return false;
		}

		if (!(flag & 0x80)) {
			pr_err("Flag of AWB is wrong, flag = %d", flag);
			return false;
		}

		if(!_read_ov13856_eeprom(0x0021, OV13856_eeprom_data, 21)){
			pr_err("Read awb error\n");
			return false;
		}

		for(i=0; i<20; i++) {
			sum += (int)(OV13856_eeprom_data[i]);
		}

		checksum = OV13856_eeprom_data[20];

		if (checksum != (sum%0xFF + 1)) {
			pr_err("checksum of AWB is wrong, checksum=%d, sum=%d\n", checksum, sum);
			return false;
		}

		pr_err("AWB data read ok\n");

		memcpy(ov13856_eeprom_awb_data, OV13856_eeprom_data, 20);
		awb_get_done = 1;
	}
	memcpy(data, ov13856_eeprom_awb_data, 20);
	return true;
}
/* [linyimin end] */

/* [linyimin start] Read 13856 mid */
int read_imx13856_eeprom_MID(void)
{
	BYTE mid;

	if (!selective_read_eeprom(0x0004, &mid)) {
		pr_err("Read mid error\n");
		return 0;
	}

	return (int)mid;
}

bool read_ov13856_eeprom_OQC(BYTE* data, kal_uint32 size){
        BYTE flag;
        int i, checksum=0, sum=0;
        if (!oqc_get_done) {
                if (!selective_read_eeprom(0x0e00, &flag)){
                        pr_err("Read OQC flag error\n");
                        return false;
                }

                if (flag != 0x01) {
                        pr_err("Flag of OQC is wrong, flag = %d", flag);
                        return false;
                }

                if(!_read_ov13856_eeprom(0x0e01, ov13856_eeprom_oqc_data, 2049)){
                        pr_err("Read awb error\n");
                        return false;
                }

                for(i=0; i<2048; i++) {
                        sum += (int)(ov13856_eeprom_oqc_data[i]);
                }

                checksum = ov13856_eeprom_oqc_data[2048];

                if (checksum != (sum%255)) {
                        pr_err("checksum of OQC is wrong, checksum=%d, sum=%d\n", checksum, sum);
                        return false;
                }

                pr_err("OQC data read ok\n");

                oqc_get_done = 1;
        }
        memcpy(data, ov13856_eeprom_oqc_data, size);
        return true;
}
/* [linyimin end] */
