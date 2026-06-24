// SPDX-License-Identifier:  GPL-2.0+
/*
 * Microchip "MTCH" Touch Sensor IC
 *
 * Copyright (c) 2026 Microchip Technology Inc.
 * Author: Michael Gong <michael.gong@microchip.com>
 * 
 */

#include <linux/version.h>
#include <linux/acpi.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/unaligned.h>
#include <linux/pm.h>

#define DRIVER_VERSION_NUMBER "6.12-20260624"

/* Firmware file */
#define MTCH_FW_NAME	"mtch.fw"

/* Configuration file */
#define MTCH_CFG_NAME	"mtch.cfg"
#define CFG_MAGIC_V1	"OBP_RAW V1"
#define CFG_MAGIC_V2	"OBP_RAW V2"
#define CFG_MAGIC_V3	"OBP_RAW V3"
#define CFG_MAGIC_V4	"OBP_RAW V4"

/* Registers */
#define OBJECT_START		0x07
#define INFO_CRC_SIZE		3
#define MAX_BLOCK_RD_WR		255

/* Objects accessed through driver */
#define GEN_ENCRYPTIONSTATUS_T2				2
#define GEN_MESSAGE_T5						5
#define GEN_COMMAND_T6						6
#define GEN_POWER_T7						7
#define SPT_SELFTESTCONTROL_T10				10
#define SPT_SELFTESTPINFAULT_T11			11
#define SPT_SELFTESTSIGLIMIT_T12			12
#define SPT_COMMSCONFIG_T18					18
#define SPT_GPIOPWM_T19						19
#define DEBUG_DIAGNOSTIC_T37				37
#define SPT_USERDATA_T38					38
#define SPT_MESSAGECOUNT_T44				44
#define SPT_SERIALDATACOMMAND_T68			68
#define SPT_LOWERPOWERIDLECONFIG_T126		126
#define GEN_EXTENDEDOBJECTTABLE_T254		254
#define SPT_MTCHKEYCTRL_T501				501
#define TOUCH_MTCHKEYARRAY_T500				500
#define SPT_MTCHSLIDERCTRL_T502				502

/* T501 status field values */
#define T501_STATUS_NO_DET		0x03	/* No touch detected (confirmed release) */
#define T501_STATUS_FILT_IN		0x04	/* Touch delta >= threshold, DI starting */
#define T501_STATUS_DETECT		0x85	/* Touch confirmed (held for TCHDI+1 cycles) */
#define T501_STATUS_FILT_OUT	0x86	/* Release filtering, delta below threshold */

/* T501 control byte bit definitions (Byte 0) */
#define T501_CTRL_ENABLE		BIT(0)	/* Enable bit */
#define T501_CTRL_RPTEN			BIT(1)	/* Report enable bit */

/* T501 key codes for fixed keys on EVK sensor */
static const unsigned int mtch_key_codes[7] = {
	KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7
};

/* T501 button codes for button group on EVK sensor */
static const unsigned int mtch_button_codes[4] = {
	BTN_0, BTN_1, BTN_2, BTN_3
};

/* T502 status field bit definitions */
#define T502_STATUS_DETECT		BIT(7)	/* Touch detected */
#define T502_STATUS_DIRCHG		BIT(1)	/* Direction changed */
#define T502_STATUS_POSCHG		BIT(0)	/* Position changed */

/* T502 control byte bit definitions (Byte 0) */
#define T502_CTRL_ENABLE		BIT(0)	/* Enable bit */
#define T502_CTRL_RPTEN			BIT(1)	/* Report enable bit */
#define T502_CTRL_TYPE_MASK		(BIT(7) | BIT(6))	/* Type field: bits 7:6 */
#define T502_CTRL_TYPE_SHIFT	6
#define T502_TYPE_SLIDER		0
#define T502_TYPE_WHEEL			1

/* Maximum T502 instances */
#define T502_MAX_INSTANCES		4	/* Max value dependent on chip */

/* T502 configuration byte offsets */
#define T502_CFG_CTRL			0
#define T502_CFG_STARTKEY		1
#define T502_CFG_NUMKEYS		2

/* T502 instance info structure */
struct t502_instance_info {
	u8 type;		/* 0 = slider, 1 = wheel */
	bool valid;		/* enabled and reporting */
	u8 dev_index;	/* index into wheels[] or sliders[] array */
	u8 startkey;	/* first key index used by this wheel/slider */
	u8 numkeys;		/* number of keys in this wheel/slider */
	u8 range;		/* RANGE field from RESOLUTION byte (bits 7:4) */
	u16 max_pos;	/* Maximum position value: (1 << range) - 1 */
};

/* Maximum T501 instances (keys) */
#define T501_MAX_INSTANCES		64

/* T501 key usage types */
#define T501_KEY_UNUSED			0	/* Key not enabled or not reporting */
#define T501_KEY_WHEEL_SLIDER	1	/* Key belongs to a wheel/slider */
#define T501_KEY_BUTTON			2	/* Key is a standalone button */

/* T501 key mapping info - maps T501 instance to keycode */
struct t501_key_info {
	u8 usage;			/* T501_KEY_UNUSED, WHEEL_SLIDER, or BUTTON */
	unsigned int keycode;		/* Linux keycode to report (if BUTTON) */
};

/* T254 Extended Object Table constants */
#define T254_OBJECTS_PER_BLOCK		3
#define T254_ELEMENT_SIZE			7
#define T254_CRC_SIZE				3

/* T2 encryption status bits */
#define CONFIGENCEN		BIT(1)
#define MSGENCEN		BIT(2)

/* T2 object offsets */
#define T2_PAYLOADCRC_OFS	0x02
#define T2_ENCKEYCRC_OFS	0x05

/* Device encryption state flags */
#define CFG_ENCRYPTED		BIT(0)
#define DEV_ENCRYPTED		BIT(1)
#define MSG_ENCRYPTED		BIT(2)

/* Define encryption check macros */
#define CHECK_ANY_BITS(state, flag) ((state) & (flag))
#define IS_BITS_ENABLED(var, bits) (((var) & (bits)) != 0)
#define CLEAR_BITS(var, bits) ((var) &= !(bits))
#define SET_BITS(var, bits) ((var) |= (bits))

/* GEN_MESSAGE_T5 object */
#define RPTID_NOMSG		0xff
#define RPTID_RVSD		0x00

/* GEN_COMMAND_T6 field */
#define CMD_RESET_OFS		0
#define CMD_BACKUPNV_OFS	1
#define CMD_CALIBRATE_OFS	2
#define CMD_REPORTALL_OFS	3

/* T6 Definitions */
#define BTLDR_RESET_VAL		0xa5
#define RESET_VALUE			0x01
#define BACKUP_UNFREEZE		0x11
#define BACKUP_FREEZE		0x22
#define BACKUP_W_STOP		0x33
#define BACKUP_VALUE		0x55
#define BACKUP_NVM_VALID	0x66

/* T6 status byte BIT definitions */
#define T6_STATUS_RESET		BIT(7)
#define T6_STATUS_OFL		BIT(6)
#define T6_STATUS_SIGERR	BIT(5)
#define T6_STATUS_CAL		BIT(4)
#define T6_STATUS_CFGERR	BIT(3)
#define T6_STATUS_COMSERR	BIT(2)
#define T6_STATUS_FROZEN	BIT(1)

/* Define for T68 Error Status byte */
#define T68_SUCCESS				0x00
#define T68_OUT_OF_SYNC			0x01
#define T68_DTYPE_NOTSUP		0x02
#define T68_DLEN_MISMATCH		0x03
#define T68_MAX_LENGTH_ERR		0x04
#define T68_INVALID_DATA		0x05
#define T68_NO_ACTION_ERR		0x06

/* T68 datatype for encryption */
#define T68_ENC_DTYPE		0x000C

/* T68 BIT definitions */
#define T68_CTRL_ENABLE			BIT(0)
#define T68_CTRL_RPTEN			BIT(1)

/* T68 definitions */
#define T68_CTRL_OFS		1	/* byte offset */			
#define T68_DTYPE_OFS		3
#define T68_LENGTH_OFS		5
#define T68_DATA_OFS		6
#define T68_CMD_OFS			70
#define T68_TIMEOUT			50	/* default 50ms - adjust */

/* T68 cmd definitions */
#define T68_CMD_NONE		0
#define T68_CMD_START		1
#define T68_CMD_CONTINUE	2
#define T68_CMD_END			3

/* GEN_POWER_T7 field */
struct t7_config {
	u8 idle;
	u8 active;
} __packed;

#define POWER_CFG_INIT			0
#define POWER_CFG_DEEPSLEEP		1
#define POWER_CFG_RUN			2

/* SPT_COMMSCONFIG_T18 */
#define COMMS_CTRL_OFS		0x00
#define COMMS_RETRIGEN		BIT(6)

/* Delay times */
#define BACKUP_TIME			50		/* msec */
#define RESET_GPIO_TIME		20		/* msec */
#define RESET_INVALID_CHG	1000	/* msec */
#define BTLDR_RESET_TIME	300		/* msec */
#define RESET_TIMEOUT		3000	/* msec */
#define CRC_TIMEOUT			1000	/* msec */
#define FW_FLASH_TIME		1000	/* msec */
#define FW_RESET_TIME		3000	/* msec */
#define FW_CHG_TIMEOUT		300		/* msec */
#define BOOTLOADER_WAIT		1000	/* msec */

/* Command to unlock bootloader */
#define UNLOCK_CMD_MSB	0xaa
#define UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status codes 	*/
#define WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define WAITING_FRAME_DATA		0x80	/* valid 7 6 bit only */
#define APP_CRC_FAIL			0x40	/* valid 7 6 bit only */
#define FRAME_CRC_CHECK			0x02
#define FRAME_CRC_FAIL			0x03
#define FRAME_CRC_PASS			0x04

/* Bootloader maks, BIT defitions */
#define BOOT_STATUS_MASK		0x3f
#define BOOT_ID_MASK			0x1f
#define BOOT_EXTENDED_ID		BIT(5)

/* Debug message size max */
#define DEBUG_MSG_MAX			200

/* Write and read flags - For use later */
#define F_R_CHIP_ID		BIT(0)
#define F_W_SYSFS_ID	BIT(1)
#define F_R_SYSFS_ID	BIT(2)

/* Host Client Max */
#define HC_DEV_MAX		4

struct mtch_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

/* Host Client context */
struct mxt_hc {
	bool hc_capable;
	bool hc_mode;
	u8 num_client_spt;
	u8 num_of_devs;
	u8 devid_num;
	u8 reqd_device;
	u16 max_rd_size;
};

struct mtch_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

struct mtch_object_ext {
	u16 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

/* Config update context */
struct mtch_cfg {
	u8 *raw;
	size_t raw_size;
	off_t raw_pos;
	off_t tmp_raw_pos;
	u8 **mem;
	size_t mem_size;
	int start_ofs;
	struct mtch_info info;
	u16 object_skipped_ofs;
};

/* Global context  */
struct mtch_data {
	struct i2c_client *client;
	struct mutex i2c_lock;	/* Avoid conflict on I2C bus */
	struct input_dev *input_dev;
	struct input_dev **input_dev_sliders;	/* Dynamic array of slider devices */
	struct input_dev **input_dev_wheels;	/* Dynamic array of wheel devices */
	char phys[64];		/* device physical location */
	struct mtch_object *object_table;
	struct mtch_info *info;
	void *raw_info_block;
	struct mxt_hc hc;	/* Host client structure */
	unsigned int irq;
	bool in_bootloader;
	u16 mem_size;
	struct bin_attribute mem_access_attr;
	bool crc_enabled;
	bool debug_enabled;
	bool debug_v2_enabled;
	bool skip_crc_write;
	u8 *debug_msg_data;
	u16 debug_msg_count;
	struct bin_attribute debug_msg_attr;
	struct mutex debug_msg_lock;
	u8 max_reportid;
	u32 dev_cfg_crc[5];
	u32 nvm_dev_crc[5];
	u32 nvm_cfg_crc[5];
	u32 info_crc;
	u8 bootloader_addr;
	u8 *msg_buf;
	u8 t6_status;
	bool update_input;
	u8 last_message_count;
	u8 num_touchids;
	bool irq_enabled;
	struct t7_config t7_cfg;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data regulators[2];
	const char *cfg_name;

	/* T68 variables */
	u8 *t68_buf;
	u16 t68_cmd_addr;
	u32 t68_data_crc;
	u8 t68_datasize;
	u16 t68_datatype;
	u32 t68_checksum;
	u16 t68_length;
	bool t68_last_frame;

	/* Max encryption block size */
	u8 enc_blocksize;

	/* CRC of the encrypted payload */
	u32 enc_payload_crc;

	/* Encrpytion state of device */
	u8 encryption_state;

	/* Encrypted data size, less padding */
	u8 enc_datasize;
	u16 num_datasize_bytes;

	/* Cached parameters from object table */
	u16 T2_address;
	u16 T5_address;
	u8 T5_msg_size;
	u8 T6_reportid;
	u16 T6_address;
	u8 T6_obj_size;
	u16 T7_address;
	u16 T38_address;
	u8 T38_obj_size;
	u16 T18_address;
	u8 T19_reportid_min;
	u16 T44_address;
	u16 T68_address;
	u8 T68_obj_size;
	u8 T68_reportid_min;
	u8 T500_reportid_min;
	u8 T501_reportid_min;
	u8 T501_reportid_max;
	u8 T502_reportid_min;
	u8 T502_reportid_max;

	/* T501 key mapping: tracks usage and keycode per T501 instance */
	struct t501_key_info t501_keys[T501_MAX_INSTANCES];
	u8 t501_num_instances;
	u8 t501_num_buttons;	/* Number of T501 keys assigned as buttons */

	/* T502 wheel/slider configuration and state tracking */
	struct t502_instance_info t502_instances[T502_MAX_INSTANCES];
	u8 t502_num_instances;
	u8 t502_num_wheels;
	u8 t502_num_sliders;
	u16 *t502_last_pos;		/* Dynamic array: last position per instance */
	bool *t502_in_detect;	/* Dynamic array: touch detection state per instance */
	bool wheel_relative;	/* true = report REL_WHEEL, false = report ABS_WHEEL */

	/* flag fw update in bootloader */
	struct completion bl_completion;

	/* flag reset handling */
	struct completion reset_completion;

	/* flag config update handling */
	struct completion crc_completion;

	/* flag t68 message handling */
	struct completion t68_completion;

	u32 *t19_keymap;
	unsigned int t19_num_keys;

	u32 *t500_keymap;
	unsigned int t500_num_keys;

	/* T500 button state storage for change detection */
	u8 t500_button_state[8];

	/* for config_fw updates, power up, irq handling, reset */
	bool irq_processing;
	bool system_power_up;
	bool reset_state;
	bool sysfs_updating_cfg_fw;

	/* Debugfs variables */
	struct dentry *debug_dir;

	/* Optional read/write crc byte */
	bool T5_msg_crc_enabled;
	bool write_crc_enabled;

	/* Extended Ojbect Table T254 */
	u16 T254_address;
	u16 T254_obj_size;
	struct mtch_object_ext *ext_object_table;
	u8 ext_object_num;
};

static size_t mtch_obj_size(const struct mtch_object *obj)
{
	return obj->size_minus_one + 1;
}

static size_t mtch_obj_instances(const struct mtch_object *obj)
{
	return obj->instances_minus_one + 1;
}

static size_t mtch_obj_ext_size(const struct mtch_object_ext *obj)
{
	return obj->size_minus_one + 1;
}

static size_t mtch_obj_ext_instances(const struct mtch_object_ext *obj)
{
	return obj->instances_minus_one + 1;
}

static int mtch_wait_for_completion(struct mtch_data *data,
				   struct completion *comp,
				   unsigned int timeout_ms)
{
	struct device *dev = &data->client->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	/* Initialize debug message variable */
	char *debug_msg = NULL;

	if (comp == &data->bl_completion)
		debug_msg = "bl_completion";
	else if (comp == &data->reset_completion)
		debug_msg = "reset_completion";
	else if (comp == &data->crc_completion)
		debug_msg = "crc_completion";
	else if (comp == &data->t68_completion)
		debug_msg = "t68_completion";
	else
		debug_msg = "unknown_completion";

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret > 0) {
		dev_dbg(dev, "Time left in jiffies %li", ret);
	} else if (ret == 0) {
		dev_err(dev, "[%s] Wait for completion timed out.\n",
			debug_msg);
		return -ETIMEDOUT;
	} else if (ret == -ERESTARTSYS) {
		dev_warn(dev, "Completion event was interrupted\n");
	}

	return 0;
}

static bool mtch_lookup_chips(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	u8 family_id;
	u8 variant_id;
	bool is_chip_found = false;

	if (!data->info)
		return false;

	family_id = data->info->family_id;
	variant_id = data->info->variant_id;

	switch (family_id) {
	case 0xA9:
		if (variant_id & 0x80)
			data->encryption_state |= DEV_ENCRYPTED;
		else
			data->encryption_state &= ~(DEV_ENCRYPTED);

		is_chip_found = true;

		switch (variant_id & 0x7F) {
		case 0x00:	/* MTCH3380P */
			dev_info(dev, "Found MTCH3380P\n");
			break;
		case 0x01:	/* MTCH3240P */
			dev_info(dev, "Found MTCH3240P\n");
			break;
		default:
			dev_info(dev, "Found MTCH device\n");
			break;
		}
		break;
	default:
		dev_info(dev, "Found unknown device\n");
		break;
	}

	return is_chip_found;
}

static int mtch_bootloader_read(struct mtch_data *data,
			       u8 *val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		dev_err(&data->client->dev, "%s: i2c recv failed (%d)\n",
			__func__, ret);
	}

	return ret;
}

static int mtch_bootloader_write(struct mtch_data *data,
				const u8 * const val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;
	u8 *data_buf;

	data_buf = kmalloc(count, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

	memcpy(&data_buf[0], val, count);

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = data_buf;

	ret = i2c_transfer(data->client->adapter, &msg, 1);

	if (ret == 1) {
		ret = 0;
	} else {
		ret = ret < 0 ? ret : -EIO;
		dev_err(&data->client->dev, "%s: i2c send failed (%d)\n",
			__func__, ret);
	}

	kfree(data_buf);
	return ret;
}

static int mtch_lookup_bootloader_address(struct mtch_data *data)
{
	u8 appmode = data->client->addr;
	u8 bootloader;

	switch (appmode) {
	case 0x3a:
		bootloader = 0x16;
		break;
	case 0x3b:
		bootloader = 0x17;
		break;
	default:
		dev_err(&data->client->dev,
			"Unsupported I2C address 0x%02x (only 0x3A and 0x3B supported)\n",
			appmode);
		return -EINVAL;
	}

	data->bootloader_addr = bootloader;

	dev_info(&data->client->dev, "Bootloader address: 0x%02x\n", bootloader);

	return 0;
}

static int mtch_probe_bootloader(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	int error;
	u8 val;
	bool crc_failure;

	error = mtch_lookup_bootloader_address(data);
	if (error)
		return error;

	error = mtch_bootloader_read(data, &val, 1);
	if (error)
		return error;

	/* Check app crc fail mode */
	crc_failure = (val & ~BOOT_STATUS_MASK) == APP_CRC_FAIL;

	dev_err(dev, "Detected bootloader, status:%02X%s\n",
			val, crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static u8 mtch_get_bootloader_version(struct mtch_data *data, u8 val)
{
	struct device *dev = &data->client->dev;
	u8 buf[3];
	u8 ret = 0;

	if (val & BOOT_EXTENDED_ID) {
		if (mtch_bootloader_read(data, &buf[0], 3) != 0) {
			dev_err(dev, "%s: i2c failure\n", __func__);
			return val;
		}

		dev_dbg(dev, "Bootloader ID:%d Version:%d\n", buf[1], buf[2]);
		ret = buf[0];
	} else {
		dev_dbg(dev, "Bootloader ID:%d\n", val & BOOT_ID_MASK);
		ret = val;
	}
	return ret;
}

static int mtch_check_bootloader(struct mtch_data *data, unsigned int state,
				bool wait)
{
	struct device *dev = &data->client->dev;
	u8 val;
	int ret;
	int poll_count;

	if (wait) {
		/*
		 * In application update mode, the interrupt
		 * line signals state transitions. We must wait for the
		 * CHG assertion before reading the status byte.
		 * Once the status byte has been read, the line is deasserted.
		 */
		ret = mtch_wait_for_completion(data, &data->bl_completion,
					      FW_CHG_TIMEOUT);
		if (ret) {
			/*
			 * Timeout occurred. Read bootloader status anyway to
			 * check if state has advanced (CHG may have been missed).
			 */
			dev_dbg(dev, "Completion timeout, checking bootloader status\n");
		}
	}

	ret = mtch_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	dev_dbg(dev, "Bootloader status: 0x%02X (expected state: 0x%02X)\n",
		val, state);

	if (state == WAITING_BOOTLOAD_CMD)
		val = mtch_get_bootloader_version(data, val);

	switch (state) {
	case WAITING_BOOTLOAD_CMD:
	case WAITING_FRAME_DATA:
	case APP_CRC_FAIL:
		val &= ~BOOT_STATUS_MASK;
		break;
	case FRAME_CRC_PASS:
		if (val == FRAME_CRC_CHECK) {
			/*
			 * The transition from FRAME_CRC_CHECK to FRAME_CRC_PASS
			 * can be very fast (as short as 90us). With IRQF_ONESHOT,
			 * the IRQ is masked until the threaded handler completes,
			 * so rapid interrupts can be lost.
			 *
			 * Poll the status instead of waiting for interrupt since
			 * the CRC check is typically very fast.
			 */
			for (poll_count = 0; poll_count < 100; poll_count++) {
				usleep_range(100, 200);
				ret = mtch_bootloader_read(data, &val, 1);
				if (ret)
					return ret;

				if (val != FRAME_CRC_CHECK)
					break;
			}

			if (val == FRAME_CRC_CHECK) {
				dev_err(dev, "Timeout waiting for CRC check to complete\n");
				return -ETIMEDOUT;
			}

			dev_dbg(dev, "CRC check complete, status: 0x%02X\n", val);
		}

		if (val == FRAME_CRC_FAIL) {
			dev_err(dev, "Bootloader CRC fail\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(dev, "Invalid bootloader state %02X != %02X\n",
			val, state);
		return -EINVAL;
	}

	return 0;
}

static int mtch_send_bootloader_cmd(struct mtch_data *data, bool unlock)
{
	int ret;
	u8 buf[2];

	if (unlock) {
		buf[0] = UNLOCK_CMD_LSB;
		buf[1] = UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mtch_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

static u8 __mtch_calc_crc8(unsigned char crc, unsigned char data)
{
	static const u8 crcpoly = 0x8C;
	u8 index;
	u8 fb;

	index = 8;

	do {
		fb = (crc ^ data) & 0x01;
		data >>= 1;
		crc >>= 1;
		if (fb)
			crc ^= crcpoly;
	} while (--index);

	return crc;
}

static int __mtch_read_reg_ex(struct mtch_data *data,
			     u16 reg, u16 len, void *val, bool skip_addr_write)
{
	struct device *dev = &data->client->dev;
	struct i2c_msg xfer[2];
	u8 buf[3];
	u8 crc_data = 0;
	char *ptr_data;
	int ret = 0;
	int i;
	int num_msgs;

	if (skip_addr_write) {
		/*
		 * Read only - address pointer was already set by a prior write
		 * (e.g., mtch_write_info_addr). Just do a read transaction.
		 */
		xfer[0].addr = data->client->addr;
		xfer[0].flags = I2C_M_RD;
		xfer[0].len = len;
		xfer[0].buf = val;
		num_msgs = 1;
	} else {
		buf[0] = reg & 0xff;
		buf[1] = (reg >> 8) & 0xff;

		/* Calculate and send TX CRC - 2 byte onlys */
		if (data->T5_msg_crc_enabled) {
			buf[1] = ((reg >> 8) & 0xff) | 0x80;	/* Set upper MSBit */

			for (i = 0; i < 2; i++)
				crc_data = __mtch_calc_crc8(crc_data, buf[i]);

			buf[2] = crc_data;
		}

		/* Write register */
		xfer[0].addr = data->client->addr;
		xfer[0].flags = 0;
		if (data->T5_msg_crc_enabled)
			xfer[0].len = 3;
		else
			xfer[0].len = 2;
		xfer[0].buf = buf;

		/* Read data */
		xfer[1].addr = data->client->addr;
		xfer[1].flags = I2C_M_RD;
		xfer[1].len = len;
		xfer[1].buf = val;
		num_msgs = 2;
	}

	ret = i2c_transfer(data->client->adapter, xfer, num_msgs);
	if (ret == num_msgs) {
		ret = 0;
	} else {
		if (ret >= 0)
			ret = -EIO;
		dev_err(dev, "%s: i2c transfer failed (%d)\n",
			__func__, ret);
	}

	/* Calculate and check message CRC */
	crc_data = 0;
	ptr_data = val;

	/* Use only if message crc enabled */
	/* Incoming len grows by 1, if crc byte enabled */
	if (reg == data->T5_address && data->T5_msg_crc_enabled) {
		for (i = 0; i < len - 1; i++)
			crc_data = __mtch_calc_crc8(crc_data, ptr_data[i]);

		if (crc_data == ptr_data[len - 1]) {
			dev_dbg(dev, "Read msg crc passed [%x] = [%x]\n",
				crc_data, ptr_data[len - 1]);
		} else {
			dev_dbg(dev, "Read msg crc failed [%x] != [%x]\n",
					crc_data, ptr_data[len - 1]);
			/* Pending action if failed ?? */
		}
	}

	return ret;
}

static int __mtch_read_reg(struct mtch_data *data,
			  u16 reg, u16 len, void *val)
{
	return __mtch_read_reg_ex(data, reg, len, val, false);
}

static int __mtch_write_reg(struct mtch_data *data, u16 reg, u16 len,
			   const void *val)
{
	struct i2c_client *client = data->client;
	u8 *buf;
	size_t msg_count = 4;
	u16 data_size = 0;
	u16 bytesToWrite = 0;
	u16 message_length = 0;
	u16 bytesWritten = 0;
	u16 write_addr = 0;
	u8 retry_counter = 0;
	u8 crc_data = 0;
	int i;
	int ret;

	/* Make copy of full message length */
	bytesToWrite = len;
	message_length = len;

	if (IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED)) {
		if (len > data->enc_blocksize)
			message_length = data->enc_blocksize;
	}

	/* Inc. header 16bit addr, opt CRC byte and 2 byte datasize */
	buf = kmalloc((len + 5), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	do {
		/* reg address + offset */
		write_addr = reg + bytesWritten;

		buf[0] = write_addr & 0xff;
		buf[1] = (write_addr >> 8) & 0xff;

		if (data->write_crc_enabled)
			buf[1] = buf[1] | 0x80;

		if (IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED)) {
			if (write_addr >= (data->T38_address +
			    data->T38_obj_size)) {
				if (data->enc_datasize < message_length)
					data_size = data->enc_datasize;
				else
					data_size = message_length;
			} else {
				data_size = 0x0000;
			}

			buf[2] = data_size & 0xff;
			buf[3] = (data_size >> 8) & 0xff;

			msg_count = message_length + 4;

			/* Copy all data to buf */
			memcpy(&buf[4], (val + bytesWritten), len);

		} else { /* unencrypted write , optional crc */
			memcpy(&buf[2], val, len);

			if (data->write_crc_enabled) {
				for (i = 0; i < len + 2; i++)
					crc_data = __mtch_calc_crc8(crc_data,
								   buf[i]);
				/* Address and CRC */
				msg_count = message_length + 3;
				buf[msg_count-1] = crc_data;
			} else {
				/* Address only */
				msg_count = message_length + 2;
			}
		}

		ret = i2c_master_send(client, buf, msg_count);
		if (ret == msg_count) {
			ret = 0;
			bytesWritten += message_length;
			bytesToWrite -= message_length;
			data->enc_datasize -= message_length;

			if (bytesToWrite < message_length)
				message_length = bytesToWrite;

			retry_counter = 0;
		} else {
			if (ret >= 0)
				ret = -EIO;
			dev_err(&client->dev, "%s: i2c send failed (%d)\n",
				__func__, ret);
			retry_counter++;
		}

		if (retry_counter == 3)
			break;

	} while (bytesToWrite > 0);

	kfree(buf);
	return ret;
}

static int mtch_write_addr_ptr(struct mtch_data *data, u16 addr)
{
	u8 buf[4];
	int ret = 0;

	/* Pending -- Encryption support */
	buf[0] = 0x00;	/* datasize LSB */
	buf[1] = 0x00;	/* datasize MSB */
	buf[2] = 0x00;	/* Tx seq */
	buf[3] = 0x00;	/* CRC byte */

	ret = __mtch_write_reg(data, addr, 4, buf);
	if (ret)
		return ret;

	return ret;
}

static int mtch_write_info_addr(struct mtch_data *data)
{
	return mtch_write_addr_ptr(data, 0x0000);
}

static int mtch_write_reg(struct mtch_data *data, u16 reg, u8 val)
{
	return __mtch_write_reg(data, reg, 1, &val);
}

static struct mtch_object *mtch_get_object(struct mtch_data *data, u8 type)
{
	struct mtch_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	return NULL;
}

static struct mtch_object_ext *mtch_get_object_ext(struct mtch_data *data, u16 type)
{
	struct mtch_object_ext *object;
	int i;

	if (!data->ext_object_table)
		return NULL;

	for (i = 0; i < data->ext_object_num; i++) {
		object = data->ext_object_table + i;
		if (object->type == type)
			return object;
	}

	return NULL;
}

static int mtch_check_encryption(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	u32 checksum = 0;
	int ret = 0;
	u8 val[3];
	
	if (data->T2_address) {
		ret = __mtch_read_reg(data, data->T2_address, 1, &val);
		if (ret) {
			dev_info(dev, "Error reading T2 status\n");
			return -EIO;
		}

		if ((val[0] & MSGENCEN) || (val[0] & CONFIGENCEN)) {
			SET_BITS(data->encryption_state, DEV_ENCRYPTED);

			if (val[0] & MSGENCEN)
				SET_BITS(data->encryption_state, MSG_ENCRYPTED);
			else
				CLEAR_BITS(data->encryption_state, MSG_ENCRYPTED);

			if (val[0] & CONFIGENCEN) {
				SET_BITS(data->encryption_state, CFG_ENCRYPTED);
				data->enc_blocksize = 0x30;
			} else {
				CLEAR_BITS(data->encryption_state, CFG_ENCRYPTED);
			}

			ret = __mtch_read_reg(data, data->T2_address +
					     T2_PAYLOADCRC_OFS, 3, &val);
			if (ret) {
				dev_err(dev, "Error reading payload crc\n");
				return -EIO;
			}
			checksum = (val[0] | (val[1] << 8) | (val[2] << 16));
			dev_info(dev, "T2 Payload CRC = 0x%06X", checksum);
			
			ret = __mtch_read_reg(data, data->T2_address +
					     T2_ENCKEYCRC_OFS, 3, &val);
			if (ret) {
				dev_err(dev, "Error reading enc_key_crc\n");
				return -EIO;
			}

			checksum = (val[0] | (val[1] << 8) | (val[2] << 16));
			dev_info(dev, "Enc Key CRC = 0x%06X", checksum);
		}
	}

	return ret;
}

static void mtch_proc_t68_messages(struct mtch_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 err_code = msg[1] & 0x0f;
	u32 data_crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

	if (data->t68_last_frame) {
		if (data_crc != data->t68_data_crc) {
			dev_err(dev, "T68 data crc 0x%06X, does not match file CRC 0x%06X\n",
				data_crc, data->t68_data_crc);
		} else {
			dev_info(dev, "T68 data and file CRC match = 0x%06X\n",
				 data_crc);
		}

		data->t68_last_frame = false;
	}

	complete(&data->t68_completion);

	switch (err_code) {                                                                               
    case T68_SUCCESS:
    	dev_info(dev, "T68 Status 0x%02X: OK\n", err_code);
		break;
	case T68_OUT_OF_SYNC:
		dev_info(dev, "T68 Status 0x%02X: Out of sequence error\n", err_code);
		break;
	case T68_DTYPE_NOTSUP:
		dev_info(dev, "T68 Status 0x%02X: Datatype not supported\n", err_code);
		break;
	case T68_DLEN_MISMATCH:
		dev_info(dev, "T68 Status 0x%02X: Invalid length\n", err_code);
		break;
	case T68_MAX_LENGTH_ERR:
		dev_info(dev, "T68 Status 0x%02X: Invalid number of bytes\n", err_code);
		break;
	case T68_INVALID_DATA:
		dev_info(dev, "T68 Status 0x%02X: Data is invalid\n", err_code);
		break;
	case T68_NO_ACTION_ERR:
		dev_info(dev, "T68 Status 0x%02X: Incomplete action error\n", err_code);
		break;
	default:
		dev_info(dev, "T68 Status 0x%02X: Unknown error\n", err_code);
		break;
	}
}

static void mtch_proc_t6_messages(struct mtch_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

	if (crc != data->dev_cfg_crc[0]) {
		data->dev_cfg_crc[0] = crc;
		dev_dbg(dev, "T6 Config Checksum: 0x%06X\n", crc);
	}

	complete(&data->crc_completion);

	/* Detect reset */
	if (status & T6_STATUS_RESET)
		complete(&data->reset_completion);

	/* Output debug if status has changed */
	if (status != data->t6_status)
		dev_dbg(dev, "T6 Status 0x%02X%s%s%s%s%s%s%s%s\n",
			status,
			status == 0 ? " OK" : "",
			status & T6_STATUS_RESET ? " RESET" : "",
			status & T6_STATUS_OFL ? " OFL" : "",
			status & T6_STATUS_SIGERR ? " SIGERR" : "",
			status & T6_STATUS_CAL ? " CAL" : "",
			status & T6_STATUS_CFGERR ? " CFGERR" : "",
			status & T6_STATUS_COMSERR ? " COMSERR" : "",
			status & T6_STATUS_FROZEN ? " FROZEN" : "");

	if (status & T6_STATUS_SIGERR) {
		dev_info(dev, "SIGERR Fields = 0x%x02, 0x%x02, 0x%x02\n",
			msg[5], msg[6], msg[7]);
	}

	/* Save current status */
	data->t6_status = status;

}

static void mtch_input_button(struct mtch_data *data, u8 *message)
{
	struct input_dev *input = data->input_dev;
	int i;

	for (i = 0; i < data->t19_num_keys; i++) {
		if (data->t19_keymap[i] == KEY_RESERVED)
			continue;

		/* Active-low switch */
		input_report_key(input, data->t19_keymap[i],
				 !(message[1] & BIT(i)));
	}
}

static void mtch_input_sync(struct mtch_data *data)
{

	if (data->update_input)
		input_sync(data->input_dev);
}

static void mtch_proc_t500_message(struct mtch_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	u8 keystate[8];
	u8 status = message[1];
	int byte_idx, bit_idx, key_id;
	u8 changed;

	/* Extract keystate bytes from message (8 bytes = 64 keys max) */
	for (int i = 0; i < 8; i++)
		keystate[i] = message[i + 2];

	dev_dbg(dev, "T500 STATUS:%02X KEYMAP:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
		status, keystate[0], keystate[1], keystate[2], keystate[3],
		keystate[4], keystate[5], keystate[6], keystate[7]);

	/* Compare with previous state and report key changes */
	for (byte_idx = 0; byte_idx < 8; byte_idx++) {
		changed = keystate[byte_idx] ^ data->t500_button_state[byte_idx];
		if (!changed)
			continue;

		for (bit_idx = 0; bit_idx < 8; bit_idx++) {
			if (!(changed & BIT(bit_idx)))
				continue;

			key_id = (byte_idx * 8) + bit_idx;
			dev_dbg(dev, "T500 key %d %s\n", key_id,
				(keystate[byte_idx] & BIT(bit_idx)) ? "pressed" : "released");
		}
	}

	memcpy(data->t500_button_state, keystate, sizeof(data->t500_button_state));
}

static void mtch_proc_t501_message(struct mtch_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input = data->input_dev;
	u8 reportid = message[0];
	u8 status = message[1];
	int key_index;
	struct t501_key_info *key_info;

	if (!input)
		return;

	dev_dbg(dev, "T501:%02X %s DATA:%02X %02X %02X %02X %02X %02X %02X\n",
		reportid,
		(status == T501_STATUS_DETECT ||
		 status == T501_STATUS_FILT_OUT) ? "DOWN" : "UP",
		message[1], message[2], message[3],
		message[4], message[5], message[6], message[7]);

	/*
	 * Convert report ID to T501 instance index (key index).
	 * Report IDs are assigned starting at T501_reportid_min.
	 */
	key_index = reportid - data->T501_reportid_min;

	/* Validate key index */
	if (key_index < 0 || key_index >= data->t501_num_instances) {
		dev_dbg(dev, "T501: reportid 0x%02X out of range (key_index=%d)\n",
			reportid, key_index);
		return;
	}

	key_info = &data->t501_keys[key_index];

	/* Check key usage - only report buttons, ignore wheel/slider keys */
	switch (key_info->usage) {
	case T501_KEY_UNUSED:
		dev_dbg(dev, "T501: key %d not enabled, ignoring\n", key_index);
		return;
	case T501_KEY_WHEEL_SLIDER:
		/* This key belongs to a wheel/slider - ignore T501 messages for it */
		dev_dbg(dev, "T501: key %d belongs to wheel/slider, ignoring\n", key_index);
		return;
	case T501_KEY_BUTTON:
		/* This is a button - process it */
		break;
	default:
		return;
	}

	/*
	 * T501 STATUS field:
	 *   0x03 = NO_DET    - No touch detected (confirmed release)
	 *   0x04 = FILT_IN   - Touch delta >= threshold, DI starting (transitional)
	 *   0x85 = DETECT    - Touch confirmed (held for TCHDI+1 cycles)
	 *   0x86 = FILT_OUT  - Release filtering, delta below threshold (transitional)
	 *
	 * Typical press:   0x04 -> 0x85
	 * Typical release: 0x86 -> 0x03
	 *
	 * Report key DOWN on 0x85 (confirmed touch)
	 * Report key UP on 0x03 (confirmed release)
	 * Ignore transitional states 0x04 and 0x86
	 */
	switch (status) {
	case T501_STATUS_DETECT:	/* 0x85 - confirmed touch */
		dev_dbg(dev, "T501: key %d DOWN (keycode %u)\n",
			key_index, key_info->keycode);
		input_report_key(input, key_info->keycode, 1);
		data->update_input = true;
		break;
	case T501_STATUS_NO_DET:	/* 0x03 - confirmed release */
		dev_dbg(dev, "T501: key %d UP (keycode %u)\n",
			key_index, key_info->keycode);
		input_report_key(input, key_info->keycode, 0);
		data->update_input = true;
		break;
	default:
		/* Ignore transitional states: 0x04 (FILT_IN), 0x86 (FILT_OUT), etc. */
		break;
	}
}

/**
 * mtch_t501_build_key_map - Build T501 key mapping after T502 is configured
 * @data: driver data structure
 *
 * This function must be called AFTER mtch_t502_get_valid_instances().
 *
 * Algorithm:
 * 1. Mark all T501 keys used by enabled T502 wheels/sliders as WHEEL_SLIDER
 * 2. For remaining enabled T501 keys, assign as buttons:
 *    - First 7 buttons → mtch_key_codes[0..6] (KEY_1 to KEY_7)
 *    - Next 4 buttons → mtch_button_codes[0..3] (BTN_0 to BTN_3)
 * 	  - Note: May be swapped between EVK sensors 3240P vs 3380P
 * 3. Error if more than 11 buttons needed
 *
 * Return: 0 on success, negative error code on failure
 */
static int mtch_t501_build_key_map(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	struct mtch_object_ext *object;
	u8 num_instances;
	u8 ctrl_byte;
	u16 instance_addr;
	size_t instance_size;
	int button_index = 0;
	int max_buttons;
	int ret, i, j, k;

	/* Calculate max buttons: 7 key_codes + 4 button_codes = 11 */
	max_buttons = ARRAY_SIZE(mtch_key_codes) + ARRAY_SIZE(mtch_button_codes);

	object = mtch_get_object_ext(data, SPT_MTCHKEYCTRL_T501);
	if (!object) {
		dev_dbg(dev, "T501 object not found\n");
		return -ENOENT;
	}

	num_instances = mtch_obj_ext_instances(object);
	instance_size = mtch_obj_ext_size(object);

	if (num_instances > T501_MAX_INSTANCES) {
		dev_err(dev, "T501: %u instances exceeds max %d\n",
			num_instances, T501_MAX_INSTANCES);
		return -EINVAL;
	}

	/* Initialize all keys as unused */
	memset(data->t501_keys, 0, sizeof(data->t501_keys));
	data->t501_num_instances = num_instances;
	data->t501_num_buttons = 0;

	dev_dbg(dev, "T501: checking %u instances, size %zu bytes each\n",
		num_instances, instance_size);

	/*
	 * Step 1: Mark keys used by enabled T502 wheels/sliders
	 */
	for (i = 0; i < data->t502_num_instances; i++) {
		if (!data->t502_instances[i].valid)
			continue;

		u8 startkey = data->t502_instances[i].startkey;
		u8 numkeys = data->t502_instances[i].numkeys;

		for (j = 0; j < numkeys; j++) {
			k = startkey + j;
			if (k < T501_MAX_INSTANCES) {
				data->t501_keys[k].usage = T501_KEY_WHEEL_SLIDER;
				dev_dbg(dev, "T501 key %d: assigned to T502 %s\n",
					k, data->t502_instances[i].type == T502_TYPE_WHEEL ?
					"wheel" : "slider");
			}
		}
	}

	/*
	 * Step 2: Read T501 instances and assign remaining enabled keys as buttons
	 */
	for (i = 0; i < num_instances; i++) {
		/* Skip keys already assigned to wheel/slider */
		if (data->t501_keys[i].usage == T501_KEY_WHEEL_SLIDER)
			continue;

		instance_addr = object->start_address + (i * instance_size);

		ret = __mtch_read_reg(data, instance_addr, 1, &ctrl_byte);
		if (ret) {
			dev_err(dev, "T501: failed to read instance %d ctrl byte\n", i);
			return ret;
		}

		dev_dbg(dev, "T501 instance %d: addr=0x%04X ctrl=0x%02X\n",
			i, instance_addr, ctrl_byte);

		/* Check if enabled and reporting */
		if ((ctrl_byte & T501_CTRL_ENABLE) && (ctrl_byte & T501_CTRL_RPTEN)) {
			/* Check if we have room for another button */
			if (button_index >= max_buttons) {
				dev_err(dev, "T501: too many buttons! instance %d exceeds max %d\n",
					i, max_buttons);
				return -EINVAL;
			}

			data->t501_keys[i].usage = T501_KEY_BUTTON;

			/* Assign keycode: first 4 from button_codes, next 7 from key_codes */
			if (button_index < ARRAY_SIZE(mtch_button_codes)) {
				data->t501_keys[i].keycode = mtch_button_codes[button_index];
			} else {
				data->t501_keys[i].keycode =
					mtch_key_codes[button_index - ARRAY_SIZE(mtch_button_codes)];
			}

			dev_info(dev, "T501 key %d: button #%d (keycode %u)\n",
				 i, button_index, data->t501_keys[i].keycode);

			button_index++;
		} else {
			data->t501_keys[i].usage = T501_KEY_UNUSED;
			dev_dbg(dev, "T501 instance %d: disabled (en=%d rpt=%d)\n",
				i, !!(ctrl_byte & T501_CTRL_ENABLE),
				!!(ctrl_byte & T501_CTRL_RPTEN));
		}
	}

	data->t501_num_buttons = button_index;

	dev_info(dev, "T501: %d button(s) configured (max %d)\n",
		 button_index, max_buttons);

	return 0;
}

/**
 * mtch_t502_get_valid_instances - Read T502 instances and determine type/validity
 * @data: driver data structure
 *
 * Reads configuration bytes of each T502 instance:
 *   - CTRL (byte 0): TYPE (bits 7:6), ENABLE (bit 0), RPTEN (bit 1)
 *   - STARTKEY (byte 1): first T501 key index used by this wheel/slider
 *   - NUMKEYS (byte 2): number of keys in this wheel/slider
 *
 * Populates data->t502_instances[] with type, validity, and key range info.
 * Also counts total wheels and sliders for dynamic allocation.
 *
 * Return: 0 on success, negative error code on failure
 */
static int mtch_t502_get_valid_instances(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	struct mtch_object_ext *object;
	u8 num_instances;
	u8 cfg_bytes[3];	/* CTRL, STARTKEY, NUMKEYS */
	u16 instance_addr;
	size_t instance_size;
	int num_wheels = 0, num_sliders = 0;
	int ret, i;

	object = mtch_get_object_ext(data, SPT_MTCHSLIDERCTRL_T502);
	if (!object) {
		dev_dbg(dev, "T502 object not found\n");
		return -ENOENT;
	}

	num_instances = mtch_obj_ext_instances(object);
	instance_size = mtch_obj_ext_size(object);

	if (num_instances > T502_MAX_INSTANCES) {
		dev_err(dev, "T502: %u instances exceeds max %d\n",
			num_instances, T502_MAX_INSTANCES);
		return -EINVAL;
	}

	memset(data->t502_instances, 0, sizeof(data->t502_instances));
	data->t502_num_instances = num_instances;

	dev_dbg(dev, "T502: checking %u instances, size %zu bytes each\n",
		num_instances, instance_size);

	for (i = 0; i < num_instances; i++) {
		u8 resolution_byte;

		instance_addr = object->start_address + (i * instance_size);

		/* Read CTRL, STARTKEY, NUMKEYS, and RESOLUTION (bytes 0-3) */
		ret = __mtch_read_reg(data, instance_addr, 4, cfg_bytes);
		if (ret) {
			dev_err(dev, "T502: failed to read instance %d config\n", i);
			return ret;
		}

		resolution_byte = cfg_bytes[3];

		dev_dbg(dev, "T502 instance %d: addr=0x%04X ctrl=0x%02X startkey=%u numkeys=%u resol=0x%02X\n",
			i, instance_addr, cfg_bytes[0], cfg_bytes[1], cfg_bytes[2], resolution_byte);

		/* Extract type from bits 7:6 */
		data->t502_instances[i].type =
			(cfg_bytes[T502_CFG_CTRL] & T502_CTRL_TYPE_MASK) >> T502_CTRL_TYPE_SHIFT;

		/* Store STARTKEY and NUMKEYS */
		data->t502_instances[i].startkey = cfg_bytes[T502_CFG_STARTKEY];
		data->t502_instances[i].numkeys = cfg_bytes[T502_CFG_NUMKEYS];

		/* Extract RANGE from RESOLUTION byte bits 7:4 */
		data->t502_instances[i].range = (resolution_byte >> 4) & 0x0F;
		/* Calculate max position: 2^range - 1 (e.g., range=6 gives max_pos=63) */
		data->t502_instances[i].max_pos = (1 << data->t502_instances[i].range) - 1;

		/* Check if enabled and reporting */
		if ((cfg_bytes[T502_CFG_CTRL] & T502_CTRL_ENABLE) &&
		    (cfg_bytes[T502_CFG_CTRL] & T502_CTRL_RPTEN)) {
			data->t502_instances[i].valid = true;

			if (data->t502_instances[i].type == T502_TYPE_WHEEL) {
				data->t502_instances[i].dev_index = num_wheels;
				num_wheels++;
			} else {
				data->t502_instances[i].dev_index = num_sliders;
				num_sliders++;
			}

			dev_info(dev, "T502 instance %d: %s (dev_index=%u, keys %u-%u, range=%u, max_pos=%u)\n",
				 i,
				 data->t502_instances[i].type == T502_TYPE_WHEEL ?
				 "wheel" : "slider",
				 data->t502_instances[i].dev_index,
				 data->t502_instances[i].startkey,
				 data->t502_instances[i].startkey +
				 data->t502_instances[i].numkeys - 1,
				 data->t502_instances[i].range,
				 data->t502_instances[i].max_pos);
		} else {
			data->t502_instances[i].valid = false;
			dev_dbg(dev, "T502 instance %d: disabled (en=%d rpt=%d)\n",
				i, !!(cfg_bytes[T502_CFG_CTRL] & T502_CTRL_ENABLE),
				!!(cfg_bytes[T502_CFG_CTRL] & T502_CTRL_RPTEN));
		}
	}

	data->t502_num_wheels = num_wheels;
	data->t502_num_sliders = num_sliders;

	dev_info(dev, "T502: %d wheel(s), %d slider(s) configured\n",
		 num_wheels, num_sliders);

	return 0;
}

static void mtch_proc_t502_message(struct mtch_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	u8 reportid = message[0];
	u8 status = message[1];
	u16 pos = message[2] | (message[3] << 8);
	int id = reportid - data->T502_reportid_min;
	struct t502_instance_info *inst;
	struct input_dev *input_dev;
	bool currently_detected;
	bool was_detected;

	if (data->debug_enabled) {
		dev_dbg(dev, "T502:%02X DATA:%02X %02X %02X %02X %02X %02X %02X\n",
			reportid, message[1], message[2], message[3],
			message[4], message[5], message[6], message[7]);
	}

	/* Validate instance id */
	if (id < 0 || id >= data->t502_num_instances) {
		dev_err(dev, "T502: invalid instance id %d (max %d)\n",
			id, data->t502_num_instances - 1);
		return;
	}

	inst = &data->t502_instances[id];

	/* Skip if this instance is not configured as valid */
	if (!inst->valid) {
		dev_dbg(dev, "T502: instance %d not valid, ignoring\n", id);
		return;
	}

	/* Get the appropriate input device based on type */
	if (inst->type == T502_TYPE_WHEEL) {
		if (!data->input_dev_wheels ||
		    inst->dev_index >= data->t502_num_wheels) {
			dev_dbg(dev, "T502: no wheel device for instance %d\n", id);
			return;
		}
		input_dev = data->input_dev_wheels[inst->dev_index];
	} else {
		if (!data->input_dev_sliders ||
		    inst->dev_index >= data->t502_num_sliders) {
			dev_dbg(dev, "T502: no slider device for instance %d\n", id);
			return;
		}
		input_dev = data->input_dev_sliders[inst->dev_index];
	}

	if (!input_dev)
		return;

	currently_detected = (status & T502_STATUS_DETECT) != 0;
	was_detected = data->t502_in_detect[id];

	if (currently_detected) {
		if (data->debug_enabled) {
			dev_dbg(dev, "T502:%d (%s) DETECT pos:%u%s%s\n",
				id, inst->type == T502_TYPE_WHEEL ? "wheel" : "slider",
				pos,
				status & T502_STATUS_DIRCHG ? " DIRCHG" : "",
				status & T502_STATUS_POSCHG ? " POSCHG" : "");
		}

		if (!was_detected) {
			/* First touch - record position */
			data->t502_last_pos[id] = pos;
			data->t502_in_detect[id] = true;

			if (data->debug_enabled)
				dev_dbg(dev, "T502: %s %d initial touch at %u\n",
					inst->type == T502_TYPE_WHEEL ? "wheel" : "slider",
					inst->dev_index, pos);

			if (inst->type == T502_TYPE_WHEEL) {
				/* For absolute mode, report initial position */
				if (!data->wheel_relative)
					input_report_abs(input_dev, ABS_WHEEL, pos);
			} else {
				input_report_abs(input_dev, ABS_X, pos);
			}
			input_report_key(input_dev, BTN_TOUCH, 1);
			input_sync(input_dev);
		} else if (status & T502_STATUS_POSCHG) {
			/* Position changed */
			if (inst->type == T502_TYPE_WHEEL && data->wheel_relative) {
				/* Calculate relative movement for wheel with wraparound */
				int delta = (int)pos - (int)data->t502_last_pos[id];
				int half_range = (inst->max_pos + 1) / 2;

				/*
				 * Handle wraparound: if delta magnitude exceeds half
				 * the range, the shorter path crosses the boundary.
				 */
				if (delta > half_range)
					delta -= (inst->max_pos + 1);
				else if (delta < -half_range)
					delta += (inst->max_pos + 1);

				/* Clamp delta to ±5 to filter spurious large jumps */
				if (delta > 5)
					delta = 5;
				else if (delta < -5)
					delta = -5;

				if (delta != 0) {
					if (data->debug_enabled)
						dev_dbg(dev, "T502: wheel rel:%d (was:%u now:%u max:%u)\n",
							delta, data->t502_last_pos[id], pos,
							inst->max_pos);

					input_report_rel(input_dev, REL_WHEEL, delta);
					input_sync(input_dev);
				}
			} else {
				/* Report absolute position */
				if (data->debug_enabled)
					dev_dbg(dev, "T502: %s %d abs:%u\n",
						inst->type == T502_TYPE_WHEEL ? "wheel" : "slider",
						inst->dev_index, pos);

				if (inst->type == T502_TYPE_WHEEL)
					input_report_abs(input_dev, ABS_WHEEL, pos);
				else
					input_report_abs(input_dev, ABS_X, pos);
				input_sync(input_dev);
			}
			data->t502_last_pos[id] = pos;
		}
	} else {
		/* No detect - finger lifted */
		if (was_detected) {
			if (data->debug_enabled)
				dev_dbg(dev, "T502:%d (%s) released at %u\n",
					id, inst->type == T502_TYPE_WHEEL ? "wheel" : "slider",
					pos);

			data->t502_in_detect[id] = false;
			input_report_key(input_dev, BTN_TOUCH, 0);
			input_sync(input_dev);
		}
	}
}

static int mtch_t6_command(struct mtch_data *data, u16 cmd_offset,
			  u8 value, bool wait)
{
	u16 reg;
	u8 command_register;
	int timeout_counter = 0;
	int ret;

	reg = data->T6_address + cmd_offset;

	if (!data->crc_enabled)
		ret = mtch_write_reg(data, reg, value);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(20);
		if (!(data->crc_enabled))
			ret = __mtch_read_reg(data, reg, 1, &command_register);
		if (ret)
			return ret;

	} while (command_register != 0 && timeout_counter++ <= 100);

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "T6 command failed!\n");
		return -EIO;
	}

	return 0;
}

static void mtch_update_crc(struct mtch_data *data, u8 cmd, u8 value,
			   bool clear_crc)
{
	int i;
	/*
	 * On failure, CRC is set to 0 and config will always be
	 * downloaded.
	 */

	if (clear_crc) {
		for (i = 0; i < data->hc.num_of_devs; i++) {
			if (i == HC_DEV_MAX)
				break;
			data->nvm_dev_crc[i] = 0;
			data->dev_cfg_crc[i] = 0;
		}
	}

	if ((!data->crc_enabled))
		reinit_completion(&data->crc_completion);

	mtch_t6_command(data, cmd, value, true);

	/*
	 * Wait for crc message. On failure, CRC is set to 0 and config will
	 * always be downloaded.
	 */

	if (!(data->crc_enabled))
		mtch_wait_for_completion(data, &data->crc_completion,
					CRC_TIMEOUT);
}

static void mtch_dump_message(struct mtch_data *data, u8 *message)
{
	dev_dbg(&data->client->dev, "message: %*ph\n",
		data->T5_msg_size, message);
}

static void mtch_debug_msg_add(struct mtch_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;

	mutex_lock(&data->debug_msg_lock);

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return;
	}

	if (data->debug_msg_count < DEBUG_MSG_MAX) {
		memcpy(data->debug_msg_data +
		       data->debug_msg_count * data->T5_msg_size,
		       msg,
		       data->T5_msg_size);
		data->debug_msg_count++;
	} else {
		dev_dbg(dev, "Discarding %u messages\n", data->debug_msg_count);
		data->debug_msg_count = 0;
	}

	mutex_unlock(&data->debug_msg_lock);

	sysfs_notify(&data->client->dev.kobj, NULL, "debug_notify");
}

static int mtch_proc_message(struct mtch_data *data, u8 *message)
{
	u8 report_id = message[0];
	bool dump = data->debug_enabled;
	bool handled = false;

	switch (report_id) {
	case RPTID_NOMSG:
	case RPTID_RVSD:
		return 0;
	default:
		break;
	}

	if (report_id == data->T6_reportid) {
		mtch_proc_t6_messages(data, message);
		handled = true;
	} else if (report_id == data->T68_reportid_min) {
		mtch_proc_t68_messages(data, message);
		handled = true;
	} else if (data->T500_reportid_min &&
		report_id == data->T500_reportid_min) {
		mtch_proc_t500_message(data, message);
		handled = true;
	} else if (data->T501_reportid_min &&
		report_id >= data->T501_reportid_min &&
		report_id <= data->T501_reportid_max) {
		mtch_proc_t501_message(data, message);
		handled = true;
	} else if (data->T502_reportid_min &&
		report_id >= data->T502_reportid_min &&
		report_id <= data->T502_reportid_max) {
		mtch_proc_t502_message(data, message);
		handled = true;
	} else if (report_id == data->T19_reportid_min) {
		mtch_input_button(data, message);
		data->update_input = true;
		handled = true;
	}

	if (!handled || dump)
		mtch_dump_message(data, message);

	if (data->debug_v2_enabled)
		mtch_debug_msg_add(data, message);

	return 1;
}

static int mtch_read_and_process_messages(struct mtch_data *data, u8 count,
					 bool crc8)
{
	struct device *dev = &data->client->dev;
	int ret;
	int i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	for (i = 0; i < count; i++) {

		/* Process remaining messages if necessary */
		ret = __mtch_read_reg(data, data->T5_address,
			data->T5_msg_size, data->msg_buf);

		if (ret) {
			dev_err(dev, "Failed to read %u messages (%d)\n",
				count, ret);
			return ret;
		}

		if (data->msg_buf[0] == 0xFF)
			break;

		ret = mtch_proc_message(data, data->msg_buf);

		if (ret == 1)
			num_valid++;
	}

	return num_valid;
}

static irqreturn_t mtch_process_messages_t44(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 count, num_left;

	/* Read T44 and T5 together for legacy devices */

	if (data->T5_msg_crc_enabled) {	/* Read only count */
		ret = __mtch_read_reg(data, data->T44_address,
			1, data->msg_buf);
	} else {	/* Read count and first msg */
		ret = __mtch_read_reg(data, data->T44_address,
			data->T5_msg_size + 1, data->msg_buf);
	}

	if (ret) {
		dev_dbg(dev, "Failed to read T44 and T5 (%d)\n", ret);
		return IRQ_HANDLED;
	}

	count = data->msg_buf[0];

	if (count > 0) {
		if (data->T5_msg_crc_enabled) {
			ret = __mtch_read_reg(data, data->T5_address,
				data->T5_msg_size, data->msg_buf + 1);
		}
	}

	if (ret) {
		dev_dbg(dev, "Failed to read T5, (%d)\n", ret);
		return IRQ_HANDLED;
	}

	/*
	 * This condition may be caused by the CHG line being configured in
	 * Mode 0. It results in unnecessary I2C operations but it is benign.
	 */
	if (count == 0) {
		dev_dbg(dev, "Interrupt occurred but no message\n");
		return IRQ_HANDLED;
	}

	if (count > data->max_reportid) {
		dev_warn(dev, "T44 count %d exceeded max report id\n",
			 count);

		return IRQ_HANDLED;
	}

	/* Process first message, first byte rsvd for count */
	ret = mtch_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		dev_warn(dev, "Process: Unexpected invalid message\n");
		return IRQ_HANDLED;
	}

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
		dev_dbg(dev, "Remaining messages to process\n");

		ret = mtch_read_and_process_messages(data, num_left, true);
		if (ret < 0)
			goto end;
		else if (ret != num_left)
			dev_dbg(dev, "Read: Unexpected invalid message\n");
	}

end:
	if (data->update_input) {
		mtch_input_sync(data);
		data->update_input = false;
	}

	data->skip_crc_write = false;

	return IRQ_HANDLED;
}

static int mtch_process_messages_until_invalid(struct mtch_data *data,
					      bool bypass)
{
	struct device *dev = &data->client->dev;
	int count, read;
	u8 tries = 2;

	count = data->max_reportid;

	/* Read messages until we force an invalid */
	do {
		read = mtch_read_and_process_messages(data, count, true);
		if (read < count && bypass) {
			if (read == 0)
				return 0;
		}
	} while (--tries);

	if (data->update_input) {
		mtch_input_sync(data);
		data->update_input = false;
	}

	/* Unread messages in T5 buffer, make sure RETRIGEN is enabled */
	dev_warn(dev, "CHG pin isn't cleared\n");

	return 0;
}

static irqreturn_t mtch_process_messages(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	int total_handled, num_handled;
	u8 count = data->last_message_count;

	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mtch_read_and_process_messages(data, count + 1, true);
	if (total_handled < 0) {
		dev_dbg(dev, "Interrupt occurred but no message\n");
		return IRQ_HANDLED;
	}

	/* if there were invalid messages, then we are done */
	else if (total_handled <= count)
		goto update_count;

	/* keep reading two msgs until one is invalid or reportid limit */
	do {
		num_handled = mtch_read_and_process_messages(data, 2, true);
		if (num_handled < 0) {
			dev_dbg(dev, "Interrupt occurred but no message\n");
			return IRQ_HANDLED;
		}

		total_handled += num_handled;

		if (num_handled < 2)
			break;
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->update_input) {
		mtch_input_sync(data);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static irqreturn_t mtch_interrupt(int irq, void *dev_id)
{
	struct mtch_data *data = dev_id;

	if (data->in_bootloader) {
		/* bootloader state transition completion */
		complete(&data->bl_completion);
		return IRQ_HANDLED;
	}

	if (!data->object_table)
		return IRQ_HANDLED;

	if (data->irq_processing) {
		if (data->T44_address)
			return mtch_process_messages_t44(data);
		else
			return mtch_process_messages(data);
	}

	return IRQ_HANDLED;
}

static int mtch_acquire_irq(struct mtch_data *data)
{
	int error;

	error = mtch_process_messages_until_invalid(data, true);
	if (error)
		return error;

	if (!data->irq_enabled) {
		enable_irq(data->irq);
		data->irq_enabled = true;
	}

	return 0;
}

static int mtch_soft_reset(struct mtch_data *data, bool reset_enabled)
{
	struct device *dev = &data->client->dev;
	int ret = 0;

	dev_info(dev, "Resetting device\n");

	reinit_completion(&data->reset_completion);

	ret = mtch_t6_command(data, CMD_RESET_OFS, RESET_VALUE, false);
	if (ret)
		return ret;

	/* Ignore CHG line after reset */
	msleep(RESET_INVALID_CHG);

	ret = mtch_wait_for_completion(data, &data->reset_completion,
				      RESET_TIMEOUT);
	if (ret)
		return ret;

	return 0;
}

static void mtch_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mtch_calculate_crc(u8 *base, off_t start_off, off_t end_off, u32 crc)
{
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mtch_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mtch_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

static int mtch_check_retrigen(struct mtch_data *data)
{
	struct i2c_client *client = data->client;
	int ret;
	int val;
	int buf;

	/* Ignore when using level triggered mode */
	if (irq_get_trigger_type(data->irq) & IRQF_TRIGGER_LOW) {
		dev_info(&client->dev, "Level triggered\n");
		return 0;
	}

	if (data->T18_address) {
		ret = __mtch_read_reg(data,
			data->T18_address + COMMS_CTRL_OFS,
			1, &val);

		if (ret)
			return ret;

		if ((val & COMMS_RETRIGEN) != COMMS_RETRIGEN) {
			dev_info(&client->dev, "Enabling RETRIGEN feature\n");

			buf = val | COMMS_RETRIGEN;

			ret = __mtch_write_reg(data,
							   data->T18_address +
							   COMMS_CTRL_OFS,
							   1, &buf);
			if (ret)
				return ret;
		} else {
			dev_info(&client->dev, "RETRIGEN bit already enabled\n");
		}
	}

	return 0;
}

static bool mtch_object_is_volatile(struct mtch_data *data, uint16_t object_type)
{
	switch (object_type) {
	case GEN_ENCRYPTIONSTATUS_T2:
	case GEN_MESSAGE_T5:
	case GEN_COMMAND_T6:
	case DEBUG_DIAGNOSTIC_T37:
	case SPT_MESSAGECOUNT_T44:
	case SPT_SERIALDATACOMMAND_T68:

		return true;

	default:
		return false;

	}
}

/*
 * Check chip is not in deep sleep
 */
static int mtch_t68_check_power_cfg(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	uint8_t buf[2];
	int ret = 0;

	ret = __mtch_read_reg(data, data->T7_address, sizeof(buf),
		&buf);
	if (ret)
		return ret;

	dev_info(dev, "T7 IDLEACQINT=%u ACTVACQINT=%u",
		 buf[0], buf[1]);

	if (buf[0] == 0 || buf[1] == 0) {
		dev_err(dev, "Chip in deep sleep, T68 cannot be programmed");

		return -EINVAL;
	} else {
		return 0;
	}
}

static int mtch_t68_enable(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	u8 cmd = T68_CTRL_RPTEN | T68_CTRL_ENABLE;
	int ret = 0;

	dev_dbg(dev, "Writing %u to ctrl register", cmd);

	ret = __mtch_write_reg(data, data->T68_address +
				     T68_CTRL_OFS, 1, &cmd);

	dev_info(dev, "Enabling T68 object");

	return ret;
}

static int mtch_t68_write_datatype(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;

	u8 buf[2];
	int ret = 0;

	buf[0] = (data->t68_datatype & 0xFF);
	buf[1] = (data->t68_datatype & 0xFF00) >> 8;

	dev_dbg(dev, "Writing %u to DATATYPE register\n",
		data->t68_datatype);

	ret = __mtch_write_reg(data, data->T68_address +
			      T68_DTYPE_OFS, 2, &buf);
	if (ret) {
		dev_err(dev, "Write T68 datatype failed\n");
	}

	return ret;
}

static int mtch_t68_write_length(struct mtch_data *data, u16 length)
{
	struct device *dev = &data->client->dev;
	int ret = 0;

	dev_dbg(dev, "Writing LENGTH=%u", length);

	ret = __mtch_write_reg(data, data->T68_address +
			      T68_LENGTH_OFS, 1, &length);
	if (ret) {
		dev_err(dev, "Write to T68 length failed\n");
	}
	return ret;
}

static int mtch_t68_command(struct mtch_data *data, u8 cmd)
{
	struct device *dev = &data->client->dev;
	int ret = 0;

	dev_dbg(dev, "Writing 0x%02X to CMD register", cmd);

	reinit_completion(&data->t68_completion);

	ret = __mtch_write_reg(data, data->T68_address +
				      T68_CMD_OFS, 1, &cmd);
	if (ret) {
		dev_err(dev, "Write CMD register failed\n");
		return ret;
	}

	/* Return error if timeout */
	ret = mtch_wait_for_completion(data, &data->t68_completion,
	      T68_TIMEOUT);

	if (ret == -ETIMEDOUT) {
		dev_warn(dev, "T68 payload may not have been programmed\n");
		ret = 0;
	}

	return ret;
}

static int mtch_t68_send_frames(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	size_t offset = 0;
	u8 data_size;
	int frame = 1;
	u32 temp_crc = 0;
	u8 cmd = T68_CMD_NONE;
	int ret = 0;

	data->t68_last_frame = false;

	while (cmd != T68_CMD_END) {
		/* Limit size to T68 data size */
		data_size = MIN(data->t68_length - offset, data->t68_datasize);

		dev_dbg(dev, "Writing frame %u, data_size %u bytes", frame,
			data_size);

		/* Write and calculate data only if frame_size > 0 */
		if (data_size > 0) {
			/* Accumulate CRC per frame */
			temp_crc = mtch_calculate_crc((data->t68_buf + offset),
						     0, data_size,
						     data->t68_checksum);

			/* Update T68 checksum for each frame written */
			data->t68_checksum = temp_crc;

			dev_dbg(dev, "Calcuted CRC for frame [%d] = %06X",
				data->t68_checksum, frame);
		}

		ret = __mtch_write_reg(data, data->T68_address +
				      T68_DATA_OFS,
				      data->t68_datasize,
				      (data->t68_buf + offset));
		if (ret) {
			dev_err(dev, "Write frames failed\n");
			return ret;
		}

		/* Always update length before sending command, even if 0*/
		ret = mtch_t68_write_length(data, data_size);

		if (ret) {
			dev_err(dev, "Write t68 length failed\n");
			return ret;
		}

		if (frame == 1) {
			cmd = T68_CMD_START;
		} else if ((data->t68_length - offset) < data->t68_datasize) {
			cmd = T68_CMD_END;
			data->t68_last_frame = true;
		} else {
			cmd = T68_CMD_CONTINUE;
		}

		offset += data_size;

		ret = mtch_t68_command(data, cmd);
		if (ret) {
			dev_err(dev, "Write t68 command failed\n");
			return ret;
		}

		frame++;
	}

	return 0;
}

/*
 * Zero entire T68 object
 */
static int mtch_t68_zero_data(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;

	u8 zeros[64];
	int ret = 0;

	dev_dbg(dev, "Zeroing T68 DATA");

	memset(&zeros, 0, sizeof(zeros));

	ret = __mtch_write_reg(data, data->T68_address +
				      T68_DATA_OFS,
				      data->t68_datasize, zeros);
	if (ret)
		dev_err(dev, "Failed to clear T68 data\n");

	return ret;
}

static int mtch_upload_t68_payload(struct mtch_data *data, 
									struct mtch_cfg *cfg)
{
	struct device *dev = &data->client->dev;
	struct mtch_object *object;
	int ret = 0;

	if (!(IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED))) {
		if (data->hc.devid_num == 0) {
			dev_info(dev, "Checking T7 Power Config");
			ret = mtch_t68_check_power_cfg(data);
			if (ret)
				return ret;
		} else {
			/* Enable T7 only on Host */
			dev_warn(dev, "Checking T7 power skipped");
		}
	}

	/* Check for existence of T68 object */
	object = mtch_get_object(data, SPT_SERIALDATACOMMAND_T68);
	if (!object) {
		dev_err(dev, "T68 object does not exist");
		return -EINVAL;
	}

	/* Calculate position of CMD register */
	data->t68_cmd_addr = data->T68_address + data->T68_obj_size - 3;

	/* Calculate frame size */
	data->t68_datasize = data->T68_obj_size - 9;

	ret = mtch_t68_enable(data);
	if (ret) {
		dev_err(dev, "Error enabling T68 object\n");
		goto release;
	}

	/* Zero only once */
	ret = mtch_t68_zero_data(data);
	if (ret) {
		dev_err(dev, "Error zeroing content of T68\n");
		goto release;
	}

	ret = mtch_t68_write_datatype(data);
	if (ret) {
		dev_err(dev, "Error writing datatype\n");
		goto release;
	}

	dev_info(dev, "Writing T68 payload data");

	ret = mtch_t68_send_frames(data);
	if (ret) {
		dev_err(dev, "Error sending T68 payload data");
		goto release;
	}

	dev_info(dev, "T68 device[%d] payload programming done\n",
		data->hc.devid_num);

	ret = 0;

release:
	kfree(data->t68_buf);

	return ret;
}

static bool is_type_used_for_crc(const uint32_t type)
{
	switch (type) {
	case GEN_ENCRYPTIONSTATUS_T2:
	case GEN_MESSAGE_T5:
	case GEN_COMMAND_T6:
	case DEBUG_DIAGNOSTIC_T37:
	case SPT_USERDATA_T38:
	case SPT_MESSAGECOUNT_T44:
	case SPT_SERIALDATACOMMAND_T68:
		return false;

	default:
		return true;
	}
}

static int mtch_prepare_cfg_mem(struct mtch_data *data, struct mtch_cfg *cfg)
{
	struct device *dev = &data->client->dev;
	struct mtch_object *object;
	struct mtch_object_ext *object_ext;
	unsigned int type, instance, size;
	unsigned int bytesperline = 0;
	int offset, tmp_offset, wr_offset = 0;
	int totalBytesToWrite = 0;
	unsigned int first_obj_type = 0;
	unsigned int first_obj_addr = 0;
	int ret, error = 0;
	int byte_offset = 0;
	char newline;
	u8 tmpval, val;
	int dev_id = 0;
	char tmp[255];
	u16 reg;
	int i = 0;
	/* Variables to hold effective object properties from either table */
	u16 obj_start_address;
	size_t obj_size;
	size_t obj_instances;

	cfg->object_skipped_ofs = 0;

	/* Loop until end of raw file */
	while (cfg->raw_pos < cfg->raw_size) {
	       dev_dbg(dev, "%s %d %s %d %s %d %s %zu\n",
		       "cfg->object_skipped_ofs", cfg->object_skipped_ofs,
		       "first_obj_addr", first_obj_addr,
		       "cfg->start_ofs", cfg->start_ofs,
		       "cfg->mem_size", cfg->mem_size);

		ret = sscanf(cfg->raw + cfg->raw_pos, "%s%n", tmp, &offset);

		if ((!strncmp(tmp, "DEVICE_0", 8)) ||
		    (!strncmp(tmp, "[DEVICE_0", 9))) {
			dev_id = 0;
			cfg->raw_pos += offset;
			cfg->object_skipped_ofs = 0;
			first_obj_type = 0;
			cfg->mem_size = data->mem_size;

			continue;
		} else if ((!strncmp(tmp, "DEVICE_1", 8)) ||
			   (!strncmp(tmp, "[DEVICE_1", 9))) {
			dev_id = 1;
			cfg->raw_pos += offset;

			continue;
		} else {
			/* Read type, instance, length */
			ret = sscanf(cfg->raw + cfg->raw_pos, "%x %x %x%n",
			      &type, &instance, &size, &offset);

			if (ret == 0) {
				/* EOF */
				break;
			} else if (ret != 3) {
				dev_err(dev,
					"Bad format: Cannot parse object\n");
				return -EINVAL;
			}
		}

		/* Update position in raw file to start of obj data */
		cfg->raw_pos += offset;

		/*
		 * Look up object in both standard and extended object tables.
		 * Extended objects (T500, T501, T502, etc.) have 16-bit types
		 * and are stored in the T254 extended object table.
		 */
		object = NULL;
		object_ext = NULL;

		/* First try standard object table (8-bit types) */
		if (type <= 255)
			object = mtch_get_object(data, (u8)type);

		/* If not found, try extended object table (16-bit types) */
		if (!object && data->ext_object_table)
			object_ext = mtch_get_object_ext(data, (u16)type);

		if ((!object && !object_ext) ||
		    ((mtch_object_is_volatile(data, type)) &&
		    (instance & 0x8000) != 0x8000)) {
			/* Skip object if not present in device or volatile */

			dev_dbg(dev, "Skipping object T[%d] Instance %d\n",
				type, instance);

			for (i = 0; i < size; i++) {
				ret = sscanf(cfg->raw + cfg->raw_pos, "%hhx%n",
					     &val, &offset);
				if (ret != 1) {
					dev_err(dev,
						"Bad format in T%d at %d\n",
						type, i);
					return -EINVAL;
				}
				/*Update position in raw file to next obj */
				cfg->raw_pos += offset;

				/* Adjust byte_offset for skipped objects */
				cfg->object_skipped_ofs =
					cfg->object_skipped_ofs + 1;

				/* Adjust config memory size, less to program */
				/* Only for non-volatile T objects */
				if (first_obj_addr != 0) {
					cfg->mem_size--;
					dev_dbg(dev, "cfg->mem_size [%zu]\n",
					cfg->mem_size);
				}
			}
			continue;
		} else {
			/*
			 * Get object properties from whichever table it was found in.
			 * This allows handling both standard and extended objects.
			 */
			if (object) {
				obj_start_address = object->start_address;
				obj_size = mtch_obj_size(object);
				obj_instances = mtch_obj_instances(object);
			} else {
				obj_start_address = object_ext->start_address;
				obj_size = mtch_obj_ext_size(object_ext);
				obj_instances = mtch_obj_ext_instances(object_ext);
			}

			/* Find first object in cfg file; if not first in device */
			if (first_obj_type == 0) {
				cfg->object_skipped_ofs = 0;
				first_obj_type = type;
				first_obj_addr = obj_start_address;

				dev_dbg(dev, "First object found T[%d]\n", type);

				if (first_obj_addr > cfg->start_ofs) {

					dev_dbg(dev, "%s %d, %s %d, %s %d\n",
						"cfg->skipped_ofs",
						cfg->object_skipped_ofs,
						"first_obj_addr",
						first_obj_addr,
						"cfg->start_ofs",
						cfg->start_ofs);

					cfg->mem_size -= first_obj_addr;
				}
			}
		}

		if (type == SPT_SERIALDATACOMMAND_T68 &&
				(instance & 0x8000)) {
			/* T68 must be last packet in file */
			dev_info(dev, "T68 payload found\n");

			data->t68_datatype = (instance & 0x00FF);

			/* Allocate data to struct first */
			data->t68_buf = kzalloc((size + 1), GFP_KERNEL);
			if (!data->t68_buf)
				return -ENOMEM;

			/* remove checksum from length or status returns 0x04*/
			data->t68_length = size - 4;

			/* use object_size to load buffer, incl payload CRC */
			for (i = 0; i < size; i++) {
				ret = sscanf(cfg->raw + cfg->raw_pos, "%hhx%n",
					     &val, &offset);
				if (ret != 1) {
					dev_err(dev,
						"Cannot read T%d at byte %d\n",
						type, i);
					return -EINVAL;
				}

				*(data->t68_buf + i) = val;

				cfg->raw_pos += offset;
			}

			/* Capture T68 payload data CRC in file */
			data->t68_data_crc = ((data->t68_buf[size-4] << 24) |
				(data->t68_buf[size - 3] << 16) |
				(data->t68_buf[size - 2] << 8) |
				(data->t68_buf[size - 1]));

			dev_info(dev, "T68 data CRC from file = 0x%06X",
				 data->t68_data_crc);

			if (data->t68_datatype == T68_ENC_DTYPE) {

				data->enc_payload_crc = ((data->t68_buf[34]) |
					(data->t68_buf[35] << 8) |
					(data->t68_buf[36] << 16));

				dev_info(dev,
					 "Encrypted file payload CRC = 0x%06X",
					 data->enc_payload_crc);

			}

			error = mtch_upload_t68_payload(data, cfg);

			if (error) {
				dev_err(dev, "T68 write err, behavior may be unexpected");
				return error;	/* Return error and stop */
			}

			continue;
		}

		if (size > obj_size) {
			/*
			 * Either we are in fallback mode due to wrong
			 * config or config from a later fw version,
			 * or the file is corrupt or hand-edited.
			 */
			dev_warn(dev, "Discarding %zu byte(s) in T%u\n",
				 size - obj_size, type);
		} else if (obj_size > size) {
			/*
			 * If firmware is upgraded, new bytes may be added to
			 * end of objects. It is generally forward compatible
			 * to zero these bytes - previous behaviour will be
			 * retained. However this does invalidate the CRC and
			 * will force fallback mode until the configuration is
			 * updated. We warn here but do nothing else - the
			 * malloc has zeroed the entire configuration.
			 */
			dev_warn(dev, "Zeroing %zu byte(s) in T%d\n",
				 obj_size - size, type);
		}

		if (instance >= obj_instances) {
			dev_err(dev, "Object instances exceeded!\n");
			return -EINVAL;
		}

		reg = obj_start_address + obj_size * instance;

		/* Synchronize write offset with byte_offset in cfg->mem */
		/* Add to support missing objects within config raw file */
		wr_offset = reg - first_obj_addr - cfg->object_skipped_ofs;

		dev_dbg(dev, "%s %d, %s %d, %s %d, %s %d\n",
			"wr_offset", wr_offset,
			"reg", reg,
			"cfg->start_ofs", cfg->start_ofs,
			"cfg->object_skipped_ofs", cfg->object_skipped_ofs);

		if (IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED)) {
			/* Copy current postion in file */
			cfg->tmp_raw_pos = cfg->raw_pos;

			do {
				/* Get number of bytes in line */
				ret = sscanf(cfg->raw + cfg->tmp_raw_pos,
					     "%hhx%c%n",
					     &tmpval, &newline, &tmp_offset);

				if (ret != 2) {
					dev_err(dev,
						"Bad format in T%d at %d\n",
						type, i);
					return -EINVAL;
				}

				bytesperline++;

				cfg->tmp_raw_pos += tmp_offset;

				if (newline == '\r')
					dev_dbg(dev, "Found a new line\n");

			} while (newline != '\r');

			/* make copy of original size less padding */
			data->enc_datasize = size;

			if (bytesperline > size)
				size = bytesperline;

			bytesperline = 0;
		}

		for (i = 0; i < size; i++) {
			ret = sscanf(cfg->raw + cfg->raw_pos, "%hhx%n", &val,
				     &offset);
			if (ret != 1) {
				dev_err(dev, "Bad format in T%d at %d\n",
					type, i);
				return -EINVAL;
			}

			/* Update position in raw file to next byte */
			cfg->raw_pos += offset;

			if (!IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED)) {
				if (i > obj_size)
					continue;
			}

			byte_offset = reg + i - first_obj_addr -
				cfg->object_skipped_ofs;


			if ((byte_offset >= 0) &&
			    (byte_offset < cfg->mem_size)) {
				cfg->mem[dev_id][byte_offset] = val;
			} else {
				dev_err(dev,
					"Bad object: reg: %d, T%u, ofs=%d\n",
					reg, type, byte_offset);
				return -EINVAL;
			}
		}

		totalBytesToWrite = size;

		/* Write per object per inst per obj_size w/data in cfg.mem */
		while (totalBytesToWrite > 0) {

			if (totalBytesToWrite > MAX_BLOCK_RD_WR)
				size = MAX_BLOCK_RD_WR;
			else
				size = totalBytesToWrite;

			error = __mtch_write_reg(data,
				reg, size,
				(&cfg->mem[dev_id][wr_offset]));

			if (error)
				return error;

			wr_offset = wr_offset + size;
			totalBytesToWrite = totalBytesToWrite - size;

			if (!is_type_used_for_crc(type)) {
				cfg->object_skipped_ofs += size;
				cfg->mem_size -= size;
				dev_dbg(dev,
					"Remove T[%d] from crc calculation",
					type);
			}

		} /* End of while - write routine */

		msleep(20);

	} /* End of while - parse of raw file */

	return 0;
}

static void mtch_free_object_table(struct mtch_data *data)
{
	data->object_table = NULL;
	data->info = NULL;
	kfree(data->raw_info_block);
	data->raw_info_block = NULL;
	kfree(data->msg_buf);
	data->msg_buf = NULL;
	data->T2_address = 0;
	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_address = 0;
	data->T6_obj_size = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T18_address = 0;
	data->T19_reportid_min = 0;
	data->T44_address = 0;
	data->max_reportid = 0;
	kfree(data->ext_object_table);
	data->ext_object_table = NULL;
	data->ext_object_num = 0;
	data->T254_address = 0;
	data->T254_obj_size = 0;
	data->T500_reportid_min = 0;
	data->T501_reportid_min = 0;
	data->T501_reportid_max = 0;
	data->T502_reportid_min = 0;
	data->T502_reportid_max = 0;
}

static int mtch_read_t254_object_table(struct mtch_data *data);
static int mtch_debug_msg_init(struct mtch_data *data);

static int mtch_parse_object_table(struct mtch_data *data,
				  struct mtch_object *object_table)
{
	struct i2c_client *client = data->client;
	u16 end_address = 0;
	u8 min_id, max_id;
	u8 reportid = 1;	/* Valid report IDs start from 1 */
	size_t i;
	u8 num_instances;

	data->mem_size = 0;
	data->num_datasize_bytes = 0;

	struct mtch_object *obj = object_table;

	for (i = 0; i < data->info->object_num; i++) {
		le16_to_cpus(&obj->start_address);

		num_instances = mtch_obj_instances(obj);

		data->num_datasize_bytes += num_instances * 2;

		if (obj->num_report_ids) {
			min_id = reportid;
			reportid += obj->num_report_ids * mtch_obj_instances(obj);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		dev_info(&data->client->dev,
			"T%u Start:%u Size:%zu Instances:%zu Rpt IDs:%u-%u\n",
			obj->type, obj->start_address,
			mtch_obj_size(obj), mtch_obj_instances(obj),
			min_id, max_id);

		switch (obj->type) {
		case GEN_ENCRYPTIONSTATUS_T2:
			data->T2_address = obj->start_address;
			break;
		case GEN_MESSAGE_T5:
			/* rc1 -- No support for CRC byte */
			/* rc2 -- Add support for CRC */
			data->T5_msg_size = mtch_obj_size(obj) - 1;
			data->T5_address = obj->start_address;
			break;
		case GEN_COMMAND_T6:
			data->T6_reportid = min_id;
			data->T6_address = obj->start_address;
			data->T6_obj_size = mtch_obj_size(obj);
			break;
		case GEN_POWER_T7:
			data->T7_address = obj->start_address;
			break;
		case SPT_COMMSCONFIG_T18:
			data->T18_address = obj->start_address;
			break;
		case SPT_GPIOPWM_T19:
			data->T19_reportid_min = min_id;
			break;
		case SPT_MESSAGECOUNT_T44:
			data->T44_address = obj->start_address;
			break;
		case GEN_EXTENDEDOBJECTTABLE_T254:
			data->T254_address = obj->start_address;
			data->T254_obj_size = mtch_obj_size(obj);
			break;
		}

		end_address = obj->start_address
			+ mtch_obj_size(obj) * mtch_obj_instances(obj) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;

		if (IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED)) {
			data->mem_size += data->num_datasize_bytes;
			dev_info(&client->dev, "data->mem_size %d\n",
				 data->mem_size);
		}

		obj++;
	}	/* End of obj_num for-statement */

	dev_dbg(&client->dev, "data->mem_size %d\n", data->mem_size);

	/* Store maximum reportid */
	data->max_reportid = reportid;

	/* If T44 exists, T5 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		dev_err(&client->dev, "Invalid T44 position\n");
		return -EINVAL;
	}

	data->msg_buf = kcalloc(data->max_reportid,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf)
		return -ENOMEM;

	return 0;
}

static int mtch_parse_ext_object_report_ids(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	u8 reportid = data->max_reportid;
	int i;
	u16 end_address;

	if (!data->ext_object_table || data->ext_object_num == 0)
		return 0;

	for (i = 0; i < data->ext_object_num; i++) {
		struct mtch_object_ext *object = &data->ext_object_table[i];
		u8 min_id, max_id;
		u8 num_instances = mtch_obj_ext_instances(object);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids * num_instances;
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		dev_info(dev,
			"T%u (ext) Start:%u Size:%zu Instances:%u Rpt IDs:%u-%u\n",
			object->type, object->start_address,
			mtch_obj_ext_size(object), num_instances,
			min_id, max_id);

		switch (object->type) {
		case TOUCH_MTCHKEYARRAY_T500:
			data->T500_reportid_min = min_id;
			break;
		case SPT_MTCHKEYCTRL_T501:
			data->T501_reportid_min = min_id;
			data->T501_reportid_max = max_id;
			break;
		case SPT_MTCHSLIDERCTRL_T502:
			data->T502_reportid_min = min_id;
			data->T502_reportid_max = max_id;
			break;
		}

		end_address = object->start_address +
			      mtch_obj_ext_size(object) * num_instances - 1;
		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	data->max_reportid = reportid;
	return 0;
}


static int mtch_read_info_block(struct mtch_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size;
	void *id_buf, *buf;
	uint8_t num_objects;
	u32 calculated_crc;
	u8 *crc_ptr;

	/* If info block already allocated, free it */
	if (data->raw_info_block) {
		mtch_free_object_table(data);
		kfree(id_buf);
		id_buf = NULL;
	}

	/* Write done separate, due to HA and encryption init */
	error = mtch_write_info_addr(data);
	if (error)
		return error;

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mtch_info);
	id_buf = kzalloc(size, GFP_KERNEL);
	if (!id_buf)
		return -ENOMEM;

	/*
	 * Use read-only since address pointer was set by mtch_write_info_addr()
	 * which includes datasize, tx seq, and CRC bytes.
	 */
	error = __mtch_read_reg_ex(data, 0, size, id_buf, true);
	if (error)
		goto err_free_mem;

	//Possible relocation of this to get info on chip before CRC
	data->info = (struct mtch_info *)id_buf;

	dev_info(&client->dev,
		 "Family: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
		 data->info->family_id, data->info->variant_id,
		 data->info->version >> 4, data->info->version & 0xf,
		 data->info->build, data->info->object_num);

	if (!(mtch_lookup_chips(data)))
		dev_warn(&client->dev, "maXTouch device found\n");

	/* Resize buffer to give space for rest of info block */
	num_objects = ((struct mtch_info *)id_buf)->object_num;

	/* Calculate size of memory for u8 type objects */
	size += (num_objects * sizeof(struct mtch_object))
		+ INFO_CRC_SIZE;

	/* Allocate memory for u8 object type */
	buf = krealloc(id_buf, size, GFP_KERNEL);
	if (!buf) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	id_buf = buf;

	data->info = (struct mtch_info *)id_buf;

	/* Set address pointer to OBJECT_START for reading rest of info block */
	error = mtch_write_addr_ptr(data, OBJECT_START);
	if (error)
		goto err_free_mem;

	/*
	 * Read rest of info block after id block.
	 * Use read-only since address pointer was set by mtch_write_addr_ptr().
	 */
	error = __mtch_read_reg_ex(data, OBJECT_START, (size - OBJECT_START),
				   (id_buf + OBJECT_START), true);
	if (error)
		goto err_free_mem;

	/* Extract & calculate checksum */
	crc_ptr = id_buf + size - INFO_CRC_SIZE;

	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = mtch_calculate_crc(id_buf, 0,
					   size - INFO_CRC_SIZE, 0);

	dev_dbg(&client->dev, "Calculated crc %x\n", calculated_crc);

	/*
	 * CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or device is not using OBP protocol
	 */
	if (data->info_crc == 0 || data->info_crc != calculated_crc) {
		dev_err(&client->dev,
			"Info Block CRC error calculated=0x%06X read=0x%06X\n",
			calculated_crc, data->info_crc);
		error = -EIO;
		goto err_free_mem;
	} else {
		dev_info(&client->dev, "Device Info Block CRC matches calculated CRC\n");
	}

	data->raw_info_block = id_buf;
	data->info = (struct mtch_info *)id_buf;

	/* Parse object table information */
	error = mtch_parse_object_table(data, id_buf + OBJECT_START);
	if (error) {
		dev_err(&client->dev, "Error %d parsing object table\n",
			error);
		mtch_free_object_table(data);
		goto err_free_mem;
	}

	data->object_table = (struct mtch_object *)(id_buf + OBJECT_START);

	dev_dbg(&client->dev, "Stage 2 of extending parsing function\n");
	/* Stage 2 - Extended object read and parse */
	/* Read and parse T254 extended object table if present */
	if (data->T254_address) {
		error = mtch_read_t254_object_table(data);
		if (error) {
			dev_err(&client->dev, "Failed to read T254: %d\n",
				 error);
		} else {
			error = mtch_parse_ext_object_report_ids(data);
			if (error)
				dev_warn(&client->dev,
					 "Failed to parse T254 report IDs\n");
		}
	}

	/* Check if device using encryption */
	error = mtch_check_encryption(data);
	if (error)
		dev_warn(&client->dev, "Warning, encryption state unknown\n");

	/* Captures messages from buffer after startup */
	error = mtch_process_messages_until_invalid(data, false);
	if (error)
		dev_warn(&client->dev, "Warning, possible unread messages\n");

	if (data->hc.reqd_device != data->hc.devid_num) {
		dev_err(&client->dev, "Error, Requested device and selected device does not match\n");
		goto err_free_mem;
	}

	return 0;

err_free_mem:
	kfree(id_buf);

	return error;
}

static int mtch_sync_t7_power(struct mtch_data *data, u8 mode)
{
	struct device *dev = &data->client->dev;
	struct t7_config *new_cfg;
	struct t7_config deepsleep = { .active = 0, .idle = 0 };
	int ret;

	if (mode == POWER_CFG_INIT) {
		ret = __mtch_read_reg(data, data->T7_address, sizeof(data->t7_cfg),
			&data->t7_cfg);
		if (ret < 0) {
			dev_err(dev, "Could not read T7 power object");
		}

		if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
			dev_info(dev, "Invalid T7 cfg,734 apply defaults");
			data->t7_cfg.active = 10;
			data->t7_cfg.idle = 30;
		}
		new_cfg = &data->t7_cfg;
	} else {  /* if POWER_CFG_RUN, program current t7_cfg */
		new_cfg = (mode == POWER_CFG_DEEPSLEEP) ? &deepsleep : &data->t7_cfg;
	}

	ret = __mtch_write_reg(data, data->T7_address, 2, (const u8 *)new_cfg);
	if (ret < 0) {
		dev_err(dev, "Failed to set the T7 power object");
	} else {
		dev_info(dev, "T7 Power set - ACTV:%d IDLE:%d\n", new_cfg->active,
				new_cfg->idle);
	}

	return ret;
}

/*
 * mtch_update_cfg - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first 4 to 6 lines of the raw config file contain:
 *  1) Configuration version
 *  2) If present, two line encryption header
 *  3) Chip ID Information (first 7 bytes of device memory)
 *  4) Chip Information Block 24-bit CRC Checksum
 *  5) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *   <TYPE> - 2-byte object type as hex
 *   <INSTANCE> - 2-byte object instance number as hex
 *   <SIZE> - 2-byte object size as hex
 *   <CONTENTS> - array of <SIZE> 1-byte hex values
 */
static int mtch_update_cfg(struct mtch_data *data, const struct firmware *fw)
{
	struct device *dev = &data->client->dev;
	struct mtch_cfg cfg;
	u32 info_crc = 0;
	u32 *config_crc = 0;
	u32 config_val;
	u32 calculated_crc[10] = {0};
	int cfg_version = 0;
	int cfg_blksize = 0;
	int cfg_enc = 0;
	int numofdevs = 1;
	char tmp[255];
	bool upd_cfg_reqd = false;
	u8 num_of_elements = 1;
	int num_of_bytes;
	bool skip_backup = false;
	int offset;
	int error;
	int ret = 0;
	int i, j;

	/* Make zero terminated copy of the OBP_RAW file */
	cfg.raw = kmemdup_nul(fw->data, fw->size, GFP_KERNEL);
	if (!cfg.raw)
		return -ENOMEM;

	cfg.raw_size = fw->size;

	mtch_update_crc(data, CMD_REPORTALL_OFS, 1, true);

	//Clear messages after update in cases /CHG low
	error = mtch_process_messages_until_invalid(data, true);
	if (error)
		dev_dbg(dev, "Unable to read all messages.\n");

	if (!strncmp(cfg.raw, CFG_MAGIC_V1,
	   strlen(CFG_MAGIC_V1))) {
		dev_info(dev, "Found V1 config file\n");
		cfg_version = 0x01;
		cfg.raw_pos = strlen(CFG_MAGIC_V1);
	} else if (!strncmp(cfg.raw, CFG_MAGIC_V2,
		strlen(CFG_MAGIC_V2))) {
		dev_info(dev, "Found V2 config file\n");
		cfg_version = 0x02;
		cfg.raw_pos = strlen(CFG_MAGIC_V2);
	} else if (!strncmp(cfg.raw, CFG_MAGIC_V3,
		strlen(CFG_MAGIC_V3))) {
		dev_info(dev, "Found V3 config file\n");
		cfg_version = 0x03;
		cfg.raw_pos = strlen(CFG_MAGIC_V3);
	} else if (!strncmp(cfg.raw, CFG_MAGIC_V4,
		strlen(CFG_MAGIC_V4))) {
		dev_info(dev, "Found V4 config file\n");
		cfg_version = 0x04;
		cfg.raw_pos = strlen(CFG_MAGIC_V4);
	} else {
		dev_err(dev, "Unrecognised config file\n");
		ret = -EINVAL;
		goto release_raw;
	}

	if (cfg_version == 0x03 || cfg_version == 0x04) {
		if (sscanf(cfg.raw + cfg.raw_pos, "%s%d%n", tmp, &cfg_enc,
		    &offset) != 2) {
			dev_err(dev, "Failed to get encryption state\n");
			goto release_raw;
		}

		if (cfg_enc) {
			if (IS_BITS_ENABLED(data->encryption_state, CFG_ENCRYPTED)) {
				/*  Verify if chip is encrypted */
				dev_info(dev,
					 "CFG and device are encrypted. Okay to update CFG");
			} else {
				dev_err(dev,
					"Cannot write encrypted CFG to unencrypted device\n");
					ret = -EINVAL;
					goto release_raw;
			}
		} else {
			dev_info(dev, "CFG file is unencrypted. Checking device encryption");

			if (IS_BITS_ENABLED(data->encryption_state, CFG_ENCRYPTED)) {
				/* Verify if device is unencrypted */
				dev_err(dev, "Cannot write unencrypted CFG to encrypted device\n");
					ret = -EINVAL;
					goto release_raw;
			} else {
				dev_info(dev, "Device CFG is unencrypted. Okay to write CFG.");
			}
		}

		if (!(strcmp("ENCRYPTION ", tmp))) {
			dev_err(dev, "ENCRYPTION header not found");
			goto release_raw;
		}

		cfg.raw_pos += offset;

		if (sscanf(cfg.raw + cfg.raw_pos, "%s%d%n", tmp, &cfg_blksize,
		    &offset) != 2)
			dev_err(dev, "Failed to get encryption block\n");

		if (!(strcmp("MAX_ENCRYPTION_BLOCKS ", tmp))) {
			dev_err(dev, "MAX ENCRYPTION BLOCK header not found");
			goto release_raw;
		}

		data->enc_blocksize = cfg_blksize * 16;

		cfg.raw_pos += offset;
	}

	if (cfg_version == 0x04) {
		if (sscanf(cfg.raw + cfg.raw_pos, "%s%d%n", tmp, &numofdevs,
			   &offset) != 2) {
			dev_err(dev, "Failed to get number of devices\n");
			goto release_raw;
		}

		cfg.raw_pos += offset;
	}

	/* Load 7byte infoblock from config file */
	for (i = 0; i < sizeof(struct mtch_info) ; i++) {
		ret = sscanf(cfg.raw + cfg.raw_pos, "%hhx%n",
			     (unsigned char *)&cfg.info + i,
			     &offset);
		if (ret != 1) {
			dev_err(dev, "Bad format\n");
			ret = -EINVAL;
			goto release_raw;
		}

		/* Update position in raw file to info CRC */
		cfg.raw_pos += offset;
	}

	/* Compare family id, file vs chip */
	if (cfg.info.family_id != data->info->family_id) {
		dev_err(dev, "Family ID mismatch!\n");
		ret = -EINVAL;
		goto release_raw;
	}

	/* Compare variant id, file vs chip */
	if (cfg.info.variant_id != data->info->variant_id) {
		dev_err(dev, "Variant ID mismatch!\n");
		ret = -EINVAL;
		goto release_raw;
	}

	/* Read Infoblock CRCs */
	ret = sscanf(cfg.raw + cfg.raw_pos, "%x%n", &info_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Info CRC\n");
		ret = -EINVAL;
		goto release_raw;
	}
	/* Update position in raw file to config CRC */
	cfg.raw_pos += offset;

	config_crc = kcalloc(data->hc.num_of_devs, sizeof(u32), GFP_KERNEL);
	if (!config_crc)
		goto release_raw;

	/* Be careful of checksum order.  Swapped in xcfg file */
	for (i = 0; i < data->hc.num_of_devs; i++) {
		ret = sscanf(cfg.raw + cfg.raw_pos, "%x%n", &config_val,
			     &offset);
		if (ret != 1) {
			dev_err(dev, "Bad format: failed to parse Config CRC\n");
			ret = -EINVAL;
			goto release_config_crc;
		}

		dev_info(dev, "Config file checksum[%d]: 0x%06X\n", i,
			 config_val);
		dev_info(dev, "Device checksum[%d]: 0x%06X\n", i,
			 data->dev_cfg_crc[i]);

		config_crc[i] = config_val;

		if (config_crc[i] != data->dev_cfg_crc[i]) {
			dev_info(dev,
				 "Config and device checksum[%d] not equal", i);
			upd_cfg_reqd = true;
		} else {
			dev_info(dev, "Config and device checksum[%d] match 0x%06X",
				 i, data->dev_cfg_crc[i]);
		}

		if (data->dev_cfg_crc[i] == 0) {
			dev_info(dev, "Device config_crc[%d] is zero", i);
			upd_cfg_reqd = true;
		}

		/* Update position in raw file to first T object */
		cfg.raw_pos += offset;
	}

	if (cfg_version == 4) {
		ret = sscanf(cfg.raw + cfg.raw_pos, "%s%n", tmp, &offset);

		if ((!strncmp(tmp, "DEVICE_0", 8)) ||
		    (!strncmp(tmp, "[DEVICE_0", 9))) {
			dev_dbg(dev, "Found first Device\n");
		} else {
			for (i = 0; i < data->hc.num_of_devs; i++) {
				ret = sscanf(cfg.raw + cfg.raw_pos, "%x%n",
				      &data->nvm_cfg_crc[i], &offset);
				if (ret != 1)
					dev_warn(dev,
						 "Bad format: NVM checksum not present\n");

				dev_info(dev, "Config NVM checksum[%d]: 0x%06X",
					 i, data->nvm_cfg_crc[i]);

				cfg.raw_pos += offset;
			}
		}
	}

	/*
	 * The Info Block CRC is calculated over mtch_info and the object
	 * table. If it does not match then we are trying to load the
	 * configuration from a different chip or firmware version, so
	 * the configuration CRC is invalid anyway.
	 * Updating required if checksum is zero or not equal to file
	 * checksum.
	 */
	if (info_crc == data->info_crc) {
		if (upd_cfg_reqd) {
			dev_info(dev, "Device configuration update required\n");
		} else {
			dev_info(dev, "Skipping device configuration update");
			ret = 0;
			goto release_config_crc;
		}
	} else {
		dev_warn(dev, "Device and file infoblock crc do not match\n");
		dev_warn(dev, "Device 0x%06X, File 0x%06X. Failed programming",
			 data->info_crc, info_crc);
		goto release_config_crc;
	}

	/* Freeze the devide for programming if hc_capable */
	if (data->hc.hc_capable) {
		mtch_update_crc(data, CMD_BACKUPNV_OFS, BACKUP_FREEZE,
			       false);
	} else {
		/* Stop T70 Dynamic Configuration before calculation of CRC */
		mtch_update_crc(data, CMD_BACKUPNV_OFS, BACKUP_W_STOP,
			       false);
	}

	/* Malloc memory to store configuration */
	cfg.start_ofs = OBJECT_START +
			data->info->object_num * sizeof(struct mtch_object) +
			INFO_CRC_SIZE;

	cfg.mem_size = data->mem_size;

	num_of_elements = data->hc.num_of_devs * sizeof(u8);

	cfg.mem = kzalloc(num_of_elements, GFP_KERNEL);

	if (!cfg.mem) {
		ret = -ENOMEM;
		dev_err(dev, "Memory allocation failed for mem");
		goto release_config_crc;
	}

	num_of_bytes = cfg.mem_size * sizeof(u8);

	for (i = 0; i < data->hc.num_of_devs; i++) {
		cfg.mem[i] = kzalloc(num_of_bytes, GFP_KERNEL);

		if (cfg.mem[i] == NULL) {
			ret = -ENOMEM;
			dev_err(dev, "Memory allocation failed for mem[%i]", i);
			for (j = 0; j < i; j++)
				kfree(cfg.mem[j]);
			goto release_cfg_mem;
		}
	}

	dev_info(dev, "Updating device configuration\n");

	/* Prepares and programs configuration */
	ret = mtch_prepare_cfg_mem(data, &cfg);
	if (ret)
		goto release_mem;

	/* Calculate crc of the config file */
	/* Config file must include all objects used in CRC calculation */

	if (!(IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED))) {
		for (i = 0; i < data->hc.num_of_devs; i++) {
			dev_dbg(dev, "%s %d %s %zu\n",
				 "cfg.object_skipped_ofs",
				 cfg.object_skipped_ofs,
				 "cfg.mem_size", cfg.mem_size);

			/* maybe dont need this */
			if (i == HC_DEV_MAX)
				break;

			calculated_crc[i] = mtch_calculate_crc(&cfg.mem[i][0],
					    0, cfg.mem_size, 0);

			dev_info(dev, "Calculated cfg checksum[%d] 0x%06X\n",
				 i, calculated_crc[i]);
		}
	}

	msleep(50);	//Allow delay before issuing backup and reset

	if (data->hc.hc_capable)
		mtch_update_crc(data, CMD_BACKUPNV_OFS, BACKUP_UNFREEZE,
			       false);

	if (!skip_backup)
		mtch_update_crc(data, CMD_BACKUPNV_OFS, BACKUP_VALUE,
			       true);

	msleep(100);	//Allow 100ms before issuing reset

	mtch_soft_reset(data, true);

	for (i = 0; i < data->hc.num_of_devs; i++) {
		if (i == HC_DEV_MAX)
			break;

		if (calculated_crc[i] != data->dev_cfg_crc[i])
			dev_info(dev, "Calculated cfg and device checksum[%d] do not match.\n",
				 i);
		else
			dev_info(dev, "Calculated cfg and device checksum[%d] match.",
				 i);

		dev_info(dev, "Calculated: 0x%06X Device: 0x%06X\n",
				calculated_crc[i], data->dev_cfg_crc[i]);

	}

	mtch_check_encryption(data);

	ret = mtch_read_info_block(data);

	if (!ret)
		dev_dbg(dev, "Read InfoBlock success");
	else
		dev_err(dev, "Read InfoBlock failed(%d), check for bootloader",
			error);

	if (!(IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED))) {
	/* T7 config may have changed */
		mtch_sync_t7_power(data, POWER_CFG_INIT);

		error = mtch_check_retrigen(data);

		if (error)
			dev_err(dev, "RETRIGEN Not Enabled or unavailable\n");
	}

release_mem:
	for (i = 0; i < data->hc.num_of_devs; i++)
		kfree(cfg.mem[i]);
release_cfg_mem:
	kfree(cfg.mem);
release_config_crc:
	kfree(config_crc);
release_raw:
	kfree(cfg.raw);

	return ret;
}

static int mtch_clear_cfg(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	struct mtch_cfg config;
	int writeByteSize = 0;
	int write_offset = 0;
	int totalBytesToWrite = 0;
	int error;

	/* Avoid crash if chip in bootloader at bootup */
	if (!data->info)
		return -ENOENT;

	/* Start of first Tobject address */
	config.start_ofs = OBJECT_START +
			data->info->object_num * sizeof(struct mtch_object) +
			INFO_CRC_SIZE;

	config.mem_size = data->mem_size;
	totalBytesToWrite = config.mem_size;

	/* Allocate memory for full size of config space */
	config.mem = kzalloc(config.mem_size, GFP_KERNEL);
	if (!config.mem) {
		error = -ENOMEM;
		goto release_mem;
	}

	dev_dbg(dev, "clear_cfg: config.mem_size %zu, config.start_ofs %i\n",
		config.mem_size, config.start_ofs);

	while (totalBytesToWrite > 0) {
		if ((IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED)) &&
			(totalBytesToWrite > data->enc_blocksize)) {

			writeByteSize = data->enc_blocksize;

		} else if (totalBytesToWrite > MAX_BLOCK_RD_WR) {
			writeByteSize = MAX_BLOCK_RD_WR;
		} else {
			writeByteSize = totalBytesToWrite;
		}

		error = __mtch_write_reg(data,
					(config.start_ofs +
					write_offset),
					writeByteSize,
					config.mem);
		if (error) {
			dev_info(dev, "Error writing configuration\n");
			goto release_mem;
		}

		write_offset = write_offset + writeByteSize;
		totalBytesToWrite = totalBytesToWrite - writeByteSize;
	}

	mtch_update_crc(data, CMD_BACKUPNV_OFS, BACKUP_VALUE, true);

	msleep(300);

	dev_info(dev, "Config successfully cleared\n");

release_mem:
	kfree(config.mem);
	return error;
}

static void mtch_free_input_device(struct mtch_data *data)
{
	int i;

	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}

	/* Free dynamically allocated wheel devices */
	if (data->input_dev_wheels) {
		for (i = 0; i < data->t502_num_wheels; i++) {
			if (data->input_dev_wheels[i]) {
				input_unregister_device(data->input_dev_wheels[i]);
				data->input_dev_wheels[i] = NULL;
			}
		}
		devm_kfree(&data->client->dev, data->input_dev_wheels);
		data->input_dev_wheels = NULL;
	}

	/* Free dynamically allocated slider devices */
	if (data->input_dev_sliders) {
		for (i = 0; i < data->t502_num_sliders; i++) {
			if (data->input_dev_sliders[i]) {
				input_unregister_device(data->input_dev_sliders[i]);
				data->input_dev_sliders[i] = NULL;
			}
		}
		devm_kfree(&data->client->dev, data->input_dev_sliders);
		data->input_dev_sliders = NULL;
	}

	/* Free T502 state tracking arrays */
	if (data->t502_last_pos) {
		devm_kfree(&data->client->dev, data->t502_last_pos);
		data->t502_last_pos = NULL;
	}
	if (data->t502_in_detect) {
		devm_kfree(&data->client->dev, data->t502_in_detect);
		data->t502_in_detect = NULL;
	}

	data->t502_num_wheels = 0;
	data->t502_num_sliders = 0;
}

static int mtch_read_t254_object_table(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	u8 *buf;
	int error = 0;
	int i;
	u8 num_ext_objects = 0;
	u16 t254_size;
	u32 calculated_crc, stored_crc;
	u8 *crc_ptr;

	if (data->T254_address == 0 || data->T254_obj_size == 0)
		return 0;

	t254_size = data->T254_obj_size;
	buf = kzalloc(t254_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	dev_err(dev, "T254 addr=0x%04X size=%u\n", data->T254_address, t254_size);

	/* Read T254 object data */
	error = __mtch_read_reg(data, data->T254_address, t254_size, buf);

	if (error) {
		dev_err(dev, "Failed to read T254 object %d\n", error);
		goto err_free;
	}

	/* Count valid extended objects (non-zero start address) */
	for (i = 0; i < T254_OBJECTS_PER_BLOCK; i++) {
		u16 start_addr = get_unaligned_le16(buf +
					(i * T254_ELEMENT_SIZE) + 2);
		if (start_addr != 0)
			num_ext_objects++;
	}

	if (num_ext_objects == 0) {
		dev_err(dev, "T254 present but contains no extended objects\n");
		kfree(buf);
		return 0;
	}

	/* Validate CRC */
	crc_ptr = buf + t254_size - T254_CRC_SIZE;
	stored_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);
	calculated_crc = mtch_calculate_crc(buf, 0,
					   t254_size - T254_CRC_SIZE, 0);

	if (stored_crc != calculated_crc) {
		dev_err(dev, "T254 CRC mismatch: calc=0x%06X read=0x%06X\n",
			calculated_crc, stored_crc);
		error = -EIO;
		goto err_free;
	} else {
		dev_info(dev, "T254 InfoBlock CRC match 0x%06X", calculated_crc);
	}

	/* Allocate extended object table */
	data->ext_object_table = kcalloc(num_ext_objects,
					 sizeof(struct mtch_object_ext),
					 GFP_KERNEL);
	if (!data->ext_object_table) {
		error = -ENOMEM;
		goto err_free;
	}

	data->ext_object_num = num_ext_objects;

	/* Parse extended objects */
	num_ext_objects = 0;
	for (i = 0; i < T254_OBJECTS_PER_BLOCK; i++) {
		u8 *entry = buf + (i * T254_ELEMENT_SIZE);
		u16 start_addr = get_unaligned_le16(entry + 2);

		if (start_addr == 0)
			continue;

		data->ext_object_table[num_ext_objects].type =
			get_unaligned_le16(entry);
		data->ext_object_table[num_ext_objects].start_address =
			start_addr;
		data->ext_object_table[num_ext_objects].size_minus_one =
			entry[4];
		data->ext_object_table[num_ext_objects].instances_minus_one =
			entry[5];
		data->ext_object_table[num_ext_objects].num_report_ids =
			entry[6];

		dev_info(dev,
			 "T254 ext object: T%u Start:%u Size:%u Instances:%u ReportIDs:%u\n",
			 data->ext_object_table[num_ext_objects].type,
			 data->ext_object_table[num_ext_objects].start_address,
			 data->ext_object_table[num_ext_objects].size_minus_one + 1,
			 data->ext_object_table[num_ext_objects].instances_minus_one + 1,
			 data->ext_object_table[num_ext_objects].num_report_ids);

		num_ext_objects++;
	}

	kfree(buf);
	return 0;

err_free:
	kfree(buf);
	return error;
}

static void mtch_start(struct mtch_data *data)
{

	struct device *dev = &data->client->dev;

	dev_info(dev, "%s: Starting ...\n", __func__);

	mtch_sync_t7_power(data, POWER_CFG_RUN);

	/* Recalibrate chip after wake from deep sleep */
	mtch_t6_command(data, CMD_CALIBRATE_OFS, 1, false);

	msleep(200);	//Wait for calibration cmmand

	mtch_process_messages_until_invalid(data, true);

}

static void mtch_stop(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;

	/* TBD - deep sleep, wake-on-touch */
	dev_info(dev, "%s: Stopping ...\n", __func__);
	mtch_sync_t7_power(data, POWER_CFG_DEEPSLEEP);

}

static int mtch_input_open(struct input_dev *dev)
{
	struct mtch_data *data = input_get_drvdata(dev);

	mtch_start(data);

	return 0;
}

static void mtch_input_close(struct input_dev *dev)
{
	struct mtch_data *data = input_get_drvdata(dev);

	mtch_stop(data);
}

static int mtch_initialize_input_device(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev;
	struct input_dev *input_keys;
	char name_buf[32];
	int error;
	int i;

	/*
	 * Step 1: Read T502 configuration to determine wheel/slider types and counts.
	 * This must be done BEFORE T501 so we know which keys belong to wheels/sliders.
	 */
	error = mtch_t502_get_valid_instances(data);
	if (error && error != -ENOENT) {
		dev_err(dev, "Failed to read T502 configuration: %d\n", error);
		return error;
	}

	/*
	 * Step 2: Build T501 key mapping based on T502 wheel/slider key usage.
	 * Keys used by wheels/sliders are excluded; remaining enabled keys become buttons.
	 */
	error = mtch_t501_build_key_map(data);
	if (error && error != -ENOENT) {
		dev_err(dev, "Failed to build T501 key map: %d\n", error);
		return error;
	}

	/*
	 * Allocate state tracking arrays for T502 instances
	 */
	if (data->t502_num_instances > 0) {
		data->t502_last_pos = devm_kcalloc(dev, data->t502_num_instances,
						   sizeof(*data->t502_last_pos),
						   GFP_KERNEL);
		if (!data->t502_last_pos)
			return -ENOMEM;

		data->t502_in_detect = devm_kcalloc(dev, data->t502_num_instances,
						    sizeof(*data->t502_in_detect),
						    GFP_KERNEL);
		if (!data->t502_in_detect)
			return -ENOMEM;
	}

	/*
	 * 1. Initialize Wheel Input Devices (dynamically based on T502 config)
	 */
	if (data->t502_num_wheels > 0) {
		data->input_dev_wheels = devm_kcalloc(dev, data->t502_num_wheels,
						      sizeof(*data->input_dev_wheels),
						      GFP_KERNEL);
		if (!data->input_dev_wheels)
			return -ENOMEM;

		for (i = 0; i < data->t502_num_wheels; i++) {
			input_dev = devm_input_allocate_device(dev);
			if (!input_dev) {
				dev_err(dev, "Failed to allocate wheel %d input device\n", i);
				return -ENOMEM;
			}

			if (data->t502_num_wheels == 1)
				snprintf(name_buf, sizeof(name_buf), "MTCH Wheel");
			else
				snprintf(name_buf, sizeof(name_buf), "MTCH Wheel %d", i);

			input_dev->name = devm_kstrdup(dev, name_buf, GFP_KERNEL);
			input_dev->phys = data->phys;
			input_dev->id.bustype = BUS_I2C;
			input_dev->dev.parent = dev;
			input_dev->open = mtch_input_open;
			input_dev->close = mtch_input_close;
			input_set_drvdata(input_dev, data);

			/* Support both absolute and relative wheel reporting */
			input_set_capability(input_dev, EV_ABS, ABS_WHEEL);
			input_set_capability(input_dev, EV_REL, REL_WHEEL);
			input_set_capability(input_dev, EV_KEY, BTN_TOUCH);
			input_set_abs_params(input_dev, ABS_WHEEL, 0, 4095, 0, 0);

			error = input_register_device(input_dev);
			if (error) {
				dev_err(dev, "Failed to register wheel %d input device\n", i);
				return error;
			}

			data->input_dev_wheels[i] = input_dev;
			dev_info(dev, "Registered wheel input device: %s\n", name_buf);
		}
	}

	/*
	 * 2. Initialize Slider Input Devices (dynamically based on T502 config)
	 */
	if (data->t502_num_sliders > 0) {
		data->input_dev_sliders = devm_kcalloc(dev, data->t502_num_sliders,
						       sizeof(*data->input_dev_sliders),
						       GFP_KERNEL);
		if (!data->input_dev_sliders)
			return -ENOMEM;

		for (i = 0; i < data->t502_num_sliders; i++) {
			input_dev = devm_input_allocate_device(dev);
			if (!input_dev) {
				dev_err(dev, "Failed to allocate slider %d input device\n", i);
				return -ENOMEM;
			}

			if (data->t502_num_sliders == 1)
				snprintf(name_buf, sizeof(name_buf), "MTCH Slider");
			else
				snprintf(name_buf, sizeof(name_buf), "MTCH Slider %d", i);

			input_dev->name = devm_kstrdup(dev, name_buf, GFP_KERNEL);
			input_dev->phys = data->phys;
			input_dev->id.bustype = BUS_I2C;
			input_dev->dev.parent = dev;
			input_dev->open = mtch_input_open;
			input_dev->close = mtch_input_close;
			input_set_drvdata(input_dev, data);

			/* Could be ABS_X or ABS_Y, adjust if needed */
			input_set_capability(input_dev, EV_ABS, ABS_X);
			input_set_capability(input_dev, EV_KEY, BTN_TOUCH);
			input_set_abs_params(input_dev, ABS_X, 0, 4095, 0, 0);

			error = input_register_device(input_dev);
			if (error) {
				dev_err(dev, "Failed to register slider %d input device\n", i);
				return error;
			}

			data->input_dev_sliders[i] = input_dev;
			dev_info(dev, "Registered slider input device: %s\n", name_buf);
		}
	}

	/*
	 * 3. Initialize Keys Input Device (only if buttons are configured)
	 */
	if (data->t501_num_buttons > 0) {
		input_keys = devm_input_allocate_device(dev);
		if (!input_keys) {
			dev_err(dev, "Failed to allocate keys input device\n");
			return -ENOMEM;
		}

		input_keys->name = "MTCH Keys";
		input_keys->phys = data->phys;
		input_keys->id.bustype = BUS_I2C;
		input_keys->dev.parent = dev;
		input_keys->open = mtch_input_open;
		input_keys->close = mtch_input_close;
		input_set_drvdata(input_keys, data);

		/*
		 * Register only the keycodes that are actually assigned to buttons.
		 * This ensures the input device only advertises capabilities it has.
		 */
		for (i = 0; i < data->t501_num_instances; i++) {
			if (data->t501_keys[i].usage == T501_KEY_BUTTON) {
				input_set_capability(input_keys, EV_KEY,
						     data->t501_keys[i].keycode);
			}
		}

		error = input_register_device(input_keys);
		if (error) {
			dev_err(dev, "Failed to register keys input device\n");
			return error;
		}

		data->input_dev = input_keys;
		dev_info(dev, "Registered keys input device with %d button(s)\n",
			 data->t501_num_buttons);
	}

	dev_info(dev, "Input devices registered: %d wheel(s), %d slider(s), %d button(s)\n",
		 data->t502_num_wheels, data->t502_num_sliders, data->t501_num_buttons);

	return 0;
}

static int mtch_configure_objects(struct mtch_data *data,
				 const struct firmware *cfg)
{
	struct device *dev = &data->client->dev;
	int error;

	if (!(IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED))) {
		error = mtch_sync_t7_power(data, POWER_CFG_INIT);
		if (error) {
			dev_err(dev, "Failed to initialize power cfg\n");
			return error;
		}
	}

	if (cfg) {
		error = mtch_update_cfg(data, cfg);
		if (error)
			dev_warn(dev, "Error %d updating config\n", error);
	} else {
		data->irq_processing = true;
	}

	if (!(IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED))) {
		error = mtch_check_retrigen(data);
		if (error)
			dev_err(dev, "RETRIGEN Not Enabled or unavailable\n");
	}

	if (data->system_power_up && !(data->sysfs_updating_cfg_fw)) {
		dev_info(dev, "mtch_config: Registering devices\n");
		error = mtch_initialize_input_device(data);
		if (error)
			return error;

		mtch_debug_msg_init(data);
	}

	msleep(100);

	data->irq_processing = true;
	data->sysfs_updating_cfg_fw = false;

	return 0;
}
static __maybe_unused int mtch_touchscreen_parse_properties(struct mtch_data *data)
{
	/* Needed for encrypted configuration properties */
	/* Set in the device tree */

	return 0;
}

static void mtch_config_cb(const struct firmware *cfg, void *ctx)
{
	struct mtch_data *data = ctx;

	mtch_configure_objects(data, cfg);
	release_firmware(cfg);
}

static int mtch_initialize(struct mtch_data *data)
{
	struct i2c_client *client = data->client;
	int recovery_attempts = 0;
	int error;

	while (1) {
		error = mtch_read_info_block(data);
		if (error) {
			dev_err(&client->dev, "Read Info Block failed(%d)",
				error);
		} else {
			dev_dbg(&client->dev, "Read Info Block success\n");
			break;
		}

		dev_info(&client->dev, "Checking for bootloader mode.\n");

		if (!data->crc_enabled) {
			/* Check bootloader state */
			error = mtch_probe_bootloader(data);
			/* Chip is not in appmode or bootloader mode */
			if (error)
				return error;

			/* OK, we are in bootloader, see if we can recover */
			if (++recovery_attempts > 1) {
				dev_err(&client->dev,
					"Cannot recover from btldr mode\n");
				/*
				 * We can reflash from this state, so do not
				 * abort initialization.
				 */
				data->in_bootloader = true;
				return 0;
			}

			/* Attempt to exit bootloader into app mode */
			mtch_send_bootloader_cmd(data, false);
			msleep(FW_RESET_TIME);
		}
	}

	data->system_power_up = true;

	if (!(IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED))) {
		error = mtch_check_retrigen(data);
		if (error)
			dev_err(&client->dev,
				"RETRIGEN Not Enabled or unavailable\n");
	}

	error = mtch_acquire_irq(data);
	if (error)
		return error;

	if (data->cfg_name) {
		/* As built-in driver, root filesystem may not be ready yet */
		error = request_firmware_nowait(THIS_MODULE, true,
						MTCH_CFG_NAME, &client->dev,
						GFP_KERNEL, data,
						mtch_config_cb);
		if (error)
			dev_warn(&client->dev,
				 "Failed to invoke firmware loader: %d\n",
				 error);
	} else {
		error = mtch_configure_objects(data, NULL);
		if (error)
			goto err_free_object_table;
	}

	return 0;

err_free_object_table:
	mtch_free_object_table(data);
	return error;
}

static void mtch_prepare_debugfs(struct mtch_data *data,
					 const char *debugfs_name)
{
	data->debug_dir = debugfs_create_dir(debugfs_name, NULL);
	if (!data->debug_dir)
		return;
	/* 4(read), 2(write), 1(exe) */
	debugfs_create_bool("debug_irq", 0644, data->debug_dir,
			    &data->irq_processing);
}

static void mtch_teardown_debugfs(struct mtch_data *data)
{
	debugfs_remove_recursive(data->debug_dir);
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	struct mtch_info *info = data->info;

	return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
			 info->version >> 4, info->version & 0xf, info->build);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t hw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	struct mtch_info *info = data->info;

	return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
			 info->family_id, info->variant_id);
}

/* Configuration crc check sum is returned as hex xxxxxx */
static ssize_t config_crc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	int i;

	struct mtch_data *data = dev_get_drvdata(dev);

	for (i = 0; i < data->hc.num_of_devs; i++)
		count += scnprintf(buf + count, PAGE_SIZE, "0x%06X\n",
			 data->dev_cfg_crc[i]);

	return count;
}

/* Return driver version as a string */
static ssize_t drvr_version_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%s\n",
		DRIVER_VERSION_NUMBER);
}

static int mtch_check_firmware_format(struct device *dev,
				     const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/*
	 * To convert file to binary:
	 * xxd -r -p mtchXXX_APP_VX-X-XX.enc > maxtouch.fw
	 */
	dev_err(dev, "Aborting: firmware file must be in binary format\n");

	return -EINVAL;
}

static int mtch_load_fw(struct device *dev, const char *fn)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	unsigned int retry = 0;
	unsigned int frame = 0;
	int ret;

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		goto release_firmware;
	} else {
		dev_info(dev, "Opened firmware file: %s\n", fn);
	}

	/* Check for incorrect enc file */
	ret = mtch_check_firmware_format(dev, fw);

	if (ret)
		goto release_firmware;
	else
		dev_info(dev, "File format is okay\n");

	if (!data->in_bootloader) {
		/* Clear any pending messages before entering bootloader */
		ret = mtch_process_messages_until_invalid(data, true);
		if (ret)
			dev_warn(dev, "Failed to clear messages before bootloader\n");

		/* Ensure IRQ is enabled for bootloader state transitions */
		if (!data->irq_enabled) {
			enable_irq(data->irq);
			data->irq_enabled = true;
		}

		/* Change to the bootloader mode */
		data->in_bootloader = true;

		ret = mtch_t6_command(data, CMD_RESET_OFS,
				     BTLDR_RESET_VAL, false);
		if (ret)
			goto release_firmware;
		else
			dev_info(dev, "Sent bootloader command.\n");

		/* Switch to bootloader wait time */
		msleep(BTLDR_RESET_TIME);

		/* Do not need to scan since we know family ID */
		ret = mtch_lookup_bootloader_address(data);
		if (ret)
			goto release_firmware;
		else
			dev_info(dev, "Found bootloader I2C address\n");

		reinit_completion(&data->bl_completion);

		/* Fresh entry to bootloader - check for WAITING_BOOTLOAD_CMD */
		ret = mtch_check_bootloader(data, WAITING_BOOTLOAD_CMD, false);
		if (ret) {
			/* Bootloader may still be unlocked from previous attempt */
			ret = mtch_check_bootloader(data, WAITING_FRAME_DATA, false);
			if (ret)
				goto disable_irq;
		} else {
			dev_info(dev, "Unlocking bootloader\n");

			/* Unlock bootloader */
			ret = mtch_send_bootloader_cmd(data, true);
			if (ret)
				goto disable_irq;
		}
	} else {
		/*
		 * Already in bootloader from bad flash.
		 * Bootloader should already be unlocked and waiting for data.
		 */
		if (!data->irq_enabled) {
			enable_irq(data->irq);
			data->irq_enabled = true;
		}

		reinit_completion(&data->bl_completion);

		/* Check for WAITING_FRAME_DATA first (expected after bad flash) */
		ret = mtch_check_bootloader(data, WAITING_FRAME_DATA, false);
		if (ret) {
			/* Try WAITING_BOOTLOAD_CMD in case it needs unlock */
			ret = mtch_check_bootloader(data, WAITING_BOOTLOAD_CMD, false);
			if (ret)
				goto disable_irq;

			dev_info(dev, "Unlocking bootloader\n");
			ret = mtch_send_bootloader_cmd(data, true);
			if (ret)
				goto disable_irq;
		} else {
			dev_info(dev, "Bootloader already unlocked, ready for data\n");
		}
	}

	while (pos < fw->size) {
		ret = mtch_check_bootloader(data, WAITING_FRAME_DATA, true);
		if (ret)
			goto disable_irq;

		frame_size = ((*(fw->data + pos) << 8) |
			     *(fw->data + pos + 1));

		/* Take account of CRC bytes */
		frame_size += 2;

		/* Write one frame to device */
		ret = mtch_bootloader_write(data, &fw->data[pos], frame_size);
		if (ret)
			goto disable_irq;

		ret = mtch_check_bootloader(data, FRAME_CRC_PASS, true);
		if (ret) {
			retry++;

			/* Back off by 20ms per retry */
			msleep(retry * 20);

			if (retry > 20) {
				dev_err(dev, "Retry count exceeded\n");
				goto disable_irq;
			}
		} else {
			retry = 0;
			pos += frame_size;
			frame++;
		}

		if (pos >= fw->size) {
			dev_info(dev, "Sent %u frames, %zu bytes\n",
				frame, fw->size);
		} else if (frame % 50 == 0) {
			dev_info(dev, "Sent %u frames, %d/%zu bytes\n",
				frame, pos, fw->size);
		}
	}

	dev_dbg(dev, "Sent %d frames, %d bytes\n", frame, pos);

	/*
	 * Wait for device to reset. Some bootloader versions do not assert
	 * the CHG line after bootloading has finished, so ignore potential
	 * errors.
	 */

	msleep(BOOTLOADER_WAIT);	/* Wait for chip to leave bootloader*/

	ret = mtch_wait_for_completion(data, &data->bl_completion,
				      BOOTLOADER_WAIT);
	if (ret < 0)
		goto disable_irq;

	data->in_bootloader = false;

disable_irq:
	disable_irq(data->irq);
	data->irq_enabled = false;
release_firmware:
	release_firmware(fw);
	return ret;
}

static int mtch_update_file_name(struct device *dev, char **file_name,
				const char *buf, size_t count)
{
	char *file_name_tmp;

	/* Simple sanity check */
	if (count > 64) {
		dev_warn(dev, "File name too long\n");
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1, GFP_KERNEL);
	if (!file_name_tmp)
		return -ENOMEM;

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}

static ssize_t update_fw_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	char *file_name = NULL;
	int error;

	data->sysfs_updating_cfg_fw = true;
	data->irq_processing = true;

	error = mtch_update_file_name(dev, &file_name, buf, count);
	if (error) {
		dev_err(dev, "Failed to get filename: %s\n", buf);
		return error;
	}

	/* May not need this if FREEZE and UNFREEZE are used */
	error = mtch_clear_cfg(data);
	if (error)
		dev_err(dev, "Warning: Failed to clear configuration\n");

	error = mtch_load_fw(dev, MTCH_FW_NAME);

	if (error) {
		dev_err(dev,
			"The firmware update failed(%d)\n. IRQ disabled.",
			error);

		disable_irq(data->irq);
		data->irq_enabled = false;

		count = error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");
	}

	/* add wait after fw load before IRQ turn on */
	msleep(FW_FLASH_TIME);

	error = mtch_acquire_irq(data);
	if (error)
		return error;

	kfree(file_name);

	mtch_soft_reset(data, true);

	/* Read infoblock to determine device type */
	error = mtch_read_info_block(data);
	if (error)
		return error;

	error = request_firmware_nowait(THIS_MODULE, true, MTCH_CFG_NAME,
					dev, GFP_KERNEL, data,
					mtch_config_cb);

	if (error) {
		dev_warn(dev, "Failed to invoke firmware loader: %d\n",
			error);
		return error;
	}

	return count;
}

static ssize_t update_cfg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	const struct firmware *cfg;
	int ret, error;

	data->sysfs_updating_cfg_fw = true;

	ret = request_firmware(&cfg, MTCH_CFG_NAME, dev);
	if (ret < 0) {
		dev_err(dev, "Failure to request config file %s\n",
			MTCH_CFG_NAME);
		ret = -ENOENT;
		goto out;
	} else {
		dev_info(dev, "Found configuration file: %s\n",
			 MTCH_CFG_NAME);
	}

	/* Clear T5 message buffer before write */
	error = mtch_process_messages_until_invalid(data, true);
	if (error)
		dev_err(dev, "Failed process configuration\n");

	/* Cannot do this because I haven't saved the t7 cfg yet */
	if (!(IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED))) {
		error = mtch_sync_t7_power(data, POWER_CFG_RUN);
		if (error) {
			dev_err(dev, "Failed to initialize power cfg\n");
			return error;
		}
	}

	if (cfg) {
		error = mtch_update_cfg(data, cfg);
		if (error)
			dev_warn(dev, "Error %d updating config\n", error);
	} else {
		data->irq_processing = true;
	}

	if (!(IS_BITS_ENABLED(data->encryption_state, DEV_ENCRYPTED))) {
		error = mtch_check_retrigen(data);
		if (error)
			dev_err(dev, "RETRIGEN Not Enabled or unavailable\n");
	}

	data->sysfs_updating_cfg_fw = false;
	data->irq_processing = true;
	release_firmware(cfg);
out:
	return count;
}

static ssize_t mtch_debug_msg_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	return -EIO;
}

static ssize_t mtch_debug_msg_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t bytes)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mtch_data *data = dev_get_drvdata(dev);
	size_t count;
	size_t bytes_read = 0;

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return 0;
	}

	count = bytes / data->T5_msg_size;

	if (count > DEBUG_MSG_MAX)
		count = DEBUG_MSG_MAX;

	mutex_lock(&data->debug_msg_lock);

	if (count > data->debug_msg_count)
		count = data->debug_msg_count;

	if ((count * data->T5_msg_size) > PAGE_SIZE)
		dev_err(dev, "Message size is over limit\n");
	else
		bytes_read = count * data->T5_msg_size;

	memcpy(buf, data->debug_msg_data, bytes_read);
	data->debug_msg_count = 0;

	mutex_unlock(&data->debug_msg_lock);

	return bytes_read;
}

static int mtch_debug_msg_init(struct mtch_data *data)
{
	sysfs_bin_attr_init(&data->debug_msg_attr);
	data->debug_msg_attr.attr.name = "debug_msg";
	data->debug_msg_attr.attr.mode = 0666;
	data->debug_msg_attr.read = mtch_debug_msg_read;
	data->debug_msg_attr.write = mtch_debug_msg_write;
	data->debug_msg_attr.size = data->T5_msg_size * DEBUG_MSG_MAX;

	if (sysfs_create_bin_file(&data->client->dev.kobj,
				  &data->debug_msg_attr) < 0) {
		dev_err(&data->client->dev, "Failed to create %s\n",
			data->debug_msg_attr.attr.name);
		return -EINVAL;
	}

	return 0;
}

static void mtch_debug_msg_enable(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;

	if (data->debug_v2_enabled)
		return;

	mutex_lock(&data->debug_msg_lock);

	data->debug_msg_data = kcalloc(DEBUG_MSG_MAX,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->debug_msg_data)
		return;

	data->debug_v2_enabled = true;
	mutex_unlock(&data->debug_msg_lock);

	dev_dbg(dev, "Enabled message output\n");
}

static void mtch_debug_msg_disable(struct mtch_data *data)
{
	struct device *dev = &data->client->dev;

	if (!data->debug_v2_enabled)
		return;

	data->debug_v2_enabled = false;

	mutex_lock(&data->debug_msg_lock);
	kfree(data->debug_msg_data);
	data->debug_msg_data = NULL;
	data->debug_msg_count = 0;
	mutex_unlock(&data->debug_msg_lock);
	dev_dbg(dev, "Disabled message output\n");
}

static void mtch_debug_msg_remove(struct mtch_data *data)
{
	if (data->debug_msg_attr.attr.name)
		sysfs_remove_bin_file(&data->client->dev.kobj,
				      &data->debug_msg_attr);
}

static ssize_t debug_irq_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	char c;

	c = data->irq_processing ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t debug_irq_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		data->irq_processing = i;

		dev_dbg(dev, "%s\n", i ?
			"Debug IRQ enabled" :
			"Debug IRQ disabled");
		ret = count;
	} else {
		dev_dbg(dev, "debug_irq write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t debug_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		data->debug_enabled = (i == 1);

		dev_dbg(dev, "%s\n", i ? "debug enabled" : "debug disabled");
		ret = count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t debug_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	char c;

	c = data->debug_enabled ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mtch_debug_v2_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	u8 i;
	ssize_t ret;

	if (kstrtou8(buf, 0, &i) == 0 && i < 2) {
		if (i == 1)
			mtch_debug_msg_enable(data);
		else
			mtch_debug_msg_disable(data);

		ret = count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t debug_notify_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t mtch_reset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	char c;

	c = data->reset_state ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mtch_reset_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	ssize_t ret = 0;

	if (data->reset_state == false) {
		data->reset_state = true;
		ret = mtch_soft_reset(data, true);
		data->reset_state = false;
	}

	return count;
}

static int mtch_check_mem_access_params(struct mtch_data *data, loff_t off,
				       size_t *count)
{
	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	if (*count > MAX_BLOCK_RD_WR)
		*count = MAX_BLOCK_RD_WR;

	return 0;
}

static ssize_t mtch_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mtch_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mtch_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0) {
		ret = __mtch_read_reg(data, off, count, buf);
	}

	return ret == 0 ? count : ret;
}

static ssize_t mtch_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mtch_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mtch_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0) {
		ret = __mtch_write_reg(data, off, count, buf);
	}

	return ret == 0 ? count : ret;
}

static ssize_t wheel_relative_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mtch_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%c\n",
			 data->wheel_relative ? '1' : '0');
}

static ssize_t wheel_relative_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mtch_data *data = dev_get_drvdata(dev);
	u8 val;
	ssize_t ret;

	if (kstrtou8(buf, 0, &val) == 0 && val < 2) {
		data->wheel_relative = (val == 1);
		dev_dbg(dev, "Wheel mode set to %s\n",
			data->wheel_relative ? "relative" : "absolute");
		ret = count;
	} else {
		dev_dbg(dev, "wheel_relative write error\n");
		ret = -EINVAL;
	}

	return ret;
}

static DEVICE_ATTR_RO(fw_version);
static DEVICE_ATTR_RO(hw_version);
static DEVICE_ATTR_RO(drvr_version);
static DEVICE_ATTR_RO(config_crc);
static DEVICE_ATTR_RO(debug_notify);
static DEVICE_ATTR_WO(update_cfg);
static DEVICE_ATTR_WO(update_fw);

static DEVICE_ATTR(debug_v2_enable, 0600, NULL, mtch_debug_v2_enable_store);
static DEVICE_ATTR(debug_enable, 0600, debug_enable_show,
		   debug_enable_store);
static DEVICE_ATTR(debug_irq, 0600, debug_irq_show, debug_irq_store);
static DEVICE_ATTR(mtch_reset, 0600, mtch_reset_show, mtch_reset_store);
static DEVICE_ATTR(wheel_relative, 0644, wheel_relative_show,
		   wheel_relative_store);

static struct attribute *mtch_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_drvr_version.attr,
	&dev_attr_config_crc.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_update_cfg.attr,
	&dev_attr_debug_irq.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_debug_v2_enable.attr,
	&dev_attr_debug_notify.attr,
	&dev_attr_mtch_reset.attr,
	&dev_attr_wheel_relative.attr,
	NULL
};

static const struct attribute_group mtch_attr_group = {
	.attrs = mtch_attrs,
};

static const struct attribute_group *mtch_groups[] = {
	&mtch_attr_group,
	NULL
};

static int mtch_sysfs_init(struct mtch_data *data)
{
	struct i2c_client *client = data->client;
	int error;

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = 0644;
	data->mem_access_attr.read = mtch_mem_access_read;
	data->mem_access_attr.write = mtch_mem_access_write;

	if (data->T2_address) {
		data->mem_access_attr.size = data->mem_size +
		data->num_datasize_bytes;
	} else {
		data->mem_access_attr.size = data->mem_size;
	}

	error = sysfs_create_bin_file(&client->dev.kobj,
				      &data->mem_access_attr);
	if (error) {
		dev_err(&client->dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		return error;
	}

	return 0;
}

static void mtch_sysfs_remove(struct mtch_data *data)
{
	struct i2c_client *client = data->client;

	if (data->mem_access_attr.attr.name)
		sysfs_remove_bin_file(&client->dev.kobj,
				      &data->mem_access_attr);
}

static int mtch_parse_device_properties(struct mtch_data *data)
{
	static const char keymap_property[] = "linux,gpio-keymap";
	static const char config_property[] = "mtch,cfg_name";
	struct device *dev = &data->client->dev;
	u32 *keymap;
	int n_keys;
	int error;

	if (device_property_present(dev, keymap_property)) {
		n_keys = device_property_read_u32_array(dev, keymap_property,
							NULL, 0);
		if (n_keys <= 0) {
			error = n_keys < 0 ? n_keys : -EINVAL;
			dev_err(dev, "invalid/malformed '%s' property: %d\n",
				keymap_property, error);
			return error;
		}

		keymap = devm_kmalloc_array(dev, n_keys, sizeof(*keymap),
					    GFP_KERNEL);
		if (!keymap)
			return -ENOMEM;

		error = device_property_read_u32_array(dev, keymap_property,
						       keymap, n_keys);
		if (error) {
			dev_err(dev, "failed to parse '%s' property: %d\n",
				keymap_property, error);
			return error;
		}

		data->t19_keymap = keymap;
		data->t19_num_keys = n_keys;
	}

	if (device_property_present(dev, config_property)) {

		error = device_property_read_string(dev, config_property,
			&data->cfg_name);

		if (error)
			dev_err(dev, "failed to parse '%s' property: %d\n",
				config_property, error);
	}

	/* Parse wheel relative mode property (default: absolute) */
	data->wheel_relative = device_property_read_bool(dev, "mtch,wheel-relative");
	if (data->wheel_relative)
		dev_info(dev, "Wheel configured for relative reporting\n");

	return 0;
}

static int mtch_probe(struct i2c_client *client)
{
	struct mtch_data *data;
	int error;

	/*
	 * Ignore devices that do not have device properties attached to
	 * them.
	 */
	if (!device_property_present(&client->dev, "compatible"))
		return -ENXIO;

	data = devm_kzalloc(&client->dev, sizeof(struct mtch_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);

	data->client = client;
	data->irq = client->irq;
	i2c_set_clientdata(client, data);

	init_completion(&data->bl_completion);
	init_completion(&data->reset_completion);
	init_completion(&data->crc_completion);
	init_completion(&data->t68_completion);
	mutex_init(&data->i2c_lock);

	/* Search for device properties */
	error = mtch_parse_device_properties(data);
	if (error)
		return error;

	/* Regulator setup */
	/*
	 * VDDA is the analog voltage supply 2.57..3.47 V
	 * VDD  is the digital voltage supply 1.71..3.47 V
	 * VDDIO is supplied by VDD or from Host
	 */
	data->regulators[0].supply = "vdda";
	data->regulators[1].supply = "vdd";
	error = devm_regulator_bulk_get(&client->dev, ARRAY_SIZE(data->regulators),
					data->regulators);
	if (error) {
		if (error != -EPROBE_DEFER)
			dev_err(&client->dev, "Failed to get regulators %d\n",
				error);
		return error;
	}

	/* Deassert RESET line, keep it 'HIGH' on power up */
	/* Otherwise, might be in bootloader mode */
	data->reset_gpio = devm_gpiod_get_optional(&client->dev,
						   "reset", GPIOD_OUT_LOW);
	if (IS_ERR(data->reset_gpio)) {
		error = PTR_ERR(data->reset_gpio);
		dev_err(&client->dev, "Failed to get reset gpio: %d\n", error);
		return error;
	}

	/* Request IRQ but do not enable until later */
	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, mtch_interrupt,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN,
					  client->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		return error;
	}

	error = regulator_bulk_enable(ARRAY_SIZE(data->regulators),
				      data->regulators);
	if (error) {
		dev_err(&client->dev, "failed to enable regulators: %d\n",
			error);
		return error;
	}

	msleep(BACKUP_TIME);

	if (data->reset_gpio) {
		dev_dbg(&client->dev, "Resetting chip\n");
		/* Wait a while and then de-assert the RESET GPIO line */
		msleep(RESET_GPIO_TIME);
		gpiod_set_value(data->reset_gpio, 0);
		msleep(RESET_INVALID_CHG);
	}

	/* Initialize varaiables used for power up conditions */
	data->sysfs_updating_cfg_fw = false;
	data->irq_processing = true;
	data->system_power_up = true;
	data->hc.num_of_devs = 0x01;

	error = mtch_initialize(data);
	if (error)
		goto err_disable_regulators;

	/* Enable debugfs */
	mtch_prepare_debugfs(data, dev_driver_string(&client->dev));

	/* Enable sysfs */

	error = mtch_sysfs_init(data);
	if (error)
		return error;

	return 0;

err_disable_regulators:
	regulator_bulk_disable(ARRAY_SIZE(data->regulators),
			       data->regulators);
	return error;
}

static void mtch_remove(struct i2c_client *client)
{
	struct mtch_data *data = i2c_get_clientdata(client);

	disable_irq(data->irq);
	data->irq_enabled = false;
	mtch_debug_msg_remove(data);
	mtch_sysfs_remove(data);
	mtch_free_input_device(data);
	mtch_free_object_table(data);
	mtch_teardown_debugfs(data);
	regulator_bulk_disable(ARRAY_SIZE(data->regulators),
			       data->regulators);
}

static int mtch_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mtch_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	if (!input_dev)
		return 0;

	mutex_lock(&input_dev->mutex);

	if (input_device_enabled(input_dev))
		mtch_stop(data);

	mutex_unlock(&input_dev->mutex);

	disable_irq(data->irq);
	data->irq_enabled = false;

	return 0;
}

static int mtch_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mtch_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	if (!input_dev)
		return 0;

	mutex_lock(&input_dev->mutex);

	if (input_device_enabled(input_dev))
		mtch_start(data);

	mutex_unlock(&input_dev->mutex);

	return 0;

}

static DEFINE_SIMPLE_DEV_PM_OPS(mtch_pm_ops, mtch_suspend, mtch_resume);

static const struct of_device_id mtch_of_match[] = {
	{ .compatible = "mchp,mtch", },
	{ },
};
MODULE_DEVICE_TABLE(of, mtch_of_match);

static const struct i2c_device_id mtch_id[] = {
	{ "mchp_mtch" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mtch_id);

static struct i2c_driver mtch_driver = {
	.driver = {
		.name = "mchp_mtch",
		.dev_groups = mtch_groups,
		.of_match_table = mtch_of_match,
		.pm = pm_sleep_ptr(&mtch_pm_ops),
	},
	.probe = mtch_probe,
	.remove = mtch_remove,
	.id_table = mtch_id,
};

module_i2c_driver(mtch_driver);

/* Module information */
MODULE_AUTHOR("Michael Gong <michael.gong@microchip.com>");
MODULE_DESCRIPTION("Microchip MTCH driver");
MODULE_LICENSE("GPL");