/*
 * Driver for MAXIM CMOS Image Sensor from MAXIM
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/module.h>
#include <linux/v4l2-mediabus.h>
#include <linux/media-bus-format.h>

#include <linux/clk.h>
#include <media/soc_camera.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>

/*
 * About MAX resolution, cropping and binning:
 * This sensor supports it all, at least in the feature description.
 * Unfortunately, no combination of appropriate registers settings could make
 * the chip work the intended way. As it works with predefined register lists,
 * some undocumented registers are presumably changed there to achieve their
 * goals.
 * This driver currently only works for resolutions up to 720 lines with a
 * 1:1 scale. Hopefully these restrictions will be removed in the future.
 */
#define MAXIM_MAX_WIDTH	2592
#define MAXIM_MAX_HEIGHT	720
#define MAX9286_ADDR 0x94
#define MAX9286_ID 0x40
#define MAX9286_LINK_ENABLE_REG_ADDR 0x00U
#define MAX9286_FRAME_SYNC_REG_ADDR 0x01U
#define MAX9286_FSYNC_PERIODM_REG_ADDR 0x07U
#define MAX9286_OUT_ORDER_REG_ADDR 0x0BU
#define MAX9286_OUTLANE_REG_ADDR 0x12U
#define MAX9286_F_R_CTL_REG_ADDR 0x0AU
#define MAX_REG_LEN 2
#define MAX96705_INIT_ADDR 0x80
#define MAX96705_ALL_ADDR 0x80
#define MAX96705_CH0_ADDR 0x82
#define MAX96705_CH1_ADDR 0x84
#define MAX96705_CH2_ADDR 0x86
#define MAX96705_CH3_ADDR 0x88
#define ISX016_CH0_ADDR 0x62
#define ISX016_INIT_ADDR 0x34
#define ISX016_CH0_MAP_ADDR 0x60
#define ISX016_CH1_MAP_ADDR 0x62
#define ISX016_CH2_MAP_ADDR 0x64
#define ISX016_CH3_MAP_ADDR 0x66
#define MAX9286_1CH_LANE 1
//#define KSS_TEST_PATTERN

#define MAX9286_LINK_CONFIG_REG 0x00
#define MAX9286_ID_REG   0x1E
#define MAX9286_LOCK_REG 0x27
#define MAX9286_LINK_REG 0x49

/* minimum extra blanking */
#define BLANKING_EXTRA_WIDTH		500
#define BLANKING_EXTRA_HEIGHT		20

#define SENSOR_MAX_LINK_NUM 4

#define MAX9286_CAMERA_DIS_TP0_VALUE 0x00U
#define MAX9286_CAMERA_DIS_TP1_VALUE 0x01U
#define MAX9286_CAMERA_DIS_TP2_VALUE 0x03U


/*
 * the sensor's autoexposure is buggy when setting total_height low.
 * It tries to expose longer than 1 frame period without taking care of it
 * and this leads to weird output. So we set 1000 lines as minimum.
 */
#define BLANKING_MIN_HEIGHT	1000
#define SENSOR_ID               (0x86)

#define max9286_info(fmt, args...)                \
		((void)pr_info("[max9286_gladius][info] %s %d: %s " fmt "\n",\
			__func__, __LINE__, dev_name(&client->dev), ##args))

#define max9286_err(fmt, args...)                \
		((void)pr_info("[max9286_gladius][error] %s %d: %s " fmt "\n",\
			__func__, __LINE__, dev_name(&client->dev), ##args))

/*add for debug*/
#define __DEBUG__

#ifdef __DEBUG__
#define debug(fmt,...) printk("[debug_info] %s:%d " fmt "\n",__func__,__LINE__,##__VA_ARGS__)
#else
#define debug(fmt,...)
#endif

struct max9286_pinctrl_info{
    struct pinctrl *pinctrl;
    struct pinctrl_state *gpio_state_active;//defaut
    struct pinctrl_state *gpio_state_suspend;
    struct pinctrl_state *gpio6_enable_state;
    struct pinctrl_state *gpio6_disenable_state;
    struct regulator  *vdd1v8;
}max9286_pctrl;

struct reg_val_ops {
	u16 slave_addr;
	u8 reg[MAX_REG_LEN];
	u8 val;
	unsigned int reg_len;
	int (*i2c_ops)(struct i2c_client *client, u16 slave_addr,
		u8 *reg, unsigned int reg_len, u8 *val);
};

static int i2c_write(struct i2c_client *client, u16 slave_addr,
		u8 *reg, unsigned int reg_len, u8 *val);
static int i2c_read(struct i2c_client *client, u16 slave_addr,
		u8 *reg, unsigned int reg_len, u8 *val);
static int i2c_delay(struct i2c_client *client, u16 slave_addr,
		u8 *reg, unsigned int reg_len, u8 *value);
static int read_max9286_id(struct i2c_client *client, u8 *id_val);
static int max9286_write_array(struct i2c_client *client,
		struct reg_val_ops *cmd, unsigned long len);
static int pmu1v8_power_init(struct device *dev,struct device_node *node);

/* Supported resolutions */
enum max9286_width {
	W_TESTWIDTH	= 720,
	W_WIDTH		= 736,
};

enum max9286_height {
	H_1CHANNEL	= 480,
	H_4CHANNEL	= 1920,
};

struct max9286_win_size {
	enum max9286_width width;
	enum max9286_height height;
};

static const struct max9286_win_size max9286_supported_win_sizes[] = {
	{ W_TESTWIDTH,	H_1CHANNEL },
	{ W_TESTWIDTH,	H_4CHANNEL },
	{ W_WIDTH,	H_1CHANNEL },
	{ W_WIDTH,	H_4CHANNEL },
};

static int is_testpattern;
static int is_exec_testpattern;
struct sensor_addr {
	u16 ser_init_addr;
	u16 ser_last_addr;
	u16 isp_init_addr;
	u16 isp_map_addr;
};

#define KSS_WIDTH 1280U
#define KSS_HEIGHT 800U

static u16 ch_addr[] = {
	MAX96705_CH0_ADDR,
	MAX96705_CH1_ADDR,
	MAX96705_CH2_ADDR,
	MAX96705_CH3_ADDR,
};

static struct reg_val_ops MAX9286_camera_r_cmd[] = {
	{MAX96705_CH0_ADDR, {0x00, 0x00}, MAX96705_CH0_ADDR,    0x01, i2c_read},
	{MAX96705_CH1_ADDR, {0x00, 0x00}, MAX96705_CH1_ADDR,    0x01, i2c_read},
	{MAX96705_CH2_ADDR, {0x00, 0x00}, MAX96705_CH2_ADDR,    0x01, i2c_read},
	{MAX96705_CH3_ADDR, {0x00, 0x00}, MAX96705_CH3_ADDR,    0x01, i2c_read},
	//{MAX9286_ADDR,     {0x15, 0x00}, 0x9B,                0x01, i2c_read},
};

static struct reg_val_ops MAX9286_camera_pre_init_cmd[] = {
	{MAX9286_ADDR,       {0x0D, 0x00}, 0x03,               0x01, i2c_write}, // add by nio
	{MAX9286_ADDR,       {0x3F, 0x00}, 0x4F,               0x01, i2c_write},
	{MAX9286_ADDR,       {0x00, 0x00}, 0x02,               0x01, i2c_delay},
	{MAX9286_ADDR,       {0x3B, 0x00}, 0x1E,               0x01, i2c_write},
	{MAX9286_ADDR,       {0x00, 0x00}, 0x02,               0x01, i2c_delay},
	{MAX96705_INIT_ADDR, {0x04, 0x00}, 0x43,               0x01, i2c_write},
	{MAX96705_INIT_ADDR, {0x03, 0x00}, 0x80,               0x01, i2c_write}, // add by nio
	{MAX9286_ADDR,       {0x00, 0x00}, 0x05,               0x01, i2c_delay},
	{MAX9286_ADDR,       {0x28, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9286_ADDR,       {0x28, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9286_ADDR,       {0x29, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9286_ADDR,       {0x29, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9286_ADDR,       {0x2A, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9286_ADDR,       {0x2A, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9286_ADDR,       {0x2B, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9286_ADDR,       {0x2B, 0x00}, 0x00,               0x01, i2c_read},

	{MAX96705_INIT_ADDR, {0x08, 0x00}, 0x01,               0x01, i2c_write},
	{MAX96705_INIT_ADDR, {0x97, 0x00}, 0xAF,               0x01, i2c_write}, // add by nio
	{MAX9286_ADDR,       {0x00, 0x00}, 0x02,               0x01, i2c_delay},
	{MAX9286_ADDR,       {0x3B, 0x00}, 0x19,               0x01, i2c_write}, // remove by nio
	{MAX9286_ADDR,       {0x00, 0x00}, 0x02,               0x01, i2c_delay},
	{MAX9286_ADDR,	     {0x34, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9286_ADDR,	     {0x1B, 0x00}, 0x0F,               0x01, i2c_read}, // add by nio

	{MAX9286_ADDR,       {0x15, 0x00}, 0x13,                0x01, i2c_write},
};

static struct reg_val_ops MAX9286_camera_init_cmd[] = {
	{MAX9286_ADDR,      {0x19, 0x00}, 0xa3,               0x01, i2c_write},
	{MAX9286_ADDR,      {0x41, 0x00}, 0x10,               0x01, i2c_write},
	{MAX9286_ADDR,      {0x02, 0x00}, 0x00,               0x01, i2c_write},
	{MAX9286_ADDR,      {0x63, 0x00}, 0x00,               0x01, i2c_write},
	{MAX9286_ADDR,      {0x64, 0x00}, 0x00,               0x01, i2c_write},
};

#define MAX9286_CAMERA_CH_ADDR_INIT_CMD(ch)\
	static struct reg_val_ops MAX9286_CAMERA_CH##ch##_addr_init_cmd[] = {\
		{MAX9286_ADDR,           {0x0A, 0x00}, (0x01U) << ch | 0xF0,     0x01, i2c_write}, \
		{MAX96705_INIT_ADDR,     {0x00, 0x00}, MAX96705_CH##ch##_ADDR,   0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x06, 0x00}, 0x80,	                 0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x0E, 0x00}, 0x00,	                 0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x3f, 0x00}, 0x0d,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x41, 0x00}, 0x0e,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x43, 0x00}, 0x01,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x44, 0x00}, 0x23,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x45, 0x00}, 0xa9,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x46, 0x00}, 0xd4,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x47, 0x00}, 0x01,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x48, 0x00}, 0x00,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x49, 0x00}, 0x00,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x4a, 0x00}, 0x24,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x4b, 0x00}, 0xd7,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x4c, 0x00}, 0x80,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x43, 0x00}, 0x21,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x4d, 0x00}, 0x00,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x67, 0x00}, 0xc4,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x07, 0x00}, 0x84,                     0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x09, 0x00}, ISX016_CH##ch##_MAP_ADDR, 0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x0A, 0x00}, ISX016_INIT_ADDR,         0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x0B, 0x00}, MAX96705_ALL_ADDR,        0x01, i2c_write}, \
		{MAX96705_CH##ch##_ADDR, {0x0C, 0x00}, MAX96705_CH##ch##_ADDR,	 0x01, i2c_write}, \
		{MAX9286_ADDR,           {0x00, 0x00}, 0x02,                     0x01, i2c_delay}, \
	}

static struct reg_val_ops MAX9286_camera_en_cmd[] = {
	{MAX96705_ALL_ADDR, {0x04, 0x00}, 0x83,               0x01, i2c_write},
	{MAX9286_ADDR,      {0x15, 0x00}, 0x9B,               0x01, i2c_write},
};

MAX9286_CAMERA_CH_ADDR_INIT_CMD(0);
MAX9286_CAMERA_CH_ADDR_INIT_CMD(1);
MAX9286_CAMERA_CH_ADDR_INIT_CMD(2);
MAX9286_CAMERA_CH_ADDR_INIT_CMD(3);

static struct reg_val_ops *init_ch[] = {
	MAX9286_CAMERA_CH0_addr_init_cmd,
	MAX9286_CAMERA_CH1_addr_init_cmd,
	MAX9286_CAMERA_CH2_addr_init_cmd,
	MAX9286_CAMERA_CH3_addr_init_cmd,
};

#define MAX96705_CROSS_BAR_CH_ADDR_CMD(ch)\
	static struct reg_val_ops MAX96705_CROSS_BAR_CH##ch##_addr_cmd[] = {\
		{MAX96705_CH##ch##_ADDR,  {0x20, 0x00}, 0x17,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x21, 0x00}, 0x16,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x22, 0x00}, 0x15,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x23, 0x00}, 0x14,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x24, 0x00}, 0x13,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x25, 0x00}, 0x12,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x26, 0x00}, 0x11,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x27, 0x00}, 0x10,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x28, 0x00}, 0x18,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x29, 0x00}, 0x19,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x2a, 0x00}, 0x1a,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x2b, 0x00}, 0x1b,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x2c, 0x00}, 0x1c,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x2d, 0x00}, 0x0d,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x2e, 0x00}, 0x0e,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x2f, 0x00}, 0x0f,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x30, 0x00}, 0x07,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x31, 0x00}, 0x06,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x32, 0x00}, 0x05,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x33, 0x00}, 0x04,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x34, 0x00}, 0x03,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x35, 0x00}, 0x02,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x36, 0x00}, 0x01,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x37, 0x00}, 0x00,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x38, 0x00}, 0x08,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x39, 0x00}, 0x09,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x3a, 0x00}, 0x0a,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x3b, 0x00}, 0x0b,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x3c, 0x00}, 0x0c,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x3d, 0x00}, 0x0d,              0x01, i2c_write},	\
		{MAX96705_CH##ch##_ADDR,  {0x3e, 0x00}, 0x0e,              0x01, i2c_write},	\
	}

MAX96705_CROSS_BAR_CH_ADDR_CMD(0);
MAX96705_CROSS_BAR_CH_ADDR_CMD(1);
MAX96705_CROSS_BAR_CH_ADDR_CMD(2);
MAX96705_CROSS_BAR_CH_ADDR_CMD(3);

static struct reg_val_ops *cross_bar_ch[] = {
	MAX96705_CROSS_BAR_CH0_addr_cmd,
	MAX96705_CROSS_BAR_CH1_addr_cmd,
	MAX96705_CROSS_BAR_CH2_addr_cmd,
	MAX96705_CROSS_BAR_CH3_addr_cmd,
};

#define MAX9286_CAMERA_TEST_PATTERN(num)\
	static struct reg_val_ops MAX9286_camera_dis_tp##num##_cmd[] = {\
		{ISX016_INIT_ADDR,    {0xFF, 0xFD}, 0x80,                                0x02, i2c_write}, \
		{ISX016_INIT_ADDR,    {0xFF, 0xFE}, 0x19,                                0x02, i2c_write}, \
		{ISX016_INIT_ADDR,    {0x50, 0x00}, MAX9286_CAMERA_DIS_TP##num##_VALUE,  0x02, i2c_write}, \
		{ISX016_INIT_ADDR,    {0xFF, 0xFE}, 0x80,                                0x02, i2c_write}, \
		{ISX016_INIT_ADDR,    {0x00, 0xC0}, 0xD6,                                0x02, i2c_write}, \
	}

MAX9286_CAMERA_TEST_PATTERN(0);
MAX9286_CAMERA_TEST_PATTERN(1);
MAX9286_CAMERA_TEST_PATTERN(2);



struct max9286_datafmt {
	__u32 code;
	enum v4l2_colorspace colorspace;
};

struct max9286 {
	struct v4l2_subdev		subdev;
	const struct max9286_datafmt	*fmt;
	struct v4l2_clk			*clk;
	struct i2c_client *client;

	/* blanking information */
	u32 link;
};

static const struct max9286_datafmt max9286_colour_fmts[] = {
	{MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_DEFAULT},
	{MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_DEFAULT},
};

static struct max9286 *to_max9286(struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct max9286, subdev);
}

u8 max9286_clear_bit(u8 orig, u8 which_bit)
{
	u8 result = orig & (~(0x01 << which_bit));
	return result;
}

u8 max9286_set_bit(u8 orig, u8 which_bit)
{
	u8 result = orig | (0x01 << which_bit);
	return result;
}
static u8 linked_ch_count(u8 *link_reg_val)
{
	u8 i = 0U;
	u8 count = 0U;

	for (i = 0U; i < SENSOR_MAX_LINK_NUM; i++)
		count += ((*link_reg_val >> i) & 0x01U);
	return count;
}

static int i2c_delay(struct i2c_client *client, u16 slave_addr,
		u8 *reg, unsigned int reg_len, u8 *value)
{
	max9286_info("delay %d ms", *value);
	mdelay(*value);

	return 0;
}

static int i2c_read(struct i2c_client *client, u16 slave_addr,
		u8 *reg, unsigned int reg_len, u8 *val)
{
	int ret = 0;
	u8 *data = NULL;
	struct i2c_msg msg[2];
	unsigned int size = reg_len + 1u;

	if ((reg == NULL) || (val == NULL) || (reg_len <= 0u)) {
		max9286_err("reg/val/reg_len is %02x/%02x/%d",
			*reg, *val, reg_len);
		return -EINVAL;
	}

	data = kzalloc(size, GFP_KERNEL);
	if (data == NULL)
		return -ENOSPC;

	(void)memcpy(data, reg, reg_len);
	(void)memset(msg, 0, sizeof(msg));

	msg[0].addr = (slave_addr >> 1);
	msg[0].flags = 0;
	msg[0].len = (__u16)reg_len;
	msg[0].buf = data;

	msg[1].addr = (slave_addr >> 1);
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + reg_len;

	client->addr = (slave_addr >> 1);

	ret = i2c_transfer(client->adapter, msg, 2);

	*val = *(data + reg_len);
	kfree(data);
	data = NULL;
	max9286_info("read dev/reg/val/ret is %02x/%02x/%02x/%d",
		slave_addr, *reg, *val, ret);

	return ret;
}

static int i2c_write(struct i2c_client *client, u16 slave_addr,
		u8 *reg, unsigned int reg_len, u8 *val)
{
	u8 *data = NULL;
	int ret = 0;
	struct i2c_msg msg;
	unsigned int size = reg_len + 1u;

	if ((reg == NULL) || (val == NULL) || (reg_len <= 0u)) {
		max9286_err("reg/val/reg_len is %02x/%02x/%d",
			*reg, *val, reg_len);
		return -EINVAL;
	}

	data = kzalloc(size, GFP_KERNEL);
	if (data == NULL)
		return -ENOSPC;

	(void)memcpy(data, reg, reg_len);
	*(data + reg_len) = *val;

	(void)memset(&msg, 0, sizeof(msg));
	msg.addr = (slave_addr >> 1);
	msg.flags = 0;
	msg.len = (u16)size;
	msg.buf = data;

	client->addr = (slave_addr >> 1);
	ret = i2c_transfer(client->adapter, &msg, 1);

	kfree(data);
	data = NULL;
	//max9286_info("write dev/reg/val/ret is %02x/%02x/%02x/%d",
	//	slave_addr, *reg, *val, ret);
	return ret;
}

int i2c_write_t(struct i2c_client *client,u16 slave_addr, u8 *addr, u16 *val)
{
	int ret;
	u8 buf[3] = { *addr & 0xff, *val >> 8, *val & 0xff };
	client->addr = (slave_addr >> 1);
	ret = i2c_master_send(client, buf, 3);
	debug("i2c_write: slave:%02x 0x%02x : 0x%04x\n",client->addr,*addr, *val);
	return ret == 3 ? 0 : ret;
}

int i2c_read_t(struct i2c_client *client, u16 slave_addr,
		u8 *reg, unsigned int reg_len, u16 *val)
{
	int ret = 0;
	u8 *data = NULL;
	struct i2c_msg msg[2];
	u8 rbuf[2];
	unsigned int size = reg_len + 1u;

	if ((reg == NULL) || (val == NULL) || (reg_len <= 0u)) {
		max9286_err("reg/val/reg_len is %02x/%02x/%d",
			*reg, *val, reg_len);
		return -EINVAL;
	}

	data = kzalloc(size, GFP_KERNEL);
	if (data == NULL)
		return -ENOSPC;

	(void)memcpy(data, reg, reg_len);
	(void)memset(msg, 0, sizeof(msg));

	msg[0].addr = (slave_addr >> 1);
	msg[0].flags = 0;
	msg[0].len = (__u16)reg_len;
	msg[0].buf = data;

	msg[1].addr = (slave_addr >> 1);
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = rbuf;

	client->addr = (slave_addr >> 1);

	ret = i2c_transfer(client->adapter, msg, 2);
	*val = be16_to_cpu(*((__be16 *)rbuf));
	debug("0x%02X : 0x%04x\n", *reg, *val);
	debug("rbuf[0]=%02x,rbuf[1]=%02x\n",rbuf[0],rbuf[1]);
	kfree(data);
	data = NULL;
	max9286_info("read dev/reg/val/ret is %02x/%02x/%04x/%d",
		slave_addr, *reg, *val, ret);

	return ret;
}

static int read_max9286_id(struct i2c_client *client, u8 *id_val)
{
	int ret = 0;
	u8 max9286_id_reg = MAX9286_ID_REG;

	ret = i2c_read(client, MAX9286_ADDR, &max9286_id_reg, 1, id_val);
	if (ret != 2) {
		max9286_err("ret=%d", ret);
		return -EIO;
	}

	if (*id_val != (u8)MAX9286_ID) {
		max9286_err("max9286 ID not match. Default is %x but read from register is %x. ret=%d",
			MAX9286_ID, *id_val, ret);
		ret = -ENODEV;
	} else {
		max9286_info("max9286 ID match. Default is %x and read from register is %x. ret=%d",
			MAX9286_ID, *id_val, ret);
	}

	return ret;
}

static int max9286_write_array(struct i2c_client *client,
		struct reg_val_ops *cmd, unsigned long len)
{
	int ret = 0;
	unsigned long index = 0;

	for (; index < len; ++index) {
		ret = cmd[index].i2c_ops(client, cmd[index].slave_addr,
			cmd[index].reg, cmd[index].reg_len, &(cmd[index].val));
		if (ret < 0) {
			max9286_err("dev/reg/val/ret/index %x/%x/%x/%d/%lu",
				cmd[index].slave_addr, cmd[index].reg[0],
				cmd[index].val, ret, index);
			return ret;
		}
	}

	return 0;
}

static int max9286_get_lock_status(struct i2c_client *client, u8 *val)
{
	int ret = 0;
	u8 lock_reg = MAX9286_LOCK_REG;

	ret = i2c_read(client, MAX9286_ADDR, &lock_reg, 1, val);
	if (ret != 2) {
		max9286_err("ret=%d", ret);
		return -EIO;
	}

	if ((*val & (u8)0x80) != 0u) {
		max9286_info("camera links are locked");
	} else {
		max9286_err("camera links are not locked");
		ret =  -ENODEV;
	}

	return ret;
}

static int max9286_get_link(struct i2c_client *client, u8 *val)
{
	int ret = 0;
	u8 link_reg = MAX9286_LINK_REG;

	ret = i2c_read(client, MAX9286_ADDR, &link_reg, 1, val);
	if (ret != 2) {
		max9286_err("ret=%d", ret);
		ret = -EIO;
	}
    debug("val=%d ret=%d\n",*val,ret);
	return ret;
}

static int max9286_camera_has_init(struct i2c_client *client, u8 *link_reg_val)
{
	unsigned long len = ARRAY_SIZE(MAX9286_camera_r_cmd);
	struct reg_val_ops *cmd = MAX9286_camera_r_cmd;
	unsigned long index = 0UL;
	int ret = 0;
	u8 val = 0U;
	u8 count = 0U;
	u8 reg_addr[2] = {0x00U};
	u8 reg_value = 0x00U;
	struct max9286 *priv = NULL;

	max9286_info("in max9286_camera_has_init, link_reg_val is 0x%x\n",
		*link_reg_val);
	for (index = 0; index < len; ++index) {
		if (((*link_reg_val >> index) & 0x01) == 0x00U)
			continue;
		val = cmd[index].val;
		ret = cmd[index].i2c_ops(client, cmd[index].slave_addr,
			cmd[index].reg, cmd[index].reg_len, &(cmd[index].val));
		if (ret < 0) {
			max9286_info("channel %lu has reseted\n", index);
		} else {
			count++;
			max9286_info("channel %lu keep last setting\n", index);

		}
	}
	priv = to_max9286(client);
	if (count == priv->link) {
		max9286_info("all channel keep the last time setting, will not initialize again\n");
		ret = 0;
	} else if (count == 0) {
		max9286_info("all channel has reseted, will initialize again\n");
		ret = 1;
	} else {
		max9286_info("some of channel has reseted, will set all channel address to 0x80\n");
		for (index = 0; index < len; ++index) {
			if (((*link_reg_val >> index) & 0x01) == 0x00U)
				continue;

			reg_value = MAX96705_INIT_ADDR;
			reg_addr[0] = 0x00U;
			ret = i2c_write(client, ch_addr[index],
				reg_addr, 0x01, &reg_value);
			if (ret < 0) {
				max9286_err("dev/reg/val/ret/index %x/%x/%x/%d/%lu",
					cmd[index].slave_addr,
					cmd[index].reg[0],
					cmd[index].val, ret, index);
				return ret;
			}
		}
		ret = 2;
	}
	return ret;
}

static int set_output_order(struct i2c_client *client, u8 *link_reg_val)
{
	int ret = 0;
	u8 reg_addr[2] = {0x00U, 0x00U};
	u8 reg_value = 0x00;
	u8 linked_count = 0U;
	u8 unlinked_count = 0U;
	u8 i = 0U;

	reg_addr[0] = 0x0B;
	for (i = 0; i < SENSOR_MAX_LINK_NUM; i++) {
		if (((*link_reg_val >> i) & 0x01U) == 0x01U) {
			switch (linked_count) {
			case 0:
				reg_value = max9286_clear_bit(reg_value,
					i * 2U);
				reg_value = max9286_clear_bit(reg_value,
					i * 2U + 1);
				break;
			case 1:
				reg_value = max9286_set_bit(reg_value,
					i * 2U);
				reg_value = max9286_clear_bit(reg_value,
					i * 2U + 1);
				break;
			case 2:
				reg_value = max9286_clear_bit(reg_value,
					i * 2U);
				reg_value = max9286_set_bit(reg_value,
					i * 2U + 1);
				break;
			case 3:
				reg_value = max9286_set_bit(reg_value,
					i * 2U);
				reg_value = max9286_set_bit(reg_value,
					i * 2U + 1);
				break;
			default:
				break;
			}
			linked_count++;
		} else {
			switch (unlinked_count) {
			case 0:
				reg_value = max9286_set_bit(reg_value,
					i * 2U);
				reg_value = max9286_set_bit(reg_value,
					i * 2U + 1);
				break;
			case 1:
				reg_value = max9286_clear_bit(reg_value,
					i * 2U);
				reg_value = max9286_set_bit(reg_value,
					i * 2U + 1);
				break;
			case 2:
				reg_value = max9286_set_bit(reg_value,
					i * 2U);
				reg_value = max9286_clear_bit(reg_value,
					i * 2U + 1);
				break;
			case 3:
				reg_value = max9286_clear_bit(reg_value,
					i * 2U);
				reg_value = max9286_clear_bit(reg_value,
					i * 2U + 1);
				break;
			default:
				break;
			}
			unlinked_count++;
		}
	}
	max9286_info("the output order register value is 0x%x\n", reg_value);

	ret = i2c_write(client, MAX9286_ADDR, reg_addr, 0x01U, &reg_value);
	if (ret < 0) {
		max9286_err("dev/0x90/val/ret/index /%x/%x/%d",
			reg_addr[0], reg_value, ret);
		return ret;
	}

	reg_addr[0] = 0x00U;
	reg_value = 0xEF & (*link_reg_val | 0xF0);
	max9286_info("the 0x00 reg_value is 0x%x\n", reg_value);
	ret = i2c_write(client, MAX9286_ADDR, reg_addr, 0x01U, &reg_value);
	if (ret < 0) {
		max9286_err("dev/0x90/val/ret/index /%x/%x/%d",
			reg_addr[0], reg_value, ret);
		return ret;
	}
	return 0;
}


static int max9286_camera_ch_addr_init(struct i2c_client *client,
	struct reg_val_ops *cmd, unsigned long len, int ch)
{
	int ret = 0;

	ret = max9286_write_array(client, cmd, len);
	if (ret < 0) {
		max9286_err("camera ch addr init failed\n");
		return -1;
	}
	return 0;
}

static int max96705_cross_bar_ch_addr_init(struct i2c_client *client,
	struct reg_val_ops *cmd, unsigned long len, int ch)
{
	int ret = 0;

	ret = max9286_write_array(client, cmd, len);
	if (ret < 0) {
		max9286_err("max96705 cross bar init failed\n");
		return -1;
	}
	return 0;
}

static int camera_module_init(struct i2c_client *client, u8 *link_reg_val)
{
	int ret = 0;
	u8 reg_addr[2] = {0x00U, 0x00U};
	u8 reg_value = 0x0U;
	u8 cam_count = 0U;
	u8 i = 0;

	ret  = max9286_camera_has_init(client, link_reg_val);
	if (ret == 0) {
		max9286_info("max9286 and camera have been initialized");
		return 0;
	}
	ret = max9286_write_array(client,
		MAX9286_camera_pre_init_cmd,
		ARRAY_SIZE(MAX9286_camera_pre_init_cmd));
	if (ret < 0)
		return ret;

	//set output lane number and frame sync
	cam_count = linked_ch_count(link_reg_val);
	if (cam_count == 1U) {
		//set output with 1 data dane when linked 1 camera
		reg_addr[0] = MAX9286_OUTLANE_REG_ADDR;
		reg_value = 0x33U;
		ret = i2c_write(client, MAX9286_ADDR,
			reg_addr, 0x01U, &reg_value);
		if (ret < 0) {
			max9286_err("dev/0x90/val/ret/index /%x/%x/%d",
				reg_addr[0], reg_value, ret);
			return ret;
		}

		//set frame sync
		reg_addr[0] = MAX9286_FRAME_SYNC_REG_ADDR;
		reg_value = 0xE2U;
		ret = i2c_write(client, MAX9286_ADDR,
			reg_addr, 0x01U, &reg_value);
		if (ret < 0) {
			max9286_err("dev/0x90/val/ret/index /%x/%x/%d",
				reg_addr[0], reg_value, ret);
			return ret;
		}
	} else {
		//set output with 4 lanes when linked 2 or 3 or 4 cameras
		reg_addr[0] = MAX9286_OUTLANE_REG_ADDR;
		reg_value = 0xF3U;
		ret = i2c_write(client, MAX9286_ADDR,
			reg_addr, 0x01U, &reg_value);
		if (ret < 0) {
			max9286_err("dev/0x90/val/ret/index /%x/%x/%d",
				reg_addr[0], reg_value, ret);
			return ret;
		}

		//set frame sync
		reg_addr[0] = MAX9286_FRAME_SYNC_REG_ADDR;
		reg_value = 0x02U;
		ret = i2c_write(client, MAX9286_ADDR,
			reg_addr, 0x01U, &reg_value);
		if (ret < 0) {
			max9286_err("dev/0x90/val/ret/index /%x/%x/%d",
				reg_addr[0], reg_value, ret);
			return ret;
		}
	}

	ret = max9286_write_array(client,
		MAX9286_camera_init_cmd,
		ARRAY_SIZE(MAX9286_camera_init_cmd));
	reg_addr[0] = MAX9286_LINK_ENABLE_REG_ADDR;
	reg_value = 0xEF & (*link_reg_val | 0xF0);
	max9286_info("the 0x00 reg_value is 0x%x\n", reg_value);
	ret = i2c_write(client, MAX9286_ADDR, reg_addr, 0x01U, &reg_value);
	if (ret < 0) {
		max9286_err("dev/0x90/val/ret/index /%x/%x/%d",
			reg_addr[0], reg_value, ret);
		return ret;
	}

	reg_addr[0] = MAX9286_F_R_CTL_REG_ADDR;
	reg_value = *link_reg_val | 0xF0;
	max9286_info("the 0x0A reg_value is 0x%x\n", reg_value);
	ret = i2c_write(client, MAX9286_ADDR, reg_addr, 0x01U, &reg_value);
	if (ret < 0) {
		max9286_err("dev/0x90/val/ret/index /%x/%x/%d",
			reg_addr[0], reg_value, ret);
		return ret;
	}

	reg_addr[0] = MAX9286_FSYNC_PERIODM_REG_ADDR;
	reg_value = 0x84;
	max9286_info("the 0x07 reg_value is 0x%x\n", reg_value);
	ret = i2c_write(client, MAX96705_INIT_ADDR, reg_addr, 0x01U, &reg_value);
	if (ret < 0) {
		max9286_err("dev/0x90/val/ret/index /%x/%x/%d",
			reg_addr[0], reg_value, ret);
		return ret;
	}

	for (i = 0; i < SENSOR_MAX_LINK_NUM; i++) {
		if (((*link_reg_val >> i) & 0x01U) == 0x01U) {
			max9286_info("channel %d linked, will init it", i);
			ret = max9286_camera_ch_addr_init(client, init_ch[i],
				ARRAY_SIZE(MAX9286_CAMERA_CH0_addr_init_cmd),
					i);
			if (ret < 0)
				return ret;
			max9286_info("channel %d linked, will init cross bar at max96705", i);
			ret = max96705_cross_bar_ch_addr_init(client, cross_bar_ch[i],
				ARRAY_SIZE(MAX96705_CROSS_BAR_CH0_addr_cmd), i);
			if (ret < 0)
				return ret;
				
			}
	}

	reg_addr[0] = MAX9286_F_R_CTL_REG_ADDR;
	reg_value = *link_reg_val | 0xF0;
	ret = i2c_write(client, MAX9286_ADDR, reg_addr, 0x01U, &reg_value);
	if (ret < 0) {
		max9286_err("dev/0x90/val/ret/index /%x/%x/%d",
			reg_addr[0], reg_value, ret);
		return ret;
	}

	ret = max9286_write_array(client,
		MAX9286_camera_en_cmd,
		ARRAY_SIZE(MAX9286_camera_en_cmd));
	if (ret < 0)
		return ret;

	return 0;
}

static int max9286_camera_init(struct i2c_client *client)
{
	int ret = 0;
	u8 max9286_id_val = 0;
	u8 lock_reg_val = 0;
	u8 link_reg_val = 0;
	int index = 0;
	int read_cnt = 0;
	int retry_num = 3;
	u32 link_cnt = 0;
	struct max9286 *priv = NULL;
	u64 delay = 10000;

    debug("%s:%d\n",__func__,__LINE__);
	/*MAX9286 ID confirm*/
	for (read_cnt = 0; read_cnt < retry_num; ++read_cnt) {
		ret = read_max9286_id(client, &max9286_id_val);
		if (ret == 2)
			break;

		if (read_cnt == 2) {
			max9286_err("read max9286 ID time out");
			return -EIO;
		}

		udelay(delay);
	}

	/*check video link*/
	for (read_cnt = 0; read_cnt < retry_num; ++read_cnt) {
		ret = max9286_get_link(client, &link_reg_val);
		if ((ret == 2) && (link_reg_val > 0u)) {
			max9286_info("the link_reg_val is %d\n", link_reg_val);
			break;
		}

		if (read_cnt == 2) {
			max9286_err("no camera linked link_reg_val=%x",
				link_reg_val);
			return -ENODEV;
		}

		udelay(delay);
	}

	//show max9286 link information
	for (index = 0; index < SENSOR_MAX_LINK_NUM; ++index) {
		if (((link_reg_val >> index) & 0x01U) == 0x01U) {
			++link_cnt;
			max9286_info("channel %d linked", index);
		} else {
			max9286_info("channel %d not linked", index);
		}
	}
	max9286_info("%u camera linked", link_cnt);

	//set max9286 output order
	ret = set_output_order(client, &link_reg_val);
	if (ret != 0) {
		max9286_info("set output order failed\n");
		return ret;
	}

	/* check camera links are locked or not */
	for (read_cnt = 0; read_cnt < 3; ++read_cnt) {
		ret = max9286_get_lock_status(client, &lock_reg_val);
		if (ret == 2)
			break;

		if (read_cnt == 2) {
			max9286_err("read lock status time out");
			return -EIO;
		}

		udelay(delay);
	}

	priv = to_max9286(client);
	priv->link = link_cnt;

	// init max9286 and max96705
	ret = camera_module_init(client, &link_reg_val);
	if (ret != 0) {
		max9286_info("camera_module_init failed\n");
		return ret;
	}
	return 0;
}

static int max9286_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_4_LANE | V4L2_MBUS_CSI2_CHANNELS |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	return 0;
}

static int max9286_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int max9286_s_mbus_config(struct v4l2_subdev *sd,
			     const struct v4l2_mbus_config *cfg)
{
	return 0;
}

static int max9286_s_power(struct v4l2_subdev *sd, int on)
{
    //struct i2c_client *client = v4l2_get_subdevdata(sd);
    //struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
    //struct max9286 *priv = to_max9286(client);
    int ret = 0;
    debug("on = %d\n",on);
#if 0
    if (on == 0){
        ret = pinctrl_select_state(max9286_pctrl.pinctrl,max9286_pctrl.gpio6_disenable_state);
        if (ret < 0) {
            max9286_err("gpio6 have been pulled down!!!!\n");
            //return ret;
        }
        ret = pinctrl_select_state(max9286_pctrl.pinctrl,max9286_pctrl.gpio_state_suspend);
        if (ret < 0) {
            max9286_err("%s, %d\n", __func__, __LINE__);
            return ret;
        }
        if (max9286_pctrl.vdd1v8 != NULL){
            ret = regulator_disable(max9286_pctrl.vdd1v8);
            if (ret != 0) {
                max9286_err("Failed to disable vdd1v8!\n");
                return ret;
            }
        }
        //return soc_camera_power_off(&client->dev, ssdd, priv->clk);
    }else{
        ret = pinctrl_select_state(max9286_pctrl.pinctrl,max9286_pctrl.gpio6_enable_state);
        if (ret < 0) {
            max9286_err("gpio6 have been pulled up!!!!\n");
            //return ret;
        }
        ret = pinctrl_select_state(max9286_pctrl.pinctrl,max9286_pctrl.gpio_state_active);
        if (ret < 0) {
            max9286_err("%s, %d\n", __func__, __LINE__);
            return ret;
        }
        if (pmu1v8_power_init(&client->dev,client->dev.of_node)<0){
            max9286_err("%s, %d\n", __func__, __LINE__);
            return -EINVAL;
        }
    }
    mdelay(500);
    //return soc_camera_power_on(&client->dev, ssdd, priv->clk);
#endif
    return ret;
}

static int max9286_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if ((code->pad != 0u) ||
	    (unsigned long)code->index >= ARRAY_SIZE(max9286_colour_fmts))
		return -EINVAL;

	code->code = max9286_colour_fmts[code->index].code;
	return 0;
}

static const struct max9286_datafmt *max9286_find_datafmt(u32 code)
{
	__u64 i;

	for (i = 0; i < ARRAY_SIZE(max9286_colour_fmts); i++)
		if (max9286_colour_fmts[i].code == code)
			return max9286_colour_fmts + i;

	return NULL;
}

static int max9286_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct max9286_datafmt *fmt = max9286_find_datafmt(mf->code);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct max9286 *priv = to_max9286(client);
	int ret = 0;
	unsigned long cmd_len = 0;

    if (format->pad != 0u)
        return -EINVAL;
    	debug("------->>>>in\n");
	if (fmt == NULL) {
		/* MIPI CSI could have changed the format, double-check */
		if (format->which == (__u32)V4L2_SUBDEV_FORMAT_ACTIVE)
			return -EINVAL;
		mf->code	= max9286_colour_fmts[0].code;
		mf->colorspace	= (__u32)max9286_colour_fmts[0].colorspace;
	}

	if ((mf->width != (__u32)KSS_WIDTH) ||
		(mf->height != (__u32)KSS_HEIGHT * priv->link)) {
		max9286_err("width/height %d/%d not support. Set %d/%d as default",
			mf->width, mf->height,
			KSS_WIDTH, KSS_HEIGHT * priv->link);
	}

	mf->width	= KSS_WIDTH;
	mf->height	= KSS_HEIGHT * priv->link;
	mf->field	= (__u32)V4L2_FIELD_NONE;

	if (format->which == (__u32)V4L2_SUBDEV_FORMAT_ACTIVE)
		priv->fmt = max9286_find_datafmt(mf->code);
	else
		cfg->try_fmt = *mf;

	if (is_testpattern == 1) {
		/*enable CAMERA test pattern only once*/
		if (is_exec_testpattern == 1)
			return 0;
		if (priv->link == 1U) {
			cmd_len = ARRAY_SIZE(MAX9286_camera_dis_tp2_cmd);
			ret = max9286_write_array(client,
				MAX9286_camera_dis_tp2_cmd,
			cmd_len);
		} else if (priv->link == 4U) {
			cmd_len = ARRAY_SIZE(MAX9286_camera_dis_tp0_cmd);
			ret = max9286_write_array(client,
				MAX9286_camera_dis_tp0_cmd,
				cmd_len);
			cmd_len = ARRAY_SIZE(MAX9286_camera_dis_tp1_cmd);
			ret = max9286_write_array(client,
				MAX9286_camera_dis_tp1_cmd,
				cmd_len);
		} else {
			max9286_err("error link cnt %d", priv->link);
			ret = -EINVAL;
		}
		if (is_exec_testpattern == 0)
			is_exec_testpattern = 1;
	} else if (is_testpattern == 2) {
		/*disable CAMERA test pattern only once*/
		if (is_exec_testpattern == 0)
			return 0;
		if (priv->link == 1U) {
			cmd_len = ARRAY_SIZE(MAX9286_camera_dis_tp0_cmd);
			ret = max9286_write_array(client,
				MAX9286_camera_dis_tp0_cmd,
				cmd_len);
		} else if (priv->link == 4U) {
			cmd_len = ARRAY_SIZE(MAX9286_camera_dis_tp2_cmd);
			ret = max9286_write_array(client,
				MAX9286_camera_dis_tp2_cmd,
				cmd_len);
			cmd_len = ARRAY_SIZE(MAX9286_camera_dis_tp0_cmd);
			ret = max9286_write_array(client,
				MAX9286_camera_dis_tp0_cmd,
				cmd_len);
			} else {
			max9286_err("error link cnt %d", priv->link);
			ret = -EINVAL;
		}
		if (is_exec_testpattern == 1)
			is_exec_testpattern = 0;
	}
	return ret;
}

static int max9286_g_register(struct v4l2_subdev *sd,
		struct v4l2_dbg_register *reg)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;
    u8 val = 0;
    u8 max9286_reg = (u8)(reg->reg);
    debug("-----> in \n");
    if (reg == NULL){
        max9286_err("reg is NULL!!!\n");
        return -EINVAL;
    }

    ret =  max9286_camera_init(client);
    if (ret < 0){
        max9286_err("%s --->>> %d\n",__func__,__LINE__);
        return ret;
    }

	max9286_info("read max9286 reg %llx", reg->reg);
	if (reg->match.type == (u8)0) {
		ret = i2c_read(client, MAX9286_ADDR, &max9286_reg, 1, &val);
		if (ret != 2) {
			max9286_err("ret=%d", ret);
			return -EIO;
		}
		reg->val = val;
	}

	return ret;
}

static const struct v4l2_subdev_video_ops max9286_subdev_video_ops = {
	.g_mbus_config	= max9286_g_mbus_config,
	.s_stream	= max9286_s_stream,
	.s_mbus_config = max9286_s_mbus_config,
};

static const struct v4l2_subdev_core_ops max9286_subdev_core_ops = {
	.s_power	= max9286_s_power,
	.g_register	= max9286_g_register,
};

static const struct v4l2_subdev_pad_ops max9286_subdev_pad_ops = {
	.enum_mbus_code = max9286_enum_mbus_code,
	.set_fmt	= max9286_set_fmt,
};

static const struct v4l2_subdev_ops max9286_subdev_ops = {
	.core	= &max9286_subdev_core_ops,
	.video	= &max9286_subdev_video_ops,
	.pad = &max9286_subdev_pad_ops,
};

int max9286_pinctrl_init(struct device *dev)
{
    int ret = 0;
    debug("%s:%d input\n",__func__,__LINE__);
    max9286_pctrl.pinctrl = devm_pinctrl_get(dev);
    if(IS_ERR(max9286_pctrl.pinctrl)) {
        debug("%s:%d Getting pinctrl handle failed\n",
                __func__,__LINE__);
        return-EINVAL;
    }
    max9286_pctrl.gpio6_enable_state = pinctrl_lookup_state(max9286_pctrl.pinctrl,"gpio6_enable");
    if(IS_ERR(max9286_pctrl.gpio6_enable_state)) {
        debug("%s:%d failed to get the active state pinctrl handle!!\n",
                __func__,__LINE__);
        return-EINVAL;
    }
    max9286_pctrl.gpio6_disenable_state = pinctrl_lookup_state(max9286_pctrl.pinctrl,"gpio6_disenable");
    if(IS_ERR(max9286_pctrl.gpio6_disenable_state)) {
        debug("%s:%d failed to get the active state pinctrl handle!!\n",
                __func__,__LINE__);
        return-EINVAL;
    }
    max9286_pctrl.gpio_state_active = pinctrl_lookup_state(max9286_pctrl.pinctrl,"max9286pins_active");
    if(IS_ERR(max9286_pctrl.gpio_state_active)) {
        debug("%s:%d failed to get the active state pinctrl handle!!\n",
                __func__,__LINE__);
        return-EINVAL;
    }
    max9286_pctrl.gpio_state_suspend = pinctrl_lookup_state(max9286_pctrl.pinctrl,"max9286pins_suspend");
    if(IS_ERR(max9286_pctrl.gpio_state_suspend)) {
        debug("%s:%d failed to get the suspend state pinctrl handle!!\n",
                __func__,__LINE__);
        return-EINVAL;
    }
    /*detect gpio6 state*/
    ret = pinctrl_select_state(max9286_pctrl.pinctrl,max9286_pctrl.gpio6_enable_state);
    if (ret < 0) {
        debug("gpio6 have been pulled up!!!!\n");
        //return ret;
    }
    debug("%s:%d out\n",__func__,__LINE__);
    return ret;
}

static int pmu1v8_power_init(struct device *dev,struct device_node *node)
{
    int len;
    int ret;
    const char *vdd1v8;

    vdd1v8 = of_get_property(node, "pmu1v8-supply", &len);
    if (vdd1v8 != NULL) {
        max9286_pctrl.vdd1v8 = devm_regulator_get(dev, vdd1v8);
        if (IS_ERR(max9286_pctrl.vdd1v8)) {
            ret = PTR_ERR(max9286_pctrl.vdd1v8);
            goto err_vdd1v8;
        }
        ret = regulator_enable(max9286_pctrl.vdd1v8);
        if (ret < 0) {
            pr_err("Failed to enable vddoa\n");
            goto err_vdd1v8;
        }
    }
    usleep_range(3000,3000);

    return 0;

err_vdd1v8:
    regulator_disable(max9286_pctrl.vdd1v8);

    return ret;
}

#define sensor_register_debug
#ifdef sensor_register_debug
static struct i2c_client *sensor_client = NULL;
static u16 data_t;
static u8 data;
static u8 addr[2];
static ssize_t addr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    debug("luozh: addr=0x%02x%02x\n", addr[0],addr[1]);
    return sprintf(buf, "0x%02x%02x\n", addr[0],addr[1]);
}
static ssize_t addr_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    addr[0] = (0xFF00 & (unsigned short)simple_strtoul(buf, NULL, 16))>>8;
    addr[1] = (0x00FF & (unsigned short)simple_strtoul(buf, NULL, 16));

    debug("luozh: addr=0x%02x%02x\n", addr[0],addr[1]);
    return count;
}
//for max20088
#define MAX20088_ADDR 0x50 //0x28
static ssize_t register_show_max20088(struct device *dev, struct device_attribute *attr, char *buf)
{
    addr[0] = addr[1];
    i2c_read(sensor_client,MAX20088_ADDR, addr,1,&data);
    debug("luozh: addr=0x%02x, data=0x%02x\n", addr[0], data);
    return sprintf(buf, "0x%02x\n", data);
}

static ssize_t register_store_max20088(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    addr[0] = addr[1];
    data = (unsigned short)simple_strtoul(buf, NULL, 16);
    i2c_write(sensor_client,MAX20088_ADDR,addr, 1, &data);
    debug("luozh: addr=0x%02x, data=0x%02x\n", addr[0], data);
    return count;
}

#define TEMP_ADDR 0x90 //0x48
//TMP102 register addresses
#define TMP102_REG_ADDR_TEMPERATURE             0x00
#define TMP102_REG_ADDR_CONFIG                  0x01
u8 bConfig_Data_Write[2] = {0x70, 0xA0};
static ssize_t register_show_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
    addr[0] = addr[1];
    i2c_read_t(sensor_client,TEMP_ADDR, addr,1,&data_t);
    debug("luozh: addr=0x%02x, data=0x%04x\n", addr[0], data_t);
    return sprintf(buf, "0x%04x\n", data_t);
}
static ssize_t register_store_temp(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    addr[0] = addr[1];
    data_t = (unsigned short)simple_strtoul(buf, NULL, 16);
    i2c_write_t(sensor_client,TEMP_ADDR,addr,&data_t);
    debug("luozh: addr=0x%02x, data=0x%04x\n", addr[0], data_t);
    return count;
}
static DEVICE_ATTR(register_addr, 0644, addr_show, addr_store);
static DEVICE_ATTR(android_register_max20088, 0644, register_show_max20088, register_store_max20088);
static DEVICE_ATTR(android_register_temp102, 0644, register_show_temp, register_store_temp);
#endif
static int max9286_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct max9286 *priv = NULL;
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
    struct v4l2_subdev *subdev = NULL;
    int ret = 0;
    debug("%s IN--->>> %d\n",__func__,__LINE__);
	if (client->dev.of_node != NULL) {
		ssdd = devm_kzalloc(&client->dev, sizeof(*ssdd), GFP_KERNEL);
		if (ssdd == NULL)
			return -ENOMEM;
		client->dev.platform_data = ssdd;
	}

	if (ssdd == NULL) {
		max9286_err("MAX9286: missing platform data");
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct max9286), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

    if (max9286_pinctrl_init(&client->dev)<0){
        max9286_err("%s, %d\n", __func__, __LINE__);
        return -EINVAL;
    }
    if (pmu1v8_power_init(&client->dev,client->dev.of_node)<0){
        max9286_err("%s, %d\n", __func__, __LINE__);
        return -EINVAL;
    }
    ret = pinctrl_select_state(max9286_pctrl.pinctrl,max9286_pctrl.gpio_state_active);
    if (ret < 0) {
        max9286_err("%s, %d\n", __func__, __LINE__);
        return ret;
    }

    mdelay(500);
    v4l2_i2c_subdev_init(&priv->subdev, client, &max9286_subdev_ops);

	priv->fmt		= &max9286_colour_fmts[0];
#ifdef sensor_register_debug
    sensor_client = client;
    ret = device_create_file(&client->dev, &dev_attr_register_addr);
    if (ret) {
        debug("luozh: register_addr probe error....\n");
    }
    //for max20088
    ret = device_create_file(&client->dev, &dev_attr_android_register_max20088);
    if (ret) {
        debug("luozh: register_addr probe error....\n");
    }
    ret = device_create_file(&client->dev, &dev_attr_android_register_temp102);
    if (ret) {
        debug("luozh: register_addr probe error....\n");
    }
#endif
	subdev = i2c_get_clientdata(client);

    ret =  max9286_camera_init(client);
    if (ret < 0){
        max9286_err("max9286 camera init failed!\n");
        if(max9286_s_power(subdev,0)<0){
            max9286_err("power off failed!\n");
        }
        return ret;
    }
	priv->subdev.dev = &client->dev;

    debug("%s out--->>> %d\n",__func__,__LINE__);
    return v4l2_async_register_subdev(&priv->subdev);
}

static int max9286_remove(struct i2c_client *client)
{
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);

	if (ssdd->free_bus != NULL)
		ssdd->free_bus(ssdd);

	return 0;
}

static const struct i2c_device_id max9286_id[] = {
	{ "max9286", 0},
	{ }
};
static const struct of_device_id max9286_camera_of_match[] = {
	{ .compatible = "maxim,max9286", },
	{},
};

static struct i2c_driver max9286_i2c_driver = {
	.driver = {
		.name = "max9286",
		.of_match_table = max9286_camera_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= max9286_probe,
	.remove		= max9286_remove,
	.id_table	= max9286_id,
};

module_param(is_testpattern, int, 0644);
MODULE_PARM_DESC(is_testpattern, "Whether the MAX9286 get test pattern data");
module_i2c_driver(max9286_i2c_driver);

MODULE_DESCRIPTION("MAXIM Camera driver");
MODULE_AUTHOR("Fangyuan Liu <fangyuan.liu@nio.com>");
MODULE_LICENSE("GPL v2");
