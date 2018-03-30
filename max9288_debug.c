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
#define MAX9288_ADDR 0xD0
#define MAX9288_ID 0x2A
#define MAX_REG_LEN 2

#define MAX9271_INIT_ADDR 0x80

#define MAX9271_ALL_ADDR 0x80

#define MAX9271_CH0_ADDR 0xA2
#define MAX9271_CH1_ADDR 0xA4
#define MAX9271_CH2_ADDR 0x86
#define MAX9271_CH3_ADDR 0x88

#define SENSOR_INIT_ADDR 0x60
#define OV490_CH0_ADDR 0x62
#define OV490_INIT_ADDR 0x48
#define OV490_CH0_MAP_ADDR 0x60
#define OV490_CH1_MAP_ADDR 0x62
#define OV490_CH2_MAP_ADDR 0x64
#define OV490_CH3_MAP_ADDR 0x66
#define MAX9288_1CH_LANE 1
//#define CAB888_TEST_PATTERN


#define OV490_ID 0xB888
#define MAX9288_LINK_CONFIG_REG 0x00
#define MAX9288_ID_REG   0x1E
#define MAX9288_LOCK_REG 0x04
#define MAX9288_LINK_REG 0x49

/* minimum extra blanking */
#define BLANKING_EXTRA_WIDTH		500
#define BLANKING_EXTRA_HEIGHT		20

/*
 * the sensor's autoexposure is buggy when setting total_height low.
 * It tries to expose longer than 1 frame period without taking care of it
 * and this leads to weird output. So we set 1000 lines as minimum.
 */
#define BLANKING_MIN_HEIGHT	1000
#define SENSOR_ID               (0x86)


#define max9288_info(fmt, args...)                \
		(void)pr_info("[max9288][info] %s %d: %s " fmt "\n",\
			__func__, __LINE__, dev_name(&client->dev), ##args)
#define max9288_err(fmt, args...)                \
		(void)pr_info("[max9288][error] %s %d: %s " fmt "\n",\
			__func__, __LINE__, dev_name(&client->dev), ##args)

/*add debug info*/
//#define __DEBUG__
#ifdef __DEBUG__
#define debug(fmt,...) printk("[debug_info] %s:%d" fmt "\n",__func__,__LINE__,##__VA_ARGS__)
#else
#define debug(fmt,...)
#endif

struct max9288_pinctrl_info{
    struct pinctrl *pinctrl;
    struct pinctrl_state *gpio_state_active;//defaut
    struct pinctrl_state *gpio_state_suspend;
    struct pinctrl_state *gpio6_enable_state;
    struct pinctrl_state *gpio6_disenable_state;
    struct regulator  *vdd1v8;
}max9288_pctrl;

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
static int read_max9288_id(struct i2c_client *client, u8 *id_val);
static int max9288_write_array(struct i2c_client *client,
		struct reg_val_ops *cmd, unsigned long len);
static int pmu1v8_power_init(struct device *dev,struct device_node *node);

/* Supported resolutions */
enum max9288_width {
	W_TESTWIDTH	= 720,
	W_WIDTH		= 736,
};

enum max9288_height {
	H_1CHANNEL	= 480,
	H_4CHANNEL	= 1920,
};

struct max9288_win_size {
	enum max9288_width width;
	enum max9288_height height;
};

static const struct max9288_win_size max9288_supported_win_sizes[] = {
	{ W_TESTWIDTH,	H_1CHANNEL },
	{ W_TESTWIDTH,	H_4CHANNEL },
	{ W_WIDTH,	H_1CHANNEL },
	{ W_WIDTH,	H_4CHANNEL },
};

static int is_testpattern;

struct sensor_addr {
	u16 ser_init_addr;
	u16 ser_last_addr;
	u16 isp_init_addr;
	u16 isp_map_addr;
};

#define CAB888_WIDTH 1280U
#define CAB888_HEIGHT 800U

static struct reg_val_ops MAX9288_CAB888_4v4_init_cmd[] = {
	{MAX9288_ADDR,      {0x0D, 0x00}, 0x03,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x3F, 0x00}, 0x4F,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x3B, 0x00}, 0x1E,               0x01, i2c_write},

	{MAX9288_ADDR,      {0x28, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x28, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x29, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x29, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x2A, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x2A, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x2B, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x2B, 0x00}, 0x00,               0x01, i2c_read},

	{MAX9271_INIT_ADDR, {0x08, 0x00}, 0x01,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x3B, 0x00}, 0x19,               0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x04, 0x00}, 0x43,               0x01, i2c_write},

	{MAX9271_INIT_ADDR, {0x06, 0x00}, 0xA0,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x15, 0x00}, 0x13,               0x01, i2c_write},

	{MAX9288_ADDR,      {0x12, 0x00}, 0xF3,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x01, 0x00}, 0x02,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x02, 0x00}, 0x20,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x63, 0x00}, 0x00,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x64, 0x00}, 0x00,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x05, 0x00}, 0x19,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x00, 0x00}, 0xEF,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x0A, 0x00}, 0xFF,               0x01, i2c_write},

	{MAX9271_INIT_ADDR, {0x07, 0x00}, 0x84,               0x01, i2c_write},

	{MAX9288_ADDR,	    {0x0A, 0x00}, 0xF1,	              0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, 0x00,               0x01, i2c_read},

	{MAX9271_INIT_ADDR, {0x00, 0x00}, MAX9271_CH0_ADDR,   0x01, i2c_write},
	{MAX9271_CH0_ADDR,  {0x07, 0x00}, 0x84,	              0x01, i2c_write},
	{MAX9271_CH0_ADDR,  {0x09, 0x00}, OV490_CH0_MAP_ADDR, 0x01, i2c_write},
	{MAX9271_CH0_ADDR,  {0x0A, 0x00}, OV490_INIT_ADDR,    0x01, i2c_write},
	{MAX9271_CH0_ADDR,  {0x0B, 0x00}, 0x8A,	              0x01, i2c_write},
	{MAX9271_CH0_ADDR,  {0x0C, 0x00}, MAX9271_CH0_ADDR,   0x01, i2c_write},

	{MAX9288_ADDR,	    {0x0A, 0x00}, 0xF2,	              0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, MAX9271_CH1_ADDR,   0x01, i2c_write},
	{MAX9271_CH1_ADDR,  {0x07, 0x00}, 0x84,	              0x01, i2c_write},
	{MAX9271_CH1_ADDR,  {0x09, 0x00}, OV490_CH1_MAP_ADDR, 0x01, i2c_write},
	{MAX9271_CH1_ADDR,  {0x0A, 0x00}, OV490_INIT_ADDR,    0x01, i2c_write},
	{MAX9271_CH1_ADDR,  {0x0B, 0x00}, 0x8A,		      0x01, i2c_write},
	{MAX9271_CH1_ADDR,  {0x0C, 0x00}, MAX9271_CH1_ADDR,   0x01, i2c_write},

	{MAX9288_ADDR,	    {0x0A, 0x00}, 0xF4,	              0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, MAX9271_CH2_ADDR,   0x01, i2c_write},
	{MAX9271_CH2_ADDR,  {0x07, 0x00}, 0x84,	              0x01, i2c_write},
	{MAX9271_CH2_ADDR,  {0x09, 0x00}, OV490_CH2_MAP_ADDR, 0x01, i2c_write},
	{MAX9271_CH2_ADDR,  {0x0A, 0x00}, OV490_INIT_ADDR,    0x01, i2c_write},
	{MAX9271_CH2_ADDR,  {0x0B, 0x00}, 0x8A,	              0x01, i2c_write},
	{MAX9271_CH2_ADDR,  {0x0C, 0x00}, MAX9271_CH2_ADDR,   0x01, i2c_write},

	{MAX9288_ADDR,	    {0x0A, 0x00}, 0xF8,	              0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, MAX9271_CH3_ADDR,   0x01, i2c_write},
	{MAX9271_CH3_ADDR,  {0x07, 0x00}, 0x84,	              0x01, i2c_write},
	{MAX9271_CH3_ADDR,  {0x09, 0x00}, OV490_CH3_MAP_ADDR, 0x01, i2c_write},
	{MAX9271_CH3_ADDR,  {0x0A, 0x00}, OV490_INIT_ADDR,    0x01, i2c_write},
	{MAX9271_CH3_ADDR,  {0x0B, 0x00}, 0x8A,		      0x01, i2c_write},
	{MAX9271_CH3_ADDR,  {0x0C, 0x00}, MAX9271_CH3_ADDR,   0x01, i2c_write},

	{MAX9288_ADDR,	    {0x0A, 0x00}, 0xFF,               0x01, i2c_write},

	{MAX9271_ALL_ADDR,  {0x04, 0x00}, 0x83,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x15, 0x00}, 0x9B,               0x01, i2c_write},

	{OV490_INIT_ADDR,   {0xFF, 0xFD}, 0x80,               0x03, i2c_write},
	{OV490_INIT_ADDR,   {0xFF, 0xFE}, 0x19,               0x03, i2c_write},
	{OV490_INIT_ADDR,   {0x50, 0x00}, 0x00,               0x03, i2c_write},
	{OV490_INIT_ADDR,   {0xFF, 0xFE}, 0x80,               0x03, i2c_write},
	{OV490_INIT_ADDR,   {0x00, 0xC0}, 0xD7,               0x03, i2c_write},

	{OV490_INIT_ADDR,   {0xFF, 0xFD}, 0x80,               0x03, i2c_write},
	{OV490_INIT_ADDR,   {0xFF, 0xFE}, 0x19,               0x03, i2c_write},
	{OV490_INIT_ADDR,   {0x50, 0x00}, 0x01,               0x03, i2c_write},
	{OV490_INIT_ADDR,   {0xFF, 0xFE}, 0x80,               0x03, i2c_write},
	{OV490_INIT_ADDR,   {0x00, 0xC0}, 0xD6,               0x03, i2c_write},
};

static struct reg_val_ops MAX9288_CAB888_4V4_r_cmd[] = {
	{MAX9271_CH0_ADDR, {0x00, 0x00}, MAX9271_CH0_ADDR,    0x01, i2c_read},
	{MAX9271_CH1_ADDR, {0x00, 0x00}, MAX9271_CH1_ADDR,    0x01, i2c_read},
	{MAX9271_CH2_ADDR, {0x00, 0x00}, MAX9271_CH2_ADDR,    0x01, i2c_read},
	{MAX9271_CH3_ADDR, {0x00, 0x00}, MAX9271_CH3_ADDR,    0x01, i2c_read},
	{MAX9288_ADDR,     {0x15, 0x00}, 0x9B,                0x01, i2c_read},
};

static struct reg_val_ops MAX9288_CAB888_4V4_pre_init_cmd[] = {
	{MAX9288_ADDR,      {0x3F, 0x00}, 0x4F,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x00, 0x00}, 0x02,               0x01, i2c_delay},
	{MAX9288_ADDR,      {0x3B, 0x00}, 0x1E,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x00, 0x00}, 0x02,               0x01, i2c_delay},
	{MAX9271_INIT_ADDR, {0x04, 0x00}, 0x43,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x00, 0x00}, 0x05,               0x01, i2c_delay},
	{MAX9288_ADDR,      {0x28, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x28, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x29, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x29, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x2A, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x2A, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x2B, 0x00}, 0x00,               0x01, i2c_read},
	{MAX9288_ADDR,      {0x2B, 0x00}, 0x00,               0x01, i2c_read},

	{MAX9271_INIT_ADDR, {0x08, 0x00}, 0x01,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x00, 0x00}, 0x02,               0x01, i2c_delay},
	{MAX9288_ADDR,      {0x3B, 0x00}, 0x19,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x00, 0x00}, 0x02,               0x01, i2c_delay},
	{MAX9288_ADDR,	    {0x34, 0x00}, 0x00,               0x01, i2c_read},

	{MAX9288_ADDR,      {0x15, 0x00}, 0x13,               0x01, i2c_write},

	{MAX9288_ADDR,      {0x12, 0x00}, 0xF3,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x01, 0x00}, 0x02,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x02, 0x00}, 0x20,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x63, 0x00}, 0x00,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x64, 0x00}, 0x00,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x05, 0x00}, 0x19,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x00, 0x00}, 0xEF,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x0A, 0x00}, 0xFF,               0x01, i2c_write},

	{MAX9271_INIT_ADDR, {0x07, 0x00}, 0x84,               0x01, i2c_write},
};

static struct reg_val_ops MAX9288_CAB888_CH0_addr_init_cmd[] = {
	{MAX9288_ADDR,	    {0x0A, 0x00}, 0xF1,	              0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, 0x80,               0x01, i2c_read},
	{MAX9271_CH0_ADDR,  {0x00, 0x00}, MAX9271_CH0_ADDR,   0x01, i2c_read},
	{MAX9271_CH0_ADDR, {0x00, 0x00},  MAX9271_INIT_ADDR,  0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, MAX9271_CH0_ADDR,   0x01, i2c_read},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, MAX9271_CH0_ADDR,   0x01, i2c_write},
	{MAX9271_CH0_ADDR,  {0x07, 0x00}, 0x84,	              0x01, i2c_write},
	{MAX9271_CH0_ADDR,  {0x09, 0x00}, OV490_CH0_MAP_ADDR, 0x01, i2c_write},
	{MAX9271_CH0_ADDR,  {0x0A, 0x00}, OV490_INIT_ADDR,    0x01, i2c_write},
	{MAX9271_CH0_ADDR,  {0x0B, 0x00}, MAX9271_ALL_ADDR,   0x01, i2c_write},
	{MAX9271_CH0_ADDR,  {0x0C, 0x00}, MAX9271_CH0_ADDR,   0x01, i2c_write},
};

static struct reg_val_ops MAX9288_CAB888_CH1_addr_init_cmd[] = {
	{MAX9288_ADDR,	    {0x0A, 0x00}, 0xF2,               0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, 0x80,               0x01, i2c_read},
	{MAX9271_CH1_ADDR,  {0x00, 0x00}, MAX9271_CH1_ADDR,   0x01, i2c_read},
	{MAX9271_CH1_ADDR,  {0x00, 0x00}, MAX9271_INIT_ADDR,  0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, MAX9271_CH1_ADDR,   0x01, i2c_read},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, MAX9271_CH1_ADDR,   0x01, i2c_write},
	{MAX9271_CH1_ADDR,  {0x07, 0x00}, 0x84,               0x01, i2c_write},
	{MAX9271_CH1_ADDR,  {0x09, 0x00}, OV490_CH1_MAP_ADDR, 0x01, i2c_write},
	{MAX9271_CH1_ADDR,  {0x0A, 0x00}, OV490_INIT_ADDR,    0x01, i2c_write},
	{MAX9271_CH1_ADDR,  {0x0B, 0x00}, MAX9271_ALL_ADDR,   0x01, i2c_write},
	{MAX9271_CH1_ADDR,  {0x0C, 0x00}, MAX9271_CH1_ADDR,   0x01, i2c_write},
};

static struct reg_val_ops MAX9288_CAB888_CH2_addr_init_cmd[] = {
	{MAX9288_ADDR,	    {0x0A, 0x00}, 0xF4,	              0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, 0x80,               0x01, i2c_read},
	{MAX9271_CH2_ADDR,  {0x00, 0x00}, MAX9271_CH2_ADDR,   0x01, i2c_read},
	{MAX9271_CH2_ADDR,  {0x00, 0x00}, MAX9271_INIT_ADDR,  0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, MAX9271_CH2_ADDR,   0x01, i2c_read},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, MAX9271_CH2_ADDR,   0x01, i2c_write},
	{MAX9271_CH2_ADDR,  {0x07, 0x00}, 0x84,	              0x01, i2c_write},
	{MAX9271_CH2_ADDR,  {0x09, 0x00}, OV490_CH2_MAP_ADDR, 0x01, i2c_write},
	{MAX9271_CH2_ADDR,  {0x0A, 0x00}, OV490_INIT_ADDR,    0x01, i2c_write},
	{MAX9271_CH2_ADDR,  {0x0B, 0x00}, MAX9271_ALL_ADDR,   0x01, i2c_write},
	{MAX9271_CH2_ADDR,  {0x0C, 0x00}, MAX9271_CH2_ADDR,   0x01, i2c_write},
};

static struct reg_val_ops MAX9288_CAB888_CH3_addr_init_cmd[] = {
	{MAX9288_ADDR,	    {0x0A, 0x00}, 0xF8,	              0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, 0x80,               0x01, i2c_read},
	{MAX9271_CH3_ADDR,  {0x00, 0x00}, MAX9271_CH3_ADDR,   0x01, i2c_read},
	{MAX9271_CH3_ADDR,  {0x00, 0x00}, MAX9271_INIT_ADDR,  0x01, i2c_write},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, MAX9271_CH3_ADDR,   0x01, i2c_read},
	{MAX9271_INIT_ADDR, {0x00, 0x00}, MAX9271_CH3_ADDR,   0x01, i2c_write},
	{MAX9271_CH3_ADDR,  {0x07, 0x00}, 0x84,	              0x01, i2c_write},
	{MAX9271_CH3_ADDR,  {0x09, 0x00}, OV490_CH3_MAP_ADDR, 0x01, i2c_write},
	{MAX9271_CH3_ADDR,  {0x0A, 0x00}, OV490_INIT_ADDR,    0x01, i2c_write},
	{MAX9271_CH3_ADDR,  {0x0B, 0x00}, MAX9271_ALL_ADDR,   0x01, i2c_write},
	{MAX9271_CH3_ADDR,  {0x0C, 0x00}, MAX9271_CH3_ADDR,   0x01, i2c_write},
};

static struct reg_val_ops MAX9288_CAB888_4V4_en_cmd[] = {
	{MAX9288_ADDR,	    {0x0A, 0x00}, 0xFF,               0x01, i2c_write},
	{MAX9271_ALL_ADDR,  {0x04, 0x00}, 0x83,               0x01, i2c_write},
	{MAX9288_ADDR,      {0x15, 0x00}, 0x9B,               0x01, i2c_write},
};

static struct reg_val_ops MAX9288_CAB888_1v1_init_cmd[] = {
        {MAX9288_ADDR,      {0x65, 0x00}, 0x07,             0x01, i2c_write}, // enable CSI 0
        {MAX9288_ADDR,      {0x00, 0x00}, 0x02,             0x01, i2c_delay}, // delay 2ms

        {MAX9288_ADDR,      {0x00, 0x00}, 0x80,             0x01, i2c_write}, // modify ser addr
        {MAX9288_ADDR,      {0x00, 0x00}, 0x02,             0x01, i2c_delay}, // delay 2ms

        {MAX9271_INIT_ADDR, {0x04, 0x00}, 0x43,             0x01, i2c_write}, // off serializer
        {MAX9288_ADDR,      {0x00, 0x00}, 0x05,             0x01, i2c_delay}, // delay 5ms

        {MAX9271_INIT_ADDR, {0x01, 0x00}, 0xD0,             0x01, i2c_write}, // modify des addr
        {MAX9288_ADDR,      {0x00, 0x00}, 0x02,             0x01, i2c_delay}, // delay 2ms

        {MAX9288_ADDR,      {0x02, 0x00}, 0x0f,             0x01, i2c_write}, // disable I2S
        {MAX9288_ADDR,      {0x09, 0x00}, 0x40,             0x01, i2c_write}, // auto pixel count
        {MAX9288_ADDR,      {0x60, 0x00}, 0x33,             0x01, i2c_write}, // yuv422 8bit, normal mode
        {MAX9288_ADDR,      {0x00, 0x00}, 0x02,             0x01, i2c_delay}, // delay 2ms

        /* Sensor Setting */
        {SENSOR_INIT_ADDR,  {0x01, 0x03}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x00, 0x00}, 0x05,             0x01, i2c_delay}, // delay 5ms

        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x0c}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x1b}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x1c}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x1a}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x11}, 0x42,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x69, 0x00}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x69, 0x01}, 0x11,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x35, 0x03}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x25}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x03}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x04}, 0x21,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x05}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x06}, 0x91,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x36, 0x00}, 0x74,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x36, 0x01}, 0x2b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x36, 0x12}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x36, 0x11}, 0x67,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x36, 0x33}, 0xca,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x36, 0x02}, 0x2f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x36, 0x03}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x36, 0x30}, 0x28,             0x02, i2c_write},

        {SENSOR_INIT_ADDR,  {0x36, 0x31}, 0x16,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x37, 0x14}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x37, 0x1d}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x43, 0x00}, 0x38,             0x02, i2c_write},  // #38 YUYV, 3A UYVY
        {SENSOR_INIT_ADDR,  {0x30, 0x07}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x24}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x20}, 0x0b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x37, 0x02}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x37, 0x03}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x37, 0x04}, 0x32,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x37, 0x09}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x37, 0x09}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x37, 0x0c}, 0xc7,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x37, 0x0d}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x37, 0x12}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x37, 0x13}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x37, 0x15}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x1d}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x1c}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x22}, 0x50,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x24}, 0x50,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x15}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x04}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x05}, 0x1f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x00}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x01}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x06}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x07}, 0x29,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x02}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x03}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x08}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x09}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x0a}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x0b}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x0c}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x0d}, 0x71,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6e, 0x42}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6e, 0x43}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x0e}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x0f}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x13}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x11}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x1f}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x28}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x29}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x2a}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x38, 0x2b}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x36, 0x21}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x50, 0x05}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xd5}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xd6}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xd7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xd8}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xd9}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xda}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xdb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xdc}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xe8}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xe9}, 0x7f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xea}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xeb}, 0x7f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x00}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x01}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x02}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x03}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x04}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x05}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x06}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x07}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x08}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x09}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x0a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x0b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x0c}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x0d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x0e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x0f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x10}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x11}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x12}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x13}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x14}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x15}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x16}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x17}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x18}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x19}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x1a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x1b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x1c}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x1d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x1e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x1f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0xd0}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x50, 0x06}, 0x24,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x08}, 0x0d,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0xd7}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x8d}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x93}, 0x12,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0xd3}, 0x12,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x88}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x89}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0xc8}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0xc9}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0xcd}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x81}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x82}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0x89}, 0x76,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0x8a}, 0x47,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0x8b}, 0xef,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0x8c}, 0xc9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0x8d}, 0x49,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0x8e}, 0x30,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0x8f}, 0x67,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0x90}, 0x3f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0x91}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0x92}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0xa2}, 0x6d,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0xa3}, 0x55,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0xa4}, 0xc3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0xa5}, 0xb5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0xa6}, 0x43,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0xa7}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0xa8}, 0x5f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0xa9}, 0x4b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0xaa}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0xab}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x55, 0x81}, 0x52,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x00}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x01}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x02}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x03}, 0x0e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x04}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x05}, 0x0e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x06}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x07}, 0x36,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x08}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x09}, 0xd9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x0a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x0b}, 0x0f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x0c}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x0d}, 0x2c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x0e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x0f}, 0x59,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x10}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x11}, 0x7b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x12}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x13}, 0x22,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x14}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x15}, 0xd5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x16}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x17}, 0x13,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x18}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x19}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x1a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x1b}, 0x26,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x1c}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x1d}, 0xdc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x1e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x1f}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x20}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x21}, 0x24,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x22}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x23}, 0x56,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x24}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x25}, 0x85,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x26}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x53, 0x27}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x09}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x0a}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x0b}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x0c}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x0d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x0e}, 0xfa,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x0f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x10}, 0xfa,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x11}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x12}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x13}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x14}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x15}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x16}, 0x2c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x17}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x18}, 0x2c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x3b}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x3c}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x3d}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x3e}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x3f}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x40}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x41}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x42}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x43}, 0x09,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x44}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x45}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x46}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x47}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x51}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x56, 0x52}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x1a}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x1b}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x1c}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x1d}, 0x0a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x1e}, 0x0e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x1f}, 0x12,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x20}, 0x16,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x23}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x25}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x27}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x29}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x2b}, 0x12,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x2d}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x2f}, 0x1e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x41}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x42}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x43}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x44}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x45}, 0x0a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x46}, 0x0e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x47}, 0x12,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x48}, 0x16,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x4a}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x4c}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x4e}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x50}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x52}, 0x12,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x54}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x52, 0x56}, 0x1e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x06}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x07}, 0x71,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x0a}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x0b}, 0x70,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x0c}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x20}, 0x0e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x47, 0x00}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x47, 0x01}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x47, 0x02}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x40, 0x04}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x40, 0x05}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x40, 0x01}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x40, 0x50}, 0x22,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x40, 0x51}, 0x24,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x40, 0x52}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x40, 0x57}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x40, 0x5a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x42, 0x02}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x23}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x01, 0x00}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x01, 0x00}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x10}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x11}, 0x82,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x12}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x13}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x14}, 0x1f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x15}, 0xdd,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x16}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x17}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x18}, 0x36,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x19}, 0x66,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x1a}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x1b}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x1c}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x1d}, 0xe7,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x1e}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x1f}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x00}, 0x19,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x01}, 0xa0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x02}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x03}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x04}, 0xa9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x05}, 0xad,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x06}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x07}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x08}, 0x44,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x09}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x0a}, 0x68,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x0b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x0c}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x0d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x0e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x0f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x10}, 0x19,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x11}, 0xa0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x12}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x13}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x14}, 0xa9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x15}, 0xad,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x16}, 0x13,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x17}, 0xd0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x18}, 0x44,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x19}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x1a}, 0x68,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x1b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x1c}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x1d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x1e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x1f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x20}, 0x19,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x21}, 0xa0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x22}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x23}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x24}, 0xa9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x25}, 0xad,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x26}, 0x14,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x27}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x28}, 0x44,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x29}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x2a}, 0x68,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x2b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x2c}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x2d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x2e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x2f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x30}, 0x19,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x31}, 0xa0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x32}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x33}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x34}, 0xa9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x35}, 0xad,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x36}, 0x14,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x37}, 0xdc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x38}, 0x44,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x39}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x3a}, 0x68,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x3b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x3c}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x3d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x3e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x3f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x40}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x41}, 0x21,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x42}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x43}, 0xe4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x44}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x45}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x46}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x47}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x48}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x49}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x4a}, 0x50,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x4b}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x4c}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x4d}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x4e}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x4f}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x50}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x51}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x52}, 0x70,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x53}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x54}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x55}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x56}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x57}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x58}, 0x19,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x59}, 0xc0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x5a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x5b}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x5c}, 0xa9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x5d}, 0xce,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x5e}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x5f}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x60}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x61}, 0xa0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x62}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x63}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x64}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x65}, 0x6e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x66}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x67}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x68}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x69}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x6a}, 0x28,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x6b}, 0x76,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x6c}, 0x1a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x6d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x6e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x6f}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x70}, 0xaa,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x71}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x72}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x73}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x74}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x75}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x76}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x77}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x78}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x79}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x7a}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x7b}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x7c}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x7d}, 0xa0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x7e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x7f}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x80}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x81}, 0xc0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x82}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x83}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x84}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x85}, 0xc6,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x86}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x87}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x88}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x89}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x8a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x8b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x8c}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x8d}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x8e}, 0x28,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x8f}, 0x14,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x90}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x91}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x92}, 0x30,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x93}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x94}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x95}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x96}, 0xf8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x97}, 0xfd,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x98}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x99}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x9a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x9b}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x9c}, 0xa5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x9d}, 0x6b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x9e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0x9f}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xa0}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xa1}, 0xc0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xa2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xa3}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xa4}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xa5}, 0xc6,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xa6}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xa7}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xa8}, 0xe1,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xa9}, 0x6b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xaa}, 0x58,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xab}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xac}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xad}, 0x8e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xae}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xaf}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xb0}, 0xe1,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xb1}, 0x6b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xb2}, 0x30,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xb3}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xb4}, 0x98,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xb5}, 0xb0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xb6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xb7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xb8}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xb9}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xba}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xbb}, 0x6e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xbc}, 0xe5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xbd}, 0xa5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xbe}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xbf}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xc0}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xc1}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xc2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xc3}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xc4}, 0x95,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xc5}, 0x8b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xc6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xc7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xc8}, 0x94,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xc9}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xca}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xcb}, 0x70,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xcc}, 0xe5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xcd}, 0x65,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xce}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xcf}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xd0}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xd1}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xd2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xd3}, 0x62,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xd4}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xd5}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xd6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xd7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xd8}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xd9}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xda}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xdb}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xdc}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xdd}, 0x83,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xde}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xdf}, 0x29,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xe0}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xe1}, 0xe3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xe2}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xe3}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xe4}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xe5}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xe6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xe7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xe8}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xe9}, 0xa3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xea}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xeb}, 0x09,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xec}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xed}, 0xc3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xee}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xef}, 0x2a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xf0}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xf1}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xf2}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xf3}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xf4}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xf5}, 0x66,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xf6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xf7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xf8}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xf9}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xfa}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xfb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xfc}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xfd}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xfe}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd0, 0xff}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x00}, 0x98,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x01}, 0x90,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x02}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x03}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x04}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x05}, 0xae,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x06}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x07}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x08}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x09}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x0a}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x0b}, 0x4c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x0c}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x0d}, 0xc0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x0e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x0f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x10}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x11}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x12}, 0x30,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x13}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x14}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x15}, 0x65,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x16}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x17}, 0x6e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x18}, 0xe5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x19}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x1a}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x1b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x1c}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x1d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x1e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x1f}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x20}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x21}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x22}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x23}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x24}, 0x94,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x25}, 0x65,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x26}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x27}, 0x70,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x28}, 0xe5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x29}, 0x43,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x2a}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x2b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x2c}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x2d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x2e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x2f}, 0x3e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x30}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x31}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x32}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x33}, 0x24,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x34}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x35}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x36}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x37}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x38}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x39}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x3a}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x3b}, 0x24,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x3c}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x3d}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x3e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x3f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x40}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x41}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x42}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x43}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x44}, 0xbc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x45}, 0x23,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x46}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x47}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x48}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x49}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x4a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x4b}, 0x2a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x4c}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x4d}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x4e}, 0x6e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x4f}, 0x44,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x50}, 0x19,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x51}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x52}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x53}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x54}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x55}, 0xe8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x56}, 0x3d,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x57}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x58}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x59}, 0x67,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x5a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x5b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x5c}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x5d}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x5e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x5f}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x60}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x61}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x62}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x63}, 0x98,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x64}, 0xbc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x65}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x66}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x67}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x68}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x69}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x6a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x6b}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x6c}, 0xa9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x6d}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x6e}, 0x67,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x6f}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x70}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x71}, 0xa3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x72}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x73}, 0x19,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x74}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x75}, 0x8a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x76}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x77}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x78}, 0xa9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x79}, 0x68,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x7a}, 0x67,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x7b}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x7c}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x7d}, 0xc4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x7e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x7f}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x80}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x81}, 0x6b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x82}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x83}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x84}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x85}, 0x85,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x86}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x87}, 0x98,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x88}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x89}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x8a}, 0x30,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x8b}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x8c}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x8d}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x8e}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x8f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x90}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x91}, 0x83,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x92}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x93}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x94}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x95}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x96}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x97}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x98}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x99}, 0x0a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x9a}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x9b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x9c}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x9d}, 0x0b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x9e}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0x9f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xa0}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xa1}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xa2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xa3}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xa4}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xa5}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xa6}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xa7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xa8}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xa9}, 0x68,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xaa}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xab}, 0x22,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xac}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xad}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xae}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xaf}, 0x70,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xb0}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xb1}, 0xe8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xb2}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xb3}, 0x43,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xb4}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xb5}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xb6}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xb7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xb8}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xb9}, 0xa0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xba}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xbb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xbc}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xbd}, 0xc8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xbe}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xbf}, 0x42,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xc0}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xc1}, 0x66,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xc2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xc3}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xc4}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xc5}, 0xa5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xc6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xc7}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xc8}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xc9}, 0x83,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xca}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xcb}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xcc}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xcd}, 0xa5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xce}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xcf}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xd0}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xd1}, 0x67,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xd2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xd3}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xd4}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xd5}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xd6}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xd7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xd8}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xd9}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xda}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xdb}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xdc}, 0xbc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xdd}, 0x43,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xde}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xdf}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xe0}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xe1}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xe2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xe3}, 0x5b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xe4}, 0xbc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xe5}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xe6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xe7}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xe8}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xe9}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xea}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xeb}, 0xf6,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xec}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xed}, 0xa0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xee}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xef}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xf0}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xf1}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xf2}, 0x55,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xf3}, 0x86,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xf4}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xf5}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xf6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xf7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xf8}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xf9}, 0xc4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xfa}, 0x6e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xfb}, 0x45,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xfc}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xfd}, 0xe4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xfe}, 0x55,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd1, 0xff}, 0x87,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x00}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x01}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x02}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x03}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x04}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x05}, 0x66,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x06}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x07}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x08}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x09}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x0a}, 0x6e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x0b}, 0x46,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x0c}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x0d}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x0e}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x0f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x10}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x11}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x12}, 0x55,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x13}, 0x88,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x14}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x15}, 0x65,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x16}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x17}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x18}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x19}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x1a}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x1b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x1c}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x1d}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x1e}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x1f}, 0xce,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x20}, 0x19,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x21}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x22}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x23}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x24}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x25}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x26}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x27}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x28}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x29}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x2a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x2b}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x2c}, 0xbc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x2d}, 0x23,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x2e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x2f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x30}, 0x13,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x31}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x32}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x33}, 0xc8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x34}, 0x9d,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x35}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x36}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x37}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x38}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x39}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x3a}, 0x55,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x3b}, 0x86,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x3c}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x3d}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x3e}, 0x55,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x3f}, 0x87,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x40}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x41}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x42}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x43}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x44}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x45}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x46}, 0x55,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x47}, 0x88,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x48}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x49}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x4a}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x4b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x4c}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x4d}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x4e}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x4f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x50}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x51}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x52}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x53}, 0xc1,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x54}, 0x19,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x55}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x56}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x57}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x58}, 0x94,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x59}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x5a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x5b}, 0x72,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x5c}, 0xe5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x5d}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x5e}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x5f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x60}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x61}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x62}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x63}, 0x3f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x64}, 0x9d,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x65}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x66}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x67}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x68}, 0x85,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x69}, 0x4e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x6a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x6b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x6c}, 0x98,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x6d}, 0x70,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x6e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x6f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x70}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x71}, 0x8a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x72}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x73}, 0x6f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x74}, 0xe5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x75}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x76}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x77}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x78}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x79}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x7a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x7b}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x7c}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x7d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x7e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x7f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x80}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x81}, 0xaa,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x82}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x83}, 0x6e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x84}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x85}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x86}, 0x28,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x87}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x88}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x89}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x8a}, 0x28,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x8b}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x8c}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x8d}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x8e}, 0xf8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x8f}, 0x66,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x90}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x91}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x92}, 0x5b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x93}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x94}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x95}, 0x6a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x96}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x97}, 0x77,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x98}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x99}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x9a}, 0x5b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x9b}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x9c}, 0xbd,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x9d}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x9e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0x9f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xa0}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xa1}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xa2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xa3}, 0x3c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xa4}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xa5}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xa6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xa7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xa8}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xa9}, 0x8a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xaa}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xab}, 0x78,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xac}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xad}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xae}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xaf}, 0x88,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xb0}, 0xe1,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xb1}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xb2}, 0x5b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xb3}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xb4}, 0xbd,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xb5}, 0x6b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xb6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xb7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xb8}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xb9}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xba}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xbb}, 0x34,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xbc}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xbd}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xbe}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xbf}, 0x14,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xc0}, 0xb9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xc1}, 0x6b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xc2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xc3}, 0x88,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xc4}, 0x85,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xc5}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xc6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xc7}, 0x14,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xc8}, 0xbd,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xc9}, 0x68,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xca}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xcb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xcc}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xcd}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xce}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xcf}, 0x2c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xd0}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xd1}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xd2}, 0x58,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xd3}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xd4}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xd5}, 0x81,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xd6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xd7}, 0x14,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xd8}, 0xbd,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xd9}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xda}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xdb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xdc}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xdd}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xde}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xdf}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xe0}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xe1}, 0xc1,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xe2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xe3}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xe4}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xe5}, 0xa0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xe6}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xe7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xe8}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xe9}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xea}, 0x28,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xeb}, 0x14,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xec}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xed}, 0xc1,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xee}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xef}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xf0}, 0xbd,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xf1}, 0x66,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xf2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xf3}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xf4}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xf5}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xf6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xf7}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xf8}, 0x9d,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xf9}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xfa}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xfb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xfc}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xfd}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xfe}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd2, 0xff}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x00}, 0xbd,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x01}, 0xa3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x02}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x03}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x04}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x05}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x06}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x07}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x08}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x09}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x0a}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x0b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x0c}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x0d}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x0e}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x0f}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x10}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x11}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x12}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x13}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x14}, 0x85,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x15}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x16}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x17}, 0x14,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x18}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x19}, 0x83,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x1a}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x1b}, 0x29,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x1c}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x1d}, 0xc3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x1e}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x1f}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x20}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x21}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x22}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x23}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x24}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x25}, 0xa3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x26}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x27}, 0x2a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x28}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x29}, 0xe3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x2a}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x2b}, 0x09,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x2c}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x2d}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x2e}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x2f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x30}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x31}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x32}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x33}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x34}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x35}, 0x65,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x36}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x37}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x38}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x39}, 0x81,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x3a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x3b}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x3c}, 0xe3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x3d}, 0xe3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x3e}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x3f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x40}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x41}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x42}, 0xf8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x43}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x44}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x45}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x46}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x47}, 0x6f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x48}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x49}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x4a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x4b}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x4c}, 0x0f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x4d}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x4e}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x4f}, 0x9d,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x50}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x51}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x52}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x53}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x54}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x55}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x56}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x57}, 0x11,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x58}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x59}, 0x83,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x5a}, 0x6e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x5b}, 0x43,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x5c}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x5d}, 0x6c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x5e}, 0x28,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x5f}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x60}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x61}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x62}, 0x28,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x63}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x64}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x65}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x66}, 0xf8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x67}, 0x30,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x68}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x69}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x6a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x6b}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x6c}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x6d}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x6e}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x6f}, 0xc0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x70}, 0x85,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x71}, 0x4e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x72}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x73}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x74}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x75}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x76}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x77}, 0xe7,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x78}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x79}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x7a}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x7b}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x7c}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x7d}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x7e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x7f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x80}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x81}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x82}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x83}, 0xdb,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x84}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x85}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x86}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x87}, 0x14,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x88}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x89}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x8a}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x8b}, 0xce,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x8c}, 0x9d,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x8d}, 0x6b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x8e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x8f}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x90}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x91}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x92}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x93}, 0xc6,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x94}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x95}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x96}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x97}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x98}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x99}, 0xe3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x9a}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x9b}, 0x0f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x9c}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x9d}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x9e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0x9f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xa0}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xa1}, 0xa3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xa2}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xa3}, 0x0e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xa4}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xa5}, 0xc3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xa6}, 0x6e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xa7}, 0x42,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xa8}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xa9}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xaa}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xab}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xac}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xad}, 0x66,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xae}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xaf}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xb0}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xb1}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xb2}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xb3}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xb4}, 0x85,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xb5}, 0x21,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xb6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xb7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xb8}, 0x85,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xb9}, 0x41,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xba}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xbb}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xbc}, 0x85,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xbd}, 0x81,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xbe}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xbf}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xc0}, 0x85,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xc1}, 0xc1,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xc2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xc3}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xc4}, 0x86,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xc5}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xc6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xc7}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xc8}, 0x44,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xc9}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xca}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xcb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xcc}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xcd}, 0x21,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xce}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xcf}, 0x1c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xd0}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xd1}, 0x21,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xd2}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xd3}, 0xfc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xd4}, 0xd4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xd5}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xd6}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xd7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xd8}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xd9}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xda}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xdb}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xdc}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xdd}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xde}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xdf}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xe0}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xe1}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xe2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xe3}, 0x68,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xe4}, 0xbc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xe5}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xe6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xe7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xe8}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xe9}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xea}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xeb}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xec}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xed}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xee}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xef}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xf0}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xf1}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xf2}, 0xd9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xf3}, 0x98,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xf4}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xf5}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xf6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xf7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xf8}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xf9}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xfa}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xfb}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xfc}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xfd}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xfe}, 0xc4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd3, 0xff}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x00}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x01}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x02}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x03}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x04}, 0xbc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x05}, 0x23,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x06}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x07}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x08}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x09}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x0a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x0b}, 0x25,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x0c}, 0x9d,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x0d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x0e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x0f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x10}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x11}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x12}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x13}, 0x0b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x14}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x15}, 0xe8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x16}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x17}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x18}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x19}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x1a}, 0xd6,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x1b}, 0x24,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x1c}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x1d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x1e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x1f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x20}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x21}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x22}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x23}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x24}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x25}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x26}, 0xc4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x27}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x28}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x29}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x2a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x2b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x2c}, 0xbc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x2d}, 0x23,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x2e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x2f}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x30}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x31}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x32}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x33}, 0x1b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x34}, 0x9d,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x35}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x36}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x37}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x38}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x39}, 0xe8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x3a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x3b}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x3c}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x3d}, 0xc0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x3e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x3f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x40}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x41}, 0xa0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x42}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x43}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x44}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x45}, 0x67,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x46}, 0x30,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x47}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x48}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x49}, 0xa5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x4a}, 0xce,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x4b}, 0xb0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x4c}, 0x19,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x4d}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x4e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x4f}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x50}, 0xa9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x51}, 0x6b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x52}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x53}, 0x14,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x54}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x55}, 0x83,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x56}, 0x28,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x57}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x58}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x59}, 0xc6,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x5a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x5b}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x5c}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x5d}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x5e}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x5f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x60}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x61}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x62}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x63}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x64}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x65}, 0xa3,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x66}, 0x58,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x67}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x68}, 0xa4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x69}, 0xc6,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x6a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x6b}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x6c}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x6d}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x6e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x6f}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x70}, 0xbc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x71}, 0x46,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x72}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x73}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x74}, 0x94,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x75}, 0x85,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x76}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x77}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x78}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x79}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x7a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x7b}, 0x98,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x7c}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x7d}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x7e}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x7f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x80}, 0x0f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x81}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x82}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x83}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x84}, 0xdc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x85}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x86}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x87}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x88}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x89}, 0x68,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x8a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x8b}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x8c}, 0xa5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x8d}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x8e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x8f}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x90}, 0xbc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x91}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x92}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x93}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x94}, 0x0f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x95}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x96}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x97}, 0xea,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x98}, 0xb8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x99}, 0xe8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x9a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x9b}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x9c}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x9d}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x9e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0x9f}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xa0}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xa1}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xa2}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xa3}, 0x14,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xa4}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xa5}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xa6}, 0xe4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xa7}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xa8}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xa9}, 0x83,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xaa}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xab}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xac}, 0x85,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xad}, 0x21,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xae}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xaf}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xb0}, 0x44,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xb1}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xb2}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xb3}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xb4}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xb5}, 0x21,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xb6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xb7}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xb8}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xb9}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xba}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xbb}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xbc}, 0x9c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xbd}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xbe}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xbf}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xc0}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xc1}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xc2}, 0x09,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xc3}, 0xef,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xc4}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xc5}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xc6}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xc7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xc8}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xc9}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xca}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xcb}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xcc}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xcd}, 0x63,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xce}, 0xc9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xcf}, 0xef,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xd0}, 0xd8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xd1}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xd2}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xd3}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xd4}, 0x44,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xd5}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xd6}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xd7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xd8}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xd9}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xda}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xdb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xdc}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xdd}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xde}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xdf}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xe0}, 0xa8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xe1}, 0x84,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xe2}, 0x0a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xe3}, 0x12,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xe4}, 0x8c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xe5}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xe6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xe7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xe8}, 0xbc,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xe9}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xea}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xeb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xec}, 0x13,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xed}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xee}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xef}, 0xfe,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xf0}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xf1}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xf2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xf3}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xf4}, 0x44,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xf5}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xf6}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xf7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xf8}, 0x15,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xf9}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xfa}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xfb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xfc}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xfd}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xfe}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd4, 0xff}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd5, 0x00}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd5, 0x01}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd5, 0x02}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xd5, 0x03}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x0e}, 0x33,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x0f}, 0x33,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x0e}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x0f}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x10}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x11}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x12}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x13}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x05}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x08}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x46, 0x09}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x68, 0x04}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x68, 0x05}, 0x06,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x68, 0x06}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x51, 0x20}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x35, 0x10}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x35, 0x04}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x68, 0x00}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x0d}, 0x0f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x50, 0x00}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x50, 0x01}, 0xbf,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x50, 0x02}, 0x7e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x50, 0x03}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x50, 0x3d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x50}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x52}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x53}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x54}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x55}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x56}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x57}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x58}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x59}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x5b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x5c}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x5d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x5e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x5f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x60}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x61}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x62}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x64}, 0x88,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x65}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x66}, 0x8a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x67}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x68}, 0x86,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x69}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x6a}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x6b}, 0x50,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x6c}, 0x30,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x6d}, 0x28,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x6e}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x6f}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x7c}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x7d}, 0x38,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x7e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x7f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x80}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x81}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x82}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x83}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x84}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x85}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x86}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x87}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x88}, 0x34,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x89}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x8a}, 0x34,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x8b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x8c}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x8d}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x8e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x8f}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x90}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x92}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x93}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x98}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x99}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x9a}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x9b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x9c}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x9d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x9e}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0x9f}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xa0}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xa1}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xa2}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xa3}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xa4}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xa5}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xa6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xa7}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xa8}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xa9}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xaa}, 0x0d,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xab}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xac}, 0x0f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xad}, 0xc0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xb4}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xb5}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xb6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xb7}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xb8}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xb9}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xba}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xbb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xbc}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xbd}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xbe}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xbf}, 0x33,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xc8}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xc9}, 0xd0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xca}, 0x0e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xcb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xcc}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xcd}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xce}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xcf}, 0x18,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xd0}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xd1}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xe0}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xe1}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xe2}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xe4}, 0x10,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xe5}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xe6}, 0x30,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xe7}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xe8}, 0x50,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xe9}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xea}, 0x70,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xeb}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xec}, 0x90,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xed}, 0xa0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xee}, 0xb0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xef}, 0xc0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xf0}, 0xd0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xf1}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xf2}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xf3}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xf4}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xf5}, 0x20,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xf6}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xf7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xf8}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xf9}, 0x0b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xfa}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xfb}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xfc}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xfd}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xfe}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xff}, 0x02,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x00}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x01}, 0x74,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x02}, 0x58,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x03}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x04}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x05}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x06}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x07}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x08}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x09}, 0xc0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x0a}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x0b}, 0xa0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x0c}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x0d}, 0x2c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x0e}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x0f}, 0x0a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x10}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x11}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x12}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x13}, 0x80,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x14}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x15}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x18}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x19}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x1a}, 0x07,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc5, 0x1b}, 0x70,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc2, 0xe0}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc2, 0xe1}, 0x51,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc2, 0xe2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc2, 0xe3}, 0xd6,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc2, 0xe4}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc2, 0xe5}, 0x5e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc2, 0xe9}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc2, 0xea}, 0x7a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc2, 0xeb}, 0x90,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc2, 0xed}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc2, 0xee}, 0x7a,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc2, 0xef}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x08}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x09}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x0a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x0c}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x0d}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x0e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x0f}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x10}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x11}, 0x60,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x12}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x13}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x14}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x15}, 0x7f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x16}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x17}, 0x0b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x18}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x19}, 0x0c,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x1a}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x1b}, 0xe0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x1c}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x1d}, 0x14,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x1e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x1f}, 0xc5,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x20}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x21}, 0x4b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x22}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x23}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x24}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x25}, 0xe8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x26}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x27}, 0x46,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x28}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x29}, 0xd2,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x2a}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x2b}, 0xe4,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x2c}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x2d}, 0xbb,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x2e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x2f}, 0x61,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x30}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x31}, 0xf9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x32}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x33}, 0xd9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x34}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x35}, 0x2e,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x36}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x37}, 0xb1,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x38}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x39}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x3a}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x3b}, 0xeb,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x3c}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x3d}, 0xe8,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x3e}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x3f}, 0x48,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x40}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x41}, 0xd0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x42}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x43}, 0xed,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x44}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x45}, 0xad,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x46}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x47}, 0x66,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x48}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x49}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x67, 0x00}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x67, 0x01}, 0x7b,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x67, 0x02}, 0xfd,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x67, 0x03}, 0xf9,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x67, 0x04}, 0x3d,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x67, 0x05}, 0x71,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x67, 0x06}, 0x78,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x67, 0x08}, 0x05,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x06}, 0x6f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x07}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x0a}, 0x6f,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x0b}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x6f, 0x00}, 0x03,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x4c}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x4d}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x4e}, 0x46,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x4f}, 0x55,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x50}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x51}, 0x40,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x52}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x53}, 0xff,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x54}, 0x04,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x55}, 0x08,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x56}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x57}, 0xef,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x58}, 0x30,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x59}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x5a}, 0x64,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x5b}, 0x46,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc3, 0x5c}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x42}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x1b}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x1c}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0x30, 0x1a}, 0xf0,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xce, 0xb0}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xce, 0xb1}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xce, 0xb2}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xce, 0xb3}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xce, 0xb4}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xce, 0xb5}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xce, 0xb6}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xce, 0xb7}, 0x00,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xbc}, 0x01,             0x02, i2c_write},
        {SENSOR_INIT_ADDR,  {0xc4, 0xbd}, 0x60,             0x02, i2c_write},
        // end Sensor setting.
        {MAX9271_INIT_ADDR,  {0x07, 0x01}, 0x80,            0x01, i2c_write},  // DBL=1, HIBW=0, rising edge
        {MAX9271_INIT_ADDR,  {0x00, 0x00}, 0x05,            0x01, i2c_delay}, // delay 5ms

        //-----------------------------------------------------------------------------------------dd
        {MAX9271_INIT_ADDR,  {0x00, 0x00}, 0x05,            0x01, i2c_delay},  // Delay 5ms
        {MAX9271_INIT_ADDR,  {0x20, 0x00}, 0x05,            0x01, i2c_write},  // xbar CbY
        {MAX9271_INIT_ADDR,  {0x21, 0x00}, 0x06,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x22, 0x00}, 0x07,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x23, 0x00}, 0x08,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x24, 0x00}, 0x09,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x25, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x26, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x27, 0x00}, 0x0e,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x28, 0x00}, 0x0f,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x29, 0x00}, 0x0e,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x2a, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x2b, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x2c, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x2d, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x2e, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x2f, 0x00}, 0x40,            0x01, i2c_write},

        {MAX9271_INIT_ADDR,  {0x30, 0x00}, 0x12,            0x01, i2c_write},  // xbar CrY
        {MAX9271_INIT_ADDR,  {0x31, 0x00}, 0x13,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x32, 0x00}, 0x14,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x33, 0x00}, 0x15,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x34, 0x00}, 0x16,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x35, 0x00}, 0x17,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x36, 0x00}, 0x18,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x37, 0x00}, 0x19,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x38, 0x00}, 0x02,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x39, 0x00}, 0x03,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x3a, 0x00}, 0x04,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x3b, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x3c, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x3d, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x3e, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x3f, 0x00}, 0x0e,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x40, 0x00}, 0x0f,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x41, 0x00}, 0x0e,            0x01, i2c_write},
//-------//-----------------------------------------------------------------------------------------dd
        {MAX9271_INIT_ADDR,  {0x20, 0x00}, 0x05,            0x01, i2c_write},  // xbar CbY
        {MAX9271_INIT_ADDR,  {0x21, 0x00}, 0x06,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x22, 0x00}, 0x07,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x23, 0x00}, 0x08,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x24, 0x00}, 0x09,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x25, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x26, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x27, 0x00}, 0x0e,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x28, 0x00}, 0x0f,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x29, 0x00}, 0x0e,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x2a, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x2b, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x2c, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x2d, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x2e, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x2f, 0x00}, 0x40,            0x01, i2c_write},

        {MAX9271_INIT_ADDR,  {0x30, 0x00}, 0x12,            0x01, i2c_write},  // xbar CrY
        {MAX9271_INIT_ADDR,  {0x31, 0x00}, 0x13,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x32, 0x00}, 0x14,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x33, 0x00}, 0x15,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x34, 0x00}, 0x16,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x35, 0x00}, 0x17,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x36, 0x00}, 0x18,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x37, 0x00}, 0x19,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x38, 0x00}, 0x02,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x39, 0x00}, 0x03,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x3a, 0x00}, 0x04,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x3b, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x3c, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x3d, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x3e, 0x00}, 0x40,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x3f, 0x00}, 0x0e,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x40, 0x00}, 0x0f,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x41, 0x00}, 0x0e,            0x01, i2c_write},
//-------------------------------------------------------------------------------------
        {MAX9271_INIT_ADDR,  {0x43, 0x00}, 0x01,            0x01, i2c_write},  // #disable vsync re-gen

        {MAX9271_INIT_ADDR,  {0x44, 0x00}, 0x30,            0x01, i2c_write},  // vsync delay; 30 d5 90
        {MAX9271_INIT_ADDR,  {0x45, 0x00}, 0x9a,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x46, 0x00}, 0x00,            0x01, i2c_write},

        {MAX9271_INIT_ADDR,  {0x47, 0x00}, 0x01,            0x01, i2c_write},  // vsync high
        {MAX9271_INIT_ADDR,  {0x48, 0x00}, 0x00,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x49, 0x00}, 0x00,            0x01, i2c_write},

        {MAX9271_INIT_ADDR,  {0x4a, 0x00}, 0x00,            0x01, i2c_write},  // vsync low; 2f d5 90
        {MAX9271_INIT_ADDR,  {0x4b, 0x00}, 0x00,            0x01, i2c_write},
        {MAX9271_INIT_ADDR,  {0x4c, 0x00}, 0x00,            0x01, i2c_write},

        {MAX9271_INIT_ADDR,  {0x43, 0x00}, 0x21,            0x01, i2c_write},  // enable vsync re-gen
        {MAX9271_INIT_ADDR,  {0x4d, 0x00}, 0xc8,            0x01, i2c_write},  // HIMM, Coax, Inv Vsync
        {MAX9271_INIT_ADDR,  {0x67, 0x00}, 0xe2,            0x01, i2c_write},  // Auto link, Force align (e4 align at HS rising edge)

        {MAX9271_INIT_ADDR,  {0x04, 0x00}, 0x83,            0x01, i2c_write},  // enable Serial Interface

        {MAX9271_INIT_ADDR,  {0x00, 0x00}, 0x05,            0x01, i2c_delay},  // Delay 5ms


	{OV490_CH0_ADDR,    {0xFF, 0xFD}, 0x80,             0x02, i2c_write},
	{OV490_CH0_ADDR,    {0xFF, 0xFE}, 0x19,             0x02, i2c_write},
	{OV490_CH0_ADDR,    {0x50, 0x00}, 0x03,             0x02, i2c_write},
	{OV490_CH0_ADDR,    {0xFF, 0xFE}, 0x80,             0x02, i2c_write},
	{OV490_CH0_ADDR,    {0x00, 0xC0}, 0xD6,             0x02, i2c_write},
};

struct max9288_datafmt {
	__u32 code;
	enum v4l2_colorspace colorspace;
};

struct max9288 {
	struct v4l2_subdev		subdev;
	const struct max9288_datafmt	*fmt;
	struct v4l2_clk			*clk;
	struct i2c_client *client;

	/* blanking information */
	u32 link;
};

static const struct max9288_datafmt max9288_colour_fmts[] = {
	{MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_DEFAULT},
};

static struct max9288 *to_max9288(struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct max9288, subdev);
}

static int i2c_delay(struct i2c_client *client, u16 slave_addr,
		u8 *reg, unsigned int reg_len, u8 *value)
{
	max9288_info("delay %d ms", *value);
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
		max9288_err("reg/val/reg_len is %02x/%02x/%d",
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
#if 1
    if(slave_addr == 0xD0){
        max9288_info(" luozh read dev/reg/val/ret is 0x%02x/0x%02x%02x/0x%x/%d\n",
                slave_addr, reg[0],reg[1], *val, ret);
    }
#endif
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
		max9288_err("reg/val/reg_len is %02x/%02x/%d",
			*reg, *val, reg_len);
		return -EINVAL;
	}

	data = kzalloc(size, GFP_KERNEL);
	if (data == NULL) {
		max9288_err("kzalloc failed!\n");
		return -ENOSPC;
	}
#if 1
    if(slave_addr == 0xD0){
        max9288_info(" luozhanhong  reg = 0x%x, 0x%02x%02x,val = 0x%x",slave_addr,reg[0],reg[1],val[0]);
    }
#endif
	(void)memcpy(data, reg, reg_len);
	*(data + reg_len) = *val;

	(void)memset(&msg, 0, sizeof(msg));
	msg.addr = (slave_addr >> 1);
	msg.flags = 0;
	msg.len = (u16)size;
	msg.buf = data;

	client->addr = (slave_addr >> 1);
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
	    max9288_err("write dev/reg/val/ret is %02x/%02x/%02x/%d",
	        slave_addr, *reg, *val, ret);
    }

	kfree(data);
	data = NULL;
	//max9288_info("write dev/reg/val/ret is %02x/%02x/%02x/%d",
	//          slave_addr, *reg, *val, ret);
	return ret;
}

static int read_max9288_id(struct i2c_client *client, u8 *id_val)
{
	int ret = 0;
	u8 max9288_id_reg = MAX9288_ID_REG;

	ret = i2c_read(client, MAX9288_ADDR, &max9288_id_reg, 1, id_val);
	if (ret != 2) {
		max9288_err("ret=%d", ret);
		return -EIO;
	}

	if (*id_val != (u8)MAX9288_ID) {
		max9288_err("max9288 ID not match. Default is %x but read from register is %x. ret=%d",
			MAX9288_ID, *id_val, ret);
		ret = -ENODEV;
	} else {
		max9288_info("max9288 ID match. Default is %x and read from register is %x. ret=%d",
			MAX9288_ID, *id_val, ret);
	}

	return ret;
}

static int max9288_write_array(struct i2c_client *client,
		struct reg_val_ops *cmd, unsigned long len)
{
	int ret = 0;
	unsigned long index = 0;
	u8 val;

	for (; index < len; ++index) {
		 //mdelay(5);
		ret = cmd[index].i2c_ops(client, cmd[index].slave_addr,
			cmd[index].reg, cmd[index].reg_len, &(cmd[index].val));
		if (ret < 0) {
			max9288_err("dev/reg/val/ret/index %x/%x/%x/%d/%lu",
				cmd[index].slave_addr, cmd[index].reg[0],
				cmd[index].val, ret, index);
			return ret;
		}
#if 1
        if(cmd[index].slave_addr == 0xD0){
            mdelay(1);
            ret = i2c_read(client, cmd[index].slave_addr, cmd[index].reg, 1, &val);
            if (ret < 0) debug("i2c_read error!!!!\n");
        }
#endif
	}

	return 0;
}

/* static int max9288_set_link_config(struct i2c_client *client, u8 *val) */
/* { */
/* 	int ret = 0; */
/* 	u8 link_config_reg = MAX9288_LINK_CONFIG_REG; */

/* 	ret = i2c_write(client, MAX9288_ADDR, &link_config_reg, 1, val); */
/* 	if (ret != 1) { */
/* 		max9288_info("link config set fail. ret = %d", ret); */
/* 		return -EIO; */
/* 	} */

/* 	return ret; */
/* } */

int max9288_get_lock_status(struct i2c_client *client, u8 *val)
{
	int ret = 0;
	u8 lock_reg = MAX9288_LOCK_REG;

	ret = i2c_read(client, MAX9288_ADDR, &lock_reg, 1, val);
	if (ret != 2) {
		max9288_err("ret=%d", ret);
		return -EIO;
	}
	max9288_info("camera locke val = 0x%x\n",*val);
	if ((*val & (u8)0x80) != 0u) {
		max9288_info("camera links are locked");
	} else {
		max9288_err("camera links are not locked");
		ret =  -ENODEV;
	}

	return ret;
}

/* static int max9288_get_link(struct i2c_client *client, u8 *val) */
/* { */
/* 	int ret = 0; */
/* 	u8 link_reg = MAX9288_LINK_REG; */

/* 	ret = i2c_read(client, MAX9288_ADDR, &link_reg, 1, val); */
/* 	if (ret != 2) { */
/* 		max9288_err("ret=%d", ret); */
/* 		ret = -EIO; */
/* 	} */

/* 	return ret; */
/* } */

static int max9288_camera_ch_addr_init(struct i2c_client *client,
	struct reg_val_ops *cmd, unsigned long len, int ch)
{
	int ret = 0;
	unsigned long index = 0;

	/*choose channel*/
	ret = cmd[index].i2c_ops(client, cmd[index].slave_addr,
		cmd[index].reg, cmd[index].reg_len, &(cmd[index].val));
	if (ret < 0) {
		max9288_err("dev/reg/val/ret/index %x/%x/%x/%d/%lu",
			cmd[index].slave_addr, cmd[index].reg[0],
			cmd[index].val, ret, index);
		return -1;
	}

	index = 1;
	/*read max9271 id use 0x80*/
	ret = cmd[index].i2c_ops(client, cmd[index].slave_addr,
		cmd[index].reg, cmd[index].reg_len, &(cmd[index].val));
	/*
	 * device is not exist. Maybe device address has been changed but
	 * device is not reset because power off time is too short.
	 * Now reset it.
	 */
	if (ret < 0) {
		/*use channel device id to read*/
		index = 2;
		ret = cmd[index].i2c_ops(client, cmd[index].slave_addr,
			cmd[index].reg, cmd[index].reg_len, &(cmd[index].val));
		if (ret < 0) {
			max9288_err("dev/reg/val/ret/index %x/%x/%x/%d/%lu",
				cmd[index].slave_addr, cmd[index].reg[0],
				cmd[index].val, ret, index);
			return -1;
		}
		max9288_info("ch %d addr %02x has not been cleared",
			ch, cmd[index].val);

		/*use channel id to set address to 0x80*/
		index = 3;
		ret =  cmd[index].i2c_ops(client, cmd[index].slave_addr,
			cmd[index].reg, cmd[index].reg_len, &(cmd[index].val));
		if (ret < 0) {
			max9288_err("dev/reg/val/ret/index %x/%x/%x/%d/%lu",
				cmd[index].slave_addr, cmd[index].reg[0],
				cmd[index].val, ret, index);
			return -1;
		}

		/*use 0x80 to read id*/
		index = 4;
		ret =  cmd[index].i2c_ops(client, cmd[index].slave_addr,
			cmd[index].reg, cmd[index].reg_len, &(cmd[index].val));
		if (ret < 0) {
			max9288_err("dev/reg/val/ret/index %x/%x/%x/%d/%lu",
				cmd[index].slave_addr, cmd[index].reg[0],
				cmd[index].val, ret, index);
			return -1;
		}

	}

	for (index = 5; index < len; ++index) {
		ret =  cmd[index].i2c_ops(client, cmd[index].slave_addr,
			cmd[index].reg, cmd[index].reg_len, &(cmd[index].val));
		if (ret < 0) {
			max9288_err("dev/reg/val/ret/index %x/%x/%x/%d/%lu",
				cmd[index].slave_addr, cmd[index].reg[0],
				cmd[index].val, ret, index);
			return -1;
		}
	}

	return 0;
}

static int max9288_cab888_4v4_is_init(struct i2c_client *client)
{
	unsigned long len = ARRAY_SIZE(MAX9288_CAB888_4V4_r_cmd);
	struct reg_val_ops *cmd = MAX9288_CAB888_4V4_r_cmd;
	unsigned long index = 0UL;
	int ret = 0;
	u8 val = 0;

	for (index = 0; index < len; ++index) {
		val = cmd[index].val;
		ret = cmd[index].i2c_ops(client, cmd[index].slave_addr,
			cmd[index].reg, cmd[index].reg_len, &(cmd[index].val));
		if ((ret < 0) || (val != cmd[index].val))
			return -1;
	}

	return 0;
}

static int max9288_cab888_4v4_init(struct i2c_client *client)
{
	int ret = 0;

	ret  = max9288_cab888_4v4_is_init(client);
	if (ret == 0) {
		max9288_info("max9288 and cab888 have been initialized");
		return 0;
	}

	ret = max9288_write_array(client,
		MAX9288_CAB888_4V4_pre_init_cmd,
		ARRAY_SIZE(MAX9288_CAB888_4V4_pre_init_cmd));
	if (ret < 0)
		return ret;

	ret = max9288_camera_ch_addr_init(client,
		MAX9288_CAB888_CH0_addr_init_cmd,
		ARRAY_SIZE(MAX9288_CAB888_CH0_addr_init_cmd),
		0);
	if (ret < 0)
		return ret;

	ret = max9288_camera_ch_addr_init(client,
		MAX9288_CAB888_CH1_addr_init_cmd,
		ARRAY_SIZE(MAX9288_CAB888_CH1_addr_init_cmd),
		0);
	if (ret < 0)
		return ret;

	ret = max9288_camera_ch_addr_init(client,
		MAX9288_CAB888_CH2_addr_init_cmd,
		ARRAY_SIZE(MAX9288_CAB888_CH2_addr_init_cmd),
		0);
	if (ret < 0)
		return ret;

	ret = max9288_camera_ch_addr_init(client,
		MAX9288_CAB888_CH3_addr_init_cmd,
		ARRAY_SIZE(MAX9288_CAB888_CH3_addr_init_cmd),
		0);
	if (ret < 0)
		return ret;

	ret = max9288_write_array(client,
		MAX9288_CAB888_4V4_en_cmd,
		ARRAY_SIZE(MAX9288_CAB888_4V4_en_cmd));
	if (ret < 0)
		return ret;

	return 0;
}

static int max9288_camera_init(struct i2c_client *client)
{
	int ret = 0;
	u8 max9288_id_val = 0;
    u8 link_reg_val = 0;
    u8 lock_reg_val = 0;
    int read_cnt = 0;
    u32 link_cnt = 0;
    struct max9288 *priv = NULL;
	unsigned long skip_len = 0;
	//u64 delay = 10000;

    max9288_info("MAX9288!");
    debug("--->>>in\n");
    /*MAX9288 ID confirm*/
    for (read_cnt = 0; read_cnt < 3; ++read_cnt) {
		ret = read_max9288_id(client, &max9288_id_val);
		if (ret == 2)
			break;

		if (read_cnt == 2) {
			max9288_err("read max9288 ID time out");
			return -EIO;
		}

		//udelay(delay);
    }

    link_reg_val = 1;
    link_cnt = 1;

	priv = to_max9288(client);
	priv->link = link_cnt;

	/*init max9288 command*/
	if (link_cnt == 1U) {
		skip_len = 5;
		ret = max9288_write_array(client,
			MAX9288_CAB888_1v1_init_cmd,
			ARRAY_SIZE(MAX9288_CAB888_1v1_init_cmd) - skip_len);
	} else if (link_cnt == 4U) {
		ret = max9288_cab888_4v4_init(client);
		if (ret < 0)
			return ret;
	} else {
		max9288_err("error link cnt %u", link_cnt);
		ret = -EINVAL;
    }

	ret = max9288_get_lock_status(client, &lock_reg_val);
    if ( 2 == ret){
	    max9288_info("camera locked ok!\n");
    }else{
        max9288_err("error locked %d\n",ret);
        return -EINVAL;
    }
    debug("--->>>out\n");
    return ret;
}

static int max9288_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_4_LANE | V4L2_MBUS_CSI2_CHANNELS |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	return 0;
}

static int max9288_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int max9288_s_mbus_config(struct v4l2_subdev *sd,
			     const struct v4l2_mbus_config *cfg)
{
	return 0;
}

static int max9288_s_power(struct v4l2_subdev *sd, int on)
{
    //struct i2c_client *client = v4l2_get_subdevdata(sd);
    //struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
    //struct max9288 *priv = to_max9288(client);
    int ret = 0;
    debug("on = %d\n",on);
#if 0
    if (on == 0){
        ret = pinctrl_select_state(max9288_pctrl.pinctrl,max9288_pctrl.gpio_state_suspend);
        if (ret < 0) {
            max9288_err("%s, %d\n", __func__, __LINE__);
            return ret;
        }
        if (max9288_pctrl.vdd1v8 != NULL){
            ret = regulator_disable(max9288_pctrl.vdd1v8);
            if (ret != 0) {
                max9288_err("Failed to disable vdd1v8!\n");
                return ret;
            }
        }
        //return soc_camera_power_off(&client->dev, ssdd, priv->clk);
    }else{
        ret = pinctrl_select_state(max9288_pctrl.pinctrl,max9288_pctrl.gpio_state_active);
        if (ret < 0) {
            max9288_err("%s, %d\n", __func__, __LINE__);
            return ret;
        }
        if (pmu1v8_power_init(&client->dev,client->dev.of_node)<0){
            max9288_err("%s, %d\n", __func__, __LINE__);
            return -EINVAL;
        }
    }
    mdelay(500);
#endif
    //return soc_camera_power_on(&client->dev, ssdd, priv->clk);
    return ret;
}

static int max9288_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if ((code->pad != 0u) ||
	    (unsigned long)code->index >= ARRAY_SIZE(max9288_colour_fmts))
		return -EINVAL;

	code->code = max9288_colour_fmts[code->index].code;
	return 0;
}

static const struct max9288_datafmt *max9288_find_datafmt(u32 code)
{
	__u64 i;

	for (i = 0; i < ARRAY_SIZE(max9288_colour_fmts); i++)
		if (max9288_colour_fmts[i].code == code)
			return max9288_colour_fmts + i;

	return NULL;
}

static int max9288_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct max9288_datafmt *fmt = max9288_find_datafmt(mf->code);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct max9288 *priv = to_max9288(client);
	int ret = 0;
    unsigned long cmd_len = 0;
    unsigned long skip = 0;
    if (format->pad != 0u)
        return -EINVAL;

    //debug("------->>>>in\n");
	if (fmt == NULL) {
		/* MIPI CSI could have changed the format, double-check */
		if (format->which == (__u32)V4L2_SUBDEV_FORMAT_ACTIVE)
			return -EINVAL;
		mf->code	= max9288_colour_fmts[0].code;
		mf->colorspace	= (__u32)max9288_colour_fmts[0].colorspace;
	}

	if ((mf->width != (__u32)CAB888_WIDTH) ||
		(mf->height != (__u32)CAB888_HEIGHT * priv->link)) {
		max9288_err("width/height %d/%d not support. Set %d/%d as default",
			mf->width, mf->height,
			CAB888_WIDTH, CAB888_HEIGHT * priv->link);
	}

	mf->width	= CAB888_WIDTH;
	mf->height	= CAB888_HEIGHT * priv->link;
	mf->field	= (__u32)V4L2_FIELD_NONE;

	if (format->which == (__u32)V4L2_SUBDEV_FORMAT_ACTIVE)
		priv->fmt = max9288_find_datafmt(mf->code);
	else
		cfg->try_fmt = *mf;

	if (is_testpattern > 0) {
		/*init max9288 command*/
		if (priv->link == 1U) {
			cmd_len = ARRAY_SIZE(MAX9288_CAB888_1v1_init_cmd);
			skip = 5;
			ret = max9288_write_array(client,
				MAX9288_CAB888_1v1_init_cmd + cmd_len - skip,
				5);
		} else if (priv->link == 4U) {
			cmd_len = ARRAY_SIZE(MAX9288_CAB888_4v4_init_cmd);
			skip = 10;
			ret = max9288_write_array(client,
				MAX9288_CAB888_4v4_init_cmd + cmd_len - skip,
				10);
		} else {
			max9288_err("error link cnt %d", priv->link);
			ret = -EINVAL;
		}
	}
	return ret;
}

static int max9288_g_register(struct v4l2_subdev *sd,
		struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	u8 val = 0;
    u8 max9288_reg = (u8)(reg->reg);
    if (reg == NULL)
        return -EINVAL;

    debug("-----> in \n");
    //ret =  max9288_camera_init(client);
    if (ret < 0){
        debug("debug error!!!!!\n");
        return ret;
    }
	max9288_info("read max9288 reg %llx", reg->reg);
	if (reg->match.type == (u8)0) {
		ret = i2c_read(client, MAX9288_ADDR, &max9288_reg, 1, &val);
		if (ret != 2) {
			max9288_err("ret=%d", ret);
			return -EIO;
		}
        ret = 2;
		reg->val = 1;
	}

	return ret;
}

static const struct v4l2_subdev_video_ops max9288_subdev_video_ops = {
	.g_mbus_config	= max9288_g_mbus_config,
	.s_stream	= max9288_s_stream,
	.s_mbus_config = max9288_s_mbus_config,
};

static const struct v4l2_subdev_core_ops max9288_subdev_core_ops = {
	.s_power	= max9288_s_power,
	.g_register	= max9288_g_register,
};

static const struct v4l2_subdev_pad_ops max9288_subdev_pad_ops = {
	.enum_mbus_code = max9288_enum_mbus_code,
	.set_fmt	= max9288_set_fmt,
};

static const struct v4l2_subdev_ops max9288_subdev_ops = {
	.core	= &max9288_subdev_core_ops,
	.video	= &max9288_subdev_video_ops,
	.pad = &max9288_subdev_pad_ops,
};

int max9288_pinctrl_init(struct device *dev)
{
    int ret = 0;
    debug("%s:%d input\n",__func__,__LINE__);
    max9288_pctrl.pinctrl = devm_pinctrl_get(dev);
    if(IS_ERR(max9288_pctrl.pinctrl)) {
        debug("%s:%d Getting pinctrl handle failed\n",
                __func__,__LINE__);
        return-EINVAL;
    }
    max9288_pctrl.gpio_state_active = pinctrl_lookup_state(max9288_pctrl.pinctrl,"max9288pins_active");
    if(IS_ERR(max9288_pctrl.gpio_state_active)) {
        debug("%s:%d failed to get the active state pinctrl handle!!\n",
                __func__,__LINE__);
        return-EINVAL;
    }

    max9288_pctrl.gpio_state_suspend = pinctrl_lookup_state(max9288_pctrl.pinctrl,"max9288pins_suspend");
    if(IS_ERR(max9288_pctrl.gpio_state_suspend)) {
        debug("%s:%d failed to get the suspend state pinctrl handle!!\n",
                __func__,__LINE__);
        return-EINVAL;
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
        max9288_pctrl.vdd1v8 = devm_regulator_get(dev, vdd1v8);
        if (IS_ERR(max9288_pctrl.vdd1v8)) {
            ret = PTR_ERR(max9288_pctrl.vdd1v8);
            goto err_vdd1v8;
        }
        ret = regulator_enable(max9288_pctrl.vdd1v8);
        if (ret < 0) {
            pr_err("Failed to enable vddoa\n");
            goto err_vdd1v8;
        }
    }
    usleep_range(3000,3000);

    return 0;

err_vdd1v8:
    regulator_disable(max9288_pctrl.vdd1v8);

    return ret;
}
#define sensor_register_debug
#ifdef sensor_register_debug
static struct i2c_client *sensor_client = NULL;
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
//for sensor ov10635
static ssize_t register_show_ov10635(struct device *dev, struct device_attribute *attr, char *buf)
{
    i2c_read(sensor_client,SENSOR_INIT_ADDR, addr,2,&data);
    debug("luozh: addr=0x%02x%02x, data=0x%x\n", addr[0],addr[1], data);
    return sprintf(buf, "%x\n", data);
}

static ssize_t register_store_ov10635(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    data = (unsigned short)simple_strtoul(buf, NULL, 16);
    i2c_write(sensor_client,SENSOR_INIT_ADDR,addr, 2, &data);

    debug("luozh: addr=0x%02x%02x, data=0x%x\n", addr[0],addr[1], data);
    return count;
}
//for max9288
static ssize_t register_show_max9288(struct device *dev, struct device_attribute *attr, char *buf)
{
	addr[0] = addr[1];
    i2c_read(sensor_client,MAX9288_ADDR, addr,1,&data);
    debug("luozh: addr=0x%02x, data=0x%x\n", addr[0], data);
    return sprintf(buf, "%x\n", data);
}

static ssize_t register_store_max9288(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	addr[0] = addr[1];
    data = (unsigned short)simple_strtoul(buf, NULL, 16);
    i2c_write(sensor_client,MAX9288_ADDR,addr, 1, &data);

    debug("luozh: addr=0x%02x, data=0x%x\n", addr[0], data);
    return count;
}
//for max96705
static ssize_t register_show_max96705(struct device *dev, struct device_attribute *attr, char *buf)
{
	addr[0] = addr[1];
    i2c_read(sensor_client,MAX9271_INIT_ADDR, addr,1,&data);
    debug("luozh: addr=0x%02x, data=0x%x\n", addr[0], data);
    return sprintf(buf, "%x\n", data);
}

static ssize_t register_store_max96705(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	addr[0] = addr[1];
    data = (unsigned short)simple_strtoul(buf, NULL, 16);
    i2c_write(sensor_client,MAX9271_INIT_ADDR,addr, 1, &data);

    debug("luozh: addr=0x%02x, data=0x%x\n", addr[0], data);
    return count;
}
//for max20088
#define  MAX20088_ADDR  0x52//0x29
static ssize_t register_show_max20088(struct device *dev, struct device_attribute *attr, char *buf)
{
	addr[0] = addr[1];
    i2c_read(sensor_client,MAX20088_ADDR, addr,1,&data);
    debug("luozh: addr=0x%02x, data=0x%x\n", addr[0], data);
    return sprintf(buf, "0x%02x\n", data);
}

static ssize_t register_store_max20088(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	addr[0] = addr[1];
    data = (unsigned short)simple_strtoul(buf, NULL, 16);
    i2c_write(sensor_client,MAX20088_ADDR,addr, 1, &data);

    debug("luozh: addr=0x%02x, data=0x%x\n", addr[0], data);
    return count;
}

//for max20086
#define MAX20086_ADDR 0x50
ssize_t register_show_max20086(struct device *dev, struct device_attribute *attr, char *buf)
{
	addr[0] = addr[1];
    i2c_read(sensor_client,MAX20086_ADDR, addr,1,&data);
    debug("luozh: addr=0x%02x, data=0x%x\n", addr[0], data);
    return sprintf(buf, "0x%02x\n", data);
}

ssize_t register_store_max20086(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	addr[0] = addr[1];
    data = (unsigned short)simple_strtoul(buf, NULL, 16);
    i2c_write(sensor_client,MAX20086_ADDR,addr, 1, &data);

    debug("luozh: addr=0x%02x, data=0x%x\n", addr[0], data);
    return count;
}

static DEVICE_ATTR(sensor_register_ov10635, 0644, register_show_ov10635, register_store_ov10635);
static DEVICE_ATTR(sensor_register_max9288, 0644, register_show_max9288, register_store_max9288);
static DEVICE_ATTR(sensor_register_max96705, 0644, register_show_max96705, register_store_max96705);
static DEVICE_ATTR(register_addr, 0644, addr_show, addr_store);
//-------------------------------------------------------------
static DEVICE_ATTR(linux_register_max20088, 0644, register_show_max20088, register_store_max20088);
//static DEVICE_ATTR(linux_register_max20086, 0644, register_show_max20086, register_store_max20086);

#endif
static int max9288_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct max9288 *priv = NULL;
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct v4l2_subdev *subdev = NULL;
    int ret = 0;
    debug("--->>>>in\n");
    if (client->dev.of_node != NULL) {
		ssdd = devm_kzalloc(&client->dev, sizeof(*ssdd), GFP_KERNEL);
		if (ssdd == NULL)
			return -ENOMEM;
		client->dev.platform_data = ssdd;
	}

	if (ssdd == NULL) {
		max9288_err("MAX9288: missing platform data");
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct max9288), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	if (max9288_pinctrl_init(&client->dev) < 0) {
		max9288_err("%s, %d\n", __func__, __LINE__);
		return -EINVAL;
	}
	if (pmu1v8_power_init(&client->dev,client->dev.of_node)<0){
	    max9288_err("%s, %d\n", __func__, __LINE__);
        return -EINVAL;
	}
	ret = pinctrl_select_state(max9288_pctrl.pinctrl,max9288_pctrl.gpio_state_active);
    if (ret < 0) {
        max9288_err("%s, %d\n", __func__, __LINE__);
        return ret;
    }
    mdelay(500);

	v4l2_i2c_subdev_init(&priv->subdev, client, &max9288_subdev_ops);

	priv->fmt		= &max9288_colour_fmts[0];

	subdev = i2c_get_clientdata(client);

#ifdef sensor_register_debug
    sensor_client = client;
    ret = device_create_file(&client->dev, &dev_attr_sensor_register_ov10635);
    if (ret) {
        debug("luozh: register_test probe error....\n");
    }
    ret = device_create_file(&client->dev, &dev_attr_sensor_register_max96705);
    if (ret) {
        debug("luozh: register_test probe error....\n");
    }
    ret = device_create_file(&client->dev, &dev_attr_sensor_register_max9288);
    if (ret) {
        debug("luozh: register_test probe error....\n");
    }

    ret = device_create_file(&client->dev, &dev_attr_register_addr);
    if (ret) {
        debug("luozh: register_addr probe error....\n");
    }
    //for max20088 and max20086
    ret = device_create_file(&client->dev, &dev_attr_linux_register_max20088);
    if (ret) {
        debug("luozh: register_addr probe error....\n");
    }
    /*
    ret = device_create_file(&client->dev, &dev_attr_linux_register_max20086);
    if (ret) {
        debug("luozh: register_addr probe error....\n");
    }*/
#endif

	ret =  max9288_camera_init(client);
    if (ret < 0){
        max9288_err("max9288 camera init failed!\n");
        if(max9288_s_power(subdev,0)<0){
            max9288_err("power off failed!\n");
        }
        return ret;
    }
    priv->subdev.dev = &client->dev;

    debug("--->>>>out\n");
    return v4l2_async_register_subdev(&priv->subdev);
}

static int max9288_remove(struct i2c_client *client)
{
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);

	if (ssdd->free_bus != NULL)
		ssdd->free_bus(ssdd);

	return 0;
}

static const struct i2c_device_id max9288_id[] = {
	{ "max9288", 0},
	{ }
};
static const struct of_device_id max9288_camera_of_match[] = {
	{ .compatible = "maxim,max9288", },
	{},
};

static struct i2c_driver max9288_i2c_driver = {
	.driver = {
		.name = "max9288",
		.of_match_table = max9288_camera_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= max9288_probe,
	.remove		= max9288_remove,
	.id_table	= max9288_id,
};

module_param(is_testpattern, int, 0644);
MODULE_PARM_DESC(is_testpattern, "Whether the MAX9288 get test pattern data");
module_i2c_driver(max9288_i2c_driver);

MODULE_DESCRIPTION("MAXIM Camera driver");
MODULE_AUTHOR("Baoyin Zhang<baoyin.zhang@mediatek.com>, modified by Alex<fangyuan.liu@nio.com>");
MODULE_LICENSE("GPL v2");
