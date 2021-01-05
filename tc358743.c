/*		
* References (c = chapter, p = page):		
* REF_01 - Toshiba, TC358743XBG (H2C), Functional Specification, Rev 0.60
* REF_02 - Toshiba, TC358743XBG_HDMI-CSI_Tv11p_nm.xls
*/
#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/hdmi.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-of.h>
#include <media/camera_common.h>
#include <media/i2c/tc358743.h>

#include <dt-bindings/gpio/tegra-gpio.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/i2c-dev.h>
#include <linux/fs.h>

// #include <media/v4l2-chip-ident.h>
#include <media/tegra-v4l2-camera.h>
#include <media/soc_camera.h>

#include "tc358743_regs.h"

/* RGB ouput selection */
// #define TC358743_VOUT_RGB

static int debug = 1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

MODULE_DESCRIPTION("Toshiba TC358743 HDMI to CSI-2 bridge driver");
MODULE_AUTHOR("Ramakrishnan Muthukrishnan <ram@rkrishnan.org>");
MODULE_AUTHOR("Mikhail Khelik <mkhelik@cisco.com>");
MODULE_AUTHOR("Mats Randgaard <matrandg@cisco.com>");
MODULE_LICENSE("GPL");

/* mode */
enum {
	tc358743_MODE_1280X720,
	tc358743_MODE_1920X1080,
};

/* frame rate */
static const int tc358743_30fps[] = {
	30,
};
static const int tc358743_30_60fps[] = {
	30,
	50,
	60,
};
static const int tc358743_50fps[] = {
	50,
};

/* frame format */
static const struct camera_common_frmfmt tc358743_frmfmt[] = {
	{{1280,  720}, tc358743_30_60fps, 3, 0, tc358743_MODE_1280X720},//framme format set "tc358743_30_60fps, 3, 1,
	{{1920, 1080}, tc358743_30_60fps, 3, 0, tc358743_MODE_1920X1080},
};

// static const struct camera_common_colorfmt tc358743_color_fmts[] = {
// 	{
// 		MEDIA_BUS_FMT_SRGGB12_1X12,
// 		V4L2_COLORSPACE_SRGB,
// 		V4L2_PIX_FMT_SRGGB12,
// 	},
// 	{
// 		MEDIA_BUS_FMT_UYVY8_1X16,
// 		V4L2_COLORSPACE_SRGB,
// 		V4L2_PIX_FMT_UYVY,
// 	},
// };

#define EDID_NUM_BLOCKS_MAX 8
#define EDID_BLOCK_SIZE 128
static u8 edid[] = {

// #ifdef TC358743_VOUT_RGB
	0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,
	0x50,0x21,0x9C,0x27,0x00,0x00,0x00,0x00,
	0x19,0x12,0x01,0x03,0x80,0x00,0x00,0x78,
	0x0E,0x00,0xB2,0xA0,0x57,0x49,0x9B,0x26,
	0x10,0x48,0x4F,0x2F,0xCF,0x00,0x31,0x59,
	0x45,0x59,0x61,0x59,0x81,0x99,0x01,0x01,
	0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x3A,
	0x80,0x18,0x71,0x38,0x2D,0x40,0x58,0x2C,
	0x46,0x00,0x00,0x00,0x00,0x00,0x00,0x1E,
	0x00,0x00,0x00,0xFD,0x00,0x31,0x55,0x18,
	0x5E,0x11,0x00,0x0A,0x20,0x20,0x20,0x20,
	0x20,0x20,0x00,0x00,0x00,0xFC,0x00,0x54,
	0x6F,0x73,0x68,0x69,0x62,0x61,0x2D,0x48,
	0x32,0x43,0x0A,0x20,0x00,0x00,0x00,0xFD,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xc3,
	0x02,0x03,0x1a,0xc0,0x48,0xa2,0x10,0x04,
	0x02,0x01,0x21,0x14,0x13,0x23,0x09,0x07,
	0x07,0x65,0x03,0x0c,0x00,0x10,0x00,0xe2,
	0x00,0x2a,0x01,0x1d,0x00,0x80,0x51,0xd0,
	0x1c,0x20,0x40,0x80,0x35,0x00,0x00,0x00,
	0x00,0x00,0x00,0x1e,0x8c,0x0a,0xd0,0x8a,
	0x20,0xe0,0x2d,0x10,0x10,0x3e,0x96,0x00,
	0x00,0x00,0x00,0x00,0x00,0x18,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xd7,
// #else
	// 0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,
	// 0x52,0x62,0x88,0x88,0x00,0x88,0x88,0x88,
	// 0x1C,0x15,0x01,0x03,0x80,0x00,0x00,0x78,
	// 0x0A,0x0D,0xC9,0xA0,0x57,0x47,0x98,0x27,
	// 0x12,0x48,0x4C,0x00,0x00,0x00,0x01,0x01,
	// 0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	// 0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x3A,
	// 0x80,0xD0,0x72,0x38,0x2D,0x40,0x10,0x2C,
	// 0x45,0x80,0x66,0x4C,0x00,0x00,0x00,0x1E,
	// 0x01,0x1D,0x00,0xBC,0x52,0xD0,0x1E,0x20,
	// 0xB8,0x28,0x55,0x40,0x66,0x4C,0x00,0x00,
	// 0x00,0x1E,0x00,0x00,0x00,0xFC,0x00,0x54,
	// 0x6F,0x73,0x68,0x69,0x62,0x61,0x2D,0x48,
	// 0x32,0x43,0x0A,0x20,0x00,0x00,0x00,0xFD,
	// 0x00,0x14,0x78,0x01,0xFF,0x10,0x00,0x0A,
	// 0x20,0x20,0x20,0x20,0x20,0x20,0x00,0xBA,
	// 0x02,0x03,0x1A,0x71,0x47,0x9F,0x13,0x22,
	// 0x1F,0x02,0x11,0x1F,0x23,0x09,0x07,0x01,
	// 0x83,0x01,0x00,0x00,0x65,0x03,0x0C,0x00,
	// 0x10,0x00,0x01,0x1D,0x80,0x18,0x71,0x38,
	// 0x2D,0x40,0x58,0x2C,0x45,0x00,0x66,0x4C,
	// 0x00,0x00,0x00,0x1E,0x02,0x3A,0x80,0xD0,
	// 0x72,0x38,0x2D,0x40,0x10,0x2C,0x45,0x80,
	// 0x66,0x4C,0x00,0x00,0x00,0x1E,0x8C,0x0A,
	// 0xD0,0x8A,0x20,0xE0,0x2D,0x10,0x10,0x3E,
	// 0x96,0x00,0x66,0x4C,0x00,0x00,0x00,0x18,
	// 0x8C,0x0A,0xD0,0x90,0x20,0x40,0x31,0x20,
	// 0x0C,0x40,0x55,0x00,0x66,0x4C,0x00,0x00,
	// 0x00,0x18,0x00,0x00,0x00,0x00,0x00,0x00,
	// 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	// 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	// 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,
// #endif
};
/* Max transfer size done by I2C transfer functions */
#define MAX_XFER_SIZE  (EDID_NUM_BLOCKS_MAX * EDID_BLOCK_SIZE + 2)

static const struct v4l2_dv_timings_cap tc358743_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },
	/* Pixel clock from REF_01 p. 20. Min/max height/width are unknown */
	V4L2_INIT_BT_TIMINGS(1, 10000, 1, 10000, 0, 165000000,
			V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT,
			V4L2_DV_BT_CAP_PROGRESSIVE |
			V4L2_DV_BT_CAP_REDUCED_BLANKING |
			V4L2_DV_BT_CAP_CUSTOM)
};

struct tc358743_state {
	struct tc358743_platform_data pdata;
	// struct v4l2_of_bus_mipi_csi2 bus;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler hdl;
	struct i2c_client *i2c_client;
    struct regmap *regmap;
	/* CONFCTL is modified in ops and tc358743_hdmi_sys_int_handler */
	struct mutex confctl_mutex;

	/* controls */
	struct v4l2_ctrl *detect_tx_5v_ctrl;
	struct v4l2_ctrl *audio_sampling_rate_ctrl;
	struct v4l2_ctrl *audio_present_ctrl;

	/* work queues */
	struct workqueue_struct *work_queues;
	struct delayed_work delayed_work_enable_hotplug;

	/* edid  */
	u8 edid_blocks_written;

	/* used by i2c_wr() */
	u8 wr_data[MAX_XFER_SIZE];

	struct v4l2_dv_timings timings;
	u32 mbus_fmt_code;

	struct gpio_desc *reset_gpio;
};

static inline struct tc358743_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tc358743_state, sd);
}
/*
static char * sdo_bit_len [] = {
    [0b000] = "16bit (lower 8bit discarded)",
    [0b001] = "16bit (lower 8bit + 1 discarded)",
    [0b010] = "18bit (lower 6bit discarded)",
    [0b011] = "18bit (lower 6bit + 1 discarded)",
    [0b100] = "20bit (lower 4bit discarded)",
    [0b101] = "20bit (lower 4bit + 1 discarded)",
    [0b110] = "24bit no rounding",
    [0b111] = "Output OFF (Mute)",
};

static char * sdo_fmt [] = {
    [MASK_SDO_FMT_RIGHT] = "Right justified",
    [MASK_SDO_FMT_LEFT] = "Left justified",
    [MASK_SDO_FMT_I2S] = "I2S",
    [0b011] = "I2S",
};

static char * no_yes [] = {
    [0] = "No",
    [1] = "Yes",
};

static char * no_with [] = {
    [0] = "No",
    [1] = "With",
};

static char * off_on [] = {
    [0b000] = "Off",
    [0b001] = "On",
};

static char * audout_sel [] = {
    [0b00] = "CSI2-TX",
    [0b01] = "Reseved",
    [0b10] = "I2S",
    [0b11] = "TDM",
};
*/
/* --------------- I2C --------------- */
static int i2c_rd(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358743_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	u8 buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = n,
			.buf = values,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		v4l2_err(sd, "%s: #### reading register0x%x from0x%x failed\n",
				__func__, reg, client->addr);
		return -1;
	}
	//udelay(10);
	return 0;
}

static int i2c_wr(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358743_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	u8 *data = state->wr_data;
	int err, i;
	struct i2c_msg msg;

	if ((2 + n) > sizeof(state->wr_data)){
		v4l2_warn(sd, "i2c wr reg=%04x: len=%d is too big!\n",
			  reg, 2 + n);
		return -1;
	}

	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2 + n;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;

	for (i = 0; i < n; i++)
		data[2 + i] = values[i];

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		v4l2_err(sd, "%s: writing register0x%x from0x%x failed\n",
				__func__, reg, client->addr);
		return -1;
	}
	return 0;
}
static u8 i2c_rd8(struct v4l2_subdev *sd, u16 reg)
{
	u8 val;

	i2c_rd(sd, reg, &val, 1);
	//v4l2_info(sd, "i2c_rd8 val %d\n",val);

	return val;
}

static void i2c_wr8(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	i2c_wr(sd, reg, &val, 1);
}

static void i2c_wr8_and_or(struct v4l2_subdev *sd, u16 reg,	u8 mask, u8 val)
{
	i2c_wr8(sd, reg, (i2c_rd8(sd, reg) & mask) | val);
}

static u16 i2c_rd16(struct v4l2_subdev *sd, u16 reg)
{
	u16 val;
	int ret;
	//v4l2_info(sd, "Reading i2c_rd16\n");
	

	ret = i2c_rd(sd, reg, (u8 *)&val, 2);
	//v4l2_info(sd, "RET %d\n", ret);

	if (ret == -1) {
		v4l2_info(sd, "Reading i2c_rd16 failed %d and ret ==99 now !\n", ret);
		// Read failed
		return 99;  // TODO. Make this better!
	}
	
	return val;
}

static void i2c_wr16(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	i2c_wr(sd, reg, (u8 *)&val, 2);
}

static void i2c_wr16_and_or(struct v4l2_subdev *sd, u16 reg, u16 mask, u16 val)
{
	i2c_wr16(sd, reg, (i2c_rd16(sd, reg) & mask) | val);
}

static u32 i2c_rd32(struct v4l2_subdev *sd, u16 reg)
{
	u32 val;

	i2c_rd(sd, reg, (u8 *)&val, 4);

	return val;
}

static void i2c_wr32(struct v4l2_subdev *sd, u16 reg, u32 val)
{
	i2c_wr(sd, reg, (u8 *)&val, 4);
}
/* --------------- STATUS --------------- */

static inline bool is_hdmi(struct v4l2_subdev *sd)
{
	return i2c_rd8(sd, SYS_STATUS) & MASK_S_HDMI;
}

static inline bool tx_5v_power_present(struct v4l2_subdev *sd)
{
	return i2c_rd8(sd, SYS_STATUS) & MASK_S_DDC5V;
}

static inline bool no_signal(struct v4l2_subdev *sd)
{
	return !(i2c_rd8(sd, SYS_STATUS) & MASK_S_TMDS);
}

static inline bool no_sync(struct v4l2_subdev *sd)
{
	return !(i2c_rd8(sd, SYS_STATUS) & MASK_S_SYNC);
}

static inline bool audio_present(struct v4l2_subdev *sd)
{
	return i2c_rd8(sd, AU_STATUS0) & MASK_S_A_SAMPLE;
}

static int get_audio_sampling_rate(struct v4l2_subdev *sd)
{
	static const int code_to_rate[] = {
		44100, 0, 48000, 32000, 22050, 384000, 24000, 352800,
		88200, 768000, 96000, 705600, 176400, 0, 192000, 0
	};

	/* Register FS_SET is not cleared when the cable is disconnected */
	if (no_signal(sd))
		return 0;

	return code_to_rate[i2c_rd8(sd, FS_SET) & MASK_FS];
}

static unsigned tc358743_num_csi_lanes_in_use(struct v4l2_subdev *sd)
{
	return ((i2c_rd32(sd, CSI_CONTROL) & MASK_NOL) >> 1) + 1;
}

/* --------------- TIMINGS --------------- */

static inline unsigned fps(const struct v4l2_bt_timings *t)
{
	if (!V4L2_DV_BT_FRAME_HEIGHT(t) || !V4L2_DV_BT_FRAME_WIDTH(t))
		return 0;

	return DIV_ROUND_CLOSEST((unsigned)t->pixelclock,
			V4L2_DV_BT_FRAME_HEIGHT(t) * V4L2_DV_BT_FRAME_WIDTH(t));
}

static int tc358743_get_detected_timings(struct v4l2_subdev *sd,
				                         struct v4l2_dv_timings *timings)
{
	struct v4l2_bt_timings *bt = &timings->bt;
	unsigned width, height, frame_width, frame_height, frame_interval, fps;

	memset(timings, 0, sizeof(struct v4l2_dv_timings));

	if (no_signal(sd)) {
		v4l2_info(sd, "%s: no valid signal\n", __func__);
		return -ENOLINK;
	}
	if (no_sync(sd)) {
		v4l2_info(sd, "%s: no sync on signal\n", __func__);
		return -ENOLCK;
	}

	timings->type = V4L2_DV_BT_656_1120;
	bt->interlaced = i2c_rd8(sd, VI_STATUS1) & MASK_S_V_INTERLACE ?
		V4L2_DV_INTERLACED : V4L2_DV_PROGRESSIVE;

	width = ((i2c_rd8(sd, DE_WIDTH_H_HI) & 0x1f) << 8) +
		i2c_rd8(sd, DE_WIDTH_H_LO);
	height = ((i2c_rd8(sd, DE_WIDTH_V_HI) & 0x1f) << 8) +
		i2c_rd8(sd, DE_WIDTH_V_LO);
	frame_width = ((i2c_rd8(sd, H_SIZE_HI) & 0x1f) << 8) +
		i2c_rd8(sd, H_SIZE_LO);
	frame_height = (((i2c_rd8(sd, V_SIZE_HI) & 0x3f) << 8) +
		i2c_rd8(sd, V_SIZE_LO)) / 2;
	/* frame interval in milliseconds * 10
	 * Require SYS_FREQ0 and SYS_FREQ1 are precisely set */
	frame_interval = ((i2c_rd8(sd, FV_CNT_HI) & 0x3) << 8) +
		i2c_rd8(sd, FV_CNT_LO);
	fps = (frame_interval > 0) ?
		DIV_ROUND_CLOSEST(10000, frame_interval) : 0;

	bt->width = width;
	bt->height = height;
	bt->vsync = frame_height - height;
	bt->hsync = frame_width - width;
	bt->pixelclock = frame_width * frame_height * fps;
	if (bt->interlaced == V4L2_DV_INTERLACED) {
		bt->height *= 2;
		bt->il_vsync = bt->vsync + 1;
		bt->pixelclock /= 2;
	}
	v4l2_info(sd,"%d:%s: width %d heigh %d interlaced %d\n",__LINE__,__FUNCTION__,		
	        bt->width,		
	        bt->height,		
	        bt->interlaced);
	return 0;
}
/* --------------- HOTPLUG / HDCP / EDID --------------- */

static void tc358743_delayed_work_enable_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct tc358743_state *state = container_of(dwork,
			struct tc358743_state, delayed_work_enable_hotplug);
	struct v4l2_subdev *sd = &state->sd;

	v4l2_info(sd, "%s:\n", __func__);

    //i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_CTL0, MASK_HPD_CTL0);
	i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_OUT0, MASK_HPD_OUT0);//
	// /*hainh 
    //i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_CTL0, MASK_HPD_CTL0);
	// */
}

static void tc358743_set_hdmi_hdcp(struct v4l2_subdev *sd, bool enable)
{
	v4l2_info(sd, "%s: %s\n", __func__, enable ?
				"enable" : "disable");

	i2c_wr8_and_or(sd, HDCP_REG1,
			~(MASK_AUTH_UNAUTH_SEL | MASK_AUTH_UNAUTH),
			MASK_AUTH_UNAUTH_SEL_16_FRAMES | MASK_AUTH_UNAUTH_AUTO);

	i2c_wr8_and_or(sd, HDCP_REG2, ~MASK_AUTO_P3_RESET,
			SET_AUTO_P3_RESET_FRAMES(0x0f));

	/* HDCP is disabled by configuring the receiver as HDCP repeater. The
	 * repeater mode require software support to work, so HDCP
	 * authentication will fail. */
	i2c_wr8_and_or(sd, HDCP_REG3, ~KEY_RD_CMD, enable ? KEY_RD_CMD : 0);
	i2c_wr8_and_or(sd, HDCP_MODE, ~(MASK_AUTO_CLR | MASK_MODE_RST_TN),
			enable ?  (MASK_AUTO_CLR | MASK_MODE_RST_TN) : 0);

	/* Apple MacBook Pro gen.8 has a bug that makes it freeze every fifth
	 * second when HDCP is disabled, but the MAX_EXCED bit is handled
	 * correctly and HDCP is disabled on the HDMI output. */
	i2c_wr8_and_or(sd, BSTATUS1, ~MASK_MAX_EXCED,
			enable ? 0 : MASK_MAX_EXCED);
	i2c_wr8_and_or(sd, BCAPS, ~(MASK_REPEATER | MASK_READY),
			enable ? 0 : MASK_REPEATER | MASK_READY);
}

static void tc358743_disable_edid(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	v4l2_info(sd, "%s:\n", __func__);

	cancel_delayed_work_sync(&state->delayed_work_enable_hotplug);

	/* DDC access to EDID is also disabled when hotplug is disabled. See
	 * register DDC_CTL */
	i2c_wr8_and_or(sd, HPD_CTL, ~MASK_HPD_OUT0,0x0);
}



static void tc358743_erase_bksv(struct v4l2_subdev *sd)
{
	int i;

	for (i = 0; i < 5; i++)
		i2c_wr8(sd, BKSV + i, 0);
}

/* --------------- AVI infoframe --------------- */

static void print_avi_infoframe(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	union hdmi_infoframe frame;
	u8 buffer[HDMI_INFOFRAME_SIZE(AVI)];

	if (!is_hdmi(sd)) {
		v4l2_info(sd, "DVI-D signal - AVI infoframe not supported\n");
		return;
	}

	i2c_rd(sd, PK_AVI_0HEAD, buffer, HDMI_INFOFRAME_SIZE(AVI));

	if (hdmi_infoframe_unpack(&frame, buffer) < 0) {
		v4l2_err(sd, "%s: unpack of AVI infoframe failed\n", __func__);
		return;
	}

	hdmi_infoframe_log(KERN_INFO, dev, &frame);
}

/* --------------- CTRLS --------------- */

static int tc358743_s_ctrl_detect_tx_5v(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	return v4l2_ctrl_s_ctrl(state->detect_tx_5v_ctrl,
			tx_5v_power_present(sd));
}

static int tc358743_s_ctrl_audio_sampling_rate(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	return v4l2_ctrl_s_ctrl(state->audio_sampling_rate_ctrl,
			get_audio_sampling_rate(sd));
}

static int tc358743_s_ctrl_audio_present(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	return v4l2_ctrl_s_ctrl(state->audio_present_ctrl, audio_present(sd));
}

static int tc358743_update_controls(struct v4l2_subdev *sd)
{
	int ret = 0;

	ret |= tc358743_s_ctrl_detect_tx_5v(sd);
	ret |= tc358743_s_ctrl_audio_sampling_rate(sd);
	ret |= tc358743_s_ctrl_audio_present(sd);

	return ret;
}
static unsigned tc358743_num_csi_lanes_needed(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct v4l2_bt_timings *bt = &state->timings.bt;
	struct tc358743_platform_data *pdata = &state->pdata;
	u32 bits_pr_pixel =
		(state->mbus_fmt_code == MEDIA_BUS_FMT_UYVY8_1X16) ?  16 : 24;
	u32 bps = bt->width * bt->height * fps(bt) * bits_pr_pixel;
	u32 bps_pr_lane = (pdata->refclk_hz / pdata->pll_prd) * pdata->pll_fbd;

	return DIV_ROUND_UP(bps, bps_pr_lane);
}
static int tc358743_get_edid(struct v4l2_subdev *sd){
	//static int i2c_rd(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
	//int i = 0;
	u8 edid_read[256];
	int result = 0;
	u32 n = sizeof(edid_read);

	result = i2c_rd(sd,EDID_RAM,edid_read,n);
	v4l2_info(sd, "%s i2c_rd return %d\r\n",__func__,result);
	// for(i = 0; i< n;i++)
	// 	printk("%02x ",edid_read[i]);
	v4l2_info(sd, "%s done\r\n",__func__);
	return 0;
}
static int tc358743_log_status(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct v4l2_dv_timings timings;
	uint8_t hdmi_sys_status =  i2c_rd8(sd, SYS_STATUS);
	uint16_t sysctl = i2c_rd16(sd, SYSCTL);
	u8 vi_status3 =  i2c_rd8(sd, VI_STATUS3);
	const int deep_color_mode[4] = { 8, 10, 12, 16 };
	static const char * const input_color_space[] = {
		"RGB", "YCbCr 601", "Adobe RGB", "YCbCr 709", "NA (4)",
		"xvYCC 601", "NA(6)", "xvYCC 709", "NA(8)", "sYCC601",
		"NA(10)", "NA(11)", "NA(12)", "Adobe YCC 601"};
	tc358743_get_edid(sd);
	v4l2_info(sd, "-----Chip status-----\n");
	v4l2_info(sd, "Chip ID:0x%02x\n",
			(i2c_rd16(sd, CHIPID) & MASK_CHIPID) >> 8);
	v4l2_info(sd, "Chip revision:0x%02x\n",
			i2c_rd16(sd, CHIPID) & MASK_REVID);
	v4l2_info(sd, "Reset: IR: %d, CEC: %d, CSI TX: %d, HDMI: %d\n",
			!!(sysctl & MASK_IRRST),
			!!(sysctl & MASK_CECRST),
			!!(sysctl & MASK_CTXRST),
			!!(sysctl & MASK_HDMIRST));
	v4l2_info(sd, "Sleep mode: %s\n", sysctl & MASK_SLEEP ? "on" : "off");
	v4l2_info(sd, "Cable detected (+5V power): %s\n",
			hdmi_sys_status & MASK_S_DDC5V ? "yes" : "no");
	v4l2_info(sd, "DDC lines enabled: %s\n",
			(i2c_rd8(sd, EDID_MODE) & MASK_EDID_MODE_E_DDC) ?
			"yes" : "no");
	v4l2_info(sd, "Hotplug enabled: %s\n",
			(i2c_rd8(sd, HPD_CTL) & MASK_HPD_OUT0) ?
			"yes" : "no");
	v4l2_info(sd, "Hotplug ctl: %s\n",
			(i2c_rd8(sd, HPD_CTL) & MASK_HPD_CTL0) ?
			"yes" : "no");
	v4l2_info(sd, "CEC enabled: %s\n",
			(i2c_rd16(sd, CECEN) & MASK_CECEN) ?  "yes" : "no");
	v4l2_info(sd, "-----Signal status-----\n");
	v4l2_info(sd, "TMDS signal detected: %s\n",
			hdmi_sys_status & MASK_S_TMDS ? "yes" : "no");
	v4l2_info(sd, "Stable sync signal: %s\n",
			hdmi_sys_status & MASK_S_SYNC ? "yes" : "no");
	v4l2_info(sd, "PHY PLL locked: %s\n",
			hdmi_sys_status & MASK_S_PHY_PLL ? "yes" : "no");
	v4l2_info(sd, "PHY DE detected: %s\n",
			hdmi_sys_status & MASK_S_PHY_SCDT ? "yes" : "no");

	if (tc358743_get_detected_timings(sd, &timings)) {
		v4l2_info(sd, "No video detected\n");
	} else {
		v4l2_print_dv_timings(sd->name, "Detected format: ", &timings, true);
	}
	
	v4l2_print_dv_timings(sd->name, "Configured format: ", &state->timings, true);

	v4l2_info(sd, "-----CSI-TX status-----\n");
	v4l2_info(sd, "Lanes needed: %d\n",
			tc358743_num_csi_lanes_needed(sd));
	v4l2_info(sd, "Lanes in use: %d\n",
			tc358743_num_csi_lanes_in_use(sd));
	v4l2_info(sd, "Waiting for particular sync signal: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & MASK_S_WSYNC) ?
			"yes" : "no");
	v4l2_info(sd, "Transmit mode: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & MASK_S_TXACT) ?
			"yes" : "no");//add register MASK_S_TXACT 
	v4l2_info(sd, "Receive mode: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & MASK_S_RXACT) ?
			"yes" : "no");
	v4l2_info(sd, "Stopped: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & MASK_S_HLT) ?
			"yes" : "no");
	v4l2_info(sd, "Color space: %s\n",
			state->mbus_fmt_code == MEDIA_BUS_FMT_UYVY8_1X16 ?
			"YCbCr 422 16-bit" :
			state->mbus_fmt_code == MEDIA_BUS_FMT_RGB888_1X24 ?
			"RGB 888 24-bit" : "Unsupported");

	v4l2_info(sd, "-----%s status-----\n", is_hdmi(sd) ? "HDMI" : "DVI-D");
	v4l2_info(sd, "HDCP encrypted content: %s\n",
			hdmi_sys_status & MASK_S_HDCP ? "yes" : "no");
	v4l2_info(sd, "Input color space: %s %s range\n",
			input_color_space[(vi_status3 & MASK_S_V_COLOR) >> 1],
			(vi_status3 & MASK_LIMITED) ? "limited" : "full");
	if (!is_hdmi(sd))
		return 0;
	v4l2_info(sd, "AV Mute: %s\n", hdmi_sys_status & MASK_S_AVMUTE ? "on" :
			"off");
	v4l2_info(sd, "Deep color mode: %d-bits per channel\n",
			deep_color_mode[(i2c_rd8(sd, VI_STATUS1) &
				MASK_S_DEEPCOLOR) >> 2]);
	print_avi_infoframe(sd);

	return 0;
}

/* --------------- INIT --------------- */

static void tc358743_reset_phy(struct v4l2_subdev *sd)
{
	v4l2_info(sd, "%s:\n", __func__);

	i2c_wr8_and_or(sd, PHY_RST, ~MASK_RESET_CTRL, 0);
	i2c_wr8_and_or(sd, PHY_RST, ~MASK_RESET_CTRL, MASK_RESET_CTRL);
}

static void tc358743_reset(struct v4l2_subdev *sd, uint16_t mask)
{
	u16 sysctl = i2c_rd16(sd, SYSCTL);

	i2c_wr16(sd, SYSCTL, sysctl | mask);
	i2c_wr16(sd, SYSCTL, sysctl & ~mask);
}

static inline void tc358743_sleep_mode(struct v4l2_subdev *sd, bool enable)
{
	i2c_wr16_and_or(sd, SYSCTL, ~MASK_SLEEP, enable ? MASK_SLEEP : 0);
}

static inline void enable_stream(struct v4l2_subdev *sd, bool enable)
{
	struct tc358743_state *state = to_state(sd);

	v4l2_info(sd, "%s: %sable\n",	__func__, enable ? "en" : "dis");

	if (enable) {
		/* It is critical for CSI receiver to see lane transition
		 * LP11->HS. Set to non-continuous mode to enable clock lane
		 * LP11 state. */
		i2c_wr32(sd, TXOPTIONCNTRL, 0);
		/* Set to continuous mode to trigger LP11->HS transition */
		i2c_wr32(sd, TXOPTIONCNTRL, MASK_CONTCLKMODE);
		/* Unmute video */
		i2c_wr8(sd, VI_MUTE, MASK_AUTO_MUTE);
	} else {
		/* Mute video so that all data lanes go to LSP11 state.
		 * No data is output to CSI Tx block. */
		i2c_wr8(sd, VI_MUTE, MASK_AUTO_MUTE | MASK_VI_MUTE);
	}

	mutex_lock(&state->confctl_mutex);
	i2c_wr16_and_or(sd, CONFCTL, ~(MASK_VBUFEN | MASK_ABUFEN),
			enable ? (MASK_VBUFEN | MASK_ABUFEN) :0x0);
	mutex_unlock(&state->confctl_mutex);
	v4l2_info(sd,"%d:%s: end\n",__LINE__,__FUNCTION__);		
	if (enable)	{		
        tc358743_log_status(sd);		
    }
}
static void tc358743_set_pll(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct tc358743_platform_data *pdata = &state->pdata;
	u16 pllctl0 = i2c_rd16(sd, PLLCTL0);
	u16 pllctl1 = i2c_rd16(sd, PLLCTL1);
	u16 pllctl0_new = SET_PLL_PRD(pdata->pll_prd) |
		SET_PLL_FBD(pdata->pll_fbd);
	u32 hsck = (pdata->refclk_hz / pdata->pll_prd) * pdata->pll_fbd;

	v4l2_info(sd, "%s:\n", __func__);

	/* Only rewrite when needed (new value or disabled), since rewriting
	 * triggers another format change event. */
	if ((pllctl0 != pllctl0_new) || ((pllctl1 & MASK_PLL_EN) == 0)) {
		u16 pll_frs;

		if (hsck > 500000000)
			pll_frs =0x0;
		else if (hsck > 250000000)
			pll_frs =0x1;
		else if (hsck > 125000000)
			pll_frs =0x2;
		else
			pll_frs =0x3;
		v4l2_info(sd, "%s: updating PLL clock\n", __func__);
		tc358743_sleep_mode(sd, true);
		i2c_wr16(sd, PLLCTL0, pllctl0_new);
		i2c_wr16_and_or(sd, PLLCTL1,
				~(MASK_PLL_FRS | MASK_RESETB | MASK_PLL_EN),
				(SET_PLL_FRS(pll_frs) | MASK_RESETB |
				 MASK_PLL_EN));
		udelay(10); /* REF_02, Sheet "Source HDMI" */
		i2c_wr16_and_or(sd, PLLCTL1, ~MASK_CKEN, MASK_CKEN);
		tc358743_sleep_mode(sd, false);
	}
}

static void tc358743_set_ref_clk(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct tc358743_platform_data *pdata = &state->pdata;
	u32 sys_freq;
	u32 lockdet_ref;
	u16 fh_min;
	u16 fh_max;

	sys_freq = pdata->refclk_hz / 10000;
	i2c_wr8(sd, SYS_FREQ0, sys_freq & 0x00ff);
	i2c_wr8(sd, SYS_FREQ1, (sys_freq & 0xff00) >> 8);

	i2c_wr8_and_or(sd, PHY_CTL0, ~MASK_PHY_SYSCLK_IND,
			(pdata->refclk_hz == 42000000) ? MASK_PHY_SYSCLK_IND :0x0);

	fh_min = pdata->refclk_hz / 100000;
	i2c_wr8(sd, FH_MIN0, fh_min & 0x00ff);
	i2c_wr8(sd, FH_MIN1, (fh_min & 0xff00) >> 8);

	fh_max = (fh_min * 66) / 10;
	i2c_wr8(sd, FH_MAX0, fh_max & 0x00ff);
	i2c_wr8(sd, FH_MAX1, (fh_max & 0xff00) >> 8);

	lockdet_ref = pdata->refclk_hz / 100;
	i2c_wr8(sd, LOCKDET_REF0, lockdet_ref & 0x0000ff);
	i2c_wr8(sd, LOCKDET_REF1, (lockdet_ref & 0x00ff00) >> 8);
	i2c_wr8(sd, LOCKDET_REF2, (lockdet_ref & 0x0f0000) >> 16);

	i2c_wr8_and_or(sd, NCO_F0_MOD, ~MASK_NCO_F0_MOD,
			(pdata->refclk_hz == 27000000) ? MASK_NCO_F0_MOD_27MHZ :0x0);
}
static void tc358743_set_csi_color_space(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	switch (state->mbus_fmt_code) {
		case MEDIA_BUS_FMT_UYVY8_1X16:
			v4l2_info(sd, "%s: YCbCr 422 16-bit\n", __func__);
			i2c_wr8_and_or(sd, VOUT_SET2,
					~(MASK_SEL422 | MASK_VOUT_422FIL_100) & 0xff,
					MASK_SEL422 | MASK_VOUT_422FIL_100);
			i2c_wr8_and_or(sd, VI_REP, ~MASK_VOUT_COLOR_SEL & 0xff,
					MASK_VOUT_COLOR_601_YCBCR_LIMITED);
			mutex_lock(&state->confctl_mutex);
			i2c_wr16_and_or(sd, CONFCTL, ~MASK_YCBCRFMT,
					MASK_YCBCRFMT_422_8_BIT);
			mutex_unlock(&state->confctl_mutex);
			break;
		// case MEDIA_BUS_FMT_RGB888_1X24:
		// 	v4l2_info(sd, "%s: RGB 888 24-bit\n", __func__);
		// 	i2c_wr8_and_or(sd, VOUT_SET2,
		// 			~(MASK_SEL422 | MASK_VOUT_422FIL_100) & 0xff,
		// 			0x00);
		// 	i2c_wr8_and_or(sd, VI_REP, ~MASK_VOUT_COLOR_SEL & 0xff,
		// 			MASK_VOUT_COLOR_RGB_FULL);
		// 	mutex_lock(&state->confctl_mutex);
		// 	i2c_wr16_and_or(sd, CONFCTL, ~MASK_YCBCRFMT, 0);
		// 	mutex_unlock(&state->confctl_mutex);
		// 	break;
		default:
			v4l2_dbg(2, debug, sd, "%s: Unsupported format code 0x%x\n",
				__func__, state->mbus_fmt_code);
			break;
	}

	// enable_stream(sd, true);  // Just put here for testing
}



static void tc358743_set_csi(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct tc358743_platform_data *pdata = &state->pdata;
	unsigned lanes = tc358743_num_csi_lanes_needed(sd);

	v4l2_info(sd, "%s:\n", __func__);

	tc358743_reset(sd, MASK_CTXRST);

	if (lanes < 1)
		i2c_wr32(sd, CLW_CNTRL, MASK_CLW_LANEDISABLE);
	if (lanes < 1)
		i2c_wr32(sd, D0W_CNTRL, MASK_D0W_LANEDISABLE);
	if (lanes < 2)
		i2c_wr32(sd, D1W_CNTRL, MASK_D1W_LANEDISABLE);
	if (lanes < 3)
		i2c_wr32(sd, D2W_CNTRL, MASK_D2W_LANEDISABLE);
	if (lanes < 4)
		i2c_wr32(sd, D3W_CNTRL, MASK_D3W_LANEDISABLE);

	i2c_wr32(sd, LINEINITCNT, pdata->lineinitcnt);
	i2c_wr32(sd, LPTXTIMECNT, pdata->lptxtimecnt);
	i2c_wr32(sd, TCLK_HEADERCNT, pdata->tclk_headercnt);
	i2c_wr32(sd, TCLK_TRAILCNT, pdata->tclk_trailcnt);
	i2c_wr32(sd, THS_HEADERCNT, pdata->ths_headercnt);
	i2c_wr32(sd, TWAKEUP, pdata->twakeup);
	i2c_wr32(sd, TCLK_POSTCNT, pdata->tclk_postcnt);
	i2c_wr32(sd, THS_TRAILCNT, pdata->ths_trailcnt);
	i2c_wr32(sd, HSTXVREGCNT, pdata->hstxvregcnt);

	i2c_wr32(sd, HSTXVREGEN,
			((lanes > 0) ? MASK_CLM_HSTXVREGEN :0x0) |
			((lanes > 0) ? MASK_D0M_HSTXVREGEN :0x0) |
			((lanes > 1) ? MASK_D1M_HSTXVREGEN :0x0) |
			((lanes > 2) ? MASK_D2M_HSTXVREGEN :0x0) |
			((lanes > 3) ? MASK_D3M_HSTXVREGEN :0x0));

	i2c_wr32(sd, TXOPTIONCNTRL, (pdata->endpoint.bus.mipi_csi2.flags &
		 V4L2_MBUS_CSI2_CONTINUOUS_CLOCK) ? MASK_CONTCLKMODE : 0);
	i2c_wr32(sd, STARTCNTRL, MASK_START);
	i2c_wr32(sd, CSI_START, MASK_STRT);

	i2c_wr32(sd, CSI_CONFW, MASK_MODE_SET |
			MASK_ADDRESS_CSI_CONTROL |
			MASK_CSI_MODE |
			MASK_TXHSMD |
			((lanes == 4) ? MASK_NOL_4 :
			 (lanes == 3) ? MASK_NOL_3 :
			 (lanes == 2) ? MASK_NOL_2 : MASK_NOL_1));

	i2c_wr32(sd, CSI_CONFW, MASK_MODE_SET |
			MASK_ADDRESS_CSI_ERR_INTENA | MASK_TXBRK | MASK_QUNK |
			MASK_WCER | MASK_INER);

	i2c_wr32(sd, CSI_CONFW, MASK_MODE_CLEAR |
			MASK_ADDRESS_CSI_ERR_HALT | MASK_TXBRK | MASK_QUNK);

	i2c_wr32(sd, CSI_CONFW, MASK_MODE_SET |
			MASK_ADDRESS_CSI_INT_ENA | MASK_INTER);
}

static void tc358743_set_hdmi_phy(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct tc358743_platform_data *pdata = &state->pdata;

	/* Default settings from REF_02, sheet "Source HDMI"
	 * and custom settings as platform data */
	// turn of physics
	i2c_wr8_and_or(sd, PHY_EN, ~MASK_ENABLE_PHY,0x0);
	i2c_wr8(sd, PHY_CTL1, SET_PHY_AUTO_RST1_US(1600) |
			SET_FREQ_RANGE_MODE_CYCLES(1));
	i2c_wr8_and_or(sd, PHY_CTL2, ~MASK_PHY_AUTO_RSTn,
			(pdata->hdmi_phy_auto_reset_tmds_detected ?
			 MASK_PHY_AUTO_RST2 : 0) |
			(pdata->hdmi_phy_auto_reset_tmds_in_range ?
			 MASK_PHY_AUTO_RST3 : 0) |
			(pdata->hdmi_phy_auto_reset_tmds_valid ?
			 MASK_PHY_AUTO_RST4 : 0));
	i2c_wr8(sd, PHY_BIAS,0x40);
	i2c_wr8(sd, PHY_CSQ, SET_CSQ_CNT_LEVEL(0x0a));
	i2c_wr8(sd, AVM_CTL, 45);
	i2c_wr8_and_or(sd, HDMI_DET, ~MASK_HDMI_DET_V,
			pdata->hdmi_detection_delay << 4);

	i2c_wr8_and_or(sd, HV_RST, ~(MASK_H_PI_RST | MASK_V_PI_RST),
			(pdata->hdmi_phy_auto_reset_hsync_out_of_range ?
			 MASK_H_PI_RST : 0) |
			(pdata->hdmi_phy_auto_reset_vsync_out_of_range ?
			 MASK_V_PI_RST : 0));
	// turn on physics
	i2c_wr8_and_or(sd, PHY_EN, ~MASK_ENABLE_PHY, MASK_ENABLE_PHY);
}

static void tc358743_set_hdmi_audio(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	/* Default settings from REF_02, sheet "Source HDMI" */
	i2c_wr8(sd, FORCE_MUTE,0x00);
	i2c_wr8(sd, AUTO_CMD0, MASK_AUTO_MUTE7 | MASK_AUTO_MUTE6 |
			MASK_AUTO_MUTE5 | MASK_AUTO_MUTE4 |
			MASK_AUTO_MUTE1 | MASK_AUTO_MUTE0);
	i2c_wr8(sd, AUTO_CMD1, MASK_AUTO_MUTE9);
	i2c_wr8(sd, AUTO_CMD2, MASK_AUTO_PLAY3 | MASK_AUTO_PLAY2);
	i2c_wr8(sd, BUFINIT_START, SET_BUFINIT_START_MS(500));
	i2c_wr8(sd, FS_MUTE,0x00);
	i2c_wr8(sd, FS_IMODE, MASK_NLPCM_SMODE | MASK_FS_SMODE);
	i2c_wr8(sd, ACR_MODE, MASK_CTS_MODE);
	i2c_wr8(sd, ACR_MDF0, MASK_ACR_L2MDF_1976_PPM | MASK_ACR_L1MDF_976_PPM);
	i2c_wr8(sd, ACR_MDF1, MASK_ACR_L3MDF_3906_PPM);
	i2c_wr8(sd, SDO_MODE1, MASK_SDO_FMT_I2S);
	i2c_wr8(sd, DIV_MODE, SET_DIV_DLY_MS(100));

	mutex_lock(&state->confctl_mutex);
	i2c_wr16_and_or(sd, CONFCTL,0xffff, MASK_AUDCHNUM_2 |
			MASK_AUDOUTSEL_I2S | MASK_AUTOINDEX);
	mutex_unlock(&state->confctl_mutex);
}

static void tc358743_set_hdmi_info_frame_mode(struct v4l2_subdev *sd)
{
	/* Default settings from REF_02, sheet "Source HDMI" */
	i2c_wr8(sd, PK_INT_MODE, MASK_ISRC2_INT_MODE | MASK_ISRC_INT_MODE |
			MASK_ACP_INT_MODE | MASK_VS_INT_MODE |
			MASK_SPD_INT_MODE | MASK_MS_INT_MODE |
			MASK_AUD_INT_MODE | MASK_AVI_INT_MODE);
	i2c_wr8(sd, NO_PKT_LIMIT,0x2c);
	i2c_wr8(sd, NO_PKT_CLR,0x53);
	i2c_wr8(sd, ERR_PK_LIMIT,0x01);
	i2c_wr8(sd, NO_PKT_LIMIT2,0x30);
	i2c_wr8(sd, NO_GDB_LIMIT,0x10);
}

static void tc358743_initial_setup(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct tc358743_platform_data *pdata = &state->pdata;

	/* CEC and IR are not supported by this driver */
	i2c_wr16_and_or(sd, SYSCTL, ~(MASK_CECRST | MASK_IRRST),
			(MASK_CECRST | MASK_IRRST));

	tc358743_reset(sd, MASK_CTXRST | MASK_HDMIRST);
	tc358743_sleep_mode(sd, false);

	i2c_wr16(sd, FIFOCTL, pdata->fifo_level);

	tc358743_set_ref_clk(sd);

	i2c_wr8_and_or(sd, DDC_CTL, ~MASK_DDC5V_MODE, 
	               pdata->ddc5v_delay & MASK_DDC5V_MODE);

	i2c_wr8_and_or(sd, EDID_MODE, ~MASK_EDID_MODE, MASK_EDID_MODE_E_DDC);

	tc358743_set_hdmi_phy(sd);
	tc358743_set_hdmi_hdcp(sd, pdata->enable_hdcp);
	tc358743_set_hdmi_audio(sd);
	tc358743_set_hdmi_info_frame_mode(sd);

	/* All CE and IT formats are detected as RGB full range in DVI mode */
	i2c_wr8_and_or(sd, VI_MODE, ~MASK_RGB_DVI, 0);

	i2c_wr8_and_or(sd, VOUT_SET2, ~MASK_VOUTCOLORMODE,
			MASK_VOUTCOLORMODE_AUTO);
	i2c_wr8(sd, VOUT_SET3, MASK_VOUT_EXTCNT);
}

/* --------------- IRQ --------------- */

static void tc358743_format_change(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);
	struct v4l2_dv_timings timings;
	const struct v4l2_event tc358743_ev_fmt = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	v4l2_info(sd, "%s: Format changed\n", __func__);

	if (tc358743_get_detected_timings(sd, &timings)) {
		enable_stream(sd, false);

		v4l2_info(sd, "%s: Format changed. No signal\n", __func__);
	} else {
		if (!v4l2_match_dv_timings(&state->timings, &timings, 0, false))
			enable_stream(sd, false);

		v4l2_print_dv_timings(sd->name,
				"tc358743_format_change: Format change`d. New format: ",	&timings, false);
	}

	if (sd->devnode)
		v4l2_subdev_notify_event(sd, &tc358743_ev_fmt);
}

static void tc358743_init_interrupts(struct v4l2_subdev *sd)
{
	u16 i;

	/* clear interrupt status registers */
	for (i = SYS_INT; i <= KEY_INT; i++)
		i2c_wr8(sd, i,0xff);

	i2c_wr16(sd, INTSTATUS,0xffff);
}

static void tc358743_enable_interrupts(struct v4l2_subdev *sd, bool cable_connected)
{
	v4l2_info(sd, "%s: cable connected = %d\n", __func__, cable_connected);

	if (cable_connected) {
		i2c_wr8(sd, SYS_INTM, ~(MASK_M_DDC | MASK_M_DVI_DET |
					MASK_M_HDMI_DET) & 0xff);
		i2c_wr8(sd, CLK_INTM, ~MASK_M_IN_DE_CHG);
		i2c_wr8(sd, CBIT_INTM, ~(MASK_M_CBIT_FS | MASK_M_AF_LOCK |
					MASK_M_AF_UNLOCK) & 0xff);
		i2c_wr8(sd, AUDIO_INTM, ~MASK_M_BUFINIT_END);
		i2c_wr8(sd, MISC_INTM, ~MASK_M_SYNC_CHG);
	} else {
		i2c_wr8(sd, SYS_INTM, ~MASK_M_DDC & 0xff);
		i2c_wr8(sd, CLK_INTM,0xff);
		i2c_wr8(sd, CBIT_INTM,0xff);
		i2c_wr8(sd, AUDIO_INTM,0xff);
		i2c_wr8(sd, MISC_INTM,0xff);
	}
}

static void tc358743_hdmi_audio_int_handler(struct v4l2_subdev *sd,	bool *handled)
{
	u8 audio_int_mask = i2c_rd8(sd, AUDIO_INTM);
	u8 audio_int = i2c_rd8(sd, AUDIO_INT) & ~audio_int_mask;

	i2c_wr8(sd, AUDIO_INT, audio_int);

	v4l2_info(sd, "%s: AUDIO_INT =0x%02x\n", __func__, audio_int);

	tc358743_s_ctrl_audio_sampling_rate(sd);
	tc358743_s_ctrl_audio_present(sd);
}

static void tc358743_csi_err_int_handler(struct v4l2_subdev *sd, bool *handled)
{
	v4l2_err(sd, "%s: CSI_ERR =0x%x\n", __func__, i2c_rd32(sd, CSI_ERR));

	i2c_wr32(sd, CSI_INT_CLR, MASK_ICRER);
}

static void tc358743_hdmi_misc_int_handler(struct v4l2_subdev *sd, bool *handled)
{
	u8 misc_int_mask = i2c_rd8(sd, MISC_INTM);
	u8 misc_int = i2c_rd8(sd, MISC_INT) & ~misc_int_mask;

	i2c_wr8(sd, MISC_INT, misc_int);

	v4l2_info(sd, "%s: MISC_INT =0x%02x\n", __func__, misc_int);

	if (misc_int & MASK_I_SYNC_CHG) {
		/* Reset the HDMI PHY to try to trigger proper lock on the
		 * incoming video format. Erase BKSV to prevent that old keys
		 * are used when a new source is connected. */
		if (no_sync(sd) || no_signal(sd)) {
			tc358743_reset_phy(sd);
			tc358743_erase_bksv(sd);
		}

		tc358743_format_change(sd);

		misc_int &= ~MASK_I_SYNC_CHG;
		if (handled)
			*handled = true;
	}

	if (misc_int) {
		v4l2_err(sd, "%s: Unhandled MISC_INT interrupts:0x%02x\n",	__func__, misc_int);
	}
}

static void tc358743_hdmi_cbit_int_handler(struct v4l2_subdev *sd, bool *handled)
{
	u8 cbit_int_mask = i2c_rd8(sd, CBIT_INTM);
	u8 cbit_int = i2c_rd8(sd, CBIT_INT) & ~cbit_int_mask;

	i2c_wr8(sd, CBIT_INT, cbit_int);

	v4l2_info(sd, "%s: CBIT_INT =0x%02x\n", __func__, cbit_int);

	if (cbit_int & MASK_I_CBIT_FS) {

		v4l2_info(sd, "%s: Audio sample rate changed\n", __func__);
		tc358743_s_ctrl_audio_sampling_rate(sd);

		cbit_int &= ~MASK_I_CBIT_FS;
		if (handled)
			*handled = true;
	}

	if (cbit_int & (MASK_I_AF_LOCK | MASK_I_AF_UNLOCK)) {

		v4l2_info(sd, "%s: Audio present changed\n", __func__);
		tc358743_s_ctrl_audio_present(sd);

		cbit_int &= ~(MASK_I_AF_LOCK | MASK_I_AF_UNLOCK);
		if (handled)
			*handled = true;
	}

	if (cbit_int) {
		v4l2_err(sd, "%s: Unhandled CBIT_INT interrupts:0x%02x\n", __func__, cbit_int);
	}
}

static void tc358743_hdmi_clk_int_handler(struct v4l2_subdev *sd, bool *handled)
{
	u8 clk_int_mask = i2c_rd8(sd, CLK_INTM);
	u8 clk_int = i2c_rd8(sd, CLK_INT) & ~clk_int_mask;

	/* Bit 7 and bit 6 are set even when they are masked */
	i2c_wr8(sd, CLK_INT, clk_int |0x80 | MASK_I_OUT_H_CHG);

	v4l2_info(sd, "%s: CLK_INT =0x%02x\n", __func__, clk_int);

	if (clk_int & (MASK_I_IN_DE_CHG)) {

		v4l2_info(sd, "%s: DE size or position has changed\n",
				__func__);

		/* If the source switch to a new resolution with the same pixel
		 * frequency as the existing (e.g. 1080p25 -> 720p50), the
		 * I_SYNC_CHG interrupt is not always triggered, while the
		 * I_IN_DE_CHG interrupt seems to work fine. Format change
		 * notifications are only sent when the signal is stable to
		 * reduce the number of notifications. */
		if (!no_signal(sd) && !no_sync(sd))
			tc358743_format_change(sd);

		clk_int &= ~(MASK_I_IN_DE_CHG);
		if (handled)
			*handled = true;
	}

	if (clk_int) {
		v4l2_err(sd, "%s: Unhandled CLK_INT interrupts:0x%02x\n", __func__, clk_int);
	}
}

static void tc358743_enable_edid(struct v4l2_subdev *sd)
{
	struct tc358743_state *state = to_state(sd);

	v4l2_info(sd, "%s\n", __func__);
	if (state->edid_blocks_written == 0) {
		v4l2_info(sd, "%s: no EDID -> no hotplug\n", __func__);
		return;
	}

	v4l2_info(sd, "%s:\n", __func__);

	/* Enable hotplug after 100 ms. DDC access to EDID is also enabled when
	 * hotplug is enabled. See register DDC_CTL */
	queue_delayed_work(state->work_queues,
			           &state->delayed_work_enable_hotplug, HZ / 10);

	tc358743_enable_interrupts(sd, true);
	tc358743_s_ctrl_detect_tx_5v(sd);
	v4l2_info(sd, "%s completed successfully", __FUNCTION__);
}
static void tc358743_hdmi_sys_int_handler(struct v4l2_subdev *sd, bool *handled)
{
	struct tc358743_state *state = to_state(sd);
	u8 sys_int_mask = i2c_rd8(sd, SYS_INTM);
	u8 sys_int = i2c_rd8(sd, SYS_INT) & ~sys_int_mask;

	i2c_wr8(sd, SYS_INT, sys_int);

	v4l2_info(sd, "%s: SYS_INT =0x%02x\n", __func__, sys_int);

	if (sys_int & MASK_I_DDC) {
		bool tx_5v = tx_5v_power_present(sd);
		// make hpd ctl?
		//bool tx_5v = true;

		v4l2_info(sd, "%s: Tx 5V power present: %s\n",	__func__, tx_5v ?  "yes" : "no");

		if (tx_5v) {
			tc358743_enable_edid(sd);
		} else {
			tc358743_enable_interrupts(sd, false);
			tc358743_disable_edid(sd);
			memset(&state->timings, 0, sizeof(state->timings));
			tc358743_erase_bksv(sd);
			tc358743_update_controls(sd);
		}

		sys_int &= ~MASK_I_DDC;
		if (handled)
			*handled = true;
	}

	if (sys_int & MASK_I_DVI) {
		v4l2_info(sd, "%s: HDMI->DVI change detected\n", __func__);

		/* Reset the HDMI PHY to try to trigger proper lock on the
		 * incoming video format. Erase BKSV to prevent that old keys
		 * are used when a new source is connected. */
		if (no_sync(sd) || no_signal(sd)) {
			tc358743_reset_phy(sd);
			tc358743_erase_bksv(sd);
		}

		sys_int &= ~MASK_I_DVI;
		if (handled)
			*handled = true;
	}

	if (sys_int & MASK_I_HDMI) {
		v4l2_info(sd, "%s: DVI->HDMI change detected\n", __func__);

		/* Register is reset in DVI mode (REF_01, c. 6.6.41) */
		i2c_wr8(sd, ANA_CTL, MASK_APPL_PCSX_NORMAL | MASK_ANALOG_ON);

		sys_int &= ~MASK_I_HDMI;
		if (handled)
			*handled = true;
	}

	if (sys_int) {
		v4l2_err(sd, "%s: Unhandled SYS_INT interrupts:0x%02x\n", __func__, sys_int);
	}
}


#ifdef CONFIG_VIDEO_ADV_DEBUG
static void tc358743_print_register_map(struct v4l2_subdev *sd)
{
	v4l2_info(sd, "0x0000–0x00FF: Global Control Register\n");
	v4l2_info(sd, "0x0100–0x01FF: CSI2-TX PHY Register\n");
	v4l2_info(sd, "0x0200–0x03FF: CSI2-TX PPI Register\n");
	v4l2_info(sd, "0x0400–0x05FF: Reserved\n");
	v4l2_info(sd, "0x0600–0x06FF: CEC Register\n");
	v4l2_info(sd, "0x0700–0x84FF: Reserved\n");
	v4l2_info(sd, "0x8500–0x85FF: HDMIRX System Control Register\n");
	v4l2_info(sd, "0x8600–0x86FF: HDMIRX Audio Control Register\n");
	v4l2_info(sd, "0x8700–0x87FF: HDMIRX InfoFrame packet data Register\n");
	v4l2_info(sd, "0x8800–0x88FF: HDMIRX HDCP Port Register\n");
	v4l2_info(sd, "0x8900–0x89FF: HDMIRX Video Output Port & 3D Register\n");
	v4l2_info(sd, "0x8A00–0x8BFF: Reserved\n");
	v4l2_info(sd, "0x8C00–0x8FFF: HDMIRX EDID-RAM (1024bytes)\n");
	v4l2_info(sd, "0x9000–0x90FF: HDMIRX GBD Extraction Control\n");
	v4l2_info(sd, "0x9100–0x92FF: HDMIRX GBD RAM read\n");
	v4l2_info(sd, "0x9300-      : Reserved\n");
}

static int tc358743_get_reg_size(u16 address)
{
	/* REF_01 p. 66-72 */
	if (address <=0x00ff)
		return 2;
	else if ((address >=0x0100) && (address <=0x06FF))
		return 4;
	else if ((address >=0x0700) && (address <=0x84ff))
		return 2;
	else
		return 1;
}

static int tc358743_g_register(struct v4l2_subdev *sd,
			                   struct v4l2_dbg_register *reg)
{
	if (reg->reg >0xffff) {
		tc358743_print_register_map(sd);
		return -EINVAL;
	}

	reg->size = tc358743_get_reg_size(reg->reg);

	i2c_rd(sd, reg->reg, (u8 *)&reg->val, reg->size);

	return 0;
}

static int tc358743_s_register(struct v4l2_subdev *sd,
			             const struct v4l2_dbg_register *reg)
{
	if (reg->reg >0xffff) {
		tc358743_print_register_map(sd);
		return -EINVAL;
	}

	/* It should not be possible for the user to enable HDCP with a simple
	 * v4l2-dbg command.
	 *
	 * DO NOT REMOVE THIS unless all other issues with HDCP have been
	 * resolved.
	 */
	if (reg->reg == HDCP_MODE ||
	    reg->reg == HDCP_REG1 ||
	    reg->reg == HDCP_REG2 ||
	    reg->reg == HDCP_REG3 ||
	    reg->reg == BCAPS)
		return 0;

	i2c_wr(sd, (u16)reg->reg, (u8 *)&reg->val,
		   tc358743_get_reg_size(reg->reg));

	return 0;
}
#endif

static int tc358743_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	u16 intstatus = i2c_rd16(sd, INTSTATUS);

	v4l2_info(sd, "%s: IntStatus =0x%04x\n", __func__, intstatus);

	if (intstatus & MASK_HDMI_INT) {
		u8 hdmi_int0 = i2c_rd8(sd, HDMI_INT0);
		u8 hdmi_int1 = i2c_rd8(sd, HDMI_INT1);

		if (hdmi_int0 & MASK_I_MISC)
			tc358743_hdmi_misc_int_handler(sd, handled);
		if (hdmi_int1 & MASK_I_CBIT)
			tc358743_hdmi_cbit_int_handler(sd, handled);
		if (hdmi_int1 & MASK_I_CLK)
			tc358743_hdmi_clk_int_handler(sd, handled);
		if (hdmi_int1 & MASK_I_SYS)
			tc358743_hdmi_sys_int_handler(sd, handled);
		if (hdmi_int1 & MASK_I_AUD)
			tc358743_hdmi_audio_int_handler(sd, handled);

		i2c_wr16(sd, INTSTATUS, MASK_HDMI_INT);
		intstatus &= ~MASK_HDMI_INT;
	}

	if (intstatus & MASK_CSI_INT) {
		u32 csi_int = i2c_rd32(sd, CSI_INT);

		if (csi_int & MASK_INTER)
			tc358743_csi_err_int_handler(sd, handled);

		i2c_wr16(sd, INTSTATUS, MASK_CSI_INT);
		intstatus &= ~MASK_CSI_INT;
	}

	intstatus = i2c_rd16(sd, INTSTATUS);
	if (intstatus) {
		v4l2_info(sd,
				"%s: Unhandled IntStatus interrupts:0x%02x\n",
				__func__, intstatus);
	}

	return 0;
}

static irqreturn_t tc358743_irq_handler(int irq, void *dev_id)
{
	struct tc358743_state *state = dev_id;
	bool handled = false;

	tc358743_isr(&state->sd, 0, &handled);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int tc358743_subscribe_event(struct v4l2_subdev *sd, 
                                    struct v4l2_fh *fh,
				                    struct v4l2_event_subscription *sub)
{
	v4l2_info(sd, "Calling %s\n", __FUNCTION__);
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

/* --------------- VIDEO OPS --------------- */


static int tc358743_s_dv_timings(struct v4l2_subdev *sd,
				                 struct v4l2_dv_timings *timings)
{
	struct tc358743_state *state = to_state(sd);
	v4l2_info(sd, "%s\n",__func__);
	if (!timings)
		v4l2_info(sd,"!timings----");
		return -EINVAL;

	if (v4l2_match_dv_timings(&state->timings, timings, 0, false)) {
		v4l2_info(sd, "%s: no change\n", __func__);
		return 0;
	}

	if (!v4l2_valid_dv_timings(timings,	&tc358743_timings_cap, NULL, NULL)) {
		v4l2_err(sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	state->timings = *timings;
	v4l2_info(sd,"set timing suceed!!!!");
	enable_stream(sd, false);
	tc358743_set_pll(sd);
	tc358743_set_csi(sd);
	
	return 0;
}

static int tc358743_g_dv_timings(struct v4l2_subdev *sd,
				                 struct v4l2_dv_timings *timings)
{
	struct tc358743_state *state = to_state(sd);
	v4l2_info(sd, "Calling %s\n", __FUNCTION__);

	*timings = state->timings;
	
	return 0;
}

static int tc358743_enum_dv_timings(struct v4l2_subdev *sd,
				                    struct v4l2_enum_dv_timings *timings)
{
	v4l2_info(sd, "Calling %s\n", __FUNCTION__);
	if (timings->pad != 0) {
		v4l2_err(sd, "%s: failed %d\n", __func__, EINVAL);
		return -EINVAL;
	}

	return v4l2_enum_dv_timings_cap(timings, &tc358743_timings_cap, NULL, NULL);
}

static int tc358743_query_dv_timings(struct v4l2_subdev *sd,
		                             struct v4l2_dv_timings *timings)
{
	int ret;
	v4l2_info(sd, "Calling %s\n", __FUNCTION__);

	ret = tc358743_get_detected_timings(sd, timings);
	if (ret) {
		v4l2_err(sd, "%s: @@@@@ timings detected error\n", __func__);
		return ret;
	}

	if (debug)
		v4l2_print_dv_timings(sd->name, "tc358743_query_dv_timings: ",
				timings, false);

	if (!v4l2_valid_dv_timings(timings,
				&tc358743_timings_cap, NULL, NULL)) {
		v4l2_err(sd, "%s: @@@@@ timings out of range\n", __func__);
		return -ERANGE;
	}

	return 0;
}
static int tc358743_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct tc358743_state *state = to_state(sd);
	struct v4l2_dv_timings *timings = &(state->timings);
	
	v4l2_info(sd, "Calling %s\n", __FUNCTION__);
	*status = 0;
	*status |= no_signal(sd) ? V4L2_IN_ST_NO_SIGNAL : 0;
	*status |= no_sync(sd) ? V4L2_IN_ST_NO_SYNC : 0;

	v4l2_info(sd, "%s: status =0x%x\n", __func__, *status);

	v4l2_info(sd, "Now getting and setting dv timings");
	tc358743_query_dv_timings(sd, timings);
	tc358743_s_dv_timings(sd, timings);

	return 0;
}

static int tc358743_dv_timings_cap(struct v4l2_subdev *sd,
		                           struct v4l2_dv_timings_cap *cap)
{
	v4l2_info(sd, "Calling %s\n", __FUNCTION__);
	if (cap->pad != 0)
		return -EINVAL;

	*cap = tc358743_timings_cap;

	return 0;
}

static int tc358743_g_mbus_config(struct v4l2_subdev *sd,
			                      struct v4l2_mbus_config *cfg)
{
	v4l2_info(sd, "Calling %s\n", __FUNCTION__);
	cfg->type = V4L2_MBUS_CSI2;

	/* Support for non-continuous CSI-2 clock is missing in the driver */
	cfg->flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	switch (tc358743_num_csi_lanes_in_use(sd)) {
	case 1:
		cfg->flags |= V4L2_MBUS_CSI2_1_LANE;
		break;
	case 2:
		cfg->flags |= V4L2_MBUS_CSI2_2_LANE;
		break;
	case 3:
		cfg->flags |= V4L2_MBUS_CSI2_3_LANE;
		break;
	case 4:
		cfg->flags |= V4L2_MBUS_CSI2_4_LANE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
static int tc358743_s_stream(struct v4l2_subdev *sd, int enable)
{
	v4l2_info(sd, "Calling %s\n", __FUNCTION__);
	v4l2_err(sd, "Calling %s\n", __FUNCTION__);
	/* FIXME: show log status for test */
	/*hainh
	enable_stream(sd, enable);
	if (enable)
		tc358743_log_status(sd);
	*/
	enable_stream(sd, true);
	// if (true)
	// 	tc358743_log_status(sd);
	
	return 0;
}

/* --------------- PAD OPS --------------- */

static int tc358743_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config  *cfg,
		struct v4l2_subdev_format *format)
{
	struct tc358743_state *state = to_state(sd);
	u8 vi_rep = i2c_rd8(sd, VI_REP);
	v4l2_info(sd, "Calling %s\n", __FUNCTION__);

	if (format->pad != 0) {
		v4l2_err(sd, "%s Error\n", __FUNCTION__);
		return -EINVAL;
	}

	format->format.code = state->mbus_fmt_code;
	format->format.width = state->timings.bt.width;
	format->format.height = state->timings.bt.height;
	format->format.field = V4L2_FIELD_NONE;


	switch (vi_rep & MASK_VOUT_COLOR_SEL) {
	case MASK_VOUT_COLOR_RGB_FULL:
	case MASK_VOUT_COLOR_RGB_LIMITED:
		format->format.colorspace = V4L2_COLORSPACE_SRGB;
		break;
	case MASK_VOUT_COLOR_601_YCBCR_LIMITED:
	case MASK_VOUT_COLOR_601_YCBCR_FULL:
		v4l2_info(sd, "Here 6b, colorspace: %d\n", V4L2_COLORSPACE_SMPTE170M);
		format->format.colorspace = V4L2_COLORSPACE_SMPTE170M;
		break;
	case MASK_VOUT_COLOR_709_YCBCR_FULL:
	case MASK_VOUT_COLOR_709_YCBCR_LIMITED:
		format->format.colorspace = V4L2_COLORSPACE_REC709;
		break;
	default:
		format->format.colorspace = 0;
        v4l2_info(sd,"%d:%s colorspace = 0\n",__LINE__,__FUNCTION__);
		break;
	}

	v4l2_info(sd, "get fmt complete\n");
	v4l2_info(sd, "format width %d\n", format->format.width);
	v4l2_info(sd, "format height %d\n", format->format.height);

	v4l2_info(sd, "fmt_code: %d\n", format->format.code);
	v4l2_info(sd, "RGB888 code: %d\n", MEDIA_BUS_FMT_RGB888_1X24);
	v4l2_info(sd, "UYVY8 code: %d\n", MEDIA_BUS_FMT_UYVY8_1X16);
	return 0;
}


static int tc358743_set_fmt(struct v4l2_subdev *sd,
		                    struct v4l2_subdev_pad_config  *cfg,
		                    struct v4l2_subdev_format *format)
{
	struct tc358743_state *state = to_state(sd);
	u32 code = format->format.code;
	int ret = tc358743_get_fmt(sd, cfg, format);

	v4l2_dbg(3, debug, sd, "%s(), ret: %d\n", __func__, ret);
	v4l2_dbg(3, debug, sd, "Set format code: %d\n", code);

	format->format.code = code;

	if (ret)
		return ret;

	switch (code) {
		case MEDIA_BUS_FMT_RGB888_1X24:
		case MEDIA_BUS_FMT_UYVY8_1X16:
			v4l2_dbg(3, debug, sd, "Good code %d\n", code);
			break;
		default:
			v4l2_err(sd, "Bad code %d\n", code);
			return -EINVAL;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	state->mbus_fmt_code = format->format.code;

	enable_stream(sd, false);
	tc358743_set_pll(sd);
	tc358743_set_csi(sd);
	tc358743_set_csi_color_space(sd);
	v4l2_info(sd, "Called %s, completed successfully\n", __FUNCTION__);
	return 0;
}

static int tc358743_g_edid(struct v4l2_subdev *sd,
		                   struct v4l2_subdev_edid *edid)
{
	struct tc358743_state *state = to_state(sd);
	// int i=0;
	v4l2_info(sd, "Calling %s\n", __FUNCTION__);

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block == 0 && edid->blocks == 0) {
		edid->blocks = state->edid_blocks_written;
		return 0;
	}

	if (state->edid_blocks_written == 0)
		return -ENODATA;

	if (edid->start_block >= state->edid_blocks_written || edid->blocks == 0)
		return -EINVAL;

	if (edid->start_block + edid->blocks > state->edid_blocks_written)
		edid->blocks = state->edid_blocks_written - edid->start_block;

	i2c_rd(sd, EDID_RAM + (edid->start_block * EDID_BLOCK_SIZE), edid->edid,
			edid->blocks * EDID_BLOCK_SIZE);
	v4l2_info(sd,"EDID_RAM has %d byte from: 0x%04x to 0x%04x \r\n",
		edid->blocks * EDID_BLOCK_SIZE,
		EDID_RAM + (edid->start_block * EDID_BLOCK_SIZE),
		EDID_RAM + (edid->start_block * EDID_BLOCK_SIZE) + 
		edid->blocks * EDID_BLOCK_SIZE);
	// for(i=0;i<edid->blocks * EDID_BLOCK_SIZE;i++){
	// 	printk("%02x ",edid->edid[i]);
	// }
	// v4l2_info(sd,"\r\n");
	v4l2_info(sd, "%s completed successfully", __FUNCTION__);
	return 0;
}

static int tc358743_s_edid(struct v4l2_subdev *sd,
				           struct v4l2_subdev_edid *edid)
{
	struct tc358743_state *state = to_state(sd);
	u16 edid_len = edid->blocks * EDID_BLOCK_SIZE;
	
	v4l2_info(sd, "%s, pad %d, start block %d, blocks %d\n",
		 __func__, edid->pad, edid->start_block, edid->blocks);

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block != 0)
		return -EINVAL;

	if (edid->blocks > EDID_NUM_BLOCKS_MAX) {
		edid->blocks = EDID_NUM_BLOCKS_MAX;
		return -E2BIG;
	}

	tc358743_disable_edid(sd);

	i2c_wr8(sd, EDID_LEN1, edid_len & 0xff);
	i2c_wr8(sd, EDID_LEN2, edid_len >> 8);

	if (edid->blocks == 0) {
		state->edid_blocks_written = 0;
		return 0;
	}
	i2c_wr(sd, EDID_RAM, edid->edid, edid_len);
	//richardyou edid write
	// for (i=0; i<edid_len; i++) {
	// 	i2c_wr8(sd, EDID_RAM + i, edid->edid[i]);
	// }
	
	state->edid_blocks_written = edid->blocks;

	if (tx_5v_power_present(sd))
		tc358743_enable_edid(sd);

	v4l2_info(sd, "%s completed successfully", __FUNCTION__);
	return 0;
}

// static int tc358743_mbus_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
// {
// 	struct tc358743_state *state = to_state(sd);
// 	u8 vi_rep = i2c_rd8(sd, VI_REP);

// 	mf->code = state->mbus_fmt_code;
// 	mf->width = state->timings.bt.width;
// 	mf->height = state->timings.bt.height;
// 	mf->field = V4L2_FIELD_NONE;
// 	switch (vi_rep & MASK_VOUT_COLOR_SEL) {
// 		case MASK_VOUT_COLOR_RGB_FULL:
// 		case MASK_VOUT_COLOR_RGB_LIMITED:
// 			mf->colorspace = V4L2_COLORSPACE_SRGB;
// 			break;
// 		case MASK_VOUT_COLOR_601_YCBCR_LIMITED:
// 		case MASK_VOUT_COLOR_601_YCBCR_FULL:
// 			mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
// 			break;
// 		case MASK_VOUT_COLOR_709_YCBCR_FULL:
// 		case MASK_VOUT_COLOR_709_YCBCR_LIMITED:
// 			mf->colorspace = V4L2_COLORSPACE_REC709;
// 			break;
// 		default:
// 			mf->colorspace = 0;
// 			break;
// 	}
// 	return 0;
// }




static int tc358743_enum_mbus_code(struct v4l2_subdev *sd,
				                   struct v4l2_subdev_pad_config  *cfg,
				                   struct v4l2_subdev_mbus_code_enum *code)
{
	v4l2_info(sd, "%s()\n", __func__);

	// if (code->index >= 2) {
	// 	v4l2_err(sd, "Error in %s\n", __FUNCTION__);
	// 	v4l2_err(sd, "print code-> %d\n",code->index);///!!!!add code
	// 	return -EINVAL;
	// }

	// switch (code->index) {
	// 	case 0:
	// 		code->code = MEDIA_BUS_FMT_RGB888_1X24;
			
	// 		break;
	// 	case 1:
			
	// 		code->code = MEDIA_BUS_FMT_UYVY8_1X16;
	// 		break;
	// 	default:
	// 		return -EINVAL;
	// }

    {
	switch (code->index) {
	case 0:
		code->code = MEDIA_BUS_FMT_RGB888_1X24;
		break;
	case 1:
		code->code = MEDIA_BUS_FMT_UYVY8_1X16;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
	v4l2_info(sd, "Mbus code found succsefully (%d: %d)", code->index, code->code);
	
	return 0;
}

// static int tc358743_enum_mbus_code(struct v4l2_subdev *sd,
// 		struct v4l2_subdev_pad_config *cfg,
// 		struct v4l2_subdev_mbus_code_enum *code)
// {
// 	switch (code->index) {
// 	case 0:
// 		code->code = MEDIA_BUS_FMT_RGB888_1X24;
// 		break;
// 	case 1:
// 		code->code = MEDIA_BUS_FMT_UYVY8_1X16;
// 		break;
// 	default:
// 		return -EINVAL;
// 	}
// 	return 0;
// }

static int tc358743_enum_frame_size(struct v4l2_subdev *sd,
				                    struct v4l2_subdev_pad_config  *cfg,                           
				                    struct v4l2_subdev_frame_size_enum *fse)
{
	const struct camera_common_frmfmt *frmfmt = tc358743_frmfmt;
	int num_frmfmt = ARRAY_SIZE(tc358743_frmfmt);

	v4l2_info(sd, "%s()\n", __func__);
	v4l2_info(sd, "fse->code %d, index %d\n", fse->code, fse->index);
	v4l2_info(sd, "----------------------------------------\n");

	// fse->min_width  = fse->max_width  = 1280;
	// fse->min_height = fse->max_height = 720;

	v4l2_info(sd, "Trying to find frmfmt that matches fse->code, code: %d (UYVY: %d, ARGB32: %d, MEDIA_BUS_FMT_UYVY8_1X16: %d, MEDIA_BUS_FMT_RGB888_1X24: %d)\n", fse->code, V4L2_PIX_FMT_UYVY, V4L2_PIX_FMT_ARGB32, MEDIA_BUS_FMT_UYVY8_1X16, MEDIA_BUS_FMT_RGB888_1X24);

	if (fse->code != MEDIA_BUS_FMT_UYVY8_1X16  && fse->code != MEDIA_BUS_FMT_RGB888_1X24 ) {//add fse->code != MEDIA_BUS_FMT_RGB888_1X24
		v4l2_err(sd, "Error in %s fse->code, code: %d, UYVY: %d, ARGB32: %d\n", __FUNCTION__, fse->code, V4L2_PIX_FMT_UYVY, V4L2_PIX_FMT_ARGB32);
		return -EINVAL;
	}

	v4l2_info(sd, "Code ok");

	if (fse->index >= num_frmfmt) {
		v4l2_err(sd, "Error in %s, %d outside of num_frmfmt (%d)", __FUNCTION__, fse->index, num_frmfmt);
		return -EINVAL;
	}
	
	v4l2_info(sd, "Index ok");

	fse->min_width  = fse->max_width  = frmfmt[fse->index].size.width;
	fse->min_height = fse->max_height = frmfmt[fse->index].size.height;
	v4l2_info(sd, "!!!!!!!!! %s() complete successfully, width: %d, height: %d\n", __func__, fse->min_width, fse->min_height);
	return 0;
}

static int tc358743_enum_frame_interval(struct v4l2_subdev *sd,
				                        struct v4l2_subdev_pad_config  *cfg,
				                        struct v4l2_subdev_frame_interval_enum *fie)
{
	const struct camera_common_frmfmt *frmfmt = tc358743_frmfmt;
	int num_frmfmt = ARRAY_SIZE(tc358743_frmfmt);
	int i;

	v4l2_info(sd, "%s()\n", __func__);
	v4l2_info(sd, "----------------------------------------\n");

	v4l2_info(sd, "Trying to find frame interfval that matches fie->code, code: %d (UYVY: %d, ARGB32: %d, MEDIA_BUS_FMT_UYVY8_1X16: %d)\n", fie->code, V4L2_PIX_FMT_UYVY, V4L2_PIX_FMT_ARGB32, MEDIA_BUS_FMT_UYVY8_1X16);


	if (fie->code != V4L2_PIX_FMT_UYVY && fie->code != MEDIA_BUS_FMT_UYVY8_1X16) {
		v4l2_err(sd, "Unexpected code (%d), UYUV: %d, ABGR32: %d\n", fie->code, V4L2_PIX_FMT_UYVY, V4L2_PIX_FMT_ARGB32);
		return -EINVAL;
	}

	v4l2_info(sd, "Code ok");

	for (i = 0; i < num_frmfmt; i++) {
		if (frmfmt[i].size.width == fie->width && frmfmt[i].size.height == fie->height) {
			v4l2_info(sd, "Matched width %d and %d, height %d and %d", frmfmt[i].size.width, fie->width, frmfmt[i].size.height, fie->height);
			break;
		}
	}

	v4l2_info(sd, "w/h ok or end (i=%d, num=%d)", i, num_frmfmt);
	
	if (i >= num_frmfmt) {
		v4l2_err(sd, "Error in %s, num frmfmt\n", __FUNCTION__);
		return -EINVAL;
	}

	v4l2_info(sd, "i ok");

	if (fie->index >= frmfmt[i].num_framerates) {
		v4l2_err(sd, "Error in %s num framerates (%d outside %d)\n", __FUNCTION__, fie->index, frmfmt[i].num_framerates);
		return -EINVAL;
	}

	v4l2_info(sd, "index ok");

	fie->interval.numerator = 1;
	fie->interval.denominator = frmfmt[i].framerates[fie->index];
	v4l2_info(sd, "!!!!!!!!!! %s() completed successfully, interval: 1/%d\n", __func__, fie->interval.denominator);
	return 0;
}

static int tc358743_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}
static const struct v4l2_subdev_core_ops tc358743_core_ops = {
	.s_power = tc358743_s_power,
	.log_status = tc358743_log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = tc358743_g_register,
	.s_register = tc358743_s_register,
#endif
	.interrupt_service_routine = tc358743_isr,
	.subscribe_event = tc358743_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops tc358743_video_ops = {
	.g_input_status = tc358743_g_input_status,
	.s_dv_timings = tc358743_s_dv_timings,
	.g_dv_timings = tc358743_g_dv_timings,
	.s_stream = tc358743_s_stream,
	//.mbus_fmt	= tc358743_mbus_fmt,
	.g_mbus_config = tc358743_g_mbus_config,
	.query_dv_timings = tc358743_query_dv_timings,
};

static const struct v4l2_subdev_pad_ops tc358743_pad_ops = {
	.set_fmt = tc358743_set_fmt,
	.get_fmt = tc358743_get_fmt,
	.get_edid = tc358743_g_edid,
	.set_edid = tc358743_s_edid,
	.dv_timings_cap = tc358743_dv_timings_cap,
	.enum_dv_timings = tc358743_enum_dv_timings,
	.enum_mbus_code = tc358743_enum_mbus_code,
	.enum_frame_size = tc358743_enum_frame_size,
	.enum_frame_interval = tc358743_enum_frame_interval,
};

static const struct v4l2_subdev_ops tc358743_ops = {
	.core = &tc358743_core_ops,
	.video = &tc358743_video_ops,
	.pad = &tc358743_pad_ops,
};
/* --------------- CUSTOM CTRLS --------------- */

static const struct v4l2_ctrl_config tc358743_ctrl_audio_sampling_rate = {
	.id = TC358743_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio sampling rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 768000,
	.step = 1,
	.def = 44100,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config tc358743_ctrl_audio_present = {
	.id = TC358743_CID_AUDIO_PRESENT,
	.name = "Audio present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 1,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};
static bool tc358743_parse_dt(struct tc358743_platform_data *pdata,
		struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	const u32 *property;

	v4l_dbg(1, debug, client, "Device Tree Parameters:\n");

	pdata->reset_gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (pdata->reset_gpio == 0)
		return false;
	v4l_dbg(1, debug, client, "reset_gpio = %d\n", pdata->reset_gpio);

	property = of_get_property(node, "refclk_hz", NULL);
	if (property == NULL)
		return false;
	pdata->refclk_hz = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "refclk_hz = %d\n", be32_to_cpup(property));

	return true;
}

#ifdef CONFIG_OF
static void tc358743_gpio_reset(struct tc358743_state *state)
{

	usleep_range(5000, 10000);
	// TODO: Re-implement the reset GPIO!
	//~ gpiod_set_value(state->reset_gpio, 1);
    // gpio_set_value((int)state->reset_gpio, 1);
	usleep_range(1000, 2000);
    // gpio_set_value((int)state->reset_gpio, 0);
	//~ gpiod_set_value(state->reset_gpio, 0);
	msleep(20);
}



static int tc358743_probe_of(struct tc358743_state *state)
{
	struct device *dev = &state->i2c_client->dev;
	struct v4l2_of_endpoint *endpoint;
	struct device_node *ep;
	// struct clk *refclk;
	u32 bps_pr_lane;
	int ret = -EINVAL;


	// refclk = devm_clk_get(dev, "cam_mclk1");
	// if (IS_ERR(refclk)) {
	// 	if (PTR_ERR(refclk) != -EPROBE_DEFER)
	// 		dev_err(dev, "failed to get refclk: %ld\n",
	// 			PTR_ERR(refclk));
	// 	return PTR_ERR(refclk);
	// }

	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -EINVAL;
	}

	endpoint = v4l2_of_alloc_parse_endpoint(ep);
	if (IS_ERR(endpoint)) {
		dev_err(dev, "failed to parse endpoint\n");
		return PTR_ERR(endpoint);
	}

	if (endpoint->bus_type != V4L2_MBUS_CSI2 ||
	    endpoint->bus.mipi_csi2.num_data_lanes == 0 ||
	    endpoint->nr_of_link_frequencies == 0) {
		dev_err(dev, "missing CSI-2 properties in endpoint\n");
		goto free_endpoint;
	}

	pr_info("tc358743 endpoint->bus.mipi_csi2.flags %d\n",
		endpoint->bus.mipi_csi2.flags);
	pr_info("tc358743 endpoint->bus.mipi_csi2.clock_lane %d\n",
		endpoint->bus.mipi_csi2.clock_lane);
	pr_info("tc358743 endpoint->bus.mipi_csi2.num_data_lanes %d\n",
		endpoint->bus.mipi_csi2.num_data_lanes);
	pr_info("tc358743 endpoint->bus.mipi_csi2.data_lanes [%d-%d-%d-%d]\n",
			endpoint->bus.mipi_csi2.data_lanes[0],
			endpoint->bus.mipi_csi2.data_lanes[1],
			endpoint->bus.mipi_csi2.data_lanes[2],
			endpoint->bus.mipi_csi2.data_lanes[3]);
    pr_info("tc358743 endpoint->nr_of_link_frequencies %d\n",
    	endpoint->nr_of_link_frequencies);

	// state->bus = endpoint->bus.mipi_csi2;
    // pr_info("tc358743 state->bus %s\n",state->bus);
	// clk_prepare_enable(refclk);

	// state->pdata.refclk_hz = clk_get_rate(refclk);
	state->pdata.refclk_hz = 27000000;
    // if ((state->pdata.refclk_hz != 26000000) ||
    //     (state->pdata.refclk_hz != 27000000) ||
    //     (state->pdata.refclk_hz != 42000000))
    // {
    //     pr_info("Set new clock\n");
    //     if (0 != clk_set_rate(refclk,27000000))
    //     {
    //         pr_info("Error: Set new clock\n");
    //     }
    // }

	state->pdata.ddc5v_delay = DDC5V_DELAY_100_MS;
	state->pdata.hdmi_detection_delay = HDMI_MODE_DELAY_100_MS;
	state->pdata.enable_hdcp = false;
	/* A FIFO level of 16 should be enough for 2-lane 720p60 at 594 MHz. */
	state->pdata.fifo_level = 16;
	/*
	 * The PLL input clock is obtained by dividing refclk by pll_prd.
	 * It must be between 6 MHz and 40 MHz, lower frequency is better.
	 */
	switch (state->pdata.refclk_hz) {
    //~ case 26322581:
        //~ state->pdata.refclk_hz = 26322581;
	case 26000000:
	case 27000000:
	//~ case 40800000: /* Tegra */
	case 42000000:
		state->pdata.pll_prd = state->pdata.refclk_hz / 6000000;
		break;
	default:
		dev_err(dev, "Unsupported refclk rate: %u Hz\n",
			state->pdata.refclk_hz);
		goto disable_clk;
	}

	/*
	 * The CSI bps per lane must be between 62.5 Mbps and 1 Gbps.
	 * The default is 594 Mbps for 4-lane 1080p60 or 2-lane 720p60.
	 */
	bps_pr_lane = 2 * endpoint->link_frequencies[0];
	if (bps_pr_lane < 62500000U || bps_pr_lane > 1188000000U) {
		dev_err(dev, "unsupported bps per lane: %u bps\n", bps_pr_lane);
		goto disable_clk;
	}

	/* The CSI speed per lane is refclk / pll_prd * pll_fbd */
	state->pdata.pll_fbd = bps_pr_lane /
			       state->pdata.refclk_hz * state->pdata.pll_prd;

	/*
	 * FIXME: These timings are from REF_02 for 594 Mbps per lane (297 MHz
	 * link frequency). In principle it should be possible to calculate
	 * them based on link frequency and resolution.
	 */
	if (bps_pr_lane != 594000000U)
		dev_warn(dev, "untested bps per lane: %u bps\n", bps_pr_lane);
	pr_info("tc358743 state->pdata.pll_prd=%d\r\n",state->pdata.pll_prd);
	pr_info("tc358743 state->pdata.pll_fbd=%d\r\n",state->pdata.pll_fbd);

	// freq = refclk / prd * fbd, default = 594 MHz
	state->pdata.lineinitcnt = 0xe80;
	state->pdata.lptxtimecnt = 0x003;
	/* tclk-preparecnt: 3, tclk-zerocnt: 20 */
	state->pdata.tclk_headercnt = 0x1403;
	state->pdata.tclk_trailcnt = 0x00;
	/* ths-preparecnt: 3, ths-zerocnt: 1 */
	state->pdata.ths_headercnt = 0x0103;
	state->pdata.twakeup = 0x4882;
	state->pdata.tclk_postcnt = 0x008;
	state->pdata.ths_trailcnt = 0x2;
	state->pdata.hstxvregcnt = 2;//!!!2
	
/*richard you*/	
	//state->pdata.pll_prd        = 4;
#ifdef TC358743_VOUT_RGB
	state->pdata.pll_fbd        = 132;
	
	// timing setting
	state->pdata.lineinitcnt    =0x00000fa0;
	state->pdata.lptxtimecnt    =0x00000005;
	state->pdata.tclk_headercnt =0x00001603;
	state->pdata.tclk_trailcnt  =0x00000001;
	state->pdata.ths_headercnt  =0x00000603;
	state->pdata.twakeup        =0x000032c8;
	state->pdata.tclk_postcnt   =0x00000008;
	state->pdata.ths_trailcnt   =0x00000002;
	state->pdata.hstxvregcnt    =0x00000005;
#else
// 	state->pdata.pll_fbd        = 144;//144

// 	// timing setting
// //case 972000000U:
// 	state->pdata.lineinitcnt = 0xFA0;
// 	state->pdata.lptxtimecnt = 0x007;
// 		/* tclk-preparecnt: 6, tclk-zerocnt: 40 */
// 	state->pdata.tclk_headercnt = 0x2806;
// 	state->pdata.tclk_trailcnt = 0x00;
// 		/* ths-preparecnt: 3, ths-zerocnt: 1 */
// 	state->pdata.ths_headercnt = 0x0806;
// 	state->pdata.twakeup = 0x4268;
// 	state->pdata.tclk_postcnt = 0x008;
// 	state->pdata.ths_trailcnt = 0x5;
// 	state->pdata.hstxvregcnt = 0;
		state->pdata.pll_fbd        = 176;//144

	// timing setting
//case 972000000U:
	state->pdata.lineinitcnt = 0x1d01;//
	state->pdata.lptxtimecnt = 0x008;//
		/* tclk-preparecnt: 6, tclk-zerocnt: 45 */
	state->pdata.tclk_headercnt = 0x2D06;//0x0218
	state->pdata.tclk_trailcnt = 0x09;//
		/* ths-preparecnt: 7, ths-zerocnt: 17 */
	state->pdata.ths_headercnt = 0xd06;//0x0220
	state->pdata.twakeup = 0x4883;//
	state->pdata.tclk_postcnt = 0x010;//
	state->pdata.ths_trailcnt = 0xA;//
	state->pdata.hstxvregcnt = 5;//5
#endif


	
	//~ state->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						    //~ GPIOD_OUT_LOW);
	//~ if (IS_ERR(state->reset_gpio)) {
		//~ dev_err(dev, "failed to get reset gpio\n");
		//~ ret = PTR_ERR(state->reset_gpio);
		//~ goto disable_clk;
	//~ }

	if (state->reset_gpio) {
		pr_info("Calling reset GPIO but NOT IMPLEMENTED!");
		tc358743_gpio_reset(state);
	}
	ret = 0;
	goto free_endpoint;

disable_clk:
	// clk_disable_unprepare(refclk);
free_endpoint:
	v4l2_of_free_endpoint(endpoint);
	return ret;
}
#else
static inline int tc358743_probe_of(struct tc358743_state *state)
{
	return -ENODEV;
}
#endif

static int tc358743_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

static const struct v4l2_subdev_internal_ops tc358743_subdev_internal_ops = {
	.open = tc358743_open,
};

static const struct media_entity_operations tc358743_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};


static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int tc358743_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	static struct v4l2_dv_timings default_timing =
        // V4L2_DV_BT_CEA_1920X1080P50;
        V4L2_DV_BT_CEA_1920X1080P60;
	// set edid
    struct v4l2_subdev_edid sd_edid = {
		.blocks = 2,
		.edid = edid,
	};
	struct tc358743_state *state;
	struct tc358743_platform_data *pdata = client->dev.platform_data;
	struct v4l2_subdev *sd;
	int err;
	u16 chip_id_val;

    // pr_info("%s %s %s\n",__FUNCTION__,__DATE__,__TIME__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
	v4l_dbg(1, debug, client, "chip found @0x%x (%s)\n",
		client->addr, client->adapter->name);

	state = devm_kzalloc(&client->dev, sizeof(struct tc358743_state),
			GFP_KERNEL);
	if (!state)
		return -ENOMEM;
    if (client->dev.of_node) {
        if (!tc358743_parse_dt(&state->pdata, client)) {
            pr_err("Couldn't parse device tree\n");
            return -ENODEV;
        }
    }

	state->i2c_client = client;

	/* platform data */
	if (pdata) {
		state->pdata = *pdata;
		pdata->endpoint.bus.mipi_csi2.flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	} else {
		err = tc358743_probe_of(state);
		if (err == -ENODEV)
			v4l_err(client, "No platform data!\n");
		if (err)
			return err;
	}

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &tc358743_ops);
	v4l2_info(sd,"Subdev init done\n");
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	/* i2c access */
	chip_id_val = i2c_rd16(sd, CHIPID);
	v4l2_info(sd,"Chip ID val: %d\n", chip_id_val);

	if ((chip_id_val & MASK_CHIPID) != 0 || chip_id_val == 99) {
        v4l2_info(sd,"tc358743: ERROR: not a TC358743 on address0x%x\n",
			  client->addr);
		return -ENODEV;
	}
	
	/* control handlers */
	v4l2_ctrl_handler_init(&state->hdl, 3);
	v4l2_info(sd, "ctrl handler initied\n");

	/* private controls */
	state->detect_tx_5v_ctrl = v4l2_ctrl_new_std(&state->hdl, NULL,
			V4L2_CID_DV_RX_POWER_PRESENT, 0, 1, 0, 0);

	/* custom controls */
	state->audio_sampling_rate_ctrl = v4l2_ctrl_new_custom(&state->hdl,
			&tc358743_ctrl_audio_sampling_rate, NULL);
	 v4l2_info(sd,"tc358743_ctrl_audio_sampling_rate %d",
	 		  get_audio_sampling_rate(sd));

	state->audio_present_ctrl = v4l2_ctrl_new_custom(&state->hdl,
			&tc358743_ctrl_audio_present, NULL);
	 v4l2_info(sd,"tc358743_ctrl_audio_present %s",
	 		  (audio_present(sd)? "yes":"no"));

	v4l2_info(sd, "A bunch of new cutoms done\n");

	sd->ctrl_handler = &state->hdl;
	if (state->hdl.error) {
		err = state->hdl.error;
		goto err_hdl;
	}

	if (tc358743_update_controls(sd)) {
		err = -ENODEV;
		goto err_hdl;
	}

	v4l2_info(sd, "Controls updated\n");

	/* work queues */
	state->work_queues = create_singlethread_workqueue(client->name);
	if (!state->work_queues) {
		v4l2_err(sd, "Could not create work queue\n");
		err = -ENOMEM;
		goto err_hdl;
	}
	v4l2_info(sd, "Work queue created\n");
	// sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	sd->entity.ops = &tc358743_media_ops;
	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	v4l2_info(sd, "About to call tegra_media_entity_init\n");
	err = tegra_media_entity_init(&sd->entity, 1, &state->pad, true, true);
	if (err < 0)
		goto err_hdl;
	v4l2_info(sd, "tegra_media_entity_init complete\n");

#ifdef TC358743_VOUT_RGB
	state->mbus_fmt_code = MEDIA_BUS_FMT_RGB888_1X24;
#else
	state->mbus_fmt_code = MEDIA_BUS_FMT_UYVY8_1X16;
#endif

	v4l2_info(sd, "Set mbus_fmt_code in probe to: %d\n", state->mbus_fmt_code);

	sd->dev = &client->dev;
	v4l2_info(sd, "About to register subdev\n");
	err = v4l2_async_register_subdev(sd);
	v4l_dbg(1, debug, client, "Register subdev: %d\n", err);

	if (err < 0)
		goto err_hdl;

	mutex_init(&state->confctl_mutex);

	INIT_DELAYED_WORK(&state->delayed_work_enable_hotplug,
			tc358743_delayed_work_enable_hotplug);
	v4l2_info(sd,"before tc358743_initial_setup\r\n");
	//tc358743_log_status(sd);
	tc358743_initial_setup(sd);
	v4l2_info(sd,"after tc358743_initial_setup\r\n");
	
	tc358743_set_csi_color_space(sd);
	v4l2_info(sd,"before tc358743_s_dv_timings\r\n");
	//tc358743_log_status(sd);
	tc358743_s_dv_timings(sd, &default_timing);

	v4l2_info(sd,"before tc358743_init_interrupts, irq: %d\r\n", state->i2c_client->irq);
	tc358743_log_status(sd);//changede by z
	tc358743_init_interrupts(sd);
	v4l2_info(sd,"after tc358743_init_interrupts, irq: %d\r\n", state->i2c_client->irq);
	if (state->i2c_client->irq) {
		v4l2_info(sd,"IQR request\r\n");
		err = devm_request_threaded_irq(&client->dev,
						state->i2c_client->irq,
						NULL, tc358743_irq_handler,
						IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
						"tc358743", state);
		v4l2_err(sd,"err, %d\n", err);
		if (err)
			goto err_work_queues;
	}

	tc358743_enable_interrupts(sd, true);
	i2c_wr16(sd, INTMASK, ~(MASK_HDMI_MSK | MASK_CSI_MSK) & 0xffff);

	err = v4l2_ctrl_handler_setup(sd->ctrl_handler);

	if (err)
		goto err_work_queues;

	v4l2_info(sd, "%s found @0x%x (%s)\n", client->name,
		  client->addr, client->adapter->name);
	tc358743_s_edid(sd, &sd_edid);

	tc358743_g_edid(sd, &sd_edid);

	tc358743_log_status(sd);
	v4l2_info(sd,"Probe complete\n");
	return 0;

err_work_queues:
	cancel_delayed_work(&state->delayed_work_enable_hotplug);
	destroy_workqueue(state->work_queues);
	mutex_destroy(&state->confctl_mutex);
err_hdl:
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&state->hdl);
	return err;
}

static int tc358743_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tc358743_state *state = to_state(sd);

	cancel_delayed_work(&state->delayed_work_enable_hotplug);
	destroy_workqueue(state->work_queues);
	v4l2_async_unregister_subdev(sd);
	v4l2_device_unregister_subdev(sd);
	mutex_destroy(&state->confctl_mutex);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&state->hdl);

	return 0;
}

static struct i2c_device_id tc358743_id[] = {
	{"tc358743", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tc358743_id);

static struct i2c_driver tc358743_driver = {
	.driver = {
		.name = "tc358743",
	},
	.probe = tc358743_probe,
	.remove = tc358743_remove,
	.id_table = tc358743_id,
};

module_i2c_driver(tc358743_driver);