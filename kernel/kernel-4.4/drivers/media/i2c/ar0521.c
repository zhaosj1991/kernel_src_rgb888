/*
 * ar0521_v4l2.c - ar0521 sensor driver
 *
 * Copyright (c) 2013-2017, GALAXY CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEBUG
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>

#include "ar0521.h"
#include "ar0521_mode_tbls.h"

#define AR0521_MAX_COARSE_DIFF      6

#define AR0521_GAIN_SHIFT       8
#define AR0521_MIN_GAIN     (0x2000)
#define AR0521_MAX_GAIN     (0x203f)
#define AR0521_MAX_UNREAL_GAIN  (0x0F80)
#define AR0521_MIN_FRAME_LENGTH (0x0)
#define AR0521_MAX_FRAME_LENGTH (0x7fff)
#define AR0521_MIN_EXPOSURE_COARSE  (0x0002)
#define AR0521_MAX_EXPOSURE_COARSE  \
    (AR0521_MAX_FRAME_LENGTH-AR0521_MAX_COARSE_DIFF)
#define AR0521_DEFAULT_LINE_LENGTH  (0xA80)
#define AR0521_DEFAULT_PIXEL_CLOCK  (160)

#define AR0521_DEFAULT_GAIN     AR0521_MIN_GAIN
#define AR0521_DEFAULT_FRAME_LENGTH (0x07C0)
#define AR0521_DEFAULT_EXPOSURE_COARSE  \
    (AR0521_DEFAULT_FRAME_LENGTH-AR0521_MAX_COARSE_DIFF)

#define AR0521_DEFAULT_MODE AR0521_MODE_2592X1944
#define AR0521_DEFAULT_HDR_MODE AR0521_MODE_2592X1944_HDR
#define AR0521_DEFAULT_WIDTH    2592
#define AR0521_DEFAULT_HEIGHT   1944
#define AR0521_DEFAULT_DATAFMT  MEDIA_BUS_FMT_SBGGR10_1X10
#define AR0521_DEFAULT_CLK_FREQ 24000000


#define REG_ADDR_CHIP_VERSION           0x00
#define REG_ADDR_GAIN                   0x305e
#define REG_ADDR_EXPOSURE               0x3012
#define REG_ADDR_WIDTH                  0x034c
#define REG_ADDR_HEIGHT                 0x034e
#define REG_ADDR_OFFSET_X               0x3004
#define REG_ADDR_OFFSET_Y               0x3002
#define REG_ADDR_OFFSET_X_END               0x3008
#define REG_ADDR_OFFSET_Y_END               0x3006


#define REG_ADDR_FRAME_PREAMBLE         0x31b0
#define REG_ADDR_LINE_PREAMBLE          0x31b2


struct ar0521 {
    struct camera_common_power_rail power;
    int             numctrls;
    struct v4l2_ctrl_handler    ctrl_handler;
    struct i2c_client       *i2c_client;
    struct v4l2_subdev      *subdev;
    struct media_pad        pad;

    s32             group_hold_prev;
    u32             frame_length;
    bool                group_hold_en;
    bool            override_en;
    struct regmap           *regmap;
    struct camera_common_data   *s_data;
    struct camera_common_pdata  *pdata;
    struct v4l2_ctrl        *ctrls[];
};

static struct regmap_config ar0521_regmap_config = {
    .reg_bits = 16,
    .val_bits = 16,
};

static int ar0521_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int ar0521_s_ctrl(struct v4l2_ctrl *ctrl);
static int __ar0521_s_ctrl(struct ar0521 *priv, u32 id, s32 val);
static inline int ar0521_read_reg(struct camera_common_data *s_data, u16 addr, u16 *val);
static int ar0521_write_reg(struct camera_common_data *s_data, u16 addr, u16 val);
static int ar0521_set_gain(struct ar0521 *priv, s32 val);



static int ar0521_get_temper(struct ar0521 *priv, s32 *val)
{
    u16 data, calib1;
    ar0521_write_reg(priv->s_data, 0x3126, 0x0001);
    ar0521_write_reg(priv->s_data, 0x3126, 0x0021);
    ar0521_write_reg(priv->s_data, 0x3126, 0x0011);

    usleep_range(200, 210);

    ar0521_read_reg(priv->s_data, 0x3124, &data);
    ar0521_read_reg(priv->s_data, 0x3128, &calib1);

    printk("data = %d, calib1 = %d\n", data, calib1);
    *val = (data - calib1 + 90) * 100 / 151;

    return 0;
}

static const struct v4l2_ctrl_ops ar0521_ctrl_ops = {
    .g_volatile_ctrl = ar0521_g_volatile_ctrl,
    .s_ctrl     = ar0521_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
    /* Do not change the name field for the controls! */
    {
        .ops = &ar0521_ctrl_ops,
        .id = GALAXY_CAMERA_CID_CHIP_VERSION,
        .name = "Chip Version",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
        .min = AR0521_MIN_GAIN,
        .max = AR0521_MAX_GAIN,
        .def = AR0521_DEFAULT_GAIN,
        .step = 1,
    },
    {
        .ops = &ar0521_ctrl_ops,
        .id = GALAXY_CAMERA_CID_GAIN,
        .name = "Gain",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 0,
        .max = AR0521_MAX_GAIN - AR0521_MIN_GAIN,
        .def = 0,
        .step = 1,
    },
    {
        .ops = &ar0521_ctrl_ops,
        .id = GALAXY_CAMERA_CID_EXPOSURE,
        .name = "Exposure Time",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 0,
        .max = 65535,
        .def = 2268,
        .step = 1,
    },
    /*{
        .ops = &ar0521_ctrl_ops,
        .id = GALAXY_CAMERA_CID_WIDTH,
        .name = "Width",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 4,
        .max = 2600,
        .def = 2600,
        .step = 1,
    },
    {
        .ops = &ar0521_ctrl_ops,
        .id = GALAXY_CAMERA_CID_HEIGHT,
        .name = "Height",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 4,
        .max = 1952,
        .def = 1952,
        .step = 1,
    },*/
    {
        .ops = &ar0521_ctrl_ops,
        .id = GALAXY_CAMERA_CID_OFFSET_X,
        .name = "Offset X",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 4,
        .max = 2603,
        .def = 4,
        .step = 1,
    },
    {
        .ops = &ar0521_ctrl_ops,
        .id = GALAXY_CAMERA_CID_OFFSET_Y,
        .name = "Offset Y",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 4,
        .max = 1963,
        .def = 4,
        .step = 1,
    },
    {
        .ops = &ar0521_ctrl_ops,
        .id = GALAXY_CAMERA_CID_OFFSET_X_END,
        .name = "Offset X End",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 4,
        .max = 2603,
        .def = 2603,
        .step = 1,
    },
	{
		.ops = &ar0521_ctrl_ops,
		.id = TEGRA_CAMERA_CID_HDR_EN,
		.name = "HDR enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
    {
        .ops = &ar0521_ctrl_ops,
        .id = GALAXY_CAMERA_CID_OFFSET_Y_END,
        .name = "Offset Y End",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 4,
        .max = 1963,
        .def = 1963,
        .step = 1,
    },
    {
        .ops = &ar0521_ctrl_ops,
        .id = GALAXY_CAMERA_CID_CHIP_TEMPER,
        .name = "Chip Temperature",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
        .min = -100,
        .max = 100,
        .def = 0,
        .step = 1,
    },
    {
        .ops = &ar0521_ctrl_ops,
        .id = GALAXY_CAMERA_CID_FRAME_PREAMBLE,
        .name = "Frame Preamble",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 0,
        .max = 10000,
        .def = 91,
        .step = 1,
    },
    {
        .ops = &ar0521_ctrl_ops,
        .id = GALAXY_CAMERA_CID_LINE_PREAMBLE,
        .name = "Line Preamble",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 0,
        .max = 10000,
        .def = 80,
        .step = 1,
    },
    /*{
        .ops = &ar0521_ctrl_ops,
        .id = TEGRA_CAMERA_CID_FRAME_LENGTH,
        .name = "Frame Length",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = AR0521_MIN_FRAME_LENGTH,
        .max = AR0521_MAX_FRAME_LENGTH,
        .def = AR0521_DEFAULT_FRAME_LENGTH,
        .step = 1,
    },*/
};


static int test_mode;
module_param(test_mode, int, 0644);

static inline int ar0521_read_reg(struct camera_common_data *s_data,
                                  u16 addr, u16 *val)
{
    struct ar0521 *priv = (struct ar0521 *)s_data->priv;
    int err = 0;
    u32 reg_val = 0;

    err = regmap_read(priv->regmap, addr, &reg_val);
    *val = reg_val & 0xFFFF;

    return err;
}

static int ar0521_write_reg(struct camera_common_data *s_data, u16 addr, u16 val)
{
    int err;
    struct ar0521 *priv = (struct ar0521 *)s_data->priv;

    err = regmap_write(priv->regmap, addr, val);
    if (err)
        pr_err("%s:i2c write failed, %x = %x\n",
               __func__, addr, val);

    return err;
}

static int ar0521_write_table(struct ar0521 *priv, const struct reg_16 table[],
                              u16 wait_ms_addr, u16 end_addr)
{
    int err = 0;
    const struct reg_16 *next;

    for (next = table;; next++) {

        if (next->addr == end_addr)
            break;

        if (next->addr == wait_ms_addr) {
            msleep_range(next->val);
            continue;
        }

        err = regmap_write(priv->regmap, next->addr, next->val);
        if (err) {
            pr_err("%s:i2c write failed, %x = %x\n",
                   __func__, next->addr, next->val);
            return err;
        }
    }

    return 0;
}

static void ar0521_gpio_set(struct ar0521 *priv,
                            unsigned int gpio, int val)
{
    if (priv->pdata && priv->pdata->use_cam_gpio)
        cam_gpio_ctrl(&priv->i2c_client->dev, gpio, val, 1);
    else {
        if (gpio_cansleep(gpio))
            gpio_set_value_cansleep(gpio, val);
        else
            gpio_set_value(gpio, val);
    }
}

static int ar0521_power_on(struct camera_common_data *s_data)
{
    int err = 0;
    struct ar0521 *priv = (struct ar0521 *)s_data->priv;
    struct camera_common_power_rail *pw = &priv->power;

    dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);

    if (priv->pdata && priv->pdata->power_on) {
        err = priv->pdata->power_on(pw);
        if (err)
            pr_err("%s failed.\n", __func__);
        else
            pw->state = SWITCH_ON;
        return err;
    }

    /* Power up sequence as follow refers to chapter "POWER RESET AND STANDBY TIMING &
     * Power-Up Sequences" in <AR0521-D.PDF> */

    if (pw->reset_gpio)
        ar0521_gpio_set(priv, pw->reset_gpio, 0);

    // enable 1.8v iovdd
    if (pw->iovdd)
        err = regulator_enable(pw->iovdd);
    if (err)
        goto ar0521_iovdd_fail;

    if (pw->avdd)
        err = regulator_enable(pw->avdd);
    if (err)
        goto ar0521_avdd_fail;

    /*
     * datasheet "Power Up Sequence": After 1-500 ms, set RESET_N HIGH.
     */
    usleep_range(200, 210);

    if (pw->reset_gpio)
        ar0521_gpio_set(priv, pw->reset_gpio, 1);

    pw->state = SWITCH_ON;

    return 0;

ar0521_iovdd_fail:
    regulator_disable(pw->avdd);

ar0521_avdd_fail:
    pr_err("%s failed.\n", __func__);
    return -ENODEV;
}

static int ar0521_power_off(struct camera_common_data *s_data)
{
    int err = 0;
    struct ar0521 *priv = (struct ar0521 *)s_data->priv;
    struct camera_common_power_rail *pw = &priv->power;

    dev_dbg(&priv->i2c_client->dev, "%s: power off\n", __func__);

    /* Power down sequence as follow refers to chapter "POWER RESET AND STANDBY TIMING &
     * Power-Down Sequences" in <AR0521-D.PDF> */

    if (priv->pdata && priv->pdata->power_off) {
        err = priv->pdata->power_off(pw);
        if (!err) {
            goto power_off_done;
        } else {
            pr_err("%s failed.\n", __func__);
            return err;
        }
    }

    usleep_range(1, 2);
    if (pw->reset_gpio)
        ar0521_gpio_set(priv, pw->reset_gpio, 0);

    if (pw->iovdd)
        regulator_disable(pw->iovdd);
    if (pw->avdd)
        regulator_disable(pw->avdd);

power_off_done:
    pw->state = SWITCH_OFF;
    return 0;
}

static int ar0521_power_put(struct ar0521 *priv)
{
    struct camera_common_power_rail *pw = &priv->power;

    if (unlikely(!pw))
        return -EFAULT;

    if (likely(pw->avdd))
        regulator_put(pw->avdd);

    if (likely(pw->iovdd))
        regulator_put(pw->iovdd);

    pw->avdd = NULL;
    pw->iovdd = NULL;

    if (priv->pdata && priv->pdata->use_cam_gpio)
        cam_gpio_deregister(&priv->i2c_client->dev, pw->pwdn_gpio);
    else {
        gpio_free(pw->pwdn_gpio);
        gpio_free(pw->reset_gpio);
    }

    return 0;
}

static int ar0521_power_get(struct ar0521 *priv)
{
    struct camera_common_power_rail *pw = &priv->power;
    struct camera_common_pdata *pdata = priv->pdata;
    const char *mclk_name;
    const char *parentclk_name;
    struct clk *parent;
    int err = 0, ret = 0;

    if (!pdata) {
        dev_err(&priv->i2c_client->dev, "pdata missing\n");
        return -EFAULT;
    }

    mclk_name = pdata->mclk_name ?
                pdata->mclk_name : "cam_mclk1";
    pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
    if (IS_ERR(pw->mclk)) {
        dev_err(&priv->i2c_client->dev,
                "unable to get clock %s\n", mclk_name);
        return PTR_ERR(pw->mclk);
    }

    parentclk_name = pdata->parentclk_name;
    if (parentclk_name) {
        parent = devm_clk_get(&priv->i2c_client->dev, parentclk_name);
        if (IS_ERR(parent)) {
            dev_err(&priv->i2c_client->dev,
                    "unable to get parent clcok %s",
                    parentclk_name);
        } else
            clk_set_parent(pw->mclk, parent);
    }


    /* analog 2.8v */
    err |= camera_common_regulator_get(&priv->i2c_client->dev,
                                       &pw->avdd, pdata->regulators.avdd);
    /* IO 1.8v */
    err |= camera_common_regulator_get(&priv->i2c_client->dev,
                                       &pw->iovdd, pdata->regulators.iovdd);

    if (!err) {
        pw->reset_gpio = pdata->reset_gpio;
        pw->pwdn_gpio = pdata->pwdn_gpio;
    }

    if (pdata->use_cam_gpio) {
        err = cam_gpio_register(&priv->i2c_client->dev, pw->pwdn_gpio);
        if (err)
            dev_err(&priv->i2c_client->dev,
                    "%s ERR can't register cam gpio %u!\n",
                    __func__, pw->pwdn_gpio);
    } else {
        ret = gpio_request(pw->pwdn_gpio, "cam_pwdn_gpio");
        if (ret < 0)
            dev_dbg(&priv->i2c_client->dev,
                    "%s can't request pwdn_gpio %d\n",
                    __func__, ret);
        ret = gpio_request(pw->reset_gpio, "cam_reset_gpio");
        if (ret < 0)
            dev_dbg(&priv->i2c_client->dev,
                    "%s can't request reset_gpio %d\n",
                    __func__, ret);
    }


    pw->state = SWITCH_OFF;
    return err;
}

// static int ar0521_set_frame_length(struct ov5693 *priv, s32 val);
// static int ar0521_set_coarse_time(struct ov5693 *priv, s32 val);

static int ar0521_override_ctrl_config(struct ar0521 *priv)
{
	int numctrls;
    int err = 0;
	int err_temp = 0;
    int i = 0;
	struct v4l2_control control;

    dev_dbg(&priv->i2c_client->dev, "step into %s\n", __func__);
    numctrls = ARRAY_SIZE(ctrl_config_list);

	for (i = 0; i < numctrls; i++){
		if ((ctrl_config_list[i].flags & V4L2_CTRL_FLAG_SLIDER) == 0)		
			continue;
		control.id = ctrl_config_list[i].id;
		err_temp = v4l2_g_ctrl(&priv->ctrl_handler, &control);
		err_temp |= __ar0521_s_ctrl(priv, control.id, control.value);
		if (err_temp){
			printk("ctrl_config_list[i].flags = 0x%x\n", ctrl_config_list[i].flags);
			dev_dbg(&priv->i2c_client->dev, "%s: override %s = %d failed\n",
                    __func__, ctrl_config_list[i].name, control.value);
		}
		err |= err_temp;
	}

	return err;
}

static int ar0521_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct camera_common_data *s_data = to_camera_common_data(&client->dev);
    struct ar0521 *priv = (struct ar0521 *)s_data->priv;
    int err;
    u32 frame_time;
    u16 val;

    dev_dbg(&client->dev, "%s++\n", __func__);

    if (!enable) {
        //ar0521_update_ctrl_range(priv, AR0521_MAX_FRAME_LENGTH);

        dev_dbg(&client->dev, "%s: stream off\n", __func__);

        err = ar0521_read_reg(s_data, 0x306a, &val);
        if (err)
            return err;
        printk("datapath_status = 0x%x\n", val);

        err = ar0521_write_table(priv, mode_table[AR0521_MODE_STOP_STREAM],
                                 AR0521_TABLE_WAIT_MS, AR0521_TABLE_END);
        if (err)
            return err;

        /*
         * Wait for one frame to make sure sensor is set to
         * software standby in V-blank
         *
         * frame_time = frame length rows * Tline
         * Tline = line length / pixel clock (in MHz)
         */
        frame_time = priv->frame_length *
                     AR0521_DEFAULT_LINE_LENGTH / AR0521_DEFAULT_PIXEL_CLOCK;

        usleep_range(frame_time, frame_time + 1000);
        return 0;
    }

    err = ar0521_write_table(priv, mode_table[s_data->def_mode],
                             AR0521_TABLE_WAIT_MS, AR0521_TABLE_END);

    if (err)
        goto exit;


    if (priv->override_en) {
        err = ar0521_override_ctrl_config(priv);
        if (err)
            dev_dbg(&client->dev,
                    "%s: warning ar0521_override_ctrl_config failed\n",
                    __func__);

		err = __ar0521_s_ctrl(priv, GALAXY_CAMERA_CID_WIDTH, s_data->fmt_width);
		err |= __ar0521_s_ctrl(priv, GALAXY_CAMERA_CID_HEIGHT, s_data->fmt_height);
		if (err)
            dev_dbg(&client->dev,
                    "%s: warning width or height failed\n",
                    __func__);
    }

    err = ar0521_write_table(priv, mode_table[AR0521_MODE_START_STREAM],
                             AR0521_TABLE_WAIT_MS, AR0521_TABLE_END);
    if (err)
        goto exit;
#if 0
    if (priv->pdata->v_flip) {
        ar0521_read_reg(priv->s_data, AR0521_TIMING_REG20, &val);
        ar0521_write_reg(priv->s_data, AR0521_TIMING_REG20,
                         val | VERTICAL_FLIP);
    }
    if (priv->pdata->h_mirror) {
        ar0521_read_reg(priv->s_data, AR0521_TIMING_REG21, &val);
        ar0521_write_reg(priv->s_data, AR0521_TIMING_REG21,
                         val | HORIZONTAL_MIRROR_MASK);
    } else {
        ar0521_read_reg(priv->s_data, AR0521_TIMING_REG21, &val);
        ar0521_write_reg(priv->s_data, AR0521_TIMING_REG21,
                         val & (~HORIZONTAL_MIRROR_MASK));
    }
    if (test_mode)
        err = ar0521_write_table(priv,
                                 mode_table[AR0521_MODE_TEST_PATTERN]);
#endif

    dev_dbg(&client->dev, "%s--\n", __func__);
    return 0;
exit:
    dev_dbg(&client->dev, "%s: error setting stream\n", __func__);
    return err;
}

static int ar0521_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct ar0521 *priv = (struct ar0521 *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}

static struct v4l2_subdev_video_ops ar0521_subdev_video_ops = {
    .s_stream   = ar0521_s_stream,
    .g_mbus_config  = camera_common_g_mbus_config,
    .g_input_status = ar0521_g_input_status,
};

static struct v4l2_subdev_core_ops ar0521_subdev_core_ops = {
    .s_power    = camera_common_s_power,
};

static int ar0521_get_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_pad_config *cfg,
                          struct v4l2_subdev_format *format)
{
    return camera_common_g_fmt(sd, &format->format);
}

static int ar0521_set_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_pad_config *cfg,
                          struct v4l2_subdev_format *format)
{
    int ret = 0;
	/*struct camera_common_data *common_data;
    struct ar0521 *priv;

	common_data = container_of(sd, struct camera_common_data, subdev);
	common_data->fmt_width = format->format.width;
	common_data->fmt_height = format->format.height;*/

    if (format->which == V4L2_SUBDEV_FORMAT_TRY)
        ret = camera_common_try_fmt(sd, &format->format);
    else
        ret = camera_common_s_fmt(sd, &format->format);

    return ret;
}

static struct v4l2_subdev_pad_ops ar0521_subdev_pad_ops = {
    .set_fmt = ar0521_set_fmt,
    .get_fmt = ar0521_get_fmt,
    .enum_mbus_code = camera_common_enum_mbus_code,
    .enum_frame_size    = camera_common_enum_framesizes,
    .enum_frame_interval    = camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops ar0521_subdev_ops = {
    .core   = &ar0521_subdev_core_ops,
    .video  = &ar0521_subdev_video_ops,
    .pad    = &ar0521_subdev_pad_ops,
};

static struct of_device_id ar0521_of_match[] = {
    { .compatible = "daheng,ar0521", },
    { },
};

static struct camera_common_sensor_ops ar0521_common_ops = {
    .power_on = ar0521_power_on,
    .power_off = ar0521_power_off,
};

static int ar0521_set_gain(struct ar0521 *priv, s32 val)
{
    int err = 0;
    err = regmap_write(priv->regmap, REG_ADDR_GAIN, val + AR0521_MIN_GAIN);
    if (err) {
        pr_err("%s: GAIN control error\n", __func__);
        return err;
    }

    return 0;
}

/*static int ar0521_set_frame_length(struct ar0521 *priv, s32 val)
{
    ov5693_reg reg_list[2];
    int err;
    u32 frame_length;
    int i;

    if (!priv->group_hold_prev)
        ov5693_set_group_hold(priv);

    frame_length = (u32)val;

    ov5693_get_frame_length_regs(reg_list, frame_length);
    dev_dbg(&priv->i2c_client->dev,
         "%s: val: %d\n", __func__, frame_length);

    for (i = 0; i < 2; i++) {
        err = ov5693_write_reg(priv->s_data, reg_list[i].addr,
             reg_list[i].val);
        if (err)
            goto fail;
    }

    priv->frame_length = frame_length;

    ov5693_update_ctrl_range(priv, val);
    return 0;

fail:
    dev_dbg(&priv->i2c_client->dev,
         "%s: FRAME_LENGTH control error\n", __func__);
    return err;
}

static int ar0521_set_coarse_time(struct ar0521 *priv, s32 val)
{
    ov5693_reg reg_list[3];
    int err;
    u32 coarse_time;
    int i;

    if (!priv->group_hold_prev)
        ov5693_set_group_hold(priv);

    coarse_time = (u32)val;

    ov5693_get_coarse_time_regs(reg_list, coarse_time);
    dev_dbg(&priv->i2c_client->dev,
         "%s: val: %d\n", __func__, coarse_time);

    for (i = 0; i < 3; i++) {
        err = ov5693_write_reg(priv->s_data, reg_list[i].addr,
             reg_list[i].val);
        if (err)
            goto fail;
    }

    return 0;

fail:
    dev_dbg(&priv->i2c_client->dev,
         "%s: COARSE_TIME control error\n", __func__);
    return err;
}*/

static int ar0521_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
    struct ar0521 *priv =
        container_of(ctrl->handler, struct ar0521, ctrl_handler);
    int err = 0;

    if (priv->power.state == SWITCH_OFF)
        return 0;

    switch (ctrl->id) {
    case GALAXY_CAMERA_CID_CHIP_VERSION:
        err = regmap_read(priv->regmap, REG_ADDR_CHIP_VERSION, &ctrl->val);
        if (err)
            return err;
        break;
    case GALAXY_CAMERA_CID_CHIP_TEMPER:
        err = ar0521_get_temper(priv, &ctrl->val);
        if (err)
            return err;
        break;
    default:
        pr_err("%s: unknown ctrl id.\n", __func__);
        return -EINVAL;
    }

    return err;
}

static int __ar0521_s_ctrl(struct ar0521 *priv, u32 id, s32 val)
{
	int err = 0;
	switch (id) {
    case GALAXY_CAMERA_CID_GAIN:
        err = ar0521_set_gain(priv, val);
        break;
	case GALAXY_CAMERA_CID_EXPOSURE:
		err = regmap_write(priv->regmap, REG_ADDR_EXPOSURE, val);
		break;
	case GALAXY_CAMERA_CID_WIDTH:
        err = regmap_write(priv->regmap, REG_ADDR_WIDTH, val);
        break;
	case GALAXY_CAMERA_CID_HEIGHT:
        err = regmap_write(priv->regmap, REG_ADDR_HEIGHT, val);
        break;
	case GALAXY_CAMERA_CID_OFFSET_X:
        err = regmap_write(priv->regmap, REG_ADDR_OFFSET_X, val);
        break;
	case GALAXY_CAMERA_CID_OFFSET_Y:
        err = regmap_write(priv->regmap, REG_ADDR_OFFSET_Y, val);
        break;
	case GALAXY_CAMERA_CID_OFFSET_X_END:
        err = regmap_write(priv->regmap, REG_ADDR_OFFSET_X_END, val);
        break;
	case GALAXY_CAMERA_CID_OFFSET_Y_END:
        err = regmap_write(priv->regmap, REG_ADDR_OFFSET_Y_END, val);
        break;
	case TEGRA_CAMERA_CID_HDR_EN:
		dev_dbg(&priv->i2c_client->dev,
         "%s: GALAXY_CAMERA_CID_HDR_EN is not implemented!\n", __func__);
		break;
    case GALAXY_CAMERA_CID_FRAME_PREAMBLE:
        err = regmap_write(priv->regmap, REG_ADDR_FRAME_PREAMBLE, val);
        break;
    case GALAXY_CAMERA_CID_LINE_PREAMBLE:
        err = regmap_write(priv->regmap, REG_ADDR_LINE_PREAMBLE, val);
        break;
    default:
        pr_err("%s: unknown ctrl id.\n", __func__);
        return -EINVAL;
    }

    return err;
}

static int ar0521_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct ar0521 *priv =
        container_of(ctrl->handler, struct ar0521, ctrl_handler);
	
    if (priv->power.state == SWITCH_OFF)
        return 0;
	
	return __ar0521_s_ctrl(priv, ctrl->id, ctrl->val);
}

static int ar0521_ctrls_init(struct ar0521 *priv)
{
    struct i2c_client *client = priv->i2c_client;
    struct v4l2_ctrl *ctrl;
    int numctrls;
    int err;
    int i;

    dev_dbg(&client->dev, "%s++\n", __func__);

    numctrls = ARRAY_SIZE(ctrl_config_list);
    v4l2_ctrl_handler_init(&priv->ctrl_handler, numctrls);

    for (i = 0; i < numctrls; i++) {

        ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,
                                    &ctrl_config_list[i], NULL);
        if (ctrl == NULL) {
            dev_err(&client->dev, "Failed to init %s ctrl\n",
                    ctrl_config_list[i].name);
            continue;
        }

        if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
                ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
            ctrl->p_new.p_char = devm_kzalloc(&client->dev,
                                              ctrl_config_list[i].max + 1, GFP_KERNEL);
            if (!ctrl->p_new.p_char)
                return -ENOMEM;
        }
        priv->ctrls[i] = ctrl;
    }

    priv->numctrls = numctrls;
    priv->subdev->ctrl_handler = &priv->ctrl_handler;
    if (priv->ctrl_handler.error) {
        dev_err(&client->dev, "Error %d adding controls\n",
                priv->ctrl_handler.error);
        err = priv->ctrl_handler.error;
        goto error;
    }

    err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
    if (err) {
        dev_err(&client->dev,
                "Error %d setting default controls\n", err);
        goto error;
    }

    return 0;

error:
    v4l2_ctrl_handler_free(&priv->ctrl_handler);
    return err;
}

MODULE_DEVICE_TABLE(of, ar0521_of_match);

static struct camera_common_pdata *ar0521_parse_dt(struct i2c_client *client)
{
    struct device_node *node = client->dev.of_node;
    struct camera_common_pdata *board_priv_pdata;
    const struct of_device_id *match;
    int gpio;
    int err;
    struct camera_common_pdata *ret = NULL;

    if (!node)
        return NULL;

    match = of_match_device(ar0521_of_match, &client->dev);
    if (!match) {
        dev_err(&client->dev, "Failed to find matching dt id\n");
        return NULL;
    }

    board_priv_pdata = devm_kzalloc(&client->dev,
                                    sizeof(*board_priv_pdata), GFP_KERNEL);
    if (!board_priv_pdata)
        return NULL;

    err = camera_common_parse_clocks(&client->dev, board_priv_pdata);
    if (err) {
        dev_err(&client->dev, "Failed to find clocks\n");
        goto error;
    }

    gpio = of_get_named_gpio(node, "pwdn-gpios", 0);
    if (gpio < 0) {
        if (gpio == -EPROBE_DEFER) {
            ret = ERR_PTR(-EPROBE_DEFER);
            goto error;
        }
        dev_err(&client->dev, "pwdn gpios not in DT\n");
        goto error;
    }
    board_priv_pdata->pwdn_gpio = (unsigned int)gpio;

    gpio = of_get_named_gpio(node, "reset-gpios", 0);
    if (gpio < 0) {
        /* reset-gpio is not absolutely needed */
        if (gpio == -EPROBE_DEFER) {
            ret = ERR_PTR(-EPROBE_DEFER);
            goto error;
        }
        dev_dbg(&client->dev, "reset gpios not in DT\n");
        gpio = 0;
    }
    board_priv_pdata->reset_gpio = (unsigned int)gpio;

    board_priv_pdata->use_cam_gpio =
        of_property_read_bool(node, "cam,use-cam-gpio");

    err = of_property_read_string(node, "avdd-reg",
                                  &board_priv_pdata->regulators.avdd);
    if (err) {
        dev_err(&client->dev, "avdd-reg not in DT\n");
        goto error;
    }
    err = of_property_read_string(node, "iovdd-reg",
                                  &board_priv_pdata->regulators.iovdd);
    if (err) {
        dev_err(&client->dev, "iovdd-reg not in DT\n");
        goto error;
    }

    board_priv_pdata->has_eeprom =
        of_property_read_bool(node, "has-eeprom");
    board_priv_pdata->v_flip = of_property_read_bool(node, "vertical-flip");
    board_priv_pdata->h_mirror = of_property_read_bool(node,
                                 "horizontal-mirror");
    return board_priv_pdata;

error:
    devm_kfree(&client->dev, board_priv_pdata);
    return ret;
}

static int ar0521_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    dev_dbg(&client->dev, "%s:\n", __func__);
    return 0;
}

static const struct v4l2_subdev_internal_ops ar0521_subdev_internal_ops = {
    .open = ar0521_open,
};

static const struct media_entity_operations ar0521_media_ops = {
    .link_validate = v4l2_subdev_link_validate,
};

static int ar0521_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    struct camera_common_data *common_data;
    struct device_node *node = client->dev.of_node;
    struct ar0521 *priv;
    int err;

    pr_info("[ar0521]: probing v4l2 sensor.\n");

    if (!IS_ENABLED(CONFIG_OF) || !node)
        return -EINVAL;

    common_data = devm_kzalloc(&client->dev,
                               sizeof(struct camera_common_data), GFP_KERNEL);
    if (!common_data)
        return -ENOMEM;

    priv = devm_kzalloc(&client->dev,
                        sizeof(struct ar0521) + sizeof(struct v4l2_ctrl *) *
                        ARRAY_SIZE(ctrl_config_list),
                        GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    priv->regmap = devm_regmap_init_i2c(client, &ar0521_regmap_config);
    if (IS_ERR(priv->regmap)) {
        dev_err(&client->dev,
                "regmap init failed: %ld\n", PTR_ERR(priv->regmap));
        return -ENODEV;
    }

    priv->pdata = ar0521_parse_dt(client);
    if (PTR_ERR(priv->pdata) == -EPROBE_DEFER)
        return -EPROBE_DEFER;
    if (!priv->pdata) {
        dev_err(&client->dev, "unable to get platform data\n");
        return -EFAULT;
    }

    common_data->ops        = &ar0521_common_ops;
    common_data->ctrl_handler   = &priv->ctrl_handler;
    common_data->dev        = &client->dev;
	common_data->frmfmt		= ar0521_frmfmt;
    common_data->colorfmt       = camera_common_find_datafmt(
                                      AR0521_DEFAULT_DATAFMT);
    common_data->power      = &priv->power;
    common_data->ctrls      = priv->ctrls;
    common_data->priv       = (void *)priv;
    common_data->numctrls       = ARRAY_SIZE(ctrl_config_list);
    common_data->numfmts        = ARRAY_SIZE(ar0521_frmfmt);
    common_data->def_mode       = AR0521_DEFAULT_MODE;
    common_data->def_width      = AR0521_DEFAULT_WIDTH;
    common_data->def_height     = AR0521_DEFAULT_HEIGHT;
    common_data->fmt_width      = common_data->def_width;
    common_data->fmt_height     = common_data->def_height;
    common_data->def_clk_freq   = AR0521_DEFAULT_CLK_FREQ;

    priv->i2c_client = client;
    priv->s_data            = common_data;
    priv->subdev            = &common_data->subdev;
    priv->subdev->dev       = &client->dev;
    priv->s_data->dev       = &client->dev;
    priv->override_en       = true;

    err = ar0521_power_get(priv);
    if (err)
        return err;

    err = camera_common_initialize(common_data, "ar0521");
    if (err) {
        dev_err(&client->dev, "Failed to initialize ar0521.\n");
        return err;
    }

    v4l2_i2c_subdev_init(priv->subdev, client, &ar0521_subdev_ops);

    err = ar0521_ctrls_init(priv);
    if (err)
        return err;

    priv->subdev->internal_ops = &ar0521_subdev_internal_ops;
    priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
                           V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
    priv->pad.flags = MEDIA_PAD_FL_SOURCE;
    priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
    priv->subdev->entity.ops = &ar0521_media_ops;
    err = media_entity_init(&priv->subdev->entity, 1, &priv->pad, 0);
    if (err < 0) {
        dev_err(&client->dev, "unable to init media entity\n");
        return err;
    }
#endif

    err = v4l2_async_register_subdev(priv->subdev);
    if (err)
        return err;

    dev_dbg(&client->dev, "Detected ar0521 sensor\n");


    return 0;
}

static int
ar0521_remove(struct i2c_client *client)
{
    struct camera_common_data *s_data = to_camera_common_data(&client->dev);
    struct ar0521 *priv = (struct ar0521 *)s_data->priv;

    v4l2_async_unregister_subdev(priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
    media_entity_cleanup(&priv->subdev->entity);
#endif

    v4l2_ctrl_handler_free(&priv->ctrl_handler);
    ar0521_power_put(priv);
    camera_common_cleanup(s_data);

    return 0;
}

static const struct i2c_device_id ar0521_id[] = {
    { "ar0521", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, ar0521_id);

static struct i2c_driver ar0521_i2c_driver = {
    .driver = {
        .name = "ar0521",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(ar0521_of_match),
    },
    .probe = ar0521_probe,
    .remove = ar0521_remove,
    .id_table = ar0521_id,
};

module_i2c_driver(ar0521_i2c_driver);

MODULE_DESCRIPTION("MIPI Camera driver for ar0521");
MODULE_AUTHOR("Galaxy");
MODULE_LICENSE("GPL v2");

