/*
 * imx334.c - imx334 sensor driver
 *
 * Copyright (C) 2021, Leopard  <leopard@leopardimaging.com>
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

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include "imx334_mode_tbls.h"

#define IMX334_MIN_FRAME_LENGTH	(2250)
#define IMX334_MAX_FRAME_LENGTH	(0x1FFFF)
#define IMX334_SW_RESET_ADDR				0x3003
#define IMX334_SLAVE_ADDRESS				0x37
#define IMX334_TEMP_ADDRESS				0x49

#define IMX334_FRAME_LENGTH_ADDR_MSB			0x3032 /* VMAX, MSB */
#define IMX334_FRAME_LENGTH_ADDR_MID			0x3031 /* VMAX, MID */
#define IMX334_FRAME_LENGTH_ADDR_LSB			0x3030 /* VMAX, LSB */

#define IMX334_COARSE_TIME_ADDR_MSB			0x305A /* SHS1 */
#define IMX334_COARSE_TIME_ADDR_MID			0x3059 /* SHS1 */
#define IMX334_COARSE_TIME_ADDR_LSB			0x3058 /* SHS1 */

#define IMX334_GROUP_HOLD_ADDR				0x3001 /* REG HOLD */

#define IMX334_ANALOG_GAIN_ADDR_MSB			0x30E9 /* GAIN ADDR */
#define IMX334_ANALOG_GAIN_ADDR_LSB			0x30E8 /* GAIN ADDR */

#define IMX334_TEMP_REGISTER_ADDR			0x00
#define IMX334_TEMP_CONFIG_ADDR				0x01
#define IMX334_TEMP_LOW_ADDR				0x02
#define IMX334_TEMP_HIGH_ADDR				0x03
#define IMX334_TEMP_HIGH_MAX				80
#define IMX334_TEMP_HIGH_MIN				20
#define IMX334_TEMP_HIGH_DEFAULT			50

static const struct of_device_id imx334_of_match[] = {
	{ .compatible = "nvidia,imx334",},
	{ },
};
MODULE_DEVICE_TABLE(of, imx334_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct imx334 {
	struct i2c_client	*i2c_client;
	struct i2c_client 	*tem_i2c_client;
	struct v4l2_subdev	*subdev;
	u32				frame_length;
	u32 	sensor_id;
	s64 last_wdr_et_val;
	const char 		*channel;
	int 			heater_gpio;
	bool 			heater_enable;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
	struct regmap 				*temp_regmap;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
#if KERNEL_VERSION(5, 4, 0) > LINUX_VERSION_CODE
	.use_single_rw = true,
#else
	.use_single_read = true,
	.use_single_write = true,
#endif
};
static const struct regmap_config temp_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.cache_type = REGCACHE_NONE,
#if KERNEL_VERSION(5, 4, 0) > LINUX_VERSION_CODE
	.use_single_rw = true,
#else
	.use_single_read = true,
	.use_single_write = true,
#endif
};

static inline void imx334_get_frame_length_regs(imx334_reg *regs,
				u32 frame_length)
{
	regs->addr = IMX334_FRAME_LENGTH_ADDR_MSB;
	regs->val = (frame_length >> 16) & 0x0F;
	(regs + 1)->addr = IMX334_FRAME_LENGTH_ADDR_MID;
	(regs + 1)->val = (frame_length >> 8) & 0xFF;
	(regs + 2)->addr = IMX334_FRAME_LENGTH_ADDR_LSB;
	(regs + 2)->val = (frame_length) & 0xFF;
}

static inline void imx334_get_coarse_time_regs_shs1(imx334_reg *regs,
				u32 coarse_time)
{
	regs->addr = IMX334_COARSE_TIME_ADDR_MSB;
	regs->val = (coarse_time >> 16) & 0x0f;
	(regs + 1)->addr = IMX334_COARSE_TIME_ADDR_MID;
	(regs + 1)->val = (coarse_time >> 8) & 0xff;
	(regs + 2)->addr = IMX334_COARSE_TIME_ADDR_LSB;
	(regs + 2)->val = (coarse_time) & 0xff;

}

static inline void imx334_get_gain_reg(imx334_reg *regs,
				u16 gain)
{
	regs->addr = IMX334_ANALOG_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0xff;
	(regs + 1)->addr = IMX334_ANALOG_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

static int test_mode;
module_param(test_mode, int, 0644);

static inline int imx334_temp_read_reg(struct imx334 *priv,
				u8 addr, u16 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(priv->temp_regmap, addr, &reg_val);
	*val = reg_val & 0xFFFF;

	return err;
}

static int imx334_temp_write_reg(struct imx334 *priv,
				u8 addr, u16 val)
{
	int err;

	err = regmap_write(priv->temp_regmap, addr, val);

	return err;
}

static inline int imx334_read_reg(struct camera_common_data *s_data,
				u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;

	return err;
}

static int imx334_write_reg(struct camera_common_data *s_data,
				u16 addr, u8 val)
{
	int err;
	struct device *dev = s_data->dev;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(dev, "%s: i2c write failed, 0x%x = %x\n",
			__func__, addr, val);

	return err;
}

static int imx334_write_table(struct imx334 *priv,
				const imx334_reg table[])
{
	struct camera_common_data *s_data = priv->s_data;

	return regmap_util_write_table_8(s_data->regmap,
					 table,
					 NULL, 0,
					 IMX334_TABLE_WAIT_MS,
					 IMX334_TABLE_END);
}

static int imx334_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int err;

	err = imx334_write_reg(s_data,
				IMX334_GROUP_HOLD_ADDR, val);
	if (err) {
		dev_dbg(dev,
			"%s: Group hold control error\n", __func__);
		return err;
	}

	return 0;
}

static int imx334_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx334 *priv = (struct imx334 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
	imx334_reg reg_list[2];
	int err;
	int i;
	u16 gain;

	/* translate value */
	gain = (u16) (val * 160 / (48 * mode->control_properties.gain_factor));

	imx334_get_gain_reg(reg_list, gain);
	for (i = 0; i < 2; i++) {
		err = imx334_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: GAIN control error\n", __func__);
	return 0;
}

static int imx334_set_coarse_time(struct imx334 *priv, s64 val)
{
	struct camera_common_data *s_data = priv->s_data;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode];
	struct device *dev = &priv->i2c_client->dev;
	imx334_reg reg_list[3];
	int err;
	u32 coarse_time_shs1;
	u32 reg_shs1;
	int i = 0;

	coarse_time_shs1 = mode->signal_properties.pixel_clock.val *
		val / mode->image_properties.line_length /
		mode->control_properties.exposure_factor;

	if (priv->frame_length == 0)
		priv->frame_length = imx334_frame_length[s_data->mode_prop_idx];

	reg_shs1 = priv->frame_length - coarse_time_shs1 - 1;

	if (reg_shs1 < 5)
		reg_shs1 = 5;
	else if (reg_shs1 > priv->frame_length - 5)
		reg_shs1 = priv->frame_length - 5;

	dev_dbg(dev, "%s: coarse1:%d, shs1:%d, FL:%d\n", __func__,
		 coarse_time_shs1, reg_shs1, priv->frame_length);

	imx334_get_coarse_time_regs_shs1(reg_list, reg_shs1);

	for (i = 0; i < 3; i++) {
		err = imx334_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: set coarse time error\n", __func__);
	return err;
}

static int imx334_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx334 *priv = (struct imx334 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	imx334_reg reg_list[3];
	int i = 0;
	int err;
	u32 frame_length;
	const struct sensor_mode_properties *mode =
		&s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];

	//keep fixed frame rate
	return 0;
	frame_length = mode->signal_properties.pixel_clock.val *
		mode->control_properties.framerate_factor /
		mode->image_properties.line_length / val;

	priv->frame_length = frame_length;

	if (priv->frame_length < IMX334_MIN_FRAME_LENGTH)
		priv->frame_length = IMX334_MIN_FRAME_LENGTH;
	else if (priv->frame_length > IMX334_MAX_FRAME_LENGTH)
		priv->frame_length = IMX334_MAX_FRAME_LENGTH;

	dev_dbg(dev, "%s: val: %lld, , frame_length: %d\n", __func__,
		val, priv->frame_length);

	imx334_get_frame_length_regs(reg_list, priv->frame_length);

	for (i = 0; i < 3; i++) {
		err = imx334_write_reg(priv->s_data, reg_list[i].addr,
			 reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	dev_dbg(dev, "%s: FRAME_LENGTH control error\n", __func__);
	return err;
}

static int imx334_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	struct imx334 *priv = (struct imx334 *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	int err;

	dev_dbg(dev, "%s: val: %lld\n", __func__, val);
	err = imx334_set_coarse_time(priv, val);
	if (err)
		dev_dbg(dev,
		"%s: error coarse time SHS1 override\n", __func__);

	return 0;
}

static struct tegracam_ctrl_ops imx334_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = imx334_set_gain,
	.set_exposure = imx334_set_exposure,
	.set_frame_rate = imx334_set_frame_rate,
	.set_group_hold = imx334_set_group_hold,
};

static int imx334_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);
	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	/*exit reset mode: XCLR */
	if (pw->reset_gpio) {
		// gpio_set_value(pw->reset_gpio, 0);
		// usleep_range(30, 50);
		// gpio_set_value(pw->reset_gpio, 1);
		// usleep_range(30, 50);
	}

	pw->state = SWITCH_ON;
	return 0;

}

static int imx334_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (!err)
			goto power_off_done;
		else
			dev_err(dev, "%s failed.\n", __func__);
		return err;
	}
	/* enter reset mode: XCLR */
	usleep_range(1, 2);
	// if (pw->reset_gpio)
	// 	gpio_set_value(pw->reset_gpio, 0);

power_off_done:
	pw->state = SWITCH_OFF;

	return 0;
}

static int imx334_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	const char *mclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = pdata->mclk_name ?
		    pdata->mclk_name : "extperiph1";
	pw->mclk = devm_clk_get(dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(dev, "unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parent = devm_clk_get(dev, "pllp_grtba");
	if (IS_ERR(parent))
		dev_err(dev, "devm_clk_get failed for pllp_grtba");
	else
		clk_set_parent(pw->mclk, parent);

	pw->reset_gpio = pdata->reset_gpio;

	pw->state = SWITCH_OFF;
	return err;
}

static int imx334_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	return 0;
}

static ssize_t imx334_heater_debugfs_write(struct file *s,
				const char __user *user_buf,
				size_t count, loff_t *ppos)
{
   struct imx334 *priv =
    ((struct seq_file *)s->private_data)->private;
	char buf[255];
	int buf_size;

	if (!user_buf || count <= 1)
		return -EFAULT;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	if (buf[0] == '1') {
		gpio_direction_output(priv->heater_gpio, 1);
		gpio_set_value(priv->heater_gpio, 1);
		priv->heater_enable = true;
		return count;
	}

	if (buf[0] == '0') {
		gpio_direction_output(priv->heater_gpio, 0);
		gpio_set_value(priv->heater_gpio, 0);
		priv->heater_enable = false;
		return count;
	}

	return count;
}

static ssize_t imx334_heater_debugfs_read(struct file *s,
				char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct imx334 *priv =
		((struct seq_file *)s->private_data)->private;
	// int buf_size;
	char buf[50];

	if (!user_buf || count <= 1)
		return -EFAULT;

	if (priv->heater_enable == false)
		snprintf(buf, sizeof(buf), "Heater Off(1:on, 0:off)\n");
	else
		snprintf(buf, sizeof(buf), "Heater On(1:on, 0:off)\n");

	if (clear_user(user_buf, count)) {
		return -EIO;
	}

	return simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
}

static ssize_t imx334_temp_debugfs_read(struct file *s,
				char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct imx334 *priv =
		((struct seq_file *)s->private_data)->private;
	int err;
	char buf[32];
	u16 val = 0;
	int temp_real;
	short temp;

	if (!user_buf || count <= 1)
		return -EFAULT;

	err = imx334_temp_read_reg(priv, IMX334_TEMP_REGISTER_ADDR, &val);

	//12-bit mode
	temp = (short)val;
	temp_real = ((int)temp >> 4) * 625;// uint 0.0001 degree
	if (err)
		snprintf(buf, sizeof(buf), "It is not able to access\n");
	else
		snprintf(buf, sizeof(buf), "%d.%04d degree\n", temp_real/10000, temp_real%10000);

	if (clear_user(user_buf, count)) {
		return -EIO;
	}

	return simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
}

static int imx334_set_temp_high(struct imx334 *priv, u16 t_high)
{
	u16 t_low;

	t_low = t_high - 5; // for recovery
	t_high = (u16)((u32)t_high * 10000 / 625 << 4);
	t_low = (u16)((u32)t_low * 10000 / 625 << 4);
	imx334_temp_write_reg(priv, IMX334_TEMP_HIGH_ADDR, t_high);
	imx334_temp_write_reg(priv, IMX334_TEMP_LOW_ADDR, t_low);
	return 0;
}

static ssize_t imx334_temp_high_debugfs_write(struct file *s,
				const char __user *user_buf,
				size_t count, loff_t *ppos)
{
   struct imx334 *priv =
    ((struct seq_file *)s->private_data)->private;
	char buf[255];
	int buf_size;
	short t_high;
	int tmp;

	if (!user_buf || count <= 1)
		return -EFAULT;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	if(kstrtoint(buf, 0, &tmp))
		return -EFAULT;
	tmp = (tmp > IMX334_TEMP_HIGH_MAX) ? IMX334_TEMP_HIGH_MAX : tmp;
	tmp = (tmp < IMX334_TEMP_HIGH_MIN) ? IMX334_TEMP_HIGH_MIN : tmp;
	t_high = (short)tmp;
	imx334_set_temp_high(priv, (u16)t_high);

	return count;
}
static ssize_t imx334_temp_high_debugfs_read(struct file *s,
				char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct imx334 *priv =
		((struct seq_file *)s->private_data)->private;
	int err;
	char buf[32];
	u16 val = 0;
	int t_high;
	short temp;

	if (!user_buf || count <= 1)
		return -EFAULT;

	err = imx334_temp_read_reg(priv, IMX334_TEMP_HIGH_ADDR, &val);
	//12-bit mode
	temp = (short)val;
	t_high = ((int)temp >> 4) * 625;// uint 0.0001 degree
	if (err)
		snprintf(buf, sizeof(buf), "It is not able to access\n");
	else
		snprintf(buf, sizeof(buf), "%d.%04d degree\n", t_high/10000, t_high%10000);

	if (clear_user(user_buf, count)) {
		return -EIO;
	}

	return simple_read_from_buffer(user_buf, count, ppos, buf, strlen(buf));
}

static int imx334_stats_show(struct seq_file *s, void *data)
{
	return 0;
}

static int imx334_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, imx334_stats_show, inode->i_private);
}

static const struct file_operations imx334_heater_debugfs_fops = {
	.open = imx334_debugfs_open,
	.read = imx334_heater_debugfs_read,
	.write = imx334_heater_debugfs_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations imx334_temp_debugfs_fops = {
	.open = imx334_debugfs_open,
	.read = imx334_temp_debugfs_read,
	// .write = imx334_temp_debugfs_write,
	.write = NULL,
	.llseek = seq_lseek,
	.release = single_release,
};
static const struct file_operations imx334_temp_high_debugfs_fops = {
	.open = imx334_debugfs_open,
	.read = imx334_temp_high_debugfs_read,
	.write = imx334_temp_high_debugfs_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int imx334_debugfs_init(const char *dir_name,
					struct imx334 *priv)
{
	struct dentry  *dp, *fp;
	char dev_name[20];
	int err;
	struct i2c_client *i2c_client = priv->i2c_client;
	struct device_node *np = i2c_client->dev.of_node;

	if (np) {
		err = of_property_read_string(np, "channel", &priv->channel);
		if (err)
			dev_err(&i2c_client->dev, "channel not found\n");
		snprintf(dev_name, sizeof(dev_name), "IMX334_%s",
			priv->channel);
	}

	dp = debugfs_create_dir(dev_name, NULL);
	if (dp == NULL) {
		dev_err(&i2c_client->dev, "%s: debugfs create dir failed\n",
			__func__);
		return -ENOMEM;
	}

	fp = debugfs_create_file("heater_on", S_IRUGO|S_IWUSR,
		dp, priv, &imx334_heater_debugfs_fops);
	if (!fp) {
		dev_err(&i2c_client->dev, "%s: debugfs create file failed\n",
			__func__);
		debugfs_remove_recursive(dp);
		return -ENOMEM;
	}
	fp = debugfs_create_file("temperature", S_IRUGO|S_IWUSR,
		dp, priv, &imx334_temp_debugfs_fops);
	if (!fp) {
		dev_err(&i2c_client->dev, "%s: debugfs create file failed\n",
			__func__);
		debugfs_remove_recursive(dp);
		return -ENOMEM;
	}
	fp = debugfs_create_file("t_high", S_IRUGO|S_IWUSR,
		dp, priv, &imx334_temp_high_debugfs_fops);
	if (!fp) {
		dev_err(&i2c_client->dev, "%s: debugfs create file failed\n",
			__func__);
		debugfs_remove_recursive(dp);
		return -ENOMEM;
	}

	return 0;
}

static struct camera_common_pdata *imx334_parse_dt(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	int err;
	int gpio;

	if (!np)
		return NULL;

	match = of_match_device(imx334_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = of_property_read_string(np, "mclk",
				      &board_priv_pdata->mclk_name);
	if (err)
		dev_err(dev, "mclk not in DT\n");

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "reset-gpios not found %d\n", err);
		goto error;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;
	gpio_set_value(gpio, 1);

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);
	return ret;
}

static int  imx334_device_id(struct device *dev,
				struct imx334 *priv)
{
	struct device_node *np = dev->of_node;
	const struct of_device_id *match;
	int err;
	const char *str;

	match = of_match_device(imx334_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return -1;
	}
	err = of_property_read_string(np, "sensor_operation_mode", &str);
	if (!err) {
		if (!strcmp(str, "master")) {
			priv->sensor_id = 0;
		} else if (!strcmp(str, "slave")) {
			priv->sensor_id = 1;
		}
	} else
		dev_err(dev, "Failed to find sensor_operation_mode\n");

	return priv->sensor_id;
}
static int imx334_set_mode(struct tegracam_device *tc_dev)
{
	struct imx334 *priv = (struct imx334 *)tegracam_get_privdata(tc_dev);
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	bool limit_analog_gain = false;
	const struct of_device_id *match;
	int err;

	match = of_match_device(imx334_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return -EINVAL;
	}

	limit_analog_gain = of_property_read_bool(np, "limit_analog_gain");

	err = imx334_write_table(priv, mode_table[s_data->mode_prop_idx]);
	if (err)
		return err;
	priv->frame_length = imx334_frame_length[s_data->mode_prop_idx];
	msleep(20);
	return 0;
}

static int imx334_start_streaming(struct tegracam_device *tc_dev)
{
	struct imx334 *priv = (struct imx334 *)tegracam_get_privdata(tc_dev);
	int err;

	if (priv->sensor_id == 0) {//master
		err = imx334_write_table(priv,
			mode_table[IMX334_MODE_START_STREAM]);
	} else {
		err = imx334_write_table(priv,
			mode_table[IMX334_MODE_START_STREAM_SLAVE]);
		// msleep(500); //skip bad frames
	}
	if (err)
		return err;

	msleep(500);
	return 0;
}

static int imx334_stop_streaming(struct tegracam_device *tc_dev)
{
	struct imx334 *priv = (struct imx334 *)tegracam_get_privdata(tc_dev);
	int err;

	err = imx334_write_table(priv,
			mode_table[IMX334_MODE_STOP_STREAM]);
	if (err)
		return err;

	return 0;
	if (priv->sensor_id == 0) {//master
		err = imx334_write_table(priv,
			mode_table[IMX334_MODE_STOP_STREAM]);
	} else {
		err = imx334_write_table(priv,
			mode_table[IMX334_MODE_STOP_STREAM_SLAVE]);
	}
	if (err)
		return err;
	msleep(100);

	return 0;
}

static struct camera_common_sensor_ops imx334_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx334_frmfmt),
	.frmfmt_table = imx334_frmfmt,
	.power_on = imx334_power_on,
	.power_off = imx334_power_off,
	.write_reg = imx334_write_reg,
	.read_reg = imx334_read_reg,
	.parse_dt = imx334_parse_dt,
	.power_get = imx334_power_get,
	.power_put = imx334_power_put,
	.set_mode = imx334_set_mode,
	.start_streaming = imx334_start_streaming,
	.stop_streaming = imx334_stop_streaming,
};

static int imx334_board_setup(struct imx334 *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	int err = 0;

	dev_dbg(dev, "%s++\n", __func__);

	err = camera_common_mclk_enable(s_data);
	if (err) {
		dev_err(dev,
			"Error %d turning on mclk\n", err);
		return err;
	}

	err = imx334_power_on(s_data);
	if (err) {
		dev_err(dev,
			"Error %d during power on sensor\n", err);
		return err;
	}


	imx334_power_off(s_data);
	camera_common_mclk_disable(s_data);
	return err;
}

static int imx334_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx334_subdev_internal_ops = {
	.open = imx334_open,
};

static int imx334_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = client->dev.of_node;
	struct tegracam_device *tc_dev;
	struct imx334 *priv;
	struct i2c_adapter *adap;
	struct i2c_board_info brd;
	char *dev_name = "imx334_temp";
	int err;

	dev_info(dev, "probing v4l2 sensor\n");

	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct imx334), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx334", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx334_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx334_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx334_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	imx334_device_id(dev, priv);
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = imx334_board_setup(priv);
	if (err) {
		tegracam_device_unregister(tc_dev);
		dev_err(dev, "board setup failed\n");
		return err;
	}
	adap = i2c_get_adapter(
				priv->i2c_client->adapter->nr);
	memset(&brd, 0, sizeof(brd));
	strncpy(brd.type, dev_name, sizeof(brd.type));
	brd.addr = IMX334_TEMP_ADDRESS;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
	priv->tem_i2c_client = i2c_new_device(adap, &brd);
#else
	priv->tem_i2c_client = i2c_new_client_device(adap, &brd);
#endif
	priv->temp_regmap = devm_regmap_init_i2c(
			priv->tem_i2c_client, &temp_regmap_config);
	if (IS_ERR(priv->temp_regmap)) {
		err = PTR_ERR(priv->temp_regmap);
		i2c_unregister_device(priv->tem_i2c_client);
		priv->tem_i2c_client = NULL;
		dev_err(dev, "failed to setup temp i2c client\n");
		return err;
	}

	err = imx334_debugfs_init(NULL, priv);
	if (err) {
		dev_err(dev, "debugfs setup failed\n");
		return err;
	}

	priv->heater_gpio = of_get_named_gpio(node, "heater-gpios", 0);
	if (priv->heater_gpio < 0) {
		dev_err(dev, "heater-gpios not found!\n");
		return -ENOMEM;
	}
	gpio_direction_output(priv->heater_gpio, 0);
	gpio_set_value(priv->heater_gpio, 0);
	priv->heater_enable = false;
	imx334_set_temp_high(priv, IMX334_TEMP_HIGH_DEFAULT);
	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_info(dev, "Detected IMX334 sensor\n");

	return 0;
}

static int
imx334_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx334 *priv = (struct imx334 *)s_data->priv;

	i2c_unregister_device(priv->tem_i2c_client);
	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id imx334_id[] = {
	{ "imx334", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, imx334_id);
static struct i2c_driver imx334_i2c_driver = {
	.driver = {
		.name = "imx334",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx334_of_match),
	},
	.probe = imx334_probe,
	.remove = imx334_remove,
	.id_table = imx334_id,
};

module_i2c_driver(imx334_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for Sony IMX334");
MODULE_AUTHOR("Meng Gao <mengg@leopardimaging.com>");
MODULE_LICENSE("GPL v2");
