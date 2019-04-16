/*
 * Tegra Video Input 4 device common APIs
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Frank Chen <frankc@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __T186_VI_H__
#define __T186_VI_H__

int vi4_power_on(struct tegra_channel *chan);
void vi4_power_off(struct tegra_channel *chan);
int vi4_channel_start_streaming(struct vb2_queue *vq, u32 count);
int vi4_channel_stop_streaming(struct vb2_queue *vq);
int vi4_add_ctrls(struct tegra_channel *chan);
void vi4_init_video_formats(struct tegra_channel *chan);
int vi4_mfi_work(struct tegra_mc_vi *vi, int channel);

struct tegra_vi_fops vi4_fops = {
	.vi_power_on = vi4_power_on,
	.vi_power_off = vi4_power_off,
	.vi_start_streaming = vi4_channel_start_streaming,
	.vi_stop_streaming = vi4_channel_stop_streaming,
	.vi_add_ctrls = vi4_add_ctrls,
	.vi_init_video_formats = vi4_init_video_formats,
	.vi_mfi_work = vi4_mfi_work,
};

#endif
