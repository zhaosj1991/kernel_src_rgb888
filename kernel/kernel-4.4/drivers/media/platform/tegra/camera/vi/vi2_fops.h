/*
 * Tegra Video Input 2 device common APIs
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __T210_VI_H__
#define __T210_VI_H__

void vi2_syncpt_init(struct tegra_channel *chan);
void vi2_syncpt_free(struct tegra_channel *chan);
int vi2_power_on(struct tegra_channel *chan);
void vi2_power_off(struct tegra_channel *chan);
int vi2_channel_start_streaming(struct vb2_queue *vq, u32 count);
int vi2_channel_stop_streaming(struct vb2_queue *vq);
int vi2_add_ctrls(struct tegra_channel *chan);
void vi2_init_video_formats(struct tegra_channel *chan);
int vi2_mfi_work(struct tegra_mc_vi *vi, int csiport);

struct tegra_vi_fops vi2_fops = {
	.vi_syncpt_init = vi2_syncpt_init,
	.vi_syncpt_free = vi2_syncpt_free,
	.vi_power_on = vi2_power_on,
	.vi_power_off = vi2_power_off,
	.vi_start_streaming = vi2_channel_start_streaming,
	.vi_stop_streaming = vi2_channel_stop_streaming,
	.vi_add_ctrls = vi2_add_ctrls,
	.vi_init_video_formats = vi2_init_video_formats,
	.vi_mfi_work = vi2_mfi_work,
};

#endif
