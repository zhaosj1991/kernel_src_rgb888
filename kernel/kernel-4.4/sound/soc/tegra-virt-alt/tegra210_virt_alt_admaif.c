/*
 * tegra210_admaif_alt.c - Tegra ADMAIF component driver
 *
 * Copyright (c) 2014-2017 NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "tegra210_virt_alt_admaif.h"
#include "tegra_virt_alt_ivc.h"
#include "tegra_pcm_alt.h"
#include "tegra_asoc_xbar_virt_alt.h"
#include "tegra_asoc_util_virt_alt.h"

#define NUM_SAD_CONTROLS	2

static struct tegra210_admaif *admaif;
static int tegra210_admaif_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	struct tegra210_virt_audio_cif *cif_conf = &data->cif;
	struct nvaudio_ivc_msg	msg;
	unsigned int value;
	int err;

	data->admaif_id = dai->id;
	memset(cif_conf, 0, sizeof(struct tegra210_virt_audio_cif));
	cif_conf->audio_channels = params_channels(params);
	cif_conf->client_channels = params_channels(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		cif_conf->client_bits = TEGRA210_AUDIOCIF_BITS_8;
		cif_conf->audio_bits = TEGRA210_AUDIOCIF_BITS_8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		cif_conf->client_bits = TEGRA210_AUDIOCIF_BITS_16;
		cif_conf->audio_bits = TEGRA210_AUDIOCIF_BITS_16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		cif_conf->client_bits = TEGRA210_AUDIOCIF_BITS_24;
		cif_conf->audio_bits = TEGRA210_AUDIOCIF_BITS_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		cif_conf->client_bits = TEGRA210_AUDIOCIF_BITS_32;
		cif_conf->audio_bits = TEGRA210_AUDIOCIF_BITS_32;
		break;
	default:
		dev_err(dev, "Wrong format!\n");
		return -EINVAL;
	}
	cif_conf->direction = substream->stream;

	value = (cif_conf->threshold <<
			TEGRA210_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT) |
		((cif_conf->audio_channels - 1) <<
			TEGRA210_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT) |
		((cif_conf->client_channels - 1) <<
			TEGRA210_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT) |
		(cif_conf->audio_bits <<
			TEGRA210_AUDIOCIF_CTRL_AUDIO_BITS_SHIFT) |
		(cif_conf->client_bits <<
			TEGRA210_AUDIOCIF_CTRL_CLIENT_BITS_SHIFT) |
		(cif_conf->expand <<
			TEGRA210_AUDIOCIF_CTRL_EXPAND_SHIFT) |
		(cif_conf->stereo_conv <<
			TEGRA210_AUDIOCIF_CTRL_STEREO_CONV_SHIFT) |
		(cif_conf->replicate <<
			TEGRA210_AUDIOCIF_CTRL_REPLICATE_SHIFT) |
		(cif_conf->truncate <<
			TEGRA210_AUDIOCIF_CTRL_TRUNCATE_SHIFT) |
		(cif_conf->mono_conv <<
			TEGRA210_AUDIOCIF_CTRL_MONO_CONV_SHIFT);

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.params.dmaif_info.id        = data->admaif_id;
	msg.params.dmaif_info.value     = value;
	if (!cif_conf->direction)
		msg.cmd = NVAUDIO_DMAIF_SET_TXCIF;
	else
		msg.cmd = NVAUDIO_DMAIF_SET_RXCIF;

	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));

	if (err < 0)
		pr_err("%s: error on ivc_send\n", __func__);

	return 0;
}

static void tegra210_admaif_start_playback(struct snd_soc_dai *dai)
{
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	data->admaif_id = dai->id;
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_START_PLAYBACK;
	msg.params.dmaif_info.id = data->admaif_id;
	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_send\n", __func__);
}

static void tegra210_admaif_stop_playback(struct snd_soc_dai *dai)
{
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	data->admaif_id = dai->id;
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_STOP_PLAYBACK;
	msg.params.dmaif_info.id = data->admaif_id;
	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_send\n", __func__);
}

static void tegra210_admaif_start_capture(struct snd_soc_dai *dai)
{
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	data->admaif_id = dai->id;
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_START_CAPTURE;
	msg.params.dmaif_info.id = data->admaif_id;
	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_send\n", __func__);
}

static void tegra210_admaif_stop_capture(struct snd_soc_dai *dai)
{
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	data->admaif_id = dai->id;
	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_STOP_CAPTURE;
	msg.params.dmaif_info.id = data->admaif_id;
	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_send\n", __func__);
}

static int tegra210_admaif_trigger(struct snd_pcm_substream *substream, int cmd,
				 struct snd_soc_dai *dai)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			tegra210_admaif_start_playback(dai);
		else
			tegra210_admaif_start_capture(dai);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			tegra210_admaif_stop_playback(dai);
		else
			tegra210_admaif_stop_capture(dai);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct snd_soc_dai_ops tegra210_admaif_dai_ops = {
	.hw_params	= tegra210_admaif_hw_params,
	.trigger	= tegra210_admaif_trigger,
};

static int tegra210_admaif_dai_probe(struct snd_soc_dai *dai)
{

	dai->capture_dma_data = &admaif->capture_dma_data[dai->id];
	dai->playback_dma_data = &admaif->playback_dma_data[dai->id];

	return 0;
}

#define ADMAIF_DAI(id)							\
	{							\
		.name = "ADMAIF" #id,				\
		.probe = tegra210_admaif_dai_probe,		\
		.playback = {					\
			.stream_name = "Playback " #id,		\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_192000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.capture = {					\
			.stream_name = "Capture " #id,		\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_192000,		\
			.formats = SNDRV_PCM_FMTBIT_S8 |		\
				SNDRV_PCM_FMTBIT_S16_LE |		\
				SNDRV_PCM_FMTBIT_S24_LE |		\
				SNDRV_PCM_FMTBIT_S32_LE,		\
		},						\
		.ops = &tegra210_admaif_dai_ops,			\
	}

static struct snd_soc_dai_driver tegra210_admaif_dais[] = {
	ADMAIF_DAI(1),
	ADMAIF_DAI(2),
	ADMAIF_DAI(3),
	ADMAIF_DAI(4),
	ADMAIF_DAI(5),
	ADMAIF_DAI(6),
	ADMAIF_DAI(7),
	ADMAIF_DAI(8),
	ADMAIF_DAI(9),
	ADMAIF_DAI(10),
	ADMAIF_DAI(11),
	ADMAIF_DAI(12),
	ADMAIF_DAI(13),
	ADMAIF_DAI(14),
	ADMAIF_DAI(15),
	ADMAIF_DAI(16),
	ADMAIF_DAI(17),
	ADMAIF_DAI(18),
	ADMAIF_DAI(19),
	ADMAIF_DAI(20),
};

static const struct soc_enum tegra_virt_t210ref_source =
	SOC_VALUE_ENUM_SINGLE(0, 0, 0, TEGRA_T210_SRC_NUM_MUX,
					tegra_virt_t210ref_source_text,
					tegra_virt_t210ref_source_value);

static const struct soc_enum tegra_virt_t186ref_source =
	SOC_VALUE_ENUM_SINGLE(0, 0, 0, TEGRA_T186_SRC_NUM_MUX,
					tegra_virt_t186ref_source_text,
					tegra_virt_t186ref_source_value);

static const struct soc_enum tegra_virt_t186_asrc_source =
	SOC_ENUM_SINGLE_EXT(NUM_ASRC_MODE, tegra186_asrc_ratio_source_text);

static const struct soc_enum tegra_virt_t186_arad_source =
	SOC_VALUE_ENUM_SINGLE(0, 0, 0, NUM_ARAD_SOURCES,
					tegra186_arad_mux_text,
					tegra186_arad_mux_value);

static const struct snd_kcontrol_new tegra_virt_t210ref_controls[] = {
MUX_ENUM_CTRL_DECL("ADMAIF1 Mux", 0x00, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF2 Mux", 0x01, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF3 Mux", 0x02, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF4 Mux", 0x03, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF5 Mux", 0x04, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF6 Mux", 0x05, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF7 Mux", 0x06, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF8 Mux", 0x07, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF9 Mux", 0x08, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF10 Mux", 0x09, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("I2S1 Mux", 0x10, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("I2S2 Mux", 0x11, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("I2S3 Mux", 0x12, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("I2S4 Mux", 0x13, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("I2S5 Mux", 0x14, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("SFC1 Mux", 0x18, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("SFC2 Mux", 0x19, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("SFC3 Mux", 0x1a, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("SFC4 Mux", 0x1b, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-1 Mux", 0x20, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-2 Mux", 0x21, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-3 Mux", 0x22, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-4 Mux", 0x23, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-5 Mux", 0x24, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-6 Mux", 0x25, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-7 Mux", 0x26, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-8 Mux", 0x27, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-9 Mux", 0x28, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-10 Mux", 0x29, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("SPDIF1-1 Mux", 0x34, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("SPDIF1-2 Mux", 0x35, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AFC1 Mux", 0x38, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AFC2 Mux", 0x39, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AFC3 Mux", 0x3a, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AFC4 Mux", 0x3b, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AFC5 Mux", 0x3c, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AFC6 Mux", 0x3d, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("OPE1 Mux", 0x40, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("SPKPROT1 Mux", 0x44, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("MVC1 Mux", 0x48, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("MVC2 Mux", 0x49, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AMX1-1 Mux", 0x50, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AMX1-2 Mux", 0x51, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AMX1-3 Mux", 0x52, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AMX1-4 Mux", 0x53, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AMX2-1 Mux", 0x54, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AMX2-2 Mux", 0x55, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AMX2-3 Mux", 0x56, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("AMX2-4 Mux", 0x57, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("ADX1 Mux", 0x60, &tegra_virt_t210ref_source),
MUX_ENUM_CTRL_DECL("ADX2 Mux", 0x61, &tegra_virt_t210ref_source),
MIXER_GAIN_CTRL_DECL("RX1 Gain", 0x00),
MIXER_GAIN_CTRL_DECL("RX2 Gain", 0x01),
MIXER_GAIN_CTRL_DECL("RX3 Gain", 0x02),
MIXER_GAIN_CTRL_DECL("RX4 Gain", 0x03),
MIXER_GAIN_CTRL_DECL("RX5 Gain", 0x04),
MIXER_GAIN_CTRL_DECL("RX6 Gain", 0x05),
MIXER_GAIN_CTRL_DECL("RX7 Gain", 0x06),
MIXER_GAIN_CTRL_DECL("RX8 Gain", 0x07),
MIXER_GAIN_CTRL_DECL("RX9 Gain", 0x08),
MIXER_GAIN_CTRL_DECL("RX10 Gain", 0x09),

MIXER_ADDER_CTRL_DECL("Adder1 RX1", 0x00, 0x01),
MIXER_ADDER_CTRL_DECL("Adder1 RX2", 0x00, 0x02),
MIXER_ADDER_CTRL_DECL("Adder1 RX3", 0x00, 0x03),
MIXER_ADDER_CTRL_DECL("Adder1 RX4", 0x00, 0x04),
MIXER_ADDER_CTRL_DECL("Adder1 RX5", 0x00, 0x05),
MIXER_ADDER_CTRL_DECL("Adder1 RX6", 0x00, 0x06),
MIXER_ADDER_CTRL_DECL("Adder1 RX7", 0x00, 0x07),
MIXER_ADDER_CTRL_DECL("Adder1 RX8", 0x00, 0x08),
MIXER_ADDER_CTRL_DECL("Adder1 RX9", 0x00, 0x09),
MIXER_ADDER_CTRL_DECL("Adder1 RX10", 0x00, 0x0a),

MIXER_ADDER_CTRL_DECL("Adder2 RX1", 0x01, 0x01),
MIXER_ADDER_CTRL_DECL("Adder2 RX2", 0x01, 0x02),
MIXER_ADDER_CTRL_DECL("Adder2 RX3", 0x01, 0x03),
MIXER_ADDER_CTRL_DECL("Adder2 RX4", 0x01, 0x04),
MIXER_ADDER_CTRL_DECL("Adder2 RX5", 0x01, 0x05),
MIXER_ADDER_CTRL_DECL("Adder2 RX6", 0x01, 0x06),
MIXER_ADDER_CTRL_DECL("Adder2 RX7", 0x01, 0x07),
MIXER_ADDER_CTRL_DECL("Adder2 RX8", 0x01, 0x08),

MIXER_ADDER_CTRL_DECL("Adder2 RX9", 0x01, 0x09),
MIXER_ADDER_CTRL_DECL("Adder2 RX10", 0x01, 0x0a),

MIXER_ADDER_CTRL_DECL("Adder3 RX1", 0x02, 0x01),
MIXER_ADDER_CTRL_DECL("Adder3 RX2", 0x02, 0x02),
MIXER_ADDER_CTRL_DECL("Adder3 RX3", 0x02, 0x03),
MIXER_ADDER_CTRL_DECL("Adder3 RX4", 0x02, 0x04),
MIXER_ADDER_CTRL_DECL("Adder3 RX5", 0x02, 0x05),
MIXER_ADDER_CTRL_DECL("Adder3 RX6", 0x02, 0x06),
MIXER_ADDER_CTRL_DECL("Adder3 RX7", 0x02, 0x07),
MIXER_ADDER_CTRL_DECL("Adder3 RX8", 0x02, 0x08),
MIXER_ADDER_CTRL_DECL("Adder3 RX9", 0x02, 0x09),
MIXER_ADDER_CTRL_DECL("Adder3 RX10", 0x02, 0x0a),

MIXER_ADDER_CTRL_DECL("Adder4 RX1", 0x03, 0x01),
MIXER_ADDER_CTRL_DECL("Adder4 RX2", 0x03, 0x02),
MIXER_ADDER_CTRL_DECL("Adder4 RX3", 0x03, 0x03),
MIXER_ADDER_CTRL_DECL("Adder4 RX4", 0x03, 0x04),
MIXER_ADDER_CTRL_DECL("Adder4 RX5", 0x03, 0x05),
MIXER_ADDER_CTRL_DECL("Adder4 RX6", 0x03, 0x06),
MIXER_ADDER_CTRL_DECL("Adder4 RX7", 0x03, 0x07),
MIXER_ADDER_CTRL_DECL("Adder4 RX8", 0x03, 0x08),
MIXER_ADDER_CTRL_DECL("Adder4 RX9", 0x03, 0x09),
MIXER_ADDER_CTRL_DECL("Adder4 RX10", 0x03, 0x0a),

MIXER_ADDER_CTRL_DECL("Adder5 RX1", 0x04, 0x01),
MIXER_ADDER_CTRL_DECL("Adder5 RX2", 0x04, 0x02),
MIXER_ADDER_CTRL_DECL("Adder5 RX3", 0x04, 0x03),
MIXER_ADDER_CTRL_DECL("Adder5 RX4", 0x04, 0x04),
MIXER_ADDER_CTRL_DECL("Adder5 RX5", 0x04, 0x05),
MIXER_ADDER_CTRL_DECL("Adder5 RX6", 0x04, 0x06),
MIXER_ADDER_CTRL_DECL("Adder5 RX7", 0x04, 0x07),
MIXER_ADDER_CTRL_DECL("Adder5 RX8", 0x04, 0x08),
MIXER_ADDER_CTRL_DECL("Adder5 RX9", 0x04, 0x09),
MIXER_ADDER_CTRL_DECL("Adder5 RX10", 0x04, 0x0a),

MIXER_ENABLE_CTRL_DECL("Mixer Enable", 0x00),

SFC_IN_FREQ_CTRL_DECL("SFC1 in freq", 0x00),
SFC_IN_FREQ_CTRL_DECL("SFC2 in freq", 0x01),
SFC_IN_FREQ_CTRL_DECL("SFC3 in freq", 0x02),
SFC_IN_FREQ_CTRL_DECL("SFC4 in freq", 0x03),

SFC_OUT_FREQ_CTRL_DECL("SFC1 out freq", 0x00),
SFC_OUT_FREQ_CTRL_DECL("SFC2 out freq", 0x01),
SFC_OUT_FREQ_CTRL_DECL("SFC3 out freq", 0x02),
SFC_OUT_FREQ_CTRL_DECL("SFC4 out freq", 0x03),

AMX_ENABLE_CTRL_DECL("AMX1-1 Enable", 0x01, 0x01),
AMX_ENABLE_CTRL_DECL("AMX1-2 Enable", 0x01, 0x02),

AMX_ENABLE_CTRL_DECL("AMX1-3 Enable", 0x01, 0x03),
AMX_ENABLE_CTRL_DECL("AMX1-4 Enable", 0x01, 0x04),
AMX_ENABLE_CTRL_DECL("AMX2-1 Enable", 0x02, 0x01),
AMX_ENABLE_CTRL_DECL("AMX2-2 Enable", 0x02, 0x02),
AMX_ENABLE_CTRL_DECL("AMX2-3 Enable", 0x02, 0x03),
AMX_ENABLE_CTRL_DECL("AMX2-4 Enable", 0x02, 0x04),

I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S1 Loopback", 0x01),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S2 Loopback", 0x02),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S3 Loopback", 0x03),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S4 Loopback", 0x04),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S5 Loopback", 0x05),

};

static const struct snd_kcontrol_new tegra_virt_t186ref_controls[] = {
MUX_ENUM_CTRL_DECL("ADMAIF1 Mux", 0x00, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF2 Mux", 0x01, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF3 Mux", 0x02, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF4 Mux", 0x03, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF5 Mux", 0x04, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF6 Mux", 0x05, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF7 Mux", 0x06, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF8 Mux", 0x07, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF9 Mux", 0x08, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF10 Mux", 0x09, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("I2S1 Mux", 0x10, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("I2S2 Mux", 0x11, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("I2S3 Mux", 0x12, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("I2S4 Mux", 0x13, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("I2S5 Mux", 0x14, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("SFC1 Mux", 0x18, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("SFC2 Mux", 0x19, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("SFC3 Mux", 0x1a, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("SFC4 Mux", 0x1b, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-1 Mux", 0x20, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-2 Mux", 0x21, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-3 Mux", 0x22, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-4 Mux", 0x23, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-5 Mux", 0x24, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-6 Mux", 0x25, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-7 Mux", 0x26, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-8 Mux", 0x27, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-9 Mux", 0x28, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("MIXER1-10 Mux", 0x29, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("SPDIF1-1 Mux", 0x34, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("SPDIF1-2 Mux", 0x35, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AFC1 Mux", 0x38, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AFC2 Mux", 0x39, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AFC3 Mux", 0x3a, &tegra_virt_t186ref_source),

MUX_ENUM_CTRL_DECL("AFC4 Mux", 0x3b, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AFC5 Mux", 0x3c, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AFC6 Mux", 0x3d, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("OPE1 Mux", 0x40, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("SPKPROT1 Mux", 0x44, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("MVC1 Mux", 0x48, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("MVC2 Mux", 0x49, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX1-1 Mux", 0x50, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX1-2 Mux", 0x51, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX1-3 Mux", 0x52, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX1-4 Mux", 0x53, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX2-1 Mux", 0x54, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX2-2 Mux", 0x55, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX2-3 Mux", 0x56, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX2-4 Mux", 0x57, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADX1 Mux", 0x60, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADX2 Mux", 0x61, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF11 Mux", 0x0a, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF12 Mux", 0x0b, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF13 Mux", 0x0c, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF14 Mux", 0x0d, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF15 Mux", 0x0e, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF16 Mux", 0x0f, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF17 Mux", 0x68, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF18 Mux", 0x69, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF19 Mux", 0x6a, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADMAIF20 Mux", 0x6b, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("I2S6 Mux", 0x15, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX3-1 Mux", 0x58, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX3-2 Mux", 0x59, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX3-3 Mux", 0x5a, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX3-4 Mux", 0x5b, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX4-1 Mux", 0x64, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX4-2 Mux", 0x65, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX4-3 Mux", 0x66, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("AMX4-4 Mux", 0x67, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADX3 Mux", 0x62, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ADX4 Mux", 0x63, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ASRC1-1 Mux", 0x6c, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ASRC1-2 Mux", 0x6d, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ASRC1-3 Mux", 0x6e, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ASRC1-4 Mux", 0x6f, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ASRC1-5 Mux", 0x70, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ASRC1-6 Mux", 0x71, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("ASRC1-7 Mux", 0x72, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("DSPK1 Mux", 0x30, &tegra_virt_t186ref_source),
MUX_ENUM_CTRL_DECL("DSPK2 Mux", 0x31, &tegra_virt_t186ref_source),
MIXER_GAIN_CTRL_DECL("RX1 Gain", 0x00),
MIXER_GAIN_CTRL_DECL("RX2 Gain", 0x01),
MIXER_GAIN_CTRL_DECL("RX3 Gain", 0x02),

MIXER_GAIN_CTRL_DECL("RX4 Gain", 0x03),
MIXER_GAIN_CTRL_DECL("RX5 Gain", 0x04),
MIXER_GAIN_CTRL_DECL("RX6 Gain", 0x05),
MIXER_GAIN_CTRL_DECL("RX7 Gain", 0x06),
MIXER_GAIN_CTRL_DECL("RX8 Gain", 0x07),
MIXER_GAIN_CTRL_DECL("RX9 Gain", 0x08),
MIXER_GAIN_CTRL_DECL("RX10 Gain", 0x09),

MIXER_ADDER_CTRL_DECL("Adder1 RX1", 0x00, 0x01),
MIXER_ADDER_CTRL_DECL("Adder1 RX2", 0x00, 0x02),
MIXER_ADDER_CTRL_DECL("Adder1 RX3", 0x00, 0x03),
MIXER_ADDER_CTRL_DECL("Adder1 RX4", 0x00, 0x04),
MIXER_ADDER_CTRL_DECL("Adder1 RX5", 0x00, 0x05),
MIXER_ADDER_CTRL_DECL("Adder1 RX6", 0x00, 0x06),
MIXER_ADDER_CTRL_DECL("Adder1 RX7", 0x00, 0x07),
MIXER_ADDER_CTRL_DECL("Adder1 RX8", 0x00, 0x08),
MIXER_ADDER_CTRL_DECL("Adder1 RX9", 0x00, 0x09),
MIXER_ADDER_CTRL_DECL("Adder1 RX10", 0x00, 0x0a),

MIXER_ADDER_CTRL_DECL("Adder2 RX1", 0x01, 0x01),
MIXER_ADDER_CTRL_DECL("Adder2 RX2", 0x01, 0x02),
MIXER_ADDER_CTRL_DECL("Adder2 RX3", 0x01, 0x03),
MIXER_ADDER_CTRL_DECL("Adder2 RX4", 0x01, 0x04),
MIXER_ADDER_CTRL_DECL("Adder2 RX5", 0x01, 0x05),
MIXER_ADDER_CTRL_DECL("Adder2 RX6", 0x01, 0x06),
MIXER_ADDER_CTRL_DECL("Adder2 RX7", 0x01, 0x07),
MIXER_ADDER_CTRL_DECL("Adder2 RX8", 0x01, 0x08),
MIXER_ADDER_CTRL_DECL("Adder2 RX9", 0x01, 0x09),
MIXER_ADDER_CTRL_DECL("Adder2 RX10", 0x01, 0x0a),

MIXER_ADDER_CTRL_DECL("Adder3 RX1", 0x02, 0x01),
MIXER_ADDER_CTRL_DECL("Adder3 RX2", 0x02, 0x02),
MIXER_ADDER_CTRL_DECL("Adder3 RX3", 0x02, 0x03),
MIXER_ADDER_CTRL_DECL("Adder3 RX4", 0x02, 0x04),
MIXER_ADDER_CTRL_DECL("Adder3 RX5", 0x02, 0x05),
MIXER_ADDER_CTRL_DECL("Adder3 RX6", 0x02, 0x06),
MIXER_ADDER_CTRL_DECL("Adder3 RX7", 0x02, 0x07),
MIXER_ADDER_CTRL_DECL("Adder3 RX8", 0x02, 0x08),
MIXER_ADDER_CTRL_DECL("Adder3 RX9", 0x02, 0x09),
MIXER_ADDER_CTRL_DECL("Adder3 RX10", 0x02, 0x0a),

MIXER_ADDER_CTRL_DECL("Adder4 RX1", 0x03, 0x01),
MIXER_ADDER_CTRL_DECL("Adder4 RX2", 0x03, 0x02),
MIXER_ADDER_CTRL_DECL("Adder4 RX3", 0x03, 0x03),
MIXER_ADDER_CTRL_DECL("Adder4 RX4", 0x03, 0x04),
MIXER_ADDER_CTRL_DECL("Adder4 RX5", 0x03, 0x05),
MIXER_ADDER_CTRL_DECL("Adder4 RX6", 0x03, 0x06),
MIXER_ADDER_CTRL_DECL("Adder4 RX7", 0x03, 0x07),
MIXER_ADDER_CTRL_DECL("Adder4 RX8", 0x03, 0x08),
MIXER_ADDER_CTRL_DECL("Adder4 RX9", 0x03, 0x09),


MIXER_ADDER_CTRL_DECL("Adder4 RX10", 0x03, 0x0a),

MIXER_ADDER_CTRL_DECL("Adder5 RX1", 0x04, 0x01),
MIXER_ADDER_CTRL_DECL("Adder5 RX2", 0x04, 0x02),
MIXER_ADDER_CTRL_DECL("Adder5 RX3", 0x04, 0x03),
MIXER_ADDER_CTRL_DECL("Adder5 RX4", 0x04, 0x04),
MIXER_ADDER_CTRL_DECL("Adder5 RX5", 0x04, 0x05),
MIXER_ADDER_CTRL_DECL("Adder5 RX6", 0x04, 0x06),
MIXER_ADDER_CTRL_DECL("Adder5 RX7", 0x04, 0x07),
MIXER_ADDER_CTRL_DECL("Adder5 RX8", 0x04, 0x08),
MIXER_ADDER_CTRL_DECL("Adder5 RX9", 0x04, 0x09),
MIXER_ADDER_CTRL_DECL("Adder5 RX10", 0x04, 0x0a),

MIXER_ENABLE_CTRL_DECL("Mixer Enable", 0x00),

SFC_IN_FREQ_CTRL_DECL("SFC1 in freq", 0x00),
SFC_IN_FREQ_CTRL_DECL("SFC2 in freq", 0x01),
SFC_IN_FREQ_CTRL_DECL("SFC3 in freq", 0x02),
SFC_IN_FREQ_CTRL_DECL("SFC4 in freq", 0x03),

SFC_OUT_FREQ_CTRL_DECL("SFC1 out freq", 0x00),
SFC_OUT_FREQ_CTRL_DECL("SFC2 out freq", 0x01),
SFC_OUT_FREQ_CTRL_DECL("SFC3 out freq", 0x02),
SFC_OUT_FREQ_CTRL_DECL("SFC4 out freq", 0x03),

ASRC_RATIO_INT_CTRL_DECL("ASRC1 Ratio1 Int", 0x01),
ASRC_RATIO_INT_CTRL_DECL("ASRC1 Ratio2 Int", 0x02),
ASRC_RATIO_INT_CTRL_DECL("ASRC1 Ratio3 Int", 0x03),
ASRC_RATIO_INT_CTRL_DECL("ASRC1 Ratio4 Int", 0x04),
ASRC_RATIO_INT_CTRL_DECL("ASRC1 Ratio5 Int", 0x05),
ASRC_RATIO_INT_CTRL_DECL("ASRC1 Ratio6 Int", 0x06),

ASRC_RATIO_FRAC_CTRL_DECL("ASRC1 Ratio1 Frac", 0x01),
ASRC_RATIO_FRAC_CTRL_DECL("ASRC1 Ratio2 Frac", 0x02),
ASRC_RATIO_FRAC_CTRL_DECL("ASRC1 Ratio3 Frac", 0x03),
ASRC_RATIO_FRAC_CTRL_DECL("ASRC1 Ratio4 Frac", 0x04),
ASRC_RATIO_FRAC_CTRL_DECL("ASRC1 Ratio5 Frac", 0x05),
ASRC_RATIO_FRAC_CTRL_DECL("ASRC1 Ratio6 Frac", 0x06),

ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio1 SRC", 0x01,
			&tegra_virt_t186_asrc_source),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio2 SRC", 0x02,
			&tegra_virt_t186_asrc_source),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio3 SRC", 0x03,
			&tegra_virt_t186_asrc_source),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio4 SRC", 0x04,
			&tegra_virt_t186_asrc_source),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio5 SRC", 0x05,
			&tegra_virt_t186_asrc_source),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio6 SRC", 0x06,
			&tegra_virt_t186_asrc_source),

ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream1 Enable", 0x01),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream2 Enable", 0x02),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream3 Enable", 0x03),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream4 Enable", 0x04),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream5 Enable", 0x05),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream6 Enable", 0x06),

ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Ratio1 Hwcomp Disable", 0x01),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Ratio2 Hwcomp Disable", 0x02),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Ratio3 Hwcomp Disable", 0x03),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Ratio4 Hwcomp Disable", 0x04),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Ratio5 Hwcomp Disable", 0x05),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Ratio6 Hwcomp Disable", 0x06),

ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream1 Input Thresh", 0x01),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream2 Input Thresh", 0x02),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream3 Input Thresh", 0x03),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream4 Input Thresh", 0x04),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream5 Input Thresh", 0x05),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream6 Input Thresh", 0x06),

ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream1 Output Thresh", 0x01),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream2 Output Thresh", 0x02),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream3 Output Thresh", 0x03),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream4 Output Thresh", 0x04),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream5 Output Thresh", 0x05),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream6 Output Thresh", 0x06),

ARAD_LANE_SOURCE_CTRL_DECL("Numerator1 Mux", numerator1_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Numerator2 Mux", numerator2_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Numerator3 Mux", numerator3_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Numerator4 Mux", numerator4_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Numerator5 Mux", numerator5_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Numerator6 Mux", numerator6_enum,
				&tegra_virt_t186_arad_source),

ARAD_LANE_SOURCE_CTRL_DECL("Denominator1 Mux", denominator1_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Denominator2 Mux", denominator2_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Denominator3 Mux", denominator3_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Denominator4 Mux", denominator4_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Denominator5 Mux", denominator5_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Denominator6 Mux", denominator6_enum,
				&tegra_virt_t186_arad_source),

ARAD_LANE_PRESCALAR_CTRL_DECL("Numerator1 Prescalar", numerator1_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Numerator2 Prescalar", numerator2_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Numerator3 Prescalar", numerator3_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Numerator4 Prescalar", numerator4_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Numerator5 Prescalar", numerator5_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Numerator6 Prescalar", numerator6_enum),

ARAD_LANE_PRESCALAR_CTRL_DECL("Denominator1 Prescalar", denominator1_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Denominator2 Prescalar", denominator2_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Denominator3 Prescalar", denominator3_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Denominator4 Prescalar", denominator4_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Denominator5 Prescalar", denominator5_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Denominator6 Prescalar", denominator6_enum),

ARAD_LANE_ENABLE_CTRL_DECL("Lane1 enable", 0x00),
ARAD_LANE_ENABLE_CTRL_DECL("Lane2 enable", 0x01),
ARAD_LANE_ENABLE_CTRL_DECL("Lane3 enable", 0x02),
ARAD_LANE_ENABLE_CTRL_DECL("Lane4 enable", 0x03),
ARAD_LANE_ENABLE_CTRL_DECL("Lane5 enable", 0x04),
ARAD_LANE_ENABLE_CTRL_DECL("Lane6 enable", 0x05),

ARAD_LANE_RATIO_CTRL_DECL("Lane1 Ratio", 0x00),
ARAD_LANE_RATIO_CTRL_DECL("Lane2 Ratio", 0x01),
ARAD_LANE_RATIO_CTRL_DECL("Lane3 Ratio", 0x02),
ARAD_LANE_RATIO_CTRL_DECL("Lane4 Ratio", 0x03),
ARAD_LANE_RATIO_CTRL_DECL("Lane5 Ratio", 0x04),
ARAD_LANE_RATIO_CTRL_DECL("Lane6 Ratio", 0x05),

AMX_ENABLE_CTRL_DECL("AMX1-1 Enable", 0x01, 0x01),
AMX_ENABLE_CTRL_DECL("AMX1-2 Enable", 0x01, 0x02),
AMX_ENABLE_CTRL_DECL("AMX1-3 Enable", 0x01, 0x03),
AMX_ENABLE_CTRL_DECL("AMX1-4 Enable", 0x01, 0x04),
AMX_ENABLE_CTRL_DECL("AMX2-1 Enable", 0x02, 0x01),
AMX_ENABLE_CTRL_DECL("AMX2-2 Enable", 0x02, 0x02),
AMX_ENABLE_CTRL_DECL("AMX2-3 Enable", 0x02, 0x03),
AMX_ENABLE_CTRL_DECL("AMX2-4 Enable", 0x02, 0x04),

AMX_ENABLE_CTRL_DECL("AMX3-1 Enable", 0x03, 0x01),
AMX_ENABLE_CTRL_DECL("AMX3-2 Enable", 0x03, 0x02),
AMX_ENABLE_CTRL_DECL("AMX3-3 Enable", 0x03, 0x03),
AMX_ENABLE_CTRL_DECL("AMX3-4 Enable", 0x03, 0x04),
AMX_ENABLE_CTRL_DECL("AMX4-1 Enable", 0x04, 0x01),
AMX_ENABLE_CTRL_DECL("AMX4-2 Enable", 0x04, 0x02),
AMX_ENABLE_CTRL_DECL("AMX4-3 Enable", 0x04, 0x03),

AMX_ENABLE_CTRL_DECL("AMX4-4 Enable", 0x04, 0x04),

I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S1 Loopback", 0x01),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S2 Loopback", 0x02),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S3 Loopback", 0x03),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S4 Loopback", 0x04),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S5 Loopback", 0x05),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S6 Loopback", 0x06),

/* SAD controls should be always the last ones */
SOC_SINGLE_BOOL_EXT("SAD Init", 0,
		tegra_sad_get_init, tegra_sad_set_init),
SOC_SINGLE_BOOL_EXT("SAD Enable", 0,
		tegra_sad_get_enable, tegra_sad_set_enable),
};

static struct snd_soc_component_driver tegra210_admaif_dai_driver = {
	.name		= DRV_NAME,
	.controls = tegra_virt_t186ref_controls,
	.num_controls = ARRAY_SIZE(tegra_virt_t186ref_controls),
};

int tegra210_virt_admaif_register_component(struct platform_device *pdev,
				struct tegra_virt_admaif_soc_data *data)
{
	int i = 0;
	int ret;
	int admaif_ch_num = 0;
	unsigned int admaif_ch_list[MAX_ADMAIF_IDS] = {0};
	struct tegra_virt_admaif_soc_data *soc_data = data;
	int adma_count = 0;
	bool sad_enabled = false;

	admaif = devm_kzalloc(&pdev->dev, sizeof(*admaif), GFP_KERNEL);
	if (admaif == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	admaif->client_data.hivc_client =
			nvaudio_ivc_alloc_ctxt(&pdev->dev);

	if (!admaif->client_data.hivc_client) {
		dev_err(&pdev->dev, "Failed to allocate IVC context\n");
		ret = -ENODEV;
		goto err;
	}

	admaif->capture_dma_data = devm_kzalloc(&pdev->dev,
			sizeof(struct tegra_alt_pcm_dma_params) *
				soc_data->num_ch,
			GFP_KERNEL);
	if (admaif->capture_dma_data == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	admaif->playback_dma_data = devm_kzalloc(&pdev->dev,
			sizeof(struct tegra_alt_pcm_dma_params) *
				soc_data->num_ch,
			GFP_KERNEL);
	if (admaif->playback_dma_data == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	if (of_property_read_u32(pdev->dev.of_node,
				"admaif_ch_num", &admaif_ch_num)) {
		dev_err(&pdev->dev, "number of admaif channels is not set\n");
		return -EINVAL;
	}

	if (of_property_read_u32_array(pdev->dev.of_node,
						"admaif_ch_list",
						admaif_ch_list,
						admaif_ch_num)) {
		dev_err(&pdev->dev, "admaif_ch_list is not populated\n");
		return -EINVAL;
	}


	for (i = 0; i < soc_data->num_ch; i++) {
		if ((i + 1) != admaif_ch_list[adma_count])
			continue;
	if (of_device_is_compatible(pdev->dev.of_node,
		"nvidia,tegra186-virt-pcm")) {
		admaif->playback_dma_data[i].addr = TEGRA186_ADMAIF_BASE +
				TEGRA186_ADMAIF_XBAR_TX_FIFO_WRITE +
				(i * TEGRA186_ADMAIF_CHANNEL_REG_STRIDE);
		admaif->capture_dma_data[i].addr = TEGRA186_ADMAIF_BASE +
				TEGRA186_ADMAIF_XBAR_RX_FIFO_READ +
				(i * TEGRA186_ADMAIF_CHANNEL_REG_STRIDE);
		tegra_virt_set_enum_source(&tegra_virt_t186ref_source);
		} else if (of_device_is_compatible(pdev->dev.of_node,
		"nvidia,tegra210-virt-pcm")) {
		admaif->playback_dma_data[i].addr = TEGRA210_ADMAIF_BASE +
				TEGRA210_ADMAIF_XBAR_TX_FIFO_WRITE +
				(i * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
		admaif->capture_dma_data[i].addr = TEGRA210_ADMAIF_BASE +
				TEGRA210_ADMAIF_XBAR_RX_FIFO_READ +
				(i * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
		tegra_virt_set_enum_source(&tegra_virt_t210ref_source);
		} else {
			dev_err(&pdev->dev,
				"Uncompatible device driver\n");
			ret = -ENODEV;
			goto err;
		}

		admaif->playback_dma_data[i].wrap = 4;
		admaif->playback_dma_data[i].width = 32;
		admaif->playback_dma_data[i].req_sel = i + 1;

		if (of_property_read_string_index(pdev->dev.of_node,
				"dma-names",
				(adma_count * 2) + 1,
				&admaif->playback_dma_data[i].chan_name) < 0) {
			dev_err(&pdev->dev,
				"Missing property nvidia,dma-names\n");
			ret = -ENODEV;
			goto err;
		}

		admaif->capture_dma_data[i].wrap = 4;
		admaif->capture_dma_data[i].width = 32;
		admaif->capture_dma_data[i].req_sel = i + 1;
		if (of_property_read_string_index(pdev->dev.of_node,
				"dma-names",
				(adma_count * 2),
				&admaif->capture_dma_data[i].chan_name) < 0) {
			dev_err(&pdev->dev,
				"Missing property nvidia,dma-names\n");
			ret = -ENODEV;
			goto err;
		}
		adma_count++;
	}

	/* Remove exposing sad controls if not enabled in device node */
	sad_enabled = of_property_read_bool(pdev->dev.of_node,
		"sad_enabled");
	if (!sad_enabled) {
		tegra210_admaif_dai_driver.num_controls =
		ARRAY_SIZE(tegra_virt_t186ref_controls) - NUM_SAD_CONTROLS;
	}

	ret = snd_soc_register_component(&pdev->dev,
					&tegra210_admaif_dai_driver,
					tegra210_admaif_dais,
					soc_data->num_ch);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAIs %d: %d\n",
			i, ret);
		goto err;
	}

	ret = tegra_alt_pcm_platform_register(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM: %d\n", ret);
		goto err_unregister_dais;
	}

	return 0;
err_unregister_dais:
	snd_soc_unregister_component(&pdev->dev);
err:
	return ret;
}
EXPORT_SYMBOL_GPL(tegra210_virt_admaif_register_component);

MODULE_LICENSE("GPL");
