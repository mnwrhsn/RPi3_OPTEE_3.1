/*
 * ASoC Driver for IQaudIO DAC
 *
 * Author:	Florian Meier <florian.meier@koalo.de>
 *		Copyright 2013
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>

static bool digital_gain_0db_limit = true;

static int snd_rpi_iqaudio_dac_init(struct snd_soc_pcm_runtime *rtd)
{
	if (digital_gain_0db_limit)
	{
		int ret;
		struct snd_soc_card *card = rtd->card;

		ret = snd_soc_limit_volume(card, "Digital Playback Volume", 207);
		if (ret < 0)
			dev_warn(card->dev, "Failed to set volume limit: %d\n", ret);
	}

	return 0;
}

static int snd_rpi_iqaudio_dac_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
// NOT USED	struct snd_soc_dai *codec_dai = rtd->codec_dai;
// NOT USED	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	unsigned int sample_bits =
		snd_pcm_format_physical_width(params_format(params));

	return snd_soc_dai_set_bclk_ratio(cpu_dai, sample_bits * 2);
}

/* machine stream operations */
static struct snd_soc_ops snd_rpi_iqaudio_dac_ops = {
	.hw_params = snd_rpi_iqaudio_dac_hw_params,
};

static struct snd_soc_dai_link snd_rpi_iqaudio_dac_dai[] = {
{
	.cpu_dai_name	= "bcm2708-i2s.0",
	.codec_dai_name	= "pcm512x-hifi",
	.platform_name	= "bcm2708-i2s.0",
	.codec_name	= "pcm512x.1-004c",
	.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
	.ops		= &snd_rpi_iqaudio_dac_ops,
	.init		= snd_rpi_iqaudio_dac_init,
},
};

/* audio machine driver */
static struct snd_soc_card snd_rpi_iqaudio_dac = {
	.owner        = THIS_MODULE,
	.dai_link     = snd_rpi_iqaudio_dac_dai,
	.num_links    = ARRAY_SIZE(snd_rpi_iqaudio_dac_dai),
};

static int snd_rpi_iqaudio_dac_probe(struct platform_device *pdev)
{
	int ret = 0;

	snd_rpi_iqaudio_dac.dev = &pdev->dev;

	if (pdev->dev.of_node) {
	    struct device_node *i2s_node;
	    struct snd_soc_card *card = &snd_rpi_iqaudio_dac;
	    struct snd_soc_dai_link *dai = &snd_rpi_iqaudio_dac_dai[0];
	    i2s_node = of_parse_phandle(pdev->dev.of_node,
					"i2s-controller", 0);

	    if (i2s_node) {
		dai->cpu_dai_name = NULL;
		dai->cpu_of_node = i2s_node;
		dai->platform_name = NULL;
		dai->platform_of_node = i2s_node;
	    }

	    digital_gain_0db_limit = !of_property_read_bool(pdev->dev.of_node,
					"iqaudio,24db_digital_gain");
	    if (of_property_read_string(pdev->dev.of_node, "card_name",
					&card->name))
		card->name = "IQaudIODAC";
	    if (of_property_read_string(pdev->dev.of_node, "dai_name",
					&dai->name))
		dai->name = "IQaudIO DAC";
	    if (of_property_read_string(pdev->dev.of_node, "dai_stream_name",
					&dai->stream_name))
		dai->stream_name = "IQaudIO DAC HiFi";
	}

	ret = snd_soc_register_card(&snd_rpi_iqaudio_dac);
	if (ret)
		dev_err(&pdev->dev,
			"snd_soc_register_card() failed: %d\n", ret);

	return ret;
}

static int snd_rpi_iqaudio_dac_remove(struct platform_device *pdev)
{
	return snd_soc_unregister_card(&snd_rpi_iqaudio_dac);
}

static const struct of_device_id iqaudio_of_match[] = {
	{ .compatible = "iqaudio,iqaudio-dac", },
	{},
};
MODULE_DEVICE_TABLE(of, iqaudio_of_match);

static struct platform_driver snd_rpi_iqaudio_dac_driver = {
	.driver = {
		.name   = "snd-rpi-iqaudio-dac",
		.owner  = THIS_MODULE,
		.of_match_table = iqaudio_of_match,
	},
	.probe          = snd_rpi_iqaudio_dac_probe,
	.remove         = snd_rpi_iqaudio_dac_remove,
};

module_platform_driver(snd_rpi_iqaudio_dac_driver);

MODULE_AUTHOR("Florian Meier <florian.meier@koalo.de>");
MODULE_DESCRIPTION("ASoC Driver for IQAudio DAC");
MODULE_LICENSE("GPL v2");
