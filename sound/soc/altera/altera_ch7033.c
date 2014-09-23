/*
 * tegra_wm8903.c - Tegra machine ASoC driver for boards using WM8903 codec.
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010-2012 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * (c) 2009, 2010 Nvidia Graphics Pvt. Ltd.
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define DRV_NAME "altera-snd-ch7033-hdmi"


struct altera_ch7033 {
	int gpio_dect;
	int reserved;
};

static int altera_ch7033_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_card *card = codec->card;
	int srate, mclk;
	int err;

	return 0;
}

static struct snd_soc_ops altera_ch7033_ops = {
	.hw_params = altera_ch7033_hw_params,
};

static int altera_ch7033_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static int altera_ch7033_remove(struct snd_soc_card *card)
{
	return 0;
}

/*
 *  this is the codec link to ch7033.
 */
static struct snd_soc_dai_link altera_ch7033_dai = {
	.name = "ch7033",
	.stream_name = "ch7033 hdmi PCM",
	.codec_dai_name = "ch7033-hdmi",
	.init = altera_ch7033_init,
	.ops = &altera_ch7033_ops,
	.dai_fmt = SND_SOC_DAIFMT_I2S,
};

static struct snd_soc_card snd_soc_altera_ch7033 = {
	.name = "altera-ch7033",
	.owner = THIS_MODULE,
	.dai_link = &altera_ch7033_dai,
	.num_links = 1,

	.remove = altera_ch7033_remove,

	.fully_routed = true,
};

static int altera_ch7033_driver_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_altera_ch7033;
	struct altera_ch7033 *machine;
	int ret;

	machine = devm_kzalloc(&pdev->dev, sizeof(struct altera_ch7033),
			       GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate altera_ch7033 struct\n");
		return -ENOMEM;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);
	
	machine->gpio_dect = 0;//reserved;

	altera_ch7033_dai.codec_of_node = of_parse_phandle(np,
						"altera,audio-codec", 0);
	if (!altera_ch7033_dai.codec_of_node) {
		dev_err(&pdev->dev,
			"Property 'altera,audio-codec' missing or invalid\n");
		ret = -EINVAL;
		goto err;
	}

	altera_ch7033_dai.cpu_of_node = of_parse_phandle(np,
			"altera,i2s-controller", 0);
	if (!altera_ch7033_dai.cpu_of_node) {
		dev_err(&pdev->dev,
			"Property 'altera,i2s-controller' missing or invalid\n");
		ret = -EINVAL;
		goto err;
	}

	altera_ch7033_dai.platform_of_node = altera_ch7033_dai.cpu_of_node;

	/* it will create sound_card */
	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err;
	}

	printk("lark board ch7033 audio card detected......\n");
	
	return 0;

err:
	return ret;
}

static int altera_ch7033_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct altera_ch7033 *machine = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id altera_ch7033_of_match[] = {
	{ .compatible = "altera,altera-audio-ch7033", },
	{},
};

static struct platform_driver altera_ch7033_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = altera_ch7033_of_match,
	},
	.probe = altera_ch7033_driver_probe,
	.remove = altera_ch7033_driver_remove,
};
module_platform_driver(altera_ch7033_driver);

MODULE_AUTHOR("embest");
MODULE_DESCRIPTION("altera fpga i2s + ch7033 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, altera_ch7033_of_match);
