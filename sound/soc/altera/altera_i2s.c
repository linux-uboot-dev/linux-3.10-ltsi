#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include "altera_i2s.h"
#include "altera_i2s_idma.h"

#define DRV_NAME "altera-i2s"

static void __iomem *regs;

static int altera_i2s_set_fmt(struct snd_soc_dai *dai,
				unsigned int fmt)
{
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:	
	case SND_SOC_DAIFMT_CBM_CFM:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		return -EINVAL;
		break;
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int altera_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_U16_LE:
		break;
	case SNDRV_PCM_FORMAT_S24_LE:	
	case SNDRV_PCM_FORMAT_S32_LE:
	default:
		return -EINVAL;
	}

	return 0;
}

void altera_i2s_start_playback(struct altera_i2s *i2s)
{
	spin_lock(&i2s->lock);
	
	regmap_update_bits(i2s->i2s_regmap, ALTERA_I2S_PIO_DIR,
		             ALTERA_I2S_PIO_DIR_OUT_MASK, 0x1 << ALTERA_I2S_PIO_DIR_OUT_OFF);

	regmap_update_bits(i2s->i2s_regmap, ALTERA_I2S_PIO_data,
				ALTERA_I2S_PIO_DIR_OUT_MASK, 0x1 << ALTERA_I2S_PIO_DIR_OUT_OFF);
	spin_unlock(&i2s->lock);
}
EXPORT_SYMBOL_GPL(altera_i2s_start_playback);

void altera_i2s_stop_playback(struct altera_i2s *i2s)
{
	spin_lock(&i2s->lock);
	
	regmap_update_bits(i2s->i2s_regmap, ALTERA_I2S_PIO_DIR,
		             ALTERA_I2S_PIO_DIR_OUT_MASK, 0x1 << ALTERA_I2S_PIO_DIR_OUT_OFF);

	regmap_update_bits(i2s->i2s_regmap, ALTERA_I2S_PIO_data,
				ALTERA_I2S_PIO_DIR_OUT_MASK, 0x0 << ALTERA_I2S_PIO_DIR_OUT_OFF);
	
	spin_unlock(&i2s->lock);
}
EXPORT_SYMBOL_GPL(altera_i2s_stop_playback);


static const struct snd_soc_dai_ops altera_i2s_dai_ops = {
	.set_fmt	= altera_i2s_set_fmt,
	.hw_params	= altera_i2s_hw_params,
};

static const struct snd_soc_dai_driver altera_i2s_dai_template = {
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		//.rates = SNDRV_PCM_RATE_8000_96000, /*it will exectue and operation with codec*/
		.rates = SNDRV_PCM_RATE_44100,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &altera_i2s_dai_ops,
};

static const struct snd_soc_component_driver altera_i2s_component = {
	.name		= DRV_NAME,
};

static bool altera_i2s_wr_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ALTERA_I2S_PIO_data:
	case ALTERA_I2S_PIO_DIR:
	case ALTERA_I2S_PIO_OUT_SET:
	case ALTERA_I2S_PIO_OUT_CLR:

		return true;
	default:
		return false;
	};
}

static bool altera_i2s_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ALTERA_I2S_PIO_data:
	case ALTERA_I2S_PIO_OUT_SET:
	case ALTERA_I2S_PIO_OUT_CLR:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config altera_i2s_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.writeable_reg = altera_i2s_wr_rd_reg,
	.readable_reg = altera_i2s_wr_rd_reg,
	.volatile_reg = altera_i2s_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
};

static int altera_i2s_platform_probe(struct platform_device *pdev)
{
	struct altera_i2s *i2s;
	struct resource *mem;
	//struct resource *memregion;
	int ret = -EINVAL;

	i2s = devm_kzalloc(&pdev->dev, sizeof(struct altera_i2s), GFP_KERNEL);
	if (!i2s) {
		dev_err(&pdev->dev, "Can't allocate altera_i2s\n");
		ret = -ENOMEM;
		goto err;
	}
	dev_set_drvdata(&pdev->dev, i2s);

	i2s->dai = altera_i2s_dai_template;
	i2s->dai.name = dev_name(&pdev->dev);

	spin_lock_init(&i2s->lock);

	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "I2s");
	if (!mem) {
		dev_err(&pdev->dev, "No memory resource\n");
		ret = -ENODEV;
		goto err_free_dev;
	}

#if 0
	/* the pio is also used by the frame reader in the frame buffer driver */
	memregion = devm_request_mem_region(&pdev->dev, mem->start,
					    resource_size(mem), DRV_NAME);
	if (!memregion) {
		dev_err(&pdev->dev, "Memory region already claimed\n");
		ret = -EBUSY;
		goto err_free_dev;
	}
#endif

	i2s->mem = mem; /* for release register memory resoure */

	regs = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
	if (!regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err_free_region;
	}

	i2s->i2s_regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    &altera_i2s_regmap_config);//on function free
	if (IS_ERR(i2s->i2s_regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(i2s->i2s_regmap);
		goto err_iounmap;
	}
	
	mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "IDMA");
	if (!mem) {
		dev_err(&pdev->dev, "No memory resource\n");
		ret = -ENODEV;
		goto err_iounmap;
	}
	i2s->idma_hw_reg = (void __iomem *)mem->start;
	i2s->idma_hw_reg_size = resource_size(mem);

	/* create a cpu_dai and dai dirver */
	ret = snd_soc_register_component(&pdev->dev, &altera_i2s_component,
					 &i2s->dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAI: %d\n", ret);
		ret = -ENOMEM;
		goto err_suspend;
	}
	
	i2s->dma_irq = platform_get_irq(pdev, 0);
	if(i2s->dma_irq <= 0)
	{
		dev_err(&pdev->dev, "Could not get the IRQ of the I2S dma: %d\n", i2s->dma_irq);
		goto err_bad_irqno;
		
	}

	ret = altera_pcm_platform_register(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM: %d\n", ret);
		goto err_unregister_component;
	}

	printk("altera i2s probed...\n");

	return 0;

err_unregister_component:
	snd_soc_unregister_component(&pdev->dev);
	
err_bad_irqno:
	
err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		;
		
err_iounmap:	
	devm_iounmap(&pdev->dev, regs);
	
err_free_region:
	devm_release_region(&pdev->dev, i2s->mem->start, resource_size(i2s->mem));

err_free_dev:
	devm_kfree(&pdev->dev, i2s);

err:
	
	return ret;
}

static int altera_i2s_platform_remove(struct platform_device *pdev)
{
	struct altera_i2s *i2s = dev_get_drvdata(&pdev->dev);

	if (!pm_runtime_status_suspended(&pdev->dev))
		;

	altera_pcm_platform_unregister(&pdev->dev);
	
	snd_soc_unregister_component(&pdev->dev);

	devm_iounmap(&pdev->dev, regs);

	devm_release_region(&pdev->dev, i2s->mem->start, resource_size(i2s->mem));
	devm_kfree(&pdev->dev, i2s);
	return 0;
}

static const struct of_device_id altera_i2s_of_match[] = {
	{ .compatible = "altera,fpga-i2s", },
	{},
};

static struct platform_driver altera_i2s_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = altera_i2s_of_match,
	},
	.probe = altera_i2s_platform_probe,
	.remove = altera_i2s_platform_remove,
};
module_platform_driver(altera_i2s_driver);

MODULE_AUTHOR("embest");
MODULE_DESCRIPTION("altera cyclone V I2S controller based fpga ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, altera_i2s_of_match);
