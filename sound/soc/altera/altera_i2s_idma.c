#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/delay.h>
#include "altera_i2s.h"
#include "altera_i2s_idma.h"

#define ST_RUNNING		(1<<0)
#define ST_OPENED		(1<<1)

static const struct snd_pcm_hardware idma_hardware = {
	.info =  SNDRV_PCM_INFO_INTERLEAVED |
		    /*SNDRV_PCM_INFO_NONINTERLEAVED |*/
		    SNDRV_PCM_INFO_BLOCK_TRANSFER |
		    SNDRV_PCM_INFO_MMAP |
		    SNDRV_PCM_INFO_MMAP_VALID |
		    SNDRV_PCM_INFO_PAUSE |
		    SNDRV_PCM_INFO_RESUME,
		    
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
		    SNDRV_PCM_FMTBIT_U16_LE,
	.buffer_bytes_max = MAX_IDMA_BUFFER,
	.period_bytes_min = 128,
	.period_bytes_max = MAX_IDMA_PERIOD,
	.periods_min = 1,
	.periods_max = 4,
};

struct idma_ctrl {
	spinlock_t	lock;
	int		state;
	dma_addr_t	start;
	dma_addr_t	pos;
	dma_addr_t	end;
	unsigned long	period;
	void		*token;
	void		(*cb)(void *dt, int bytes_xfer);
};

static struct idma_info {
	spinlock_t	lock;
	void		 __iomem  *regs;
} idma;


static int idma_irq;

/* init dma */
static int idma_enqueue(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct idma_ctrl *prtd = substream->runtime->private_data;

	spin_lock(&prtd->lock);
	prtd->token = (void *) substream;
	
	
	/* reset the idma the dma buffer start */
	writel(runtime->dma_addr, idma.regs + I2S_IDMA_READADDRESS);
	writel(ALTERA_I2S_FIFO, idma.regs + I2S_IDMA_WRADDRESS);

	writel(I2S_IDMA_CTRL_HALFWORD | I2S_IDMA_CTRL_NGO| 
	                           I2S_IDMA_CTRL_INT_EN| I2S_IDMA_CTRL_RADD_INC 
	                           | I2S_IDMA_CTRL_WADD_FIX | I2S_IDMA_CTRL_LEEN, 
	                           idma.regs + I2S_IDMA_CTRL);
	/* clear idma irq */
	writel(0, idma.regs + I2S_IDMA_STATUS);
	spin_unlock(&prtd->lock);
	
	return 0;
}

static void idma_setcallbk(struct snd_pcm_substream *substream,
				void (*cb)(void *, int))
{
	struct idma_ctrl *prtd = substream->runtime->private_data;

	spin_lock(&prtd->lock);
	prtd->cb = cb;
	spin_unlock(&prtd->lock);
}

static void idma_control(int op)
{
	u32 val; 

	val = readl(idma.regs + I2S_IDMA_CTRL);

	switch (op) {
	case IDMA_START:
		val = (val & I2S_IDMA_CTRL_GO_MSK) | I2S_IDMA_CTRL_GO;
		break;
	case IDMA_STOP:
		val = (val & I2S_IDMA_CTRL_GO_MSK) | I2S_IDMA_CTRL_NGO;
		break;
	default:
		return;
	}

	writel(val, idma.regs + I2S_IDMA_CTRL);
	
}

/* in irq context */
static void altra_idma_start_transfer(struct idma_ctrl *prtd,
	struct snd_pcm_substream *substream)
{
	unsigned long count;
	
	if(prtd->pos == prtd->end)
		prtd->pos = prtd->start;
		
	if(prtd->pos + prtd->period > prtd->end)
		count = prtd->end - prtd->pos;
	else
		count = prtd->period;
	
	idma_control(IDMA_STOP);
	
	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK){
		writel(prtd->pos, idma.regs + I2S_IDMA_READADDRESS);
	}
	
	writel(count, idma.regs + I2S_IDMA_XFER_SIZE);
	prtd->pos +=count;
	
	idma_control(IDMA_START);
	
}

/* in irq context */
static void idma_done(void *id, int bytes_xfer)
{
	struct snd_pcm_substream *substream = id;
	struct idma_ctrl *prtd = substream->runtime->private_data;

	if (prtd && (prtd->state & ST_RUNNING))
		snd_pcm_period_elapsed(substream);
	
	altra_idma_start_transfer(prtd, substream);
}

static int idma_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct idma_ctrl *prtd = substream->runtime->private_data;

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct altera_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);

	/* copy the dma info into runtime management */
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	/* get the max dma buffer size */
	runtime->dma_bytes = params_buffer_bytes(params);

	prtd->start = prtd->pos = runtime->dma_addr;
	prtd->period = params_period_bytes(params);
	prtd->end = runtime->dma_addr + runtime->dma_bytes;
	
	idma_setcallbk(substream, idma_done);

	/* I2S clock will sync with the ch7033, otherwise it will lose the first sound data 
	  * and it must delay above 600ms for sync.
	 */
	altera_i2s_start_playback(i2s);
	mdelay(800);
	
	return 0;
}

static int idma_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);

	return 0;
}

static int idma_prepare(struct snd_pcm_substream *substream)
{
	struct idma_ctrl *prtd = substream->runtime->private_data;

	prtd->pos = prtd->start;

	/* flush the DMA channel */
	idma_control(IDMA_STOP);
	idma_enqueue(substream);

	return 0;
}

static int idma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct idma_ctrl *prtd = substream->runtime->private_data;
	struct altera_i2s *i2s = snd_soc_dai_get_drvdata(cpu_dai);

	int ret = 0;

	spin_lock(&prtd->lock);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		prtd->state |= ST_RUNNING;
		/* it will tranfer  */
		altra_idma_start_transfer(prtd, substream);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		prtd->state &= ~ST_RUNNING;
		idma_control(IDMA_STOP);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock(&prtd->lock);

	return ret;
}

/* SNDRV_PCM_IOCTL_HWSYNC will call this function, so we must report the real
 *  bytes which the dma has transfered.
*/
static snd_pcm_uframes_t
	idma_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct idma_ctrl *prtd = runtime->private_data;
	unsigned long byte_offset;
	snd_pcm_uframes_t frames;
	unsigned long flags;

	spin_lock_irqsave(&prtd->lock, flags);

	byte_offset = prtd->pos - prtd->start;
	byte_offset -= readl(idma.regs + I2S_IDMA_XFER_SIZE);
	frames = bytes_to_frames(substream->runtime, byte_offset);
	if(frames >= runtime->buffer_size)
		frames = 0;
	
	spin_unlock_irqrestore(&prtd->lock, flags);
	return frames;
}

static int idma_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long size, offset;
	int ret;

	/* From snd_pcm_lib_mmap_iomem */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= VM_IO;
	size = vma->vm_end - vma->vm_start;
	offset = vma->vm_pgoff << PAGE_SHIFT;
	ret = io_remap_pfn_range(vma, vma->vm_start,
			(runtime->dma_addr + offset) >> PAGE_SHIFT,
			size, vma->vm_page_prot);

	return ret;
}

static irqreturn_t idma_irq_hanlder(int irqno, void *dev_id)
{
	struct idma_ctrl *prtd = (struct idma_ctrl *)dev_id;

	/* clear idma irq */
	writel(0, idma.regs + I2S_IDMA_STATUS);

	if (prtd->cb)
		prtd->cb(prtd->token, prtd->period);
	return IRQ_HANDLED;
}

static int idma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct idma_ctrl *prtd;
	int ret;

	snd_soc_set_runtime_hwparams(substream, &idma_hardware);

	prtd = kzalloc(sizeof(struct idma_ctrl), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	ret = request_irq(idma_irq, idma_irq_hanlder, 0, "fpga-i2s", prtd);
	if (ret < 0) {
		pr_err("fail to claim i2s irq , ret = %d\n", ret);
		kfree(prtd);
		return ret;
	}

	spin_lock_init(&prtd->lock);
	runtime->private_data = prtd;

	return 0;
}

static int idma_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct idma_ctrl *prtd = runtime->private_data;

	free_irq(idma_irq, prtd);

	if (!prtd)
		pr_err("idma_close called with prtd == NULL\n");

	kfree(prtd);

	writel(0, idma.regs + I2S_IDMA_CTRL);
	writel(0, idma.regs + I2S_IDMA_STATUS);

	return 0;
}

static struct snd_pcm_ops idma_ops = {
	.open		= idma_open,
	.close		= idma_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.trigger	= idma_trigger,
	.pointer	= idma_pointer,
	.mmap		= idma_mmap,
	.hw_params	= idma_hw_params,
	.hw_free	= idma_hw_free,
	.prepare	= idma_prepare,
};

static void idma_free(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;

	substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	if (!substream)
		return;

	buf = &substream->dma_buffer;
	if (!buf->area)
		return;

	dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);

	buf->area = NULL;
	buf->addr = 0;
}

static int preallocate_idma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = idma_hardware.buffer_bytes_max;

	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;

	/* Assign PCM buffer pointers, dma_alloc_writecombine will return physical address */
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size, &buf->addr, GFP_KERNEL);
	if(!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	
	return 0;
}

static u64 idma_mask = DMA_BIT_MASK(32);

static int idma_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &idma_mask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = preallocate_idma_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK);
	}

	return ret;
}


static struct snd_soc_platform_driver asoc_idma_platform = {
	.ops = &idma_ops,
	.pcm_new = idma_new,
	.pcm_free = idma_free,
};

static inline struct altera_i2s * to_altera_i2s(struct device *dev)
{
	return dev_get_drvdata(dev);
}

int altera_pcm_platform_register(struct device *dev)
{
	struct altera_i2s *i2s = to_altera_i2s(dev);
	int ret = -EINVAL;
	
	idma_irq = i2s->dma_irq;
	
	idma.regs = ioremap((unsigned int)i2s->idma_hw_reg, i2s->idma_hw_reg_size);
	if(!idma.regs)
	{
		dev_err(dev, "idma reg ioremap failed!\n");
		return -ENXIO;		
	}
	ret = snd_soc_register_platform(dev, &asoc_idma_platform);
	if(ret)
	{
		dev_err(dev, "register alsa soc platform driver failed\n");
		return -EINVAL;
	}
	spin_lock_init(&idma.lock);
	dev_info(dev, "alsa idma irq(%d) probed\n", idma_irq);

	return 0;
}
EXPORT_SYMBOL_GPL(altera_pcm_platform_register);

int altera_pcm_platform_unregister(struct device *dev)
{
	struct altera_i2s *i2s = to_altera_i2s(dev);
	
	idma_irq = i2s->dma_irq;
	
	iounmap(idma.regs);
	idma.regs = NULL;
	snd_soc_unregister_platform(dev);
	
	return 0;
}
EXPORT_SYMBOL_GPL(altera_pcm_platform_unregister);




MODULE_AUTHOR("embest");
MODULE_DESCRIPTION("altera ASoC I2S IDMA Driver");
MODULE_LICENSE("GPL");
