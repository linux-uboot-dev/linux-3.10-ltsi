#ifndef __ALTERA_I2S_H__
#define __ALTERA_I2S_H__

/* dma internal view for the I2S, it isn't connected to LWF2H bridge */
#define ALTERA_I2S_FIFO					0x0

#define ALTERA_I2S_PIO_data					0x0
#define ALTERA_I2S_PIO_DIR					0x4
#define ALTERA_I2S_PIO_INTMSK				0x8
#define ALTERA_I2S_PIO_EDGCAP				0xc
#define ALTERA_I2S_PIO_OUT_SET			0x10
#define ALTERA_I2S_PIO_OUT_CLR			0x14

#define ALTERA_I2S_PIO_DIR_OUT_OFF				(3)
#define ALTERA_I2S_PIO_DIR_OUT_MASK			(0x1 << ALTERA_I2S_PIO_DIR_OUT_OFF)


#define	ALTERA_I2S_EN_SHFT						0x0
#define ALTERA_I2S_EN_MSK							(~(0x1 << (ALTERA_I2S_EN_SHFT)))
#define ALTERA_I2S_EN(reg,on)					((reg & ALTERA_I2S_EN_MSK) | (on << ALTERA_I2S_EN_SHFT))

struct altera_i2s {
	struct snd_soc_dai_driver dai;
	//struct snd_dmaengine_dai_dma_data capture_dma_data;
	//struct snd_dmaengine_dai_dma_data playback_dma_data;
	struct regmap *i2s_regmap;
	void __iomem *idma_hw_reg;
	unsigned int idma_hw_reg_size;
	int dma_irq;
	struct resource *mem;
	spinlock_t  lock;
};

void altera_i2s_start_playback(struct altera_i2s *i2s);
void altera_i2s_stop_playback(struct altera_i2s *i2s);

#endif
