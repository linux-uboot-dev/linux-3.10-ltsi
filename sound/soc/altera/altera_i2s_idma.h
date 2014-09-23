#ifndef __SND_SOC_ALTERA_IDMA_H_
#define __SND_SOC_ALTERA_IDMA_H_


/* dma_state */
enum
{
	IDMA_START,
	IDMA_STOP
};

#define I2S_IDMA_STATUS							0x0
#define I2S_IDMA_READADDRESS				0x4
#define I2S_IDMA_WRADDRESS					0x8
#define I2S_IDMA_XFER_SIZE					0xc
#define I2S_IDMA_CTRL								0x18

/* IDMA control register */
#define I2S_IDMA_CTRL_BYTE					(0x1 << 0)
#define I2S_IDMA_CTRL_HALFWORD			(0x1 << 1)
#define I2S_IDMA_CTRL_WORD					(0x1 << 2)

#define I2S_IDMA_CTRL_GO_MSK				(~(0x1 << 3))
#define I2S_IDMA_CTRL_GO						(0x1 << 3)
#define I2S_IDMA_CTRL_NGO						(0x0 << 3)

#define I2S_IDMA_CTRL_INT_EN				(0x1 << 4)
#define I2S_IDMA_CTRL_REEN					(0x1 << 5)
#define I2S_IDMA_CTRL_WEEN					(0x1 << 6)
#define I2S_IDMA_CTRL_LEEN					(0x1 << 7)

#define I2S_IDMA_CTRL_RADD_MSK			(~(0x1 << 8))
#define I2S_IDMA_CTRL_RADD_INC			(0x0 << 8)
#define I2S_IDMA_CTRL_RADD_FIX			(0x1 << 8)
#define I2S_IDMA_CTRL_WADD_MSK			(~(0x1 << 9))
#define I2S_IDMA_CTRL_WADD_INC			(0x0 << 9)
#define I2S_IDMA_CTRL_WADD_FIX			(0x1 << 9)

#define I2S_IDMA_CTRL_DWORD					(0x1 << 10)
#define I2S_IDMA_CTRL_QWORD					(0x1 << 11)
#define I2S_IDMA_CTRL_SOFT_RST			(0x1 << 12)

#define MAX_IDMA_PERIOD (128 * 1024)
#define MAX_IDMA_BUFFER (160 * 1024)

int altera_pcm_platform_register(struct device *dev);
int altera_pcm_platform_unregister(struct device *dev);

#endif /* __SND_SOC_ALTERA_IDMA_H_ */
