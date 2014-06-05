/*
 * Designware GPIO support functions
 *
 * Copyright (C) 2012 Altera
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/platform_data/gpio-dw.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include <linux/kthread.h>


#define GPIO_INT_EN_REG_OFFSET 		(0x30)
#define GPIO_INT_MASK_REG_OFFSET 	(0x34)
#define GPIO_INT_TYPE_LEVEL_REG_OFFSET 	(0x38)
#define GPIO_INT_POLARITY_REG_OFFSET 	(0x3c)
#define GPIO_INT_STATUS_REG_OFFSET 	(0x40)
#define GPIO_PORT_A_EOI_REG_OFFSET 	(0x4c)

#define GPIO_DDR_OFFSET_PORT	 	(0x4)
#define DW_GPIO_EXT 			(0x50)
#define DW_GPIO_DR 			(0x0)
#define DRV_NAME "dw gpio"

struct dw_gpio_instance {
	struct of_mm_gpio_chip mmchip;
	u32 gpio_state;		/* GPIO state shadow register */
	u32 gpio_dir;		/* GPIO direction shadow register */
	int irq;		/* GPIO controller IRQ number */
	int irq_base;		/* base number for the "virtual" GPIO IRQs */
	u32 irq_mask;		/* IRQ mask */
	spinlock_t gpio_lock;	/* Lock used for synchronization */
	struct task_struct       *thread_handle;
};

static int dw_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);

	return (__raw_readl(mm_gc->regs + DW_GPIO_EXT) >> offset) & 1;
}

static void dw_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct dw_gpio_instance *chip = container_of(mm_gc, struct dw_gpio_instance, mmchip);
	unsigned long flags;
	u32 data_reg;

	spin_lock_irqsave(&chip->gpio_lock, flags);
	data_reg = __raw_readl(mm_gc->regs + DW_GPIO_DR);
	data_reg = (data_reg & ~(1<<offset)) | (value << offset);
	__raw_writel(data_reg, mm_gc->regs + DW_GPIO_DR);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static int dw_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct dw_gpio_instance *chip = container_of(mm_gc, struct dw_gpio_instance, mmchip);
	unsigned long flags;
	u32 gpio_ddr;

	spin_lock_irqsave(&chip->gpio_lock, flags);
	/* Set pin as input, assumes software controlled IP */
	gpio_ddr = __raw_readl(mm_gc->regs + GPIO_DDR_OFFSET_PORT);
	gpio_ddr &= ~(1 << offset);
	__raw_writel(gpio_ddr, mm_gc->regs + GPIO_DDR_OFFSET_PORT);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

static int dw_gpio_direction_output(struct gpio_chip *gc,
		unsigned offset, int value)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct dw_gpio_instance *chip = container_of(mm_gc, struct dw_gpio_instance, mmchip);
	unsigned long flags;
	u32 gpio_ddr;

	dw_gpio_set(gc, offset, value);
	
	spin_lock_irqsave(&chip->gpio_lock, flags);
	/* Set pin as output, assumes software controlled IP */
	gpio_ddr = __raw_readl(mm_gc->regs + GPIO_DDR_OFFSET_PORT);
	gpio_ddr |= (1 << offset);
	__raw_writel(gpio_ddr, mm_gc->regs + GPIO_DDR_OFFSET_PORT);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
	return 0;
}


/* add for lark board begin */
static int thread_switch_scan(void *arg)
{
	u32 gpio_value;
	u8 enable_scan = 1;
	struct of_mm_gpio_chip *mmchip = (struct of_mm_gpio_chip *)arg;
	
	printk(KERN_INFO"thread_switch_scan..............\n");
	while(enable_scan){
		msleep(100);
		gpio_value =  __raw_readl(mmchip->regs + DW_GPIO_EXT);
		if( (gpio_value&0x01e00000) != 0x01e00000){
			msleep(30);
			if( (gpio_value&0x01e00000) != 0x01e00000){
				switch(gpio_value&0x01e00000){
					case 0x01c00000:
						printk(KERN_INFO"key 1 pressed\n");
					break;
					case 0x01a00000:
						printk(KERN_INFO"key 2 pressed\n");
					break;

					case 0x01600000:
						printk(KERN_INFO"key 3 pressed\n");
					break;

					case 0x00e00000:
						printk(KERN_INFO"key 4 pressed\n");
					break;
					default:
						printk(KERN_INFO"unknow key\n");
				}
			}
		}
		
	}
	return 1;
}
/* add for lark board end */


/* 
 * dw_gpio_probe - Probe method for the GPIO device.
 * @np: pointer to device tree node
 *
 * This function probes the GPIO device in the device tree. It initializes the
 * driver data structure. It returns 0, if the driver is bound to the GPIO
 * device, or a negative value if there is an error.
 */
static int dw_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct dw_gpio_instance *chip;
	struct task_struct       *thread_handle;
	int status = 0;
	u32 reg;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
	{
		printk(KERN_ERR "%s 2 ERROR allocating memory", __func__);
		return -ENOMEM;
	}

	/* Update GPIO state shadow register with default value */
	if (of_property_read_u32(np, "resetvalue", &reg) == 0)
		chip->gpio_state = reg;

	/* Update GPIO direction shadow register with default value */
	chip->gpio_dir = 0; /* By default, all pins are inputs */

	/* Check device node for device width */
	if (of_property_read_u32(np, "width", &reg) == 0)
		chip->mmchip.gc.ngpio = reg;
	else
		chip->mmchip.gc.ngpio = 32; /* By default assume full GPIO controller */

	spin_lock_init(&chip->gpio_lock);

	chip->mmchip.gc.direction_input = dw_gpio_direction_input;
	chip->mmchip.gc.direction_output = dw_gpio_direction_output;
	chip->mmchip.gc.get = dw_gpio_get;
	chip->mmchip.gc.set = dw_gpio_set;

	/* Call the OF gpio helper to setup and register the GPIO device */
	status = of_mm_gpiochip_add(np, &chip->mmchip);
	if (status) {
		kfree(chip);
		pr_err("%s: error in probe function with status %d\n",
		       np->full_name, status);
		return status;
	}

	platform_set_drvdata(pdev, chip);

	/* add for lark board begin */
	if (of_property_read_bool(np, "enable-switch")){
		printk(KERN_INFO"switch scan enable\n");
		thread_handle = kthread_create(thread_switch_scan, &chip->mmchip, "thread_switch_scan:0:0");
		if (IS_ERR(thread_handle)) {
            pr_err("error in create thread_switch_scan thread \n");
            return 0;
    	}
		chip->thread_handle = thread_handle;
		wake_up_process(thread_handle);

	}
	else
		printk(KERN_INFO"switch scan disable\n");
	/* add for lark board end */
	
	return 0;
}

static int dw_gpio_remove(struct platform_device *pdev)
{
	/* todo check this and see that we don't have a memory leak */
	int status;
	
	struct dw_gpio_instance *chip = platform_get_drvdata(pdev);
	status = gpiochip_remove(&chip->mmchip.gc);
	if (status < 0)
		return status;
	/* add for lark board begin */
	if(chip->thread_handle){
		kthread_stop(chip->thread_handle);  
		chip->thread_handle = NULL;  
	}
	/* add for lark board end */
	kfree(chip);
	return -EIO;
}

#ifdef CONFIG_OF
static const struct of_device_id dwgpio_match[] = {
	{.compatible = DW_GPIO_COMPATIBLE,},
	{}
};
MODULE_DEVICE_TABLE(of, dwgpio_match);
#else
#define dwgpio_match NULL
#endif

static struct platform_driver dwgpio_driver = {
	.driver = {
		.name	= "dw_gpio",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(dwgpio_match),
	},
	.probe		= dw_gpio_probe,
	.remove		= dw_gpio_remove,
};

static int __init dwgpio_init(void)
{
	return platform_driver_register(&dwgpio_driver);
}
subsys_initcall(dwgpio_init);

static void __exit dwgpio_exit(void)
{
	platform_driver_unregister(&dwgpio_driver);
}
module_exit(dwgpio_exit);


MODULE_DESCRIPTION("Altera GPIO driver");
MODULE_AUTHOR("Thomas Chou <thomas@wytron.com.tw>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
