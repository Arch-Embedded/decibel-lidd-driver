/* Module for SSD1289 lcd with device tree support
 *
 * Author: Cezary Gapinski <cezary.gapinski@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This module using
 * SSD1289 Framebuffer
 * Heavily modified for the AM335X Cortex MPU and Linux 3.x+
 *
 * Copyright (c) 2009 Jean-Christian de Rivaz
 * Copyright (c) 2012 Christopher Mitchell
 *
 * The Solomon Systech SSD1289 chip drive TFT screen up to 240x320. This
 * driver expect the SSD1289 to be connected to a 16 bits local bus and
 * to be set in the 16 bits parallel interface mode. To use it you must
 * define in dts file
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/fb.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/pm_runtime.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/consolemap.h>
#include <linux/suspend.h>

#include "lidd_fb.h"
#include "lidd_fb_regs.h"

#define DRIVER_NAME "lidd-lcd"
#define LCD_NUM_BUFFERS	1		// was 2

static int ssd1289_setcolreg(unsigned regno, unsigned red, unsigned green,
                             unsigned blue, unsigned transp, struct fb_info *info);
static int fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg);

static struct lidd_par *display_params = NULL;

static struct fb_ops ssd1289_fbops = {
	.owner        = THIS_MODULE,
	.fb_setcolreg = ssd1289_setcolreg,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_ioctl = fb_ioctl,
//	.fb_blank = sys_blank,
};

static struct fb_fix_screeninfo ssd1289_fix __initdata = {
	.id          = DRIVER_NAME,
	.type        = FB_TYPE_PACKED_PIXELS,
	.visual      = FB_VISUAL_TRUECOLOR,
	.accel       = FB_ACCEL_NONE,
	.line_length = 240 * 2,
};

static struct fb_var_screeninfo ssd1289_var __initdata = {
	.xres		= 240,
	.yres		= 320,
	.xres_virtual	= 240,
	.yres_virtual	= 320,
	.bits_per_pixel	= 16,
	.red		= {6, 5, 0},
	.green		= {11, 5, 0},
	.blue		= {0, 6, 0},
	.activate	= FB_ACTIVATE_FORCE,	//FB_ACTIVATE_NOW,
	.height		= 320,
	.width		= 240,
	.vmode		= FB_VMODE_NONINTERLACED,
};

static int tilidd_suspend (struct device *dev);
static int tilidd_resume (struct device *dev);
static void lcd_context_save(struct lidd_par* item);
static void lcd_context_restore(struct lidd_par* item);

static int __init lidd_video_alloc(struct lidd_par *item);
static int lcd_cfg_dma(struct lidd_par *item, int burst_size,  int fifo_th);

static void lidd_dma_setstatus(struct lidd_par *item, int doenable);
static irqreturn_t ssd1289_irq_handler(int irq, void *arg);

static void __init ssd1289_setup(struct lidd_par *item);

static void LCD_Clear(struct lidd_par *item, uint16_t Color);
static void LCD_SetCursor(struct lidd_par *item, uint16_t Xpos,uint16_t Ypos);

static int tillid_pdev_probe(struct platform_device *pdev) {
	struct device_node *node = pdev->dev.of_node;
	struct lidd_par *priv;
	struct pinctrl *pinctrl;
	int ret = 0;
	unsigned int signature;
	struct fb_info *info;

	/* bail out early if no DT data: */
	if (!node) {
		dev_err(&pdev->dev, "device-tree data is missing\n");
		goto out;
	}

	/* Select default pin state */
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "pins are not configured\n");
		ret = -ENOMEM;
		goto out;
	}

	pinctrl_pm_select_default_state(&(pdev->dev));

	pr_debug("End of Initializing pins");

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		kfree(priv);
		dev_err(&pdev->dev, "failed to allocate private data\n");
		ret = -ENOMEM;
		goto out;
	}

	pr_debug("Kzalloc allocated\n");

	priv->pdev = pdev;

	pr_debug("platform pdev handler assigned to priv->pdev\n");

	priv->reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!priv->reg_res) {
		dev_err(&pdev->dev, "failed to get memory resource\n");
		ret = -ENOENT;
		goto out_item;
	}

	pr_debug("Platform get resource finished\n");

	priv->mmio = ioremap_nocache(priv->reg_res->start, resource_size(priv->reg_res));
	if (!priv->mmio) {
		dev_err(&pdev->dev, "failed to ioremap\n");
		ret = -EINVAL;
		goto out_item;
	}

	pr_debug("Remaping nocache finished\n");

	priv->lcdc_clk = clk_get(&pdev->dev, "fck");
	if (IS_ERR(priv->lcdc_clk)) {
		dev_err(&pdev->dev, "failed to get functional clock\n");
		ret = -ENODEV;
		goto out_ioremap;
	}

	pr_debug("Functional clk finished\n");


	pm_runtime_enable(&pdev->dev);

	pr_debug("pm_runtime enabled\n");

	pm_set_vt_switch(0);

	pr_debug("pm_set_vt_switched\n");
	//pm_runtime_irq_safe(&pdev->dev);
	pr_debug("pm_runtime_irq_safed\n");
	pm_runtime_get_sync(&pdev->dev);
	pr_debug("pm_get sync\n");

	ret = clk_enable(priv->lcdc_clk);
	if (ret) {
		dev_err(&pdev->dev, "Can not enable device clock\n");
		goto err_clk_get;
	}
	pr_debug("clk enabled\n");

	/* set_rate wants parameter in Hz */
	ret = clk_set_rate(priv->lcdc_clk, 100000000);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to set display clock rate to: %d\n", ret);
		ret = -ENODEV;
		goto out_clk_enable;
	}
	pr_debug("clk rate configured\n");
	display_params = priv;

	//AM335X LCD Controller Check
	signature = reg_read(priv,LCDC_PID_REG);
	dev_dbg(&pdev->dev, "%s: controller signature=0x%08x\n", __func__, signature);
	if (signature != 0x4F200800 && signature != 0x4F201000) {
		ret = -ENODEV;
		dev_err(&pdev->dev,
			"%s: unknown LCDC v2 controller signature 0x%08x\n", __func__, signature);
		goto out_clk_enable;
	}

	//Setting up AM335X LCDC Controller
	// Turn on LIDD clock and DMA clock, core clock doesn't help DMA :( ?
	reg_write(priv, LCDC_CLK_ENABLE_REG,
			LCDC_V2_DMA_CLK_EN | LCDC_V2_LIDD_CLK_EN | LCDC_V2_CORE_CLK_EN);
	reg_write(priv, LCDC_CTRL_REG, 0 | LCDC_CLK_DIVISOR(6)); //100 MHz / 6
	reg_write(priv, LCD_LIDD_CTRL, LCD_LIDD_TYPE_8080); // 8080 display, DMA (NOT YET) enabled
	reg_write(priv, LCD_CS0_CONF,
			(3 << CONF_TA_POS | 15 << R_HOLD_POS | 15 << R_STROBE_POS | 2 << R_SU_POS
			 | 2 << W_HOLD_POS | 2 << W_STROBE_POS | 1 << W_SU_POS));
	pr_debug("Initialized LCDC controller");

	//SSD1289 LCD Driver Check
	ssd1289_reg_set(priv, SSD1289_REG_OSCILLATION, 0x0001);
	signature = ssd1289_reg_get(priv,SSD1289_REG_DEV_CODE_READ);
	dev_dbg(&pdev->dev, "SSD1289 Reg value 0x%08x ", signature);

	ssd1289_setup(priv);

	//For test lcd is filled red color
	LCD_Clear(priv, Red);
	pr_debug("LCD cleared to red\n");

	info = framebuffer_alloc(sizeof(struct lidd_par), &pdev->dev);
	if (!info) {
		ret = -ENOMEM;
		dev_err(&pdev->dev,
			"%s: unable to framebuffer_alloc\n", __func__);
		goto out_clk_enable;
	}
	pr_debug("Framebuffer allocated\n");

	priv->info = info;
	info->par = priv;
	info->fbops = &ssd1289_fbops;

	// Palette setup
	priv->pseudo_palette[0] = 0;
	priv->pseudo_palette[1] = priv->pseudo_palette[7] = priv->pseudo_palette[15] = 0x0000ffff;
	info->pseudo_palette = priv->pseudo_palette;

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		ret = -ENOENT;
		goto out_info;
	}

	pr_debug("End of get irq\n");

	info->flags = FBINFO_FLAG_DEFAULT;
	info->fix = ssd1289_fix;
	info->var = ssd1289_var;
	fb_set_var(info,&ssd1289_var);
	dev_set_drvdata(&pdev->dev, info);
	pr_debug("Set drv data\n");

	ret = lidd_video_alloc(priv);
	if (ret) {
		ret = -ENOMEM;
		dev_err(&pdev->dev,
			"%s: unable to ssd1289_video_alloc\n", __func__);
		goto out_info;
	}

	// Set up all kinds of fun DMA
	reg_write(priv, LCDC_DMA_CTRL_REG, 0);					// Start out with a blank slate
	lcd_cfg_dma(priv, 16, 0);								// DMA burst and FIFO threshold
	reg_write(priv, LCDC_INT_ENABLE_SET_REG,
			  LCDC_V2_UNDERFLOW_INT_ENA | LCDC_SYNC_LOST | LCD_V2_DONE_INT_ENA);

	reg_write(priv, LCDC_DMA_FB_BASE_ADDR_0_REG, priv->dma_start);
	reg_write(priv, LCDC_DMA_FB_CEILING_ADDR_0_REG, priv->dma_end);
	reg_write(priv, LCDC_DMA_FB_BASE_ADDR_1_REG, priv->dma_start);
	reg_write(priv, LCDC_DMA_FB_CEILING_ADDR_1_REG, priv->dma_end);

	pr_debug("Finished video alloc\n");

	info->fbops = &ssd1289_fbops;
	ret = register_framebuffer(info);
	if (ret < 0) {
		ret = -EIO;
		dev_err(&pdev->dev,
			"%s: unable to register_frambuffer\n", __func__);
		goto out_pages;
	}
	dev_dbg(&pdev->dev, "Registered framebuffer.\n");

	// Set up LCD coordinates as necessary
	ssd1289_reg_set(priv, SSD1289_REG_GDDRAM_X_ADDR, 0);
	ssd1289_reg_set(priv, SSD1289_REG_GDDRAM_Y_ADDR, 0);
	//set up for data before DMA begins
	reg_write(priv, LCD_LIDD_CS0_ADDR, SSD1289_REG_GDDRAM_DATA);

	// Try to get IRQ for DMA
	ret = request_irq(priv->irq, ssd1289_irq_handler, 0,
				DRIVER_NAME, priv);
	if (ret) {
		ret = -EIO;
		goto out_framebuffer;
	}

	lidd_dma_setstatus(priv, 1);	//enable DMA

	return 0;

out_framebuffer:
	unregister_framebuffer(info);
out_pages:
	kfree((void *)priv->info->fix.smem_start);
out_info:
	framebuffer_release(info);
out_clk_enable:
	clk_disable(priv->lcdc_clk);
err_clk_get:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	clk_put(priv->lcdc_clk);
out_ioremap:
	iounmap(priv->mmio);
out_item:
	kfree(priv);
out:
	printk(KERN_EMERG "failed to probe/init lidd ssd1289 driver\n");
	dev_err(&pdev->dev,"failed to probe/init lidd ssd1289 driver\n");
	return ret;
}

static int tillid_pdev_remove(struct platform_device *pdev) {

	struct lidd_par *priv = display_params;
	struct resource *res;

	if (priv->info) {
		unregister_framebuffer(priv->info);
		framebuffer_release(priv->info);
	}

	if (priv->lcdc_clk)
		clk_disable(priv->lcdc_clk);

	if (priv->lcdc_clk)
		clk_put(priv->lcdc_clk);

	if (priv->mmio)
		iounmap(priv->mmio);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	platform_set_drvdata(pdev, NULL);

 	pm_runtime_disable(&pdev->dev);

	if (priv)
		kfree(priv);

	return 0;
}

static int __init lidd_video_alloc(struct lidd_par *item)
{
	unsigned int frame_size;
	int pages_count;
	struct platform_device *pdev = item->pdev;
	pr_debug("lidd_video_alloc start\n");

	// Calculate raw framebuffer size
	frame_size = item->info->fix.line_length * item->info->var.yres;
	dev_dbg(&pdev->dev, "%s: frame_size=%u\n",
		__func__, frame_size);

	pr_debug("After calculation frame_size\n");

	// Figure out how many full pages we need
	pages_count = frame_size / PAGE_SIZE;
	if ((pages_count * PAGE_SIZE) < frame_size) {
		pages_count++;
	}
	dev_dbg(&pdev->dev, "%s: pages_count=%u per each of %d buffer(s)\n",
		__func__, pages_count, LCD_NUM_BUFFERS);

	pr_debug("After pages count\n");

	// Reserve DMA-able RAM, set up fix.
	item->vram_size = pages_count * PAGE_SIZE * LCD_NUM_BUFFERS;
	item->vram_virt = dma_alloc_coherent(NULL,
							item->vram_size,
							(resource_size_t *) &item->vram_phys,
							GFP_KERNEL | GFP_DMA);
	if (!item->vram_virt) {
		dev_err(&pdev->dev, "%s: unable to vmalloc\n", __func__);
		return -ENOMEM;
	}

	pr_debug("after dma alloc\n");

	memset(item->vram_virt, 0, item->vram_size);
	item->info->fix.smem_start = (unsigned long)item->vram_virt;
	item->info->fix.smem_len = item->vram_size;

	item->info->screen_base = (char __iomem *)item->vram_virt;
	item->info->screen_size = (unsigned int)item->vram_size;

	item->dma_start = item->vram_phys;
	item->dma_end   = item->dma_start + (item->info->var.yres * item->info->fix.line_length) - 1;
	dev_dbg(&pdev->dev, "%s: DMA set from 0x%d to 0x%d, %ld bytes\n",__func__,
			item->dma_start,item->dma_end,item->vram_size);

	return 0;
}

/* Configure the Burst Size and fifo threhold of DMA */
static int lcd_cfg_dma(struct lidd_par *item, int burst_size,  int fifo_th)
{
	u32 reg;

	reg = (reg_read(item, LCDC_DMA_CTRL_REG) & 0x00000005);	// | LCD_DUAL_FRAME_BUFFER_ENABLE; not for LIDD DMA??
	switch (burst_size) {
	case 1:
		reg |= LCDC_DMA_BURST_SIZE(LCDC_DMA_BURST_1);
		break;
	case 2:
		reg |= LCDC_DMA_BURST_SIZE(LCDC_DMA_BURST_2);
		break;
	case 4:
		reg |= LCDC_DMA_BURST_SIZE(LCDC_DMA_BURST_4);
		break;
	case 8:
		reg |= LCDC_DMA_BURST_SIZE(LCDC_DMA_BURST_8);
		break;
	case 16:
		reg |= LCDC_DMA_BURST_SIZE(LCDC_DMA_BURST_16);
		break;
	default:
		return -EINVAL;
	}

	reg |= (fifo_th << 8);
	reg |= BIT(2);					// EOF_INTEN

	reg_write(item, LCDC_DMA_CTRL_REG, reg);

	return 0;
}

/* IRQ handler for version 2 of LCDC */
static irqreturn_t ssd1289_irq_handler(int irq, void *arg)
{
	struct lidd_par *item = (struct lidd_par *)arg;
	u32 stat = reg_read(item,LCDC_MASKED_STAT_REG);

	if ((stat & LCDC_SYNC_LOST) || (stat & LCDC_FIFO_UNDERFLOW)) {
		printk(KERN_ERR "LCDC sync lost or underflow error occured\nNot sure what to do...\n");
		reg_write(item, LCDC_MASKED_STAT_REG, stat);
	} else {
		lidd_dma_setstatus(item, 0);	//disable DMA

		reg_write(item, LCDC_MASKED_STAT_REG, stat);

		ssd1289_reg_set(item, SSD1289_REG_GDDRAM_X_ADDR, 0);
		ssd1289_reg_set(item, SSD1289_REG_GDDRAM_Y_ADDR, 0);
		//set up for data before DMA begins
		reg_write(item, LCD_LIDD_CS0_ADDR, SSD1289_REG_GDDRAM_DATA);

		if (!item->suspending) { //Don't re-enable DMA if we want to suspend or disable the driver
			lidd_dma_setstatus(item, 1); //enable DMA
		} else {
			item->suspending = 0;
		}
	}
	//reg_write(item, LCD_END_OF_INT_IND_REG, 0); //not a thing...?
	return IRQ_HANDLED;
}

static void lidd_dma_setstatus(struct lidd_par *item, int doenable) {
	//enable DMA
	unsigned int cfg;

	cfg = reg_read(item,LCD_LIDD_CTRL);
	cfg = (cfg & ~BIT(8)) | ((doenable&1) << 8);	//enable or disable DMA
	reg_write(item, LCD_LIDD_CTRL, cfg);
}

static int ssd1289_suspend (struct platform_device *dev, pm_message_t state) {
	return 0;
}

static int ssd1289_resume (struct platform_device *dev) {
	return 0;
}

static const struct dev_pm_ops tilidd_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tilidd_suspend, tilidd_resume)
};

static struct of_device_id cglidd_of_match[] = { { .compatible =
		"cg,am33xx-lidd", }, { }, };
MODULE_DEVICE_TABLE(of, cglidd_of_match);

static struct platform_driver tillid_platform_driver =
{ .probe = tillid_pdev_probe,
  .remove = tillid_pdev_remove,
  .resume = ssd1289_resume,
  .suspend = ssd1289_suspend,
  .driver = {
		  .name = DRIVER_NAME,
		  .pm     = &tilidd_pm_ops,
		  .of_match_table = cglidd_of_match, },
};

static int __init tillid_fb_init(void) {
	pr_debug("Init tillid\n");
	return platform_driver_register(&tillid_platform_driver);
}

static void __exit tillid_fb_fini(void) {
	pr_debug("Exit tillid\n");
	platform_driver_unregister(&tillid_platform_driver);
}

static int tilidd_suspend (struct device *dev) {

//	int i = 5000;
//	u32 stat;
//	struct ssd1289* item = (struct ssd1289*)dev->dev.platform_data;
//
//	console_lock();
//	fb_set_suspend(item->info, 1);
//	item->suspending = 1;
//	do {
//		mdelay(1);
//	} while (item->suspending && (i--));
//
//	if (item->suspending) {
//		dev_err(&dev->dev,"Failed to suspend %s driver\n",DRIVER_NAME);
//		return 1;
//	}
//	stat = lcdc_read(item,LCD_STAT_REG);
//	lcdc_write(item,stat,LCD_MASKED_STAT_REG);
//
//	// PT=0 VLE=1 SPT=0 GON=1 DTE=1 CM=0 D=0 (Turn off the display)
//	ssd1289_reg_set(item, SSD1289_REG_DISPLAY_CTRL, 0x0230);
//
//	lcd_context_save(item);
	pm_runtime_put(dev);
//	console_unlock();

	return 0;
}

static int tilidd_resume (struct device *dev) {
//	struct llid_par* item = (struct llid_par9*)dev->dev.platform_data;

//	console_lock();
	pm_runtime_get_sync(dev);
//	msleep(1);

//	lcd_context_restore(item);

	// Do SSD1289 setup
//	ssd1289_setup(item);

	// Set up LCD coordinates as necessary
//	ssd1289_reg_set(item, SSD1289_REG_GDDRAM_X_ADDR, 0);
//	ssd1289_reg_set(item, SSD1289_REG_GDDRAM_Y_ADDR, 0);
//	lcdc_write(item,SSD1289_REG_GDDRAM_DATA, LCD_LIDD_CS0_ADDR);	//set up for data before DMA begins
//	fb_set_suspend(item->info, 0);
//	ssd1289_dma_setstatus(item, 1);	//enable DMA
//	console_unlock();
//
//	dev_dbg(&dev->dev,"Resumed.");
	return 0;
}

static void lcd_context_save(struct lidd_par* item)
{
//	reg_context.clk_enable = lcdc_read(item,LCD_CLK_ENABLE_REG);
//	reg_context.ctrl = lcdc_read(item,LCD_CTRL);
//	reg_context.dma_ctrl = lcdc_read(item,LCD_DMA_CTRL_REG);
//	reg_context.int_enable_set = lcdc_read(item,LCD_INT_ENABLE_SET_REG);
//	reg_context.dma_frm_buf_base_addr_0 =
//		lcdc_read(item,LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
//	reg_context.dma_frm_buf_ceiling_addr_0 =
//		lcdc_read(item,LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
//	reg_context.dma_frm_buf_base_addr_1 =
//		lcdc_read(item,LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
//	reg_context.dma_frm_buf_ceiling_addr_1 =
//		lcdc_read(item,LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
	return;
}

static void lcd_context_restore(struct lidd_par* item)
{
//	lcdc_write(item,reg_context.clk_enable, LCD_CLK_ENABLE_REG);
//	lcdc_write(item,reg_context.ctrl, LCD_CTRL);
//	lcdc_write(item,reg_context.dma_ctrl, LCD_DMA_CTRL_REG);
//	lcdc_write(item,reg_context.int_enable_set, LCD_INT_ENABLE_SET_REG);
//	lcdc_write(item,reg_context.dma_frm_buf_base_addr_0,
//			LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
//	lcdc_write(item,reg_context.dma_frm_buf_ceiling_addr_0,
//			LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
//	lcdc_write(item,reg_context.dma_frm_buf_base_addr_1,
//			LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
//	lcdc_write(item,reg_context.dma_frm_buf_ceiling_addr_1,
//			LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
	return;
}

static void __init ssd1289_setup(struct lidd_par *item)
{
	// OSCEN=1
	ssd1289_reg_set(item, SSD1289_REG_OSCILLATION, 0x0001);
	// DCT=b1010=fosc/4 BT=b001=VGH:+6,VGL:-4
	// DC=b1010=fosc/4 AP=b010=small to medium
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_1, 0xa2a4);
	// VRC=b100:5.5V
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_2, 0x0004);
	// VRH=b1000:Vref*2.165
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_3, 0x0308);
	// VCOMG=1 VDV=b1000:VLCD63*1.05
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_4, 0x3000);
	// nOTP=1 VCM=0x2a:VLCD63*0.77
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_5, 0x00ea);
	// RL=0 REV=1 CAD=0 BGR=1 SM=0 TB=1 MUX=0x13f=319
	ssd1289_reg_set(item, SSD1289_REG_DRIVER_OUT_CTRL, 0x2b3f);
	// FLD=0 ENWS=0 D/C=1 EOR=1 WSMD=0 NW=0x00=0
	ssd1289_reg_set(item, SSD1289_REG_LCD_DRIVE_AC, 0x0600);
	// SLP=0
	ssd1289_reg_set(item, SSD1289_REG_SLEEP_MODE, 0x0000);

	msleep(40);
	// VSMode=0 DFM=3:65k TRAMS=0 OEDef=0 WMode=0 Dmode=0
	// TY=0 ID=3 AM=0 LG=0
	ssd1289_reg_set(item, SSD1289_REG_ENTRY_MODE, 0x6030);
	// PT=0 VLE=1 SPT=0 GON=1 DTE=1 CM=0 D=3
	ssd1289_reg_set(item, SSD1289_REG_DISPLAY_CTRL, 0x0233);
	// NO=0 SDT=0 EQ=0 DIV=0 SDIV=1 SRTN=1 RTN=9:25 clock
	// ssd1289_reg_set(item, SSD1289_REG_FRAME_CYCLE, 0x0039);
	// NO=0 SDT=0 EQ=0 DIV=0 SDIV=0 SRTN=0 RTN=9:25 clock
	ssd1289_reg_set(item, SSD1289_REG_FRAME_CYCLE, 0x0009);
	// SCN=0
	ssd1289_reg_set(item, SSD1289_REG_GATE_SCAN_START, 0x0000);

	// PKP1=7 PKP0=7
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_1, 0x0707);
	// PKP3=2 PKP2=4
	ssd1289_reg_set(item, SSD1289_REG_GAMME_CTRL_2, 0x0204);
	// PKP5=2 PKP4=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_3, 0x0204);
	// PRP1=5 PRP0=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_4, 0x0502);
	// PKN1=5 PKN0=7
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_5, 0x0507);
	// PKN3=2 PNN2=4
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_6, 0x0204);
	// PKN5=2 PKN4=4
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_7, 0x0204);
	// PRN1=5 PRN0=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_8, 0x0502);
	// VRP1=3 VRP0=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_9, 0x0302);
	// VRN1=3 VRN0=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_10, 0x0302);

	// WMR=0 WMG=0
	ssd1289_reg_set(item, SSD1289_REG_WR_DATA_MASK_1, 0x0000);
	// WMB=0
	ssd1289_reg_set(item, SSD1289_REG_WR_DATA_MASK_2, 0x0000);
	// OSC=b1010:548k
	ssd1289_reg_set(item, SSD1289_REG_FRAME_FREQUENCY, 0xa000);
	// SS1=0
	ssd1289_reg_set(item, SSD1289_REG_FIRST_WIN_START, 0x0000);
	// SE1=319
	ssd1289_reg_set(item, SSD1289_REG_FIRST_WIN_END,
			(320 - 1));
	// SS2=0
	ssd1289_reg_set(item, SSD1289_REG_SECND_WIN_START, 0x0000);
	// SE2=0
	ssd1289_reg_set(item, SSD1289_REG_SECND_WIN_END, 0x0000);
	// VL1=0
	ssd1289_reg_set(item, SSD1289_REG_V_SCROLL_CTRL_1, 0x0000);
	// VL2=0
	ssd1289_reg_set(item, SSD1289_REG_V_SCROLL_CTRL_2, 0x0000);
	// HEA=0xef=239 HSA=0
	ssd1289_reg_set(item, SSD1289_REG_H_RAM_ADR_POS,
			(240 - 1) << 8);
	// VSA=0
	ssd1289_reg_set(item, SSD1289_REG_V_RAM_ADR_START, 0x0000);
	// VEA=0x13f=319
	ssd1289_reg_set(item, SSD1289_REG_V_RAM_ADR_END,
			(320 - 1));
}

static int ssd1289_setcolreg(unsigned regno, unsigned red, unsigned green,
			      unsigned blue, unsigned transp,
			      struct fb_info *info)
{
	struct lidd_par *par = info->par;

	if (regno >= 16)
		return 1;

	if (info->fix.visual == FB_VISUAL_DIRECTCOLOR)
		return 1;

	if ((info->var.bits_per_pixel == 16) && regno < 16) {
		red >>= (16 - info->var.red.length);
		red <<= info->var.red.offset;

		green >>= (16 - info->var.green.length);
		green <<= info->var.green.offset;

		blue >>= (16 - info->var.blue.length);
		blue <<= info->var.blue.offset;

		par->pseudo_palette[regno] = red | green | blue;
	}

	return 0;
}

static int fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg) {
	//struct lcd_sync_arg sync_arg;

	switch (cmd) {
		case FBIOPUT_VSCREENINFO:
			return 0;
		default:
			return -EINVAL;
	}
}

static void LCD_Clear(struct lidd_par *item, uint16_t Color)
{
  uint32_t index=0;
  LCD_SetCursor(item, 0,0);
  LCD_WriteRAM_Prepare(item); /* Prepare to write GRAM */
  for(index=0;index<76800;index++)
	  reg_write(item, LCD_LIDD_CS0_DATA, (unsigned int)Color);
}

static void LCD_SetCursor(struct lidd_par *item, uint16_t Xpos,uint16_t Ypos)
{
	ssd1289_reg_set(item, SSD1289_REG_GDDRAM_X_ADDR,Xpos); /* Row */
	ssd1289_reg_set(item, SSD1289_REG_GDDRAM_Y_ADDR,Ypos); /* Line */
}

module_init(tillid_fb_init);
module_exit(tillid_fb_fini);

MODULE_AUTHOR("Cezary Gapinski");
MODULE_DESCRIPTION("TI LLID LCD Module");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");


