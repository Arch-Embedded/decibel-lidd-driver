/* Module for st7789 lcd with device tree support
 *
 * Author: Chris Desjardins <chris@arch-embedded.com>
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
 * The Sitronix ST7789V chip drive TFT screen up to 240x320. This
 * driver expect the ST7789V to be connected to an 8 bits local bus and
 * to be set in the 8 bits parallel interface mode. To use it you must
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
#include <linux/workqueue.h>
#include <linux/lcm.h>

#include "lidd_fb.h"
#include "lidd_fb_regs.h"

#define DRIVER_NAME "lidd-lcd"
#define LCD_SCREEN_WIDTH 240
#define LCD_SCREEN_HEIGHT 240
#define LCD_BITS_PER_PIXEL 16
#define LCD_INTERFACE_WIDTH 8

#define LIDD_FB_EXTERNAL_INDEX 0 // index of the fb that is exposed externally like a normal framebuffer
#define LIDD_FB_INTERNAL_INDEX 1 // index of the fb that is used internally with the 8bit gaps between each word

#define LIDD_FB_DMA_TIMEOUT 1000

enum dma_channel_state {
  	DMA_CHAN_ALLOTED,
	DMA_CHAN_NOT_ALLOTED,
};

static int tilidd_suspend(struct device *dev);
static int tilidd_resume(struct device *dev);
// static void lcd_context_save(struct lidd_par* item);
// static void lcd_context_restore(struct lidd_par* item);

static int lidd_video_alloc(struct lidd_par *item);
static int lcd_cfg_dma(struct lidd_par *item, int burst_size, int fifo_th);

static void lidd_dma_enable(struct lidd_par *item, bool doenable);
static irqreturn_t lidd_dma_irq_handler(int irq, void *arg);

static void st7789v_setup(struct lidd_par *item);

static void st7789v_SetFrameDimensions(struct lidd_par *item, uint16_t Xpos, uint16_t Ypos);

static int fb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info);
static int fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg);
static void fb_fillrect(struct fb_info *p, const struct fb_fillrect *rect);
static void fb_copyarea(struct fb_info *p, const struct fb_copyarea *area);
static ssize_t fb_read(struct fb_info *info, char __user *buf, size_t count, loff_t *ppos);
static ssize_t fb_write(struct fb_info *info, const char __user *buf, size_t count, loff_t *ppos);
void fb_imageblit(struct fb_info *p, const struct fb_image *image);
static void lidd_fb_startdma_worker(struct work_struct *work);
static int lidd_fb_blank(int blank_mode, struct fb_info *info);

static struct fb_ops st7789v_fbops =
{
    .owner        = THIS_MODULE,
    .fb_setcolreg = fb_setcolreg,
    .fb_fillrect  = fb_fillrect,
    .fb_copyarea  = fb_copyarea,
    .fb_imageblit = cfb_imageblit,
    .fb_write     = fb_write,
    .fb_read      = fb_read,
    .fb_ioctl     = fb_ioctl,
    // .fb_set_par   = lidd_fb_set_par,
    .fb_blank     = lidd_fb_blank,
};

static struct fb_fix_screeninfo panel_fix =
{
    .id          = DRIVER_NAME,
    .type        = FB_TYPE_PACKED_PIXELS,
    .visual      = FB_VISUAL_TRUECOLOR,
    .accel       = FB_ACCEL_NONE,
    .line_length = (LCD_SCREEN_WIDTH * LCD_BITS_PER_PIXEL) / 8
};

static struct fb_var_screeninfo panel_var =
{
    .xres = LCD_SCREEN_WIDTH,
    .yres = LCD_SCREEN_HEIGHT,
    .xres_virtual = LCD_SCREEN_WIDTH,
    .yres_virtual = LCD_SCREEN_HEIGHT,
    .bits_per_pixel = LCD_BITS_PER_PIXEL,
    .red = {11, 5, 0},
    .green = {5, 6, 0},
    .blue = {0, 5, 0},
    .activate = FB_ACTIVATE_FORCE, // FB_ACTIVATE_NOW,
    .width = LCD_SCREEN_HEIGHT,
    .height = LCD_SCREEN_HEIGHT,
    .vmode = FB_VMODE_NONINTERLACED,
    .pixclock = 10000,
    .left_margin = 6,
    .right_margin = 8,
    .upper_margin = 2,
    .lower_margin = 2,
    .hsync_len = 0,
    .vsync_len = 0,
};

// This kills the CPU but is required for apps that use a memory mapped framebuffer
// TODO: Use the DMA engine (if possible) to get rid of this crap!
static void setframe(struct lidd_par *item)
{
    int i;
    uint16_t *dst = (uint16_t *)item->dma_par[LIDD_FB_INTERNAL_INDEX].vram_virt;
    uint16_t *src = (uint16_t *)item->dma_par[LIDD_FB_EXTERNAL_INDEX].vram_virt;
    for (i = 0; i < (LCD_SCREEN_WIDTH * LCD_SCREEN_HEIGHT); i++)
    {
        *(dst + 0) = SET_VALHI(*src);
        *(dst + 1) = SET_VALLO(*src);
        src += 1;
        dst += 2;
    }
}

static void lidd_fb_dma_tx_callback(void *dma_async_param, const struct dmaengine_result *result)
{
    struct lidd_par *priv = (struct lidd_par *)dma_async_param;

    if (result->result == DMA_TRANS_NOERROR) {
        /* Turn display on */
        reg_write(priv, LCD_LIDD_CS0_ADDR, ST7789V_DISPON);
        /* Reset RAM interface to accept new data */
        reg_write(priv, LCD_LIDD_CS0_ADDR, ST7789V_RAMWR);

        priv->tx_status = 1;
        priv->dma_done = true;
        wake_up_interruptible(&priv->dma_done_wq);

        /* Signal EOI to LCDC now that also DMA is complete */
        reg_write(priv, LCDC_END_OF_INT_IND_REG, 0);
    }
    else
    {
        pr_err("%s: DMA error, result %d\n", __func__, result->result);
    }
}

static int lidd_fb_prepare_dma(struct lidd_par *priv)
{
    struct dma_async_tx_descriptor *tx;
    struct dma_chan *chan = priv->dma_channel;
    struct dma_interleaved_template *xt = &priv->xt;
    enum dma_status status;

    pr_info("priv addr: %08x\n", (u32)priv);

    xt->src_start = priv->dma_par[LIDD_FB_EXTERNAL_INDEX].vram_phys;
    xt->dst_start = priv->dma_par[LIDD_FB_INTERNAL_INDEX].vram_phys;

    xt->numf = priv->dma_par[LIDD_FB_EXTERNAL_INDEX].vram_size;
    xt->frame_size = 1;
    xt->sgl[0].size = 1;
    xt->sgl[0].icg = (LCD_BITS_PER_PIXEL - LCD_INTERFACE_WIDTH) / 8;

    xt->dir = DMA_MEM_TO_MEM;
    xt->src_sgl = false;
    xt->src_inc = true;
    xt->dst_sgl = true;
    xt->dst_inc = true;

    pr_info("chan addr: %08x\n", (u32)chan);
    pr_info("chan device addr: %08x\n", (u32)chan->device);
    pr_info("chan device->prep_interleaved: %08x\n", (u32)chan->device->device_prep_interleaved_dma);

    tx = dmaengine_prep_interleaved_dma(chan, xt, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
    if (tx == NULL)
    {
        pr_err("%s: DMA interleaved prep error\n", __func__);
        return -EINVAL;
    }

    tx->callback_result = lidd_fb_dma_tx_callback;
    tx->callback_param = priv;

    priv->cookie = dmaengine_submit(tx);
    if (dma_submit_error(priv->cookie))
    {
        pr_err("%s: dmaengine_submit failed (%d)\n", __func__, priv->cookie);
        return -EINVAL;
    }

    priv->tx_status = 0;
    priv->dma_done = false;
    dma_async_issue_pending(chan);

    wait_event_interruptible_timeout(priv->dma_done_wq,
                                     priv->tx_status == 1 && priv->dma_done == true,
                                     LIDD_FB_DMA_TIMEOUT);

    status = dma_async_is_tx_complete(chan, priv->cookie, NULL, NULL);

    if (priv->tx_status == 0)
    {
        pr_err("%s: Timeout while waiting for DMA\n", __func__);
        dmaengine_terminate_sync(chan);
        return -EINVAL;
    }
    else if (status != DMA_COMPLETE)
    {
        pr_err("%s: DMA completion %s status\n", __func__,
               status == DMA_ERROR ? "error" : "busy");
        dmaengine_terminate_sync(chan);
        return -EINVAL;
    }

    return 0;
}

static int lidd_fb_setup_dma(struct lidd_par *priv)
{
    dma_cap_mask_t mask;

    priv->dma_mask = DMA_BIT_MASK(32);

    dma_cap_zero(mask);
    dma_cap_set(DMA_INTERLEAVE, mask);
    priv->dma_channel = dma_request_chan_by_mask(&mask);
    if (IS_ERR(priv->dma_channel))
    {
        pr_err("%s: DMA channel request failed\n", __func__);
        priv->req_status = DMA_CHAN_NOT_ALLOTED;
        return PTR_ERR(priv->dma_channel);
    }

    priv->req_status = DMA_CHAN_ALLOTED;



    pr_info("chan addr: %08x\n", (u32)priv->dma_channel);
    pr_info("chan device addr: %08x\n", (u32)priv->dma_channel->device);
    pr_info("chan device->prep_interleaved: %08x\n", (u32)priv->dma_channel->device->device_prep_interleaved_dma);

    return 0;
}

static void lidd_fb_startdma_worker(struct work_struct *work)
{
    struct lidd_par *priv = container_of(work,
                                         struct lidd_par, dma_work);
    int ret;

    pr_info("priv addr: %08x\n", (u32)priv);

    ret = lidd_fb_prepare_dma(priv);
    if (ret)
    {
        pr_err("%s: DMA prepare failed\n", __func__);
        return;
    }

    if (!priv->frame_done)
    {
        priv->frame_done = true;
        wake_up_interruptible(&priv->frame_done_wq);
    }
}

#if 0
static int lidd_fb_set_par(struct fb_info *info)
{
    struct lidd_par *priv = info->par;
    int ret;

    return 0;
}
#endif

static int lidd_fb_blank(int blank_mode, struct fb_info *info)
{
    struct lidd_par *priv = info->par;

    switch (blank_mode)
    {
    case FB_BLANK_UNBLANK:
        pr_info("Unblanking\n");
        pr_info("priv addr: %08x\n", (u32)priv);

        priv->frame_done = false;
        lidd_dma_enable(priv, true);
        wait_event_interruptible_timeout(priv->frame_done_wq, priv->frame_done == true, msecs_to_jiffies(50));
        reg_write(priv, LCD_LIDD_CS0_ADDR, ST7789V_DISPON);
        break;
    case FB_BLANK_NORMAL:
    case FB_BLANK_VSYNC_SUSPEND:
    case FB_BLANK_HSYNC_SUSPEND:
    case FB_BLANK_POWERDOWN:
        pr_info("Blanking\n");
        lidd_dma_enable(priv, false);
        reg_write(priv, LCD_LIDD_CS0_ADDR, ST7789V_DISPOFF);
        break;
    }

    return 0;
}


static int tilidd_pdev_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    struct lidd_par *priv;
    struct pinctrl *pinctrl;
    int ret = 0;
    int irq;
    unsigned int signature;
    struct fb_info *info;
    struct gpio_desc *enable_gpio;

    enable_gpio = devm_gpiod_get_optional(&pdev->dev, "enable", GPIOD_OUT_HIGH);
    if (IS_ERR(enable_gpio))
    {
        dev_err(&pdev->dev, "failed to request enable GPIO\n");
    }

    /* bail out early if no DT data: */
    if (!node)
    {
        dev_err(&pdev->dev, "device-tree data is missing\n");
        goto out;
    }

    /* Select default pin state */
    pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
    if (IS_ERR(pinctrl))
    {
        dev_err(&pdev->dev, "pins are not configured\n");
        ret = -ENOMEM;
        goto out;
    }

    pinctrl_pm_select_default_state(&(pdev->dev));

    pr_debug("End of Initializing pins");

    info = framebuffer_alloc(sizeof(*priv), &pdev->dev);
    if (!info)
    {
        ret = -ENOMEM;
        dev_err(&pdev->dev, "%s: unable to framebuffer_alloc\n", __func__);
        goto out_clk_enable;
    }

    /* Get our private data pointer */
    priv = info->par;
    memset(priv, 0, sizeof(*priv));

    pr_debug("Framebuffer allocated\n");

    priv->pdev = pdev;

    pr_debug("Set drv data\n");
    dev_set_drvdata(&pdev->dev, priv);

    pr_debug("platform pdev handler assigned to priv->pdev\n");

    priv->reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!priv->reg_res)
    {
        dev_err(&pdev->dev, "failed to get memory resource\n");
        ret = -ENOENT;
        goto out_item;
    }

    pr_debug("Platform get resource finished\n");

    priv->mmio = ioremap(priv->reg_res->start, resource_size(priv->reg_res));
    if (!priv->mmio)
    {
        dev_err(&pdev->dev, "failed to ioremap\n");
        ret = -EINVAL;
        goto out_item;
    }

    pr_debug("Remaping nocache finished\n");

    priv->lcdc_clk = clk_get(&pdev->dev, "fck");
    if (IS_ERR(priv->lcdc_clk))
    {
        dev_err(&pdev->dev, "failed to get functional clock\n");
        ret = -ENODEV;
        goto out_ioremap;
    }

    pr_debug("Functional clk finished\n");

    pm_runtime_enable(&pdev->dev);

    pr_debug("pm_runtime enabled\n");

    // pm_set_vt_switch(0);
    // pr_debug("pm_set_vt_switched\n");
    // pm_runtime_irq_safe(&pdev->dev);
    // pr_debug("pm_runtime_irq_safed\n");

    pm_runtime_get_sync(&pdev->dev);
    pr_debug("pm_get sync\n");

    ret = clk_enable(priv->lcdc_clk);
    if (ret)
    {
        dev_err(&pdev->dev, "Can not enable device clock\n");
        goto err_clk_get;
    }
    pr_debug("clk enabled\n");

    /* set_rate wants parameter in Hz */
    ret = clk_set_rate(priv->lcdc_clk, 100000000);
    if (ret < 0)
    {
        dev_err(&pdev->dev, "failed to set display clock rate to: %d\n", ret);
        ret = -ENODEV;
        goto out_clk_enable;
    }
    pr_debug("clk rate configured\n");

    // AM335X LCD Controller Check
    signature = reg_read(priv, LCDC_PID_REG);
    dev_dbg(&pdev->dev, "%s: controller signature=0x%08x\n", __func__, signature);
    if (signature != 0x4F200800 && signature != 0x4F201000)
    {
        ret = -ENODEV;
        dev_err(&pdev->dev, "%s: unknown LCDC v2 controller signature 0x%08x\n", __func__, signature);
        goto out_clk_enable;
    }

    // Setting up AM335X LCDC Controller
    //  Turn on LIDD clock and DMA clock, core clock doesn't help DMA :( ?
    reg_write(priv, LCDC_CLK_ENABLE_REG, LCDC_V2_DMA_CLK_EN | LCDC_V2_LIDD_CLK_EN | LCDC_V2_CORE_CLK_EN);
    reg_write(priv, LCDC_CTRL_REG, 0 | LCDC_CLK_DIVISOR(1));
    reg_write(priv, LCD_LIDD_CTRL, LCD_LIDD_TYPE_8080);
    reg_write(priv, LCD_CS0_CONF, (1 << CONF_TA_POS | 9 << R_HOLD_POS | 36 << R_STROBE_POS | 1 << R_SU_POS | 4 << W_HOLD_POS | 2 << W_STROBE_POS | 1 << W_SU_POS));
    reg_write(priv, LCDC_SYSCONFIG, 0x14);

    pr_debug("Initialized LCDC controller");
    st7789v_setup(priv);

    priv->info = info;
    info->fbops = &st7789v_fbops;

    // Palette setup
    priv->pseudo_palette[0] = 0;
    priv->pseudo_palette[1] = priv->pseudo_palette[7] = priv->pseudo_palette[15] = 0x0000ffff;
    info->pseudo_palette = priv->pseudo_palette;

    irq = platform_get_irq(pdev, 0);
    if (irq < 0)
    {
        ret = -ENOENT;
        goto out_info;
    }

    pr_debug("End of get irq\n");

    info->flags = FBINFO_FLAG_DEFAULT;
    info->fix = panel_fix;
    info->var = panel_var;
    fb_set_var(info, &panel_var);

    ret = lidd_video_alloc(priv);
    if (ret)
    {
        ret = -ENOMEM;
        dev_err(&pdev->dev, "%s: unable to ldd_video_alloc\n", __func__);
        goto out_info;
    }

    // Set up all kinds of fun DMA
    lcd_cfg_dma(priv, 16, 0);
    reg_write(priv, LCDC_INT_ENABLE_SET_REG, LCDC_FIFO_UNDERFLOW | LCDC_SYNC_LOST | LCDC_V2_DONE_INT_ENA);
    reg_write(priv, LCDC_DMA_FB_BASE_ADDR_0_REG, priv->dma_par[LIDD_FB_INTERNAL_INDEX].vram_phys);
    reg_write(priv, LCDC_DMA_FB_CEILING_ADDR_0_REG, priv->dma_par[LIDD_FB_INTERNAL_INDEX].vram_phys + priv->dma_par[LIDD_FB_INTERNAL_INDEX].vram_size - 1);

    pr_debug("Finished video alloc\n");

    /* Init DMA stuff */
    init_waitqueue_head(&priv->dma_done_wq);
    init_waitqueue_head(&priv->frame_done_wq);
    INIT_WORK(&priv->dma_work, lidd_fb_startdma_worker);

    ret = lidd_fb_setup_dma(priv);
    if (ret)
    {
        pr_err("%s: DMA setup failed\n", __func__);
        goto out_framebuffer;
    }
    else
    {
        pr_info("%s: DMA setup OK\n", __func__);
    }

    init_waitqueue_head(&priv->vsync_wait);

    // Try to get IRQ for DMA
    ret = request_irq(irq, lidd_dma_irq_handler, 0, DRIVER_NAME, priv);
    if (ret)
    {
        ret = -EIO;
        goto out_framebuffer;
    }

    ret = register_framebuffer(info);
    if (ret < 0)
    {
        ret = -EIO;
        dev_err(&pdev->dev, "%s: unable to register_frambuffer\n", __func__);
        goto out_pages;
    }
    dev_dbg(&pdev->dev, "Registered framebuffer.\n");

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
    printk(KERN_EMERG "failed to probe/init lidd driver\n");
    return ret;
}

static int tilidd_pdev_remove(struct platform_device *pdev)
{
    struct lidd_par *priv = dev_get_drvdata(&pdev->dev);
    struct resource *res;

    if (priv->info)
    {
        unregister_framebuffer(priv->info);
        framebuffer_release(priv->info);
    }

    if (priv->lcdc_clk)
    {
        clk_disable(priv->lcdc_clk);
    }

    if (priv->lcdc_clk)
    {
        clk_put(priv->lcdc_clk);
    }

    if (priv->mmio)
    {
        iounmap(priv->mmio);
    }

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    release_mem_region(res->start, resource_size(res));

    platform_set_drvdata(pdev, NULL);

    pm_runtime_disable(&pdev->dev);

    if (priv)
    {
        kfree(priv);
    }

    return 0;
}

static int lidd_video_alloc_one(struct lidd_par *item, size_t vram_size, struct lidd_dma_par *dma_par)
{
    int ret = 0;
    struct platform_device *pdev = item->pdev;

    dma_par->vram_virt = dma_alloc_coherent(&pdev->dev, vram_size, (dma_addr_t *)&dma_par->vram_phys, GFP_KERNEL | GFP_DMA);
    if (!dma_par->vram_virt)
    {
        dma_par->vram_size = 0;
        dev_err(&pdev->dev, "%s: unable to vmalloc\n", __func__);
        ret = -ENOMEM;
    }
    else
    {
        dma_par->vram_size = vram_size;
        memset(dma_par->vram_virt, 0xff, vram_size);
        printk("%s: DMA set from 0x%x->0x%x to 0x%lx, %ld bytes\n", __func__, dma_par->vram_phys, (uint32_t)dma_par->vram_virt, dma_par->vram_phys + dma_par->vram_size - 1, dma_par->vram_size);
    }
    return ret;
}

static int lidd_video_alloc(struct lidd_par *item)
{
    unsigned long ulcm;
    size_t vram_size;
    int ret;

    /* Allocate frame buffer */
    vram_size = item->info->var.xres * item->info->var.yres * LCD_BITS_PER_PIXEL;
    ulcm = lcm((item->info->var.xres * LCD_BITS_PER_PIXEL) / 8, PAGE_SIZE);
    vram_size = roundup(vram_size / 8, ulcm);

    ret = lidd_video_alloc_one(item, vram_size << 1, &item->dma_par[LIDD_FB_INTERNAL_INDEX]);
    if (ret == 0)
    {
        ret = lidd_video_alloc_one(item, vram_size, &item->dma_par[LIDD_FB_EXTERNAL_INDEX]);
        if (ret == 0)
        {
            item->info->fix.smem_start = (unsigned long)(item->dma_par[LIDD_FB_EXTERNAL_INDEX].vram_phys);
            item->info->fix.smem_len = item->dma_par[LIDD_FB_EXTERNAL_INDEX].vram_size;
            item->info->screen_base = (char __iomem *)item->dma_par[LIDD_FB_EXTERNAL_INDEX].vram_virt;
            item->info->screen_size = (unsigned int)item->dma_par[LIDD_FB_EXTERNAL_INDEX].vram_size;
        }
    }
    return ret;
}

/* Configure the Burst Size and fifo threhold of DMA */
static int lcd_cfg_dma(struct lidd_par *item, int burst_size, int fifo_th)
{
    u32 reg = 0;

    switch (burst_size)
    {
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
    reg |= LCDC_DMA_FIFO_THRESHOLD(fifo_th);

    reg_write(item, LCDC_DMA_CTRL_REG, reg);

    return 0;
}

static irqreturn_t lidd_dma_irq_handler(int irq, void *arg)
{
    struct lidd_par *item = (struct lidd_par *)arg;
    u32 stat;

    /* Clear interrupt status */
    stat = reg_read(item, LCDC_MASKED_STAT_REG);
    reg_write(item, LCDC_MASKED_STAT_REG, stat);

    if (stat & (LCDC_SYNC_LOST | LCDC_FIFO_UNDERFLOW))
    {
        printk(KERN_ERR "LCDC sync lost or underflow error occured\nNot sure what to do...\n");
    }
    if (stat & LCDC_V2_DONE_INT_ENA)
    {
        lidd_dma_enable(item, false);
        schedule_work(&item-> dma_work);
    }
    return IRQ_HANDLED;
}

static void lidd_dma_enable(struct lidd_par *item, bool doenable)
{
    reg_write(item, LCD_LIDD_CTRL, (reg_read(item, LCD_LIDD_CTRL) & ~(LIDD_DMA_CS0_CS1 | LIDD_DMA_ENABLE)) | ((doenable & 1) << LIDD_DMA_ENABLE_SHIFT));
}

static int st7789v_suspend(struct platform_device *dev, pm_message_t state)
{
    printk("st7789v_suspend\n");
    return 0;
}

static int st7789v_resume(struct platform_device *dev)
{
    printk("st7789v_resume\n");
    return 0;
}

static const struct dev_pm_ops tilidd_pm_ops =
    {
        SET_SYSTEM_SLEEP_PM_OPS(tilidd_suspend, tilidd_resume)};

static struct of_device_id cglidd_of_match[] = {
    {
        .compatible = "cg,am33xx-lidd",
    },
    {},
};
MODULE_DEVICE_TABLE(of, cglidd_of_match);

static struct platform_driver tilidd_platform_driver =
    {
        .probe = tilidd_pdev_probe,
        .remove = tilidd_pdev_remove,
        .resume = st7789v_resume,
        .suspend = st7789v_suspend,
        .driver =
            {
                .name = DRIVER_NAME,
                .pm = &tilidd_pm_ops,
                .of_match_table = cglidd_of_match,
            },
};

static int __init tilidd_fb_init(void)
{
    printk("Init tilidd\n");
    return platform_driver_register(&tilidd_platform_driver);
}

static void __exit tilidd_fb_fini(void)
{
    printk("Exit tilidd\n");
    platform_driver_unregister(&tilidd_platform_driver);
}

static int tilidd_suspend(struct device *dev)
{
    printk("tilidd_suspend\n");
    //  int i = 5000;
    //  u32 stat;
    //  struct ssd1289* item = (struct ssd1289*)dev->dev.platform_data;
    //
    //  console_lock();
    //  fb_set_suspend(item->info, 1);
    //  item->suspending = 1;
    //  do {
    //      mdelay(1);
    //  } while (item->suspending && (i--));
    //
    //  if (item->suspending) {
    //      dev_err(&dev->dev,"Failed to suspend %s driver\n",DRIVER_NAME);
    //      return 1;
    //  }
    //  stat = lcdc_read(item,LCD_STAT_REG);
    //  lcdc_write(item,stat,LCD_MASKED_STAT_REG);
    //
    //  // PT=0 VLE=1 SPT=0 GON=1 DTE=1 CM=0 D=0 (Turn off the display)
    //  panel_reg_set(item, SSD1289_REG_DISPLAY_CTRL, 0x0230);
    //
    //  lcd_context_save(item);
    pm_runtime_put(dev);
    //  console_unlock();

    return 0;
}

static int tilidd_resume(struct device *dev)
{
    printk("tilidd_resume\n");
    //  struct llid_par* item = (struct llid_par9*)dev->dev.platform_data;

    //  console_lock();
    pm_runtime_get_sync(dev);
    //  msleep(1);

    //  lcd_context_restore(item);

    // Do SSD1289 setup
    //  ssd1289_setup(item);

    // Set up LCD coordinates as necessary
    //  panel_reg_set(item, SSD1289_REG_GDDRAM_X_ADDR, 0);
    //  panel_reg_set(item, SSD1289_REG_GDDRAM_Y_ADDR, 0);
    //  lcdc_write(item,SSD1289_REG_GDDRAM_DATA, LCD_LIDD_CS0_ADDR);    //set up for data before DMA begins
    //  fb_set_suspend(item->info, 0);
    //  ssd1289_dma_setstatus(item, 1); //enable DMA
    //  console_unlock();
    //
    //  dev_dbg(&dev->dev,"Resumed.");
    return 0;
}

// static void lcd_context_save(struct lidd_par* item)
//{
//     printk("lcd_context_save\n");
//   reg_context.clk_enable = lcdc_read(item,LCD_CLK_ENABLE_REG);
//   reg_context.ctrl = lcdc_read(item,LCD_CTRL);
//   reg_context.dma_ctrl = lcdc_read(item,LCD_DMA_CTRL_REG);
//   reg_context.int_enable_set = lcdc_read(item,LCD_INT_ENABLE_SET_REG);
//   reg_context.dma_frm_buf_base_addr_0 =
//       lcdc_read(item,LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
//   reg_context.dma_frm_buf_ceiling_addr_0 =
//       lcdc_read(item,LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
//   reg_context.dma_frm_buf_base_addr_1 =
//       lcdc_read(item,LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
//   reg_context.dma_frm_buf_ceiling_addr_1 =
//       lcdc_read(item,LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
//     return;
// }

// static void lcd_context_restore(struct lidd_par* item)
//{
//     printk("lcd_context_restore\n");
//   lcdc_write(item,reg_context.clk_enable, LCD_CLK_ENABLE_REG);
//   lcdc_write(item,reg_context.ctrl, LCD_CTRL);
//   lcdc_write(item,reg_context.dma_ctrl, LCD_DMA_CTRL_REG);
//   lcdc_write(item,reg_context.int_enable_set, LCD_INT_ENABLE_SET_REG);
//   lcdc_write(item,reg_context.dma_frm_buf_base_addr_0,
//           LCD_DMA_FRM_BUF_BASE_ADDR_0_REG);
//   lcdc_write(item,reg_context.dma_frm_buf_ceiling_addr_0,
//           LCD_DMA_FRM_BUF_CEILING_ADDR_0_REG);
//   lcdc_write(item,reg_context.dma_frm_buf_base_addr_1,
//           LCD_DMA_FRM_BUF_BASE_ADDR_1_REG);
//   lcdc_write(item,reg_context.dma_frm_buf_ceiling_addr_1,
//           LCD_DMA_FRM_BUF_CEILING_ADDR_1_REG);
//     return;
// }

static void panel_regs_set(struct lidd_par *item, u8 reg, u16 *values, int numValues)
{
    int i;
    reg_write(item, LCD_LIDD_CS0_ADDR, reg);
    for (i = 0; i < numValues; i++)
    {
        reg_write(item, LCD_LIDD_CS0_DATA, values[i]);
    }
}

static void panel_reg_set(struct lidd_par *item, u8 reg, u16 value)
{
    panel_regs_set(item, reg, &value, 1);
}

static void st7789v_setup(struct lidd_par *item)
{
    u16 porctrl[] = {0x0c, 0x0c, 0x00, 0x33, 0x33};
    u16 pwctrl1[] = {0xA4, 0xA1};
    u16 pvgamctrl[] = {0xf0, 0x08, 0x0E, 0x09, 0x08, 0x04, 0x2F, 0x33, 0x45, 0x36, 0x13, 0x12, 0x2A, 0x2D};
    u16 nvgamctrl[] = {0xf0, 0x0E, 0x12, 0x0C, 0x0A, 0x15, 0x2E, 0x32, 0x44, 0x39, 0x17, 0x18, 0x2B, 0x2F};

    reg_write(item, LCD_LIDD_CS0_ADDR, ST7789V_SWRESET);
    msleep(120);

    reg_write(item, LCD_LIDD_CS0_ADDR, ST7789V_SLPOUT);
    msleep(120);

    panel_reg_set(item, ST7789V_MADCTL, 0x00);
    panel_reg_set(item, ST7789V_COLMOD, 0x55);
    panel_regs_set(item, ST7789V_PORCTRL, porctrl, ARRAY_SIZE(porctrl));
    panel_reg_set(item, ST7789V_GCTRL, 0x00);
    panel_reg_set(item, ST7789V_VCOMS, 0x36);

    panel_reg_set(item, ST7789V_VDVVRHEN, 0x01);
    panel_reg_set(item, ST7789V_VRHS, 0x13);
    panel_reg_set(item, ST7789V_VDVS, 0x20);

    panel_reg_set(item, ST7789V_FRCTRL2, 0x0F);
    panel_regs_set(item, ST7789V_PWCTRL1, pwctrl1, ARRAY_SIZE(pwctrl1));

    panel_regs_set(item, ST7789V_PVGAMCTRL, pvgamctrl, ARRAY_SIZE(pvgamctrl));
    panel_regs_set(item, ST7789V_NVGAMCTRL, nvgamctrl, ARRAY_SIZE(nvgamctrl));

    reg_write(item, LCD_LIDD_CS0_ADDR, ST7789V_INVON);
    st7789v_SetFrameDimensions(item, 0, 0);
}

static int fb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info)
{
    struct lidd_par *par = info->par;
    int ret = 0;
    if ((regno >= 16) || (info->fix.visual == FB_VISUAL_DIRECTCOLOR))
    {
        ret = 1;
    }
    else if ((info->var.bits_per_pixel == LCD_BITS_PER_PIXEL) && regno < 16)
    {
        red >>= (LCD_BITS_PER_PIXEL - info->var.red.length);
        red <<= info->var.red.offset;

        green >>= (LCD_BITS_PER_PIXEL - info->var.green.length);
        green <<= info->var.green.offset;

        blue >>= (LCD_BITS_PER_PIXEL - info->var.blue.length);
        blue <<= info->var.blue.offset;

        par->pseudo_palette[regno] = red | green | blue;
    }
    return ret;
}

static int fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    // struct lcd_sync_arg sync_arg;

    switch (cmd)
    {
    case FBIOPUT_VSCREENINFO:
        return 0;

    default:
        return -EINVAL;
    }
}

static void fb_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
    printk("fb_fillrect - NOT IMPLEMENTED\n");
}

static void fb_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
    printk("fb_copyarea - NOT IMPLEMENTED\n");
}

static ssize_t fb_read(struct fb_info *info, char __user *buf, size_t count, loff_t *ppos)
{
    printk("fb_read\n");
    return 0;
}

static ssize_t fb_write(struct fb_info *info, const char __user *buf, size_t count, loff_t *ppos)
{
    printk("fb_write\n");
    return 0;
}

static void st7789v_SetFrameDimensions(struct lidd_par *item, uint16_t Xpos, uint16_t Ypos)
{
    uint16_t params[4];
    uint16_t end = (Xpos + LCD_SCREEN_WIDTH - 1);
    params[0] = SET_VALHI(Xpos);
    params[1] = SET_VALLO(Xpos);
    params[2] = SET_VALHI(end);
    params[3] = SET_VALLO(end);
    panel_regs_set(item, ST7789V_CASET, params, ARRAY_SIZE(params));

    end = (Ypos + LCD_SCREEN_HEIGHT - 1);
    params[0] = SET_VALHI(Ypos);
    params[1] = SET_VALLO(Ypos);
    params[2] = SET_VALHI(end);
    params[3] = SET_VALLO(end);
    panel_regs_set(item, ST7789V_RASET, params, ARRAY_SIZE(params));
}

module_init(tilidd_fb_init);
module_exit(tilidd_fb_fini);

MODULE_AUTHOR("Chris Desjardins");
MODULE_DESCRIPTION("TI LLID LCD Module");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
