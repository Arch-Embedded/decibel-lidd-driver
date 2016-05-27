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
#include <generated/autoconf.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/pm_runtime.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <linux/gpio.h>                 // Required for the GPIO functions
#include <linux/interrupt.h>            // Required for the IRQ code
//#include <errno.h>

#include "llid_fb.h"
#include "llid_fb_regs.h"

static struct llid_par *display_params = NULL;

//static struct fb_fix_screeninfo ssd1289_fix __initdata = {
//	.id          = DRIVER_NAME,
//	.type        = FB_TYPE_PACKED_PIXELS,
//	.visual      = FB_VISUAL_TRUECOLOR,
//	.accel       = FB_ACCEL_NONE,
//	.line_length = 240 * 2,
//};

//static struct fb_var_screeninfo ssd1289_var __initdata = {
//	.xres		= 240,
//	.yres		= 320,
//	.xres_virtual	= 240,
//	.yres_virtual	= 320,
//	.bits_per_pixel	= 16,
//	.red		= {6, 5, 0},
//	.green		= {11, 5, 0},
//	.blue		= {0, 6, 0},
//	.activate	= FB_ACTIVATE_FORCE,	//FB_ACTIVATE_NOW,
//	.height		= 320,
//	.width		= 240,
//	.vmode		= FB_VMODE_NONINTERLACED,
//};

/// Function prototype for the custom IRQ handler function -- see below for the implementation
static irq_handler_t  ebbgpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

static void __init ssd1289_setup(struct llid_par *item)
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

static int tillid_pdev_probe(struct platform_device *pdev) {

	struct resource *res;
	struct device_node *node = pdev->dev.of_node;
	struct llid_par *priv;
	struct pinctrl *pinctrl;
	int ret = 0;
	unsigned int cfg;
	unsigned int signature;

	/* bail out early if no DT data: */
	if (!node) {
		dev_err(&pdev->dev, "device-tree data is missing\n");
		return -ENXIO;
	}

	/* Select default pin state */
	pr_debug("Initializing pins");

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev, "pins are not configured\n");

	pinctrl_pm_select_default_state(&(pdev->dev));

	pr_debug("End of Initializing pins");

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		kfree(priv);
		dev_err(&pdev->dev, "failed to allocate private data\n");
		return -ENOMEM;
	}

	if(!display_params)
	display_params = priv;

	pr_debug("Kzalloc allocated\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get memory resource\n");
		return -EINVAL;
	}

	pr_debug("Platform get resource finished\n");

	priv->mmio = ioremap_nocache(res->start, resource_size(res));
	if (!priv->mmio) {
		dev_err(&pdev->dev, "failed to ioremap\n");
		return -ENOMEM;
	}

	pr_debug("Remaping nocache finished\n");

	priv->enable_gpio = devm_gpiod_get(&pdev->dev, "enable");
	if (IS_ERR(priv->enable_gpio)) {
			dev_err(&pdev->dev, "failed to request enable GPIO\n");
			goto fail_iounmap;
	} else {
		ret = gpiod_direction_output(priv->enable_gpio, 0);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to setup GPIO\n");
			goto fail_iounmap;
		}
		dev_info(&pdev->dev, "found enable GPIO\n");
		gpiod_set_value_cansleep(priv->enable_gpio, 1);
	}

	pr_debug("GPIO enable pin set to 1 finished\n");

	priv->input_test_gpio = devm_gpiod_get(&pdev->dev, "input");
	if (IS_ERR(priv->input_test_gpio)) {
			dev_err(&pdev->dev, "failed to request input test GPIO\n");
			goto fail_iounmap;
	} else {
		ret = gpiod_direction_input(priv->input_test_gpio);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to setup input test GPIO\n");
			goto fail_iounmap;
		}
		dev_info(&pdev->dev, "found input test GPIO\n");
	}

	pr_debug("GPIO input test pin configuration finished\n");

	// GPIO numbers and IRQ numbers are not the same! This function performs the mapping for us
	priv->irqNumber = gpiod_to_irq(priv->input_test_gpio);
	printk(KERN_INFO "GPIO_TEST: The button is mapped to IRQ: %d\n", priv->irqNumber);

	// This next call requests an interrupt line
	ret = request_irq(priv->irqNumber,             // The interrupt number requested
	                  (irq_handler_t) ebbgpio_irq_handler, // The pointer to the handler function below
	                  IRQF_TRIGGER_FALLING,   // Interrupt on rising edge (button press, not release)
	                  "ebb_gpio_handler",    // Used in /proc/interrupts to identify the owner
	                  NULL);                 // The *dev_id for shared interrupt lines, NULL is okay

	if(ret) {
		printk(KERN_INFO "GPIO_TEST: The interrupt request result is: %d\n", ret);
		goto fail_iounmap;
	}

	priv->lcdc_clk = clk_get(&pdev->dev, "fck");
	if (IS_ERR(priv->lcdc_clk)) {
		dev_err(&pdev->dev, "failed to get functional clock\n");
		ret = -ENODEV;
		goto fail_iounmap;
	}

	pr_debug("Functional clk finished");

	pm_runtime_enable(&pdev->dev);
//	pm_runtime_irq_safe(&pdev->dev);

	/* Determine LCD IP Version */
	pm_runtime_get_sync(&pdev->dev);

	ret = clk_enable(priv->lcdc_clk);
	if (ret) {
		dev_err(&pdev->dev, "Can not enable device clock\n");
		goto err_clk_get;
	}

	dev_dbg(&pdev->dev, "PID Reg value 0x%08x, ",
			reg_read(priv, LCDC_PID_REG));

	//pm_runtime_put_sync(&pdev->dev);

	/* set_rate wants parameter in Hz */
	ret = clk_set_rate(priv->lcdc_clk, 100000000);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to set display clock rate to: %d\n", ret);
		goto out_clk_enable;
	}

	//Setting up AM335X LCDC Controller
//	cfg = LCDC_V2_DMA_CLK_EN | LCDC_V2_LIDD_CLK_EN | LCDC_V2_CORE_CLK_EN; // Turn on LIDD clock and DMA clock, core clock doesn't help DMA :( ?
	cfg = LCDC_V2_LIDD_CLK_EN;
	reg_write(priv, LCDC_CLK_ENABLE_REG, cfg);
	cfg = 0 | LCDC_CLK_DIVISOR(6);						// LCDC Mode = LIDD
	reg_write(priv, LCDC_CTRL_REG, cfg);
	reg_write(priv, LCD_LIDD_CTRL, LCD_LIDD_TYPE_8080); // 8080 display, DMA (NOT YET) enabled

#define CONF_TA_POS 0
#define R_HOLD_POS 2
#define R_STROBE_POS 6
#define R_SU_POS 12
#define W_HOLD_POS 17
#define W_STROBE_POS 21
#define W_SU_POS 27

	cfg = (3 << CONF_TA_POS | 15 << R_HOLD_POS | 15 << R_STROBE_POS | 2 << R_SU_POS
		| 2 << W_HOLD_POS | 2 << W_STROBE_POS | 1 << W_SU_POS);
	reg_write(priv, LCD_CS0_CONF, cfg);
	pr_debug("Initialized LCDC controller");

	dev_dbg(&pdev->dev, "SSD1289 Reg LCDC_CLK_ENABLE_REG value 0x%08x ",
			reg_read(priv,LCDC_CLK_ENABLE_REG));
	dev_dbg(&pdev->dev, "SSD1289 Reg LCDC_CTRL_REG value 0x%08x ",
			reg_read(priv,LCDC_CTRL_REG));
	dev_dbg(&pdev->dev, "SSD1289 LCD_LIDD_CTRL value 0x%08x ",
			reg_read(priv,LCD_LIDD_CTRL));
	dev_dbg(&pdev->dev, "LCD_CS0_CONF value 0x%08x ",
			reg_read(priv,LCD_CS0_CONF));

	msleep(50);

	//SSD1289 LCD Driver Check
	ssd1289_reg_set(priv, SSD1289_REG_OSCILLATION, 0x0001);
	signature = ssd1289_reg_get(priv,SSD1289_REG_DEV_CODE_READ);
	dev_dbg(&pdev->dev, "SSD1289 Reg value 0x%08x ",
			signature);

	ssd1289_setup(priv);

//	pm_runtime_put_sync(&pdev->dev);

	return 0;

out_clk_enable:
	clk_disable(priv->lcdc_clk);
err_clk_get:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	clk_put(priv->lcdc_clk);
fail_iounmap:
	iounmap(priv->mmio);

	return ret;
}

static int tillid_pdev_remove(struct platform_device *pdev) {

	struct llid_par *priv = display_params;

	free_irq(priv->irqNumber, NULL);               // Free the IRQ number, no *dev_id required in this case


	if(priv->lcdc_clk)
		clk_disable(priv->lcdc_clk);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	if (priv->lcdc_clk)
		clk_put(priv->lcdc_clk);

	if (priv->mmio)
		iounmap(priv->mmio);

	if(display_params)
		kfree(display_params);

	return 0;
}

static struct of_device_id tillid_of_match[] = { { .compatible =
		"ti,am33xx-llid,llid-lcd", }, { }, };
MODULE_DEVICE_TABLE(of, tillid_of_match);

static struct platform_driver tillid_platform_driver =
{ .probe = tillid_pdev_probe,
  .remove = tillid_pdev_remove,
  .driver = {
		  .name ="llid-lcd",
		  .of_match_table = tillid_of_match, },
};

static int __init tillid_fb_init(void) {
	pr_debug("Init tillid\n");
	return platform_driver_register(&tillid_platform_driver);
}

static void __exit tillid_fb_fini(void) {
	pr_debug("Exit tillid\n");
	platform_driver_unregister(&tillid_platform_driver);
}

static irq_handler_t ebbgpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
	pr_debug("We are in interrupt\n");
   return (irq_handler_t) IRQ_HANDLED;      // Announce that the IRQ has been handled correctly
}

module_init(tillid_fb_init);
module_exit(tillid_fb_fini);

MODULE_AUTHOR("Cezary Gapinski");
MODULE_DESCRIPTION("TI LLID LCD Module");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");


