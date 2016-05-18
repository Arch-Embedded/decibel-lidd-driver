#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/fb.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <generated/autoconf.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/pm_runtime.h>
//#include <errno.h>

#include "llid_fb.h"
#include "llid_fb_regs.h"

static struct llid_par *display_params;

static int tillid_pdev_probe(struct platform_device *pdev) {

	struct resource *res;
	struct llid_par *priv;
	int ret;

	/* bail out early if no DT data: */
	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "device-tree data is missing\n");
		return -ENXIO;
	}

	/* Select default pin state */
	pr_debug("Initializing pins");
	pinctrl_pm_select_default_state(&(pdev->dev));

	pr_debug("End of Initializing pins");

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		kfree(priv);
		dev_err(&pdev->dev, "failed to allocate private data\n");
		return -ENOMEM;
	}

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

	priv->lcdc_clk = clk_get(&pdev->dev, "fck");
	if (IS_ERR(priv->lcdc_clk)) {
		dev_err(&pdev->dev, "failed to get functional clock\n");
		ret = -ENODEV;
		goto fail_iounmap;
	}

	pr_debug("Functional clk finished");

	pm_runtime_enable(&pdev->dev);
	pm_runtime_irq_safe(&pdev->dev);

	/* Determine LCD IP Version */
	pm_runtime_get_sync(&pdev->dev);

	dev_warn(&pdev->dev, "PID Reg value 0x%08x, ",
			reg_read(priv, LCDC_PID_REG));

//		switch (reg_read(priv, LCDC_PID_REG)) {
//		case 0x4c100102:
//			priv->rev = 1;
//			break;
//		case 0x4f200800:
//		case 0x4f201000:
//			priv->rev = 2;
//			break;
//		default:
//			dev_warn(&pdev->dev, "Unknown PID Reg value 0x%08x, "
//					"defaulting to LCD revision 1\n",
//					reg_read(priv, LCDC_PID_REG));
//			priv->rev = 1;
//			break;
//		}

	return 0;

fail_iounmap:
	iounmap(priv->mmio);
	return ret;

}

static int tillid_pdev_remove(struct platform_device *pdev) {

	struct llid_par *priv = display_params;

	if (priv->lcdc_clk)
		clk_put(priv->lcdc_clk);

	if (priv->mmio)
		iounmap(priv->mmio);

	kfree(priv);

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

module_init(tillid_fb_init);
module_exit(tillid_fb_fini);

MODULE_AUTHOR("Cezary Gapinski");
MODULE_DESCRIPTION("TI LLID LCD Module");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");


