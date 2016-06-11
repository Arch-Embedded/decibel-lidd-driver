/*
 * ldid_fb.h
 *
 *  Created on: May 8, 2016
 *      Author: Cezary Gapinski <cezary.gapinski@gmail.com>
 */

#ifndef LIDD_FB_H_
#define LIDD_FB_H_

/* LCD color */
#define White          0xFFFF
#define Black          0x0000
#define Grey           0xF7DE
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0

struct lidd_par;

struct llid_framebuffer_ops {
	int (*write)(struct lidd_par *par, void *buf, size_t len);
	int (*read)(struct lidd_par *par, void *buf, size_t len);
	int (*write_vmem)(struct lidd_par *par, size_t offset, size_t len);
	void (*write_register)(struct lidd_par *par, int len, ...);

	void (*set_addr_win)(struct lidd_par *par,
		int xs, int ys, int xe, int ye);
	void (*reset)(struct lidd_par *par);
	void (*mkdirty)(struct fb_info *info, int from, int to);
	void (*update_display)(struct lidd_par *par,
				unsigned start_line, unsigned end_line);
	int (*init_display)(struct lidd_par *par);
	int (*blank)(struct lidd_par *par, bool on);

	void (*register_backlight)(struct lidd_par *par);
	void (*unregister_backlight)(struct lidd_par *par);

	int (*set_var)(struct lidd_par *par);
	int (*set_gamma)(struct lidd_par *par, unsigned long *curves);
};

/**
 * struct llid_display - Describes the display properties
 * @width: Width of display in pixels
 * @height: Height of display in pixels
 * @regwidth: LCD Controller Register width in bits
 * @buswidth: Display interface bus width in bits
 * @backlight: Backlight type.
 * @ops: llid operations provided by driver or device (platform_data)
 * @bpp: Bits per pixel
 * @fps: Frames per second
 * @txbuflen: Size of transmit buffer
 * @init_sequence: Pointer to LCD initialization array
 * @gamma: String representation of Gamma curve(s)
 * @gamma_num: Number of Gamma curves
 * @gamma_len: Number of values per Gamma curve
 * @debug: Initial debug value
 *
 * This structure is not stored by llid except for init_sequence.
 */
struct llid_display {
	unsigned width;
	unsigned height;
	unsigned regwidth;
	unsigned buswidth;
	unsigned backlight;
	struct llid_framebuffer_ops ops;
	unsigned bpp;
	unsigned fps;
	int txbuflen;
	int *init_sequence;
	char *gamma;
	int gamma_num;
	int gamma_len;
	unsigned long debug;
};

/**
 * struct fbtft_platform_data - Passes display specific data to the driver
 * @display: Display properties
 * @gpios: Pointer to an array of pinname to gpio mappings
 * @rotate: Display rotation angle
 * @bgr: LCD Controller BGR bit
 * @fps: Frames per second (this will go away, use @fps in @fbtft_display)
 * @txbuflen: Size of transmit buffer
 * @startbyte: When set, enables use of Startbyte in transfers
 * @gamma: String representation of Gamma curve(s)
 * @extra: A way to pass extra info
 */
struct llid_platform_data {
	struct llid_display display;
	unsigned rotate;
	bool bgr;
	unsigned fps;
	int txbuflen;
	u8 startbyte;
	char *gamma;
	void *extra;
};



/* @pdev: Set if it is a platform device
 * @info: Pointer to framebuffer fb_info structure
 * @pdata: Pointer to platform data
 * @pseudo_palette: Used by fb_set_colreg()
 * @txbuf.buf: Transmit buffer
 * @txbuf.len: Transmit buffer length
 * @buf: Small buffer used when writing init data over SPI
 * @startbyte: Used by some controllers when in SPI mode.
 *             Format: 6 bit Device id + RS bit + RW bit
 * @llidops: llid operations provided by driver or device (platform_data)
 * @dirty_lock: Protects dirty_lines_start and dirty_lines_end
 * @dirty_lines_start: Where to begin updating display
 * @dirty_lines_end: Where to end updating display
 * @init_sequence: Pointer to LCD initialization array
 * @gamma.lock: Mutex for Gamma curve locking
 * @gamma.curves: Pointer to Gamma curve array
 * @gamma.num_values: Number of values per Gamma curve
 * @gamma.num_curves: Number of Gamma curves
 * @debug: Pointer to debug value
 * @current_debug:
 * @first_update_done: Used to only time the first display update
 * @update_time: Used to calculate 'fps' in debug output
 * @bgr: BGR mode/\n
 * @extra: Extra info needed by driver
 */
struct lidd_par {
	struct platform_device *pdev;
	struct fb_info *info;
	struct llid_platform_data *pdata;
	struct resource *reg_res;
	unsigned int irqNumber;
	u32 pseudo_palette[16];
	struct {
		void *buf;
		dma_addr_t dma;
		size_t len;
	} txbuf;
	u8 *buf;
	u8 startbyte;
	struct llid_framebuffer_ops ops;
	spinlock_t dirty_lock;
	unsigned dirty_lines_start;
	unsigned dirty_lines_end;
	int *init_sequence;
	struct {
		struct mutex lock;
		unsigned long *curves;
		int num_values;
		int num_curves;
	} gamma;
	unsigned long debug;
	bool first_update_done;
	ktime_t update_time;
	bool bgr;
	void *extra;
	int rev;                 /* IP revision */
	void __iomem *mmio;
	struct display_timings *timings;
	struct clk *lcdc_clk;			//Power enable for the LCDC
#ifdef CONFIG_CPU_FREQ
	struct notifier_block freq_transition;
	unsigned int lcd_fck_rate;
#endif
	//DMA/Memory things
	int 				irq;		//irq resource number
	dma_addr_t			vram_phys;	//should be same as dma_start
	unsigned long		vram_size;
	void				*vram_virt;
	unsigned int		dma_start;	//physical addresses
	unsigned int		dma_end;
	int 				blank;		//?
	int 				suspending;	//whether a suspend is in progress
};

#endif /* LIDD_FB_H_ */
