/*
 * llid_fb_regs.h
 *
 *  Created on: May 13, 2016
 *      Author: gapa
 */

#ifndef LLID_FB_REGS_H_
#define LLID_FB_REGS_H_

/* LCDC Status Register */
#define LCDC_END_OF_FRAME1                       BIT(9)
#define LCDC_END_OF_FRAME0                       BIT(8)
#define LCDC_PL_LOAD_DONE                        BIT(6)
#define LCDC_FIFO_UNDERFLOW                      BIT(5)
#define LCDC_SYNC_LOST                           BIT(2)
#define LCDC_FRAME_DONE                          BIT(0)

/* LCDC DMA Control Register */
#define LCDC_DMA_BURST_SIZE(x)                   ((x) << 4)
#define LCDC_DMA_BURST_1                         0x0
#define LCDC_DMA_BURST_2                         0x1
#define LCDC_DMA_BURST_4                         0x2
#define LCDC_DMA_BURST_8                         0x3
#define LCDC_DMA_BURST_16                        0x4
#define LCDC_V1_END_OF_FRAME_INT_ENA             BIT(2)
#define LCDC_V2_END_OF_FRAME0_INT_ENA            BIT(8)
#define LCDC_V2_END_OF_FRAME1_INT_ENA            BIT(9)
#define LCDC_DUAL_FRAME_BUFFER_ENABLE            BIT(0)

/* LCDC Control Register */
#define LCDC_CLK_DIVISOR(x)                      ((x) << 8)
#define LCDC_RASTER_MODE                         0x01

/* LCDC Raster Control Register */
#define LCDC_PALETTE_LOAD_MODE(x)                ((x) << 20)
#define PALETTE_AND_DATA                         0x00
#define PALETTE_ONLY                             0x01
#define DATA_ONLY                                0x02

#define LCDC_MONO_8BIT_MODE                      BIT(9)
#define LCDC_RASTER_ORDER                        BIT(8)
#define LCDC_TFT_MODE                            BIT(7)
#define LCDC_V1_UNDERFLOW_INT_ENA                BIT(6)
#define LCDC_V2_UNDERFLOW_INT_ENA                BIT(5)
#define LCDC_V1_PL_INT_ENA                       BIT(4)
#define LCDC_V2_PL_INT_ENA                       BIT(6)
#define LCDC_MONOCHROME_MODE                     BIT(1)
#define LCDC_RASTER_ENABLE                       BIT(0)
#define LCDC_TFT_ALT_ENABLE                      BIT(23)
#define LCDC_STN_565_ENABLE                      BIT(24)
#define LCDC_V2_DMA_CLK_EN                       BIT(2)
#define LCDC_V2_LIDD_CLK_EN                      BIT(1)
#define LCDC_V2_CORE_CLK_EN                      BIT(0)
#define LCDC_V2_LPP_B10                          26
#define LCDC_V2_TFT_24BPP_MODE                   BIT(25)
#define LCDC_V2_TFT_24BPP_UNPACK                 BIT(26)
#define LCD_V2_END_OF_FRAME0_INT_ENA	BIT(8)
#define LCD_V2_END_OF_FRAME1_INT_ENA	BIT(9)
#define LCD_V2_DONE_INT_ENA					BIT(0)

/* LCDC Raster Timing 2 Register */
#define LCDC_AC_BIAS_TRANSITIONS_PER_INT(x)      ((x) << 16)
#define LCDC_AC_BIAS_FREQUENCY(x)                ((x) << 8)
#define LCDC_SYNC_CTRL                           BIT(25)
#define LCDC_SYNC_EDGE                           BIT(24)
#define LCDC_INVERT_PIXEL_CLOCK                  BIT(22)
#define LCDC_INVERT_HSYNC                        BIT(21)
#define LCDC_INVERT_VSYNC                        BIT(20)
#define LCDC_LPP_B10                             BIT(26)

/* LCD_CS_CONF position helpers */
#define CONF_TA_POS 0
#define R_HOLD_POS 2
#define R_STROBE_POS 6
#define R_SU_POS 12
#define W_HOLD_POS 17
#define W_STROBE_POS 21
#define W_SU_POS 27

/* LCDC Block */
#define LCDC_PID_REG                             0x0
#define LCDC_CTRL_REG                            0x4
#define LCDC_STAT_REG                            0x8
#define LCD_LIDD_CTRL							 0x0C
#define LCD_CS0_CONF							 0x10
#define LCD_LIDD_CS0_ADDR						 0x14
#define LCD_LIDD_CS0_DATA						 0x18
#define LCDC_RASTER_CTRL_REG                     0x28
#define LCDC_RASTER_TIMING_0_REG                 0x2c
#define LCDC_RASTER_TIMING_1_REG                 0x30
#define LCDC_RASTER_TIMING_2_REG                 0x34
#define LCDC_DMA_CTRL_REG                        0x40
#define LCDC_DMA_FB_BASE_ADDR_0_REG              0x44
#define LCDC_DMA_FB_CEILING_ADDR_0_REG           0x48
#define LCDC_DMA_FB_BASE_ADDR_1_REG              0x4c
#define LCDC_DMA_FB_CEILING_ADDR_1_REG           0x50

/* Interrupt Registers available only in Version 2 */
#define LCDC_RAW_STAT_REG                        0x58
#define LCDC_MASKED_STAT_REG                     0x5c
#define LCDC_INT_ENABLE_SET_REG                  0x60
#define LCDC_INT_ENABLE_CLR_REG                  0x64
#define LCDC_END_OF_INT_IND_REG                  0x68

/* Clock registers available only on Version 2 */
#define LCDC_CLK_ENABLE_REG                      0x6c
#define LCDC_CLK_RESET_REG                       0x70
#define LCDC_CLK_MAIN_RESET                      BIT(3)

#define LCD_LIDD_TYPE_8080						BIT(0) | BIT(1)

#define SSD1289_REG_OSCILLATION      0x00
#define SSD1289_REG_DEV_CODE_READ    0x00
#define SSD1289_REG_DRIVER_OUT_CTRL  0x01
#define SSD1289_REG_LCD_DRIVE_AC     0x02
#define SSD1289_REG_POWER_CTRL_1     0x03
#define SSD1289_REG_DISPLAY_CTRL     0x07
#define SSD1289_REG_FRAME_CYCLE      0x0b
#define SSD1289_REG_POWER_CTRL_2     0x0c
#define SSD1289_REG_POWER_CTRL_3     0x0d
#define SSD1289_REG_POWER_CTRL_4     0x0e
#define SSD1289_REG_GATE_SCAN_START  0x0f
#define SSD1289_REG_SLEEP_MODE       0x10
#define SSD1289_REG_ENTRY_MODE       0x11
#define SSD1289_REG_POWER_CTRL_5     0x1e
#define SSD1289_REG_GDDRAM_DATA      0x22
#define SSD1289_REG_WR_DATA_MASK_1   0x23
#define SSD1289_REG_WR_DATA_MASK_2   0x24
#define SSD1289_REG_FRAME_FREQUENCY  0x25
#define SSD1289_REG_GAMMA_CTRL_1     0x30
#define SSD1289_REG_GAMME_CTRL_2     0x31
#define SSD1289_REG_GAMMA_CTRL_3     0x32
#define SSD1289_REG_GAMMA_CTRL_4     0x33
#define SSD1289_REG_GAMMA_CTRL_5     0x34
#define SSD1289_REG_GAMMA_CTRL_6     0x35
#define SSD1289_REG_GAMMA_CTRL_7     0x36
#define SSD1289_REG_GAMMA_CTRL_8     0x37
#define SSD1289_REG_GAMMA_CTRL_9     0x3a
#define SSD1289_REG_GAMMA_CTRL_10    0x3b
#define SSD1289_REG_V_SCROLL_CTRL_1  0x41
#define SSD1289_REG_V_SCROLL_CTRL_2  0x42
#define SSD1289_REG_H_RAM_ADR_POS    0x44
#define SSD1289_REG_V_RAM_ADR_START  0x45
#define SSD1289_REG_V_RAM_ADR_END    0x46
#define SSD1289_REG_FIRST_WIN_START  0x48
#define SSD1289_REG_FIRST_WIN_END    0x49
#define SSD1289_REG_SECND_WIN_START  0x4a
#define SSD1289_REG_SECND_WIN_END    0x4b
#define SSD1289_REG_GDDRAM_X_ADDR    0x4e
#define SSD1289_REG_GDDRAM_Y_ADDR    0x4f

/*
 * Helpers:
 */

static inline void reg_write(struct llid_par *par, u32 reg, u32 data)
{
	iowrite32(data, par->mmio + reg);
}

static inline u32 reg_read(struct llid_par *par, u32 reg)
{
	return ioread32(par->mmio + reg);
}

static inline void reg_set(struct llid_par *par, u32 reg, u32 mask)
{
	reg_write(par, reg, reg_read(par, reg) | mask);
}

static inline void reg_clear(struct llid_par *par, u32 reg, u32 mask)
{
	reg_write(par, reg, reg_read(par, reg) & ~mask);
}

/* the register to read/clear irqstatus differs between v1 and v2 of the IP */
static inline u32 llid_irqstatus_reg(struct llid_par *par)
{
	return (par->rev == 2) ? LCDC_MASKED_STAT_REG : LCDC_STAT_REG;
}

static inline u32 llid_read_irqstatus(struct llid_par *par)
{
	return reg_read(par, llid_irqstatus_reg(par));
}

static inline void llid_clear_irqstatus(struct llid_par *par, u32 mask)
{
	reg_write(par, llid_irqstatus_reg(par), mask);
}

static inline void ssd1289_reg_set(struct llid_par *item, unsigned char reg,
				   unsigned short value)
{
	reg_write(item, LCD_LIDD_CS0_ADDR, 0x000000FF&(unsigned int)reg);
	reg_write(item, LCD_LIDD_CS0_DATA, (unsigned int)value);
}

static inline unsigned int ssd1289_reg_get(struct llid_par *item, unsigned char reg)
{
	reg_write(item, LCD_LIDD_CS0_ADDR, 0x000000FF&(unsigned int)reg);
	return reg_read(item,LCD_LIDD_CS0_DATA);
}

static inline void LCD_WriteRAM_Prepare(struct llid_par *item)
{
	reg_write(item, LCD_LIDD_CS0_ADDR, (unsigned int)SSD1289_REG_GDDRAM_DATA);
}

#endif /* LLID_FB_REGS_H_ */
