/*
 * lidd_fb_regs.h
 *
 *  Created on: May 13, 2016
 *      Author: Cezary Gapinski <cezary.gapinski@gmail.com>
 */

#ifndef LIDD_FB_REGS_H_
#define LIDD_FB_REGS_H_

/* LCDC DMA Control Register */
#define LCDC_DMA_BURST_SIZE(x)                   ((x & 0x07) << 4)
#define LCDC_DMA_BURST_1                         0x0
#define LCDC_DMA_BURST_2                         0x1
#define LCDC_DMA_BURST_4                         0x2
#define LCDC_DMA_BURST_8                         0x3
#define LCDC_DMA_BURST_16                        0x4
#define LCDC_DMA_FIFO_THRESHOLD(t)               ((t & 0x07) << 8)

/* LCDC Control Register */
#define LCDC_CLK_DIVISOR(x)                      ((x) << 8)
#define LCDC_RASTER_MODE                         0x01

/* LCDC Raster Control Register */
#define LCDC_PALETTE_LOAD_MODE(x)                ((x) << 20)
#define PALETTE_AND_DATA                         0x00
#define PALETTE_ONLY                             0x01
#define DATA_ONLY                                0x02
/* Cloc enable register */
#define LCDC_V2_DMA_CLK_EN                       BIT(2)
#define LCDC_V2_LIDD_CLK_EN                      BIT(1)
#define LCDC_V2_CORE_CLK_EN                      BIT(0)

/* Interrupt enable and status registers */
#define LCDC_V2_END_OF_FRAME0_INT_ENA            BIT(8)
#define LCDC_V2_END_OF_FRAME1_INT_ENA            BIT(9)
#define LCDC_FIFO_UNDERFLOW                      BIT(5)
#define LCDC_SYNC_LOST                           BIT(2)
#define LCDC_V2_DONE_INT_ENA                     BIT(0)

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
#define LCD_LIDD_CTRL                            0x0C
#define LCD_CS0_CONF                             0x10
#define LCD_LIDD_CS0_ADDR                        0x14
#define LCD_LIDD_CS0_DATA                        0x18
#define LCDC_RASTER_CTRL_REG                     0x28
#define LCDC_RASTER_TIMING_0_REG                 0x2c
#define LCDC_RASTER_TIMING_1_REG                 0x30
#define LCDC_RASTER_TIMING_2_REG                 0x34
#define LCDC_DMA_CTRL_REG                        0x40
#define LCDC_DMA_FB_BASE_ADDR_0_REG              0x44
#define LCDC_DMA_FB_CEILING_ADDR_0_REG           0x48
#define LCDC_DMA_FB_BASE_ADDR_1_REG              0x4c
#define LCDC_DMA_FB_CEILING_ADDR_1_REG           0x50
#define LCDC_SYSCONFIG                           0x54

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

#define LCD_LIDD_TYPE_8080                      BIT(0) | BIT(1)

#define ST7789V_SWRESET              0x01
#define ST7789V_RDDID                0x04
#define ST7789V_SLPOUT               0x11
#define ST7789V_INVON                0x21
#define ST7789V_DISPON               0x29
#define ST7789V_CASET                0x2A
#define ST7789V_RASET                0x2B
#define ST7789V_RAMWR                0x2C
#define ST7789V_MADCTL               0x36
#define ST7789V_COLMOD               0x3A
#define ST7789V_PORCTRL              0xB2
#define ST7789V_GCTRL                0xB7
#define ST7789V_VCOMS                0xBB
#define ST7789V_VDVVRHEN             0xC2
#define ST7789V_VRHS                 0xC3
#define ST7789V_VDVS                 0xC4
#define ST7789V_FRCTRL2              0xC6
#define ST7789V_PWCTRL1              0xD0
#define ST7789V_RDID1                0xDA
#define ST7789V_RDID2                0xDB
#define ST7789V_RDID3                0xDC
#define ST7789V_PVGAMCTRL            0xE0
#define ST7789V_NVGAMCTRL            0xE1

/*
 * Helpers:
 */

static inline void reg_write(struct lidd_par* par, u32 reg, u32 data)
{
    iowrite32(data, par->mmio + reg);
}

static inline u32 reg_read(struct lidd_par* par, u32 reg)
{
    return ioread32(par->mmio + reg);
}

static inline void reg_set(struct lidd_par* par, u32 reg, u32 mask)
{
    reg_write(par, reg, reg_read(par, reg) | mask);
}

static inline void reg_clear(struct lidd_par* par, u32 reg, u32 mask)
{
    reg_write(par, reg, reg_read(par, reg) & ~mask);
}

#endif /* LIDD_FB_REGS_H_ */
