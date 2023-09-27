/*
 * ldid_fb.h
 *
 *  Created on: May 8, 2016
 *      Author: Cezary Gapinski <cezary.gapinski@gmail.com>
 */

#ifndef LIDD_FB_H_
#define LIDD_FB_H_

struct lidd_par
{
    struct platform_device*    pdev;
    struct fb_info* info;
    struct resource* reg_res;
    u32 pseudo_palette[16];
    bool          first_update_done;
    void __iomem* mmio;
    struct clk*   lcdc_clk;         //Power enable for the LCDC
    //DMA/Memory things
    dma_addr_t    vram_phys;        //should be same as dma_start
    unsigned long vram_size;
    uint8_t*      vram_virt;
    uint32_t      dma_start;        //physical addresses
    uint32_t      dma_end;
};

#define SET_VALHI(v)        (((v) & 0xff00) >> 8)
#define SET_VALLO(v)        (((v) & 0x00ff) >> 0)

#endif /* LIDD_FB_H_ */
