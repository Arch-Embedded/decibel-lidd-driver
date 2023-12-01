/*
 * ldid_fb.h
 *
 *  Created on: May 8, 2016
 *      Author: Cezary Gapinski <cezary.gapinski@gmail.com>
 */

#ifndef LIDD_FB_H_
#define LIDD_FB_H_

#include <linux/dmaengine.h>

#define LIDD_FB_DMA_BUFFERS 2

struct lidd_dma_par
{
    dma_addr_t    vram_phys;        //should be same as dma_start
    unsigned long vram_size;
    uint8_t*      vram_virt;
};

struct lidd_par
{
    struct platform_device*    pdev;
    struct fb_info* info;
    struct resource* reg_res;
    u32 pseudo_palette[16];
    void __iomem* mmio;
    struct clk*   lcdc_clk;         //Power enable for the LCDC
    int blank_mode;
    //DMA/Memory things
    struct lidd_dma_par  dma_par[LIDD_FB_DMA_BUFFERS];
    bool frame_done;
    bool dma_done;
    wait_queue_head_t frame_done_wq;
    wait_queue_head_t dma_done_wq;

    struct work_struct dma_work;

    // DMA engine stuff
    struct dma_chan *dma_channel;
    int req_status;
	int tx_status;
    u64 dma_mask;
    dma_cookie_t cookie;
    /*
     * WARNING: Do not change the order of these two!!
     * The xt var contains a field that references to
     * the declared sg variable below...
     */
    struct dma_interleaved_template xt;
	struct data_chunk sg;

    wait_queue_head_t vsync_wait;
};

#define SET_VALHI(v)        (((v) & 0xff00) >> 8)
#define SET_VALLO(v)        (((v) & 0x00ff) >> 0)

#endif /* LIDD_FB_H_ */
