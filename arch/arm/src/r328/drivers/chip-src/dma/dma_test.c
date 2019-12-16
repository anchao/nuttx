/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the people's Republic of China and other countries.
* All Allwinner Technology Co.,Ltd. trademarks are used with permission.
*
* DISCLAIMER
* THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
* IF YOU NEED TO INTEGRATE THIRD PARTY¡¯S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
* IN ALLWINNERS¡¯SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
* ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
* ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
* COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
* YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY¡¯S TECHNOLOGY.
*
*
* THIS SOFTWARE IS PROVIDED BY ALLWINNER"AS IS" AND TO THE MAXIMUM EXTENT
* PERMITTED BY LAW, ALLWINNER EXPRESSLY DISCLAIMS ALL WARRANTIES OF ANY KIND,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING WITHOUT LIMITATION REGARDING
* THE TITLE, NON-INFRINGEMENT, ACCURACY, CONDITION, COMPLETENESS, PERFORMANCE
* OR MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* IN NO EVENT SHALL ALLWINNER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "dma.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*
use:
	dma_slave_config_t config;
	sunxi_dma_init();

	hchan = sunxi_dma_chan_request();

	dma_slave_config(hchan,&config);

	sunxi_prep_dma_cyclic();

	sunxi_dma_start();

	sunxi_dma_free();
*/


void dma_test()
{
	int i;
	struct sunxi_dma_chan *chan = NULL;
	char *buf1 = NULL,*buf2 = NULL;
	struct dma_slave_config config = {0};
	uint32_t size = 0;

	printf("run in dma test\n");
	char *buf = NULL;
	buf1 = malloc(1024);
	buf2 = malloc(1024);
	if (buf1 == NULL || buf2 == NULL) {
		printf("malloc fail\n");
		return ;
	}

	for (i = 0; i < 1023; i++) {
		buf1[i] = 'a';
	}
	buf1[998] = 'b';
	buf1[999] = '\0';

	sunxi_dma_init();				//init dma,should init once

	//request dma chan
	chan = (struct sunxi_dma_chan *)sunxi_dma_chan_request();
	if (chan == NULL)
		return;
	if (chan < 0)
		printf("request chan err\n");
	config.direction = DMA_MEM_TO_MEM;
	config.dst_addr = (uint32_t)buf2;
	config.src_addr = (uint32_t)buf1;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
	config.dst_maxburst = DMA_SLAVE_BURST_16;
	config.src_maxburst = DMA_SLAVE_BURST_16;
	config.slave_id = sunxi_slave_id(DRQDST_SDRAM, DRQSRC_SDRAM);

	dma_slave_config(chan, &config);

	//sunxi_prep_dma_cyclic(chan, (uint32_t)buf1, 1000,(uint32_t)100, DMA_MEM_TO_MEM);
	sunxi_prep_dma_memcpy(chan, (uint32_t)buf2, (uint32_t)buf1,(uint32_t)1024);

	sunxi_dma_start_desc(chan);

	for(i = 50000; i == 0;i--);

	//when to stop?
	while (sunxi_tx_status(chan, &size) != 0);

	sunxi_dma_stop_desc(chan);

	sunxi_dma_chan_free(chan);

	printf("\n\ndma test end:\n");
	printf("\t buf1:%s\n",buf1);
	printf("\t buf2:%s\n",buf2);
	free(buf1);
	free(buf2);
}

