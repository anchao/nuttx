#include <stdio.h>
#include <stdlib.h>
#include "hal_spi.h"

extern const sunxi_hal_driver_spi_t sunxi_hal_spi_driver;
const sunxi_hal_driver_spi_t *hal_spi_driver = &sunxi_hal_spi_driver;

static int nor_read_write(int hlen, void *tbuf, int tlen, void *rbuf, int rlen)
{
    hal_spi_master_transfer_t tr;
    int ret;

    tr.tx_buf = tbuf;
    tr.tx_len = tlen;
    tr.rx_buf = rbuf;
    tr.rx_len = rlen;
    tr.tx_single_len = hlen;
    tr.dummy_byte = 0;
    tr.mode = SUNXI_SPI_SYNC;
    //tr.mode = SUNXI_SPI_ASYNC;

    tr.rx_nbits = tr.tx_nbits = SPI_NBITS_SINGLE;
#if 0
    switch (cmd)
    {
        case NOR_CMD_FAST_READ:
            tr.dummy_byte = 1;
            break;
        case NOR_CMD_DUAL_READ:
        case NOR_CMD_DUAL_IO_READ:
            tr.rx_nbits = SPI_NBITS_DUAL;
            tr.dummy_byte = 1;
            break;
        case NOR_CMD_QUAD_READ:
        case NOR_CMD_QUAD_IO_READ:
            tr.rx_nbits = SPI_NBITS_QUAD;
            tr.dummy_byte = 1;
            break;
        case NOR_CMD_QUAD_PROG:
        case NOR_CMD_QUAD_IO_PROG:
            tr.tx_nbits = SPI_NBITS_QUAD;
            break;
    }
#endif
    /* ret = hal_spi_driver->transfer(nor->spim.port, &tr); */
    ret = hal_spi_driver->control(0, SPI_WRITE_READ, &tr);

    if (ret)
    {
        printf("spi transfer failed %d\n", ret);
    }
    return ret;
}


#define NOR_CMD_RDID 0x9F
char cmd[1] = {NOR_CMD_RDID};
static int nor_read_id(char *id, int len)
{
	int ret;

	ret = nor_read_write(1, cmd, 1, id, len);
	if (ret)
	{
		printf("read nor id failed - %d\n", ret);
	}
	return ret;
}

char id[30] = {0};
int nor_test(void)
{
	nor_read_id(id, 30);
	printf("\n\n===== nor id %0x  %0x  %0x====\n", id[0], id[1], id[2]);

	return 0;
}

static bool has_init = false;
int spi_test(void)
{
	int ret;

	printf("======spi read nor id test========\n");
	if (has_init == false) {
		ret = hal_spi_driver->initialize(0);
		if (ret < 0)
		{
			printf("init spi err %d\n", ret);
			return -1;
		}
		has_init = true;
	}

	nor_test();

	return 0;
}


