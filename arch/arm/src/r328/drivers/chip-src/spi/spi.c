#include <stdlib.h>
#include <nuttx/spi/spi.h>
#include "hal_spi.h"

extern const sunxi_hal_driver_spi_t sunxi_hal_spi_driver;

void sunxi_spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
		  FAR void *rxbuffer, size_t nwords);

uint32_t sunxi_spi_set_freq(FAR struct spi_dev_s *dev, uint32_t frequency);

void sunxi_spi_set_mode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);

struct sunxi_spi_dev
{
	struct spi_dev_s spi_dev;
	uint32_t port;
	const sunxi_hal_driver_spi_t *low_ops;
	void *priv;
};

const struct spi_ops_s sunxi_spi_ops = {
	.setfrequency = sunxi_spi_set_freq,
	.setmode = sunxi_spi_set_mode,
	.exchange = sunxi_spi_exchange,
};

#ifdef CONFIG_SUNXI_SPI0
struct sunxi_spi_dev sunxi_spi0 = {
	.spi_dev = {
		.ops = &sunxi_spi_ops,
	},
	.port = 0,
	.low_ops = &sunxi_hal_spi_driver,
};
#endif

#ifdef CONFIG_SUNXI_SPI1
struct sunxi_spi_dev sunxi_spi1 = {
	.spi_dev = {
		.ops = &sunxi_spi_ops,
	},
	.port = 1,
	.low_ops = &sunxi_hal_spi_driver,
};
#endif

#ifdef CONFIG_SUNXI_SPI2
struct sunxi_spi_dev sunxi_spi2 = {
	.spi_dev = {
		.ops = &sunxi_spi_ops,
	},
	.port = 2,
	.low_ops = &sunxi_hal_spi_driver,
};
#endif


void sunxi_spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
		  FAR void *rxbuffer, size_t nwords)
{
	struct sunxi_spi_dev *spi = (struct sunxi_spi_dev *)dev;
	sinfo("sunxi_spi_exchange\n");

	if (txbuffer != NULL && rxbuffer != NULL) {
		//spi->low_ops->control();
	}
	else if (txbuffer != NULL) {
		spi->low_ops->send(spi->port, txbuffer, nwords, SUNXI_SPI_SYNC);
	}
	else if (rxbuffer != NULL) {
		spi->low_ops->receive(spi->port, rxbuffer, nwords, SUNXI_SPI_SYNC);
	}
}

uint32_t sunxi_spi_set_freq(FAR struct spi_dev_s *dev, uint32_t frequency)
{
	struct sunxi_spi_dev *spi = (struct sunxi_spi_dev *)dev;

	sinfo("sunxi_spi_set_freq\n");
	spi->low_ops->control(spi->port, SPI_BUS_SPEED, &frequency);

}

void sunxi_spi_set_mode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
	uint32_t spi_mode = mode;
	struct sunxi_spi_dev *spi = (struct sunxi_spi_dev *)dev;

	sinfo("sunxi_spi_set_mode\n");
	spi->low_ops->control(spi->port, SPI_MODE, &spi_mode);
}


int sunxi_spi_driver_init(void)
{
#ifdef CONFIG_SUNXI_SPI0
	spi_register(&sunxi_spi0.spi_dev, 0);
#endif

#ifdef CONFIG_SUNXI_SPI1
	spi_register(&sunxi_spi1.spi_dev, 1);
#endif

#ifdef CONFIG_SUNXI_SPI2
	spi_register(&sunxi_spi2.spi_dev, 2);
#endif

	return 0;
}

