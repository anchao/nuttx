#ifndef SUNXI_DRV_SPINOR_H
#define SUNXI_DRV_SPINOR_H

#include <stdint.h>
#include <nuttx/mtd/mtd.h>

struct sunxi_spinor_dev_s
{
  FAR struct mtd_dev_s  mtd;         /* MTD interface */
  FAR struct spi_dev_s  *spi;        /* Saved SPI interface instance */
  uint16_t              nsectors;    /* Number of erase sectors */
  uint8_t               prev_instr;  /* Previous instruction given to W25 device */

  uint8_t               flags;       /* Buffered sector flags */
  uint16_t              esectno;     /* Erase sector number in the cache*/
  FAR uint8_t           *sector;     /* Allocated sector data */
  void                  *user_data;
};

#endif  /*SUNXI_DRV_SPINOR_H*/
