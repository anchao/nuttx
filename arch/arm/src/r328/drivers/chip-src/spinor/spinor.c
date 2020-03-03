#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>

#include <sunxi_hal_spinor.h>

#define NOR_CMD_READ 0x03
#define NOR_CMD_FAST_READ 0x0B
#define NOR_CMD_DUAL_READ 0x3B
#define NOR_CMD_QUAD_READ 0x6B
#define NOR_CMD_DUAL_IO_READ 0xBB
#define NOR_CMD_QUAD_IO_READ 0xEB
#define NOR_CMD_PROG 0x02
#define NOR_CMD_QUAD_PROG 0x32
#define NOR_CMD_QUAD_IO_PROG 0x38
#define NOR_CMD_ERASE_BLK4K 0x20
#define NOR_CMD_ERASE_BLK32K 0x52
#define NOR_CMD_ERASE_BLK64K 0xD8
#define NOR_CMD_ERASE_CHIP 0x60
#define NOR_CMD_WREN 0x06
#define NOR_CMD_READ_SR 0x05
#define NOR_CMD_WRITE_SR 0x01
#define NOR_CMD_READ_CR 0x15
#define NOR_CMD_READ_SCUR 0x2B
#define NOR_CMD_RESET_EN 0x66
#define NOR_CMD_RESET 0x99
#define NOR_CMD_RDID 0x9F

#define NOR_BUSY_MASK (1 << 0)
#define NOR_WEL_BIT (1 << 1)
#define NOR_DEFAULT_FREQUENCY 50
#define NOR_PAGE_SIZE 256

/* XTX */
#define NOR_XTX_QE_BIT (0x01 << 1)
#define NOR_XTX_CMD_READ_SR1 (0x35)

#ifndef MIN
#define MIN(a, b) (a > b ? b : a)
#endif

#define SZ_64K (64 * 1024)
#define SZ_32K (32 * 1024)
#define SZ_4K (4 * 1024)

#define MAX_WAIT_LOOP ((unsigned int)(-1))
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array)/sizeof(array[0]))
#endif

#ifndef pr_fmt
#define pr_fmt(fmt) fmt
#endif

#if 0
#define pr_emerg(fmt, ...) \
    printf(pr_fmt(fmt), ##__VA_ARGS__)
#define pr_alert(fmt, ...) \
    printf(pr_fmt(fmt), ##__VA_ARGS__)
#define pr_crit(fmt, ...) \
    printf(pr_fmt(fmt), ##__VA_ARGS__)
#define pr_err(fmt, ...) \
    printf(pr_fmt(fmt), ##__VA_ARGS__)
#define pr_warn(fmt, ...) \
    printf(pr_fmt(fmt), ##__VA_ARGS__)
#define pr_notice(fmt, ...) \
    printf(pr_fmt(fmt), ##__VA_ARGS__)
#define pr_info(fmt, ...) \
    printf(pr_fmt(fmt), ##__VA_ARGS__)
#define pr_debug(fmt, ...) \
    printf(pr_fmt(fmt), ##__VA_ARGS__)

#else
#define pr_emerg(fmt, ...)
#define pr_alert(fmt, ...)
#define pr_crit(fmt, ...)
#define pr_err(fmt, ...)
#define pr_warn(fmt, ...)
#define pr_notice(fmt, ...)
#define pr_info(fmt, ...)
#define pr_debug(fmt, ...)
#endif

extern const sunxi_hal_driver_spi_t sunxi_hal_spi_driver;
const sunxi_hal_driver_spi_t *hal_spi_driver = &sunxi_hal_spi_driver;

struct nor_info nor_ids[] =
{
    /* MXIC */
    {
        .name = "mx25l6433f",
        .id = {0xc2, 0x20, 0x17},
        .blk_size = 4 * 1024,
        .blk_cnt = 2048,
        .flag = EN_IO_PROG_X4,
    },
    {
        .name = "mx25l12833f",
        .id = {0xc2, 0x20, 0x18},
        .blk_size = 4 * 1024,
        .blk_cnt = 4096,
        .flag = EN_IO_PROG_X4 | NO_ERASE_64K,
    },
    /* Winbond */
    {
        .name = "w25q64jv",
        .id = {0xef, 0x40, 0x17},
        .blk_size = 4 * 1024,
        .blk_cnt = 2048,
        .flag = 0,
    },
    {
        .name = "w25q128jv",
        .id = {0xef, 0x40, 0x18},
        .blk_size = 4 * 1024,
        .blk_cnt = 4096,
        .flag = 0,
    },
    /* FM */
    {
        .name = "fm25w128",
        .id = {0xa1, 0x28, 0x18},
        .blk_size = 4 * 1024,
        .blk_cnt = 4096,
        .flag = 0,
    },
    /* ESMT */
    {
        .name = "en25qh128",
        .id = {0x1c, 0x70, 0x18},
        .blk_size = 4 * 1024,
        .blk_cnt = 4096,
        .flag = 0,
    },
    /* GD */
    {
        .name = "GD25Q127C",
        .id = {0xc8, 0x40, 0x18},
        .blk_size = 4 * 1024,
        .blk_cnt = 4096,
        .flag = 0,
    },
    /* XTX */
    {
        .name = "xt25f128",
        .id = {0x0b, 0x40, 0x18},
        .blk_size = 4 * 1024,
        .blk_cnt = 4096,
        .flag = 0,
    },
    /* Default */
    {
        .name = "Default",
        .id = {0, 0, 0},
        .blk_size = 4 * 1024,
        .blk_cnt = 4096,
        .flag = 0,
    }
};

static sunxi_hal_spinor_info hal_spinor_info = {0};

static sunxi_hal_version_t hal_spinor_driver =
{
    SUNXI_HAL_SPINOR_API_VERSION,
    SUNXI_HAL_SPINOR_DRV_VERSION
};

static const sunxi_hal_spinor_capabilities_t spinor_driver_capabilities =
{
    1, 0, 1, 0
};

static sunxi_hal_spinor_status_t spinor_status = {0, 0, 0};

static struct nor_flash g_nor = {0};
static struct nor_flash *nor = &g_nor;

static void nor_msleep(unsigned int msec)
{
    usleep(msec * 1000);
}

static int nor_lock(void)
{
    return sem_wait(&nor->hal_sem);
}

static int nor_unlock(void)
{
    return sem_post(&nor->hal_sem);
}

static int cmd_bit(unsigned char cmd)
{
    switch (cmd)
    {
        case NOR_CMD_DUAL_READ:
        case NOR_CMD_DUAL_IO_READ:
            return 2;
        case NOR_CMD_QUAD_READ:
        case NOR_CMD_QUAD_IO_READ:
            return 4;
        case NOR_CMD_QUAD_PROG:
        case NOR_CMD_QUAD_IO_PROG:
            return 4;
        default:
            return 1;
    }
}

static int nor_read_write(int hlen, void *tbuf, int tlen, void *rbuf, int rlen)
{
    hal_spi_master_transfer_t tr;
    unsigned char cmd = *(unsigned char *)tbuf;
    int ret;

    tr.tx_buf = tbuf;
    tr.tx_len = tlen;
    tr.rx_buf = rbuf;
    tr.rx_len = rlen;
    tr.tx_single_len = hlen;
    tr.dummy_byte = 0;

    tr.rx_nbits = tr.tx_nbits = SPI_NBITS_SINGLE;
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

    /* ret = hal_spi_driver->transfer(nor->spim.port, &tr); */
    ret = hal_spi_driver->control(nor->spim.port, SPI_WRITE_READ, &tr);

    if (ret)
    {
        pr_err("spi transfer failed %d\n", ret);
    }
    return ret;
}

static int nor_read_status(unsigned char *sr)
{
    int ret;
    char cmd[1] = {NOR_CMD_READ_SR};
    char reg[2] = {0};

    ret = nor_read_write(1, cmd, 1, reg, 2);
    if (ret)
    {
        pr_err("read status register fail\n");
        return ret;
    }

    *sr = reg[1];
    return 0;
}

static int nor_mxic_read_conf_reg(unsigned char *cr)
{
    int ret;
    char cmd[1] = {NOR_CMD_READ_CR};
    char reg[2] = {0};

    ret = nor_read_write(1, cmd, 1, reg, 2);
    if (ret)
    {
        pr_err("read configure register fail\n");
        return ret;
    }

    *cr = reg[1];
    return 0;
}

static int nor_is_busy(void)
{
    int ret;
    unsigned char reg;

    ret = nor_read_status(&reg);
    if (ret)
    {
        return ret;
    }

    if (reg & NOR_BUSY_MASK)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static int nor_wait_ready_us(unsigned int ms, unsigned int times)
{
    unsigned int _ms = ms, _times = times;

    do
    {
        if (nor_is_busy() == false)
        {
            return 0;
        }
        if (_ms)
        {
            unsigned int per_ms = 1000 / CONFIG_USEC_PER_TICK;

            nor_msleep(per_ms);
            _ms -= MIN(per_ms, _ms);
        }
    } while (_ms > 0);

    do
    {
        if (nor_is_busy() == false)
        {
            return 0;
        }
    } while (_times-- > 0);

    /* check the last time */
    if (nor_is_busy() == false)
    {
        return 0;
    }

    pr_err("wait nor flash for %d ms and %d loop timeout\n", ms, times);
    return -EBUSY;
}

static int nor_read_id(char *id, int len)
{
    int ret;
    char cmd[1] = {NOR_CMD_RDID};

    if (nor_wait_ready_us(0, 500))
    {
        pr_err("nor is busy before read id\n");
        return -EBUSY;
    }

    ret = nor_read_write(1, cmd, 1, id, MIN(len, (int)MAX_ID_LEN));
    if (ret)
    {
        pr_err("read nor id failed - %d\n", ret);
    }
    return ret;
}

static int nor_reset(void)
{
    int ret;
    char cmd[2] = {NOR_CMD_RESET_EN, NOR_CMD_RESET};

    ret = nor_read_write(2, cmd, 2, NULL, 0);
    if (ret)
    {
        pr_err("reset nor failed - %d\n", ret);
    }
    return ret;
}

static int nor_write_enable(void)
{
    int ret;
    char cmd = NOR_CMD_WREN;
    unsigned char sr;

    ret = nor_read_write(1, &cmd, 1, NULL, 0);
    if (ret)
    {
        pr_err("send WREN failed - %d\n", ret);
        return ret;
    }

    ret = nor_wait_ready_us(0, 500);
    if (ret)
    {
        return ret;
    }

    ret = nor_read_status(&sr);
    if (ret)
    {
        return ret;
    }

    if (!(sr & NOR_WEL_BIT))
    {
        pr_err("enable write failed\n");
        return -EINVAL;
    }
    return 0;
}

static struct nor_info *match_nor(char *id, int id_len)
{
    int i;
    struct nor_info *info;

    for (i = 0; i < ARRAY_SIZE(nor_ids) - 1; i++)
    {
        info = &nor_ids[i];
        if (!memcmp(info->id, id, id_len))
        {
            pr_debug("match nor %s on table\n", info->name);
            return info;
        }
    }
    pr_warn("unrecognized id (hex): %02x %02x %02x\n", id[0], id[1], id[2]);
    pr_warn("set it to 8M(64K block) default\n");
    return &nor_ids[i];
}

static int nor_write_status(unsigned char *sr, unsigned int len)
{
    int ret, i;
    char tbuf[5] = {0};

    ret = nor_write_enable();
    if (ret)
        return ret;

    if (len > 5)
        return -EINVAL;

    tbuf[0] = NOR_CMD_WRITE_SR;
    for (i = 0; i < len; i++)
        tbuf[i + 1] = *(sr + i);
    i++;
    ret = nor_read_write(i, tbuf, i, NULL, 0);
    if (ret) {
        pr_err("write status register fail\n");
        return ret;
    }

    return nor_wait_ready_us(10, MAX_WAIT_LOOP);
}

#define NOR_MXIC_QE_BIT (1 << 6)
static int nor_mxic_quad_mode(void)
{
    int ret;
    unsigned char cmd[3], sr, cr;

    ret = nor_write_enable();
    if (ret)
    {
        return ret;
    }

    ret = nor_read_status(&sr);
    if (ret)
    {
        return ret;
    }

    ret = nor_mxic_read_conf_reg(&cr);
    if (ret)
    {
        return ret;
    }

    cmd[0] = NOR_CMD_WRITE_SR;
    cmd[1] = sr | NOR_MXIC_QE_BIT;
    cmd[2] = cr;
    ret = nor_read_write(3, cmd, 3, NULL, 0);
    if (ret)
    {
        pr_err("set status register fail\n");
        return ret;
    }

    sr = 0;
    ret = nor_read_status(&sr);
    if (ret)
    {
        return ret;
    }
    if (!(sr & NOR_MXIC_QE_BIT))
    {
        pr_err("set mxic QE failed\n");
        return -EINVAL;
    }
    return 0;
}
#define NOR_GD_QE_BIT (1 << 1)
#define NOR_CMD_GD_RDSR2 0x35
#define NOR_CMD_GD_WRSR2 0x31
static int nor_gd_quad_mode(void)
{
    int ret;
    unsigned char cmd[3];
    char reg[2] = {0};

    cmd[0] = NOR_CMD_GD_RDSR2;
    ret = nor_read_write(1, cmd, 1, reg, 2);
    if (ret)
    {
        pr_err("read status register2 fail\n");
        return ret;
    }

    ret = nor_write_enable();
    if (ret)
    {
        return ret;
    }

    cmd[0] = NOR_CMD_GD_WRSR2;
    cmd[1] = reg[1] | NOR_GD_QE_BIT;
    ret = nor_read_write(2, cmd, 2, NULL, 0);
    if (ret)
    {
        pr_err("set status register fail\n");
        return ret;
    }

    if (nor_wait_ready_us(0, 500))
    {
        pr_err("wait set qd mode failed\n");
        return -EBUSY;
    }

    cmd[0] = NOR_CMD_GD_RDSR2;
    ret = nor_read_write(1, cmd, 1, reg, 2);
    if (ret)
    {
        pr_err("read status register2 fail\n");
        return ret;
    }
    if (!(reg[1] & NOR_GD_QE_BIT))
    {
        pr_err("set gd QE failed\n");
        return -EINVAL;
    }
    return 0;
}

/* xtx private function */
static int nor_xtx_read_status1(unsigned char *sr1)
{
    int ret;
    char cmd[1] = {NOR_XTX_CMD_READ_SR1};
    char reg[2] = {0};

    ret = nor_read_write(1, cmd, 1, reg, 2);
    if (ret) {
        pr_err("read xtx status1 register fail\n");
        return ret;
    }

    *sr1 = reg[1];
    return 0;
}

static int nor_xtx_quad_mode(void)
{
    int ret;
    unsigned char sr[2];

    ret = nor_xtx_read_status1(&sr[1]);
    if (ret)
        return ret;

    if (sr[1] & NOR_XTX_QE_BIT)
        return 0;

    sr[1] |= NOR_XTX_QE_BIT;

    ret = nor_read_status(&sr[0]);
    if (ret)
        return ret;

    ret = nor_write_status(sr, 2);
    if (ret)
        return ret;

    ret = nor_xtx_read_status1(&sr[1]);
    if (ret)
        return ret;
    if (!(sr[1] & NOR_XTX_QE_BIT)) {
        pr_err("set xtx QE failed (0x%x)\n", sr[1]);
        return -EINVAL;
    }
    return 0;
}

static int nor_set_quad_mode(void)
{
    switch (nor->info->id[0])
    {
        case FACTORY_MXIC:
            return nor_mxic_quad_mode();
        case FACTORY_GD:
            return nor_gd_quad_mode();
        case FACTORY_XTX:
            return nor_xtx_quad_mode();
        default:
            return 0;
    }
}

static int nor_scan(void)
{
    int ret;
    char id[MAX_ID_LEN];

    ret = nor_read_id(id, MAX_ID_LEN);
    if (ret)
    {
        goto err;
    }

    nor->info = match_nor(id, MAX_ID_LEN);
    nor->page_size = NOR_PAGE_SIZE;
#if defined(CONFIG_DRIVERS_NOR_FLASH_4K_SECTORS)
    nor->blk_size = 4 * 1024;
    nor->cmd_erase = NOR_CMD_ERASE_BLK4K;
#else
    nor->blk_size = nor->info->blk_size;
    switch (nor->blk_size)
    {
        case 64 * 1024:
            nor->cmd_erase = NOR_CMD_ERASE_BLK64K;
            break;
        case 32 * 1024:
            nor->cmd_erase = NOR_CMD_ERASE_BLK32K;
            break;
        case 4 * 1024:
            nor->cmd_erase = NOR_CMD_ERASE_BLK4K;
            break;
        default:
            pr_err("invalid blk size %u\n", nor->info->blk_size);
            ret = -EINVAL;
            goto err;
    }
#endif

    /* program property */
    nor->w_cmd_slen = 4;
#if defined(CONFIG_DRIVERS_NOR_FLASH_PROG_QUAD)
    nor->cmd_write = NOR_CMD_QUAD_PROG;
    if (nor->info->flag & EN_IO_PROG_X4)
    {
        nor->cmd_write = NOR_CMD_QUAD_IO_PROG;
        nor->w_cmd_slen = 1;
    }
#else
    nor->cmd_write = NOR_CMD_PROG;
#endif

    /* read property */
    nor->r_cmd_slen = 5;
#if defined(CONFIG_DRIVERS_NOR_FLASH_READ_FAST)
    nor->cmd_read = NOR_CMD_FAST_READ;
#elif defined(CONFIG_DRIVERS_NOR_FLASH_READ_DUAL)
    nor->cmd_read = NOR_CMD_DUAL_READ;
    if (nor->info->flag & EN_IO_READ_X2)
    {
        nor->cmd_read = NOR_CMD_DUAL_IO_READ;
        nor->r_cmd_slen = 1;
    }
#elif defined(CONFIG_DRIVERS_NOR_FLASH_READ_QUAD)
    nor->cmd_read = NOR_CMD_QUAD_READ;
    if (nor->info->flag & EN_IO_READ_X4)
    {
        nor->cmd_read = NOR_CMD_QUAD_IO_READ;
        nor->r_cmd_slen = 1;
    }
#else
    nor->cmd_read = NOR_CMD_READ;
#endif

    if (cmd_bit(nor->cmd_read) == 4 || cmd_bit(nor->cmd_write) == 4)
    {
        ret = nor_set_quad_mode();
        if (ret)
        {
            goto err;
        }
    }

    return 0;
err:
    pr_err("scan nor flash failed\n");
    nor->info = NULL;
    return ret;
}

static int nor_spi_master_init(struct spi_master *spim)
{
    int ret;

    spim->port = HAL_SPI_MASTER_0;

#if defined(CONFIG_DRIVERS_NOR_FLASH_FREQ)
    spim->cfg.clock_frequency = CONFIG_DRIVERS_NOR_FLASH_FREQ;
#else
    spim->cfg.clock_frequency = NOR_DEFAULT_FREQUENCY;
#endif
    spim->cfg.clock_frequency *= 1000 * 1000;
    spim->cfg.slave_port = HAL_SPI_MASTER_SLAVE_0;
    spim->cfg.cpha = HAL_SPI_MASTER_CLOCK_PHASE0;
    spim->cfg.cpol = HAL_SPI_MASTER_CLOCK_POLARITY0;
    spim->cfg.bit_order = HAL_SPI_MASTER_LSB_FIRST;

    ret = hal_spi_driver->initialize(spim->port);
    if (ret != HAL_SPI_MASTER_STATUS_OK)
    {
        pr_err("init spi master failed - %d\n", ret);
    }
    return ret;
}

static int nor_erase_do(char cmd, unsigned int addr)
{
    int ret = -EBUSY;
    char tbuf[4] = {0};

    if (!nor->info)
    {
        goto out;
    }

    ret = nor_lock();
    if (ret)
    {
        pr_err("erase: lock nor failed\n");
        goto out;
    }

    ret = nor_write_enable();
    if (ret)
    {
        goto unlock;
    }

    tbuf[0] = cmd;
    tbuf[1] = addr >> 16;
    tbuf[2] = addr >> 8;
    tbuf[3] = addr & 0xFF;

    ret = nor_read_write(4, tbuf, 4, NULL, 0);
    if (ret)
    {
        goto unlock;
    }
    if (cmd == NOR_CMD_ERASE_BLK64K)
    {
        ret = nor_wait_ready_us(150, 1000 * 1000);
    }
    else if (cmd == NOR_CMD_ERASE_BLK32K)
    {
        ret = nor_wait_ready_us(120, 1000 * 1000);
    }
    else
    {
        ret = nor_wait_ready_us(40, 1000 * 1000);
    }

unlock:
    if (nor_unlock())
    {
        ret = -EBUSY;
        pr_err("erase: unlock nor failed\n");
    }
out:
    if (ret)
    {
        pr_err("erase address 0x%x with cmd 0x%x failed\n", addr, cmd);
    }
    return ret;
}

static int nor_read_do(unsigned int addr, const char *buf, unsigned int len)
{
    char cmd[4] = {0};

    if (len > NOR_PAGE_SIZE)
    {
        return -EINVAL;
    }

    cmd[0] = nor->cmd_read;
    cmd[1] = addr >> 16;
    cmd[2] = addr >> 8;
    cmd[3] = addr & 0xFF;
    return nor_read_write(nor->r_cmd_slen, (void *)cmd, 4, (void *)buf, len);
}

static inline int nor_erase_4k(unsigned int addr)
{
    return nor_erase_do((char)NOR_CMD_ERASE_BLK4K, addr);
}

static inline int nor_erase_32k(unsigned int addr)
{
    return nor_erase_do((char)NOR_CMD_ERASE_BLK32K, addr);
}

static inline int nor_erase_64k(unsigned int addr)
{
    return nor_erase_do((char)NOR_CMD_ERASE_BLK64K, addr);
}

static inline int nor_erase_all(void)
{
    int ret = -1;
    char cmd = NOR_CMD_ERASE_CHIP;

    if (!nor->info)
    {
        goto out;
    }

    ret = nor_lock();
    if (ret)
    {
        pr_err("erase: lock nor failed\n");
        goto out;
    }

    pr_debug("try to erase all chip\n");

    ret = nor_write_enable();
    if (ret)
    {
        goto unlock;
    }

    ret = nor_read_write(1, &cmd, 1, NULL, 0);
    if (ret)
    {
        goto unlock;
    }
    ret = nor_wait_ready_us(26 * 1000, 1000 * 1000);

unlock:
    if (nor_unlock())
    {
        ret = -EBUSY;
        pr_err("erase: unlock nor failed\n");
    }
out:
    if (ret)
    {
        pr_err("erase all chip failed\n");
    }
    return ret;
}


static int nor_write_do(unsigned int addr, const char *buf, unsigned int len)
{
    int ret = -EINVAL;
    char tbuf[NOR_PAGE_SIZE + 4] = {0};

    if (len > NOR_PAGE_SIZE)
    {
        return -EINVAL;
    }

    ret = nor_write_enable();
    if (ret)
    {
        return ret;
    }

    tbuf[0] = nor->cmd_write;
    tbuf[1] = addr >> 16;
    tbuf[2] = addr >> 8;
    tbuf[3] = addr & 0xFF;
    memcpy(tbuf + 4, buf, MIN(len, (unsigned int)NOR_PAGE_SIZE));
    ret = nor_read_write(nor->w_cmd_slen, tbuf, len + 4, NULL, 0);
    if (ret)
    {
        return ret;
    }

    return nor_wait_ready_us(0, 8 * 1000);
}

static unsigned int nor_total_size(void)
{
    if (!nor->info)
    {
        return 0;
    }
    return nor->info->blk_cnt * nor->info->blk_size;
}

static sunxi_hal_version_t spinor_get_version(int32_t dev)
{
    (void) dev;
    return hal_spinor_driver;
}

static sunxi_hal_spinor_capabilities_t spinor_get_capabilities(void)
{
    return spinor_driver_capabilities;
}

static sunxi_hal_spinor_status_t spinor_get_status(void)
{
    return spinor_status;
}

int32_t spinor_initialize(sunxi_hal_spinor_signal_event_t cb_event)
{
    int ret = 0;

    if (nor->info)
    {
        return 0;
    }

    ret = sem_init(&nor->hal_sem, 0, 1);
    if (ret < 0)
    {
        pr_err("create hal_sem lock for nor_flash failed\n");
        goto out;
    }

    ret = nor_lock();
    if (ret)
    {
        pr_err("init: lock nor failed\n");
        goto out;
    }

    ret = nor_spi_master_init(&nor->spim);
    if (ret)
    {
        pr_err("init spi failed!\n");
        goto unlock;
    }

    ret = nor_reset();
    if (ret)
    {
        goto unlock;
    }

    ret = nor_scan();
    if (ret)
    {
        goto unlock;
    }

    printf("Nor Flash %s size %uMB write %dbit read %dbit blk size %uKB\n",
            nor->info->name, nor->info->blk_cnt * nor->info->blk_size / 1024 / 1024,
            cmd_bit(nor->cmd_write), cmd_bit(nor->cmd_read), nor->blk_size / 1024);
    printf("Nor Flash ID (hex): %02x %02x %02x\n", nor->info->id[0],
            nor->info->id[1], nor->info->id[2]);

unlock:
    if (nor_unlock())
    {
        ret = -EBUSY;
        pr_err("unlock nor failed\n");
    }
out:
    if (ret)
    {
        pr_err("init nor flash failed\n");
    }
    else
    {
        pr_info("nor flash init ok\n");
    }
    return ret;
}

static int32_t spinor_uninitialize(void)
{
    if (nor_lock())
    {
        pr_err("deinit: lock nor failed\n");
        return -EBUSY;
    }

    if (nor->info)
    {
        /* hal_spi_master_deinit(nor->spim.port); */
        memset(nor, 0, sizeof(struct nor_flash));
    }

    if (nor_unlock())
    {
        pr_err("erase: unlock nor failed\n");
        return -EBUSY;
    }

    return SUNXI_HAL_OK;
}

static int32_t spinor_power_control(sunxi_hal_power_state_e state)
{
    return SUNXI_HAL_OK;
}

int32_t spinor_read_data(uint32_t addr, const void *buf, uint32_t cnt)
{
    int ret = -1;

    ret = nor_lock();
    if (ret)
    {
        pr_err("read: lock nor failed\n");
        goto out;
    }

    ret = -EBUSY;
    if (!nor->info)
    {
        goto unlock;
    }

    pr_debug("try to read addr 0x%x with len %u\n", addr, cnt);

    if (nor_wait_ready_us(0, 500))
    {
        goto unlock;
    }

    while (cnt)
    {
        unsigned int rlen = MIN(cnt, (unsigned int)NOR_PAGE_SIZE);

        ret = nor_read_do(addr, buf, rlen);
        if (ret)
        {
            goto unlock;
        }

        addr += rlen;
        buf += rlen;
        cnt -= rlen;
    }

unlock:
    if (nor_unlock())
    {
        ret = -EBUSY;
        pr_err("read: unlock nor failed\n");
    }
out:
    if (ret)
    {
        pr_err("read address 0x%x with len %u failed\n", addr, cnt);
    }
    return ret;
}


int32_t spinor_program_data(uint32_t addr, const void *buf, uint32_t cnt)
{
    int ret;

    ret = nor_lock();
    if (ret)
    {
        pr_err("write: lock nor failed\n");
        goto out;
    }

    ret = -EBUSY;
    if (!nor->info)
    {
        goto unlock;
    }

    pr_debug("try to write addr 0x%x with len %u\n", addr, cnt);

    if (nor_wait_ready_us(0, 500))
    {
        goto unlock;
    }

    while (cnt)
    {
        unsigned int wlen = MIN(cnt, (unsigned int)nor->page_size);

        ret = nor_write_do(addr, (const char *)buf, wlen);
        if (ret)
        {
            goto unlock;
        }

        addr += wlen;
        buf += wlen;
        cnt -= wlen;
    }

unlock:
    if (nor_unlock())
    {
        ret = -EBUSY;
        pr_err("write: unlock nor failed\n");
    }
out:
    if (ret)
    {
        pr_err("write address 0x%x with len %u failed\n", addr, cnt);
    }
    return ret;
}

int32_t spinor_erase_sector(uint32_t addr, uint32_t size)
{
    int ret = 0;
    unsigned int total_size;

    /*
     * nor erase size can be 4k/32k/64k, driver should erase block size base
     * on given size. Here must be align to 4k
     */
    if (size % SZ_4K)
    {
        pr_err("erase size 4k is not align to %u\n", size);
        return -EINVAL;
    }

    total_size = nor->info->blk_cnt * nor->info->blk_size;
    if (addr + size > total_size)
    {
        pr_err("addr 0x%x with size %u over total size %u\n",
               addr, size, total_size);
        return -EINVAL;
    }

    pr_debug("try to erase addr 0x%x with size %u\n", addr, size);

    if (addr == 0 && size == total_size)
    {
        return nor_erase_all();
    }

    while (size >= SZ_64K)
    {
        if (nor->info->flag & NO_ERASE_64K)
        {
            break;
        }
        if (addr % SZ_64K)
        {
            break;
        }
        pr_debug("try to erase 64k from %u\n", addr);
        ret = nor_erase_64k(addr);
        if (ret)
        {
            return ret;
        }
        addr += SZ_64K;
        size -= SZ_64K;
    }

    while (size >= SZ_32K)
    {
        if (addr % SZ_32K)
        {
            break;
        }
        pr_debug("try to erase 32k from %u\n", addr);
        ret = nor_erase_32k(addr);
        if (ret)
        {
            return ret;
        }
        addr += SZ_32K;
        size -= SZ_32K;
    }

    while (size >= SZ_4K)
    {
        if (addr % SZ_4K)
        {
            break;
        }
        pr_debug("try to erase 4k from %u\n", addr);
        ret = nor_erase_4k(addr);
        if (ret)
        {
            return ret;
        }
        addr += SZ_4K;
        size -= SZ_4K;
    }

    if (size)
    {
        pr_err("erase fail as addr %u not align 64k/32k/4k\n", addr);
        return -EINVAL;
    }

    return SUNXI_HAL_OK;
}

static int32_t spinor_erase_chip(void)
{
    int ret = -1;
    char cmd = NOR_CMD_ERASE_CHIP;

    if (!nor->info)
    {
        goto out;
    }

    ret = nor_lock();
    if (ret)
    {
        pr_err("erase: lock nor failed\n");
        goto out;
    }

    pr_debug("try to erase all chip\n");

    ret = nor_write_enable();
    if (ret)
    {
        goto unlock;
    }

    ret = nor_read_write(1, &cmd, 1, NULL, 0);
    if (ret)
    {
        goto unlock;
    }
    ret = nor_wait_ready_us(26 * 1000, 1000 * 1000);

unlock:
    if (nor_unlock())
    {
        ret = -EBUSY;
        pr_err("erase: unlock nor failed\n");
    }
out:
    if (ret)
    {
        pr_err("erase all chip failed\n");
    }
    return ret;
}

static sunxi_hal_spinor_info *spinor_get_info(void)
{
    sunxi_hal_spinor_info *info;
    if (!nor->info)
    {
        return NULL;
    }

    info = &hal_spinor_info;
    info->sector_size = nor->info->blk_size;
    info->sector_count = nor->info->blk_cnt;
    info->page_size = nor->page_size;

    return info;
}

static void spinor_signal_event(uint32_t event)
{

}

static int32_t spinor_control(int32_t dev, uint32_t command, uint32_t arg)
{
    return SUNXI_HAL_OK;
}

const sunxi_hal_driver_spinor_t sunxi_hal_spinor_driver =
{
    .get_version  = spinor_get_version,
    .get_capabilities = spinor_get_capabilities,
    .initialize = spinor_initialize,
    .uninitialize = spinor_uninitialize,
    .power_control = spinor_power_control,
    .read_data = spinor_read_data,
    .program_data = spinor_program_data,
    .erase_sector = spinor_erase_sector,
    .erase_chip = spinor_erase_chip,
    .get_status = spinor_get_status,
    .get_info = spinor_get_info,
    .signal_event = spinor_signal_event,
    .control = spinor_control,
};
