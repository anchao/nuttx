#include <stdlib.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <nuttx/mtd/mtd.h>
#include <sys/mount.h>

#include <sunxi_drv_spinor.h>
#include <sunxi_hal_spinor.h>

#include "gptpart/part_efi.h"

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef pr_fmt
#define pr_fmt(fmt) fmt
#endif

#ifdef DEBUG
#define pr_debug(fmt, ...) \
    printf(pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_debug(fmt, ...)
#endif

#define MTD_PARTITION_DEV_PATH "/dev"
#define MTD_PARTITION_NAME_MAX 32
#define MTD_SPINOR_DEV_PATH "/dev/spinor"

#define SECTOR_SHIFT 9
#define SECTOR_SIZE (1 << SECTOR_SHIFT)

#define BLKPART_OFF_APPEND UINT32_MAX
#define BLKPART_SIZ_FULL UINT32_MAX
#define MAX_BLKNAME_LEN 16

struct part
{
    /* public */
    uint32_t off;
    uint32_t bytes;
    char name[MAX_BLKNAME_LEN];      /* name: UDISK */

    /* private */
    char devname[MAX_BLKNAME_LEN];   /* name: nor0p1 */
    struct mtd_dev_s *phdev;
    struct part *next;
};

struct syspart
{
    char name[MAX_BLKNAME_LEN];
    uint32_t offset;
    uint32_t bytes;
};

static const struct syspart syspart[] =
{
    {"boot0", 0, 112 * 1024},
    {"gpt", 112 * 1024, 16 * 1024},
};

static const sunxi_hal_driver_spinor_t *spinor_driver = &sunxi_hal_spinor_driver;

static int sunxi_spinor_erase(FAR struct mtd_dev_s *mtd_dev, off_t startblock, size_t nblocks)
{
    int ret = -1;
    struct mtd_geometry_s geo;

    if (mtd_dev == NULL)
    {
        return -EINVAL;
    }

    memset(&geo, 0, sizeof(struct mtd_geometry_s));

    ret = MTD_IOCTL(mtd_dev, MTDIOC_GEOMETRY, (unsigned long)&geo);
    if (ret)
    {
        return -EIO;
    }

    if (spinor_driver && spinor_driver->erase_sector)
    {
        ret = spinor_driver->erase_sector(startblock * geo.erasesize, nblocks * geo.erasesize);
    }
    else
    {
        ret = -EIO;
    }

    return ret;
}

static ssize_t sunxi_spinor_bread(FAR struct mtd_dev_s *mtd_dev, off_t startblock, size_t nblocks, uint8_t *buffer)
{
    int ret = -1;
    struct mtd_geometry_s geo;

    if (mtd_dev == NULL)
    {
        return -EINVAL;
    }

    memset(&geo, 0, sizeof(struct mtd_geometry_s));

    ret = MTD_IOCTL(mtd_dev, MTDIOC_GEOMETRY, (unsigned long)&geo);
    if (ret)
    {
        return -EIO;
    }

    ret = MTD_READ(mtd_dev, startblock * geo.blocksize, nblocks * geo.blocksize, buffer);
    if (ret == nblocks * geo.blocksize)
    {
        return nblocks;
    }

    return -EIO;
}

static ssize_t sunxi_spinor_bwrite(FAR struct mtd_dev_s *mtd_dev, off_t startblock, size_t nblocks, const uint8_t *buffer)
{
    int ret = -1;
    struct mtd_geometry_s geo;

    if (mtd_dev == NULL)
    {
        return -EINVAL;
    }

    memset(&geo, 0, sizeof(struct mtd_geometry_s));

    ret = MTD_IOCTL(mtd_dev, MTDIOC_GEOMETRY, (unsigned long)&geo);
    if (ret)
    {
        return -EIO;
    }

    ret = MTD_WRITE(mtd_dev, startblock * geo.blocksize, nblocks * geo.blocksize, buffer);
    if (ret == nblocks * geo.blocksize)
    {
        return nblocks;
    }

    return -EIO;
}

static int sunxi_spinor_read(FAR struct mtd_dev_s *mtd_dev, off_t pos, size_t size, uint8_t *buffer)
{
    int ret = -1;

    if (mtd_dev == NULL)
    {
        return -EINVAL;
    }

    if (spinor_driver && spinor_driver->read_data)
    {
        ret = spinor_driver->read_data(pos, buffer, size);
    }

    return ret ? ret : size;
}

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t sunxi_spinor_write(FAR struct mtd_dev_s *mtd_dev, off_t pos, size_t size, const uint8_t *buffer)
{
    int ret = -1;
    if (!mtd_dev)
    {
        return -EINVAL;
    }

    if (spinor_driver && spinor_driver->program_data)
    {
        ret = spinor_driver->program_data(pos, buffer, size);
    }

    return ret ? ret : size;
}
#endif

static int sunxi_spinor_ioctl(FAR struct mtd_dev_s *mtd_dev, int cmd, unsigned long arg)
{
    int ret = -1;
    struct mtd_geometry_s *geo = NULL;
    sunxi_hal_spinor_info *spinor_info = NULL;

    switch (cmd)
    {
        case MTDIOC_GEOMETRY:
            geo = (struct mtd_geometry_s *) arg;
            if (spinor_driver->get_info)
            {
                spinor_info = spinor_driver->get_info();
                if (spinor_info)
                {
                    geo->blocksize = spinor_info->page_size;
                    geo->erasesize = spinor_info->sector_size;
                    geo->neraseblocks = spinor_info->sector_count;
                    ret = 0;
                }
                else
                {
                    ret = -EIO;
                }
            }
        case BIOC_FLUSH:
            ret = 0;
            break;
        default:
            break;
    }
    return ret;
}

static int nor_get_gpt(struct mtd_dev_s *mtd_dev, uint8_t *buf, int len)
{
    int ret = -1;
    if (len < GPT_TABLE_SIZE)
    {
        pr_debug("buf too small for gpt\n");
        return -EINVAL;
    }

    pr_debug("read gpt from 0x%x\n", GPT_ADDRESS);
    ret = MTD_READ(mtd_dev, GPT_ADDRESS, GPT_TABLE_SIZE, buf);
    return ret <= 0 ? -EIO : 0;
}


static int register_mtd_part_device(struct mtd_dev_s *mtd_dev, const char *parent)
{
    int ret = -1, index = 0;
    uint8_t *gpt_buf;
    struct gpt_part *gpt_part;
    struct part *part;
    unsigned int offset = 0;
    unsigned int total_bytes = 0;
    unsigned int blk_bytes = 0;
    struct mtd_geometry_s geo ;

    struct part *parts;
    int n_parts = 0;

    gpt_buf = zalloc(GPT_TABLE_SIZE);
    if (!gpt_buf)
    {
        ret = -ENOMEM;
        goto err;
    }

    memset(&geo, 0, sizeof(struct mtd_geometry_s));

    MTD_IOCTL(mtd_dev, MTDIOC_GEOMETRY, (unsigned long)&geo);
    blk_bytes = geo.blocksize;
    total_bytes = geo.erasesize * geo.neraseblocks;

    ret = nor_get_gpt(mtd_dev, gpt_buf, GPT_TABLE_SIZE);
    if (ret)
    {
        pr_debug("get gpt from nor flash failed - %d\n", ret);
        goto err;
    }

#ifdef DEBUG
    show_gpt_part(gpt_buf);
#endif

    ret = gpt_part_cnt(gpt_buf);
    if (ret < 0)
    {
        pr_debug("get part count from gpt failed\n");
        goto err;
    }

    n_parts = ret + ARRAY_SIZE(syspart);
    parts = malloc(sizeof(struct part) * n_parts);
    if (!parts)
    {
        pr_debug("allocate part array failed.\n");
        ret = -ENOMEM;
        goto err;
    }
    memset(parts, 0, sizeof(struct part) * n_parts);

    for (index = 0; index < ARRAY_SIZE(syspart); index++)
    {
        part = &parts[index];
        part->bytes = syspart[index].bytes;
        part->off = syspart[index].offset;
        snprintf(part->name, MAX_BLKNAME_LEN, "%s", syspart[index].name);
        offset += part->bytes;
    }

    foreach_gpt_part(gpt_buf, gpt_part)
    {
        part = &parts[index++];
        part->bytes = gpt_part->sects << SECTOR_SHIFT;
        part->off = gpt_part->off_sects << SECTOR_SHIFT;
        snprintf(part->name, MAX_BLKNAME_LEN, "%s", gpt_part->name);
        offset += part->bytes;
    }

    if (index > 0)
    {
        index--;
        parts[index].bytes = total_bytes - (offset - parts[index].bytes);
    }

    free(gpt_buf);

    for (index = 0; index < n_parts; index++)
    {
        part = &parts[index];
        if ((part->bytes % blk_bytes) || (part->off % blk_bytes))
        {
            pr_debug("part %s with bytes %u off %u should align to block size %u\n",
                     part->name, part->bytes, part->off, blk_bytes);
            continue;
        }
        else
        {
            char partition_name[MTD_PARTITION_NAME_MAX];
            memset(partition_name, 0, sizeof(partition_name));
            sprintf(partition_name, "%s/%s", MTD_PARTITION_DEV_PATH, part->name);

            int first_block = part->off / blk_bytes;
            int nblocks = part->bytes / blk_bytes;

            pr_debug("part %s, firstblock %d, nblocks %d, bytes %d, off = %d\n",
                partition_name, first_block, nblocks, part->bytes, part->off);

            ret = register_mtdpartition(partition_name, 0755, MTD_SPINOR_DEV_PATH, first_block, nblocks);
            if (ret)
            {
                pr_debug("register mtd part %s failed, %d\n", partition_name, ret);
                ret = -EIO;
                break;
            }
        }
    }
    free(parts);
err:
    return ret;
}


int sunxi_driver_spinor_init(void)
{
    int ret = -1;
    FAR struct sunxi_spinor_dev_s *priv;

    if (!spinor_driver || !spinor_driver->initialize)
    {
        pr_debug("spinor driver is null!\n");
        return -1;
    }
    ret = spinor_driver->initialize(0);
    if (ret)
    {
        pr_debug("spinor init failed!\n");
        return -1;
    }

    priv = (FAR struct sunxi_spinor_dev_s *)zalloc(sizeof(struct sunxi_spinor_dev_s));
    if (!priv)
    {
        pr_debug("%s alloc memory failed %d!\n", __func__, __LINE__);
        return -ENOMEM;
    }

    priv->mtd.erase  = sunxi_spinor_erase;
    priv->mtd.bread  = sunxi_spinor_bread;
    priv->mtd.bwrite = sunxi_spinor_bwrite;
    priv->mtd.read   = sunxi_spinor_read;
    priv->mtd.ioctl  = sunxi_spinor_ioctl;
#ifdef CONFIG_MTD_BYTE_WRITE
    priv->mtd.write  = sunxi_spinor_write;
#endif
    priv->mtd.name   = "sunxi-spinor";
    priv->spi        = NULL;

    ret = register_mtddriver(MTD_SPINOR_DEV_PATH, &priv->mtd, 0755, NULL);
    if (ret)
    {
        pr_debug("register spinor mtd driver failed\n");
        return ret;
    }

    ret = register_mtd_part_device(&priv->mtd, MTD_SPINOR_DEV_PATH);
    if (ret)
    {
        pr_debug("register mtd part failed\n");
    }

    ret = mount("/dev/UDISK", "/data", "littlefs", 0, NULL);
    if (ret)
    {
        pr_debug("mount file system failed, %d!\n", ret);
    }

    return ret;
}
