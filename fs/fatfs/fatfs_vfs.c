/****************************************************************************
 * fs/fatfs/fatfs_vfs.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <debug.h>
#include <assert.h>
#include <stdatomic.h>

#include <nuttx/fs/dirent.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

#include <sys/stat.h>
#include <sys/statfs.h>

#include "inode/inode.h"

#define DIR DIR_
#include "ff.h"
#include "diskio.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fatfs_file_s
{
  FIL        f;
  atomic_int refs;
  char       path[PATH_MAX];
};

struct fatfs_mountpt_s
{
  FATFS      fat;
  BYTE       pdrv;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     fatfs_open(FAR struct file *filep, FAR const char *relpath,
                          int oflags, mode_t mode);
static int     fatfs_close(FAR struct file *filep);
static ssize_t fatfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t fatfs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static off_t   fatfs_seek(FAR struct file *filep, off_t offset, int whence);
static int     fatfs_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

static int     fatfs_sync(FAR struct file *filep);
static int     fatfs_dup(FAR const struct file *oldp, FAR struct file *newp);
static int     fatfs_fstat(FAR const struct file *filep,
                           FAR struct stat *buf);
static int     fatfs_fchstat(FAR const struct file *filep,
                             FAR const struct stat *buf, int flags);
static int     fatfs_truncate(FAR struct file *filep, off_t length);

static int     fatfs_opendir(FAR struct inode *mountpt,
                             FAR const char *relpath,
                             FAR struct fs_dirent_s *dir);
static int     fatfs_closedir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir);
static int     fatfs_readdir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir);
static int     fatfs_rewinddir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir);

static int     fatfs_bind(FAR struct inode *driver,
                          FAR const void *data, FAR void **handle);
static int     fatfs_unbind(FAR void *handle, FAR struct inode **driver,
                            unsigned int flags);
static int     fatfs_statfs(FAR struct inode *mountpt,
                            FAR struct statfs *buf);

static int     fatfs_unlink(FAR struct inode *mountpt,
                            FAR const char *relpath);
static int     fatfs_mkdir(FAR struct inode *mountpt,
                           FAR const char *relpath, mode_t mode);
static int     fatfs_rmdir(FAR struct inode *mountpt,
                           FAR const char *relpath);
static int     fatfs_rename(FAR struct inode *mountpt,
                            FAR const char *oldrelpath,
                            FAR const char *newrelpath);
static int     fatfs_stat(FAR struct inode *mountpt,
                          FAR const char *relpath, FAR struct stat *buf);
static int     fatfs_chstat(FAR struct inode *mountpt,
                            FAR const char *relpath,
                            FAR const struct stat *buf, int flags);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_sem = SEM_INITIALIZER(1);

/* The number of block device mounted */

static FAR struct inode *g_drv[FF_VOLUMES];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations fatfs_operations =
{
  fatfs_open,          /* open */
  fatfs_close,         /* close */
  fatfs_read,          /* read */
  fatfs_write,         /* write */
  fatfs_seek,          /* seek */
  fatfs_ioctl,         /* ioctl */

  fatfs_sync,          /* sync */
  fatfs_dup,           /* dup */
  fatfs_fstat,         /* fstat */
  fatfs_fchstat,       /* fchstat */
  fatfs_truncate,      /* truncate */

  fatfs_opendir,       /* opendir */
  fatfs_closedir,      /* closedir */
  fatfs_readdir,       /* readdir */
  fatfs_rewinddir,     /* rewinddir */

  fatfs_bind,          /* bind */
  fatfs_unbind,        /* unbind */
  fatfs_statfs,        /* statfs */

  fatfs_unlink,        /* unlink */
  fatfs_mkdir,         /* mkdir */
  fatfs_rmdir,         /* rmdir */
  fatfs_rename,        /* rename */
  fatfs_stat,          /* stat */
  fatfs_chstat         /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static BYTE fatfs_alloc_slot(FAR struct inode *drv)
{
  BYTE i;

  if (nxsem_wait_uninterruptible(&g_sem) < 0)
    {
      return UCHAR_MAX;
    }

  for (i = 0; i < FF_VOLUMES; i++)
    {
      if (g_drv[i] == NULL)
        {
          g_drv[i] = drv;
          nxsem_post(&g_sem);
          return i;
        }
    }

  nxsem_post(&g_sem);
  return UCHAR_MAX;
}

static FAR struct inode *fatfs_free_slot(BYTE pdrv)
{
  FAR struct inode *drv;

  if (nxsem_wait_uninterruptible(&g_sem) < 0)
    {
      return NULL;
    }

  drv = g_drv[pdrv];
  g_drv[pdrv] = NULL;
  nxsem_post(&g_sem);
  return drv;
}

static int fatfs_convert_result(FRESULT ret)
{
  switch (ret)
    {
      case FR_OK:
        return 0;

      case FR_DISK_ERR:
      case FR_NOT_READY:
      case FR_MKFS_ABORTED:
        return -EIO;

      case FR_NO_FILE:
      case FR_NO_PATH:
        return -ENOENT;

      case FR_INVALID_NAME:
      case FR_INVALID_OBJECT:
      case FR_INVALID_DRIVE:
      case FR_NOT_ENABLED:
      case FR_NO_FILESYSTEM:
      case FR_INVALID_PARAMETER:
        return -EINVAL;

      case FR_DENIED:
      case FR_WRITE_PROTECTED:
      case FR_LOCKED:
        return -EPERM;

      case FR_EXIST:
        return -EEXIST;

      case FR_TIMEOUT:
        return -ETIMEDOUT;

      case FR_NOT_ENOUGH_CORE:
        return -ENOMEM;

      case FR_TOO_MANY_OPEN_FILES:
        return -ENMFILE;

      case FR_INT_ERR:
      default:
        return -EFAULT;
    }
}

static int fatfs_convert_oflags(int oflags)
{
  int ret = 0;

  if ((oflags & O_RDONLY) != 0)
    {
      ret |= FA_READ;
    }

  if ((oflags & O_WRONLY) != 0)
    {
      ret |= FA_WRITE;
    }

  if ((oflags & O_CREAT) != 0)
    {
      ret |= FA_OPEN_ALWAYS;
    }

  if ((oflags & O_APPEND) != 0)
    {
      ret |= FA_OPEN_APPEND;
    }

  if ((oflags & O_TRUNC) != 0)
    {
      ret |= FA_CREATE_ALWAYS;
    }

  return ret;
}

/****************************************************************************
 * Name: fatfs_open
 ****************************************************************************/

static int fatfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct fatfs_mountpt_s *fs;
  FAR struct fatfs_file_s *fp;
  int ret;

  fp = kmm_malloc(sizeof(*fp));
  if (!fp)
    {
      return -ENOMEM;
    }

  fs = filep->f_inode->i_private;

  fp->path[0] = '0' + fs->pdrv;
  fp->path[1] = ':';
  fp->path[2] = '\0';
  oflags = fatfs_convert_oflags(oflags);
  ret = fatfs_convert_result(f_open(&fp->f, strcat(fp->path, relpath),
                                    oflags));
  if (ret < 0)
    {
      kmm_free(fp);
      return ret;
    }

  atomic_store(&fp->refs, 1);
  filep->f_priv = fp;
  return ret;
}

/****************************************************************************
 * Name: fatfs_close
 ****************************************************************************/

static int fatfs_close(FAR struct file *filep)
{
  FAR struct fatfs_file_s *fp;
  int ret = OK;

  fp = filep->f_priv;
  if (atomic_fetch_sub(&fp->refs, 1) <= 1)
    {
      ret = fatfs_convert_result(f_close(&fp->f));
      if (ret < 0)
        {
          atomic_fetch_add(&fp->refs, 1);
        }
      else
        {
          kmm_free(fp);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: fatfs_read
 ****************************************************************************/

static ssize_t fatfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct fatfs_file_s *fp;
  ssize_t ret;
  UINT size;

  fp = filep->f_priv;
  ret = fatfs_convert_result(f_read(&fp->f, buffer, buflen, &size));
  if (ret >= 0)
    {
      filep->f_pos += size;
      ret = size;
    }

  return ret;
}

/****************************************************************************
 * Name: fatfs_write
 ****************************************************************************/

static ssize_t fatfs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  FAR struct fatfs_file_s *fp;
  ssize_t ret;
  UINT size;

  fp = filep->f_priv;
  ret = fatfs_convert_result(f_write(&fp->f, buffer, buflen, &size));
  if (ret >= 0)
    {
      filep->f_pos += size;
      ret = size;
    }

  return ret;
}

/****************************************************************************
 * Name: fatfs_seek
 ****************************************************************************/

static off_t fatfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct fatfs_file_s *fp;
  off_t ret;

  fp = filep->f_priv;

  /* Map the offset according to the whence option */

  switch (whence)
    {
      case SEEK_SET:
          break;

      case SEEK_CUR:
          offset += filep->f_pos;
          break;

      case SEEK_END:
          offset += f_size(&fp->f);
          break;

      default:
          return -EINVAL;
    }

  ret = fatfs_convert_result(f_lseek(&fp->f, offset));
  if (ret >= 0)
    {
      filep->f_pos = offset;
    }

  return ret < 0 ? ret : filep->f_pos;
}

/****************************************************************************
 * Name: fatfs_ioctl
 ****************************************************************************/

static int fatfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct fatfs_mountpt_s *fs;
  FAR struct fatfs_file_s *fp;

  /* Recover our private data from the struct file instance */

  fs = filep->f_inode->i_private;
  fp = filep->f_priv;
  if (cmd == FIOC_FILEPATH)
    {
      FAR char *ptr = (FAR char *)((uintptr_t)arg);
      inode_getpath(filep->f_inode, ptr);
      strcat(ptr, &fp->path[2]);
      return OK;
    }
  else
    {
      FAR struct inode *drv = g_drv[fs->pdrv];
      return drv->u.i_bops->ioctl(drv, cmd, arg);
    }
}

/****************************************************************************
 * Name: fatfs_sync
 *
 * Description: Synchronize the file state on disk to match internal, in-
 *   memory state.
 *
 ****************************************************************************/

static int fatfs_sync(FAR struct file *filep)
{
  FAR struct fatfs_file_s *fp;

  fp = filep->f_priv;
  return fatfs_convert_result(f_sync(&fp->f));
}

/****************************************************************************
 * Name: fatfs_dup
 *
 * Description: Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int fatfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct fatfs_file_s *fp;

  /* Recover our private data from the struct file instance */

  fp = oldp->f_priv;
  atomic_fetch_add(&fp->refs, 1);
  newp->f_priv = fp;
  return OK;
}

/****************************************************************************
 * Name: fatfs_stat_i
 ****************************************************************************/

static int fatfs_stat_i(FAR const char *path, FAR struct stat *buf)
{
  struct tm gm;
  FILINFO fno;
  int ret;

  memset(buf, 0, sizeof(struct stat));
  if (path[2] == '\0')
    {
      buf->st_mode |= S_IFDIR;
      return OK;
    }

  ret = fatfs_convert_result(f_stat(path, &fno));
  if (ret < 0)
    {
      return ret;
    }

  if (fno.fattrib & AM_DIR)
    {
      buf->st_mode |= S_IFDIR;
    }

  buf->st_size  = fno.fsize;

  gm.tm_year    = 80 + (fno.fdate >> 9);
  gm.tm_mon     = ((fno.fdate >> 5) & 0xf) - 1;
  gm.tm_mday    = fno.fdate & 0x1f;
  gm.tm_hour    = fno.ftime >> 11;
  gm.tm_min     = (fno.ftime >> 5) & 0x3f;
  gm.tm_sec     = (fno.ftime & 0x1f) << 1;
  buf->st_mtime = timegm(&gm);
  buf->st_atime = buf->st_mtime;
  buf->st_ctime = buf->st_mtime;

  return ret;
}

/****************************************************************************
 * Name: fatfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int fatfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct fatfs_file_s *fp;
  int ret;

  /* Get the mountpoint private data from the inode structure */

  fp = filep->f_priv;

  /* Synchronize the File */

  ret = fatfs_convert_result(f_sync(&fp->f));
  if (ret < 0)
    {
      return ret;
    }

  return fatfs_stat_i(fp->path, buf);
}

/****************************************************************************
 * Name: fatfs_chstat_i
 ****************************************************************************/

static int fatfs_chstat_i(FAR const char *path,
                          FAR const struct stat *buf, int flags)
{
  if (flags & CH_STAT_MODE)
    {
      const mode_t VALID_MODE_MASK = S_IFREG | S_IFDIR |
                                     S_IRWXU | S_IRWXG | S_IRWXO;
      if (buf->st_mode & ~VALID_MODE_MASK)
        {
          return -EPERM;
        }
    }

  if ((flags & CH_STAT_GID) || (flags & CH_STAT_UID))
    {
      return -EPERM;
    }

  if (flags & CH_STAT_MTIME)
    {
      struct tm gm;
      FILINFO fno;

      gmtime_r(&buf->st_mtime, &gm);
      fno.fdate = ((gm.tm_year - 80) << 9) | ((gm.tm_mon + 1) << 5) |
                  gm.tm_mday;
      fno.ftime = (gm.tm_hour << 11) | (gm.tm_min << 5) | (gm.tm_sec >> 1);
      return fatfs_convert_result(f_utime(path, &fno));
    }

  return OK;
}

/****************************************************************************
 * Name: fatfs_fchstat
 ****************************************************************************/

static int fatfs_fchstat(FAR const struct file *filep,
                         FAR const struct stat *buf, int flags)
{
  FAR struct fatfs_file_s *fp;

  /* Recover our private data from the struct file instance */

  fp = filep->f_priv;
  return fatfs_chstat_i(fp->path, buf, flags);
}

/****************************************************************************
 * Name: fatfs_truncate
 *
 * Description:
 *   Set the length of the open, regular file associated with the file
 *   structure 'filep' to 'length'.
 *
 ****************************************************************************/

static int fatfs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct fatfs_file_s *fp;
  FSIZE_t pos;
  int ret;

  /* Recover our private data from the struct file instance */

  fp = filep->f_priv;

  pos = f_tell(&fp->f);
  ret = fatfs_convert_result(f_lseek(&fp->f, length));
  if (ret < 0)
    {
      return ret;
    }

  if (length < f_size(&fp->f))
    {
      ret = fatfs_convert_result(f_truncate(&fp->f));
    }

  fatfs_convert_result(f_lseek(&fp->f, pos));
  return ret;
}

/****************************************************************************
 * Name: fatfs_opendir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int fatfs_opendir(FAR struct inode *mountpt,
                         FAR const char *relpath,
                         FAR struct fs_dirent_s *dir)
{
  FAR struct fatfs_mountpt_s *fs;
  char path[strlen(relpath) + 3];
  FAR DIR *dp;
  int ret;

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Allocate memory for the open directory */

  dp = kmm_malloc(sizeof(*dp));
  if (dp == NULL)
    {
      return -ENOMEM;
    }

  path[0] = '0' + fs->pdrv;
  path[1] = ':';
  path[2] = '\0';
  ret = fatfs_convert_result(f_opendir(dp, strcat(path, relpath)));
  if (ret < 0)
    {
      kmm_free(dp);
      return ret;
    }

  dir->u.fatfs = dp;
  return ret;
}

/****************************************************************************
 * Name: fatfs_closedir
 *
 * Description: Close a directory
 *
 ****************************************************************************/

static int fatfs_closedir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir)
{
  int ret;

  ret = fatfs_convert_result(f_closedir(dir->u.fatfs));
  if (ret >= 0)
    {
      kmm_free(dir->u.fatfs);
    }

  return ret;
}

/****************************************************************************
 * Name: fatfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int fatfs_readdir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir)
{
  FAR DIR *dp;
  FILINFO fno;
  int ret;

  /* Recover our private data from the inode instance */

  dp = dir->u.fatfs;
  ret = fatfs_convert_result(f_readdir(dp, &fno));
  if (ret < 0)
    {
      return ret;
    }

  if (!fno.fname[0])
    {
      return -ENOENT;
    }

  if (fno.fattrib & AM_DIR)
    {
      dir->fd_dir.d_type = DTYPE_DIRECTORY;
    }
  else
    {
      dir->fd_dir.d_type = DTYPE_FILE;
    }

  strlcpy(dir->fd_dir.d_name, fno.fname, sizeof(dir->fd_dir.d_name));

  return ret;
}

/****************************************************************************
 * Name: fatfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int fatfs_rewinddir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  return fatfs_convert_result(f_rewinddir(dir->u.fatfs));
}

/****************************************************************************
 * Name: fatfs_bind
 ****************************************************************************/

static bool fatfs_match_option(FAR const char *options,
                               FAR const char *option_name)
{
  FAR const char *p;
  size_t length = strlen(option_name);

  for (p = strstr(options, option_name); p; p = strstr(p + 1, option_name))
    {
      if ((p == options || *(p - 1) == ',') &&
          (p[length] == ',' || p[length] == '\0'))
        {
          return true;
        }
    }

  return false;
}

static int fatfs_bind(FAR struct inode *driver, FAR const void *data,
                      FAR void **handle)
{
  FAR struct fatfs_mountpt_s *fs;
  MKFS_PARM opt;
  char path[3];
  int ret;

  /* Create an instance of the mountpt state structure */

  fs = kmm_zalloc(sizeof(*fs));
  if (!fs)
    {
      return -ENOMEM;
    }

  /* Initialize the allocated mountpt state structure. The filesystem is
   * responsible for one reference on the driver inode and does not
   * have to addref() here (but does have to release in unbind().
   */

  fs->pdrv = fatfs_alloc_slot(driver);
  if (fs->pdrv == UCHAR_MAX)
    {
      ret = -ENMFILE;
      goto errout_with_fs;
    }

  path[0] = '0' + fs->pdrv;
  path[1] = ':';
  path[2] = '\0';
  if (driver->u.i_bops->open)
    {
      ret = driver->u.i_bops->open(driver);
      if (ret < 0)
        {
          ferr("ERROR: %d driver open failed\n", fs->pdrv);
          goto errout_with_pdrv;
        }
    }

  /* Force format the device if -o forceformat/audoformat  */

  if (data && fatfs_match_option(data, "forceformat"))
    {
      /* format the device if -o "forceformat"  */

      memset(&opt, 0, sizeof(opt));
      opt.fmt = FM_ANY | FM_SFD;
      opt.n_fat = 2;
      ret = fatfs_convert_result(f_mkfs(path, &opt, NULL, FF_MAX_SS));
      if (ret < 0)
        {
          goto errout_with_open;
        }
    }

  ret = fatfs_convert_result(f_mount(&fs->fat, path, 1));
  if (ret < 0)
    {
      /* Auto format the device if -o "autoformat" */

      if ((ret != -EIO && ret != -EINVAL) ||
          !data || !fatfs_match_option(data, "autoformat"))
        {
          goto errout_with_open;
        }

      memset(&opt, 0, sizeof(opt));
      opt.fmt = FM_ANY | FM_SFD;
      opt.n_fat = 2;
      ret = fatfs_convert_result(f_mkfs(path, &opt, NULL, FF_MAX_SS));
      if (ret < 0)
        {
          goto errout_with_open;
        }

      /* Try to mount the device again */

      ret = fatfs_convert_result(f_mount(&fs->fat, path, 1));
      if (ret < 0)
        {
          goto errout_with_open;
        }
    }

  *handle = fs;
  return OK;

errout_with_open:
  if (driver->u.i_bops->close)
    {
      driver->u.i_bops->close(driver);
    }

errout_with_pdrv:
  fatfs_free_slot(fs->pdrv);
errout_with_fs:
  kmm_free(fs);
  return ret;
}

/****************************************************************************
 * Name: fatfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *  operation.
 *
 ****************************************************************************/

static int fatfs_unbind(FAR void *handle, FAR struct inode **driver,
                        unsigned int flags)
{
  FAR struct fatfs_mountpt_s *fs = handle;
  char path[3];
  int ret;

  path[0] = '0' + fs->pdrv;
  path[1] = ':';
  path[2] = '\0';
  ret = fatfs_convert_result(f_unmount(path));

  /* We hold a reference to the driver but should not but
   * mucking with inodes in this context. So, we will just return
   * our contained reference to the driver inode and let the
   * umount logic dispose of it.
   */

  *driver = fatfs_free_slot(fs->pdrv);
  if ((*driver)->u.i_bops->close)
    {
      (*driver)->u.i_bops->close(*driver);
    }

  /* Release the mountpoint private data */

  kmm_free(fs);
  return ret;
}

/****************************************************************************
 * Name: fatfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int fatfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct fatfs_mountpt_s *fs;
  FAR FATFS *fat;
  DWORD nclst;
  char path[3];
  int ret;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Return something for the file system description */

  path[0] = '0' + fs->pdrv;
  path[1] = ':';
  path[2] = '\0';
  ret = fatfs_convert_result(f_getfree(path, &nclst, &fat));
  if (ret < 0)
    {
      return ret;
    }

  memset(buf, 0, sizeof(*buf));
  buf->f_type    = FATFS_SUPER_MAGIC;
  buf->f_namelen = FF_MAX_LFN;
  buf->f_bsize   = fat->csize * fat->ssize;
  buf->f_blocks  = fat->n_fatent - 2;
  buf->f_bfree   = nclst;
  buf->f_bavail  = nclst;

  return ret;
}

/****************************************************************************
 * Name: fatfs_unlink
 *
 * Description: Remove a file
 *
 ****************************************************************************/

static int fatfs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct fatfs_mountpt_s *fs;
  char path[strlen(relpath) + 3];

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  path[0] = '0' + fs->pdrv;
  path[1] = ':';
  path[2] = '\0';
  return fatfs_convert_result(f_unlink(strcat(path, relpath)));
}

/****************************************************************************
 * Name: fatfs_mkdir
 *
 * Description: Create a directory
 *
 ****************************************************************************/

static int fatfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                       mode_t mode)
{
  FAR struct fatfs_mountpt_s *fs;
  char path[strlen(relpath) + 3];

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  path[0] = '0' + fs->pdrv;
  path[1] = ':';
  path[2] = '\0';
  return fatfs_convert_result(f_mkdir(strcat(path, relpath)));
}

/****************************************************************************
 * Name: fatfs_rmdir
 *
 * Description: Remove a directory
 *
 ****************************************************************************/

static int fatfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct fatfs_mountpt_s *fs;
  char path[strlen(relpath) + 3];

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  path[0] = '0' + fs->pdrv;
  path[1] = ':';
  path[2] = '\0';
  return fatfs_convert_result(f_rmdir(strcat(path, relpath)));
}

/****************************************************************************
 * Name: fatfs_rename
 *
 * Description: Rename a file or directory
 *
 ****************************************************************************/

static int fatfs_rename(FAR struct inode *mountpt,
                        FAR const char *oldrelpath,
                        FAR const char *newrelpath)
{
  FAR struct fatfs_mountpt_s *fs;
  char oldpath[strlen(oldrelpath) + 3];
  char newpath[strlen(newrelpath) + 3];

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  oldpath[0] = '0' + fs->pdrv;
  oldpath[1] = ':';
  oldpath[2] = '\0';
  newpath[0] = '0' + fs->pdrv;
  newpath[1] = ':';
  newpath[2] = '\0';
  return fatfs_convert_result(f_rename(strcat(oldpath, oldrelpath),
                                       strcat(newpath, newrelpath)));
}

/****************************************************************************
 * Name: fatfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int fatfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                      FAR struct stat *buf)
{
  FAR struct fatfs_mountpt_s *fs;
  char path[strlen(relpath) + 3];

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  path[0] = '0' + fs->pdrv;
  path[1] = ':';
  path[2] = '\0';
  return fatfs_stat_i(strcat(path, relpath), buf);
}

/****************************************************************************
 * Name: fatfs_chstat
 ****************************************************************************/

static int fatfs_chstat(FAR struct inode *mountpt, FAR const char *relpath,
                        FAR const struct stat *buf, int flags)
{
  FAR struct fatfs_mountpt_s *fs;
  char path[strlen(relpath) + 3];

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  path[0] = '0' + fs->pdrv;
  path[1] = ':';
  path[2] = '\0';
  return fatfs_chstat_i(strcat(path, relpath), buf, flags);
}

/****************************************************************************
 * Name: disk_status
 *
 * Description:
 *   Get drive status.
 *
 * Input Parameters:
 *   pdrv - Physical drive nmuber to identify the drive.
 *
 * Returned Value:
 *   The status of disk is returned.
 *
 ****************************************************************************/

DSTATUS disk_status(BYTE pdrv)
{
  DEBUGASSERT(pdrv < FF_VOLUMES);
  if (g_drv[pdrv])
    {
      return OK;
    }

  return STA_NOINIT;
}

/****************************************************************************
 * Name: disk_initialize
 *
 * Description:
 *   Initialize a drive
 *
 * Input Parameters:
 *   pdrv - Physical drive nmuber to identify the drive.
 *
 * Returned Value:
 *   The status of disk is returned.
 *
 ****************************************************************************/

DSTATUS disk_initialize(BYTE pdrv)
{
  return disk_status(pdrv);
}

/****************************************************************************
 * Name: disk_read
 *
 * Description:
 *   Read Sector(s)
 *
 * Input Parameters:
 *   pdrv   - Physical drive nmuber to identify the drive.
 *   buff   - Data buffer to store read data.
 *   sector - Start sector in LBA
 *   count  - Number of sectors to read
 *
 * Returned Value:
 *   RES_OK if the read was successfully; A other value (REX_*) is returned
 *   on any failure.
 *
 ****************************************************************************/

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
  FAR struct inode *drv;
  ssize_t size;

  DEBUGASSERT(pdrv < FF_VOLUMES);
  drv = g_drv[pdrv];
  size = drv->u.i_bops->read(drv, buff, sector, count);
  if (size != count)
    {
      ferr("Read failed: %zd\n", size);
      return RES_ERROR;
    }

  return RES_OK;
}

/****************************************************************************
 * Name: disk_write
 *
 * Description:
 *   Write Sector(s)
 *
 * Input Parameters:
 *   pdrv   - Physical drive nmuber to identify the drive.
 *   buff   - Data to be written
 *   sector - Start sector in LBA
 *   count  - Number of sectors to write
 *
 * Returned Value:
 *   RES_OK if the write was successfully; A other value (REX_*) is returned
 *   on any failure.
 *
 ****************************************************************************/

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
  FAR struct inode *drv;
  ssize_t size;

  DEBUGASSERT(pdrv < FF_VOLUMES);
  drv = g_drv[pdrv];
  size = drv->u.i_bops->write(drv, buff, sector, count);
  if (size != count)
    {
      ferr("Write failed: %zd\n", size);
      return RES_ERROR;
    }

  return RES_OK;
}

/****************************************************************************
 * Name: disk_ioctl
 *
 * Description:
 *   Control disk.
 *
 * Input Parameters:
 *   pdrv   - Physical drive nmuber to identify the drive.
 *   cmd    - Control code
 *   buff   - Buffer to send/receive control data
 *   count  - Number of sectors to write
 *
 * Returned Value:
 *   The status of disk is returned.
 *
 ****************************************************************************/

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  FAR struct inode *drv;
  int ret = -ENOTTY;

  DEBUGASSERT(pdrv < FF_VOLUMES);
  drv = g_drv[pdrv];
  switch (cmd)
    {
      case CTRL_SYNC:
        {
          ret = drv->u.i_bops->ioctl(drv, BIOC_FLUSH, 0);
          if (ret == -ENOTTY)
            {
              ret = 0;
            }
        }
        break;

      case GET_SECTOR_COUNT:
      case GET_SECTOR_SIZE:
      case GET_BLOCK_SIZE:
        {
          struct geometry geo;

          ret = drv->u.i_bops->geometry(drv, &geo);
          if (ret < 0)
            {
              ferr("ERROR: geometry failed: %d\n", ret);
              break;
            }

          if (cmd == GET_SECTOR_COUNT)
            {
              *(FAR LBA_t *)buff = geo.geo_nsectors;
            }
          else if (cmd == GET_SECTOR_SIZE)
            {
              *(FAR WORD *)buff = geo.geo_sectorsize;
            }
          else if (cmd == GET_BLOCK_SIZE)
            {
              *(FAR DWORD *)buff = geo.geo_sectorsize;
            }
        }
        break;
    };

  return ret < 0 ? RES_ERROR : RES_OK;
}

#if FF_FS_NORTC == 0
/****************************************************************************
 * Name: get_fattime
 *
 * Description:
 *   get time
 *
 ****************************************************************************/

DWORD get_fattime(void)
{
  struct tm gm;
  time_t now;
  WORD fdate;
  WORD ftime;

  now = time(NULL);
  gmtime_r(&now, &gm);

  fdate = ((gm.tm_year - 80) << 9) | ((gm.tm_mon + 1) << 5) | gm.tm_mday;
  ftime = (gm.tm_hour << 11) | (gm.tm_min << 5) | (gm.tm_sec >> 1);
  return (fdate << 16) | ftime;
}
#endif

#if FF_USE_LFN == 3
/****************************************************************************
 * Name: ff_memalloc
 *
 * Description:
 *   Allocate a memory block
 *
 ****************************************************************************/

void *ff_memalloc(UINT msize)
{
  return kmm_malloc(msize);
}

/****************************************************************************
 * Name: ff_memfree
 *
 * Description:
 *   Free a memory block
 *
 ****************************************************************************/

void ff_memfree(void *mblock)
{
  kmm_free(mblock);
}
#endif

#if FF_FS_REENTRANT
/****************************************************************************
 * Name: ff_cre_syncobj
 *
 * Description:
 *   Create a Synchronization Object
 *   This function is called in f_mount() function to create a new
 *   synchronization object for the volume, such as semaphore and mutex.
 *   When a 0 is returned, the f_mount() function fails with FR_INT_ERR.
 *
 ****************************************************************************/

int ff_cre_syncobj(BYTE vol, FF_SYNC_t *sobj)
{
  return !nxsem_init(sobj, 0, 1);
}

/****************************************************************************
 * Name: ff_del_syncobj
 *
 * Description:
 *   Delete a Synchronization Object
 *   This function is called in f_mount() function to delete a
 *   synchronization object that created with ff_cre_syncobj() function.
 *   When a 0 is returned, the f_mount() function fails with FR_INT_ERR.
 *
 ****************************************************************************/

int ff_del_syncobj(FF_SYNC_t sobj)
{
  return !nxsem_destroy(&sobj);
}

/****************************************************************************
 * Name: ff_req_grant
 *
 * Description:
 *   Request Grant to Access the Volume.
 *   This function is called on entering file functions to lock the volume.
 *   When a 0 is returned, the file function fails.
 *   When a 1 is returned, got a grant to access the volume.
 *
 ****************************************************************************/

int ff_req_grant(FF_SYNC_t sobj)
{
  return !nxsem_wait_uninterruptible(&sobj);
}

/****************************************************************************
 * Name: ff_rel_grant
 *
 * Description:
 *   Release Grant to Access the Volume.
 *   This function is called on leaving file functions to unlock the volume.
 *
 ****************************************************************************/

void ff_rel_grant(FF_SYNC_t sobj)
{
  nxsem_post(&sobj);
}
#endif
