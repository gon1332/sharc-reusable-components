/**
 * Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#include <string.h>

#include "fs_dev_adi_modes.h"

#include "fs_devman_cfg.h"
#include "fs_devman_priv.h"
#include "fs_devman.h"

#ifdef FS_DEVMAN_ENABLE_ROMFS

#include "romfs.h"
#include "romfs_devman.h"

static const char *dev_romfs_strip_path(const char *path)
{
   /* Lua looks for modules starting with a './' path */
   if ((strlen(path) > 2) && (strncmp(path, "./", 2) == 0)) {
      path += 2;
   }

   /* Strip leading slash */
   if ((strlen(path) > 1) && (strncmp(path, "/", 1) == 0)) {
      path += 1;
   }

   return(path);
}

static int dev_romfs_open(const char *path, int flags, int mode, void *pdata)
{
   int fd;
   int romfsFlags;

   romfsFlags = 0;

   if (mode & ADI_READ) romfsFlags |= O_RDONLY;
   if (mode & ADI_WRITE) romfsFlags |= O_WRONLY;
   if (mode & ADI_APPEND) romfsFlags |= O_APPEND;
   if (mode & ADI_CREAT) romfsFlags |= O_CREAT;
   if (mode & ADI_TRUNC) romfsFlags |= O_TRUNC;

   /* Lua looks for modules starting with a './' path */
   if ((strlen(path) > 2) && (strncmp(path, "./", 2) == 0)) {
      path += 2;
   }

   fd = fs_open(path, romfsFlags);

   return(fd);
}

static int dev_romfs_close(int fd, void *pdata)
{
    return(fs_close(fd));
}

static ssize_t dev_romfs_read(int fd, void *ptr, size_t len, void *pdata)
{
    return((ssize_t)fs_read(fd, (unsigned char *)ptr, (int)len));
}

static ssize_t dev_romfs_write(int fd, const void *ptr, size_t len, void *pdata)
{
    return((ssize_t)fs_write(fd, (unsigned char *)ptr, (int)len));
}

static off_t dev_romfs_lseek(int fd, off_t off, int whence, void *pdata)
{
    return((long)fs_seek(fd, (long)off, (int)whence));
}

static void *dev_romfs_opendir(const char *name, void *pdata)
{
    FS_DEVMAN_DIR *ddir = NULL;
    void *dir;

    dir = (void *)dm_opendir(name);
    if (dir) {
        ddir = FS_DEVMAN_CALLOC(1, sizeof(*ddir));
        if (ddir) {
            ddir->dir = dir;
        }
    }

    return(ddir);
}

static FS_DEVMAN_DIRENT *dev_romfs_readdir(void *dir, void *pdata)
{
    FS_DEVMAN_DIR *ddir = (FS_DEVMAN_DIR *)dir;
    struct dm_dirent *romfsDirent;
    FS_DEVMAN_DIRENT *dirent = NULL;
    size_t  size;

    romfsDirent = dm_readdir(ddir->dir);
    if (romfsDirent) {
        dirent = &ddir->dirent;
        dirent->fsize = romfsDirent->fsize;
        dirent->ftime = romfsDirent->ftime;
        dirent->fdate = 0;
        dirent->flags = 0;
        if (romfsDirent->flags & DM_DIRENT_FLAG_DIR) {
            dirent->flags |= FS_DEVMAN_DIRENT_FLAG_DIR;
        }
        size = strlen(romfsDirent->fname)+1;
        if (dirent->fname) {
            FS_DEVMAN_FREE((void *)dirent->fname);
        }
        dirent->fname = FS_DEVMAN_CALLOC(size, sizeof(*dirent->fname));
        if (dirent->fname) {
            memcpy((void *)dirent->fname, romfsDirent->fname, size);
        } else {
            dirent = NULL;
        }
    }

    return(dirent);
}

static int dev_romfs_closedir(void *dir, void *pdata)
{
    FS_DEVMAN_DIR *ddir = (FS_DEVMAN_DIR *)dir;
    int result;

    result = dm_closedir(ddir->dir);
    if (ddir->dirent.fname) {
        FS_DEVMAN_FREE((void *)ddir->dirent.fname);
    }
    FS_DEVMAN_FREE(ddir);

    return(result);
}

static int dev_romfs_unlink(const char *fname, void *pdata)
{
    int result;

    fname = dev_romfs_strip_path(fname);

    result = dm_unlink(fname);
    if (result == FS_FILE_OK) {
        result = 0;
    } else {
        result = -1;
    }

    return(result);
}

int dev_romfs_stat(const char* fname, FS_DEVMAN_STAT *stat, void *pdata)
{
    int result = FS_ERROR;
    DM_STAT s;

    fname = dev_romfs_strip_path(fname);

    result = fs_stat(fname, &s);
    if (result == FS_FILE_OK) {
        stat->fsize = s.fsize;
        stat->ftime = 0;
        stat->fdate = 1 << 5;
        stat->flags = 0;
    }

    return((result == FS_FILE_OK) ? 0 : -1);
}

static FS_DEVMAN_DEVICE FS_DEV_ROMFS = {
  .fsd_open = dev_romfs_open,
  .fsd_close = dev_romfs_close,
  .fsd_read = dev_romfs_read,
  .fsd_write = dev_romfs_write,
  .fsd_lseek = dev_romfs_lseek,
  .fsd_opendir = dev_romfs_opendir,
  .fsd_readdir = dev_romfs_readdir,
  .fsd_closedir = dev_romfs_closedir,
  .fsd_unlink = dev_romfs_unlink,
  .fsd_stat = dev_romfs_stat,
  .fsd_rename = NULL
};

FS_DEVMAN_DEVICE *fs_dev_romfs_device(void)
{
    return(&FS_DEV_ROMFS);
}

#endif
