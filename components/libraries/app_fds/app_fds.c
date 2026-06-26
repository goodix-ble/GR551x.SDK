/**
 *****************************************************************************************
 *
 * @file app_fds.c
 *
 * @brief App Flash Data Storage Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_fds.h"

#define APP_FDS_VERIFY(ret)           \
do                                    \
{                                     \
    if (ret)                          \
    {                                 \
        return ret;                   \
    }                                 \
} while(0)

#define APP_FDS_CACHE_SIZE        512
#define APP_FDS_LOOKAHEAD_SIZE    16

typedef  lfs_t app_fds_instance_t;

extern app_fds_config_t s_app_fds_config;

static __attribute__ ((aligned (4))) uint8_t s_file_buffer[APP_FDS_CACHE_SIZE];
static __attribute__ ((aligned (4))) uint8_t s_read_buffer[APP_FDS_CACHE_SIZE];
static __attribute__ ((aligned (4))) uint8_t s_prog_buffer[APP_FDS_CACHE_SIZE];
static __attribute__ ((aligned (4))) uint8_t s_lookahead_buffer[APP_FDS_LOOKAHEAD_SIZE];

app_fds_instance_t s_default_instance;


static int app_fds_lock(const struct lfs_config *c);
static int app_fds_unlock(const struct lfs_config *c);
static int app_fds_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
static int app_fds_write(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
static int app_fds_erase(const struct lfs_config *c, lfs_block_t block);
static int app_fds_sync(const struct lfs_config *c);

struct lfs_config s_lfs_cfg = 
{
    .context            = NULL,
    .read               = app_fds_read,
    .prog               = app_fds_write,
    .erase              = app_fds_erase,
    .sync               = app_fds_sync,
    .lock               = app_fds_lock,
    .unlock             = app_fds_unlock,
    .read_size          = 16,
    .prog_size          = 16,
    .block_cycles       = 500,
    .cache_size         = APP_FDS_CACHE_SIZE,
    .lookahead_size     = APP_FDS_LOOKAHEAD_SIZE,
    .read_buffer        = s_read_buffer,
    .prog_buffer        = s_prog_buffer,
    .lookahead_buffer   = s_lookahead_buffer,
};


#if !APP_FDS_KEY_STRING_TYPE
static char     s_key_char[10] = {0};
#endif

#if !APP_FDS_KEY_STRING_TYPE
static uint32_t app_fds_char_key_convert(const char *str)
{
    uint32_t num = 0;
    const char *p = str;

    while (*p >= '0' && *p <= '9')
    {
        num = num * 10 + (*p - '0');
        p++;
    }

    return num;
}

static char * app_fds_int_key_convert(uint32_t key)
{
    uint16_t digit = 0;
    uint32_t temp = key;

    do
    {
        temp /= 10;
        digit++;
    } while (temp > 0);

    s_key_char[digit] = '\0';

    do {
        digit--;
        s_key_char[digit] = key % 10 + '0';
        key /= 10;
    } while (key > 0);

    return s_key_char;
}
#endif

static int app_fds_lock(const struct lfs_config *c)
{
    if (((app_fds_config_t *)c->context)->fds_lock)
    {
        return ((app_fds_config_t *)c->context)->fds_lock();
    }

    return APP_FDS_ERR_CORRUPT;
}

static int app_fds_unlock(const struct lfs_config *c)
{
    if (((app_fds_config_t *)c->context)->fds_unlock)
    {
        return ((app_fds_config_t *)c->context)->fds_unlock();
    }

    return APP_FDS_ERR_CORRUPT;
}

static int app_fds_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    if (((app_fds_config_t *)c->context)->fds_read)
    {
        return ((app_fds_config_t *)c->context)->fds_read(((app_fds_config_t *)c->context)->fds_start_addr + block * c->block_size + off, (uint8_t *)buffer, size);
    }

    return APP_FDS_ERR_CORRUPT;
}

static int app_fds_write(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    if (((app_fds_config_t *)c->context)->fds_write)
    {
        return ((app_fds_config_t *)c->context)->fds_write(((app_fds_config_t *)c->context)->fds_start_addr + block * c->block_size + off, (uint8_t *)buffer, size);
    }

    return APP_FDS_ERR_CORRUPT;
}

static int app_fds_erase(const struct lfs_config *c, lfs_block_t block)
{
    if (((app_fds_config_t *)c->context)->fds_erase)
    {
        return ((app_fds_config_t *)c->context)->fds_erase(((app_fds_config_t *)c->context)->fds_start_addr + block * c->block_size, c->block_size);
    }

    return APP_FDS_ERR_CORRUPT;
}
static int app_fds_sync(const struct lfs_config *c)
{
    if (((app_fds_config_t *)c->context)->fds_sync)
    {
        return((app_fds_config_t *)c->context)->fds_sync();
    }

    return APP_FDS_ERR_CORRUPT;
}

int app_fds_init(uint32_t fds_start_addr, uint32_t fds_block_cnt)
{
    int ret;

    if (fds_start_addr % 0x1000 || !fds_block_cnt)
    {
        return APP_FDS_ERR_INVAL;
    }
    s_app_fds_config.fds_start_addr   = fds_start_addr;
    s_app_fds_config.fds_4k_block_cnt = fds_block_cnt;

    s_lfs_cfg.block_count = fds_block_cnt;
    s_lfs_cfg.block_size  = 0x1000;
    s_lfs_cfg.context     = &s_app_fds_config;

    if (lfs_mount(&s_default_instance, &s_lfs_cfg))
    {
        ret = lfs_format(&s_default_instance, &s_lfs_cfg);
        APP_FDS_VERIFY(ret);
        ret = lfs_mount(&s_default_instance, &s_lfs_cfg);
        APP_FDS_VERIFY(ret);
    }

    return APP_FDS_ERR_OK;
}

int app_fds_value_write(
#if  APP_FDS_KEY_STRING_TYPE
                              char *p_key,
#else
                              uint32_t key,
#endif
                              const void *p_value,
                              uint32_t length)
{
    int ret;

    lfs_file_t              file_id;
    struct lfs_file_config  file_config = {0};

    file_config.buffer = s_file_buffer;
 
#if  APP_FDS_KEY_STRING_TYPE
    ret = lfs_file_opencfg(&s_default_instance, &file_id, p_key, LFS_O_RDWR | LFS_O_CREAT, &file_config);
    APP_FDS_VERIFY(ret);
#else
    ret = lfs_file_opencfg(&s_default_instance, &file_id, app_fds_int_key_convert(key), LFS_O_RDWR | LFS_O_CREAT, &file_config);
    APP_FDS_VERIFY(ret);
#endif

    ret = lfs_file_rewind(&s_default_instance, &file_id);
    if (ret)
    {
        lfs_file_close(&s_default_instance, &file_id);
        return ret;
    }

    ret = lfs_file_write(&s_default_instance, &file_id, p_value, length);
    if (ret)
    {
        lfs_file_truncate(&s_default_instance, &file_id, length);
        lfs_file_close(&s_default_instance, &file_id);
        return ret;
    }

    ret = lfs_file_close(&s_default_instance, &file_id);
    APP_FDS_VERIFY(ret);

    return 0;
}

int app_fds_value_read(
#if  APP_FDS_KEY_STRING_TYPE
                             char *p_key,
#else
                             uint32_t key,
#endif
                             void *p_buffer,
                             uint32_t length)
{
    int ret;

    lfs_file_t              file_id;
    struct lfs_file_config  file_config = {0};

    file_config.buffer = s_file_buffer;
 
#if  APP_FDS_KEY_STRING_TYPE
    ret = lfs_file_opencfg(&s_default_instance, &file_id, p_key, LFS_O_RDWR | LFS_O_CREAT, &file_config);
    APP_FDS_VERIFY(ret);
#else
    ret = lfs_file_opencfg(&s_default_instance, &file_id, app_fds_int_key_convert(key), LFS_O_RDONLY, &file_config);
    APP_FDS_VERIFY(ret);
#endif
    ret = lfs_file_rewind(&s_default_instance, &file_id);
    if (ret)
    {
        lfs_file_close(&s_default_instance, &file_id);
        return ret;
    }

    ret = lfs_file_read(&s_default_instance, &file_id, p_buffer, length);
    if (ret)
    {
        lfs_file_close(&s_default_instance, &file_id);
        return ret;
    }

    ret = lfs_file_close(&s_default_instance, &file_id);
    APP_FDS_VERIFY(ret);

    return 0;
}


int app_fds_value_delete(
#if  APP_FDS_KEY_STRING_TYPE
                               char *p_key)
#else
                               uint32_t key)
#endif
{
#if  APP_FDS_KEY_STRING_TYPE
    return lfs_remove(&s_default_instance, p_key);
#else
     return lfs_remove(&s_default_instance, app_fds_int_key_convert(key));
#endif
}


int app_fds_traverse(app_fds_traverse_cb_t traverse_cb)
{
    int ret;
    lfs_dir_t dir;
    struct lfs_info info;

    ret =lfs_dir_open(&s_default_instance, &dir, ".");
    APP_FDS_VERIFY(ret);

    bool is_continue;

    while(1)
    {
        ret =  lfs_dir_read(&s_default_instance, &dir, &info);
        if (ret <= APP_FDS_ERR_OK)
        {
            return lfs_dir_close(&s_default_instance, &dir);
        }

        if (LFS_TYPE_REG == info.type && traverse_cb)
        {
#if  APP_FDS_KEY_STRING_TYPE
            traverse_cb(info.name, info.size, &is_continue);
#else
            traverse_cb(app_fds_char_key_convert(info.name), info.size, &is_continue);
#endif
            if (!is_continue)
            {
                lfs_dir_close(&s_default_instance, &dir);
                return APP_FDS_ERR_OK;
            }
        }
    };
}


