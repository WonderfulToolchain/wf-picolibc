/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright © 2019 Keith Packard
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio-bufio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

/* Buffered I/O routines for tiny stdio */

static int
__bufio_flush_locked(FILE *f)
{
	struct __file_bufio *bf = (struct __file_bufio *) f;
        char *buf;
        int ret = 0;
        off_t backup;

        switch (bf->dir) {
        case __SWR:
		/* Flush everything, drop contents if that doesn't work */
                buf = bf->buf;
		while (bf->len) {
                        ssize_t this = (bf->write) (bf->fd, buf, bf->len);
			if (this <= 0) {
                                bf->len = 0;
                                ret = -1;
                                break;
			}
			bf->pos += this;
			bf->len -= this;
		}
                break;
        case __SRD:
                /* Move the FD back to the current read position */
                backup = bf->len - bf->off;
                if (backup) {
                        bf->pos -= backup;
                        if (bf->lseek)
                                (void) (bf->lseek)(bf->fd, bf->pos, SEEK_SET);
                }
                bf->len = 0;
                bf->off = 0;
                break;
        default:
                break;
	}
	return ret;
}

int
__bufio_flush(FILE *f)
{
        int ret;

	__bufio_lock(f);
        ret = __bufio_flush_locked(f);
	__bufio_unlock(f);
	return ret;
}

/* Set I/O direction, flushing when it changes */
static int
__bufio_setdir_locked(FILE *f, uint8_t dir)
{
	struct __file_bufio *bf = (struct __file_bufio *) f;
        int ret = 0;

        if (bf->dir != dir) {
                ret = __bufio_flush_locked(f);
                bf->dir = dir;
        }
        return ret;
}

int
__bufio_put(char c, FILE *f)
{
	struct __file_bufio *bf = (struct __file_bufio *) f;
        int ret = (unsigned char) c;

	__bufio_lock(f);
        if (__bufio_setdir_locked(f, __SWR) < 0) {
                ret = _FDEV_ERR;
                goto bail;
        }

	bf->buf[bf->len++] = c;

	/* flush if full, or if sending newline when linebuffered */
	if (bf->len >= bf->size || (c == '\n' && (bf->bflags & __BLBF)))
		if (__bufio_flush_locked(f) < 0)
                        ret = _FDEV_ERR;

bail:
	__bufio_unlock(f);
	return ret;
}

int
__bufio_write(const void *buf, size_t len, FILE *f)
{
	struct __file_bufio *bf = (struct __file_bufio *) f;
        int ret = len;
        const uint8_t *bc = buf;
        size_t i;
        size_t len_step;

	__bufio_lock(f);
        if (__bufio_setdir_locked(f, __SWR) < 0) {
                ret = _FDEV_ERR;
                goto bail;
        }

        if (bf->bflags & __BLBF) {
                /* line-buffered - use slow code path */
                for (i = 0; i < len; i++) {
                        bf->buf[bf->len++] = bc[i];

                        /* flush if full, or if sending newline */
                        if (bf->len >= bf->size || bc[i] == '\n')
                                if (__bufio_flush_locked(f) < 0) {
                                        ret = _FDEV_ERR;
                                        break;
                                }
                }
        } else {
                while (len > 0) {
                        /* If the length > the buffer size, write directly. */
                        if (len >= bf->size && bf->len == 0) {
                                len_step = len - (len % bf->size);
                                len_step = (bf->write)(bf->fd, bc, len_step);
                                if (len_step <= 0) {
                                        ret = _FDEV_EOF;
                                        goto bail;
                                }

                                bf->pos += len_step;
                        } else {
                                len_step = bf->size - bf->len;
                                if (len_step > len)
                                        len_step = len;
                                
                                memcpy(bf->buf + bf->len, bc, len_step);

                                bf->len += len_step;

                                if (bf->len >= bf->size)
                                        if (__bufio_flush_locked(f) < 0) {
                                                ret = _FDEV_ERR;
                                                goto bail;
                                        }
                        }
        
                        bc += len_step;
                        len -= len_step;
                }
        }

bail:
	__bufio_unlock(f);
	return ret;
}

int
__bufio_get(FILE *f)
{
	struct __file_bufio *bf = (struct __file_bufio *) f;
        int ret;
        bool flushed = false;

again:
	__bufio_lock(f);
        if (__bufio_setdir_locked(f, __SRD) < 0) {
                ret = _FDEV_ERR;
                goto bail;
        }

	if (bf->off >= bf->len) {

		/* Flush stdout if reading from stdin */
		if (f == stdin && !flushed) {
                        flushed = true;
			__bufio_unlock(f);
			fflush(stdout);
                        goto again;
		}

		/* Reset read pointer, read some data */
		bf->off = 0;
		bf->len = (bf->read)(bf->fd, bf->buf, bf->size);

		if (bf->len <= 0) {
			bf->len = 0;
                        ret = _FDEV_EOF;
                        goto bail;
		}

                /* Update FD pos */
                bf->pos += bf->len;
	}

	/*
	 * Cast to unsigned avoids sign-extending chars with high-bit
	 * set
	 */
	ret = (unsigned char) bf->buf[bf->off++];
bail:
	__bufio_unlock(f);
	return ret;
}

int
__bufio_read(void *buf, size_t len, FILE *f)
{
	struct __file_bufio *bf = (struct __file_bufio *) f;
        int ret = len;
        bool flushed = false;
        char *bc = buf;
        size_t len_step;

again:
	__bufio_lock(f);
        if (__bufio_setdir_locked(f, __SRD) < 0) {
                ret = _FDEV_ERR;
                goto bail;
        }

        while (len > 0) {
                /* TODO: Share this code with __bufio_get */
                if (bf->off >= bf->len) {

                        /* Flush stdout if reading from stdin */
                        if (f == stdin && !flushed) {
                                flushed = true;
                                __bufio_unlock(f);
                                fflush(stdout);
                                goto again;
                        }

                        /* Reset read pointer */
                        bf->off = 0;
                        bf->len = 0;

                        /* If the length > the buffer size, read directly. */
                        if (len >= bf->size)
                        {
                                len_step = len - (len % bf->size);
                                len_step = (bf->read)(bf->fd, bc, len_step);
                                if (len_step <= 0) {
                                        ret = _FDEV_EOF;
                                        goto bail;
                                }

                                bf->pos += len_step;
                                bc += len_step;
                                len -= len_step;
                                continue;
                        }

                        if (bf->lseek) {
                                /* Word-align data */
                                bf->off = bf->pos & 3;
                                bf->pos -= bf->off;
                                if (bf->off)
                                        (bf->lseek)(bf->fd, bf->pos, SEEK_SET);
                        }
                        /* Read some data */
                        bf->len = (bf->read)(bf->fd, bf->buf, bf->size);

                        if (bf->len <= 0) {
                                bf->len = 0;
                                ret = _FDEV_EOF;
                                goto bail;
                        }

                        /* Update FD pos */
                        bf->pos += bf->len;
                }

                len_step = bf->len - bf->off;
                if (len < len_step)
                        len_step = len;
                memcpy(bc, bf->buf + bf->off, len_step);
                bf->off += len_step;
                bc += len_step;
                len -= len_step;
        }
bail:
	__bufio_unlock(f);
	return ret;
}

off_t
__bufio_seek(FILE *f, off_t offset, int whence)
{
	struct __file_bufio *bf = (struct __file_bufio *) f;
	off_t ret;
        off_t pos_start;

	__bufio_lock(f);
        if (!bf->lseek) {
                ret = _FDEV_ERR;
        } else {
                if (bf->dir == __SRD) {
                        pos_start = bf->pos - bf->len;
                        if (whence == SEEK_CUR) {
                                whence = SEEK_SET;
                                offset += pos_start + bf->off;
                        }

                        if (offset >= pos_start && (offset - pos_start) < bf->len) {
                                // If we're reading within the scope of the buffer, we can
                                // seek locally without invoking the underlying file.
                                bf->off = offset - pos_start;
                                ret = offset;
                                goto done;
                        }
                }

                if (__bufio_setdir_locked(f, 0) < 0)
                        ret = _FDEV_ERR;
                else {
                        if (whence == SEEK_CUR) {
                                whence = SEEK_SET;
                                offset += bf->pos;
                        }
                        ret = (bf->lseek)(bf->fd, offset, whence);
                        bf->pos = ret;
                }
        }
done:
        __bufio_unlock(f);
        return ret;
}

int
__bufio_setvbuf(FILE *f, char *buf, int mode, size_t size)
{
	struct __file_bufio *bf = (struct __file_bufio *) f;
        int ret = -1;

	__bufio_lock(f);
        bf->bflags &= ~__BLBF;
        switch (mode) {
        case _IONBF:
                buf = NULL;
                size = 1;
                break;
        case _IOLBF:
                bf->bflags |= __BLBF;
                break;
        case _IOFBF:
                break;
        default:
                goto bail;
        }
        if (bf->bflags & __BALL) {
                if (buf) {
                        free(bf->buf);
                        bf->bflags &= ~__BALL;
                } else {
                        /*
                         * Handling allocation failures here is a bit tricky;
                         * we don't want to lose the existing buffer. Instead,
                         * we try to reallocate it
                         */
                        buf = realloc(bf->buf, size);
                        if (!buf)
                                goto bail;
                }
        } else if (!buf) {
                buf = malloc(size);
                if (!buf)
                        goto bail;
                bf->bflags |= __BALL;
        }
        bf->buf = buf;
        bf->size = size;
        ret = 0;
bail:
        __bufio_unlock(f);
        return ret;
}

int
__bufio_close(FILE *f)
{
	struct __file_bufio *bf = (struct __file_bufio *) f;
	int ret = 0;

	__bufio_lock(f);
        ret = __bufio_flush_locked(f);

        if (bf->bflags & __BALL)
                free(bf->buf);

	__bufio_lock_close(f);
	/* Don't close stdin/stdout/stderr fds */
	if (bf->fd > 2)
		(bf->close)(bf->fd);
	free(f);
	return ret;
}

