/*
 * Copyright (c) 2016 Thomas Pornin <pornin@bolet.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining 
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _GHASH_H_
#define _GHASH_H_

#include <string.h>
#include <limits.h>
#include <stdint.h>

extern void br_ghash_ctmul32(void *y, const void *h, const void *data, size_t len);
extern void br_ghash_ctmul(void *y, const void *h, const void *data, size_t len);
extern void br_ghash_ctmul64(void *y, const void *h, const void *data, size_t len);

/* ==================================================================== */
/*
 * Encoding/decoding functions.
 *
 * 32-bit and 64-bit decoding, both little-endian and big-endian, is
 * implemented with the inline functions below. These functions are
 * generic: they don't depend on the architecture natural endianness,
 * and they can handle unaligned accesses. Optimized versions for some
 * specific architectures may be implemented at a later time.
 */

static inline void
br_enc16le(void *dst, unsigned x)
{
	unsigned char *buf;

	buf = dst;
	buf[0] = (unsigned char)x;
	buf[1] = (unsigned char)(x >> 8);
}

static inline void
br_enc16be(void *dst, unsigned x)
{
	unsigned char *buf;

	buf = dst;
	buf[0] = (unsigned char)(x >> 8);
	buf[1] = (unsigned char)x;
}

static inline unsigned
br_dec16le(const void *src)
{
	const unsigned char *buf;

	buf = src;
	return (unsigned)buf[0] | ((unsigned)buf[1] << 8);
}

static inline unsigned
br_dec16be(const void *src)
{
	const unsigned char *buf;

	buf = src;
	return ((unsigned)buf[0] << 8) | (unsigned)buf[1];
}

static inline void
br_enc32le(void *dst, uint32_t x)
{
	unsigned char *buf;

	buf = dst;
	buf[0] = (unsigned char)x;
	buf[1] = (unsigned char)(x >> 8);
	buf[2] = (unsigned char)(x >> 16);
	buf[3] = (unsigned char)(x >> 24);
}

static inline void
br_enc32be(void *dst, uint32_t x)
{
	unsigned char *buf;

	buf = dst;
	buf[0] = (unsigned char)(x >> 24);
	buf[1] = (unsigned char)(x >> 16);
	buf[2] = (unsigned char)(x >> 8);
	buf[3] = (unsigned char)x;
}

static inline uint32_t
br_dec32le(const void *src)
{
	const unsigned char *buf;

	buf = src;
	return (uint32_t)buf[0]
		| ((uint32_t)buf[1] << 8)
		| ((uint32_t)buf[2] << 16)
		| ((uint32_t)buf[3] << 24);
}

static inline uint32_t
br_dec32be(const void *src)
{
	const unsigned char *buf;

	buf = src;
	return ((uint32_t)buf[0] << 24)
		| ((uint32_t)buf[1] << 16)
		| ((uint32_t)buf[2] << 8)
		| (uint32_t)buf[3];
}

static inline void
br_enc64le(void *dst, uint64_t x)
{
	unsigned char *buf;

	buf = dst;
	br_enc32le(buf, (uint32_t)x);
	br_enc32le(buf + 4, (uint32_t)(x >> 32));
}

static inline void
br_enc64be(void *dst, uint64_t x)
{
	unsigned char *buf;

	buf = dst;
	br_enc32be(buf, (uint32_t)(x >> 32));
	br_enc32be(buf + 4, (uint32_t)x);
}

static inline uint64_t
br_dec64le(const void *src)
{
	const unsigned char *buf;

	buf = src;
	return (uint64_t)br_dec32le(buf)
		| ((uint64_t)br_dec32le(buf + 4) << 32);
}

static inline uint64_t
br_dec64be(const void *src)
{
	const unsigned char *buf;

	buf = src;
	return ((uint64_t)br_dec32be(buf) << 32)
		| (uint64_t)br_dec32be(buf + 4);
}

/* ==================================================================== */

#endif
