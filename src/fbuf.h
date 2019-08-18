/*
 * Copyright (c) 2003 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef FBUF_H
#define FBUF_H

typedef struct {
#if defined(EMSCRIPTEN)
    int handle;
#else
    uint8_t *data;
#endif
    size_t allocated_size;
} FileBuffer;

void file_buffer_init(FileBuffer *bs);
void file_buffer_reset(FileBuffer *bs);
int file_buffer_resize(FileBuffer *bs, size_t new_size);
void file_buffer_write(FileBuffer *bs, size_t offset, const uint8_t *buf,
                       size_t size);
void file_buffer_set(FileBuffer *bs, size_t offset, int val, size_t size);
void file_buffer_read(FileBuffer *bs, size_t offset, uint8_t *buf,
                      size_t size);

#endif /* FBUF_H */
