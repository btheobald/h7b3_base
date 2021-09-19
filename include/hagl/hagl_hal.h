/*
MIT License
Copyright (c) 2018-2021 Mika Tuupola
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-cut-
This file is part of the GD HAL for the HAGL graphics library:
https://github.com/tuupola/hagl_gd
SPDX-License-Identifier: MIT
*/

#ifndef _HAGL_GD_HAL_H
#define _HAGL_GD_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <bitmap.h>

/* HAL must provide display dimensions and depth. */
#ifndef DISPLAY_WIDTH
#define DISPLAY_WIDTH   (480)
#endif
#ifndef DISPLAY_HEIGHT
#define DISPLAY_HEIGHT  (272)
#endif
#define DISPLAY_DEPTH   (8)

/* These are the optional features this HAL provides. */
#define HAGL_HAS_HAL_COLOR
//#define HAGL_HAS_HAL_FILL_RECTANGLE
//#define HAGL_HAS_HAL_BLIT

/** HAL must provide typedef for colors. This HAL uses RGB888. */
typedef uint8_t color_t;

/**
 * @brief Draw a single pixel
 *
 * This is the only mandatory function HAL must provide.
 *
 * @param x0 X coordinate
 * @param y0 Y coorginate
 * @param color color
 */
void hagl_hal_put_pixel(int16_t x0, int16_t y0, color_t color);

//void hagl_hal_fill_rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, color_t color);

//void hagl_hal_blit(int16_t x0, int16_t y0, bitmap_t *source);

/**
 * @brief Convert RGB to HAL color type
 *
 * This is used for HAL implementations which use some other pixel
 * format than RGB332.
 */
static inline color_t hagl_hal_color(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t r3 = ((r >> 4) & 0b00000110) | (r & 0b00000001);
    uint8_t g3 = ((g >> 4) & 0b00000110) | (g & 0b00000001);
    uint8_t b3 = ((b >> 4) & 0b00000110) | (b & 0b00000001);
    return (r3 << 5) | (g3 << 2) | (b3);
}

#ifdef __cplusplus
}
#endif
#endif /* _HAGL_GD_HAL_H */
