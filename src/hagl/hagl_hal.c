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

#include <stdint.h>

#include "main.h"
#include "hagl_hal.h"

//MDMA_HandleTypeDef hmdma_mdma_channel40_sw_0;

void hagl_hal_put_pixel(int16_t x0, int16_t y0, color_t color)
{
    framebuffer_l8[x0 + y0*DISPLAY_WIDTH] = color;
}

/*void hagl_hal_fill_rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, color_t color)
{
    int width = x1-x0+1;
    int height = y1-y0+1;
      hmdma_mdma_channel40_sw_0.Instance = MDMA_Channel0;
  hmdma_mdma_channel40_sw_0.Init.Request = MDMA_REQUEST_SW;
  hmdma_mdma_channel40_sw_0.Init.TransferTriggerMode = MDMA_REPEAT_BLOCK_TRANSFER;
  hmdma_mdma_channel40_sw_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel40_sw_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel40_sw_0.Init.SourceInc = MDMA_SRC_INC_DISABLE;
  hmdma_mdma_channel40_sw_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel40_sw_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel40_sw_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel40_sw_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel40_sw_0.Init.BufferTransferLength = 1;
  hmdma_mdma_channel40_sw_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel40_sw_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel40_sw_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel40_sw_0.Init.DestBlockAddressOffset = (DISPLAY_WIDTH - width);
  HAL_MDMA_Init(&hmdma_mdma_channel40_sw_0); 

  HAL_MDMA_Start(&hmdma_mdma_channel40_sw_0, (uint32_t*)&color, (uint32_t)fb + ((x0 + y0 * DISPLAY_WIDTH)), width, height);
  HAL_MDMA_PollForTransfer(&hmdma_mdma_channel40_sw_0, MDMA_REPEAT_BLOCK_TRANSFER, 10);
}

void hagl_hal_blit(int16_t x0, int16_t y0, bitmap_t *source)
{
    hmdma_mdma_channel40_sw_0.Instance = MDMA_Channel0;
    hmdma_mdma_channel40_sw_0.Init.Request = MDMA_REQUEST_SW;
    hmdma_mdma_channel40_sw_0.Init.TransferTriggerMode = MDMA_REPEAT_BLOCK_TRANSFER;
    hmdma_mdma_channel40_sw_0.Init.Priority = MDMA_PRIORITY_LOW;
    hmdma_mdma_channel40_sw_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
    hmdma_mdma_channel40_sw_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
    hmdma_mdma_channel40_sw_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
    hmdma_mdma_channel40_sw_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
    hmdma_mdma_channel40_sw_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
    hmdma_mdma_channel40_sw_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
    hmdma_mdma_channel40_sw_0.Init.BufferTransferLength = 1;
    hmdma_mdma_channel40_sw_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
    hmdma_mdma_channel40_sw_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
    hmdma_mdma_channel40_sw_0.Init.SourceBlockAddressOffset = 0;
    hmdma_mdma_channel40_sw_0.Init.DestBlockAddressOffset = (DISPLAY_WIDTH - source->width);
    HAL_MDMA_Init(&hmdma_mdma_channel40_sw_0); 

    HAL_MDMA_Start(&hmdma_mdma_channel40_sw_0, source->buffer, (uint32_t)fb + ((x0 + y0 * DISPLAY_WIDTH)), source->width, source->height);
    HAL_MDMA_PollForTransfer(&hmdma_mdma_channel40_sw_0, MDMA_REPEAT_BLOCK_TRANSFER, 10);
}*/