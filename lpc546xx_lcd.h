/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef __LPC546XX_LCD_H__
#define __LPC546XX_LCD_H__

#include "fsl_lcdc.h"
#include "fsl_sctimer.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"
#include "board.h"
#include "pin_mux.h"
#include <stdio.h>
#include <string.h>
#include "stdio_thread.h"

#include "image.h"
#include "mpl_lcd.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_LCD LCD
#define LCD_PANEL_CLK 9000000
#define LCD_PPL 480
#define LCD_HSW 2
#define LCD_HFP 8
#define LCD_HBP 43
#define LCD_LPP 272
#define LCD_VSW 10
#define LCD_VFP 4
#define LCD_VBP 12
#define LCD_POL_FLAGS kLCDC_InvertVsyncPolarity | kLCDC_InvertHsyncPolarity
#define IMG_HEIGHT 272
#define IMG_WIDTH 480
#define LCD_INPUT_CLK_FREQ CLOCK_GetFreq(kCLOCK_LCD)
#define APP_LCD_IRQHandler LCD_IRQHandler
#define APP_LCD_IRQn LCD_IRQn
#define EXAMPLE_I2C_MASTER_BASE (I2C2_BASE)
#define I2C_MASTER_CLOCK_FREQUENCY (12000000)
#define APP_PIXEL_PER_BYTE 8

#define EXAMPLE_I2C_MASTER ((I2C_Type *)EXAMPLE_I2C_MASTER_BASE)
#define I2C_MASTER_SLAVE_ADDR_7BIT 0x7EU
#define I2C_BAUDRATE 100000U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

#if (defined(__CC_ARM) || defined(__GNUC__))
__attribute__((aligned(8)))
#elif defined(__ICCARM__)
#pragma data_alignment = 8
#else
#error Toolchain not support.
#endif

//void *vram_ptr = s_frameBufs;

/* Frame end flag. */
static volatile bool s_frameEndFlag;

/* Color palette. */
static const uint32_t palette[] = { 0x001F00000 }; //{0x0000001F}; //{0x001F0000U, 0x7C0003E0U};

/* 32x32 pixel cursor image. */
#if (defined(__CC_ARM) || defined(__GNUC__))
__attribute__((aligned(4)))
#elif defined(__ICCARM__)
#pragma data_alignment = 4
#else
#error Toolchain not support.
#endif



/*
class lpc546xx_lcd

constructor

destructor

clearImage

showImage
*/

#endif // __IMAGE_H__
