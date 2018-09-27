#ifndef __LPC546XX_LCD_H__
#define __LPC546XX_LCD_H__

#include "fsl_lcdc.h"
#include "fsl_sctimer.h"
#include "fsl_gpio.h"
#include "board.h"
#include "pin_mux.h"

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


class LPC546XX_LCD {
public:
    LPC546XX_LCD(uint8_t* framebuf);
    void on(void);
    void off(void);
    void update(uint8_t* framebuf);
    
private:  
  #if (defined(__CC_ARM) || defined(__GNUC__))
  __attribute__((aligned(8)))
  #elif defined(__ICCARM__)
  #pragma data_alignment = 8
  #else
  #error Toolchain not support.
  #endif
  /* Frame end flag. */
  volatile bool s_frameEndFlag;
  
  /* Color palette. */
  //uint32_t palette[];
  void APP_LCD_IRQHandler(void);
  void BOARD_InitPWM(void);

};

#endif
