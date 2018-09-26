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
#include "lpc546xx_lcd.h"


/*******************************************************************************
 * Code
 ******************************************************************************/
static void BOARD_InitPWM(void)
{
    sctimer_config_t config;
    sctimer_pwm_signal_param_t pwmParam;
    uint32_t event;

    CLOCK_AttachClk(kMCLK_to_SCT_CLK);

    CLOCK_SetClkDiv(kCLOCK_DivSctClk, 2, true);

    SCTIMER_GetDefaultConfig(&config);

    SCTIMER_Init(SCT0, &config);

    pwmParam.output = kSCTIMER_Out_5;
    pwmParam.level = kSCTIMER_HighTrue;
    pwmParam.dutyCyclePercent = 5;

    SCTIMER_SetupPwm(SCT0, &pwmParam, kSCTIMER_CenterAlignedPwm, 1000U, CLOCK_GetFreq(kCLOCK_Sct), &event);
}

void APP_LCD_IRQHandler(void)
{
    uint32_t intStatus = LCDC_GetEnabledInterruptsPendingStatus(APP_LCD);

    LCDC_ClearInterruptsStatus(APP_LCD, intStatus);

    if (intStatus & kLCDC_VerticalCompareInterrupt)
    {
        s_frameEndFlag = true;
    }
    __DSB();
}

status_t APP_LCDC_Init(void)
{
    /* Initialize the display. */
    lcdc_config_t lcdConfig;
    //lcdc_cursor_config_t cursorConfig;

    LCDC_GetDefaultConfig(&lcdConfig);

    lcdConfig.panelClock_Hz = LCD_PANEL_CLK;
    lcdConfig.ppl = LCD_PPL;
    lcdConfig.hsw = LCD_HSW;
    lcdConfig.hfp = LCD_HFP;
    lcdConfig.hbp = LCD_HBP;
    lcdConfig.lpp = LCD_LPP;
    lcdConfig.vsw = LCD_VSW;
    lcdConfig.vfp = LCD_VFP;
    lcdConfig.vbp = LCD_VBP;
    lcdConfig.polarityFlags = LCD_POL_FLAGS;
    lcdConfig.upperPanelAddr = (uint32_t)s_frameBufs;
    lcdConfig.bpp = kLCDC_1BPP;
    lcdConfig.display = kLCDC_DisplayTFT;
    lcdConfig.swapRedBlue = true;  //false;
    lcdConfig.dataFormat = kLCDC_WinCeMode;

    LCDC_Init(APP_LCD, &lcdConfig, LCD_INPUT_CLK_FREQ);

    LCDC_SetPalette(APP_LCD, palette, ARRAY_SIZE(palette));

    /* Setup the Cursor. */
  /*
    LCDC_CursorGetDefaultConfig(&cursorConfig);

    cursorConfig.size = kLCDC_CursorSize32;
    cursorConfig.syncMode = kLCDC_CursorSync;
    cursorConfig.image[0] = (uint32_t *)cursor32Img0;

    LCDC_SetCursorConfig(APP_LCD, &cursorConfig);
    LCDC_ChooseCursor(APP_LCD, 0);
*/
    /* Trigger interrupt at start of every vertical back porch. */
    LCDC_SetVerticalInterruptMode(APP_LCD, kLCDC_StartOfBackPorch);
    LCDC_EnableInterrupts(APP_LCD, kLCDC_VerticalCompareInterrupt);
    // NVIC_EnableIRQ(APP_LCD_IRQn);

//    LCDC_EnableCursor(APP_LCD, true);

    LCDC_Start(APP_LCD);
    LCDC_PowerUp(APP_LCD);

    return kStatus_Success;
}

status_t APP_I2C_Init(void)
{
    i2c_master_config_t masterConfig;

    I2C_MasterGetDefaultConfig(&masterConfig);

    /* Change the default baudrate configuration */
    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    /* Initialize the I2C master peripheral */
    I2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, I2C_MASTER_CLOCK_FREQUENCY);

    return kStatus_Success;
}
/*
void APP_SetCursorPosition(int posX, int posY)
{
    posX -= 12;
    posY -= 5;

    LCDC_SetCursorPosition(APP_LCD, posX, posY);
}
*/
void LCD_Init(void)
{
  status_t status;

  gpio_pin_config_t pin_config = {
      kGPIO_DigitalOutput, 0,
  };

  /* Board pin, clock, debug console init */
  /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
  CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

  /* Route Main clock to LCD. */
  CLOCK_AttachClk(kMCLK_to_LCD_CLK);

  /* attach 12 MHz clock to FLEXCOMM2 (I2C master for touch controller) */
  CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);

  CLOCK_EnableClock(kCLOCK_Gpio2);

  CLOCK_SetClkDiv(kCLOCK_DivLcdClk, 1, true);

  BOARD_InitPins();
  BOARD_BootClockFROHF48M();
  // BOARD_InitDebugConsole();

  /* Set the back light PWM. */
  BOARD_InitPWM();

  // APP_FillBuffer((void *)(s_frameBufs));

  status = APP_LCDC_Init();
  if (status != kStatus_Success)
  {
      safe_printf("LCD init failed\n");
  }
  assert(status == kStatus_Success);

  status = APP_I2C_Init();
  if (status != kStatus_Success)
  {
      safe_printf("I2C init failed\n");
  }
  assert(status == kStatus_Success);

  GPIO_PinInit(GPIO, 2, 27, &pin_config);
  // GPIO_WritePinOutput(GPIO, 2, 27, 1);
  GPIO->B[2][27] = 1;


}
