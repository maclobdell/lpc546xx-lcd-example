#include "mbed.h"
#include "lpc546xx_lcd.h"

LPC546XX_LCD::LPC546XX_LCD(uint8_t* framebuf)
{

  static const uint32_t palette[] = { 0x001F00000 }; //{0x0000001F}; //{0x001F0000U, 0x7C0003E0U};
    
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

  /* Set the back light PWM. */
  BOARD_InitPWM();

  /* Initialize the display. */
  lcdc_config_t lcdConfig;

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
  lcdConfig.upperPanelAddr = (uint32_t)framebuf;
  lcdConfig.bpp = kLCDC_1BPP;
  lcdConfig.display = kLCDC_DisplayTFT;
  lcdConfig.swapRedBlue = true;  //false;
  lcdConfig.dataFormat = kLCDC_WinCeMode;

  LCDC_Init(APP_LCD, &lcdConfig, LCD_INPUT_CLK_FREQ);

  LCDC_SetPalette(APP_LCD, palette, ARRAY_SIZE(palette));


}

void LPC546XX_LCD::on(void)
{

  gpio_pin_config_t pin_config = {
      kGPIO_DigitalOutput, 0,
  };
  
  //turn on the display
  
  /* Trigger interrupt at start of every vertical back porch. */
  LCDC_SetVerticalInterruptMode(APP_LCD, kLCDC_StartOfBackPorch);
  LCDC_EnableInterrupts(APP_LCD, kLCDC_VerticalCompareInterrupt);

  LCDC_Start(APP_LCD);
  LCDC_PowerUp(APP_LCD);

  GPIO_PinInit(GPIO, 2, 27, &pin_config);
  // GPIO_WritePinOutput(GPIO, 2, 27, 1);
  GPIO->B[2][27] = 1;

  
}    

void LPC546XX_LCD::off(void)
{
  //turn off the display
  
   
}    

void LPC546XX_LCD::BOARD_InitPWM(void)
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

void LPC546XX_LCD::APP_LCD_IRQHandler(void)
{
    uint32_t intStatus = LCDC_GetEnabledInterruptsPendingStatus(APP_LCD);

    LCDC_ClearInterruptsStatus(APP_LCD, intStatus);

    if (intStatus & kLCDC_VerticalCompareInterrupt)
    {
        s_frameEndFlag = true;
    }
    __DSB();
}
