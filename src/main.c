#include "main.h"
#include "stm32h7b3xxq.h"
#include <stdio.h>
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_sdmmc.h"
#include "rk043fn48h_lcd.h"
#include "clut_l8.h"

#include "hagl.h"
#include "hagl_hal.h"
#include "font9x18_ISO8859_1.h"

void RCC_Config(void);
void UART1_Config(void);
void LTDC_Config(void);
void LTDC_GPIO_Config(void);
void LTDC_L8_Layer_Config(LTDC_Layer_TypeDef * layer, uint8_t * fb_base, uint16_t s_x, uint16_t s_y, uint16_t e_x, uint16_t e_y);
void LTDC_CLUT_Config(LTDC_Layer_TypeDef * layer, const uint32_t * clut_base);
void LCD_Colour_Test(void);
void LCD_Clear(void);
uint32_t SDMMC1_Config(void);

unsigned char framebuffer_l8[272*480] = {0}; // Fixed Framebuffer

wchar_t buffer[64];

int main(void)
{
  UART1_Config();

  printf("USART1 Configured \n\r\t- CSI = %.1fM\r\n", CSI_VALUE/1E6);

  RCC_Config();

  LL_RCC_ClocksTypeDef clk;
  LL_RCC_GetSystemClocksFreq(&clk);
  printf("Clock Configured\r\n");

  printf("\t- SYS = %.1fM\tHCLK = %.1fM\r\n", clk.SYSCLK_Frequency/1E6, clk.HCLK_Frequency/1E6);
  printf("\t- PCLK = %.1fM, %.1fM, %.1fM, %.1fM \r\n", clk.PCLK1_Frequency/1E6, clk.PCLK2_Frequency/1E6, clk.PCLK3_Frequency/1E6, clk.PCLK4_Frequency/1E6);

  // Enable MCO1
  LL_GPIO_SetPinMode(MCO1_GPIO_Port, MCO1_Pin, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinOutputType(MCO1_GPIO_Port, MCO1_Pin, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinSpeed(MCO1_GPIO_Port, MCO1_Pin, LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetAFPin_8_15(MCO_GPIO_Port, MCO1_Pin, LL_GPIO_AF_0);

  // Enable LED
  LL_GPIO_SetPinMode(USER_LED1_GPIO_Port, USER_LED1_Pin, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetOutputPin(USER_LED1_GPIO_Port, USER_LED1_Pin);

  printf("GPIO Configured\r\n");

  LTDC_Config();

  printf("LTDC Configured\r\n");

  LCD_Colour_Test();
  printf("\tLCD Colour Test\r\n");
  LL_mDelay(1000);
  LCD_Clear();
  printf("\tLCD Cleared\r\n");

  SDMMC1_Config();

  printf("Configure Success\r\n");

  swprintf(buffer, 64, L"HAGL Init...\n");
  hagl_put_text(buffer, 0, 0, 247, font9x18_ISO8859_1);

  char rxb = '\0';
  uint8_t line = 0;
  uint8_t col = 0;

  while (1)
  {
    USER_LED1_GPIO_Port->ODR ^= (1 << 11);

    while( !( USART1->ISR & USART_ISR_RXNE_RXFNE ) ) {};
    rxb = USART1->RDR;
    
    /*if(rxb == '\r' | rxb == '\n') {
      line++;
      col = 0;
    } else if(rxb >= 32 && rxb <= 127 ) {
      swprintf(message, 64, L"%c", rxb);
      hagl_put_text(message, 10+(9*col), 10+(18*line), 247, font9x18_ISO8859_1);
      col++;   
    } if (rxb == 27) {
      line = 0;
      col = 0;
      LCD_Clear();
    }*/

    //LL_mDelay(500);
  }
}

void RCC_Config(void) {
  // Configure SMPS
  LL_PWR_ConfigSupply(LL_PWR_DIRECT_SMPS_SUPPLY);
  // Scale to boost mode
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);
  // Wait for VOS to be ready
  while (LL_PWR_IsActiveFlag_VOS() == 0) {
    
  }

  // Configure HSE
  LL_RCC_HSE_Enable();
  // Wait for HSE to be ready
  while(!LL_RCC_HSE_IsReady()) { }

  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_7);

  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE); // 24M

  //***** PLL1 *****//
  // Enable PLL Outputs
  LL_RCC_PLL1P_Enable();
  LL_RCC_PLL1Q_Enable();
  LL_RCC_PLL1R_Enable();
  // Disable Fractional
  LL_RCC_PLL1FRACN_Disable();
  // VCO Ranges
  LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_2_4);
  LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  // Prescalers
  LL_RCC_PLL1_SetM(12);
  LL_RCC_PLL1_SetN(280);
  LL_RCC_PLL1_SetP(2);
  LL_RCC_PLL1_SetQ(14);
  LL_RCC_PLL1_SetR(4);
  // Enable PLL
  LL_RCC_PLL1_Enable();
  // Ensure PLL is locked
  while(LL_RCC_PLL1_IsReady() != 1) { }

  //***** PLL2 *****//
  // Enable PLL Outputs
  LL_RCC_PLL2P_Enable();
  LL_RCC_PLL2Q_Enable();
  LL_RCC_PLL2R_Enable();
  // Disable Fractional
  LL_RCC_PLL2FRACN_Disable();
  // VCO Ranges
  LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_2_4);
  LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  // Prescalers
  LL_RCC_PLL2_SetM(6);
  LL_RCC_PLL2_SetN(100);
  LL_RCC_PLL2_SetP(2);
  LL_RCC_PLL2_SetQ(2);
  LL_RCC_PLL2_SetR(8);
  // Enable PLL
  LL_RCC_PLL2_Enable();
  // Ensure PLL is locked
  while(LL_RCC_PLL2_IsReady() != 1) { }

    //***** PLL3 *****//
  // Enable PLL Outputs
  LL_RCC_PLL3P_Enable();
  LL_RCC_PLL3Q_Enable();
  LL_RCC_PLL3R_Enable();
  // Disable Fractional
  LL_RCC_PLL3FRACN_Disable();
  // VCO Ranges
  LL_RCC_PLL3_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_2_4);
  LL_RCC_PLL3_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  // Prescalers
  LL_RCC_PLL3_SetM(12);
  LL_RCC_PLL3_SetN(270);
  LL_RCC_PLL3_SetP(2);
  LL_RCC_PLL3_SetQ(2);
  LL_RCC_PLL3_SetR(60);
  // Enable PLL
  LL_RCC_PLL3_Enable();
  // Ensure PLL is locked
  while(LL_RCC_PLL3_IsReady() != 1) { }

  // Configure SYS/AHB/APB Prescalers
  LL_RCC_SetSysPrescaler( LL_RCC_SYSCLK_DIV_1 );
  LL_RCC_SetAHBPrescaler( LL_RCC_AHB_DIV_1 );
  LL_RCC_SetAPB1Prescaler( LL_RCC_APB1_DIV_2 );
  LL_RCC_SetAPB2Prescaler( LL_RCC_APB2_DIV_2 );
  LL_RCC_SetAPB3Prescaler( LL_RCC_APB3_DIV_2 );
  LL_RCC_SetAPB4Prescaler( LL_RCC_APB4_DIV_2 );

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1) {}

  LL_Init1msTick(280000000);
  LL_SetSystemCoreClock(280000000);

  LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_PLL1QCLK, LL_RCC_MCO1_DIV_4);

  // Enable GPIO Banks A-K Clocks
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA | LL_AHB4_GRP1_PERIPH_GPIOG);
}

void UART1_Config(void) {
  // Configure CSI
  RCC->CR |= RCC_CR_CSION;
  // Wait for HSE to be ready
  while(!(RCC->CR & RCC_CR_CSIRDY)) { }

  // Select CSI as clock
  RCC->CDCCIP2R &= ~RCC_CDCCIP2R_USART16910SEL_Msk;
  RCC->CDCCIP2R |= RCC_CDCCIP2R_USART16910SEL_2;

  // Enable USART1 Peripheral Clock
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  // Enable USART1 GPIO Clock
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);

  // USART1 Pin Config
  LL_GPIO_SetPinMode(VCP_TX_GPIO_Port, VCP_TX_Pin, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinMode(VCP_RX_GPIO_Port, VCP_RX_Pin, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(VCP_TX_GPIO_Port, VCP_TX_Pin, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetPinSpeed(VCP_RX_GPIO_Port, VCP_RX_Pin, LL_GPIO_SPEED_FREQ_VERY_HIGH);
  LL_GPIO_SetAFPin_8_15(VCP_TX_GPIO_Port, VCP_TX_Pin, LL_GPIO_AF_7);
  LL_GPIO_SetAFPin_8_15(VCP_RX_GPIO_Port, VCP_RX_Pin, LL_GPIO_AF_7);

  // USART1 Enable
  USART1->BRR = 0x22; // 115200 Baud
  USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
  USART1->CR2 |= USART_CR2_ABREN;

  // Clear Terminal
  printf("\033c");
}

void LTDC_Config(void) {
  // Configure GPIO
  LTDC_GPIO_Config();

  // Enable LTDC Peripheral Clock
  LL_APB3_GRP1_EnableClock(LL_APB3_GRP1_PERIPH_LTDC);
  
  // Timing Config
  LTDC->SSCR = (RK043FN48H_HSYNC-1) << 16 | (RK043FN48H_VSYNC-1); // Sync Pulses
  LTDC->BPCR = (  RK043FN48H_HBP-1) << 16 | (  RK043FN48H_VBP-1); // Back Porch
  LTDC->AWCR = (   RK043FN48H_AW-1) << 16 | (   RK043FN48H_AH-1); // Active Period
  LTDC->TWCR = (   RK043FN48H_TW-1) << 16 | (   RK043FN48H_TH-1); // Total Period

  // Background Colour
  LTDC->BCCR = 0x00550000;

  // Configure Layer
  LTDC_L8_Layer_Config(LTDC_Layer1, framebuffer_l8, 0, 0, 480, 272);

  LTDC->SRCR |= LTDC_SRCR_IMR;

  // Enable LTDC
  LTDC->GCR |= LTDC_GCR_LTDCEN;
}

void LTDC_GPIO_Config(void) {
  //LTDC Enable GPIO Clocks
  LL_AHB4_GRP1_EnableClock( LL_AHB4_GRP1_PERIPH_GPIOA | LL_AHB4_GRP1_PERIPH_GPIOH \
                          | LL_AHB4_GRP1_PERIPH_GPIOI | LL_AHB4_GRP1_PERIPH_GPIOJ \
                          | LL_AHB4_GRP1_PERIPH_GPIOK );

  // Config Pins
  LL_GPIO_SetPinMode(   LCD_BL_CTRL_GPIO_Port,LCD_BL_CTRL_Pin, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetOutputPin( LCD_BL_CTRL_GPIO_Port,LCD_BL_CTRL_Pin);
  LL_GPIO_SetPinMode(   LCD_ON_OFF_GPIO_Port, LCD_ON_OFF_Pin, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetOutputPin( LCD_ON_OFF_GPIO_Port, LCD_ON_OFF_Pin);  
  LL_GPIO_SetPinMode(   LCD_CLK_GPIO_Port,    LCD_CLK_Pin,    LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(LCD_CLK_GPIO_Port,    LCD_CLK_Pin,    LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_DE_GPIO_Port,     LCD_DE_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_DE_GPIO_Port,     LCD_DE_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_HSYNC_GPIO_Port,  LCD_HSYNC_Pin,  LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(LCD_HSYNC_GPIO_Port,  LCD_HSYNC_Pin,  LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_VSYNC_GPIO_Port,  LCD_VSYNC_Pin,  LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(LCD_VSYNC_GPIO_Port,  LCD_VSYNC_Pin,  LL_GPIO_AF_14);

  LL_GPIO_SetPinMode(   LCD_R0_GPIO_Port,     LCD_R0_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(LCD_R0_GPIO_Port,     LCD_R0_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_R1_GPIO_Port,     LCD_R1_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_R1_GPIO_Port,     LCD_R1_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_R2_GPIO_Port,     LCD_R2_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_R2_GPIO_Port,     LCD_R2_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_R3_GPIO_Port,     LCD_R3_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_R3_GPIO_Port,     LCD_R3_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_R4_GPIO_Port,     LCD_R4_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_R4_GPIO_Port,     LCD_R4_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_R5_GPIO_Port,     LCD_R5_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_R5_GPIO_Port,     LCD_R5_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_R6_GPIO_Port,     LCD_R6_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_R6_GPIO_Port,     LCD_R6_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_R7_GPIO_Port,     LCD_R7_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_R7_GPIO_Port,     LCD_R7_Pin,     LL_GPIO_AF_14);

  LL_GPIO_SetPinMode(   LCD_G0_GPIO_Port,     LCD_G0_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_G0_GPIO_Port,     LCD_G0_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_G1_GPIO_Port,     LCD_G1_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(LCD_G1_GPIO_Port,     LCD_G1_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_G2_GPIO_Port,     LCD_G2_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(LCD_G2_GPIO_Port,     LCD_G2_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_G3_GPIO_Port,     LCD_G3_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(LCD_G3_GPIO_Port,     LCD_G3_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_G4_GPIO_Port,     LCD_G4_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(LCD_G4_GPIO_Port,     LCD_G4_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_G5_GPIO_Port,     LCD_G5_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_G5_GPIO_Port,     LCD_G5_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_G6_GPIO_Port,     LCD_G6_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_G6_GPIO_Port,     LCD_G6_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_G7_GPIO_Port,     LCD_G7_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_G7_GPIO_Port,     LCD_G7_Pin,     LL_GPIO_AF_14);

  LL_GPIO_SetPinMode(   LCD_B0_GPIO_Port,     LCD_B0_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(LCD_B0_GPIO_Port,     LCD_B0_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_B1_GPIO_Port,     LCD_B1_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(LCD_B1_GPIO_Port,     LCD_B1_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_B2_GPIO_Port,     LCD_B2_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(LCD_B2_GPIO_Port,     LCD_B2_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_B3_GPIO_Port,     LCD_B3_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(LCD_B3_GPIO_Port,     LCD_B3_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_B4_GPIO_Port,     LCD_B4_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_B4_GPIO_Port,     LCD_B4_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_B5_GPIO_Port,     LCD_B5_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_B5_GPIO_Port,     LCD_B5_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_B6_GPIO_Port,     LCD_B6_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_B6_GPIO_Port,     LCD_B6_Pin,     LL_GPIO_AF_14);
  LL_GPIO_SetPinMode(   LCD_B7_GPIO_Port,     LCD_B7_Pin,     LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( LCD_B7_GPIO_Port,     LCD_B7_Pin,     LL_GPIO_AF_14);
}

void LTDC_L8_Layer_Config(LTDC_Layer_TypeDef * layer, uint8_t * fb_base, uint16_t s_x, uint16_t s_y, uint16_t e_x, uint16_t e_y) {
  layer->WHPCR = ((e_x+8) << LTDC_LxWHPCR_WHSPPOS_Pos) | s_x+8;
  layer->WVPCR = ((e_y+2) << LTDC_LxWVPCR_WVSPPOS_Pos) | s_y+2;
  layer->PFCR = 5; // L8
  layer->CFBAR = fb_base;
  layer->CFBLR = ((e_x - s_x) << LTDC_LxCFBLR_CFBP_Pos) | ((e_x - s_x) + 7);
  layer->CFBLNR = (e_y - s_y);

  LTDC_CLUT_Config(layer, CLUT_L8);

  layer->CACR = 0xFF;
  layer->DCCR = 0xFFFF0000;

  LTDC->SRCR |= LTDC_SRCR_IMR;

  layer->CR |= LTDC_LxCR_LEN | LTDC_LxCR_CLUTEN;
}

void LTDC_CLUT_Config(LTDC_Layer_TypeDef * layer, const uint32_t * clut_base) {
  for(int addr = 0; addr < 255; addr++) {
    uint32_t tmp = (addr << LTDC_LxCLUTWR_CLUTADD_Pos) | clut_base[addr];
    layer->CLUTWR = tmp;
  }
}

void LCD_Colour_Test(void) {
  const x_size = 30;
  const y_size = 17;
  const x_per_row = (480/x_size);
  const y_per_col = (272/y_size);

  uint8_t tmp_col = 0;
  for(int x = 0; x < 480; x++) {
    for(int y = 0; y < 272; y++) {
      tmp_col = ((x)/x_size) + (y/y_size)*x_per_row;
      framebuffer_l8[x + y*480] = tmp_col;
    }
  }
}

void LCD_Clear(void) {
  for(int x = 0; x < 480; x++) {
    for(int y = 0; y < 272; y++) {
      framebuffer_l8[x + y*480] = 0;
    }
  }
}

uint32_t SDMMC1_Config(void) {
  LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_SDMMC1);
  LL_RCC_SetSDMMCClockSource(LL_RCC_SDMMC_CLKSOURCE_PLL2R);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC | LL_AHB4_GRP1_PERIPH_GPIOD);

  LL_GPIO_SetPinMode(   SDIO1_CK_GPIO_Port,  SDIO1_CK_Pin,   LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(SDIO1_CK_GPIO_Port,  SDIO1_CK_Pin,   LL_GPIO_AF_12);
  LL_GPIO_SetPinSpeed(  SDIO1_CK_GPIO_Port,  SDIO1_CK_Pin,   LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinMode(   SDIO1_CMD_GPIO_Port, SDIO1_CMD_Pin,  LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7( SDIO1_CMD_GPIO_Port, SDIO1_CMD_Pin,  LL_GPIO_AF_12);
  LL_GPIO_SetPinSpeed(  SDIO1_CMD_GPIO_Port, SDIO1_CMD_Pin,  LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinMode(   SDIO1_D0_GPIO_Port,  SDIO1_D0_Pin,   LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(SDIO1_D0_GPIO_Port,  SDIO1_D0_Pin,   LL_GPIO_AF_12);
  LL_GPIO_SetPinSpeed(  SDIO1_D0_GPIO_Port,  SDIO1_D0_Pin,   LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinMode(   SDIO1_D1_GPIO_Port,  SDIO1_D1_Pin,   LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(SDIO1_D1_GPIO_Port,  SDIO1_D1_Pin,   LL_GPIO_AF_12);
  LL_GPIO_SetPinSpeed(  SDIO1_D1_GPIO_Port,  SDIO1_D1_Pin,   LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinMode(   SDIO1_D2_GPIO_Port,  SDIO1_D2_Pin,   LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(SDIO1_D2_GPIO_Port,  SDIO1_D2_Pin,   LL_GPIO_AF_12);
  LL_GPIO_SetPinSpeed(  SDIO1_D2_GPIO_Port,  SDIO1_D2_Pin,   LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinMode(   SDIO1_D3_GPIO_Port,  SDIO1_D3_Pin,   LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(SDIO1_D3_GPIO_Port,  SDIO1_D3_Pin,   LL_GPIO_AF_12);
  LL_GPIO_SetPinSpeed(  SDIO1_D3_GPIO_Port,  SDIO1_D3_Pin,   LL_GPIO_SPEED_HIGH);
  LL_GPIO_SetPinMode( uSD_Detect_GPIO_Port,  uSD_Detect_Pin, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinPull( uSD_Detect_GPIO_Port,  uSD_Detect_Pin, LL_GPIO_PULL_UP);

  printf("SDMMC Init\r\n");

  if (LL_GPIO_IsInputPinSet(uSD_Detect_GPIO_Port,uSD_Detect_Pin)) {
    printf("\tCard Not Present\r\n");
    return 0;
  } 

  SDMMC_InitTypeDef init;
  init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  init.ClockDiv = 125; // Init to 400kHz
  init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  init.BusWide = SDMMC_BUS_WIDE_1B;
  
  SDMMC_Init(SDMMC1, init);
  SDMMC_PowerState_ON(SDMMC1);
  
  printf("\t1B/400kHz Mode\r\n");

  LL_mDelay(10);

  uint32_t errorstate;
  
  errorstate = SDMMC_CmdGoIdleState(SDMMC1);
  if (errorstate != 0) return errorstate;

  /* CMD8: SEND_IF_COND: Command available only on V2.0 cards */
  errorstate = SDMMC_CmdOperCond(SDMMC1);
  if (errorstate != 0) 
    errorstate = SDMMC_CmdGoIdleState(SDMMC1);
    if(errorstate != 0) return errorstate;

  printf("\tCard Version V2\r\n");

  __IO uint32_t count = 0U;
  uint32_t response = 0U;
  uint32_t validvoltage = 0U;
  while ((count < SDMMC_MAX_VOLT_TRIAL) && (validvoltage == 0U)) {
    /* SEND CMD55 APP_CMD with RCA as 0 */
    errorstate = SDMMC_CmdAppCommand(SDMMC1, 0);
    if (errorstate != 0) return errorstate;

    /* Send CMD41 */
    errorstate = SDMMC_CmdAppOperCommand(SDMMC1, SDMMC_VOLTAGE_WINDOW_SD | SDMMC_HIGH_CAPACITY | SD_SWITCH_1_8V_CAPACITY);
    if (errorstate != 0) return errorstate;

    /* Get command response */
    response = SDMMC_GetResponse(SDMMC1, SDMMC_RESP1);

    /* Get operating voltage*/
    validvoltage = (((response >> 31U) == 1U) ? 1U : 0U);
    if(validvoltage) 
      printf("\tVoltage Valid\r\n");
    else
      printf("\tVoltage Not Valid\r\n");

    count++;
  }   

  if (count >= SDMMC_MAX_VOLT_TRIAL)
  {
    printf("\tInvalid Voltage Range\r\n");
    return errorstate;
  }

  if ((response & SDMMC_HIGH_CAPACITY) == SDMMC_HIGH_CAPACITY) /* (response &= SD_HIGH_CAPACITY) */
  {
    printf("\tHC/XC Detected\r\n");
  }

  if (errorstate != 0)
  {
    printf("\tPower On Failed\r\n");
    return errorstate;
  }

  if(SDMMC_GetPowerState(SDMMC1) == 0) {
    printf("\tPower State Fail\r\n");
    return errorstate;
  }

  errorstate = SDMMC_CmdSendCID(SDMMC1);
  if (errorstate != 0)
  {
    printf("\tError Reading CID\r\n");
    return errorstate;
  }
  uint32_t CID[4];
  CID[0] = SDMMC_GetResponse(SDMMC1, SDMMC_RESP1);
  CID[1] = SDMMC_GetResponse(SDMMC1, SDMMC_RESP2);
  CID[2] = SDMMC_GetResponse(SDMMC1, SDMMC_RESP3);
  CID[3] = SDMMC_GetResponse(SDMMC1, SDMMC_RESP4);
  printf("\tCID: %08X - %08X - %08X - %08X\r\n", CID[0], CID[1], CID[2], CID[3]);

  printf("\t\t- MID: %d\r\n", (CID[0] & 0xFF000000) >> 24);
  printf("\t\t- OID: %c%c\r\n", (CID[0] & 0x00FF0000) >> 16, (CID[0] & 0x0000FF00) >> 8 );
  printf("\t\t- PNM: %c%c%c%c%c\r\n", (CID[0] & 0x000000FF), (CID[1] & 0xFF000000) >> 24, (CID[1] & 0x00FF0000) >> 16, (CID[1] & 0x0000FF00) >> 8, (CID[1] & 0x000000FF));
  printf("\t\t- PRV: %01x%01x\r\n", (CID[2] & 0xF0000000) >> 28, (CID[2] & 0x0F000000) >> 24);
  printf("\t\t- PSN: %06X%02X\r\n", (CID[2] & 0x00FFFFFF), (CID[3] & 0xFF000000) >> 24);
  printf("\t\t- MDT: %d/%d\r\n", (CID[3] & 0x00000F00) >> 8, ((CID[3] & 0x000FF000) >> 12)+2000);
  printf("\t\t- CRC: %02X\t\r\n", (CID[3] & 0x000000FE) >> 1);

  uint16_t sd_rel_addr = 1;
  errorstate = SDMMC_CmdSetRelAdd(SDMMC1, &sd_rel_addr);
  if (errorstate != 0)
  {
    printf("\tError Setting Relative Address\r\n");
    return errorstate;
  }
  printf("\tRelAddr: %d\r\n", sd_rel_addr);

  errorstate = SDMMC_CmdSendCSD(SDMMC1, sd_rel_addr << 16);
  if (errorstate != 0)
  {
    printf("\tError Reading CSD\r\n");
    return errorstate;
  }
  uint32_t CSD[4];
  CSD[0] = SDMMC_GetResponse(SDMMC1, SDMMC_RESP1);
  CSD[1] = SDMMC_GetResponse(SDMMC1, SDMMC_RESP2);
  CSD[2] = SDMMC_GetResponse(SDMMC1, SDMMC_RESP3);
  CSD[3] = SDMMC_GetResponse(SDMMC1, SDMMC_RESP4);
  printf("\tCSD: %08X - %08X - %08X - %08X\r\n", CSD[0], CSD[1], CSD[2], CSD[3]);

  printf("\tClass: %d\r\n", SDMMC_GetResponse(SDMMC1, SDMMC_RESP2) >> 20);
  
  errorstate = SDMMC_CmdSelDesel(SDMMC1, (uint32_t)(((uint32_t)sd_rel_addr) << 16U));
  if (errorstate != 0)
  {
    printf("\tError Selecting Card\r\n");
    return errorstate;
  }
  
  errorstate = SDMMC_CmdAppCommand(SDMMC1, (uint32_t)(sd_rel_addr << 16U));
  if (errorstate != 0) return errorstate;

  // Send ACMD6 APP_CMD with argument as 2 for wide bus mode 
  errorstate = SDMMC_CmdBusWidth(SDMMC1, 2U);
  if (errorstate != 0) return errorstate;

  if(errorstate != 0) {
    __SDMMC_CLEAR_FLAG(SDMMC1, SDMMC_STATIC_FLAGS);
    return errorstate;
  } else {
    init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    init.ClockDiv = 125; // Init to 50MHz
    init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    init.BusWide = SDMMC_BUS_WIDE_4B;
    SDMMC_Init(SDMMC1, init);
    printf("\tACMD6: 4-Bit Bus Mode\r\n");
  }

  uint32_t index = 0U;
  uint32_t tempscr[2U] = {0UL, 0UL};
  uint32_t scr[2] = {0};
  SDMMC_DataInitTypeDef config;

  /* Set Block Size To 8 Bytes */
  errorstate = SDMMC_CmdBlockLength(SDMMC1, 8U);
  if (errorstate != 0) return errorstate;

  /* Send CMD55 APP_CMD with argument as card's RCA */
  errorstate = SDMMC_CmdAppCommand(SDMMC1, (uint32_t)(sd_rel_addr << 16U));
  if (errorstate != 0) return errorstate;

  config.DataTimeOut   = SDMMC_DATATIMEOUT;
  config.DataLength    = 8U;
  config.DataBlockSize = SDMMC_DATABLOCK_SIZE_8B;
  config.TransferDir   = SDMMC_TRANSFER_DIR_TO_SDMMC;
  config.TransferMode  = SDMMC_TRANSFER_MODE_BLOCK;
  config.DPSM          = SDMMC_DPSM_ENABLE;
  SDMMC_ConfigData(SDMMC1, &config);
  
  errorstate = SDMMC_CmdSendSCR(SDMMC1);
  if (errorstate != 0) return errorstate; 

  while (!__SDMMC_GET_FLAG(SDMMC1, SDMMC_FLAG_RXOVERR | SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_DBCKEND | SDMMC_FLAG_DATAEND)) {
    if ((!__SDMMC_GET_FLAG(SDMMC1, SDMMC_FLAG_RXFIFOE)) && (index == 0U)) {
      tempscr[0] = SDMMC_ReadFIFO(SDMMC1);
      tempscr[1] = SDMMC_ReadFIFO(SDMMC1);
      index++;
    }
  }

  if (__SDMMC_GET_FLAG(SDMMC1, SDMMC_FLAG_DTIMEOUT)) {
    __SDMMC_CLEAR_FLAG(SDMMC1, SDMMC_FLAG_DTIMEOUT);
    printf("Data Timeout");
    return SDMMC_ERROR_DATA_TIMEOUT;
  } else if (__SDMMC_GET_FLAG(SDMMC1, SDMMC_FLAG_DCRCFAIL)) {
    __SDMMC_CLEAR_FLAG(SDMMC1, SDMMC_FLAG_DCRCFAIL);
    printf("CRC Mismatch");
    return SDMMC_ERROR_DATA_CRC_FAIL; 
  } else if (__SDMMC_GET_FLAG(SDMMC1, SDMMC_FLAG_RXOVERR)) {
    __SDMMC_CLEAR_FLAG(SDMMC1, SDMMC_FLAG_RXOVERR);
    printf("RX Overrun");
    return SDMMC_ERROR_RX_OVERRUN;
  }

  scr[0] = (((tempscr[1] & SDMMC_0TO7BITS) << 24)  | ((tempscr[1] & SDMMC_8TO15BITS) << 8) | \
          ((tempscr[1] & SDMMC_16TO23BITS) >> 8) | ((tempscr[1] & SDMMC_24TO31BITS) >> 24));

  scr[1] = (((tempscr[0] & SDMMC_0TO7BITS) << 24)  | ((tempscr[0] & SDMMC_8TO15BITS) << 8) | \
          ((tempscr[0] & SDMMC_16TO23BITS) >> 8) | ((tempscr[0] & SDMMC_24TO31BITS) >> 24));

  printf("\tSCR: %08X %08X\r\n", scr[1], scr[0]);

  /* Set Block Size To 64 Bytes */
  SDMMC1->DCTRL = 0;
  errorstate = SDMMC_CmdBlockLength(SDMMC1, 64U);
  if (errorstate != 0) return errorstate;

  config.DataTimeOut   = SDMMC_DATATIMEOUT;
  config.DataLength    = 64U;
  config.DataBlockSize = SDMMC_DATABLOCK_SIZE_64B ;
  config.TransferDir   = SDMMC_TRANSFER_DIR_TO_SDMMC;
  config.TransferMode  = SDMMC_TRANSFER_MODE_BLOCK;
  config.DPSM          = SDMMC_DPSM_ENABLE;
  SDMMC_ConfigData(SDMMC1, &config);

  errorstate = SDMMC_CmdSwitch(SDMMC1, SDMMC_SDR25_SWITCH_PATTERN);
  if (errorstate != 0) return errorstate;
  __SDMMC_CLEAR_FLAG(SDMMC1, SDMMC_STATIC_FLAGS);

  index = 0;
  uint32_t sdhs[16] = {0};
  count = 0;
  while (!__SDMMC_GET_FLAG(SDMMC1, SDMMC_FLAG_RXOVERR | SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_DBCKEND | SDMMC_FLAG_DATAEND)) {
    if (__SDMMC_GET_FLAG(SDMMC1, SDMMC_FLAG_RXFIFOHF)) {
      for(count = 0; count < 8; count++) {
        sdhs[(8*index) + count] = SDMMC_ReadFIFO(SDMMC1);
      }
      index++;
    }
  }
  printf("\t%CMD6: %08X - %08X - %08X - %08X - %08X\r\n", sdhs[0], sdhs[1], sdhs[2], sdhs[3], sdhs[4]);

  if (__SDMMC_GET_FLAG(SDMMC1, SDMMC_FLAG_DTIMEOUT)) {
    __SDMMC_CLEAR_FLAG(SDMMC1, SDMMC_FLAG_DTIMEOUT);
    printf("Data Timeout");
    return SDMMC_ERROR_DATA_TIMEOUT;
  } else if (__SDMMC_GET_FLAG(SDMMC1, SDMMC_FLAG_DCRCFAIL)) {
    __SDMMC_CLEAR_FLAG(SDMMC1, SDMMC_FLAG_DCRCFAIL);
    printf("CRC Mismatch");
    return SDMMC_ERROR_DATA_CRC_FAIL; 
  } else if (__SDMMC_GET_FLAG(SDMMC1, SDMMC_FLAG_RXOVERR)) {
    __SDMMC_CLEAR_FLAG(SDMMC1, SDMMC_FLAG_RXOVERR);
    printf("RX Overrun");
    return SDMMC_ERROR_RX_OVERRUN;
  }

  if ((((uint8_t *)sdhs)[13] & 2U) != 2U) {
    printf("\t25MHz DS Mode\r\n");
    init.ClockDiv = 2; // Init to 50MHz
    SDMMC_Init(SDMMC1, init);
  } else {
    printf("\t50MHz HS Mode\r\n");    
    init.ClockDiv = 1; // Init to 50MHz
    SDMMC_Init(SDMMC1, init);
  }

  printf("SDMMC Configured\r\n");

  return 0;
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
  }
}

int _write(int handle, char* data, int size) {
  int count = size;
  while(count--) {
    while( !( USART1->ISR & USART_ISR_TXE_TXFNF ) ) {};
    USART1->TDR = *data++;
  }
  return size;
}