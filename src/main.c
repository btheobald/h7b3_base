#include "main.h"
#include "stm32h7b3xxq.h"
#include <stdio.h>
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_system.h"

void RCC_Config();
void UART1_Config();

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

  // Enable LCD Backlight
  LL_GPIO_SetPinMode(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetOutputPin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin);

  printf("GPIO Configured\r\n");

  printf("Configure Success\r\n");

  char rxb = '\0';

  while (1)
  {
    USER_LED1_GPIO_Port->ODR ^= (1 << 11);

    while( !( USART1->ISR & USART_ISR_RXNE_RXFNE ) ) {};
    rxb = USART1->RDR;
    printf("%c\r\n", rxb);
  }
}

void RCC_Config() {
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

  // PLL1
  LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE); // 24M
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
  LL_RCC_PLL1_SetQ(2);
  LL_RCC_PLL1_SetR(2);

  // Enable PLL
  LL_RCC_PLL1_Enable();

  // Ensure PLL is locked
  while(LL_RCC_PLL1_IsReady() != 1) { }

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

void UART1_Config() {
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
