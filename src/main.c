#include "main.h"
#include "stm32h7b3xxq.h"
#include <stdio.h>

void PWR_Config();
void RCC_Config();
void UART1_Config();

int main(void)
{
  UART1_Config();

  printf("USART1 Configured\r\n");

  PWR_Config();

  printf("Power Configured\r\n");

  RCC_Config();

  printf("Clock Configured\r\n");

  // MCO Alt function
  MCO1_GPIO_Port->MODER &= ~GPIO_MODER_MODE8_Msk;
  MCO1_GPIO_Port->MODER |= GPIO_MODER_MODE8_1; // AF
  MCO1_GPIO_Port->OSPEEDR |= GPIO_OSPEEDR_OSPEED8_0 | GPIO_OSPEEDR_OSPEED8_1; // Max 
  MCO1_GPIO_Port->AFR[0] &= ~GPIO_AFRL_AFSEL0_Msk;
  
  // Configure User LED
  USER_LED1_GPIO_Port->MODER &= ~GPIO_MODER_MODE11_Msk;
  USER_LED1_GPIO_Port->MODER |= GPIO_MODER_MODE11_0;
  USER_LED1_GPIO_Port->ODR |= (1 << GPIO_ODR_OD11_Pos);

  // Enable LCD Backlight
  LCD_BL_CTRL_GPIO_Port->MODER &= ~GPIO_MODER_MODE1_Msk;
  LCD_BL_CTRL_GPIO_Port->MODER |= GPIO_MODER_MODE1_0;
  LCD_BL_CTRL_GPIO_Port->ODR |= (1 << GPIO_ODR_OD1_Pos);

  printf("Configure Success\r\n");

  char rxb = '\0';

  while (1)
  {
    USER_LED1_GPIO_Port->ODR ^= (1 << 11);

    while( !( USART1->ISR & USART_ISR_RXNE_RXFNE ) ) {};
    rxb = USART1->RDR;

  }
}

void PWR_Config() {
  /* Set the power supply configuration */
  PWR->CR3 = PWR_CR3_SMPSEN;

  // Wait for permitted voltage level
  while (!(PWR->CSR1 & PWR_CSR1_ACTVOSRDY)) { }

  // Wait for SMPS Ready
  while (!(PWR->CR3 & PWR_CR3_SMPSEXTRDY)) {}

  // Configure for VOS0
  //PWR->SRDCR = PWR_SRDCR_VOS_0 | PWR_SRDCR_VOS_1;

  // Wait for VOS to be ready (Default VOS3)
  while(!(PWR->SRDCR & PWR_SRDCR_VOSRDY)) {}
}

void RCC_Config() {
  // Configure HSI
  RCC->CR |= RCC_CR_HSION;
  // Wait for HSE to be ready
  while(!(RCC->CR & RCC_CR_HSIRDY)) { }

  // Configure HSE
  RCC->CR |= RCC_CR_HSEON;
  // Wait for HSE to be ready
  while(!(RCC->CR & RCC_CR_HSERDY)) { }

  // HSE to PLL
  RCC->PLLCKSELR |= RCC_PLLCKSELR_PLLSRC_NONE;

  // HSE as main clock
  RCC->CFGR &= ~RCC_CFGR_SWS_Msk;
  RCC->CFGR |= RCC_CFGR_SWS_1;

  RCC->CR &= ~RCC_CR_HSION;

  // AXI/APB/AHB = SYSCLK/2 (64M/32M)
  //RCC->CDCFGR1 |= RCC_CDCFGR1_HPRE_3; 
  //RCC->CDCFGR1 |= RCC_CDCFGR1_CDCPRE_DIV2;
  //RCC->CDCFGR2 |= RCC_CDCFGR2_CDPPRE1_DIV2;
  //R/CC->CDCFGR2 |= RCC_CDCFGR2_CDPPRE2_DIV2;
  //RCC->SRDCFGR |= RCC_SRDCFGR_SRDPPRE_DIV2;

  // MCO1 Prescaler
  RCC->CFGR |= (1 << RCC_CFGR_MCO1PRE_Pos);
  RCC->CFGR |= (2 << RCC_CFGR_MCO1_Pos);

  // DIVM3 (/12)
  /*RCC->PLLCKSELR |= (12 << RCC_PLLCKSELR_DIVM3_Pos);

  // DIVR3 (Enable for LTDC = 9MHz)
  RCC->PLLCFGR |= RCC_PLLCFGR_DIVR3EN;
  RCC->PLL3DIVR |= 27 << RCC_PLL3DIVR_N3_Pos; // x27
  RCC->PLL3DIVR |= 15 << RCC_PLL3DIVR_R3_Pos; // /16

  // Turn on PLL3
  RCC->CR |= RCC_CR_PLL3ON;
  // Wait for PLL3 to be ready
  while(!(RCC->CR & RCC_CR_PLL3RDY)) {}*/

  // Peripheral Clocks
  // Enable GPIO Banks A-K Clocks
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOGEN;
  // Enable LTDC Clock
  //RCC->APB3ENR |= RCC_APB3ENR_LTDCEN;
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
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;

  // MCO Alt function
  VCP_TX_GPIO_Port->MODER &= ~GPIO_MODER_MODE9_Msk;
  VCP_RX_GPIO_Port->MODER &= ~GPIO_MODER_MODE10_Msk;
  VCP_TX_GPIO_Port->MODER |= GPIO_MODER_MODE9_1; // AF
  VCP_RX_GPIO_Port->MODER |= GPIO_MODER_MODE10_1; // AF
  VCP_TX_GPIO_Port->AFR[1] &= ~GPIO_AFRH_AFSEL9_Msk;
  VCP_RX_GPIO_Port->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;
  VCP_TX_GPIO_Port->AFR[1] |= 7 << GPIO_AFRH_AFSEL9_Pos;
  VCP_RX_GPIO_Port->AFR[1] |= 7 << GPIO_AFRH_AFSEL10_Pos;

  // USART1 Enable
  USART1->BRR = 0x22; // 115200 Baud
  USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
  USART1->CR2 |= USART_CR2_ABREN;
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
