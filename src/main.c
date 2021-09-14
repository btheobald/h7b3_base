#include "main.h"
#include "stm32h7b3xxq.h"

void PWR_Config();
void RCC_Config();

int main(void)
{
  PWR_Config();
  RCC_Config();

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

  // MCO Alt function
  VCP_TX_GPIO_Port->MODER &= ~GPIO_MODER_MODE9_Msk;
  VCP_RX_GPIO_Port->MODER &= ~GPIO_MODER_MODE10_Msk;
  VCP_TX_GPIO_Port->MODER |= GPIO_MODER_MODE9_1; // AF
  VCP_RX_GPIO_Port->MODER |= GPIO_MODER_MODE10_1; // AF
  VCP_TX_GPIO_Port->AFR[1] &= ~GPIO_AFRH_AFSEL9_Msk;
  VCP_RX_GPIO_Port->AFR[1] &= ~GPIO_AFRH_AFSEL10_Msk;
  VCP_TX_GPIO_Port->AFR[1] |= 7 << GPIO_AFRH_AFSEL9_Pos;
  VCP_RX_GPIO_Port->AFR[1] |= 7 << GPIO_AFRH_AFSEL10_Pos;

  USART1->BRR |= (int32_t)(24000000/115200); // 115200 Baud
  USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

  while (1)
  {
    //for(int i = 100; i < 100; i++) 
    //  for(int a = 1000; a < 1000; a++)
    //    asm("mov r0,r0");
    USER_LED1_GPIO_Port->ODR ^= (1 << 11);

    char rxb = '\0';
    while( !( USART1->ISR & USART_ISR_RXNE_RXFNE ) ) {};
    rxb = USART1->RDR;
    while( !( USART1->ISR & USART_ISR_TXE_TXFNF ) ) {};
    USART1->TDR = rxb;
    
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

  // Configure HSI
  RCC->CR |= RCC_CR_HSEON;
  // Wait for HSE to be ready
  while(!(RCC->CR & RCC_CR_HSERDY)) { }

  // AXI/APB/AHB = SYSCLK/2 (64M/32M)
  //RCC->CDCFGR1 |= RCC_CDCFGR1_HPRE_3; 
  //RCC->CDCFGR1 |= RCC_CDCFGR1_CDCPRE_DIV2;
  //RCC->CDCFGR2 |= RCC_CDCFGR2_CDPPRE1_DIV2;
  //R/CC->CDCFGR2 |= RCC_CDCFGR2_CDPPRE2_DIV2;
  //RCC->SRDCFGR |= RCC_SRDCFGR_SRDPPRE_DIV2;

  // MCO1 Prescaler
  //RCC->CFGR |= (1 << RCC_CFGR_MCO1PRE_Pos);

  // DIVM3 (/12)
  RCC->PLLCKSELR |= (12 << RCC_PLLCKSELR_DIVM3_Pos);

  // DIVR3 (Enable for LTDC = 9MHz)
  RCC->PLLCFGR |= RCC_PLLCFGR_DIVR3EN;
  RCC->PLL3DIVR |= 27 << RCC_PLL3DIVR_N3_Pos; // x27
  RCC->PLL3DIVR |= 15 << RCC_PLL3DIVR_R3_Pos; // /16

  // Turn on PLL3
  RCC->CR |= RCC_CR_PLL3ON;
  // Wait for PLL3 to be ready
  while(!(RCC->CR & RCC_CR_PLL3RDY)) {}

  // Peripheral Clocks
  // Enable GPIO Banks A-K Clocks
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOGEN;
  // Enable LTDC Clock
  RCC->APB3ENR |= RCC_APB3ENR_LTDCEN;
  // Enable USART1 Clock
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  // HSE as main clock
  RCC->CFGR |= RCC_CFGR_SWS_1;
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
  }
}
