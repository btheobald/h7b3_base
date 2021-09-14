#include "main.h"
#include "stm32h7b3xxq.h"

void PWR_Config();
void RCC_Config();

int main(void)
{
  PWR_Config();
  RCC_Config();

  // MCO Alt function
  MCO1_GPIO_Port->MODER |= (2 << GPIO_MODER_MODE8_Pos);
  MCO1_GPIO_Port->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED8_Pos);
  
  USER_LED1_GPIO_Port->MODER |= (1 << GPIO_MODER_MODE11_Pos);
  USER_LED1_GPIO_Port->ODR |= (1 << 11);

  while (1)
  {
    for(int i = 100; i < 100; i++) 
      for(int a = 100; a < 100; a++)
        asm("mov r0,r0");
    USER_LED1_GPIO_Port->ODR ^= (1 << 11);
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
  // Configure HSE
  RCC->CR |= RCC_CR_HSEON;

  // Wait for HSE to be ready (Default VOS3)
  while(!(RCC->CR & RCC_CR_HSERDY)) {}

  // Select MCO1PRE (Bypass)
  RCC->CFGR |= (1 << RCC_CFGR_MCO1PRE_Pos);

  // Select HSE as SYSCLK
  RCC->CR |= (2 << RCC_CFGR_SWS_Pos); 

  // Select HSE as PLLCLK
  RCC->PLLCKSELR |= (1 << RCC_PLLCKSELR_PLLSRC_HSE_Pos);

  // DIVM1 (Bypass)
  RCC->PLLCKSELR |= (1 << RCC_PLLCKSELR_DIVM1_Pos);
  // DIVM2 (Disable)
  RCC->PLLCKSELR |= (0 << RCC_PLLCKSELR_DIVM2_Pos);
  // DIVM3 (/8)
  RCC->PLLCKSELR |= (12 << RCC_PLLCKSELR_DIVM3_Pos);

  // DIVR1
  RCC->PLLCFGR |= RCC_PLLCFGR_DIVR1EN;
    // DIVN3 = DIVM3/12
  RCC->PLL1DIVR |= 0xF << RCC_PLL1DIVR_N1_Pos;
  // P1 = /16
  RCC->PLL1DIVR |= 0x1 << RCC_PLL3DIVR_P3_Pos;
  // Disable Q1
  RCC->PLL1DIVR |= 0x0 << RCC_PLL3DIVR_Q3_Pos;
  // R1 = /4
  RCC->PLL1DIVR |= 0x3 << RCC_PLL3DIVR_R3_Pos;

  // DIVR3 (Enable for LTDC = 9MHz)
  RCC->PLLCFGR |= RCC_PLLCFGR_DIVR3EN;
  // DIVN3 = DIVM3/12
  RCC->PLL3DIVR |= 0xB << RCC_PLL3DIVR_N3_Pos;
  // R3 = DIVN3/16
  RCC->PLL3DIVR |= 0xF << RCC_PLL3DIVR_R3_Pos;
  // Disable Q3/P3
  RCC->PLL3DIVR |= 0x0 << RCC_PLL3DIVR_Q3_Pos;
  RCC->PLL3DIVR |= 0x0 << RCC_PLL3DIVR_P3_Pos;

  // Turn on PLL3
  RCC->CR |= RCC_CR_PLL3ON;
  
  // Wait for PLL3 to be ready
  while(!(RCC->CR & RCC_CR_HSERDY)) {}

  // Enable GPIO Banks A-K Clocks
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN \
                | RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIOFEN \
                | RCC_AHB4ENR_GPIOGEN | RCC_AHB4ENR_GPIOHEN | RCC_AHB4ENR_GPIOIEN \
                | RCC_AHB4ENR_GPIOJEN | RCC_AHB4ENR_GPIOKEN;

  // Enable LTDC Clock
  RCC->APB3ENR |= RCC_APB3ENR_LTDCEN;

  // Enable USART1 Clock
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
  }
}
