#include "main.h"
#include "stm32h7b3xxq.h"

// Private Functions
void PWR_Config();

int main(void)
{
  PWR_Config();

  while (1)
  {
    
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
  PWR->SRDCR = PWR_SRDCR_VOS_0 | PWR_SRDCR_VOS_1;

  // Wait for VOS0 to be ready
  while(!(PWR->SRDCR & PWR_SRDCR_VOSRDY)) {}
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
  }
}
