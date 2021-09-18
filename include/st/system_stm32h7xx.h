/**
  ******************************************************************************
  * @file    system_stm32h7xx.h
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-Mx Device System Source File for STM32H7xx devices.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#ifndef SYSTEM_STM32H7XX_H
#define SYSTEM_STM32H7XX_H

#ifdef __cplusplus
 extern "C" {
#endif

extern uint32_t SystemCoreClock;             /*!< System Domain1 Clock Frequency  */
extern uint32_t SystemD2Clock;               /*!< System Domain2 Clock Frequency  */
extern const  uint8_t D1CorePrescTable[16] ; /*!< D1CorePrescTable prescalers table values */

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_STM32H7XX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
