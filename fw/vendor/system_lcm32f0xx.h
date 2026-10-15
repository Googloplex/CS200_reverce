/**
  ******************************************************************************
  * @file    system_lcm32f0xx.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    01-August-2013
  * @brief   CMSIS Cortex-M0 Device Peripheral Access Layer System Header File.
  ******************************************************************************

  ******************************************************************************
  */


#ifndef __SYSTEM_LCM32F0XX_H
#define __SYSTEM_LCM32F0XX_H

#ifdef __cplusplus
 extern "C" {
#endif 

extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


#ifdef __cplusplus
}
#endif

#endif 

