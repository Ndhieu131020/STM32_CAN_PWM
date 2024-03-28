/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SysTick_Delay_H
#define __SysTick_Delay_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SysTick_Delay1ms(void);
void SysTick_Delay_ms(uint16_t time);

#ifdef __cplusplus
}
#endif

#endif /*__SysTick_Delay_H*/
