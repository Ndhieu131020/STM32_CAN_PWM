/* Includes ------------------------------------------------------------------*/
#include "SysTick_Delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function ----------------------------------------------------------*/
/**
  * @brief  Systick Delay one Milisecond Function
  * @param  None
  * @retval None
  */
void SysTick_Delay1ms(void)
{
	SysTick->LOAD = 72000-1;
	SysTick->VAL = 0;
	SysTick->CTRL = 5;
	while(!(SysTick->CTRL & (1<<16)));
}

/**
  * @brief  Systick Delay Milisecond Function
  * @param  time: Milisecond
  * @retval None
  */
void SysTick_Delay_ms(uint16_t time)
{
	while(time--)
	{
		SysTick_Delay1ms();
	}
}
