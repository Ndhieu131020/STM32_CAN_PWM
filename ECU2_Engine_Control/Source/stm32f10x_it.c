/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern CanRxMsg Rx_Msg;
extern uint16_t EngineSpeedValue;
extern 	TIM_OCInitTypeDef  TIM_OCInitStructure;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**	@brief	This function handles CAN receive interupt request
  * @param	None
	* @retval	None
  */ 
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	uint16_t High_Bit, Low_Bit, Full_Msg;
	CAN_Receive(CAN1, CAN_FIFO0, &Rx_Msg);
	
	High_Bit = ((uint16_t)Rx_Msg.Data[0]);
	Low_Bit = (uint16_t)Rx_Msg.Data[1];
	Full_Msg = (High_Bit<<8) | Low_Bit;
	
	//Caculate Engine Speed (Between 0 to 100)
	EngineSpeedValue = (Full_Msg*100)/4095;
	
	//Refer value to TIM3 PWM
	TIM_OCInitStructure.TIM_Pulse = EngineSpeedValue;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
	/* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM3, ENABLE);
	
	//Release FIFO0
	CAN_FIFORelease(CAN1, CAN_FIFO0);
	
	CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
}
