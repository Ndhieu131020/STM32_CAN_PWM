/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "SysTick_Delay.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CAN_Tx_Pin 		GPIO_Pin_9
#define CAN_Rx_Pin 		GPIO_Pin_8
#define TIM3_PWM_Pin	GPIO_Pin_6
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CanRxMsg Rx_Msg;
uint16_t EngineSpeedValue;
TIM_OCInitTypeDef  TIM_OCInitStructure;
/* Private function prototypes -----------------------------------------------*/
void RCC_Config(void);
void GPIO_Config(void);
void NVIC_Config(void);
void TIM3_TimeBaseConfig(void);
void CAN_Module_Config(void);
void CAN_Filter_Config(void);
void CAN1_Message_Tx(uint32_t stdID, uint8_t DataLength, uint8_t Data[]);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/* Init Function */
	SystemCoreClockUpdate();
	RCC_Config();
	GPIO_Config();
	NVIC_Config();
	TIM3_TimeBaseConfig();
	CAN_Module_Config();
	/* User Function */
	CAN_Filter_Config();
	/* Infinite loop */
	while(1)
	{
	}
}

/**
  * @brief  RCC Config Function
  * @param  None
  * @retval None
  */
void RCC_Config(void)
{
	//Resets the RCC clock configuration to the default reset state.
	RCC_DeInit();
	//Enable HSE
	RCC_HSEConfig(RCC_HSE_ON);
	//Wait for HSE is stable
	while(RCC_WaitForHSEStartUp() != SUCCESS);
	//Power off HSI to reduce energy
	RCC_HSICmd(DISABLE);
	//HSE is selected as PLL clock entry and multify to 9
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
	//enable PLL
	RCC_PLLCmd(ENABLE);
	// Wait till PLL is ready
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	//select PLL as SYSCLK
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	//HCLK = SYSCLK
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	//APB1 = HCLK/2
	RCC_PCLK1Config(RCC_HCLK_Div2);
	//APB2 = HCLK
	RCC_PCLK2Config(RCC_HCLK_Div1);
	//Wait till PLL is used as SYSCLK source
	while(RCC_GetSYSCLKSource() != 0x08);
	//Enable clock for peripheral
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_TIM3, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

}

/**
  * @brief  GPIO Config Function
  * @param  None
  * @retval None
  */
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE); 
	
	//Configure Tim3 PWM Pin
	GPIO_InitStructure.GPIO_Pin = TIM3_PWM_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//Configure CAN1 Pin: Tx
	GPIO_InitStructure.GPIO_Pin = CAN_Tx_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//Configure CAN1 Pin: Rx
	GPIO_InitStructure.GPIO_Pin = CAN_Rx_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * @brief  CAN1 Interrupt Request Function
  * @param  None
  * @retval None
  */
void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  TIM3 PWM Mode Config Function
  * @param  None
  * @retval None
  */
void TIM3_TimeBaseConfig(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	//TIM3 Freq = 100KHz/100= 1KHz
	TIM_TimeBaseInitStructure.TIM_Prescaler = 359;
	TIM_TimeBaseInitStructure.TIM_Period = 99;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	/* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = EngineSpeedValue;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
	
	/* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM3, ENABLE);
}

/**
  * @brief  CAN1 Config Function
  * @param  None
  * @retval None
  */
void CAN_Module_Config(void)
{
	CAN_InitTypeDef CAN_InitStructure;
	
	//CAN1 Normal mode. Baudrate = 100000 bps
	CAN_InitStructure.CAN_Prescaler = 72;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_NART = ENABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	
	//Enable CAN1
	CAN_Init(CAN1, &CAN_InitStructure);
	
	//Enable CAN1 FIFO0 message pending Interrupt
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/**
  * @brief  CAN1 filter config function
	* @param  None
  * @retval None
  */
void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	
	CAN_FilterInit(&CAN_FilterInitStructure);
}

/**
  * @brief  Initiates and transmits a CAN frame message.
	* @param  stdID: Standard identifier number
  * @retval Number of mailbox that is used for transmision;
  */
void CAN1_Message_Tx(uint32_t stdID, uint8_t DataLength, uint8_t Data[])
{
	CanTxMsg TxMessageStructure;
	uint8_t index;
	
	TxMessageStructure.StdId = stdID;
	TxMessageStructure.ExtId = 0x00000000;
	TxMessageStructure.IDE = CAN_Id_Standard;
	TxMessageStructure.RTR = CAN_RTR_Data;
	TxMessageStructure.DLC = DataLength;
	for(index=0; index<8; index++)
	{
			TxMessageStructure.Data[index] = Data[index];
	}
	CAN_Transmit(CAN1, &TxMessageStructure);
}
