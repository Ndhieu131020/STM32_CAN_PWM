/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "Systick_Delay.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_ADDRESS ((uint32_t)0x4001204C)

#define CAN_TX_Source GPIO_PinSource1
#define CAN_RX_Source GPIO_PinSource0
#define CAN_TX_Pin GPIO_Pin_1
#define CAN_RX_Pin GPIO_Pin_0

#define CAN_SpeedMsg_ID ((uint32_t)0x112)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t ADC1ConvertedValue;
/* Private function prototypes -----------------------------------------------*/
void RCC_Config(void);
void GPIO_Config(void);
void ADC_Config(void);
void DMA_Config(void);
void NVIC_Config(void);
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
	DMA_Config();
	ADC_Config();
	NVIC_Config();
	CAN_Module_Config();
	/* User Function */
	CAN_Filter_Config();
	/* Infinite loop */
	while(1)
	{
		//Start ADC Conversion each 100ms
		ADC_SoftwareStartConv(ADC1);
		SysTick_Delay_ms(200);
	}
}

/**
  * @brief  RCC Config Function
  * @param  None
  * @retval None
  */
void RCC_Config(void)
{
	//Resets the RCC clock configuration to the default reset state
	RCC_DeInit();
	//Disable HSI to reduce energy
	RCC_HSICmd(DISABLE);
	//Enable HSE
	RCC_HSEConfig(RCC_HSE_ON);
	//Wait for HSE oscillator is stable and ready to use
	while(RCC_WaitForHSEStartUp() != SUCCESS);
	//PLL Param
	RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 4);
	//Enable PLL
	RCC_PLLCmd(ENABLE);
	//Wait for PLL Clock is Ready to use
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != SET);
	//Set PLLCLK as SYSCLK
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	//HCLK = SYSCLK
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	//APB1 = HCLK/4
	RCC_PCLK1Config(RCC_HCLK_Div4);
	//APB2 = HCLK/2
	RCC_PCLK2Config(RCC_HCLK_Div2);
	//Check if PLL Clock used as system clock.
	while(RCC_GetSYSCLKSource() != 0x08);
	//Enable clock for Peripheral
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOA |RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}

/**
  * @brief  GPIO Config Function
  * @param  None
  * @retval None
  */
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//Configure LED_Builtin Pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//Configure ADC1 Channel0 pin as analog input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//CAN1 Pin configuration
	GPIO_PinAFConfig(GPIOD, CAN_TX_Source, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD, CAN_RX_Source, GPIO_AF_CAN1);
	
	GPIO_InitStructure.GPIO_Pin = CAN_RX_Pin | CAN_TX_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
/**
  * @brief  DMA1 Config Function
  * @param  None
  * @retval None
  */
void DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	//DMA2 Stream0 channel2 configuration
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	//Enable DMA Interupt
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
	
  DMA_Cmd(DMA2_Stream0, ENABLE);
}

/**
  * @brief  ADC1 Config Function
  * @param  None
  * @retval None
  */
void ADC_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	
	ADC_CommonInit(&ADC_CommonInitStructure);
	//Configure ADC1 Channel0
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	
	ADC_Init(ADC1, &ADC_InitStructure);
	//ADC1 regular channel0 configuration
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles);
	//Enable DMA request after last transfer (Single-ADC mode)
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	//Enable ADC1 DMA
	ADC_DMACmd(ADC1, ENABLE);
	//Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
}

/**
  * @brief  NVIC Config Function
  * @param  None
  * @retval None
  */
void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	//Enable Interupt for DMA2 Channel0
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
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
	CAN_InitStructure.CAN_Prescaler = 84;
	CAN_InitStructure.CAN_Mode = CAN_OperatingMode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	
	CAN_Init(CAN1, &CAN_InitStructure);
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
	for(index=0; index<=8; index++)
	{
			TxMessageStructure.Data[index] = Data[index];
	}
	
	CAN_Transmit(CAN1, &TxMessageStructure);
}
