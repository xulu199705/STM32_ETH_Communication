#include "key.h"

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {KEY1_BUTTON_GPIO_PORT, KEY2_BUTTON_GPIO_PORT, KEY3_BUTTON_GPIO_PORT, KEY4_BUTTON_GPIO_PORT}; 
//const uint16_t BUTTON_PIN_NUM[BUTTONn] = {KEY1_BUTTON_PIN_NUM, KEY2_BUTTON_PIN_NUM, KEY3_BUTTON_PIN_NUM, KEY4_BUTTON_PIN_NUM}; 
//const uint16_t BUTTON_PIN[BUTTONn] = {KEY1_BUTTON_PIN, KEY2_BUTTON_PIN, KEY3_BUTTON_PIN, KEY4_BUTTON_PIN}; 
//const uint32_t BUTTON_CLK[BUTTONn] = {KEY1_BUTTON_GPIO_CLK, KEY2_BUTTON_GPIO_CLK, KEY3_BUTTON_GPIO_CLK, KEY4_BUTTON_GPIO_CLK};
//const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {KEY1_BUTTON_EXTI_LINE, KEY2_BUTTON_EXTI_LINE, KEY3_BUTTON_EXTI_LINE, KEY4_BUTTON_EXTI_LINE};
//const uint16_t BUTTON_PORT_SOURCE[BUTTONn] = {KEY1_BUTTON_EXTI_PORT_SOURCE, KEY2_BUTTON_EXTI_PORT_SOURCE, KEY3_BUTTON_EXTI_PORT_SOURCE, KEY4_BUTTON_EXTI_PORT_SOURCE};
//const uint16_t BUTTON_PIN_SOURCE[BUTTONn] = {KEY1_BUTTON_EXTI_PIN_SOURCE, KEY2_BUTTON_EXTI_PIN_SOURCE, KEY3_BUTTON_EXTI_PIN_SOURCE, KEY4_BUTTON_EXTI_PIN_SOURCE}; 
//const uint16_t BUTTON_IRQn[BUTTONn] = {KEY1_BUTTON_EXTI_IRQn, KEY2_BUTTON_EXTI_IRQn, KEY3_BUTTON_EXTI_IRQn, KEY4_BUTTON_EXTI_IRQn};

void delay(__IO uint32_t nCount){
	for(; nCount != 0; nCount--);
}

void KEYInit(){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	
	//按键GPIO管脚配置	物理上拉	浮空输入
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void SZ_STM32_KEYInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode){
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(BUTTON_CLK[Button] | RCC_APB2Periph_AFIO, ENABLE);
	
	//按键GPIO管脚配置	物理上拉	浮空输入
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
	GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);
	
	//按键外部中断配置
	if(Button_Mode == BUTTON_MODE_EXTI){
		GPIO_EXTILineConfig(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);
		
		EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = BUTTON_IRQn[Button];
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
}

//获取按键的输入电平状态
uint32_t SZ_STM32_KEYGetState(Button_TypeDef Button){
	return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

uint32_t SZ_STM32_KEYScan(void){
	if(0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)){
		delay(150000);
		if(0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)){
			return 1;
		}
	}
	
	if(0 == GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)){
		delay(150000);
		if(0 == GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)){
			return 2;
		}
	}
	
	if(0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)){
		delay(150000);
		if(0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)){
			return 3;
		}
	}
	
	if(0 == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)){
		delay(150000);
		if(0 == GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)){
			return 4;
		}
	}
	
	return 0;
}
