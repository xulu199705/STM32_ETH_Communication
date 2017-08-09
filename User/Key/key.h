#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"
#include "stm32_eval.h"

#ifdef __cplusplus
	extern "C"{
#endif

/** �������� **/
//typedef enum 
//{  
//  KEY1 = 0,
//  KEY2 = 1,
//  KEY3 = 2,  //Tamper
//  KEY4 = 3   //Wakeup
//} Button_TypeDef;

/** ����ģʽ���壬��ѯģʽ���ж�ģʽ **/
typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

/** �����ܽ���Դ��������  **/
#define BUTTONn                          4

/** KEY�����ܽ���Դ����    ��������ʱ����͵�ƽ �����ͷ�ʱ����ߵ�ƽ **/

/** KEY1�����ܽ�  **/
#define KEY1_BUTTON_PIN_NUM              4 
#define KEY1_BUTTON_PIN                  GPIO_Pin_4
#define KEY1_BUTTON_GPIO_PORT            GPIOC
#define KEY1_BUTTON_GPIO_CLK             RCC_APB2Periph_GPIOC
#define KEY1_BUTTON_EXTI_LINE            EXTI_Line4
#define KEY1_BUTTON_EXTI_PORT_SOURCE     GPIO_PortSourceGPIOC
#define KEY1_BUTTON_EXTI_PIN_SOURCE      GPIO_PinSource4
#define KEY1_BUTTON_EXTI_IRQn            EXTI4_IRQn  
#define KEY1IBB                          Periph_BB((uint32_t)&KEY1_BUTTON_GPIO_PORT->IDR, KEY1_BUTTON_PIN_NUM) //�ȼ���Periph_BB((uint32_t)&GPIOC->IDR, 4)

/** KEY2�����ܽ�  **/
#define KEY2_BUTTON_PIN_NUM              10
#define KEY2_BUTTON_PIN                  GPIO_Pin_10
#define KEY2_BUTTON_GPIO_PORT            GPIOB
#define KEY2_BUTTON_GPIO_CLK             RCC_APB2Periph_GPIOB
#define KEY2_BUTTON_EXTI_LINE            EXTI_Line10
#define KEY2_BUTTON_EXTI_PORT_SOURCE     GPIO_PortSourceGPIOB
#define KEY2_BUTTON_EXTI_PIN_SOURCE      GPIO_PinSource10
#define KEY2_BUTTON_EXTI_IRQn            EXTI15_10_IRQn  
#define KEY2IBB                          Periph_BB((uint32_t)&KEY2_BUTTON_GPIO_PORT->IDR, KEY2_BUTTON_PIN_NUM)

/** KEY3����ͬʱҲ��Tamper�ܽ�  **/
#define KEY3_BUTTON_PIN_NUM              13
#define KEY3_BUTTON_PIN                  GPIO_Pin_13
#define KEY3_BUTTON_GPIO_PORT            GPIOC
#define KEY3_BUTTON_GPIO_CLK             RCC_APB2Periph_GPIOC
#define KEY3_BUTTON_EXTI_LINE            EXTI_Line13
#define KEY3_BUTTON_EXTI_PORT_SOURCE     GPIO_PortSourceGPIOC
#define KEY3_BUTTON_EXTI_PIN_SOURCE      GPIO_PinSource13
#define KEY3_BUTTON_EXTI_IRQn            EXTI15_10_IRQn 
#define KEY3IBB                          Periph_BB((uint32_t)&KEY3_BUTTON_GPIO_PORT->IDR, KEY3_BUTTON_PIN_NUM)

/** KEY4����ͬʱҲ��Wakeup�ܽ�  **/
#define KEY4_BUTTON_PIN_NUM              0
#define KEY4_BUTTON_PIN                  GPIO_Pin_0
#define KEY4_BUTTON_GPIO_PORT            GPIOA
#define KEY4_BUTTON_GPIO_CLK             RCC_APB2Periph_GPIOA
#define KEY4_BUTTON_EXTI_LINE            EXTI_Line0
#define KEY4_BUTTON_EXTI_PORT_SOURCE     GPIO_PortSourceGPIOA
#define KEY4_BUTTON_EXTI_PIN_SOURCE      GPIO_PinSource0
#define KEY4_BUTTON_EXTI_IRQn            EXTI0_IRQn 
#define KEY4IBB                          Periph_BB((uint32_t)&KEY4_BUTTON_GPIO_PORT->IDR, KEY4_BUTTON_PIN_NUM)

void delay(__IO uint32_t nCount);
void KEYInit(void);
void SZ_STM32_KEYInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t SZ_STM32_KEYScan(void);
uint32_t SZ_STM32_KEYGetState(Button_TypeDef Button);

#ifdef __cpluscplus
	}
#endif

#endif
