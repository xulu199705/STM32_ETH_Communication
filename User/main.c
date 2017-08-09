/* Includes ------------------------------------------------------------------*/
#include "stm32_eth.h"
#include "netconf.h"
#include "main.h"
#include "helloworld.h"
#include "httpd.h"
#include "tftpserver.h"
#include <stdio.h>
#include "tcp.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  10
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;
uint32_t nCounter = 0;
/* Private function prototypes -----------------------------------------------*/
void System_Periodic_Handle(void);

void Delay_ARMJISHU(__IO uint32_t nCount);

extern void ARMJISHU_TouchScreen(void);
/* 显示系统运行时间 */
void LCD_Time_Update(uint32_t localtime);

/* Private functions ---------------------------------------------------------*/

void Show_Msg(void);

u16 Show_ETH_PHY(u16 PHYRegAddr);

void Check_ETH_PHY(void);

void Delay_ARMJISHU(__IO uint32_t nCount)
{
  for (; nCount != 0; nCount--);
}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{  
  System_Setup();
             
  /* Initilaize the LwIP satck */
  LwIP_Init();
  
  /* Initilaize the HelloWorld module*/
  HelloWorld_init();
  
  /* Infinite loop */
  while (1)
  {
	/* Periodic tasks */
    System_Periodic_Handle();
  }
}

void Time_Update(void)
{
  LocalTime += SYSTEMTICK_PERIOD_MS;
}

  /* 显示系统运行时间 */
void LCD_Time_Update(uint32_t localtime)
{
  static uint32_t LcdDisplayTimer=20000;
  uint32_t TimeVar;
  uint32_t THH = 0, TMM = 0, TSS = 0;
  uint8_t  TimeStr[20];

  /* 1 s */
  if (localtime >= (LcdDisplayTimer + 1000)) 
  {
    LcdDisplayTimer = localtime;
    TimeVar = LcdDisplayTimer/1000;
    /* Compute  hours */
    THH = (TimeVar / 3600)%24;
    /* Compute minutes */
    TMM = (TimeVar % 3600) / 60;
    /* Compute seconds */
    TSS = (TimeVar % 3600) % 60;

    sprintf((char *)TimeStr, " Run Time: %0.2d:%0.2d:%0.2d ", THH, TMM, TSS);
    LCD_DisplayStringLine(Line0, TimeStr);
  }
}

/** @brief  Handles the periodic tasks of the system
  * @param  None
  * @retval None
  */
void System_Periodic_Handle(void)
{
  /* Update the LCD display and the LEDs status */
  /* Manage the IP address setting */
  Display_Periodic_Handle(LocalTime);

  /* 显示系统运行时间 */
  LCD_Time_Update(LocalTime);
  
  /* LwIP periodic services are done here */
  LwIP_Periodic_Handle(LocalTime);
}

void Show_Msg(void)
{
    printf("\r\n\n\n ================           %s configured         ==============", EVAL_COM1_STR);
    printf("\n\r STM32 Connectivity Line Device\n\r");
    printf("\n\r WebServer;telnet;ping;tftp....\n\r");
    //printf("\n\r IP address is: 192.168.1.6\n\r");
}

u16 Show_ETH_PHY(u16 PHYRegAddr)
{
  u16 PHYRegData;
  PHYRegData = ETH_ReadPHYRegister(0,PHYRegAddr);
  printf("\n\rETH_ReadPHYRegister(0,%d):0x%X", PHYRegAddr, PHYRegData);
  return PHYRegData;
}

void Check_ETH_PHY(void)
{
    Show_ETH_PHY(0);
    Show_ETH_PHY(1);
    Show_ETH_PHY(2);
    Show_ETH_PHY(3);
    Show_ETH_PHY(4);
    Show_ETH_PHY(5);
    Show_ETH_PHY(6);
    if(Show_ETH_PHY(17) & 0x3000)
    {  
      /* Set Ethernet speed to 10M following the autonegotiation */    
      printf("\n\r==>ETH_Speed_10M!");
      LCD_DisplayStringLine(Line2, "     ETH_Speed_10M  ");
    }
    else
    {   
      /* Set Ethernet speed to 100M following the autonegotiation */ 
      printf("\n\r==>ETH_Speed_100M!");     
      LCD_DisplayStringLine(Line2, "     ETH_Speed_100M ");
    } 
    /* Configure the MAC with the Duplex Mode fixed by the autonegotiation process */
    if((Show_ETH_PHY(17) & 0xA000) != (uint32_t)RESET)
    {
      /* Set Ethernet duplex mode to FullDuplex following the autonegotiation */
      printf("\n\r==>ETH_Mode_FullDuplex!");
      LCD_DisplayStringLine(Line3, " ETH_Mode_FullDuplex");
    }
    else
    {
      /* Set Ethernet duplex mode to HalfDuplex following the autonegotiation */
      printf("\n\r==>ETH_Mode_HalfDuplex!");
      LCD_DisplayStringLine(Line3, " ETH_Mode_HalfDuplex");
    }
}

void MCU_TO_TCP(char *DataToSend){
	struct tcp_pcb *cpcb;
		
	for(cpcb = tcp_active_pcbs;cpcb != NULL; cpcb = cpcb->next){
		tcp_write(cpcb, DataToSend, strlen(DataToSend), TCP_WRITE_FLAG_COPY);
		tcp_output(cpcb);
	}
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
