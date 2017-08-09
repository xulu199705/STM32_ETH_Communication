/**
  ******************************************************************************
  * @file    sd_stm32.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11/20/2009
  * @brief   efs µSD card driver's low level interface for STM32F107 (using 
  *          µSD card provided with STM3210C-EVAL board) 
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_spi.h"
#include "interface/sd.h"
#include "config.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Select MSD Card: ChipSelect pin low  */
#define MSD_CS_LOW()     GPIO_ResetBits(GPIOD, GPIO_Pin_11)
/* Deselect MSD Card: ChipSelect pin high */
#define MSD_CS_HIGH()    GPIO_SetBits(GPIOD, GPIO_Pin_11)

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SPI_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the SPI and GPIOs used to drive the µSD card.
  * @param  None
  * @retval None
  */
void SPI_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef   SPI_InitStructure;

  /* GPIOA and GPIOC Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  //GPIO_PinRemapConfig(GPIO_Remap_SPI3, ENABLE);

  /* SPI1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  /* Configure SPI1 pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PD11 pin: CS pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* SPI1 Config */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);

  /* SPI1 enable */
  SPI_Cmd(SPI1, ENABLE);
}

/*****************************************************************************/

esint8 if_readBuf(hwInterface* file, euint32 address, euint8* buf)
{
  return(sd_readSector(file, address, buf, 512));
}
/*****************************************************************************/

esint8 if_writeBuf(hwInterface* file, euint32 address, euint8* buf)
{
  return(sd_writeSector(file, address, buf));
}
/*****************************************************************************/

esint8 if_setPos(hwInterface* file, euint32 address)
{
  return(0);
}
/*****************************************************************************/

// Utility-functions which does not toogle CS.
// Only needed during card-init. During init
// the automatic chip-select is disabled for SSP

static euint8 my_if_spiSend(hwInterface *iface, euint8 outgoing)
{
  euint8 incoming;

  SPI_I2S_SendData(SPI1, outgoing);
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  incoming = SPI_I2S_ReceiveData(SPI1);

  return(incoming);
}
/*****************************************************************************/

void if_spiInit(hwInterface *iface)
{
  euint8 i;

  SPI_Config();

  MSD_CS_HIGH();

  /* Send 20 spi commands with card not selected */
  for (i = 0;i < 21;i++)
    my_if_spiSend(iface, 0xff);
}
/*****************************************************************************/

euint8 if_spiSend(hwInterface *iface, euint8 outgoing)
{
  euint8 incoming;

  MSD_CS_LOW();

  SPI_I2S_SendData(SPI1, outgoing);

  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  incoming = SPI_I2S_ReceiveData(SPI1);

  MSD_CS_HIGH();

  return(incoming);
}
/*****************************************************************************/

esint8 if_initInterface(hwInterface* file, eint8* opts)
{
  euint32 sc;

  if_spiInit(file); /* init at low speed */

  if (sd_Init(file) < 0)
  {
    DBG((TXT("Card failed to init, breaking up...\n")));
    printf("Card failed to init, breaking up...\n");
    return(-1);
  }
  if (sd_State(file) < 0)
  {
    DBG((TXT("Card didn't return the ready state, breaking up...\n")));
    printf("Card didn't return the ready state, breaking up...\n");
    return(-2);
  }

  sd_getDriveSize(file, &sc);
  file->sectorCount = sc / 512;
  if ( (sc % 512) != 0)
  {
    file->sectorCount--;
  }

  DBG((TXT("Drive Size is %lu Bytes (%lu Sectors)\n"), sc, file->sectorCount));
  DBG((TXT("Init done...\n")));
  printf("Drive Size is %lu Bytes (%lu Sectors)\r\n", sc, file->sectorCount);
  printf("SD Card Init done...\r\n");

  return(0);
}


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
