#include "spiram.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"

extern SPI_HandleTypeDef hspi1;

uint32_t spi_read(uint32_t address)
{
	uint32_t cmd,out;
	uint16_t *pcmd, *pout;
	cmd = (0x3 << 24)|address;
	pcmd = (uint16_t*)(&cmd);
	pout = (uint16_t*)(&out);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	__HAL_SPI_ENABLE(&hspi1);
	pout[0] = hspi1.Instance->DR;
  hspi1.Instance->DR = pcmd[1];
  while (!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) || !__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE))
	{
	}
	pout[0] = hspi1.Instance->DR;
  hspi1.Instance->DR = pcmd[0];

	while (!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) || !__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE))
	{
	}
	pout[0] = hspi1.Instance->DR;
  hspi1.Instance->DR = 0;
  //read data
	while (!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) || !__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE))
	{
	}
	pout[1] = hspi1.Instance->DR;
  hspi1.Instance->DR = 0;
	
	while (!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) || !__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE))
	{
	}
	pout[0] = hspi1.Instance->DR;
	//pout[1] = 0;
	while (1)
  {
      /* Wait until TXE flag is set to send data */
      if(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) && !__HAL_SPI_GET_FLAG(&hspi1,SPI_FLAG_BSY))
      {
          break;
      }
  }
	__HAL_SPI_DISABLE(&hspi1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	
	return out;
}

void spi_write(uint32_t address, uint32_t data)
{
	uint32_t cmd,out;
	uint16_t *pcmd,*pout;
	cmd = (0x2 << 24)|address;
	pcmd = (uint16_t*)(&cmd);
	pout = (uint16_t*)(&data);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	__HAL_SPI_ENABLE(&hspi1);
  hspi1.Instance->DR = pcmd[1];
  while (!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE))
	{
	}
  hspi1.Instance->DR = pcmd[0];

	while (!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE))
	{
	}
  hspi1.Instance->DR = pout[1];
  //read data
	while (!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE))
	{
	}
  hspi1.Instance->DR = pout[0];
	
	while (1)
  {
      /* Wait until TXE flag is set to send data */
      if(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) && !__HAL_SPI_GET_FLAG(&hspi1,SPI_FLAG_BSY))
      {
          break;
      }
  }
	__HAL_SPI_DISABLE(&hspi1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	
}

void spi_test()
{
	uint32_t i,out;
	int t = 0;
	for(i=0;i<2097152;i++)
	{
		spi_write(i<<2,i);
	}
	for(i=0;i<2097152;i++)
	{
		out = spi_read(i<<2);
		if(out != i)
		{
			printf("read error %u\r\n", i);
			t = 1;
		}
		if(i%10000 == 0)
		{
			printf("step ok...%u\r\n",i);
		}
	}
	if(t)
	{
		printf("-------------------------test failed-----------------------------\r\n");
	}
	else
	{
		printf("-------------------------test ok-----------------------------\r\n");
	}
}