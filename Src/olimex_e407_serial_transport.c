#include "olimex_e407_serial_transport.h"
#include "stm32f4xx_hal_dma.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

bool uxr_init_serial_platform(struct uxrSerialPlatform* platform, int fd, uint8_t remote_addr, uint8_t local_addr)
{
    switch ( fd ){
        case 1:
          platform->uart = &huart1;
          break;
        case 3:
          platform->uart = &huart3;
          break;
        case 6:
          platform->uart = &huart6;
          break;
        default:
          return false;
    }
    return true;
}

bool uxr_close_serial_platform(struct uxrSerialPlatform* platform)
{
    return true;
}

size_t uxr_write_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, uint8_t* errcode)
{
  
  HAL_StatusTypeDef ret;
  ret = HAL_UART_Transmit(platform->uart, buf, len, 10);
  return ret == HAL_OK;
}

void hexprint(uint8_t* buf, size_t len){
  for (size_t i = 0; i < len; i++) { DEBUG_PRINT("%X ",buf[i]); }
  DEBUG_PRINT("\n");  
}

size_t uxr_read_serial_data_platform(uxrSerialPlatform* platform, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{ 
  HAL_UART_Receive_DMA(platform->uart, buf, len);

  osDelay(timeout);

  uint16_t receivedBytes = ((uint16_t) len) - __HAL_DMA_GET_COUNTER(platform->uart->hdmarx);

  HAL_UART_DMAStop(platform->uart);
  
  return receivedBytes;
}