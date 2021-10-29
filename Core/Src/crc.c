#include "crc.h"
#include "cmsis_os.h"

#ifdef CRC_SW

uint16_t CRC16_Generate(uint8_t * input, uint32_t size)
{
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < size; pos++)
  {
      crc ^= input[pos];
      for (int i = 8; i != 0; i--)
      {
          if ((crc & 0x0001) != 0)
          {
              crc >>= 1;
              crc ^= 0xA001;
          }
          else crc >>= 1;
      }
  }
  return crc;
}

#elif defined(CRC_HW)

static CRC_HandleTypeDef * handle_crc;

static osMutexId_t mutexCrc;
static const osMutexAttr_t mutexAttrs = {
  .name = "MutexCRC",
  .attr_bits = osMutexRecursive,
  .cb_mem = NULL,
  .cb_size = 0
};


void CRC16_RegisterHardware(CRC_HandleTypeDef * hcrc)
{
  if(mutexCrc == NULL)
    mutexCrc = osMutexNew(&mutexAttrs);
  handle_crc = hcrc;
}

inline uint16_t CRC16_Generate(uint8_t * input, uint32_t size)
{
  uint16_t result = 0;
  osStatus_t status;
  if(mutexCrc == NULL)
    mutexCrc = osMutexNew(&mutexAttrs);
  if(handle_crc != NULL)
  {
    status = osMutexAcquire(mutexCrc, 1000);
    if(status == osOK)
    {
      result = HAL_CRC_Calculate(handle_crc, (uint32_t*)input, size);
      osMutexRelease(mutexCrc);
    }
  }
  return result;
}

#endif

inline uint8_t CRC8_Generate(uint8_t * input, uint32_t size)
{
  uint16_t result = CRC16_Generate(input, size);
  return (result & 0xFF) ^ (result >> 8);
}
