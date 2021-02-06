/*
 * acis.c
 *
 *  Created on: Jan 26, 2021
 *      Author: VHEMaster
 */

#include "acis.h"
#include "controls.h"
#include "lcd12864.h"
#include "RREFont.h"
#include "xCommand.h"
#include "packets.h"
#include <string.h>

osThreadId_t tGuiHandler;
static const osThreadAttr_t cTaskAttributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};

typedef enum
{
  MenuUndefined = 0,
  MenuMain,
}eMenuItem_t;

volatile float StatusRPM = 0;
volatile float StatusLoad = 0;
volatile float StatusPressure = 0;
volatile float StatusIgnition = 0;
volatile float StatusVoltage = 0;
volatile uint8_t StatusTableNum = 0;
volatile uint8_t StatusValveNum = 0;
volatile uint8_t StatusCheck = 0;
char StatusTableName[TABLE_STRING_MAX] = {0};

volatile uint8_t StatusTimeout = 0;

static int8_t acis_send_command(eTransChannels xChaDst, void * msgBuf, uint32_t length);

static void acis_gui_task(void * argument)
{
  eMenuItem_t eMenuItem = MenuMain;
  eMenuItem_t eOldMenu = MenuUndefined;
  uint32_t display_timeout = Delay_Tick;
  uint32_t now;


  while(1)
  {
    if(StatusValveNum == 1)
    {
      HAL_GPIO_WritePin(LED1R_GPIO_Port, LED1R_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED1G_GPIO_Port, LED1G_Pin, GPIO_PIN_SET);
    }
    else if(StatusValveNum == 2)
    {
      HAL_GPIO_WritePin(LED1R_GPIO_Port, LED1R_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED1G_GPIO_Port, LED1G_Pin, GPIO_PIN_RESET);
    }
    else
    {
      HAL_GPIO_WritePin(LED1R_GPIO_Port, LED1R_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED1G_GPIO_Port, LED1G_Pin, GPIO_PIN_SET);
    }

    if(StatusCheck)
      HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);

    now = Delay_Tick;
    if(eMenuItem != eOldMenu)
    {
    }
    else
    {
      osDelay(1);
    }

    switch (eMenuItem)
    {
      case MenuMain :
      {

        if(DelayDiff(now, display_timeout) > 150000)
        {
          display_timeout = now;
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_12x16);
          font_printf(4,4,"RPM:   %5.0f",StatusRPM);
          font_printf(4,4+font_getHeight(),"Load:    %3.0f%%", StatusLoad);
          font_printf(4,4+font_getHeight()*2,"Ign:       %3.0fd", StatusIgnition);
          font_setFont(&rre_5x8);
          font_printf(4,53,"%d: %-11s", StatusTableNum+1, StatusTableName);
          if(StatusTimeout)
            font_printf(72,53,"Timeout!");
          else
            font_printf(72,53,"U: %4.1fV",StatusVoltage);
          lcd_update();
        }

        break;
      }
      default :
        eMenuItem = MenuMain;
        break;
    }

    eOldMenu = eMenuItem;
  }
}

void acis_main_task(void * argument)
{
  tGuiHandler = osThreadNew(acis_gui_task, NULL, &cTaskAttributes);
  while(1)
  {
    while(acis_send_command(etrACIS, &PK_GeneralStatusRequest, sizeof(PK_GeneralStatusRequest)) <= 0)
      osDelay(1);

    osDelay(50);
  }

}

void acis_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{
  switch(msgBuf[0])
  {
    case PK_GeneralStatusResponseID :
      PK_Copy(&PK_GeneralStatusResponse, msgBuf);
      StatusIgnition = PK_GeneralStatusResponse.IgnitionAngle;
      StatusLoad = PK_GeneralStatusResponse.Load;
      StatusRPM = PK_GeneralStatusResponse.RPM;
      StatusPressure = PK_GeneralStatusResponse.Pressure;
      StatusVoltage = PK_GeneralStatusResponse.Voltage;
      StatusCheck = PK_GeneralStatusResponse.check;
      StatusValveNum = PK_GeneralStatusResponse.valvenum;
      StatusTableNum = PK_GeneralStatusResponse.tablenum;
      strcpy(StatusTableName, PK_GeneralStatusResponse.tablename);
      break;
    default:
      break;
  }
}

inline int8_t acis_send_command(eTransChannels xChaDst, void * msgBuf, uint32_t length)
{
  int8_t status = xSender(xChaDst, (uint8_t*)msgBuf, length);
  if(status == -1)
  {
    StatusTimeout = 1;
    StatusCheck = 1;
  }
  else if(status == 1) StatusTimeout = 0;
  return status;
}


