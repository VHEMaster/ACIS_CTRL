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
          font_printf(4,4,"RPM:   %5d",(int)StatusRPM);
          font_printf(4,4+font_getHeight(),"Ign:       %3dd", (int)StatusIgnition);
          font_printf(4,4+font_getHeight()*2,"Load:    %3d%%", (int)StatusLoad);
          font_setFont(&rre_5x8);
          font_printf(4,53,"Fuel type: %s", "fuel 1");
          if(StatusTimeout)
            font_printf(84,53,"Timeout!");
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
      break;
    default:
      break;
  }
}

inline int8_t acis_send_command(eTransChannels xChaDst, void * msgBuf, uint32_t length)
{
  int8_t status = xSender(xChaDst, (uint8_t*)msgBuf, length);
  if(status == -1)
    StatusTimeout = 1;
  else if(status == 1) StatusTimeout = 0;
  return status;
}


