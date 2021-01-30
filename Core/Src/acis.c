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

static void acis_gui_task(void * argument)
{
  eMenuItem_t eMenuItem = MenuMain;
  eMenuItem_t eOldMenu = MenuUndefined;
  uint32_t display_timeout = Delay_Tick;
  uint32_t now;

  uint16_t rpm = 990;


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

        if(DelayDiff(now, display_timeout) > 300000)
        {
          display_timeout = now;
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_12x16);
          font_printf(4,4,"RPM:   %5d",rpm++);
          font_printf(4,4+font_getHeight(),"Ign:       %3dd", 180);
          font_printf(4,4+font_getHeight()*2,"Load:    %3d%%", 100);
          font_setFont(&rre_5x8);
          font_printf(4,53,"Fuel type: %s", "fuel 1");
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
    osDelay(100);
  }

}

void acis_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{

}

inline void acis_send_command(eTransChannels xChaDst, uint8_t * msgBuf, uint32_t length)
{
  xSender(xChaDst, msgBuf, length);
}


