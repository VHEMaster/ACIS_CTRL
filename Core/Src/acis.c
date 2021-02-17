/*
 * acis.c
 *
 *  Created on: Jan 26, 2021
 *      Author: VHEMaster
 */

#include "acis.h"
#include "delay.h"
#include "controls.h"
#include "lcd12864.h"
#include "RREFont.h"
#include "xCommand.h"
#include "packets.h"
#include "crc.h"
#include <math.h>
#include <string.h>

HAL_StatusTypeDef config_default(sAcisConfig * config);

#define Delay(x) osDelay((x)*10);
#define TableOffset(x, type) (*(type*)(((uint32_t)x)+(((uint32_t)&acis_config.tables[1]-(uint32_t)&acis_config.tables[0])*GuiTableEntry)))

osThreadId_t tGuiHandler;
osThreadId_t tSenderHandler;
static const osThreadAttr_t cTaskAttributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};

typedef enum
{
  MenuUndefined = 0,
  MenuSynchronizing,
  MenuSyncError,
  MenuPcConnected,
  MenuMain,
  MenuMainDrag,
  MenuMainConfig,
  MenuTableSelect,
  MenuTableConfig,
  MenuTableSetup,
  MenuMainLast,
  MenuTableSetupPressures,
  MenuTableSetupRotates,
  MenuTableSetupIdleRotates,
  MenuTableSetupIdleIgnitions,
  MenuTableSetupTemperatures,
  MenuTableSetupServoAccel,
  MenuTableSetupServoChoke,
  MenuTableSetupIgnitionsSelect,
  MenuTableSetupIgnitions,
  MenuTableConfigSave,
  MenuTableConfigRestore,
}eMenuItem_t;

typedef struct
{
    const char * name;
    char * values;
    int32_t * valuei;
    float * valuef;
    int32_t min;
    int32_t max;
    float step;
    int32_t guicorrective;
    float guimultiplier;
    eMenuItem_t menuitem;
    const char * title;
    float * valuedep;
}sConfigLinking;

static sAcisConfig acis_config;

#define SENDING_BUFFER_SIZE (MAX_PACK_LEN)
static uint8_t buffSendingBuffer[SENDING_BUFFER_SIZE];

#define SENDING_QUEUE_SIZE (MAX_PACK_LEN*4)
static uint8_t buffSendingQueue[SENDING_QUEUE_SIZE];
static sProFIFO fifoSendingQueue;

volatile eMenuItem_t eMenuItem = MenuUndefined;

volatile float StatusRPM = 0;
volatile float StatusLoad = 0;
volatile float StatusPressure = 0;
volatile float StatusIgnition = 0;
volatile float StatusVoltage = 0;;
volatile float StatusTemperature = 0;
volatile uint32_t StatusTableNum = 0;
volatile uint8_t StatusValveNum = 0;
volatile uint8_t StatusCheck = 0;
volatile uint8_t StatusSynchronized = 0;
volatile uint8_t StatusSynchronizing = 0;
volatile uint32_t StatusPcLast = 0;
volatile uint8_t StatusPcConnected = 0;

volatile uint8_t NeedSave = 0;
volatile uint8_t NeedLoad = 0;

volatile uint8_t SyncStep = 0;
volatile uint8_t SyncRequestDone = 0;
volatile uint8_t FlashRequestDone = 0;
volatile uint32_t SyncSize = 0;
volatile uint32_t SyncLeft = 0;
volatile uint32_t SyncOffset = 0;
volatile uint8_t SyncError = 0;

volatile uint8_t ApplyError = 0;
volatile uint8_t Applying = 0;

volatile float DragRpmFrom = 2000;
volatile float DragRpmTo = 4000;
volatile float DragTime = 0;
volatile uint8_t DragStatus = 0;
volatile uint8_t DragGraphReady = 0;
volatile uint32_t DragPointsRawCountPtr = 0;
volatile uint32_t DragPointsRawCount = 0;

static sDragPoint DragPointsRaw[DRAG_MAX_POINTS];

volatile int32_t GuiTableEntry = 0;

const char * lcd_chars = " ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-+_/";

const sConfigLinking DragConfig[] =
{
    {.name = "RPM From", .min = 500, .max = 9900, .step = 100, .valuei = NULL, .valuef = (float*)&DragRpmFrom, .guicorrective = 0, .guimultiplier = 1.0f, },
    {.name = "RPM To", .min = 500, .max = 9900, .step = 100, .valuei = NULL, .valuef = (float*)&DragRpmTo, .guicorrective = 0, .guimultiplier = 1.0f, }
};

const sConfigLinking CommonConfig[] =
{
    {.name = "Temp. compensation", .min = 0, .max = 1, .step = 1, .valuei = &acis_config.params.isTemperatureEnabled, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f, },
    {.name = "Economizer Enabled", .min = 0, .max = 1, .step = 1, .valuei = &acis_config.params.isEconomEnabled, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f, },
    {.name = "Ignition by Hall", .min = 0, .max = 1, .step = 1, .valuei = &acis_config.params.isIgnitionByHall, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f, },
    {.name = "Hall Learning Mode", .min = 0, .max = 1, .step = 1, .valuei = &acis_config.params.isHallLearningMode, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f, },
    {.name = "Econ out as Strobe", .min = 0, .max = 1, .step = 1, .valuei = &acis_config.params.isEconOutAsStrobe, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f, },
    {.name = "Force Table mode", .min = 0, .max = 1, .step = 1, .valuei = &acis_config.params.isForceTable, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f, },
    {.name = "Force Table number", .min = 0, .max = 3, .step = 1, .valuei = &acis_config.params.forceTableNumber, .valuef = NULL, .guicorrective = 1, .guimultiplier = 1.0f, },
    {.name = "External Fuel Switch", .min = 0, .max = 1, .step = 1, .valuei = &acis_config.params.isSwitchByExternal, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f, },
    {.name = "Switch Pos1 Table", .min = 0, .max = 3, .step = 1, .valuei = &acis_config.params.switchPos1Table, .valuef = NULL, .guicorrective = 1, .guimultiplier = 1.0f, },
    {.name = "Switch Pos0 Table", .min = 0, .max = 3, .step = 1, .valuei = &acis_config.params.switchPos0Table, .valuef = NULL, .guicorrective = 1, .guimultiplier = 1.0f, },
    {.name = "Switch Pos2 Table", .min = 0, .max = 3, .step = 1, .valuei = &acis_config.params.switchPos2Table, .valuef = NULL, .guicorrective = 1, .guimultiplier = 1.0f, },
    {.name = "Economizer RPM", .min = 1000, .max = 3000, .step = 100, .valuei = NULL, .valuef = &acis_config.params.EconRpmThreshold, .guicorrective = 0, .guimultiplier = 1.0f, },
    {.name = "Cutoff Enabled", .min = 0, .max = 1, .step = 1, .valuei = &acis_config.params.isCutoffEnabled, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f, },
    {.name = "Cutoff Mode", .min = 0, .max = 7, .step = 1, .valuei = &acis_config.params.CutoffMode, .valuef = NULL, .guicorrective = 1, .guimultiplier = 1.0f, },
    {.name = "Cutoff RPM", .min = 2000, .max = 9900, .step = 100, .valuei = NULL, .valuef = &acis_config.params.CutoffRPM, .guicorrective = 0, .guimultiplier = 1.0f, },
    //{.name = "Autostart Support", .min = 0, .max = 1, .step = 1, .valuei = &acis_config.params.isAutostartEnabled, .valuef = NULL, .guicorrective = 0, },
};

const sConfigLinking TableInitial[] =
{
    {.name = "Tables Count", .min = 1, .max = TABLE_SETUPS_MAX, .step = 0, .valuei = &acis_config.tables_count, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f, .menuitem = MenuUndefined },
    {.name = "Current Table", .min = 0, .max = TABLE_SETUPS_MAX - 1, .step = 0, .valuei = (int32_t*)&StatusTableNum, .valuef = NULL, .guicorrective = 1, .guimultiplier = 1.0f, .menuitem = MenuUndefined },
    {.name = "Table Setup Entry", .min = 0, .max = TABLE_SETUPS_MAX - 1, .step = 1, .valuei = (int32_t*)&GuiTableEntry, .valuef = NULL, .guicorrective = 1, .guimultiplier = 1.0f, .menuitem = MenuUndefined },
    {.name = "Save Flash", .valuei = NULL, .valuef = NULL, .menuitem = MenuTableConfigSave },
    {.name = "Restore Flash", .valuei = NULL, .valuef = NULL, .menuitem = MenuTableConfigRestore },
};

const sConfigLinking TableSetup[] =
{
    {.name = "Ignitions", .min = -45, .max = 90, .step = 0.2, .valuedep = NULL, .valuef = NULL, .valuei = (int32_t*)&acis_config.tables[0], .menuitem = MenuTableSetupIgnitionsSelect },
    {.name = "Pressures (%d)", .min = 10000, .max = 200000, .step = 200, .valuedep = NULL, .valuei = &acis_config.tables[0].pressures_count, .valuef = acis_config.tables[0].pressures, .menuitem = MenuTableSetupPressures, .title = "Pressure %d" },
    {.name = "Rotates (%d)", .min = 100, .max = 10000, .step = 50, .valuedep = NULL, .valuei = &acis_config.tables[0].rotates_count, .valuef = acis_config.tables[0].rotates, .menuitem = MenuTableSetupRotates, .title = "RPM %d" },
    {.name = "Idle Rotates (%d)", .min = 100, .max = 10000, .step = 10, .valuedep = NULL, .valuei = &acis_config.tables[0].idles_count, .valuef = acis_config.tables[0].idle_rotates, .menuitem = MenuTableSetupIdleRotates, .title = "Idle RPM %d" },
    {.name = "Idle Ignitions (%d)", .min = -45, .max = 90, .step = 0.2, .valuedep = acis_config.tables[0].idle_rotates, .valuei = &acis_config.tables[0].idles_count, .valuef = acis_config.tables[0].idle_ignitions, .menuitem = MenuTableSetupIdleIgnitions, .title = "Ign.%d (%.0f RPM)" },
    {.name = "Temperatures  (%d)", .min = -40, .max = 150, .step = 1, .valuedep = NULL, .valuei = &acis_config.tables[0].temperatures_count, .valuef = acis_config.tables[0].temperatures, .menuitem = MenuTableSetupTemperatures, .title = "Temp.%d" },
    {.name = "Servo Accel.(%d)", .min = 0, .max = 100, .step = 0.5, .valuedep = acis_config.tables[0].temperatures, .valuei = &acis_config.tables[0].temperatures_count, .valuef = acis_config.tables[0].servo_acc, .menuitem = MenuTableSetupServoAccel, .title = "Accel.%d (%.0f)" },
    {.name = "Servo Choke (%d)", .min = 0, .max = 100, .step = 0.5, .valuedep = acis_config.tables[0].temperatures, .valuei = &acis_config.tables[0].temperatures_count, .valuef = acis_config.tables[0].servo_choke, .menuitem = MenuTableSetupServoChoke, .title = "Choke %d (%.0f)" },
};

const sConfigLinking TableConfig[] =
{
    {.name = "Name", .valuei = NULL, .valuef = NULL, .values = acis_config.tables[0].name },
    {.name = "Initial", .min = -10, .max = 45, .step = 0.2f, .values = NULL, .valuei = NULL, .valuef = &acis_config.tables[0].initial_ignition, .guicorrective = 0, .guimultiplier = 1.0f },
    {.name = "Octane Cor.", .min = -10, .max = 45, .step = 0.2f, .values = NULL, .valuei = NULL, .valuef = &acis_config.tables[0].octane_corrector, .guicorrective = 0, .guimultiplier = 1.0f },
    {.name = "Valve Ch.", .min = (int32_t)ValveAllClosed, .max = (int32_t)ValvePropane, .step = 1, .values = NULL, .valuei = (int32_t*)&acis_config.tables[0].valve_channel, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f },
    {.name = "Valve Timeout(mS)", .min = 0, .max = 5000, .step = 100, .values = NULL, .valuei = &acis_config.tables[0].valve_timeout, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f },
    {.name = "Idles Cnt.", .min = 0, .max = TABLE_ROTATES_MAX, .step = 1, .values = NULL, .valuei = &acis_config.tables[0].idles_count, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f },
    {.name = "Pressures Cnt.", .min = 0, .max = TABLE_PRESSURES_MAX, .step = 1, .values = NULL, .valuei = &acis_config.tables[0].pressures_count, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f },
    {.name = "Rotates Cnt.", .min = 0, .max = TABLE_ROTATES_MAX, .step = 1, .values = NULL, .valuei = &acis_config.tables[0].rotates_count, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f },
    {.name = "Temps Cnt.", .min = 0, .max = TABLE_TEMPERATURES_MAX, .step = 1, .values = NULL, .valuei = &acis_config.tables[0].temperatures_count, .valuef = NULL, .guicorrective = 0, .guimultiplier = 1.0f },
};

char StatusTableName[TABLE_STRING_MAX] = {0};

volatile uint8_t StatusTimeout = 0;

static HAL_StatusTypeDef acis_send_command(eTransChannels xChaDst, void * msgBuf, uint32_t length);

static HAL_StatusTypeDef acis_apply_parameter(void * parameter, int size)
{
  HAL_StatusTypeDef status = HAL_ERROR;

  uint32_t addr = (uint32_t)parameter;
  uint32_t config_addr_start = (uint32_t)&acis_config;
  uint32_t config_addr_end = (uint32_t)&acis_config.tables[0];
  uint32_t tables_addr_start[TABLE_SETUPS_MAX];
  uint32_t tables_addr_end[TABLE_SETUPS_MAX];
  uint32_t configsize = config_addr_end - config_addr_start;
  uint32_t tablesize = sizeof(sAcisIgnTable);
  for(int i = 0; i < TABLE_SETUPS_MAX; i++)
  {
    tables_addr_start[i] = (uint32_t)&acis_config.tables[i];
    tables_addr_end[i] = (uint32_t)&acis_config.tables[i] + sizeof(sAcisIgnTable);
  }

  if(addr >= config_addr_start && addr < config_addr_end)
  {
    PK_ConfigMemoryData.Destination = etrACIS;
    PK_ConfigMemoryData.configsize = configsize;
    PK_ConfigMemoryData.offset = addr - config_addr_start;
    PK_ConfigMemoryData.size = sizeof(uint32_t);

    memcpy(&PK_ConfigMemoryData.data[0], &((uint8_t*)&acis_config)[PK_ConfigMemoryData.offset], PK_ConfigMemoryData.size);
    memset(&PK_ConfigMemoryData.data[PK_ConfigMemoryData.size], 0, sizeof(PK_ConfigMemoryData.data) - PK_ConfigMemoryData.size);
    PK_ConfigMemoryData.crc = CRC16_Generate(PK_ConfigMemoryData.data, sizeof(PK_ConfigMemoryData.data));

    Applying = 1;
    protPushSequence(&fifoSendingQueue, &PK_ConfigMemoryData, sizeof(PK_ConfigMemoryData));
    status = HAL_OK;
  }
  else
  {
    for(int i = 0; i < TABLE_SETUPS_MAX; i++)
    {
      if(addr >= tables_addr_start[i] && addr < tables_addr_end[i])
      {
        PK_TableMemoryData.Destination = etrACIS;
        PK_TableMemoryData.tablesize = tablesize;
        PK_TableMemoryData.table = i;
        PK_TableMemoryData.offset = addr - tables_addr_start[i];
        PK_TableMemoryData.size = size;

        memcpy(&PK_TableMemoryData.data[0], &((uint8_t*)&acis_config.tables[i])[PK_TableMemoryData.offset], PK_TableMemoryData.size);
        memset(&PK_TableMemoryData.data[PK_TableMemoryData.size], 0, sizeof(PK_TableMemoryData.data) - PK_TableMemoryData.size);
        PK_TableMemoryData.crc = CRC16_Generate(PK_TableMemoryData.data, sizeof(PK_TableMemoryData.data));

        Applying = 1;
        protPushSequence(&fifoSendingQueue, &PK_TableMemoryData, sizeof(PK_TableMemoryData));
        status = HAL_OK;
        break;
      }
    }
  }

  //if(status != HAL_OK)
    //ApplyError = 1;

  return status;
}

static void acis_gui_task(void * argument)
{
  sAcisIgnTable * table;
  eMenuItem_t eOldMenu = MenuUndefined;
  uint32_t check_last = 0;
  uint32_t display_timeout = Delay_Tick;
  uint32_t value_timeout = 0;
  uint32_t select_timeout = 0;
  uint32_t last_menu_switch = Delay_Tick;
  uint32_t now;
  uint8_t cnt = 0;

  int32_t menuitem = 0;
  uint32_t menufirst = 0;
  int32_t menuitem2 = 0;
  uint32_t menufirst2 = 0;
  int32_t menuitem3 = 0;
  uint32_t menufirst3 = 0;
  int32_t menuitem4 = 0;
  uint32_t menufirst4 = 0;

  int32_t stringchar = 0;
  uint32_t menuselecting = 0;
  uint32_t menuselected = 0;
  char tablestring[TABLE_STRING_MAX];
  int16_t tablechars[TABLE_STRING_MAX];
  uint16_t lcd_chars_len = strlen(lcd_chars);
  const sConfigLinking * tablesetupitem = NULL;
  const char * tablesetuptitle = NULL;
  float rpm = 0;
  float pres = 0;
  float ign = 0;

  //config_default(&acis_config);

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

    if(DragStatus == 1)
    {
      if(DelayDiff(now, check_last) < 400000)
      {
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_RESET);
      }
      else if(DelayDiff(now, check_last) < 800000)
      {
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_SET);
      }
      else
      {
        check_last = now;
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_RESET);
      }
    }
    else if(DragStatus == 2)
    {
      if(DelayDiff(now, check_last) < 200000)
      {
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_RESET);
      }
      else if(DelayDiff(now, check_last) < 400000)
      {
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_SET);
      }
      else
      {
        check_last = now;
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_RESET);
      }
    }
    else if(DragStatus == 3)
    {
      HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_RESET);
    }
    else if(DragStatus == 4)
    {
      HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_RESET);
    }
    else if(StatusCheck)
    {
      if(DelayDiff(now, check_last) < 1000000)
      {
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_SET);
      }
      else if(DelayDiff(now, check_last) < 2000000)
      {
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_SET);
      }
      else
      {
        check_last = now;
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_SET);
      }
    }
    else if(!StatusSynchronized)
    {
      if(DelayDiff(now, check_last) < 1000000)
      {
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_RESET);
      }
      else if(DelayDiff(now, check_last) < 2000000)
      {
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_SET);
      }
      else
      {
        check_last = now;
        HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_RESET);
      }
    }
    else
    {
      HAL_GPIO_WritePin(LED1R_GPIO_Port, LED2R_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED1G_GPIO_Port, LED2G_Pin, GPIO_PIN_SET);
      check_last = now;
    }

    now = Delay_Tick;

    if(BUT_CANCEL_PRESS && BUT_CANCEL_TIME >= 1000 && eMenuItem > MenuMain)
    {
      BUT_CANCEL_TIME = 0;
      eMenuItem = MenuMain;
    }

    if(StatusPcConnected)
    {
      eMenuItem = MenuPcConnected;
    }
    else if(StatusSynchronizing)
    {
      eMenuItem = MenuSynchronizing;
    }
    else
    {
      if(SyncError)
      {
        eMenuItem = MenuSyncError;
      }
      else if(eMenuItem == MenuSynchronizing)
        eMenuItem = MenuMain;

      if(eMenuItem == MenuPcConnected)
      {
        eMenuItem = MenuSynchronizing;
        StatusSynchronizing = 1;
        SyncStep = 0;
        SyncError = 0;
        SyncSize = 0;
      }
    }

    if(eMenuItem != eOldMenu && eMenuItem == MenuTableConfigRestore)
    {
      SyncError = 0;
      SyncRequestDone = 0;
      FlashRequestDone = 0;
      NeedLoad = 1;
    }

    if(eMenuItem != eOldMenu && eMenuItem == MenuTableConfigSave)
    {
      SyncError = 0;
      SyncRequestDone = 0;
      FlashRequestDone = 0;
      NeedSave = 1;
    }

    if(eMenuItem != eOldMenu && eOldMenu < MenuMainLast && eMenuItem < MenuTableSetup)
    {
      last_menu_switch = now;
      display_timeout = 0;
      BUT_UP = 0;
      BUT_DOWN = 0;
      BUT_LEFT = 0;
      BUT_RIGHT = 0;
      BUT_ENTER = 0;
      BUT_CANCEL = 0;
      BUT_UP_TIME = 0;
      BUT_DOWN_TIME = 0;
      BUT_LEFT_TIME = 0;
      BUT_RIGHT_TIME = 0;
      BUT_ENTER_TIME = 0;
      BUT_CANCEL_TIME = 0;
      menuselecting = 0;
      menufirst = 0;
      menuitem = 0;
      DragStatus = 0;
    }
    else
    {
      Delay(1);
    }

    eOldMenu = eMenuItem;

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
          font_printf(4,4,"RPM:");
          font_printf(-125,4,"%03.0f",StatusRPM);
          font_printf(4,4+font_getHeight(),"Pres:");
          font_printf(-125,4+font_getHeight(),"%04.0f", StatusPressure);
          font_printf(4,4+font_getHeight()*2,"Ign:");
          font_printf(-125,4+font_getHeight()*2,"%3.0fd", StatusIgnition);
          font_setFont(&rre_5x8);
          font_printf(4,53,"%d: %-11s", StatusTableNum+1, StatusTableName);

          if(StatusTimeout)
            font_printf(-125,53,"Timeout!");
          else
          {
            font_printf(-125,53,"%4.1fV", StatusVoltage);
            //font_printf(-95,53,"%02.0f%%",StatusLoad);
            font_printf(-95,53,"%3.0fC",StatusTemperature);
          }

          lcd_update();
        }
        if(BUT_RIGHT) BUT_RIGHT = 0, eMenuItem++;
        else if(BUT_LEFT) BUT_LEFT = 0, eMenuItem = MenuMainLast - 1;
        break;
      }

      case MenuMainDrag :
      {
        if(DelayDiff(now, display_timeout) > 30000)
        {
          display_timeout = now;
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_8x12);
          font_printf(0,2,"Drag Measure");
          lcd_rect(0,15,128,1,1);

          if(DragStatus <= 2)
          {
            font_setFont(&rre_5x8);
            for(int i = 0; i < sizeof(DragConfig) / sizeof(sConfigLinking); i++)
            {

              font_printf(3,18 + font_getHeight()*i,"%s:", DragConfig[i].name);
              if(DragConfig[i].valuef)
              {
                if((!menuselecting || menuitem != i))
                  font_printf(-125,18 + font_getHeight()*i,"%03.0f", *DragConfig[i].valuef);
                else if(menuitem == i && DelayDiff(now, select_timeout) < 500000)
                  font_printf(-125,18 + font_getHeight()*i,"> %03.0f", *DragConfig[i].valuef);
                else if(DelayDiff(now, select_timeout) > 800000)
                  select_timeout = now;
              }
              else if(DragConfig[i].valuei)
              {
                if((!menuselecting || menuitem != i))
                  font_printf(-125,18 + font_getHeight()*i,"%d", *DragConfig[i].valuei);
                else if(menuitem == i && DelayDiff(now, select_timeout) < 500000)
                  font_printf(-125,18 + font_getHeight()*i,"> %d", *DragConfig[i].valuei);
                else if(DelayDiff(now, select_timeout) > 800000)
                  select_timeout = now;
              }
            }
          }

          font_setFont(&rre_bold_6x8);
          if(StatusSynchronized)
          {
            if(DragStatus == 0)
              font_printf(-125,53,"Ready");
            else if(DragStatus == 1)
            {
              if(DelayDiff(now, select_timeout) < 500000)
                font_printf(-125,53,"SET");
              else if(DelayDiff(now, select_timeout) > 800000)
                select_timeout = now;
            }
            else if(DragStatus == 2)
            {
              if(DelayDiff(now, select_timeout) < 500000)
                font_printf(-125,53,"GO!");
              else if(DelayDiff(now, select_timeout) > 800000)
                select_timeout = now;
            }
            else if(DragStatus == 3)
            {
              if(DelayDiff(now, select_timeout) < 500000)
                font_printf(-125,53,"Done");
              else if(DelayDiff(now, select_timeout) > 800000)
                select_timeout = now;
            }
            else if(DragStatus == 4)
            {
              if(DelayDiff(now, select_timeout) < 500000)
                font_printf(-125,53,"Fail");
              else if(DelayDiff(now, select_timeout) > 800000)
                select_timeout = now;
            }
          }
          else
          {
            if(DelayDiff(now, select_timeout) < 500000)
              font_printf(-125,53,"N/Sync");
            else if(DelayDiff(now, select_timeout) > 800000)
              select_timeout = now;
          }

          if(DelayDiff(now, value_timeout) > 150000)
          {
            value_timeout = now;
            rpm = StatusRPM;
          }

          font_setFont(&rre_arialb_16);
          font_printf(10,34+14,"%5.2f", DragTime);
          font_printf(-80,34+14,"sec");
          if(DragStatus <= 2)
          {
            font_printf(10,34,"%03.0f", rpm);
            font_printf(-90,34,"RPM");
          }
          else if(DragGraphReady)
          {
            int cnt = DragPointsRawCount;
            float x = -1.0f ,y = -1.0f , prevx = -1.0f , prevy = -1.0f ;
            float max = DragRpmTo;
            float min = DragRpmFrom;
            float depmin = 0;
            float depmax = DragTime * 1000000.0f;
            float value, dep;
            for(int i = 0; i < cnt; i++)
            {
              dep = DragPointsRaw[i].Time;
              value = DragPointsRaw[i].RPM;
              x = 126.0f * (dep - depmin) / (depmax-depmin) + 1.0f;
              y = 63 - (value - min) / (max-min) * 47;

              if(prevx == -1.0f && prevy == -1.0f)
                prevx = x, prevy = y;

              lcd_line(prevx, prevy, x, y, 1);

              prevx = x;
              prevy = y;
            }
          }
          else
          {
            font_printf(0,30,"Please, wait...");
          }

          lcd_update();
        }
        if(BUT_ENTER_PRESS)
        {
          if(StatusSynchronized && !BUT_CANCEL_PRESS && BUT_ENTER_TIME > 500)
          {
            select_timeout = now;
            if(DragStatus == 0 || DragStatus >= 3)
            {
              menuselecting = 0;
              DragStatus = 1;
            }
          }
        }
        else if(BUT_ENTER)
        {
          display_timeout = 0;
          if(DragStatus >= 3)
            DragStatus = 0;
          else if(!menuselecting && DragStatus == 0)
          {
              menuselecting = 1;
              menuitem = 0;
              select_timeout = now;

          }
          else
          {
            menuselecting = 0;
          }
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_ENTER = 0;
        }
        if(BUT_CANCEL)
        {
          display_timeout = 0;
          if(menuselecting)
          {
            menuselecting = 0;
          }
          DragStatus = 0;
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_CANCEL = 0;
        }

        if(BUT_LEFT || (BUT_LEFT_PRESS && BUT_LEFT_TIME > 400))
        {
          display_timeout = 0;
          if(BUT_LEFT_TIME > 100) BUT_LEFT_TIME -= 100;
          if(menuselecting && StatusSynchronized)
          {
            if(DragConfig[menuitem].valuef)
            {
              if(*DragConfig[menuitem].valuef - DragConfig[menuitem].step < DragConfig[menuitem].min)
                *DragConfig[menuitem].valuef = DragConfig[menuitem].min;
              else *DragConfig[menuitem].valuef -= DragConfig[menuitem].step;
            }
            else if(DragConfig[menuitem].valuei)
            {
              if(*DragConfig[menuitem].valuei - DragConfig[menuitem].step < DragConfig[menuitem].min)
                *DragConfig[menuitem].valuei = DragConfig[menuitem].min;
              else *DragConfig[menuitem].valuei -= DragConfig[menuitem].step;
            }

            select_timeout = now;
            BUT_LEFT = 0;
          }
        }

        if(BUT_RIGHT || (BUT_RIGHT_PRESS && BUT_RIGHT_TIME > 400))
        {
          display_timeout = 0;
          if(BUT_RIGHT_TIME > 100) BUT_RIGHT_TIME -= 100;
          if(menuselecting && StatusSynchronized)
          {
            if(DragConfig[menuitem].valuef)
            {
              if(*DragConfig[menuitem].valuef + DragConfig[menuitem].step > DragConfig[menuitem].max)
                *DragConfig[menuitem].valuef = DragConfig[menuitem].max;
              else *DragConfig[menuitem].valuef += DragConfig[menuitem].step;
            }
            else if(DragConfig[menuitem].valuei)
            {
              if(*DragConfig[menuitem].valuei + DragConfig[menuitem].step > DragConfig[menuitem].max)
                *DragConfig[menuitem].valuei = DragConfig[menuitem].max;
              else *DragConfig[menuitem].valuei += DragConfig[menuitem].step;
            }

            select_timeout = now;
            BUT_RIGHT = 0;
          }
        }

        if(BUT_DOWN)
        {
          display_timeout = 0;
          if(menuselecting)
          {
            if(++menuitem >= sizeof(DragConfig) / sizeof(sConfigLinking)) menuitem = sizeof(DragConfig) / sizeof(sConfigLinking) - 1;
            select_timeout = now;
          }
          BUT_DOWN = 0;
        }

        if(BUT_UP)
        {
          display_timeout = 0;
          if(menuselecting)
          {
            if(--menuitem < 0) menuitem = 0;
            select_timeout = now;
          }
          BUT_UP = 0;
        }

        if(!BUT_ENTER_PRESS && BUT_CANCEL_PRESS && BUT_CANCEL_TIME > 300)
        {
          display_timeout = 0;
          if(DragStatus > 0)
          {
            DragStatus = 0;
          }
        }

        if(BUT_RIGHT && !menuselecting && (DragStatus == 0 || DragStatus == 3)) eMenuItem++;
        else if(BUT_LEFT && !menuselecting && (DragStatus == 0 || DragStatus == 3)) eMenuItem--;
        break;
      }

      case MenuMainConfig :
      {
        if(DelayDiff(now, display_timeout) > 150000)
        {
          display_timeout = now;
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_8x12);
          font_printf(0,2,"Configuration");
          lcd_rect(0,15,128,1,1);

          font_setFont(&rre_5x8);
          for(int i = 0; i < 6; i++)
          {
            menuselected = menufirst + i;
            if(menuselected < sizeof(CommonConfig) / sizeof(sConfigLinking))
            {
              font_printf(3,18 + font_getHeight()*i,"%s:", CommonConfig[menuselected].name);
              if(CommonConfig[menuselected].valuef)
              {
                if((!menuselecting || menuitem != menuselected))
                  font_printf(-125,18 + font_getHeight()*i,"%.0f", *CommonConfig[menuselected].valuef + CommonConfig[menuselected].guicorrective);
                else if(menuitem == menuselected && DelayDiff(now, select_timeout) < 500000)
                  font_printf(-125,18 + font_getHeight()*i,"> %.0f", *CommonConfig[menuselected].valuef + CommonConfig[menuselected].guicorrective);
                else if(DelayDiff(now, select_timeout) > 800000)
                  select_timeout = now;
              }
              else if(CommonConfig[menuselected].valuei)
              {
                if(CommonConfig[menuselected].min == 0 && CommonConfig[menuselected].max == 1 && CommonConfig[menuselected].step == 1 && CommonConfig[menuselected].guicorrective == 0 && CommonConfig[menuselected].guimultiplier == 1.0f)
                {
                  if((!menuselecting || menuitem != menuselected))
                    font_printf(-125,18 + font_getHeight()*i,"%s", *CommonConfig[menuselected].valuei ? "Y" : "N");
                  else if(menuitem == menuselected && DelayDiff(now, select_timeout) < 500000)
                    font_printf(-125,18 + font_getHeight()*i,"> %s", *CommonConfig[menuselected].valuei ? "Y" : "N");
                  else if(DelayDiff(now, select_timeout) > 800000)
                    select_timeout = now;
                }
                else
                {
                  if((!menuselecting || menuitem != menuselected))
                    font_printf(-125,18 + font_getHeight()*i,"%d", *CommonConfig[menuselected].valuei + CommonConfig[menuselected].guicorrective);
                  else if(menuitem == menuselected && DelayDiff(now, select_timeout) < 500000)
                    font_printf(-125,18 + font_getHeight()*i,"> %d", *CommonConfig[menuselected].valuei + CommonConfig[menuselected].guicorrective);
                  else if(DelayDiff(now, select_timeout) > 800000)
                    select_timeout = now;
                }
              }
            }
          }

          lcd_update();
        }
        if(BUT_ENTER)
        {
          display_timeout = 0;
          if(!menuselecting)
          {
            menuselecting = 1;
            select_timeout = now;

          }
          else
          {
            menuselecting = 0;
          }
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_ENTER = 0;
        }
        if(BUT_CANCEL)
        {
          display_timeout = 0;
          if(menuselecting)
          {
            menuselecting = 0;
          }
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_CANCEL = 0;
        }

        if(BUT_LEFT || (BUT_LEFT_PRESS && BUT_LEFT_TIME > 400))
        {
          display_timeout = 0;
          if(BUT_LEFT_TIME > 100) BUT_LEFT_TIME -= 100;
          if(menuselecting)
          {
            if(StatusSynchronized)
            {
              if(CommonConfig[menuitem].valuef)
              {
                if(*CommonConfig[menuitem].valuef - CommonConfig[menuitem].step < CommonConfig[menuitem].min)
                  *CommonConfig[menuitem].valuef = CommonConfig[menuitem].min;
                else *CommonConfig[menuitem].valuef -= CommonConfig[menuitem].step;
                acis_apply_parameter(CommonConfig[menuitem].valuef, sizeof(float));
              }
              else if(CommonConfig[menuitem].valuei)
              {
                if(*CommonConfig[menuitem].valuei - CommonConfig[menuitem].step < CommonConfig[menuitem].min)
                  *CommonConfig[menuitem].valuei = CommonConfig[menuitem].min;
                else *CommonConfig[menuitem].valuei -= CommonConfig[menuitem].step;
                acis_apply_parameter(CommonConfig[menuitem].valuei, sizeof(int32_t));
              }
            }
            select_timeout = now;
            BUT_LEFT = 0;
          }
        }

        if(BUT_RIGHT || (BUT_RIGHT_PRESS && BUT_RIGHT_TIME > 400))
        {
          display_timeout = 0;
          if(BUT_RIGHT_TIME > 100) BUT_RIGHT_TIME -= 100;
          if(menuselecting)
          {
            if(StatusSynchronized)
            {
              if(CommonConfig[menuitem].valuef)
              {
                if(*CommonConfig[menuitem].valuef + CommonConfig[menuitem].step > CommonConfig[menuitem].max)
                  *CommonConfig[menuitem].valuef = CommonConfig[menuitem].max;
                else *CommonConfig[menuitem].valuef += CommonConfig[menuitem].step;
                acis_apply_parameter(CommonConfig[menuitem].valuef, sizeof(float));
              }
              else if(CommonConfig[menuitem].valuei)
              {
                if(*CommonConfig[menuitem].valuei + CommonConfig[menuitem].step > CommonConfig[menuitem].max)
                  *CommonConfig[menuitem].valuei = CommonConfig[menuitem].max;
                else *CommonConfig[menuitem].valuei += CommonConfig[menuitem].step;
                acis_apply_parameter(CommonConfig[menuitem].valuei, sizeof(int32_t));
              }
            }
            select_timeout = now;
            BUT_RIGHT = 0;
          }
        }

        if(BUT_DOWN)
        {
          display_timeout = 0;
          if(menuselecting)
          {
            if(++menuitem >= sizeof(CommonConfig) / sizeof(sConfigLinking)) menuitem = sizeof(CommonConfig) / sizeof(sConfigLinking) - 1;
            if(menufirst + 4 < menuitem) menufirst = menuitem - 4;
            select_timeout = now;
          }
          BUT_DOWN = 0;
        }

        if(BUT_UP)
        {
          display_timeout = 0;
          if(menuselecting)
          {
            if(--menuitem < 0) menuitem = 0;
            if(menuitem < menufirst) menufirst = menuitem;
            select_timeout = now;
          }
          BUT_UP = 0;
        }

        if(BUT_RIGHT && !menuselecting) BUT_RIGHT = 0, eMenuItem++;
        else if(BUT_LEFT && !menuselecting) BUT_LEFT = 0, eMenuItem--;
        break;
      }

      case MenuTableSelect :
      {
        if(DelayDiff(now, display_timeout) > 150000)
        {
          display_timeout = now;
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_8x12);
          font_printf(0,2,"Table Select");
          lcd_rect(0,15,128,1,1);

          font_setFont(&rre_5x8);
          for(int i = 0; i < 6; i++)
          {
            menuselected = menufirst + i;
            if(menuselected < sizeof(TableInitial) / sizeof(sConfigLinking))
            {
              if(TableInitial[menuselected].valuef)
              {
                font_printf(3,18 + font_getHeight()*i,"%s:", TableInitial[menuselected].name);
                if((!menuselecting || menuitem != menuselected))
                  font_printf(-125,18 + font_getHeight()*i,"%.0f", *TableInitial[menuselected].valuef + TableInitial[menuselected].guicorrective);
                else if(menuitem == menuselected && DelayDiff(now, select_timeout) < 500000)
                  font_printf(-125,18 + font_getHeight()*i,"> %.0f", *TableInitial[menuselected].valuef + TableInitial[menuselected].guicorrective);
                else if(DelayDiff(now, select_timeout) > 800000)
                  select_timeout = now;
              }
              else if(TableInitial[menuselected].valuei)
              {
                font_printf(3,18 + font_getHeight()*i,"%s:", TableInitial[menuselected].name);
                if((!menuselecting || menuitem != menuselected))
                  font_printf(-125,18 + font_getHeight()*i,"%d", *TableInitial[menuselected].valuei + TableInitial[menuselected].guicorrective);
                else if(menuitem == menuselected && DelayDiff(now, select_timeout) < 500000)
                  font_printf(-125,18 + font_getHeight()*i,"> %d", *TableInitial[menuselected].valuei + TableInitial[menuselected].guicorrective);
                else if(DelayDiff(now, select_timeout) > 800000)
                  select_timeout = now;
              }
              else
              {
                if((!menuselecting || menuitem != menuselected))
                  font_printf(3,18 + font_getHeight()*i,"%s", TableInitial[menuselected].name);
                else if(menuitem == menuselected && DelayDiff(now, select_timeout) < 500000)
                {
                  font_printf(3,18 + font_getHeight()*i,"%s", TableInitial[menuselected].name);
                  font_printf(-125,18 + font_getHeight()*i,"<");
                }
                else if(DelayDiff(now, select_timeout) > 800000)
                  select_timeout = now;
              }
            }
          }

          lcd_update();
        }
        if(BUT_ENTER)
        {
          display_timeout = 0;
          if(!menuselecting)
          {
            menuselecting = 1;
            select_timeout = now;

          }
          else
          {
            if(TableInitial[menuitem].valuef == NULL && TableInitial[menuitem].valuei == NULL &&  TableInitial[menuitem].menuitem > MenuUndefined)
            {
              eMenuItem = TableInitial[menuitem].menuitem;
            }
            menuselecting = 0;
          }
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_ENTER = 0;
          BUT_CANCEL = 0;
        }
        if(BUT_CANCEL)
        {
          display_timeout = 0;
          if(menuselecting)
          {
            menuselecting = 0;
          }
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_CANCEL = 0;
        }

        if(BUT_LEFT || (BUT_LEFT_PRESS && BUT_LEFT_TIME > 400))
        {
          display_timeout = 0;
          if(BUT_LEFT_TIME > 100) BUT_LEFT_TIME -= 100;
          if(menuselecting)
          {
            if(StatusSynchronized && TableInitial[menuitem].step > 0)
            {
              if(TableInitial[menuitem].valuef)
              {
                if(*TableInitial[menuitem].valuef - TableInitial[menuitem].step < TableInitial[menuitem].min)
                  *TableInitial[menuitem].valuef = TableInitial[menuitem].min;
                else *TableInitial[menuitem].valuef -= TableInitial[menuitem].step;
                acis_apply_parameter(TableInitial[menuitem].valuef, sizeof(float));
              }
              else if(TableInitial[menuitem].valuei)
              {
                if(*TableInitial[menuitem].valuei - TableInitial[menuitem].step < TableInitial[menuitem].min)
                  *TableInitial[menuitem].valuei = TableInitial[menuitem].min;
                else *TableInitial[menuitem].valuei -= TableInitial[menuitem].step;
                acis_apply_parameter(TableInitial[menuitem].valuei, sizeof(int32_t));
              }
            }
            select_timeout = now;
            BUT_LEFT = 0;
          }
        }

        if(BUT_RIGHT || (BUT_RIGHT_PRESS && BUT_RIGHT_TIME > 400))
        {
          display_timeout = 0;
          if(BUT_RIGHT_TIME > 100) BUT_RIGHT_TIME -= 100;
          if(menuselecting)
          {
            if(StatusSynchronized && TableInitial[menuitem].step > 0)
            {
              if(TableInitial[menuitem].valuef)
              {
                if(*TableInitial[menuitem].valuef + TableInitial[menuitem].step > TableInitial[menuitem].max)
                  *TableInitial[menuitem].valuef = TableInitial[menuitem].max;
                else *TableInitial[menuitem].valuef += TableInitial[menuitem].step;
                acis_apply_parameter(TableInitial[menuitem].valuef, sizeof(float));
              }
              else if(TableInitial[menuitem].valuei)
              {
                if(*TableInitial[menuitem].valuei + TableInitial[menuitem].step > TableInitial[menuitem].max)
                  *TableInitial[menuitem].valuei = TableInitial[menuitem].max;
                else *TableInitial[menuitem].valuei += TableInitial[menuitem].step;
                acis_apply_parameter(TableInitial[menuitem].valuei, sizeof(int32_t));
              }
            }
            select_timeout = now;
            BUT_RIGHT = 0;
          }
        }

        if(BUT_DOWN)
        {
          display_timeout = 0;
          if(menuselecting)
          {
            if(++menuitem >= sizeof(TableInitial) / sizeof(sConfigLinking)) menuitem = sizeof(TableInitial) / sizeof(sConfigLinking) - 1;
            if(menufirst + 4 < menuitem) menufirst = menuitem - 4;
            select_timeout = now;
          }
          BUT_DOWN = 0;
        }

        if(BUT_UP)
        {
          display_timeout = 0;
          if(menuselecting)
          {
            if(--menuitem < 0) menuitem = 0;
            if(menuitem < menufirst) menufirst = menuitem;
            select_timeout = now;
          }
          BUT_UP = 0;
        }

        if(BUT_RIGHT && !menuselecting) BUT_RIGHT = 0, eMenuItem++;
        else if(BUT_LEFT && !menuselecting) BUT_LEFT = 0, eMenuItem--;

        break;
      }

      case MenuTableConfig :
      {
        if(DelayDiff(now, display_timeout) > 150000)
        {
          display_timeout = now;
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_8x12);
          font_printf(0,2,"Table Config");
          lcd_rect(0,15,128,1,1);

          font_setFont(&rre_5x8);
          for(int i = 0; i < 6; i++)
          {
            menuselected = menufirst + i;
            if(menuselected < sizeof(TableConfig) / sizeof(sConfigLinking))
            {
              font_printf(3,18 + font_getHeight()*i,"%s:", TableConfig[menuselected].name);
              if(TableConfig[menuselected].valuef)
              {
                if((!menuselecting || menuitem != menuselected))
                  font_printf(-125,18 + font_getHeight()*i,"%.1f", (TableOffset(TableConfig[menuselected].valuef, float) + TableConfig[menuselected].guicorrective) * TableConfig[menuselected].guimultiplier);
                else if(menuitem == menuselected && DelayDiff(now, select_timeout) < 500000)
                  font_printf(-125,18 + font_getHeight()*i,"> %.1f", (TableOffset(TableConfig[menuselected].valuef, float) + TableConfig[menuselected].guicorrective) * TableConfig[menuselected].guimultiplier);
                else if(DelayDiff(now, select_timeout) > 800000)
                  select_timeout = now;
              }
              else if(TableConfig[menuselected].valuei)
              {
                if((!menuselecting || menuitem != menuselected))
                  font_printf(-125,18 + font_getHeight()*i,"%d", (int32_t)roundf((TableOffset(TableConfig[menuselected].valuei, int32_t) + TableConfig[menuselected].guicorrective) * TableConfig[menuselected].guimultiplier));
                else if(menuitem == menuselected && DelayDiff(now, select_timeout) < 500000)
                  font_printf(-125,18 + font_getHeight()*i,"> %d", (int32_t)roundf((TableOffset(TableConfig[menuselected].valuei, int32_t) + TableConfig[menuselected].guicorrective) * TableConfig[menuselected].guimultiplier));
                else if(DelayDiff(now, select_timeout) > 800000)
                  select_timeout = now;
              }
              else if(&TableConfig[menuselected].values)
              {
                if((menuselecting == 0 || menuitem != menuselected))
                  font_printf(-125,18 + font_getHeight()*i,"%s", &TableOffset(TableConfig[menuselected].values, char));
                else if(menuselecting == 1 && menuitem == menuselected && DelayDiff(now, select_timeout) < 500000)
                  font_printf(-125,18 + font_getHeight()*i,"> %s", &TableOffset(TableConfig[menuselected].values, char));
                else if(menuitem == menuselected && DelayDiff(now, select_timeout) < 500000)
                {
                  int width = 0;
                  int widthbf = 0;
                  for(int j = 0; j < TABLE_STRING_MAX; j++)
                  {
                    char chr = tablestring[j];
                    width += font_printf(-125 + width,18 + font_getHeight()*i,"%c", chr);
                    if(j == stringchar)
                    {
                      lcd_rect_solid(125-width, 18 + font_getHeight() - 1, width - widthbf - 1, 1, 1);
                    }
                    widthbf = width;
                  }
                }
                else
                {
                  if(menuselecting == 2)
                  {
                    int width = 0;
                    int widthbf = 0;
                    for(int j = 0; j < TABLE_STRING_MAX; j++)
                    {
                      char chr = tablestring[j];
                      width += font_printf(-125 + widthbf,18 + font_getHeight()*i,"%c", chr);
                      if(j == stringchar)
                      {
                        lcd_rect_solid(125-width, 18, width - widthbf, font_getHeight(), 0);
                        lcd_rect_solid(125-width, 18 + font_getHeight() - 1, width - widthbf - 1, 1, 1);
                      }
                      widthbf = width;
                    }
                  }
                  if(DelayDiff(now, select_timeout) > 800000)
                  {
                    select_timeout = now;
                  }
                }
              }
            }
          }

          lcd_update();
        }
        if(BUT_ENTER)
        {
          display_timeout = 0;
          if(menuselecting == 0)
          {
            menuselecting = 1;
            select_timeout = now;

          }
          else if(menuselecting == 1)
          {
            select_timeout = now;
            if(&TableConfig[menuitem].values)
            {
              menuselecting = 2;
              memset(tablestring, ' ', sizeof(tablestring));
              memset(tablechars, 0, sizeof(tablechars));
              for(int i = 0; i < TABLE_STRING_MAX; i++)
              {
                if((&TableOffset(TableConfig[menuitem].values, char))[TABLE_STRING_MAX - i - 1] != 0)
                {
                  for(int j = i; j < TABLE_STRING_MAX; j++)
                  {
                    char chr = (&TableOffset(TableConfig[menuitem].values, char))[TABLE_STRING_MAX - j - 1];
                    uint8_t num = 0;
                    for(int k = 0; k < lcd_chars_len; k++)
                    {
                      if(lcd_chars[k] == chr)
                      {
                        num = k;
                        break;
                      }
                    }
                    tablestring[j - i] = chr;
                    tablechars[j - i] = num;
                  }
                  break;
                }
              }
              stringchar = 0;
            }
            else
            {
              menuselecting = 0;
            }
          }
          else if(menuselecting == 2)
          {
            menuselecting = 1;
          }
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_ENTER = 0;
        }
        if(BUT_CANCEL)
        {
          display_timeout = 0;
          if(menuselecting > 0)
          {
            if(menuselecting == 2)
            {
              memset(&TableOffset(TableConfig[menuitem].values, char), '\0', TABLE_STRING_MAX);
              for(int i = 0; i < TABLE_STRING_MAX; i++)
              {
                if(tablestring[TABLE_STRING_MAX - i - 1] != ' ')
                {
                  for(int j = i; j < TABLE_STRING_MAX; j++)
                    (&TableOffset(TableConfig[menuitem].values, char))[j - i] = tablestring[TABLE_STRING_MAX - j - 1];
                  break;
                }
              }
              acis_apply_parameter(&TableOffset(TableConfig[menuitem].values, char), TABLE_STRING_MAX);
            }
            select_timeout = now;
            menuselecting--;
          }
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_CANCEL = 0;
        }

        if(BUT_LEFT || (BUT_LEFT_PRESS && BUT_LEFT_TIME > 400))
        {
          display_timeout = 0;
          if(BUT_LEFT_TIME > 100) BUT_LEFT_TIME -= 100;
          if(menuselecting)
          {
            if(menuselecting == 1)
            {
              if(StatusSynchronized && TableConfig[menuitem].step)
              {
                if(TableConfig[menuitem].valuef)
                {
                  if(TableOffset(TableConfig[menuitem].valuef, float) - TableConfig[menuitem].step < TableConfig[menuitem].min)
                    TableOffset(TableConfig[menuitem].valuef, float) = TableConfig[menuitem].min;
                  else TableOffset(TableConfig[menuitem].valuef, float) -= TableConfig[menuitem].step;
                  acis_apply_parameter(&TableOffset(TableConfig[menuitem].valuef, float), sizeof(float));
                }
                else if(TableConfig[menuitem].valuei)
                {
                  if(TableOffset(TableConfig[menuitem].valuei, int32_t) - TableConfig[menuitem].step < TableConfig[menuitem].min)
                    TableOffset(TableConfig[menuitem].valuei, int32_t) = TableConfig[menuitem].min;
                  else TableOffset(TableConfig[menuitem].valuei, int32_t) -= TableConfig[menuitem].step;
                  acis_apply_parameter(&TableOffset(TableConfig[menuitem].valuei, int32_t), sizeof(int32_t));
                }
              }
            }
            else if(menuselecting == 2)
            {
              if(++stringchar >= TABLE_STRING_MAX - 1) stringchar = TABLE_STRING_MAX - 2;
            }
            select_timeout = now;
            BUT_LEFT = 0;
          }
        }

        if(BUT_RIGHT || (BUT_RIGHT_PRESS && BUT_RIGHT_TIME > 400))
        {
          display_timeout = 0;
          if(BUT_RIGHT_TIME > 100) BUT_RIGHT_TIME -= 100;
          if(menuselecting)
          {
            if(menuselecting == 1)
            {
              if(StatusSynchronized && TableConfig[menuitem].step)
              {
                if(TableConfig[menuitem].valuef)
                {
                  if(TableOffset(TableConfig[menuitem].valuef, float) + TableConfig[menuitem].step > TableConfig[menuitem].max)
                    TableOffset(TableConfig[menuitem].valuef, float) = TableConfig[menuitem].max;
                  else TableOffset(TableConfig[menuitem].valuef, float) += TableConfig[menuitem].step;
                  acis_apply_parameter(&TableOffset(TableConfig[menuitem].valuef, float), sizeof(float));
                }
                else if(TableConfig[menuitem].valuei)
                {
                  if(TableOffset(TableConfig[menuitem].valuei, int32_t) + TableConfig[menuitem].step > TableConfig[menuitem].max)
                    TableOffset(TableConfig[menuitem].valuei, int32_t) = TableConfig[menuitem].max;
                  else TableOffset(TableConfig[menuitem].valuei, int32_t) += TableConfig[menuitem].step;
                  acis_apply_parameter(&TableOffset(TableConfig[menuitem].valuei, int32_t), sizeof(int32_t));
                }
              }
            }
            else if(menuselecting == 2)
            {
              if(--stringchar < 0) stringchar = 0;
            }
            select_timeout = now;
            BUT_RIGHT = 0;
          }
        }

        if(BUT_DOWN)
        {
          display_timeout = 0;
          if(menuselecting == 1)
          {
            if(++menuitem >= sizeof(TableConfig) / sizeof(sConfigLinking)) menuitem = sizeof(TableConfig) / sizeof(sConfigLinking) - 1;
            if(menufirst + 4 < menuitem) menufirst = menuitem - 4;
            select_timeout = now;
          }
          else if(menuselecting == 2)
          {
            if(StatusSynchronized)
            {
              if(++tablechars[stringchar] >= lcd_chars_len) tablechars[stringchar] = 0;
              tablestring[stringchar] = lcd_chars[tablechars[stringchar]];
            }
            select_timeout = now;
          }
          BUT_DOWN = 0;
        }

        if(BUT_UP)
        {
          display_timeout = 0;
          if(menuselecting == 1)
          {
            if(--menuitem < 0) menuitem = 0;
            if(menuitem < menufirst) menufirst = menuitem;
            select_timeout = now;
          }
          else if(menuselecting == 2)
          {
            if(StatusSynchronized)
            {
              if(--tablechars[stringchar] < 0) tablechars[stringchar] = lcd_chars_len - 1;
              tablestring[stringchar] = lcd_chars[tablechars[stringchar]];
            }
            select_timeout = now;
          }
          BUT_UP = 0;
        }

        if(BUT_RIGHT && !menuselecting) BUT_RIGHT = 0, eMenuItem++, menuitem = 0;
        else if(BUT_LEFT && !menuselecting) BUT_LEFT = 0, eMenuItem--, menuitem = 0;

        break;
      }
      case MenuTableSetup :
      {
        if(DelayDiff(now, display_timeout) > 150000)
        {
          display_timeout = now;
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_8x12);
          font_printf(0,2,"Table Setups");
          lcd_rect(0,15,128,1,1);

          font_setFont(&rre_5x8);
          for(int i = 0; i < 6; i++)
          {
            menuselected = menufirst + i;
            if(menuselected < sizeof(TableSetup) / sizeof(sConfigLinking))
            {
              if((!menuselecting || menuitem != menuselected))
              {
                if(TableSetup[menuselected].valuei)
                  font_printf(3,18 + font_getHeight()*i,TableSetup[menuselected].name, TableOffset(TableSetup[menuselected].valuei, int32_t));
                else font_printf(3,18 + font_getHeight()*i,TableSetup[menuselected].name);
              }
              else if(menuitem == menuselected && DelayDiff(now, select_timeout) < 500000)
              {
                font_printf(-125,18 + font_getHeight()*i,"<");
                if(TableSetup[menuselected].valuei)
                  font_printf(3,18 + font_getHeight()*i,TableSetup[menuselected].name, TableOffset(TableSetup[menuselected].valuei, int32_t));
                else font_printf(3,18 + font_getHeight()*i,TableSetup[menuselected].name);
              }
              else if(DelayDiff(now, select_timeout) > 800000)
                select_timeout = now;
            }
          }

          lcd_update();
        }
        if(BUT_ENTER)
        {
          display_timeout = 0;
          if(!menuselecting)
          {
            menuselecting = 1;
            select_timeout = now;
          }
          else
          {
            select_timeout = now;
            tablesetupitem = &TableSetup[menuitem];
            eMenuItem = tablesetupitem->menuitem;
            tablesetuptitle = tablesetupitem->name;
            menufirst2 = 0;
            menuitem2 = 0;
          }
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_ENTER = 0;
        }
        if(BUT_CANCEL)
        {
          display_timeout = 0;
          if(menuselecting)
          {
            menuselecting = 0;
          }
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_CANCEL = 0;
        }

        if(BUT_DOWN)
        {
          display_timeout = 0;
          if(menuselecting)
          {
            if(++menuitem >= sizeof(TableSetup) / sizeof(sConfigLinking)) menuitem = sizeof(TableSetup) / sizeof(sConfigLinking) - 1;
            if(menufirst + 4 < menuitem) menufirst = menuitem - 4;
            select_timeout = now;
          }
          BUT_DOWN = 0;
        }

        if(BUT_UP)
        {
          display_timeout = 0;
          if(menuselecting)
          {
            if(--menuitem < 0) menuitem = 0;
            if(menuitem < menufirst) menufirst = menuitem;
            select_timeout = now;
          }
          BUT_UP = 0;
        }

        if(BUT_RIGHT && !menuselecting) BUT_RIGHT = 0, eMenuItem++;
        else if(BUT_LEFT && !menuselecting) BUT_LEFT = 0, eMenuItem--;

        break;
      }
      case MenuTableSetupPressures :
      case MenuTableSetupRotates :
      case MenuTableSetupIdleRotates :
      case MenuTableSetupIdleIgnitions :
      case MenuTableSetupTemperatures :
      case MenuTableSetupServoAccel :
      case MenuTableSetupServoChoke:
      {

        if(DelayDiff(now, display_timeout) > 150000)
        {
          display_timeout = now;
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_8x12);
          font_printf(0,2,tablesetuptitle, TableOffset(tablesetupitem->valuei, int32_t));
          lcd_rect(0,15,128,1,1);

          font_setFont(&rre_5x8);
          for(int i = 0; i < 2; i++)
          {
            menuselected = menufirst2 + i;
            if(menuselected < TableOffset(tablesetupitem->valuei, int32_t))
            {
              if(tablesetupitem->valuedep)
                font_printf(3,18 + font_getHeight()*i,tablesetupitem->title, menuselected + 1, ((&TableOffset(tablesetupitem->valuedep, float))[menuselected]));
              else font_printf(3,18 + font_getHeight()*i,tablesetupitem->title, menuselected + 1);

              if(menuitem2 != menuselected)
                font_printf(-125,18 + font_getHeight()*i,tablesetupitem->step >= 1.0f ? "%.0f" : "%.1f", ((&TableOffset(tablesetupitem->valuef, float))[menuselected]));
              else if(menuitem2 == menuselected && DelayDiff(now, select_timeout) < 500000)
                font_printf(-125,18 + font_getHeight()*i,tablesetupitem->step >= 1.0f ? "> %.0f" : "> %.1f", ((&TableOffset(tablesetupitem->valuef, float))[menuselected]));
              else if(DelayDiff(now, select_timeout) > 800000)
                select_timeout = now;

            }
          }

          lcd_rect(1,30,126,1,1);
          lcd_rect_solid(1,31,126,font_getHeight(),0);

          int cnt = TableOffset(tablesetupitem->valuei, int32_t);
          float x = -1.0f ,y = -1.0f , prevx = -1.0f , prevy = -1.0f ;
          float cx,cy;
          float max = -INFINITY;
          float min = 0;
          float depmin = INFINITY;
          float depmax = -INFINITY;
          float value, dep;
          for(int i = 0; i < cnt; i++)
          {
            value = ((&TableOffset(tablesetupitem->valuef, float))[i]);
            if(value > max)
              max = value;
            if(value < min)
              min = value;
          }
          if(tablesetupitem->valuedep)
          {
            for(int i = 0; i < cnt; i++)
            {
              dep = ((&TableOffset(tablesetupitem->valuedep, float))[i]);
              if(dep > depmax)
                depmax = dep;
              if(dep < depmin)
                depmin = dep;
            }
            for(int i = 0; i < cnt; i++)
            {
              dep = ((&TableOffset(tablesetupitem->valuedep, float))[i]);
              value = ((&TableOffset(tablesetupitem->valuef, float))[i]);
              if(eMenuItem == MenuTableSetupIdleIgnitions)
                x = 126.0f * log10f(1.0f + ((dep - depmin) / (depmax-depmin) * 9.0f)) + 1.0f;
              else
                x = 126.0f * (dep - depmin) / (depmax-depmin) + 1.0f;
              y = 63 - (value - min) / (max-min) * 32;

              if(prevx == -1.0f && prevy == -1.0f)
                prevx = x, prevy = y;

              if(i == menuitem2)
                cx = x, cy = y;

              lcd_line(prevx, prevy, x, y, 1);

              prevx = x;
              prevy = y;
            }
          }
          else
          {
            for(int i = 0; i < cnt; i++)
            {
              value = ((&TableOffset(tablesetupitem->valuef, float))[i]);
              x = 126.0f * ((float)i / (float)(cnt-1)) + 1.0f;
              y = 63 - (value - min) / (max-min) * 32;

              if(prevx == -1.0f && prevy == -1.0f)
                prevx = x, prevy = y;

              lcd_line(prevx, prevy, x, y, 1);

              if(i == menuitem2)
                cx = x, cy = y;

              prevx = x;
              prevy = y;
            }
          }
          if(cy > 28)
            lcd_circle5x5(cx,cy);

          lcd_update();
        }
        if(BUT_CANCEL)
        {
          display_timeout = 0;
          eMenuItem = MenuTableSetup;
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_CANCEL = 0;
          BUT_ENTER = 0;
        }

        if(BUT_LEFT || (BUT_LEFT_PRESS && BUT_LEFT_TIME > 400))
        {
          display_timeout = 0;
          if(BUT_LEFT_TIME > 100) BUT_LEFT_TIME -= 40;
          if(StatusSynchronized && tablesetupitem->step)
          {
            if(tablesetupitem->valuedep)
            {
              if(tablesetupitem->valuef)
              {
                if(((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) - tablesetupitem->step < tablesetupitem->min)
                  ((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) = tablesetupitem->min;
                else ((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) -= tablesetupitem->step;
                acis_apply_parameter(&((&TableOffset(tablesetupitem->valuef, float))[menuitem2]), sizeof(float));
              }
            }
            else
            {
              if(((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) - tablesetupitem->step < tablesetupitem->min)
                ((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) = tablesetupitem->min;
              else ((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) -= tablesetupitem->step;

              if(TableOffset(tablesetupitem->valuei, int32_t) > 1 && menuitem2 > 0 &&
                  ((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) - tablesetupitem->step <= ((&TableOffset(tablesetupitem->valuef, float))[menuitem2-1]))
                ((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) = ((&TableOffset(tablesetupitem->valuef, float))[menuitem2-1]) + tablesetupitem->step;


              acis_apply_parameter(&((&TableOffset(tablesetupitem->valuef, float))[menuitem2]), sizeof(float));
            }
          }
          select_timeout = now;
          BUT_LEFT = 0;

        }

        if(BUT_RIGHT || (BUT_RIGHT_PRESS && BUT_RIGHT_TIME > 400))
        {
          display_timeout = 0;
          if(BUT_RIGHT_TIME > 100) BUT_RIGHT_TIME -= 40;
          if(StatusSynchronized && tablesetupitem->step)
          {
            if(tablesetupitem->valuedep)
            {
              if(tablesetupitem->valuef)
              {
                if(((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) + tablesetupitem->step > tablesetupitem->max)
                  ((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) = tablesetupitem->max;
                else ((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) += tablesetupitem->step;
                acis_apply_parameter(&((&TableOffset(tablesetupitem->valuef, float))[menuitem2]), sizeof(float));
              }
            }
            else
            {
              if(tablesetupitem->valuef)
              {
                if(((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) + tablesetupitem->step > tablesetupitem->max)
                  ((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) = tablesetupitem->max;
                else ((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) += tablesetupitem->step;

                if(TableOffset(tablesetupitem->valuei, int32_t) > 1 && menuitem2 < TableOffset(tablesetupitem->valuei, int32_t)-1 &&
                    ((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) + tablesetupitem->step >= ((&TableOffset(tablesetupitem->valuef, float))[menuitem2+1]))
                  ((&TableOffset(tablesetupitem->valuef, float))[menuitem2]) = ((&TableOffset(tablesetupitem->valuef, float))[menuitem2+1]) - tablesetupitem->step;

                acis_apply_parameter(&((&TableOffset(tablesetupitem->valuef, float))[menuitem2]), sizeof(float));
              }
            }
          }
          select_timeout = now;
          BUT_RIGHT = 0;

        }

        if(BUT_DOWN)
        {
          display_timeout = 0;
          menuitem2++;
          select_timeout = now;
          BUT_DOWN = 0;
        }

        if(BUT_UP)
        {
          display_timeout = 0;
          menuitem2--;
          select_timeout = now;
          BUT_UP = 0;
        }

        if(menuitem2 < 0) menuitem2 = 0, display_timeout = 0;
        if(menuitem2 >= *tablesetupitem->valuei) menuitem2 = *tablesetupitem->valuei - 1, display_timeout = 0;;
        menufirst2 = menuitem2;

        break;
      }
      case MenuTableSetupIgnitionsSelect :
      {
        table = &TableOffset(tablesetupitem->valuei, sAcisIgnTable);
        if(DelayDiff(now, display_timeout) > 30000)
        {
          if(DelayDiff(now, value_timeout) > 150000)
          {
            value_timeout = now;
            rpm = StatusRPM;
            pres = StatusPressure;
            ign = StatusIgnition-table->octane_corrector;
          }
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_5x8);
          font_printf(2,2,"Igns.%dx%d", table->pressures_count, table->rotates_count);
          lcd_rect(0,11,128,1,1);

          for(int i = 0; i < 2; i++)
          {
            menuselected = menufirst3 + i;
            if(menuselected < table->pressures_count)
            {
              if(menuitem3 != menuselected)
              {
                font_printf(3,14 + font_getHeight()*i,"Pres.%04.0f", table->pressures[menuselected]);
              }
              else if(menuitem3 == menuselected && DelayDiff(now, select_timeout) < 500000)
              {
                font_printf(-125,14 + font_getHeight()*i,"<");
                font_printf(3,14 + font_getHeight()*i,"Pres.%04.0f", table->pressures[menuselected]);
              }
              else if(DelayDiff(now, select_timeout) > 800000)
                select_timeout = now;

            }
          }

          lcd_rect(1,24,126,1,1);
          lcd_rect_solid(1,25,126,font_getHeight(),0);

          int cnt = table->rotates_count;
          float x = -1.0f ,y = -1.0f , prevx = -1.0f , prevy = -1.0f ;
          float max = -INFINITY;
          float min = 0;
          float depmin = INFINITY;
          float depmax = -INFINITY;
          float value, dep;
          float cx,cy;
          for(int j = 0; j < table->pressures_count; j++)
          {
            for(int i = 0; i < cnt; i++)
            {
              value = table->ignitions[j][i];
              if(value > max)
                max = value;
              if(value < min)
                min = value;
            }
          }

          for(int i = 0; i < cnt; i++)
          {
            dep = table->rotates[i];
            if(dep > depmax)
              depmax = dep;
            if(dep < depmin)
              depmin = dep;
          }
          for(int i = 0; i < cnt; i++)
          {
            dep = table->rotates[i];
            value = table->ignitions[menuitem3][i];
            //x = 126.0f * (dep - depmin) / (depmax-depmin) + 1.0f;
            x = (126.0f * log10f(1.0f + (dep - depmin) / (depmax-depmin) * 9.0f)) + 1.0f;
            y = 63 - (value - min) / (max-min) * 37;

            if(prevx == -1.0f && prevy == -1.0f)
              prevx = x, prevy = y;

            lcd_line(prevx, prevy, x, y, 1);

            prevx = x;
            prevy = y;
          }

          cx = (126.0f * log10f(1.0f + (StatusRPM - depmin) / (depmax-depmin) * 9.0f)) + 1.0f;
          cy = 63 - (StatusIgnition - table->octane_corrector - min) / (max-min) * 37;

          if(cy > 22 && cx > 0 && cx < 128)
            lcd_circle5x5fill(cx,cy);

          font_printf(65,2,"%03.0f", rpm);
          font_printf(93,2,"%04.0f", pres);
          font_printf(94,63 - font_getHeight(),"%5.1fd", ign);


          lcd_update();
        }

        if(BUT_CANCEL)
        {
          display_timeout = 0;
          eMenuItem = MenuTableSetup;
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_CANCEL = 0;
          BUT_ENTER = 0;
        }

        if(BUT_ENTER)
        {
          display_timeout = 0;
          eMenuItem = MenuTableSetupIgnitions;
          select_timeout = now;
          value_timeout = 0;
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_CANCEL = 0;
          BUT_ENTER = 0;
        }

        if(BUT_DOWN)
        {
          display_timeout = 0;
          menuitem3++;
          select_timeout = now;
          BUT_DOWN = 0;
        }

        if(BUT_UP)
        {
          display_timeout = 0;
          menuitem3--;
          select_timeout = now;
          BUT_UP = 0;
        }

        if(menuitem3 >= table->pressures_count) menuitem3 = table->pressures_count - 1, display_timeout = 0;
        if(menuitem3 < 0) menuitem3 = 0, display_timeout = 0;
        menufirst3 = menuitem3;

        break;
      }
      case MenuTableSetupIgnitions :
      {
        if(DelayDiff(now, display_timeout) > 30000)
        {
          if(DelayDiff(now, value_timeout) > 150000)
          {
            value_timeout = now;
            rpm = StatusRPM;
            pres = StatusPressure;
            ign = StatusIgnition - table->octane_corrector;
          }
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_5x8);
          font_printf(2,2,"Igns.%dx%d", menuitem3, table->rotates_count);
          lcd_rect(0,11,128,1,1);

          for(int i = 0; i < 2; i++)
          {
            menuselected = menufirst4 + i;
            if(menuselected < table->rotates_count)
            {
              font_printf(3,14 + font_getHeight()*i,"RPM %03.0f", table->rotates[menuselected]);
              if(menuitem4 != menuselected)
              {
                font_printf(-125,14 + font_getHeight()*i,"%.1f", table->ignitions[menuitem3][menuselected]);
              }
              else if(menuitem4 == menuselected && DelayDiff(now, select_timeout) < 500000)
              {
                font_printf(-125,14 + font_getHeight()*i,"> %.1f", table->ignitions[menuitem3][menuselected]);
              }
              else if(DelayDiff(now, select_timeout) > 800000)
                select_timeout = now;

            }
          }

          lcd_rect(1,24,126,1,1);
          lcd_rect_solid(1,25,126,font_getHeight(),0);

          int cnt = table->rotates_count;
          float x = -1.0f ,y = -1.0f , prevx = -1.0f , prevy = -1.0f ;
          float max = -INFINITY;
          float min = 0;
          float depmin = INFINITY;
          float depmax = -INFINITY;
          float value, dep;
          float cx,cy;
          for(int j = 0; j < table->pressures_count; j++)
          {
            for(int i = 0; i < cnt; i++)
            {
              value = table->ignitions[j][i];
              if(value > max)
                max = value;
              if(value < min)
                min = value;
            }
          }

          for(int i = 0; i < cnt; i++)
          {
            dep = table->rotates[i];
            if(dep > depmax)
              depmax = dep;
            if(dep < depmin)
              depmin = dep;
          }
          for(int i = 0; i < cnt; i++)
          {
            dep = table->rotates[i];
            value = table->ignitions[menuitem3][i];
            x = (126.0f * log10f(1.0f + (dep - depmin) / (depmax-depmin) * 9.0f)) + 1.0f;
            y = 63 - (value - min) / (max-min) * 37;

            if(prevx == -1.0f && prevy == -1.0f)
              prevx = x, prevy = y;

            lcd_line(prevx, prevy, x, y, 1);

            prevx = x;
            prevy = y;
          }

          cx = (126.0f * log10f(1.0f + (StatusRPM - depmin) / (depmax-depmin) * 9.0f)) + 1.0f;
          cy = 63 - (StatusIgnition - table->octane_corrector - min) / (max-min) * 37;
          if(cy > 22 && cx > 0 && cx < 128)
            lcd_circle5x5fill(cx,cy);

          cx = (126.0f * log10f(1.0f + (table->rotates[menuitem4] - depmin) / (depmax-depmin) * 9.0f)) + 1.0f;
          cy = 63 - (table->ignitions[menuitem3][menuitem4] - min) / (max-min) * 37;
          if(cy > 22 && cx > 0 && cx < 128)
            lcd_circle5x5(cx,cy);

          font_printf(65,2,"%03.0f", rpm);
          font_printf(93,2,"%04.0f", pres);
          font_printf(94,63 - font_getHeight(),"%5.1fd", ign);


          lcd_update();
        }

        if(BUT_LEFT || (BUT_LEFT_PRESS && BUT_LEFT_TIME > 400))
        {
          display_timeout = 0;
          if(BUT_LEFT_TIME > 100) BUT_LEFT_TIME -= 40;
          if(StatusSynchronized && tablesetupitem->step)
          {
            if(table->ignitions[menuitem3][menuitem4] - tablesetupitem->step < tablesetupitem->min)
              table->ignitions[menuitem3][menuitem4] = tablesetupitem->min;
            else table->ignitions[menuitem3][menuitem4] -= tablesetupitem->step;

            acis_apply_parameter(&table->ignitions[menuitem3][menuitem4], sizeof(float));
          }
          select_timeout = now;
          BUT_LEFT = 0;

        }

        if(BUT_RIGHT || (BUT_RIGHT_PRESS && BUT_RIGHT_TIME > 400))
        {
          display_timeout = 0;
          if(BUT_RIGHT_TIME > 100) BUT_RIGHT_TIME -= 40;
          if(StatusSynchronized && tablesetupitem->step)
          {
            if(table->ignitions[menuitem3][menuitem4] + tablesetupitem->step > tablesetupitem->max)
              table->ignitions[menuitem3][menuitem4] = tablesetupitem->max;
            else table->ignitions[menuitem3][menuitem4] += tablesetupitem->step;

            acis_apply_parameter(&table->ignitions[menuitem3][menuitem4], sizeof(float));
          }
          select_timeout = now;
          BUT_RIGHT = 0;

        }

        if(BUT_CANCEL)
        {
          display_timeout = 0;
          eMenuItem = MenuTableSetupIgnitionsSelect;
          value_timeout = 0;
          select_timeout = now;
          BUT_UP = 0;
          BUT_DOWN = 0;
          BUT_LEFT = 0;
          BUT_RIGHT = 0;
          BUT_CANCEL = 0;
          BUT_ENTER = 0;
        }

        if(BUT_DOWN)
        {
          display_timeout = 0;
          menuitem4++;
          select_timeout = now;
          BUT_DOWN = 0;
        }

        if(BUT_UP)
        {
          display_timeout = 0;
          menuitem4--;
          select_timeout = now;
          BUT_UP = 0;
        }

        if(menuitem4 >= table->rotates_count) menuitem4 = table->rotates_count - 1, display_timeout = 0;
        if(menuitem4 < 0) menuitem4 = 0, display_timeout = 0;
        menufirst4 = menuitem4;

        break;
      }
      case MenuPcConnected :
      {
        if(DelayDiff(now, display_timeout) > 500000)
        {
          display_timeout = now;
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_arialb_16);
          font_printf(10,18,"PC Connected");
          lcd_update();
        }
        break;
      }

      case MenuSynchronizing :
      {
        if(DelayDiff(now, display_timeout) > 50000)
        {
          display_timeout = now;
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_arialb_16);
          font_printf(0,18,"Synchronizing");

          font_printf(45 + cnt * 3,18+font_getHeight(),".");
          font_printf(45 + (((cnt + 1) % 12) * 3),18+font_getHeight(),".");
          font_printf(45 + (((cnt + 2) % 12) * 3),18+font_getHeight(),".");

          cnt = (cnt + 1) % 12;

          lcd_update();
        }
        break;
      }

      case MenuTableConfigSave :
      {
        if(DelayDiff(now, display_timeout) > 50000)
        {
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_arialb_16);
          font_printf(0,18,"Saving...");
          eMenuItem = MenuSynchronizing;
          NeedSave = 1;
          lcd_update();
        }
        break;
      }

      case MenuTableConfigRestore :
      {
        if(DelayDiff(now, display_timeout) > 50000)
        {
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_arialb_16);
          font_printf(0,18,"Restoring...");
          eMenuItem = MenuSynchronizing;
          NeedLoad = 1;
          lcd_update();
        }
        break;
      }

      case MenuSyncError :
      {
        if(DelayDiff(now, display_timeout) > 500000)
        {
          display_timeout = now;
          lcd_clear();
          lcd_rect(0,0,128,64,1);
          font_setFont(&rre_arialb_16);
          font_printf(0,18,"Synchronizing");
          font_printf(0,18+font_getHeight(),"ERROR (%d)",SyncError);
          lcd_update();
        }
        if(DelayDiff(now, last_menu_switch) > 2000000)
        {
          SyncError = 0;
          eMenuItem = MenuMain;
        }
        break;
      }
      default :
        eMenuItem = MenuMain;
        break;
    }
  }

}

static void acis_sender_task(void * argument)
{
  uint8_t sending = 0;
  uint8_t destination = 0;
  uint8_t size = 0;
  uint8_t * pnt;
  HAL_StatusTypeDef status;

  while(1)
  {
    do
    {
      if(!sending && protGetSize(&fifoSendingQueue) > 4)
      {
        protLook(&fifoSendingQueue,1,&size);
        protLook(&fifoSendingQueue,2,&destination);
        if(protGetSize(&fifoSendingQueue) >= size)
        {
          pnt = buffSendingBuffer;
          for(int i = 0; i < size; i++)
            protPull(&fifoSendingQueue, pnt++);
          if(destination)
            sending = 1;
        }
      }
      if(sending)
      {
        status = acis_send_command(destination, buffSendingBuffer, size);
        if(status != HAL_BUSY)
        {
          sending = 0;
          continue;
        }
      }
    } while(0);
    osDelay(1);
  }

}

void acis_main_task(void * argument)
{
  uint32_t DragStatusOld = 0;
  uint32_t LastPacket = 0;
  uint32_t LastDragPacket = 0;
  uint32_t LastGeneralStatusPacket = 0;
  uint32_t LastFuelSwitchPacket = 0;
  uint8_t LastFuelSwitchPos = 0xFF;
  uint8_t FuelSwitchPos = 0;
  uint32_t now;
  protInit(&fifoSendingQueue, buffSendingQueue, 1, SENDING_QUEUE_SIZE);
  tGuiHandler = osThreadNew(acis_gui_task, NULL, &cTaskAttributes);
  tSenderHandler = osThreadNew(acis_sender_task, NULL, &cTaskAttributes);
  memset(DragPointsRaw, 0, sizeof(DragPointsRaw));
  StatusSynchronizing = 1; //Read config
  while(1)
  {
    now = Delay_Tick;
    if(StatusSynchronizing == 0)
    {
      if(NeedLoad)
      {
        SyncSize = 0;
        SyncStep = 0;
        StatusSynchronizing = 1;
      }
      else if(NeedSave)
      {
        SyncSize = 0;
        SyncStep = 0;
        StatusSynchronizing = 2;
      }
    }

    if(StatusSynchronizing == 1)
    {
      StatusSynchronized = 0;
      if(SyncStep == 0)
      {
        if(NeedLoad)
        {
          if(SyncSize == 0)
          {
            SyncSize = 1;
            FlashRequestDone = 0;
            SyncRequestDone = 0;
            PK_RestoreConfig.Destination = etrACIS;
            protPushSequence(&fifoSendingQueue, &PK_RestoreConfig, sizeof(PK_RestoreConfig));
            LastPacket = now;
          }
          else
          {
            if(FlashRequestDone)
            {
              FlashRequestDone = 0;
              SyncRequestDone = 0;
              SyncStep++;
              NeedLoad = 0;
              SyncSize = 0;
            }
            else if(DelayDiff(now, LastPacket) > 3000000 || SyncError > 0)
            {
              StatusSynchronizing = 0;
              SyncStep = 0;
              SyncSize = 0;
              NeedLoad = 0;
              if(SyncError == 0)
                SyncError = 1;
            }
          }
        }
        else SyncStep = 1;
      }
      else if(SyncStep == 1)
      {
        if(SyncSize == 0)
        {
          SyncSize = (uint32_t)&acis_config.tables[0] - (uint32_t)&acis_config;
          SyncLeft = SyncSize;
          SyncOffset = 0;
          SyncRequestDone = 0;
          PK_ConfigMemoryRequest.Destination = etrACIS;
          PK_ConfigMemoryRequest.configsize = SyncSize;
          PK_ConfigMemoryRequest.offset = SyncOffset;

          if(SyncLeft > PACKET_CONFIG_MAX_SIZE)
          {
            PK_ConfigMemoryRequest.offset = SyncOffset;
            PK_ConfigMemoryRequest.size = PACKET_CONFIG_MAX_SIZE;
            SyncLeft -= PACKET_CONFIG_MAX_SIZE;
            SyncOffset += PACKET_CONFIG_MAX_SIZE;
          }
          else
          {
            PK_ConfigMemoryRequest.offset = SyncOffset;
            PK_ConfigMemoryRequest.size = SyncLeft;
            SyncOffset += SyncLeft;
            SyncLeft = 0;
          }
          protPushSequence(&fifoSendingQueue, &PK_ConfigMemoryRequest, sizeof(PK_ConfigMemoryRequest));
          LastPacket = now;
        }
        else
        {
          if(SyncRequestDone)
          {
            SyncRequestDone = 0;
            if(SyncLeft == 0)
            {
              SyncStep++;
              SyncSize = 0;
            }
            else
            {
              if(SyncLeft > PACKET_CONFIG_MAX_SIZE)
              {
                PK_ConfigMemoryRequest.offset = SyncOffset;
                PK_ConfigMemoryRequest.size = PACKET_CONFIG_MAX_SIZE;
                SyncLeft -= PACKET_CONFIG_MAX_SIZE;
                SyncOffset += PACKET_CONFIG_MAX_SIZE;
              }
              else
              {
                PK_ConfigMemoryRequest.offset = SyncOffset;
                PK_ConfigMemoryRequest.size = SyncLeft;
                SyncOffset += SyncLeft;
                SyncLeft = 0;
              }
              protPushSequence(&fifoSendingQueue, &PK_ConfigMemoryRequest, sizeof(PK_ConfigMemoryRequest));
              LastPacket = now;
            }

          }
          else if(DelayDiff(now, LastPacket) > 1000000 || SyncError > 0)
          {
            StatusSynchronizing = 0;
            SyncStep = 0;
            SyncSize = 0;
            if(SyncError == 0)
              SyncError = 1;
          }
        }
      }
      else if(SyncStep > 1 && SyncStep <= TABLE_SETUPS_MAX + 1)
      {
        if(SyncSize == 0)
        {
          SyncSize = sizeof(sAcisIgnTable);
          SyncLeft = SyncSize;
          SyncOffset = 0;
          SyncRequestDone = 0;
          PK_TableMemoryRequest.Destination = etrACIS;
          PK_TableMemoryRequest.tablesize = SyncSize;
          PK_TableMemoryRequest.table = SyncStep - 2;
          PK_TableMemoryRequest.offset = SyncOffset;

          if(SyncLeft > PACKET_TABLE_MAX_SIZE)
          {
            PK_TableMemoryRequest.offset = SyncOffset;
            PK_TableMemoryRequest.size = PACKET_TABLE_MAX_SIZE;
            SyncLeft -= PACKET_TABLE_MAX_SIZE;
            SyncOffset += PACKET_TABLE_MAX_SIZE;
          }
          else
          {
            PK_TableMemoryRequest.offset = SyncOffset;
            PK_TableMemoryRequest.size = SyncLeft;
            SyncOffset += SyncLeft;
            SyncLeft = 0;
          }
          protPushSequence(&fifoSendingQueue, &PK_TableMemoryRequest, sizeof(PK_TableMemoryRequest));
          LastPacket = now;
        }
        else
        {
          if(SyncRequestDone)
          {
            SyncRequestDone = 0;
            if(SyncLeft == 0)
            {
              SyncStep++;
              SyncSize = 0;
            }
            else
            {
              if(SyncLeft > PACKET_CONFIG_MAX_SIZE)
              {
                PK_TableMemoryRequest.offset = SyncOffset;
                PK_TableMemoryRequest.size = PACKET_TABLE_MAX_SIZE;
                SyncLeft -= PACKET_TABLE_MAX_SIZE;
                SyncOffset += PACKET_TABLE_MAX_SIZE;
              }
              else
              {
                PK_TableMemoryRequest.offset = SyncOffset;
                PK_TableMemoryRequest.size = SyncLeft;
                SyncOffset += SyncLeft;
                SyncLeft = 0;
              }
              protPushSequence(&fifoSendingQueue, &PK_TableMemoryRequest, sizeof(PK_TableMemoryRequest));
              LastPacket = now;
            }

          }
          else if(DelayDiff(now, LastPacket) > 1000000 || SyncError > 0)
          {
            StatusSynchronizing = 0;
            SyncStep = 0;
            SyncSize = 0;
            if(SyncError == 0)
              SyncError = 1;
          }
        }
      }
      else
      {
        StatusSynchronizing = 0;
        SyncStep = 0;
        SyncSize = 0;
        SyncError = 0;
        StatusSynchronized = 1;
      }
    }
    else if(StatusSynchronizing == 2)
    {
      StatusSynchronized = 0;
      if(SyncStep == 0)
      {
        if(SyncSize == 0)
        {
          SyncSize = (uint32_t)&acis_config.tables[0] - (uint32_t)&acis_config;
          SyncLeft = SyncSize;
          SyncOffset = 0;
          SyncRequestDone = 0;
          PK_ConfigMemoryData.Destination = etrACIS;
          PK_ConfigMemoryData.configsize = SyncSize;
          PK_ConfigMemoryData.offset = SyncOffset;

          if(SyncLeft > PACKET_CONFIG_MAX_SIZE)
          {
            PK_ConfigMemoryData.offset = SyncOffset;
            PK_ConfigMemoryData.size = PACKET_CONFIG_MAX_SIZE;
            SyncLeft -= PACKET_CONFIG_MAX_SIZE;
            SyncOffset += PACKET_CONFIG_MAX_SIZE;
          }
          else
          {
            PK_ConfigMemoryData.offset = SyncOffset;
            PK_ConfigMemoryData.size = SyncLeft;
            SyncOffset += SyncLeft;
            SyncLeft = 0;
          }

          memcpy(&PK_ConfigMemoryData.data[0], &((uint8_t*)&acis_config)[PK_ConfigMemoryData.offset], PK_ConfigMemoryData.size);
          memset(&PK_ConfigMemoryData.data[PK_ConfigMemoryData.size], 0, sizeof(PK_ConfigMemoryData.data) - PK_ConfigMemoryData.size);
          PK_ConfigMemoryData.crc = CRC16_Generate(PK_ConfigMemoryData.data, sizeof(PK_ConfigMemoryData.data));

          protPushSequence(&fifoSendingQueue, &PK_ConfigMemoryData, sizeof(PK_ConfigMemoryData));
          LastPacket = now;
        }
        else
        {
          if(SyncRequestDone)
          {
            SyncRequestDone = 0;
            if(SyncLeft == 0)
            {
              SyncStep++;
              SyncSize = 0;
            }
            else
            {
              if(SyncLeft > PACKET_CONFIG_MAX_SIZE)
              {
                PK_ConfigMemoryData.offset = SyncOffset;
                PK_ConfigMemoryData.size = PACKET_CONFIG_MAX_SIZE;
                SyncLeft -= PACKET_CONFIG_MAX_SIZE;
                SyncOffset += PACKET_CONFIG_MAX_SIZE;
              }
              else
              {
                PK_ConfigMemoryData.offset = SyncOffset;
                PK_ConfigMemoryData.size = SyncLeft;
                SyncOffset += SyncLeft;
                SyncLeft = 0;
              }

              memcpy(&PK_ConfigMemoryData.data[0], &((uint8_t*)&acis_config)[PK_ConfigMemoryData.offset], PK_ConfigMemoryData.size);
              memset(&PK_ConfigMemoryData.data[PK_ConfigMemoryData.size], 0, sizeof(PK_ConfigMemoryData.data) - PK_ConfigMemoryData.size);
              PK_ConfigMemoryData.crc = CRC16_Generate(PK_ConfigMemoryData.data, sizeof(PK_ConfigMemoryData.data));

              protPushSequence(&fifoSendingQueue, &PK_ConfigMemoryData, sizeof(PK_ConfigMemoryData));
              LastPacket = now;
            }

          }
          else if(DelayDiff(now, LastPacket) > 1000000 || SyncError > 0)
          {
            StatusSynchronizing = 0;
            SyncStep = 0;
            SyncSize = 0;
            if(SyncError == 0)
              SyncError = 1;
          }
        }
      }
      else if(SyncStep > 0 && SyncStep <= TABLE_SETUPS_MAX)
      {
        if(SyncSize == 0)
        {
          SyncSize = sizeof(sAcisIgnTable);
          SyncLeft = SyncSize;
          SyncOffset = 0;
          SyncRequestDone = 0;
          PK_TableMemoryData.Destination = etrACIS;
          PK_TableMemoryData.tablesize = SyncSize;
          PK_TableMemoryData.table = SyncStep - 1;
          PK_TableMemoryData.offset = SyncOffset;

          if(SyncLeft > PACKET_TABLE_MAX_SIZE)
          {
            PK_TableMemoryData.offset = SyncOffset;
            PK_TableMemoryData.size = PACKET_TABLE_MAX_SIZE;
            SyncLeft -= PACKET_TABLE_MAX_SIZE;
            SyncOffset += PACKET_TABLE_MAX_SIZE;
          }
          else
          {
            PK_TableMemoryData.offset = SyncOffset;
            PK_TableMemoryData.size = SyncLeft;
            SyncOffset += SyncLeft;
            SyncLeft = 0;
          }

          memcpy(&PK_TableMemoryData.data[0], &((uint8_t*)&acis_config.tables[PK_TableMemoryData.table])[PK_TableMemoryData.offset], PK_TableMemoryData.size);
          memset(&PK_TableMemoryData.data[PK_TableMemoryData.size], 0, sizeof(PK_TableMemoryData.data) - PK_TableMemoryData.size);
          PK_TableMemoryData.crc = CRC16_Generate(PK_TableMemoryData.data, sizeof(PK_TableMemoryData.data));

          protPushSequence(&fifoSendingQueue, &PK_TableMemoryData, sizeof(PK_TableMemoryData));
          LastPacket = now;
        }
        else
        {
          if(SyncRequestDone)
          {
            SyncRequestDone = 0;
            if(SyncLeft == 0)
            {
              SyncStep++;
              SyncSize = 0;
            }
            else
            {
              if(SyncLeft > PACKET_CONFIG_MAX_SIZE)
              {
                PK_TableMemoryData.offset = SyncOffset;
                PK_TableMemoryData.size = PACKET_TABLE_MAX_SIZE;
                SyncLeft -= PACKET_TABLE_MAX_SIZE;
                SyncOffset += PACKET_TABLE_MAX_SIZE;
              }
              else
              {
                PK_TableMemoryData.offset = SyncOffset;
                PK_TableMemoryData.size = SyncLeft;
                SyncOffset += SyncLeft;
                SyncLeft = 0;
              }

              memcpy(&PK_TableMemoryData.data[0], &((uint8_t*)&acis_config.tables[PK_TableMemoryData.table])[PK_TableMemoryData.offset], PK_TableMemoryData.size);
              memset(&PK_TableMemoryData.data[PK_TableMemoryData.size], 0, sizeof(PK_TableMemoryData.data) - PK_TableMemoryData.size);
              PK_TableMemoryData.crc = CRC16_Generate(PK_TableMemoryData.data, sizeof(PK_TableMemoryData.data));

              protPushSequence(&fifoSendingQueue, &PK_TableMemoryData, sizeof(PK_TableMemoryData));
              LastPacket = now;
            }

          }
          else if(DelayDiff(now, LastPacket) > 1000000 || SyncError > 0)
          {
            StatusSynchronizing = 0;
            SyncStep = 0;
            SyncSize = 0;
            if(SyncError == 0)
              SyncError = 1;
          }
        }
      }
      else if(SyncStep == TABLE_SETUPS_MAX + 1)
      {
        if(NeedSave)
        {
          if(SyncSize == 0)
          {
            SyncSize = 1;
            FlashRequestDone = 0;
            SyncRequestDone = 0;
            PK_SaveConfig.Destination = etrACIS;
            protPushSequence(&fifoSendingQueue, &PK_SaveConfig, sizeof(PK_SaveConfig));
            LastPacket = now;
          }
          else
          {
            if(FlashRequestDone)
            {
              FlashRequestDone = 0;
              SyncRequestDone = 0;
              SyncStep++;
              NeedSave = 0;
              SyncSize = 0;
            }
            else if(DelayDiff(now, LastPacket) > 15000000 || SyncError > 0)
            {
              StatusSynchronizing = 0;
              SyncStep = 0;
              SyncSize = 0;
              NeedSave = 0;
              if(SyncError == 0)
                SyncError = 1;
            }
          }
        }
        else SyncStep++;
      }
      else
      {
        StatusSynchronizing = 0;
        SyncStep = 0;
        SyncSize = 0;
        SyncError = 0;
        StatusSynchronized = 1;
      }
    }
    else
    {

    }

    if(DelayDiff(now, LastGeneralStatusPacket) > (eMenuItem >= MenuTableSetupIgnitionsSelect ? 30000 : 50000))
    {
      LastGeneralStatusPacket = now;
      PK_GeneralStatusRequest.Destination = etrACIS;
      protPushSequence(&fifoSendingQueue, &PK_GeneralStatusRequest, sizeof(PK_GeneralStatusRequest));
    }

    FuelSwitchPos = SW_FUEL1 ? 1 : SW_FUEL2 ? 2 : 0;
    if(FuelSwitchPos != LastFuelSwitchPos || DelayDiff(now, LastFuelSwitchPacket) > 500000)
    {
      LastFuelSwitchPos = FuelSwitchPos;
      LastFuelSwitchPacket = now;
      PK_FuelSwitch.Destination = etrACIS;
      PK_FuelSwitch.FuelSwitchPos = FuelSwitchPos;
      protPushSequence(&fifoSendingQueue, &PK_FuelSwitch, sizeof(PK_FuelSwitch));

    }

    uint32_t drag = DragStatus;
    if(drag != DragStatusOld)
    {
      DragStatusOld = drag;
      switch(drag)
      {
        case 0 :
          DragGraphReady = 0;
          PK_DragStop.Destination = etrACIS;
          PK_DragStop.FromRPM = DragRpmFrom;
          PK_DragStop.ToRPM = DragRpmTo;
          protPushSequence(&fifoSendingQueue, &PK_DragStop, sizeof(PK_DragStop));
          LastDragPacket = now;
          break;
        case 1 :
          DragGraphReady = 0;
          PK_DragStart.Destination = etrACIS;
          PK_DragStart.FromRPM = DragRpmFrom;
          PK_DragStart.ToRPM = DragRpmTo;
          protPushSequence(&fifoSendingQueue, &PK_DragStart, sizeof(PK_DragStart));
          LastDragPacket = now;
          break;
        default :
          break;
      }
    }

    if(drag == 1 || drag == 2)
    {
      if(DelayDiff(now, LastDragPacket) > 30000)
      {
        PK_DragUpdateRequest.Destination = etrACIS;
        PK_DragUpdateRequest.FromRPM = DragRpmFrom;
        PK_DragUpdateRequest.ToRPM = DragRpmTo;
        protPushSequence(&fifoSendingQueue, &PK_DragUpdateRequest, sizeof(PK_DragUpdateRequest));
        LastDragPacket = now;
      }
    }

    osDelay(1);

    if(DelayDiff(now, StatusPcLast) >= 2000000)
    {
      StatusPcLast += 1000000;
      StatusPcConnected = 0;
    }
  }

}

void acis_parse_command(eTransChannels xChaSrc, uint8_t * msgBuf, uint32_t length)
{
  uint16_t crc;
  uint32_t now = Delay_Tick;
  uint32_t offset,size, table, tablesize,configsize;
  uint32_t realconfigsize = (uint32_t)&acis_config.tables[0] - (uint32_t)&acis_config;

  switch(msgBuf[0])
  {
    case PK_PingID :
      PK_Copy(&PK_Ping, msgBuf);
      PK_Pong.RandomPong = PK_Ping.RandomPing;
      PK_Pong.Destination = xChaSrc;
      protPushSequence(&fifoSendingQueue, &PK_Pong, sizeof(PK_Pong));
      if(xChaSrc == etrPC)
        StatusPcLast = now;
      return;

    case PK_PongID :
      PK_Copy(&PK_Pong, msgBuf);
      (void)PK_Pong.RandomPong;
      if(xChaSrc == etrPC)
        StatusPcLast = now;
      return;
    default :
      break;
  }

  if(xChaSrc == etrACIS)
  {
    switch(msgBuf[0])
    {
      case PK_GeneralStatusResponseID :
        PK_Copy(&PK_GeneralStatusResponse, msgBuf);
        StatusIgnition = PK_GeneralStatusResponse.IgnitionAngle;
        StatusLoad = PK_GeneralStatusResponse.Load;
        StatusRPM = PK_GeneralStatusResponse.RPM ;
        StatusPressure = PK_GeneralStatusResponse.Pressure;
        StatusVoltage = PK_GeneralStatusResponse.Voltage;
        StatusTemperature = PK_GeneralStatusResponse.Temperature;
        StatusCheck = PK_GeneralStatusResponse.check;
        StatusValveNum = PK_GeneralStatusResponse.valvenum;
        StatusTableNum = PK_GeneralStatusResponse.tablenum;
        strcpy(StatusTableName, PK_GeneralStatusResponse.tablename);
        break;

      case PK_RestoreConfigAcknowledgeID :
        PK_Copy(&PK_RestoreConfigAcknowledge, msgBuf);
        if(PK_RestoreConfigAcknowledge.ErrorCode == 0)
        {
          if(StatusSynchronizing == 1 && NeedLoad)
            FlashRequestDone = 1;
        }
        else SyncError = PK_RestoreConfigAcknowledge.ErrorCode + 100;
        break;

      case PK_SaveConfigAcknowledgeID :
        PK_Copy(&PK_SaveConfigAcknowledge, msgBuf);
        if(PK_SaveConfigAcknowledge.ErrorCode == 0)
        {
          if(StatusSynchronizing == 2 && NeedSave)
            FlashRequestDone = 1;
        }
        else SyncError = PK_SaveConfigAcknowledge.ErrorCode + 120;
        break;

      case PK_PcConnectedID :
        PK_Copy(&PK_PcConnected, msgBuf);
        StatusPcLast = Delay_Tick;
        StatusPcConnected = 1;
        break;

      case PK_TableMemoryDataID :
        PK_Copy(&PK_TableMemoryData, msgBuf);

        if(PK_TableMemoryData.ErrorCode == 0)
        {
          PK_TableMemoryAcknowledge.Destination = xChaSrc;
          offset = PK_TableMemoryAcknowledge.offset = PK_TableMemoryData.offset;
          size = PK_TableMemoryAcknowledge.size = PK_TableMemoryData.size;
          table = PK_TableMemoryAcknowledge.table = PK_TableMemoryData.table;
          tablesize = PK_TableMemoryAcknowledge.tablesize = PK_TableMemoryData.tablesize;
          PK_TableMemoryAcknowledge.ErrorCode = 0;

          if(tablesize != sizeof(sAcisIgnTable))
            PK_TableMemoryAcknowledge.ErrorCode = 1;

          if(size + offset > sizeof(sAcisIgnTable))
            PK_TableMemoryAcknowledge.ErrorCode = 2;

          if(size > PACKET_TABLE_MAX_SIZE || size > sizeof(sAcisIgnTable))
            PK_TableMemoryAcknowledge.ErrorCode = 3;

          if(table >= TABLE_SETUPS_MAX)
            PK_TableMemoryAcknowledge.ErrorCode = 4;

          if(PK_TableMemoryAcknowledge.ErrorCode == 0)
          {
            crc = CRC16_Generate(PK_TableMemoryData.data, sizeof(PK_TableMemoryData.data));
            if(crc == PK_TableMemoryData.crc)
            {
              memcpy(&((uint8_t*)&acis_config.tables[table])[offset], &PK_TableMemoryData.data[0], size);

              if(StatusSynchronizing == 1)
                SyncRequestDone = 1;
            }
            else
              PK_TableMemoryAcknowledge.ErrorCode = 5;
          }

          protPushSequence(&fifoSendingQueue, &PK_TableMemoryAcknowledge, sizeof(PK_TableMemoryAcknowledge));
        }
        else SyncError = PK_TableMemoryData.ErrorCode + 20;
        break;

      case PK_ConfigMemoryDataID :
        PK_Copy(&PK_ConfigMemoryData, msgBuf);

        if(PK_ConfigMemoryData.ErrorCode == 0)
        {
          PK_ConfigMemoryAcknowledge.Destination = xChaSrc;
          offset = PK_ConfigMemoryAcknowledge.offset = PK_ConfigMemoryData.offset;
          size = PK_ConfigMemoryAcknowledge.size = PK_ConfigMemoryData.size;
          configsize = PK_ConfigMemoryAcknowledge.configsize = PK_ConfigMemoryData.configsize;
          PK_ConfigMemoryAcknowledge.ErrorCode = 0;

          if(configsize != realconfigsize)
            PK_ConfigMemoryData.ErrorCode = 1;

          if(size + offset > realconfigsize)
            PK_ConfigMemoryData.ErrorCode = 2;

          if(size > realconfigsize || size > PACKET_CONFIG_MAX_SIZE)
            PK_ConfigMemoryData.ErrorCode = 3;

          if(PK_ConfigMemoryAcknowledge.ErrorCode == 0)
          {
            crc = CRC16_Generate(PK_ConfigMemoryData.data, sizeof(PK_ConfigMemoryData.data));
            if(crc == PK_ConfigMemoryData.crc)
            {
              memcpy(&((uint8_t*)&acis_config)[offset], &PK_ConfigMemoryData.data[0], size);

              if(StatusSynchronizing == 1)
                SyncRequestDone = 1;
            }
            else
              PK_ConfigMemoryAcknowledge.ErrorCode = 5;
          }

          protPushSequence(&fifoSendingQueue, &PK_ConfigMemoryAcknowledge, sizeof(PK_ConfigMemoryAcknowledge));
        }
        else SyncError = PK_ConfigMemoryData.ErrorCode + 40;
        break;

      case PK_ConfigMemoryAcknowledgeID :
        PK_Copy(&PK_ConfigMemoryAcknowledge, msgBuf);
        if(PK_ConfigMemoryAcknowledge.ErrorCode == 0)
        {
          if(StatusSynchronizing == 2)
            SyncRequestDone = 1;
          if(Applying)
          {
            Applying = 0;
            ApplyError = 0;
          }
        }
        else
        {
          if(StatusSynchronizing == 2) SyncError = PK_ConfigMemoryAcknowledge.ErrorCode + 60;
          if(Applying) { Applying = 0; ApplyError = PK_ConfigMemoryAcknowledge.ErrorCode + 60; }
        }
        break;
      case PK_TableMemoryAcknowledgeID :
        PK_Copy(&PK_TableMemoryAcknowledge, msgBuf);
        if(PK_TableMemoryAcknowledge.ErrorCode == 0)
        {
          if(StatusSynchronizing == 2)
            SyncRequestDone = 1;
          if(Applying)
          {
            Applying = 0;
            ApplyError = 0;
          }
        }
        else
        {
          if(StatusSynchronizing == 2) SyncError = PK_TableMemoryAcknowledge.ErrorCode + 80;
          if(Applying) { Applying = 0; ApplyError = PK_TableMemoryAcknowledge.ErrorCode + 80; }
        }
        break;

      case PK_DragUpdateResponseID :
        PK_Copy(&PK_DragUpdateResponse, msgBuf);
        DragPointsRawCount = 0;
        DragGraphReady = 0;
        if(PK_DragUpdateResponse.ErrorCode > 0)
          DragStatus = 4;
        else if(DragStatus == 1 && PK_DragUpdateResponse.Started)
          DragStatus = 2;
        else if(PK_DragUpdateResponse.Completed)
          DragStatus = 3;
        DragTime = PK_DragUpdateResponse.Time / 1000000.0f;

        if(DragStatus == 3 || DragStatus == 4)
        {
          DragPointsRawCount = PK_DragUpdateResponse.TotalPoints;
          DragPointsRawCountPtr = 0;
          if(DragPointsRawCount - DragPointsRawCountPtr > 0)
          {
            PK_DragPointRequest.Destination = etrACIS;
            PK_DragPointRequest.FromRPM = DragRpmFrom;
            PK_DragPointRequest.ToRPM = DragRpmTo;
            PK_DragPointRequest.PointNumber = DragPointsRawCountPtr;
            protPushSequence(&fifoSendingQueue, &PK_DragPointRequest, sizeof(PK_DragPointRequest));
          }
        }
        break;

      case PK_DragPointResponseID :
        PK_Copy(&PK_DragPointResponse, msgBuf);
        if(DragPointsRawCountPtr == PK_DragPointRequest.PointNumber && DragStatus > 2)
        {
          DragPointsRaw[PK_DragPointResponse.PointNumber] = PK_DragPointResponse.Point;
          DragPointsRawCountPtr++;
          if(DragPointsRawCount - DragPointsRawCountPtr > 0)
          {
            DragGraphReady = 0;
            PK_DragPointRequest.Destination = etrACIS;
            PK_DragPointRequest.FromRPM = DragRpmFrom;
            PK_DragPointRequest.ToRPM = DragRpmTo;
            PK_DragPointRequest.PointNumber = DragPointsRawCountPtr;
            protPushSequence(&fifoSendingQueue, &PK_DragPointRequest, sizeof(PK_DragPointRequest));
          }
          else
          {
            DragGraphReady = 1;
          }
        }
        break;

      default:
        break;
    }
  }
}

static inline HAL_StatusTypeDef acis_send_command(eTransChannels xChaDst, void * msgBuf, uint32_t length)
{
  HAL_StatusTypeDef result = HAL_BUSY;
  int8_t status;

  status = xSender(xChaDst, (uint8_t*)msgBuf, length);

  if(status == -1)
  {
    StatusTimeout = 1;
    StatusCheck = 1;
    result = HAL_TIMEOUT;
  }
  else if(status == 1)
  {
    StatusTimeout = 0;
    result = HAL_OK;
  }

  return result;
}




static void setconfig_standart16l(sAcisConfig * config, uint8_t i)
{

  config->tables[i].rotates_count = 16;
  config->tables[i].rotates[0] = 600;
  config->tables[i].rotates[1] = 720;
  config->tables[i].rotates[2] = 840;
  config->tables[i].rotates[3] = 990;
  config->tables[i].rotates[4] = 1170;
  config->tables[i].rotates[5] = 1380;
  config->tables[i].rotates[6] = 1650;
  config->tables[i].rotates[7] = 1950;
  config->tables[i].rotates[8] = 2310;
  config->tables[i].rotates[9] = 2730;
  config->tables[i].rotates[10] = 3210;
  config->tables[i].rotates[11] = 3840;
  config->tables[i].rotates[12] = 4530;
  config->tables[i].rotates[13] = 5370;
  config->tables[i].rotates[14] = 6360;
  config->tables[i].rotates[15] = 7500;

  config->tables[i].pressures_count = 16;
  config->tables[i].pressures[0] = 28000.0f;
  config->tables[i].pressures[1] = 32800.0f;
  config->tables[i].pressures[2] = 37600.0f;
  config->tables[i].pressures[3] = 42400.0f;
  config->tables[i].pressures[4] = 47200.0f;
  config->tables[i].pressures[5] = 52000.0f;
  config->tables[i].pressures[6] = 56800.0f;
  config->tables[i].pressures[7] = 61600.0f;
  config->tables[i].pressures[8] = 66400.0f;
  config->tables[i].pressures[9] = 71200.0f;
  config->tables[i].pressures[10] = 76000.0f;
  config->tables[i].pressures[11] = 80800.0f;
  config->tables[i].pressures[12] = 85600.0f;
  config->tables[i].pressures[13] = 90400.0f;
  config->tables[i].pressures[14] = 95200.0f;
  config->tables[i].pressures[15] = 100000.0f;

  config->tables[i].ignitions[0][0]  = 14.5f;
  config->tables[i].ignitions[0][1]  = 16.1f;
  config->tables[i].ignitions[0][2]  = 18.0f;
  config->tables[i].ignitions[0][3]  = 20.9f;
  config->tables[i].ignitions[0][4]  = 24.7f;
  config->tables[i].ignitions[0][5]  = 29.7f;
  config->tables[i].ignitions[0][6]  = 37.8f;
  config->tables[i].ignitions[0][7]  = 40.9f;
  config->tables[i].ignitions[0][8]  = 42.6f;
  config->tables[i].ignitions[0][9]  = 43.4f;
  config->tables[i].ignitions[0][10] = 44.0f;
  config->tables[i].ignitions[0][11] = 44.4f;
  config->tables[i].ignitions[0][12] = 44.3f;
  config->tables[i].ignitions[0][13] = 44.3f;
  config->tables[i].ignitions[0][14] = 44.8f;
  config->tables[i].ignitions[0][15] = 45.0f;

  config->tables[i].ignitions[1][0]  = 16.1f;
  config->tables[i].ignitions[1][1]  = 18.0f;
  config->tables[i].ignitions[1][2]  = 20.1f;
  config->tables[i].ignitions[1][3]  = 23.1f;
  config->tables[i].ignitions[1][4]  = 26.6f;
  config->tables[i].ignitions[1][5]  = 31.0f;
  config->tables[i].ignitions[1][6]  = 38.1f;
  config->tables[i].ignitions[1][7]  = 41.0f;
  config->tables[i].ignitions[1][8]  = 42.6f;
  config->tables[i].ignitions[1][9]  = 43.4f;
  config->tables[i].ignitions[1][10] = 43.0f;
  config->tables[i].ignitions[1][11] = 44.4f;
  config->tables[i].ignitions[1][12] = 44.3f;
  config->tables[i].ignitions[1][13] = 44.3f;
  config->tables[i].ignitions[1][14] = 44.8f;
  config->tables[i].ignitions[1][15] = 45.0f;

  config->tables[i].ignitions[2][0]  = 17.8f;
  config->tables[i].ignitions[2][1]  = 19.7f;
  config->tables[i].ignitions[2][2]  = 21.5f;
  config->tables[i].ignitions[2][3]  = 24.2f;
  config->tables[i].ignitions[2][4]  = 27.6f;
  config->tables[i].ignitions[2][5]  = 31.7f;
  config->tables[i].ignitions[2][6]  = 38.3f;
  config->tables[i].ignitions[2][7]  = 41.1f;
  config->tables[i].ignitions[2][8]  = 42.6f;
  config->tables[i].ignitions[2][9]  = 43.4f;
  config->tables[i].ignitions[2][10] = 43.0f;
  config->tables[i].ignitions[2][11] = 44.4f;
  config->tables[i].ignitions[2][12] = 44.3f;
  config->tables[i].ignitions[2][13] = 44.3f;
  config->tables[i].ignitions[2][14] = 44.8f;
  config->tables[i].ignitions[2][15] = 45.0f;

  config->tables[i].ignitions[3][0]  = 18.7f;
  config->tables[i].ignitions[3][1]  = 20.5f;
  config->tables[i].ignitions[3][2]  = 22.2f;
  config->tables[i].ignitions[3][3]  = 24.6f;
  config->tables[i].ignitions[3][4]  = 27.7f;
  config->tables[i].ignitions[3][5]  = 31.8f;
  config->tables[i].ignitions[3][6]  = 38.2f;
  config->tables[i].ignitions[3][7]  = 40.9f;
  config->tables[i].ignitions[3][8]  = 42.4f;
  config->tables[i].ignitions[3][9]  = 43.2f;
  config->tables[i].ignitions[3][10] = 43.7f;
  config->tables[i].ignitions[3][11] = 44.1f;
  config->tables[i].ignitions[3][12] = 44.0f;
  config->tables[i].ignitions[3][13] = 44.1f;
  config->tables[i].ignitions[3][14] = 44.7f;
  config->tables[i].ignitions[3][15] = 44.9f;

  config->tables[i].ignitions[4][0]  = 18.7f;
  config->tables[i].ignitions[4][1]  = 20.4f;
  config->tables[i].ignitions[4][2]  = 21.9f;
  config->tables[i].ignitions[4][3]  = 24.3f;
  config->tables[i].ignitions[4][4]  = 27.3f;
  config->tables[i].ignitions[4][5]  = 31.3f;
  config->tables[i].ignitions[4][6]  = 37.6f;
  config->tables[i].ignitions[4][7]  = 40.3f;
  config->tables[i].ignitions[4][8]  = 41.7f;
  config->tables[i].ignitions[4][9]  = 42.4f;
  config->tables[i].ignitions[4][10] = 42.9f;
  config->tables[i].ignitions[4][11] = 43.2f;
  config->tables[i].ignitions[4][12] = 43.1f;
  config->tables[i].ignitions[4][13] = 43.3f;
  config->tables[i].ignitions[4][14] = 43.0f;
  config->tables[i].ignitions[4][15] = 44.2f;

  config->tables[i].ignitions[5][0]  = 17.4f;
  config->tables[i].ignitions[5][1]  = 19.4f;
  config->tables[i].ignitions[5][2]  = 20.9f;
  config->tables[i].ignitions[5][3]  = 23.2f;
  config->tables[i].ignitions[5][4]  = 26.2f;
  config->tables[i].ignitions[5][5]  = 30.1f;
  config->tables[i].ignitions[5][6]  = 36.3f;
  config->tables[i].ignitions[5][7]  = 39.1f;
  config->tables[i].ignitions[5][8]  = 40.6f;
  config->tables[i].ignitions[5][9]  = 41.3f;
  config->tables[i].ignitions[5][10] = 41.9f;
  config->tables[i].ignitions[5][11] = 42.2f;
  config->tables[i].ignitions[5][12] = 42.1f;
  config->tables[i].ignitions[5][13] = 42.3f;
  config->tables[i].ignitions[5][14] = 42.9f;
  config->tables[i].ignitions[5][15] = 43.1f;

  config->tables[i].ignitions[6][0]  = 14.9f;
  config->tables[i].ignitions[6][1]  = 17.2f;
  config->tables[i].ignitions[6][2]  = 18.7f;
  config->tables[i].ignitions[6][3]  = 21.1f;
  config->tables[i].ignitions[6][4]  = 24.3f;
  config->tables[i].ignitions[6][5]  = 28.4f;
  config->tables[i].ignitions[6][6]  = 34.3f;
  config->tables[i].ignitions[6][7]  = 37.3f;
  config->tables[i].ignitions[6][8]  = 38.9f;
  config->tables[i].ignitions[6][9]  = 39.8f;
  config->tables[i].ignitions[6][10] = 40.6f;
  config->tables[i].ignitions[6][11] = 41.0f;
  config->tables[i].ignitions[6][12] = 41.0f;
  config->tables[i].ignitions[6][13] = 41.3f;
  config->tables[i].ignitions[6][14] = 41.8f;
  config->tables[i].ignitions[6][15] = 42.0f;

  config->tables[i].ignitions[7][0]  = 12.2f;
  config->tables[i].ignitions[7][1]  = 14.5f;
  config->tables[i].ignitions[7][2]  = 15.9f;
  config->tables[i].ignitions[7][3]  = 17.9f;
  config->tables[i].ignitions[7][4]  = 21.5f;
  config->tables[i].ignitions[7][5]  = 26.0f;
  config->tables[i].ignitions[7][6]  = 31.9f;
  config->tables[i].ignitions[7][7]  = 35.0f;
  config->tables[i].ignitions[7][8]  = 36.8f;
  config->tables[i].ignitions[7][9]  = 37.9f;
  config->tables[i].ignitions[7][10] = 38.9f;
  config->tables[i].ignitions[7][11] = 39.5f;
  config->tables[i].ignitions[7][12] = 39.5f;
  config->tables[i].ignitions[7][13] = 40.2f;
  config->tables[i].ignitions[7][14] = 40.8f;
  config->tables[i].ignitions[7][15] = 41.0f;

  config->tables[i].ignitions[8][0]  = 10.2f;
  config->tables[i].ignitions[8][1]  = 11.9f;
  config->tables[i].ignitions[8][2]  = 13.1f;
  config->tables[i].ignitions[8][3]  = 14.9f;
  config->tables[i].ignitions[8][4]  = 18.3f;
  config->tables[i].ignitions[8][5]  = 22.8f;
  config->tables[i].ignitions[8][6]  = 28.4f;
  config->tables[i].ignitions[8][7]  = 31.9f;
  config->tables[i].ignitions[8][8]  = 34.2f;
  config->tables[i].ignitions[8][9]  = 35.8f;
  config->tables[i].ignitions[8][10] = 37.2f;
  config->tables[i].ignitions[8][11] = 38.1f;
  config->tables[i].ignitions[8][12] = 38.7f;
  config->tables[i].ignitions[8][13] = 39.1f;
  config->tables[i].ignitions[8][14] = 39.7f;
  config->tables[i].ignitions[8][15] = 39.9f;

  config->tables[i].ignitions[9][0]  = 8.5f;
  config->tables[i].ignitions[9][1]  = 9.7f;
  config->tables[i].ignitions[9][2]  = 10.8f;
  config->tables[i].ignitions[9][3]  = 12.4f;
  config->tables[i].ignitions[9][4]  = 15.3f;
  config->tables[i].ignitions[9][5]  = 19.2f;
  config->tables[i].ignitions[9][6]  = 23.0f;
  config->tables[i].ignitions[9][7]  = 27.9f;
  config->tables[i].ignitions[9][8]  = 30.0f;
  config->tables[i].ignitions[9][9]  = 33.2f;
  config->tables[i].ignitions[9][10] = 35.4f;
  config->tables[i].ignitions[9][11] = 36.5f;
  config->tables[i].ignitions[9][12] = 36.7f;
  config->tables[i].ignitions[9][13] = 37.0f;
  config->tables[i].ignitions[9][14] = 37.5f;
  config->tables[i].ignitions[9][15] = 37.7f;

  config->tables[i].ignitions[10][0]  = 7.1f;
  config->tables[i].ignitions[10][1]  = 8.2f;
  config->tables[i].ignitions[10][2]  = 9.2f;
  config->tables[i].ignitions[10][3]  = 10.6f;
  config->tables[i].ignitions[10][4]  = 12.9f;
  config->tables[i].ignitions[10][5]  = 16.1f;
  config->tables[i].ignitions[10][6]  = 20.8f;
  config->tables[i].ignitions[10][7]  = 24.0f;
  config->tables[i].ignitions[10][8]  = 28.3f;
  config->tables[i].ignitions[10][9]  = 30.8f;
  config->tables[i].ignitions[10][10] = 33.4f;
  config->tables[i].ignitions[10][11] = 34.2f;
  config->tables[i].ignitions[10][12] = 33.7f;
  config->tables[i].ignitions[10][13] = 33.9f;
  config->tables[i].ignitions[10][14] = 34.2f;
  config->tables[i].ignitions[10][15] = 34.3f;

  config->tables[i].ignitions[11][0]  = 6.3f;
  config->tables[i].ignitions[11][1]  = 7.2f;
  config->tables[i].ignitions[11][2]  = 8.1f;
  config->tables[i].ignitions[11][3]  = 9.3f;
  config->tables[i].ignitions[11][4]  = 11.1f;
  config->tables[i].ignitions[11][5]  = 14.0f;
  config->tables[i].ignitions[11][6]  = 18.0f;
  config->tables[i].ignitions[11][7]  = 23.3f;
  config->tables[i].ignitions[11][8]  = 26.9f;
  config->tables[i].ignitions[11][9]  = 29.1f;
  config->tables[i].ignitions[11][10] = 31.6f;
  config->tables[i].ignitions[11][11] = 32.0f;
  config->tables[i].ignitions[11][12] = 31.0f;
  config->tables[i].ignitions[11][13] = 31.3f;
  config->tables[i].ignitions[11][14] = 31.7f;
  config->tables[i].ignitions[11][15] = 31.8f;

  config->tables[i].ignitions[12][0]  = 6.1f;
  config->tables[i].ignitions[12][1]  = 6.7f;
  config->tables[i].ignitions[12][2]  = 7.3f;
  config->tables[i].ignitions[12][3]  = 8.4f;
  config->tables[i].ignitions[12][4]  = 9.9f;
  config->tables[i].ignitions[12][5]  = 12.5f;
  config->tables[i].ignitions[12][6]  = 17.7f;
  config->tables[i].ignitions[12][7]  = 22.0f;
  config->tables[i].ignitions[12][8]  = 25.9f;
  config->tables[i].ignitions[12][9]  = 27.9f;
  config->tables[i].ignitions[12][10] = 30.1f;
  config->tables[i].ignitions[12][11] = 30.3f;
  config->tables[i].ignitions[12][12] = 29.2f;
  config->tables[i].ignitions[12][13] = 29.4f;
  config->tables[i].ignitions[12][14] = 30.0f;
  config->tables[i].ignitions[12][15] = 30.2f;

  config->tables[i].ignitions[13][0]  = 6.0f;
  config->tables[i].ignitions[13][1]  = 6.5f;
  config->tables[i].ignitions[13][2]  = 7.1f;
  config->tables[i].ignitions[13][3]  = 7.9f;
  config->tables[i].ignitions[13][4]  = 9.1f;
  config->tables[i].ignitions[13][5]  = 11.3f;
  config->tables[i].ignitions[13][6]  = 15.8f;
  config->tables[i].ignitions[13][7]  = 20.3f;
  config->tables[i].ignitions[13][8]  = 24.4f;
  config->tables[i].ignitions[13][9]  = 26.6f;
  config->tables[i].ignitions[13][10] = 28.7f;
  config->tables[i].ignitions[13][11] = 28.8f;
  config->tables[i].ignitions[13][12] = 27.6f;
  config->tables[i].ignitions[13][13] = 27.9f;
  config->tables[i].ignitions[13][14] = 28.6f;
  config->tables[i].ignitions[13][15] = 28.8f;

  config->tables[i].ignitions[14][0]  = 6.0f;
  config->tables[i].ignitions[14][1]  = 6.5f;
  config->tables[i].ignitions[14][2]  = 7.0f;
  config->tables[i].ignitions[14][3]  = 7.7f;
  config->tables[i].ignitions[14][4]  = 8.7f;
  config->tables[i].ignitions[14][5]  = 10.5f;
  config->tables[i].ignitions[14][6]  = 14.1f;
  config->tables[i].ignitions[14][7]  = 18.2f;
  config->tables[i].ignitions[14][8]  = 22.1f;
  config->tables[i].ignitions[14][9]  = 24.5f;
  config->tables[i].ignitions[14][10] = 26.8f;
  config->tables[i].ignitions[14][11] = 27.0f;
  config->tables[i].ignitions[14][12] = 25.9f;
  config->tables[i].ignitions[14][13] = 26.2f;
  config->tables[i].ignitions[14][14] = 26.8f;
  config->tables[i].ignitions[14][15] = 27.0f;

  config->tables[i].ignitions[15][0]  = 6.0f;
  config->tables[i].ignitions[15][1]  = 6.5f;
  config->tables[i].ignitions[15][2]  = 7.0f;
  config->tables[i].ignitions[15][3]  = 7.6f;
  config->tables[i].ignitions[15][4]  = 8.5f;
  config->tables[i].ignitions[15][5]  = 9.9f;
  config->tables[i].ignitions[15][6]  = 12.3f;
  config->tables[i].ignitions[15][7]  = 15.8f;
  config->tables[i].ignitions[15][8]  = 19.5f;
  config->tables[i].ignitions[15][9]  = 22.2f;
  config->tables[i].ignitions[15][10] = 24.8f;
  config->tables[i].ignitions[15][11] = 25.1f;
  config->tables[i].ignitions[15][12] = 24.0f;
  config->tables[i].ignitions[15][13] = 24.3f;
  config->tables[i].ignitions[15][14] = 24.8f;
  config->tables[i].ignitions[15][15] = 25.0f;
}

HAL_StatusTypeDef config_default(sAcisConfig * config)
{
  HAL_StatusTypeDef status = HAL_OK;

  for(int i = 0; i < sizeof(sAcisConfig); i++)
    ((uint8_t*)config)[i] = 0;

  config->params.isCutoffEnabled = 1;
  config->params.isTemperatureEnabled = 1;
  config->params.isEconomEnabled = 1;
  config->params.isAutostartEnabled = 0;
  config->params.isIgnitionByHall = 0;
  config->params.isForceTable = 0;
  config->params.isHallLearningMode = 0;
  config->params.isSwitchByExternal = 1;
  config->params.isEconOutAsStrobe = 0;

  config->params.switchPos1Table = 0;
  config->params.switchPos0Table = 0;
  config->params.switchPos2Table = 0;
  config->params.forceTableNumber = 0;

  config->params.EconRpmThreshold = 2000;
  config->params.CutoffRPM = 5000;

  for(int i = 0; i < TABLE_SETUPS_MAX; i++)
    memset(&config->tables[i], 0, sizeof(sAcisIgnTable));

  config->tables_count = TABLE_SETUPS_MAX;
  for(int i = 0; i < 1; i++)
  {

    strcpy(config->tables[i].name, "Default 1");

    config->tables[i].valve_channel = ValvePetrol;
    config->tables[i].valve_timeout = 0;

    config->tables[i].initial_ignition = 0.0f;
    config->tables[i].octane_corrector = 0.0f;

    config->tables[i].idles_count = 20;
    config->tables[i].idle_rotates[0] = 417;
    config->tables[i].idle_rotates[1] = 455;
    config->tables[i].idle_rotates[2] = 476;
    config->tables[i].idle_rotates[3] = 500;
    config->tables[i].idle_rotates[4] = 525;
    config->tables[i].idle_rotates[5] = 556;
    config->tables[i].idle_rotates[6] = 588;
    config->tables[i].idle_rotates[7] = 625;
    config->tables[i].idle_rotates[8] = 667;
    config->tables[i].idle_rotates[9] = 714;
    config->tables[i].idle_rotates[10] = 769;
    config->tables[i].idle_rotates[11] = 833;
    config->tables[i].idle_rotates[12] = 909;
    config->tables[i].idle_rotates[13] = 1000;
    config->tables[i].idle_rotates[14] = 1111;
    config->tables[i].idle_rotates[15] = 1250;
    config->tables[i].idle_rotates[16] = 1429;
    config->tables[i].idle_rotates[17] = 1667;
    config->tables[i].idle_rotates[18] = 2000;
    config->tables[i].idle_rotates[19] = 2500;

    config->tables[i].idle_ignitions[0] = 10;
    config->tables[i].idle_ignitions[1] = 11;
    config->tables[i].idle_ignitions[2] = 12;
    config->tables[i].idle_ignitions[3] = 13;
    config->tables[i].idle_ignitions[4] = 14;
    config->tables[i].idle_ignitions[5] = 16;
    config->tables[i].idle_ignitions[6] = 18;
    config->tables[i].idle_ignitions[7] = 20;
    config->tables[i].idle_ignitions[8] = 20;
    config->tables[i].idle_ignitions[9] = 19;
    config->tables[i].idle_ignitions[10] = 15;
    config->tables[i].idle_ignitions[11] = 11;
    config->tables[i].idle_ignitions[12] = 9;
    config->tables[i].idle_ignitions[13] = 8;
    config->tables[i].idle_ignitions[14] = 8;
    config->tables[i].idle_ignitions[15] = 9;
    config->tables[i].idle_ignitions[16] = 10;
    config->tables[i].idle_ignitions[17] = 12;
    config->tables[i].idle_ignitions[18] = 14;
    config->tables[i].idle_ignitions[19] = 17;

    //setconfig_microplex(config, i);
    setconfig_standart16l(config,i);

    config->tables[i].temperatures_count = 11;
    config->tables[i].temperatures[0] = -20.0f;
    config->tables[i].temperatures[1] = -10.0f;
    config->tables[i].temperatures[2] = 0.0f;
    config->tables[i].temperatures[3] = 10.0f;
    config->tables[i].temperatures[4] = 20.0f;
    config->tables[i].temperatures[5] = 30.0f;
    config->tables[i].temperatures[6] = 40.0f;
    config->tables[i].temperatures[7] = 50.0f;
    config->tables[i].temperatures[8] = 60.0f;
    config->tables[i].temperatures[9] = 70.0f;
    config->tables[i].temperatures[10] = 80.0f;

    config->tables[i].temperature_ignitions[0] = 5.0f;
    config->tables[i].temperature_ignitions[1] = 4.0f;
    config->tables[i].temperature_ignitions[2] = 3.0f;
    config->tables[i].temperature_ignitions[3] = 2.0f;
    config->tables[i].temperature_ignitions[4] = 2.0f;
    config->tables[i].temperature_ignitions[5] = 1.0f;
    config->tables[i].temperature_ignitions[6] = 1.0f;
    config->tables[i].temperature_ignitions[7] = 0.0f;
    config->tables[i].temperature_ignitions[8] = 0.0f;
    config->tables[i].temperature_ignitions[9] = 0.0f;
    config->tables[i].temperature_ignitions[10] = 0.0f;

    config->tables[i].servo_acc[0] = 0.0f;
    config->tables[i].servo_acc[1] = 0.0f;
    config->tables[i].servo_acc[2] = 0.0f;
    config->tables[i].servo_acc[3] = 0.0f;
    config->tables[i].servo_acc[4] = 0.0f;
    config->tables[i].servo_acc[5] = 0.0f;
    config->tables[i].servo_acc[6] = 0.0f;
    config->tables[i].servo_acc[7] = 0.0f;
    config->tables[i].servo_acc[8] = 0.0f;
    config->tables[i].servo_acc[9] = 0.0f;

    config->tables[i].servo_choke[0] = 0.0f;
    config->tables[i].servo_choke[1] = 0.0f;
    config->tables[i].servo_choke[2] = 0.0f;
    config->tables[i].servo_choke[3] = 0.0f;
    config->tables[i].servo_choke[4] = 0.0f;
    config->tables[i].servo_choke[5] = 0.0f;
    config->tables[i].servo_choke[6] = 0.0f;
    config->tables[i].servo_choke[7] = 0.0f;
    config->tables[i].servo_choke[8] = 0.0f;
    config->tables[i].servo_choke[9] = 0.0f;
  }

  return status;
}


