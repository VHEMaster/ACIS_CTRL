/*
 * xControl.xc
 *
 *  Created on: Dec 4, 2020
 *      Author: denys.prokhorov
 */

#include <string.h>
#include "xProFIFO.h"
#include "xCommand.h"
#include "semphr.h"
#include "crc.h"
#include "acis.h"
#include "delay.h"

#define MAX_ACKS 8

uint8_t fifoAcisRxBuf[MAX_PACK_LEN];
sProFIFO fifoAcisRx;
uint8_t fifoAcisTxBuf[MAX_PACK_LEN];
sProFIFO fifoAcisTx;
uint8_t fifoPcRxBuf[MAX_PACK_LEN];
sProFIFO fifoPcRx;
uint8_t fifoPcTxBuf[MAX_PACK_LEN];
sProFIFO fifoPcTx;

static volatile uint16_t NeedAckPacket = 0;
static volatile uint16_t ReceivedAckPacket = 1;
static volatile uint16_t NeededAckPacketId = 0;
static volatile uint32_t LastNotAckedTime = 0;

static uint8_t msgFullPack[MAX_PACK_LEN + 8];
static uint8_t * msgBuf = &msgFullPack[8];

static inline int Msg_GetSrc(uint8_t xValue) { return (xValue & 7); }
static inline int Msg_GetDest(uint8_t xValue) { return ((xValue >> 3) & 7); }

static inline uint16_t calculatePacketId(void)
{
  static uint16_t counter = 0;
  uint16_t returnvalue;
  do
  {
    counter++;
    uint16_t localcounter = counter;
    uint32_t now = Delay_Tick;
    uint8_t crcdata[6] = {(localcounter >> 8) & 0xFF,localcounter & 0xFF, (now >> 24) & 0xFF, (now >> 16) & 0xFF, (now >> 8) & 0xFF, now & 0xFF } ;
    returnvalue = CRC16_Generate(crcdata, sizeof(crcdata));
  } while(returnvalue == 0);
  return returnvalue;

}

static inline void packager(sProFIFO* xFifo, uint8_t* xMsgPtr, uint16_t xMsgLen, eTransChannels xChaDest, uint16_t aPacketId) {

    if (xFifo && xMsgLen<MAX_PACK_LEN) {

        // Mark magic used for packages that does not aquire payload
        uint16_t aCrc15 = 0;
        uint16_t aTotLen = xMsgLen ? xMsgLen + 10 : 8;
        uint8_t aHeadByte = ( etrCTRL | ( xChaDest << 3 ) ) & HEADER_MASK_BITS;
        uint8_t * header = msgFullPack;


        header[0] = 0x55;
        header[1] = 0x55;
        header[2] = aHeadByte;
        header[3] = aTotLen & 0xFF;
        header[4] = (aTotLen >> 8) & 0xFF;
        header[5] = aPacketId & 0xFF;
        header[6] = (aPacketId >> 8) & 0xFF;
        header[7] = CRC8_Generate(header, 7);


        if (xMsgLen)
        {
          aCrc15 = CRC16_Generate(msgFullPack, xMsgLen + 8);
        }

        xSemaphoreTake(xFifo->info.globallock, portMAX_DELAY);
        protPushSequence(xFifo,header,8);
        if (xMsgLen) {
            protPushSequence(xFifo,xMsgPtr,xMsgLen);
            protPushSequence(xFifo,&aCrc15,2);
        }
        xSemaphoreGive(xFifo->info.globallock);
    }
}

static inline void acker(sProFIFO* xFifo, uint16_t aPacketId, eTransChannels xChaDest) {

    if (xFifo)
    {
        uint16_t aTotLen = 8;
        uint8_t aHeadByte = (( etrCTRL | ( xChaDest << 3 ) ) | HEADER_ACK_BIT) & HEADER_MASK_BITS;
        uint8_t header[8];

        header[0] = 0x55;
        header[1] = 0x55;
        header[2] = aHeadByte;
        header[3] = aTotLen & 0xFF;
        header[4] = (aTotLen >> 8) & 0xFF;
        header[5] = aPacketId & 0xFF;
        header[6] = (aPacketId >> 8) & 0xFF;
        header[7] = CRC8_Generate(header, 7);

        xSemaphoreTake(xFifo->info.globallock, portMAX_DELAY);
        protPushSequence(xFifo,header,8);
        xSemaphoreGive(xFifo->info.globallock);
    }
}


uint8_t xSender(eTransChannels xChaDest, uint8_t* xMsgPtr, uint32_t xMsgLen)
{
  sProFIFO* aDestFIFO = NULL;
  uint32_t now = Delay_Tick;

  if(NeedAckPacket)
  {
    if(ReceivedAckPacket)
    {
      NeedAckPacket = 0;
      NeededAckPacketId = 0;
      return 1;
    }
    else
    {
      if(DelayDiff(now, LastNotAckedTime) > 1000)
      {
        LastNotAckedTime = now;
        packager(aDestFIFO, xMsgPtr, xMsgLen, xChaDest, NeededAckPacketId);
      }
    }
  }
  else
  {

    switch (xChaDest)
    {
      case etrPC:
        aDestFIFO = &fifoPcTx;
        break;
      case etrACIS:
        aDestFIFO = &fifoAcisTx;
        break;
      default:
        break;
    }
    if (aDestFIFO) {
        NeededAckPacketId = calculatePacketId();;
        ReceivedAckPacket = 0;
        NeedAckPacket = 1;
        LastNotAckedTime = now;
        packager(aDestFIFO, xMsgPtr, xMsgLen, xChaDest, NeededAckPacketId);
    }
  }
  return 0;

}

static inline void parser(sProFIFO* xFifo, uint32_t xPacketId, uint32_t xDataLen, eTransChannels xChaSrc, eTransChannels xChaDest) {

	uint32_t aCount;
  uint8_t data;
  uint8_t sCount;
	sProFIFO* aDest;
	uint8_t header[8];

  if(xChaDest == etrACIS) aDest = &fifoAcisTx;
  if(xChaDest == etrPC) aDest = &fifoPcTx;

    switch (xChaDest) {

        case etrCTRL:
        {
            if (xDataLen)
            {
                for(int i = 0; i < 8; i++)
                  protPull(xFifo, &header[i]);

                for (aCount = 0; aCount < xDataLen - 10; aCount++)
                {
                  protPull(xFifo, &data);
                  msgBuf[aCount]=data;
                }
                protPull(xFifo, &data);
                protPull(xFifo, &data);

                msgBuf[aCount]=0;

                if(aDest) acker(aDest,xPacketId,xChaSrc);

                acis_parse_command(xChaSrc, msgBuf, aCount);

            // Signal package
            }
            else
            {
                for (aCount = 0; aCount < 8; aCount++)
                {
                  for(int i = 0; i < 8; i++)
                    protPull(xFifo, &header[i]);
                }

                if(NeedAckPacket && NeededAckPacketId != 0 && NeededAckPacketId == xPacketId && !ReceivedAckPacket)
                {
                  ReceivedAckPacket = 1;
                }

            }

            break;
        }

        case etrACIS:
        case etrPC:
        {
          sCount = (xDataLen > 10) ? xDataLen : 8;

          if(aDest)
          {
            xSemaphoreTake(xFifo->info.globallock, portMAX_DELAY);
            for (aCount = 0; aCount < sCount; aCount++)
            {
              protPull(xFifo, &data);
              protPush(aDest, &data);
            }
            xSemaphoreGive(xFifo->info.globallock);
            break;
          }
        }
        /* no break */

        default:
        {
          sCount = (xDataLen > 10) ? xDataLen : 8;
          for (aCount = 0; aCount < sCount; aCount++)
          {
            protPull(xFifo, &data);
          }
          break;
        }
    }
}

static inline uint8_t lookByte(sProFIFO* xFifo, uint32_t xOffset) { uint8_t aByte; protLook(xFifo,xOffset,&aByte); return aByte; }

static inline uint8_t countCRC8(sProFIFO* xFifo) {
    uint32_t i; uint8_t aCrc8 = 0;
    for (i=0; i<7; i++) { msgFullPack[i] = lookByte(xFifo,i); }
    aCrc8 = CRC8_Generate(msgFullPack, 7);
    return aCrc8;
}

static inline int32_t countCRC16(sProFIFO* xFifo, uint32_t xLen) {
    uint32_t i; int32_t aCrc16 = 0;
    for (i=0; i<xLen-2; i++) { msgFullPack[i] = lookByte(xFifo,i); }
    aCrc16 = CRC8_Generate(msgFullPack, xLen-2);
    return aCrc16;
}

void initFIFOs(void)
{
	// Init FIFOs
  protInit(&fifoAcisTx,fifoAcisTxBuf,1,sizeof(fifoAcisTxBuf));
  protInit(&fifoAcisRx,fifoAcisRxBuf,1,sizeof(fifoAcisRxBuf));
  protInit(&fifoPcTx,fifoPcTxBuf,1,sizeof(fifoPcTxBuf));
  protInit(&fifoPcRx,fifoPcRxBuf,1,sizeof(fifoPcRxBuf));
}

static void Getter(sProFIFO* xFifo, uint32_t * pDataReceiving, uint32_t * pDataLen, uint16_t * pPacketId)
{
  uint32_t dataSkip = 0;
  uint32_t dataLen = *pDataLen;
  uint32_t dataReceiving = *pDataReceiving;
  uint16_t packetId = *pPacketId;

  if(dataReceiving)
  {
    // Check if we got a data
    if (protGetSize(xFifo) >= dataLen)
    {
        if (countCRC16(xFifo,dataLen) == lookByte(xFifo,dataLen-2) + (lookByte(xFifo,dataLen-1) << 8))
        {
            // Got True package
            parser(xFifo,packetId,dataLen,Msg_GetSrc(lookByte(xFifo,2)),Msg_GetDest(lookByte(xFifo,2)));
        }
        else { dataSkip=1; } // Wrong CRC16, so skip 1 byte
        dataReceiving = 0;
        dataLen = 0;
    }
  }
  else
  {
    if (protGetSize(xFifo) > 7)
    {
      if(lookByte(xFifo,0) == 0x55 && lookByte(xFifo,1) == 0x55)
      {
        if (countCRC8(xFifo) == lookByte(xFifo,7))
        {
            if (lookByte(xFifo,0) < HEADER_MASK_BITS)
            {
                dataLen = lookByte(xFifo,3) + (lookByte(xFifo,4) << 8);
                packetId = lookByte(xFifo,5) + (lookByte(xFifo,6) << 8);
                if (packetId > 0 && dataLen < MAX_PACK_LEN)
                {
                    if (dataLen>10)
                    {
                      dataReceiving = 1;
                    }
                    else
                    {
                        // Got ShortPackage (Header Only)
                        parser(xFifo,packetId,0,Msg_GetSrc(lookByte(xFifo,2)),Msg_GetDest(lookByte(xFifo,2)));
                    }
                }
                else { dataSkip=1; } // Wrong data length or packet id, so skip 1 byte
            }
            else { dataSkip=1; } // Wrong marker bits, so skip 1 byte
        }
        else { dataSkip=1; } // Wrong CRC8, so skip 1 byte
      }
      else { dataSkip=1; } // Wrong sync bytes
    }
  }
  if (dataSkip)
  {
    protMoveRead(xFifo,dataSkip);
  }
  *pDataReceiving = dataReceiving;
  *pDataLen = dataLen;
  *pPacketId = packetId;
}

void xGetter(void * arg) {
  eTransChannels xChanIn = (eTransChannels)arg;
	sProFIFO* xFifo;
  if(xChanIn == etrPC) xFifo = &fifoPcRx;
  if(xChanIn == etrACIS) xFifo = &fifoAcisRx;

  uint32_t dataReceiving = 0;
  uint32_t dataLen = 0;
  uint16_t packetId = 0;
  for (;;) {
    Getter(xFifo, &dataReceiving, &dataLen, &packetId);
    TASK_SLEEP;
  }
}

