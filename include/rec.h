/**
  * @file        rec.h
  * @author      陈维
  * @version     V01
  * @date        2016.09.21
  * @brief       解包
  * @note        
  *
  * @attention   COYPRIGHT INMOTION ROBOT
  **/
#ifndef _REC_H_
#define _REC_H_
#include "pro.h"

ResultTypeDef Unpacking(PackageDataStruct *package);
#pragma pack(push)
#pragma pack(1)

typedef struct
{
    u16 Distance;
    u8 Confidence;
}LidarPointStructDef;
#pragma pack(pop)

typedef struct
{
   u16 Temperature;
   u16 CurrSpeed;
   u16 GivenSpeed;
   LidarPointStructDef Data[360];
}LidarDataStructDef;


#endif

/************************ (C) COPYRIGHT INMOTION ROBOT *****END OF FILE****/

