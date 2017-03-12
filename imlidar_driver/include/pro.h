/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Inmotion Robot, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Inmotion Robot nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#ifndef _PRO_H_
#define _PRO_H_

#define u8               unsigned char
#define u16              unsigned short
#define u32              unsigned int
#define s16              short
#define LIDAR_ADDRESS    0x10
#define PARSE_LEN        8192
#define MIN_PRO_NUM      14

typedef enum
{
    PACK_FAIL,
    PACK_OK
} ResultTypeDef;

#define NULL    __null

typedef struct
{
    u8  DeviceAddr;   
    u8  FunctionCode; 
    u16 StartAddr;    
    u32 Len;
} SdkProtocolHeaderTypeDef;

#define P_HEADER     0xAA
#define P_TAIL       0x55
#define P_CTRL       0xA5
#define P_FAIL       0
#define P_SUCCESS    1

typedef enum
{
    PACK_LIDAR_DATA = 0x00,
    PACK_SET_SPEED = 0x04,
    PACK_START_ROTATE = 0x09,
    PACK_NULL = 0xff
} PackageIDTypeDef;

typedef struct
{
    PackageIDTypeDef DataID;
    u8               *DataInBuff;
    u32              DataInLen;
    u8               *DataOutBuff;
    u32              *DataOutLen;
} PackageDataStruct;


#endif

/************************* END OF FILE *******************************/