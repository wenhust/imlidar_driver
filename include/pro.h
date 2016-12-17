/**
 * @file        pro.h
 * @author      陈维
 * @version     V01
 * @date        2016.09.21
 * @brief       协议定义
 * @note
 *
 * @attention   COYPRIGHT INMOTION ROBOT
 **/

#ifndef _PRO_H_
#define _PRO_H_

#define u8               unsigned char
#define u16              unsigned short
#define u32              unsigned int
#define s16              short
#define LIDAR_ADDRESS    0x10
#define PARSE_LEN           8192    //>1036
#define MIN_PRO_NUM      14

typedef enum
{
    PACK_FAIL,
    PACK_OK
} ResultTypeDef;

#define NULL    0


typedef struct
{
    u8  DeviceAddr;   
    u8  FunctionCode; 
    u16 StartAddr;    
    u32 Len;
} SdkProtocolHeaderTypeDef;


//数据包头尾、控制字
#define P_HEADER     0xAA
#define P_TAIL       0x55
#define P_CTRL       0xA5
#define P_FAIL       0
#define P_SUCCESS    1


/**
 * @brief  数据包ID
 */
typedef enum
{
    PACK_LIDAR_DATA = 0x00,
    PACK_SET_SPEED = 0x04,          /*!< 设置Lidar速度 */
    PACK_START_ROTATE = 0x09,       /*!< 开始旋转 */
    PACK_NULL = 0xff         /*!< 复位值，表明当前没有数据包 */
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



/************************ (C) COPYRIGHT INMOTION ROBOT *****END OF FILE****/
