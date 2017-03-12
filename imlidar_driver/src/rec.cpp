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
#include "rec.h"

static u8 ParseBuffer[PARSE_LEN];

ResultTypeDef Unpacking(PackageDataStruct *package)
{
    u16 i = 0;

    if ((package->DataInBuff == NULL) && (package->DataInLen < MIN_PRO_NUM))
    {
        return PACK_FAIL;
    }

    if (package->DataInLen >= MIN_PRO_NUM)
    {
        if ((*(package->DataInBuff + package->DataInLen - 1) == P_TAIL) && (*(package->DataInBuff + package->DataInLen - 2) == P_TAIL))
        {
            i = MIN_PRO_NUM - 2;
            while (i++)
            {
                if (*(package->DataInBuff + package->DataInLen - i) == P_HEADER)
                {
                    if (*(package->DataInBuff + package->DataInLen - (i + 1)) == P_HEADER)
                    {
                        u8 *pbuff = package->DataInBuff + package->DataInLen - (i - 1);
                        u16 len = i - 3;
                        u16 j = 0;
                        u8 checksum = 0;
                        u16 data_out_count = 0;

						if(len > sizeof(ParseBuffer))
							return PACK_FAIL;
                        for (j = 0 ; j < len; j++)
                        {
                            if (*(pbuff + j) == P_CTRL)
                            {
                                j++;
                            }
                            ParseBuffer[data_out_count++] = *(pbuff + j);
                            if (data_out_count == PARSE_LEN)
                            {
                                package->DataID = PACK_NULL;
                                return PACK_FAIL;
                            }
                        }

                        for(j = 0 ; j < data_out_count-1;j++)
                        {
                            checksum += ParseBuffer[j];
                        }

                        if (checksum == ParseBuffer[data_out_count-1])
                        {
                            SdkProtocolHeaderTypeDef *sdk = (SdkProtocolHeaderTypeDef *)ParseBuffer;
                            *(package->DataOutLen) = data_out_count - 1 - sizeof(SdkProtocolHeaderTypeDef);
                            package->DataOutBuff = ParseBuffer + sizeof(SdkProtocolHeaderTypeDef);	
							if(sdk->DeviceAddr == LIDAR_ADDRESS)
								package->DataID = (PackageIDTypeDef)sdk->FunctionCode;
							else
								package->DataID = PACK_NULL;
                            return PACK_OK;
                        }
                        else
                        {
                            package->DataID = PACK_NULL;
                            *(package->DataOutLen) = 0;
                            return PACK_FAIL;
                        }
                    }
                }
                if (i == package->DataInLen)
                {
                    package->DataID = PACK_NULL;
                    return PACK_FAIL;
                }
            }
        }
    }
    return PACK_FAIL;
}

/************************* END OF FILE *******************************/