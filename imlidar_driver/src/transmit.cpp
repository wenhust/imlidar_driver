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
#include "transmit.h"  
#include "pro.h"

ResultTypeDef Package(PackageDataStruct package)
{
	u32 j = 0;
	u32 i = 0;
	SdkProtocolHeaderTypeDef sdk_header;
	u8 *psdk = (u8 *)&sdk_header;
	u8 checksum = 0;
	
	if((package.DataInBuff == NULL) || (package.DataOutBuff == NULL) || (package.DataOutLen == NULL))
		return PACK_FAIL;
	
	sdk_header.DeviceAddr = LIDAR_ADDRESS;
	sdk_header.FunctionCode = package.DataID;
	sdk_header.StartAddr = 0;
	sdk_header.Len = package.DataInLen;

	*(package.DataOutBuff+i ++) = P_HEADER;
	*(package.DataOutBuff+i ++) = P_HEADER;

	for(j = 0 ; j<sizeof(SdkProtocolHeaderTypeDef);j++)
	{
		if((*(psdk+j) == P_CTRL) || (*(psdk+j) == P_HEADER) || (*(psdk+j) == P_TAIL))
		{
			*(package.DataOutBuff+i ++) = P_CTRL;
		}
		*(package.DataOutBuff+i ++) = *(psdk+j);
		checksum += *(psdk+j);
	}
	
	for(j = 0 ; j<package.DataInLen; j++)
	{
		if((*(package.DataInBuff+j) == P_CTRL) || (*(package.DataInBuff+j) == P_HEADER) || (*(package.DataInBuff+j) == P_TAIL))
		{
			*(package.DataOutBuff+i++) = P_CTRL;
		}
		checksum += *(package.DataInBuff+j);
		*(package.DataOutBuff+i++) = *(package.DataInBuff+j);
	}
	
	if((checksum == P_CTRL) || (checksum == P_HEADER) || (checksum == P_TAIL))
	{
		*(package.DataOutBuff+i++) = P_CTRL;
	}
	*(package.DataOutBuff+i++) = checksum;
	
	*(package.DataOutBuff+i++) = P_TAIL;
	*(package.DataOutBuff+i++) = P_TAIL;
	
	*package.DataOutLen = i;
	
	return PACK_OK;
}

/************************* END OF FILE *******************************/