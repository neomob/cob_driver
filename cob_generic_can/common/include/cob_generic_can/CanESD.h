/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
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
 *   * Neither the name of the Neobotix nor the names of its
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

#ifndef CANESD_INCLUDEDEF_H
#define CANESD_INCLUDEDEF_H

//-----------------------------------------------

#include <iostream>
#include <cob_generic_can/ntcan.h>
#include <cob_generic_can/CanItf.h>
#include <cob_generic_can/stdDef.h>
#include <cob_utilities/IniFile.h>

//-----------------------------------------------
class CanESD : public CanItf
{

private:
	char m_DeviceNr;
	char m_BaudRate;
	NTCAN_HANDLE m_Handle;
	int m_LastID;
	int m_iCanNet;
	std::string getErrorMessage(NTCAN_RESULT res);
	int getBaudRate(int iIniBaudRate);

public:
	CanESD(int iBaudRate, int iNrNet);
	~CanESD();
	void init(int t);
	bool transmitMsg(CanMsg &CMsg);
	bool receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry);
	bool receiveMsg(CanMsg* pCMsg);
	bool emMessageError();
};
//-----------------------------------------------
#endif

