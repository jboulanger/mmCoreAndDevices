///////////////////////////////////////////////////////////////////////////////
// FILE:       PCBRotator.h
// PROJECT:    Micro-Manager
// SUBSYSTEM:  DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   PCB rotator 
//                                                                                     
// AUTHOR:        J. Boulanger (jeromeb@mrc-lmb.cam.ac.uk) and B. Sutcliffe (sutcliff@mrc-lmb.cam.ac.uk)
//                
//
// COPYRIGHT:     MRC-LMB
//
// LICENSE:       This file is distributed under the BSD license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
//

#include "PCBrotator.h"
#include "../SerialManager/SerialManager.h"
#include <boost/lexical_cast.hpp> 
//#include "boost/date_time/posix_time/posix_time.hpp" //include all types plus i/o for timestamps

//#include <ctime> //ben added to get timestamps in logs

const char* g_PCBRotatorName = "PCBrotator";
const char* g_PCBRotatorDescription = "PCB rotator driver adapter";

//////////////////////////////////////////////////////////////////////////////////
//                        Exported MMDevice API
//////////////////////////////////////////////////////////////////////////////////

MODULE_API void 
InitializeModuleData() 
{
	RegisterDevice(g_PCBRotatorName, MM::StageDevice, g_PCBRotatorDescription);
}


MODULE_API MM::Device* 
CreateDevice(const char* deviceName)                  
{
	if (deviceName == NULL) 
	{
		return 0;
	}

	if (strcmp(deviceName, g_PCBRotatorName) == 0)
	{
		return new PCBrotator();
	}
	else
	{	
		return 0;
	}
}


MODULE_API void 
DeleteDevice(MM::Device* pDevice)
{
	delete pDevice;
}
 
//////////////////////////////////////////////////////////////////////////////////
//                           Constructor
//////////////////////////////////////////////////////////////////////////////////

PCBrotator::PCBrotator(void):
	initialized_(false),
	port_("Undefined"),
	index_(0),
	maxIndex_(2880),
	command_("bi"),
	commandTerminator_("\r"), // terminator to append to to sent string
	response_("Undefined"),
	responseTerminator_("\r"), // response terminator	
	debug_(1),
	delay_(2000)
{
 	 		
	InitializeDefaultErrorMessages();	
	
	CreateProperty(MM::g_Keyword_Name, g_PCBRotatorName, MM::String, true);
	CreateProperty(MM::g_Keyword_Description, g_PCBRotatorDescription, MM::String, true);	

	CPropertyAction* pAct1;		
	pAct1 = new CPropertyAction (this, &PCBrotator::OnPort);
	CreateProperty(MM::g_Keyword_Port, "COM4", MM::String, false, pAct1, true);

	CPropertyAction* pAct2;		
	pAct2 = new CPropertyAction (this, &PCBrotator::OnDelay);
	CreateProperty("Delay", "2000", MM::Integer, false, pAct2, true);

}

PCBrotator::~PCBrotator(void)
{	
	Shutdown();
}

//////////////////////////////////////////////////////////////////////////////////
//                              Device API
//////////////////////////////////////////////////////////////////////////////////
bool 
PCBrotator::Busy()
{
	return false;
}

int 
PCBrotator::Initialize()
{
	CPropertyAction* pAct1 = new CPropertyAction (this, &PCBrotator::OnCommand);
	CreateProperty("Command", "bi", MM::String, false, pAct1); 
		
	CPropertyAction* pAct2 = new CPropertyAction (this, &PCBrotator::OnGoToIndex);
	CreateProperty("Index", "0", MM::Integer, false, pAct2);
	SetPropertyLimits("Index", 0, maxIndex_);
	
	SetOrigin();
	initialized_ = true;	
	return DEVICE_OK;
}

int 
PCBrotator::Shutdown()
{
	return DEVICE_OK;
}

void 
PCBrotator::GetName(char* name) const 
{
	 CDeviceUtils::CopyLimitedString(name, "PCB Rotator");
}

int 
PCBrotator::SetPositionUm(double pos)
{	
	long steps = (long) ((double)(pos / 360.0) * (double) maxIndex_);

	char buf[1024];
	snprintf(buf,1024,"PCBrotator::SetPositionUm : pos=%f, steps=%ld, maxIndex_=%ld\n", pos, steps, maxIndex_);		
	LogMessage(buf);
	SetPositionSteps(steps);	
	return DEVICE_OK;
}

int 
PCBrotator::GetPositionUm(double& pos) 
{	
	pos = (double) index_  / (double) maxIndex_ *  360.0;
	return DEVICE_OK;
}

double 
PCBrotator::GetStepSizeUm()
{
	return 360.0 / (double) maxIndex_;
}

int PCBrotator::SetPositionSteps(long steps)
{	
	int position = steps % maxIndex_;
	int ret = SendCommand("g"+ boost::lexical_cast<std::string>(position));
	if (ret == DEVICE_OK) {
		index_ = steps;
	}	
#ifdef WIN32
	Sleep(delay_);
#else
	sleep(delay_);
#endif
	return DEVICE_OK;
}

int PCBrotator::GetPositionSteps(long& steps)
{	
	LogMessage("PCBrotator::GetPositionSteps\n"+boost::lexical_cast<std::string>(steps)+"\n");
	steps = index_;
	return DEVICE_OK;
}

int PCBrotator::SetOrigin()
{	
	LogMessage("PCBrotator::SetOrigin\n");
	SendCommand("z");
	index_ = 0;
	return DEVICE_OK;
}

int PCBrotator::GetLimits(double& lower, double& upper)
{
	lower = 0;
	upper = 360;
	return DEVICE_OK;
}

int PCBrotator::GetStepLimits(long& lower, long& upper){
	lower = 0;
	upper = maxIndex_;
	return DEVICE_OK;
}

int PCBrotator::IsStageSequenceable(bool& isSequenceable) const{
	isSequenceable = false;
	return DEVICE_OK;
}

bool PCBrotator::IsContinuousFocusDrive() const{
	return false;
}

//////////////////////////////////////////////////////////////////////////////////
//                         Action interface
//////////////////////////////////////////////////////////////////////////////////
int PCBrotator::OnPort(MM::PropertyBase* pProp, MM::ActionType eAct) 
{
	if (eAct == MM::BeforeGet)
	{
		pProp->Set(port_.c_str());
	}
	else if (eAct == MM::AfterSet)
	{
		if (initialized_)
		{			
			pProp->Set(port_.c_str());
			return ERR_PORT_CHANGE_FORBIDDEN;
		}

		pProp->Get(port_);
	}
	return DEVICE_OK;
}

int PCBrotator::OnCommand(MM::PropertyBase* pProp, MM::ActionType eAct) 
{
	int ret = DEVICE_OK;
	if (eAct == MM::BeforeGet)
	{		
		pProp->Set(command_.c_str());
	}
	else if (eAct == MM::AfterSet)
	{
		pProp->Get(command_);     
		ret = SendCommand(command_);      
	}
	return ret;	
}

int PCBrotator::OnGoToIndex(MM::PropertyBase* pProp, MM::ActionType eAct) 
{
	int ret = DEVICE_OK;
	if (eAct == MM::BeforeGet)
	{		
		pProp->Set(index_);
	}
	else if (eAct == MM::AfterSet)
	{
		long indexTmp;
		pProp->Get(indexTmp);
		LogMessage("PCBrotator::OnGoToIndex "+ boost::lexical_cast<std::string>(indexTmp)+"\n");
		SetPositionSteps(indexTmp);		
	}
	return ret;	
}

int PCBrotator::OnDelay(MM::PropertyBase* pProp, MM::ActionType eAct) 
{
	int ret = DEVICE_OK;
	if (eAct == MM::BeforeGet)
	{		
		pProp->Set(delay_);
	}
	else if (eAct == MM::AfterSet)
	{		
		pProp->Get(delay_);
		LogMessage("PCBrotator::OnDelay "+ boost::lexical_cast<std::string>(delay_)+"\n");
	}
	return ret;	
}

int PCBrotator::OnDebugLevel(MM::PropertyBase* pProp, MM::ActionType eAct) 
{
	int ret = DEVICE_OK;
	if (eAct == MM::BeforeGet)
	{		
		pProp->Set(debug_);
	}
	else if (eAct == MM::AfterSet)
	{		
		pProp->Get(debug_);
		LogMessage("PCBrotator::OnDebugLevel "+ boost::lexical_cast<std::string>(debug_)+"\n");
	}
	return ret;	
}

///////////////////////////////////////////////////////////////////////////////
//                                 Internal 
///////////////////////////////////////////////////////////////////////////////
int PCBrotator::SendCommand(const std::string command) const 
{
	LogMessage("PCBRotator::SendCommand : [" + command + "] on port "+port_+"\n");			
	int	ret = GetCoreCallback()->SetSerialCommand(this, port_.c_str(), command.c_str(), commandTerminator_.c_str());
	LogMessage("PCBRotator::SendCommand ok\n");		
	return ret;
}
