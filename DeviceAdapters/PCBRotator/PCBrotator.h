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

#ifndef _PCBROTATOR_H_
#define _PCBROTATOR_H_

#define WIN32_LEAN_AND_MEAN

#include "DeviceBase.h"
#include <string>

class PCBrotator: public CStageBase<PCBrotator>
{
public:
	PCBrotator();
	~PCBrotator();

	// Device API	
	int Initialize();
	int Shutdown();
	void GetName(char* name) const;
	bool Busy();

	// Stage API	
	int SetPositionUm(double pos);
	int GetPositionUm(double& pos);	
	double GetStepSizeUm();
	int SetPositionSteps(long steps);
	int GetPositionSteps(long& steps);	
	int SetOrigin();	
	int GetLimits(double& lower, double& upper);
	int GetStepLimits(long& lower, long& upper);	
	int IsStageSequenceable(bool& isSequenceable) const;
	bool IsContinuousFocusDrive() const;
	
	// Action interface
	int OnPort(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnDelay(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnDebugLevel(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnCommand(MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnGoToIndex(MM::PropertyBase* pProp, MM::ActionType eAct);
	

private:
	// internal methods
	int SendCommand(const std::string command) const;		

private:
	bool initialized_;
	std::string port_;
	long index_;
	long maxIndex_;	
	std::string command_;
	std::string commandTerminator_; // terminator to append to to sent string
	std::string response_;
	std::string responseTerminator_; // response terminator	
	long debug_;
	long delay_;
};

#endif
