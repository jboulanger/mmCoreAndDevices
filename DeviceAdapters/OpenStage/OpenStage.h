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


//
// Define 
// OpenStageBase: encapsulate all functions and point to a hub
// OpenStageHub: managing the serial communications 
// OpenStageXY/OpenStageZ: expose the stages to mm

#pragma once
#ifndef _OPENSTAGE_H_
#define _OPENSTAGE_H_

#define WIN32_LEAN_AND_MEAN

#include "DeviceBase.h"
#include "MMDeviceConstants.h"
#include "MMDevice.h"
#include <string>
#include <boost/thread/thread.hpp>
/**
	Open Stage Hub class
	https://storage.googleapis.com/plos-corpus-prod/10.1371/journal.pone.0088977/1/pone.0088977.pdf?X-Goog-Algorithm=GOOG4-RSA-SHA256&X-Goog-Credential=wombat-sa%40plos-prod.iam.gserviceaccount.com%2F20210211%2Fauto%2Fstorage%2Fgoog4_request&X-Goog-Date=20210211T112358Z&X-Goog-Expires=3600&X-Goog-SignedHeaders=host&X-Goog-Signature=7d355ffcc21b12e0919b36f46bc3291465c83a930142c239335073753b1a4a95c42e1084c5e82572e1317a896a5430edfacffb8cf2cddaf4823fa13924c91247d7f9757e80b0f1e990bc061a2eb98462e38fa5504a4f00f88f296ea65d095fc48eb09f5b7def9aae924375a7d5889bd69d97abf12af6dd8c2b8a03d9ccdf5a9702332915dc57a335cf6e707f65376c0e2f776b9ce87e4e7434fad2e6ea66c0efeb513d7505793c565f0fa683bad5929450927cd64d13a7f5bc003590ab34594138b08a17431529c23325765d3d56c338011cc384d008804785dab3489d2f3ac6101664c3c6ef55bca4389ff3cd8109012c85213052310c71ed1f57cc87b67fba

	Stepping, there is 5 micro-stepping mode 1,1/2,1/4,1/8,1/16
	The step is 0.9 degree 
	The gear ratio is x nm / degree
	*/
class OpenStageHub: public HubBase<OpenStageHub> 
{
public:
	OpenStageHub();
	~OpenStageHub();

	// Device adaptor
	int Initialize();
	int Shutdown();
   void GetName(char* name) const;
   bool Busy();

   // Hub
   bool SupportsDeviceDetection(void);
   MM::DeviceDetectionStatus DetectDevice(void);
   int DetectInstalledDevices();
   bool IsPortAvailable() {return initialized_;}

	// internal methods
	int tx(const std::string &command, const std::string &terminator);	
	int rx(std::string &answer, const std::string &terminator);
	int get(const std::string & keyword, double &x, double &y, double &z);
	int set(const std::string & keyword, const long x, const long y, const long z);
	int get(const std::string & keyword, long &x, long &y, long &z);
	int goToAbsolutePositionUm(const double x, const double y, const double z);
	int goToRelativePositionUm(const double x, const double y, const double z);
	// read position 
	int updatePosition();
	// read and return position
	int getPositionUm(double &x, double &y, double &z);
	int zero();
	int beep();
	int setDelay(const int delay_ms);
	std::string info();


	int setStepSizeIdx(int options); 
	int getStepSizeValue(double & s);
	double getStepTravelUm();

	int setVelocity(const long x, const long y, const long z);
	int getVelocity(long &x, long &y, long &z);
	int setAcceleration(const long x, const long y, const long z);
	int getAcceleration(long &x, long &y, long &z);
	
	bool isPositionValid(const double x, const double y, const double z);
	
	// traceback error messages
	void logMessage(const std::string & msg) const;
	int traceback(const std::string & id, const std::string & msg, const int error) const;

	// action interface
	int OnPort      (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnCommand   (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnStepSize  (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnDelay     (MM::PropertyBase* pProp, MM::ActionType eAct);

public:
	// device and core information
	MM::Device *device_;
    MM::Core *core_;

	// serial communication
	bool initialized_;
	std::string port_;
	std::string command_;
	boost::posix_time::milliseconds delay_ms_;

	// position
	double x_,y_,z_; // current position
	double x0_,y0_,z0_; //origin
	double xmin_,xmax_,ymin_,ymax_,zmin_,zmax_; // range

	int step_x_, step_y_, step_z_;
	double step_angle_; // in degree
	double gear_ratio_; // in micron per rev
	int microstepping_idx_; // 0 to 4
	double microstepping_values_[5]; 
};

class OpenStageXY: public CXYStageBase<OpenStageXY>
{
public:
	OpenStageXY();
	~OpenStageXY();

	// Device API
   int Initialize();
   int Shutdown();
   void GetName(char* pszName) const;
   bool Busy();

	// stage API
	int SetPositionUm(double x, double y);
    int SetRelativePositionUm(double dx, double dy);
    int SetAdapterOriginUm(double x, double y);
    int GetPositionUm(double& x, double& y);
    int GetLimitsUm(double& xMin, double& xMax, double& yMin, double& yMax);
    int Move(double vx, double vy);
    int SetPositionSteps(long x, long y);
    int GetPositionSteps(long& x, long& y);
    int SetRelativePositionSteps(long x, long y);
    int Home();
    int Stop();
    int SetOrigin();
    int SetXOrigin();
    int SetYOrigin();
    int GetStepLimits(long& xMin, long& xMax, long& yMin, long& yMax);
    double GetStepSizeXUm();
    double GetStepSizeYUm();
    int IsXYStageSequenceable(bool& isSequenceable) const;
    int GetXYStageSequenceMaxLength(long& nrEvents) const;
    int StartXYStageSequence();
    int StopXYStageSequence();
    int ClearXYStageSequence();
    int AddToXYStageSequence(double positionX, double positionY);
	int SendXYStageSequence();

	void logMessage(const std::string & msg) const;
	int traceback(const std::string & id, const std::string & msg, const int error) const;

	// action interface
	//int OnPort      (MM::PropertyBase* pProp, MM::ActionType eAct);
	//int OnCommand   (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnGearRatio (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnSetRelativeXUm    (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnSetRelativeYUm    (MM::PropertyBase* pProp, MM::ActionType eAct);
	

public:
	// device and core information
	MM::Device *device_;
    MM::Core *core_;

};

class OpenStageZ: public CStageBase<OpenStageZ>
{
public:
	OpenStageZ();
	~OpenStageZ();

	// Device API	
	int Initialize();
	int Shutdown();
	void GetName(char* name) const;
	bool Busy();

	// Stage API	
	int SetPositionUm(double pos);
    int SetRelativePositionUm(double d);
    int Move(double velocity);
    int Stop();
    int Home();
    int SetAdapterOriginUm(double d);
    int GetPositionUm(double& pos);
    int SetPositionSteps(long steps);
    int GetPositionSteps(long& steps);
    int SetOrigin();
    int GetLimits(double& lower, double& upper);
    //int GetFocusDirection(FocusDirection & direction);
    int IsStageSequenceable(bool& isSequenceable) const;
    int IsStageLinearSequenceable(bool& isSequenceable) const;
    bool IsContinuousFocusDrive() const;
    int GetStageSequenceMaxLength(long& nrEvents) const;
    int StartStageSequence();
    int StopStageSequence();
    int ClearStageSequence();
    int AddToStageSequence(double position); 
    int SendStageSequence();
	int SetStageLinearSequence(double dZ_um, long nSlices);

	void logMessage(const std::string & msg) const;
	int traceback(const std::string & id, const std::string & msg, const int error) const;

	// Action interface
	int OnSetRelativeZUm    (MM::PropertyBase* pProp, MM::ActionType eAct);

public:
	// device and core information
	MM::Device *device_;
    MM::Core *core_;
};


#endif 