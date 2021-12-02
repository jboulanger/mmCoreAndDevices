#include "OpenStage.h"
#include "MMDeviceConstants.h"
#include "MMDevice.h"
#include "../SerialManager/SerialManager.h"
#include <sstream>
#include "math.h"
#include <boost/thread/thread.hpp>

#define ERR_NO_PORT_SET 108

const char* g_OpenStageHub = "OpenStageHub";
const char* g_OpenStageHubDescription = "OpenStage Hub device adaptor";

const char* g_OpenStageXY = "OpenStageXY";
const char* g_OpenStageXYDescription = "OpenStageXY device adapter";

const char* g_OpenStageZ = "OpenStageZ";
const char* g_OpenStageZDescription = "OpenStageZ device adapter";

///////////////////////////////////////////////////////////////////////////////
//                       Exported MMDevice API
///////////////////////////////////////////////////////////////////////////////
MODULE_API void InitializeModuleData()
{
	RegisterDevice(g_OpenStageHub, MM::HubDevice, "OpenStage Hub");
	RegisterDevice(g_OpenStageXY, MM::XYStageDevice, "OpenStage XY Stage");
	RegisterDevice(g_OpenStageZ,  MM::StageDevice,   "OpenStage Z Axis");
}                                                                            

MODULE_API MM::Device* CreateDevice(const char* deviceName)                  
{
   if (deviceName == 0) return 0;
   if (strcmp(deviceName, g_OpenStageHub) == 0) return new OpenStageHub();        
   if (strcmp(deviceName, g_OpenStageXY) == 0) return new OpenStageXY();                                     
   if (strcmp(deviceName, g_OpenStageZ)  == 0) return new OpenStageZ();
   return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
   delete pDevice;
}

///////////////////////////////////////////////////////////////////////////////
//                       OpenStageHub
///////////////////////////////////////////////////////////////////////////////

OpenStageHub::OpenStageHub():
	initialized_(false),
	port_("Undefined"),
	delay_ms_(10),
	x_(0),y_(0),z_(0), // current position
	x0_(0),y0_(0),z0_(0), //origin
	xmin_(-10000),xmax_(10000),
	ymin_(-10000),ymax_(10000),
	zmin_(-10000),zmax_(10000),// range
	step_x_(0), step_y_(0), step_z_(0),
	step_angle_(0.9),
	gear_ratio_(500),
	microstepping_idx_(4)
{
	for (int i = 0; i < 5; i++) {
		microstepping_values_[i] = (double)i / std::pow(2.0,(double)i);
	}

	InitializeDefaultErrorMessages();
	CreateProperty(MM::g_Keyword_Name, g_OpenStageHub, MM::String, true);
	CreateProperty(MM::g_Keyword_Description, g_OpenStageHubDescription, MM::String, true);

	CPropertyAction* pAct = new CPropertyAction (this, &OpenStageHub::OnPort);
	CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);

	CPropertyAction* pAct2 = new CPropertyAction(this, &OpenStageHub::OnDelay);
	CreateProperty("Delay", "40", MM::Integer, false, pAct2, true);
}

OpenStageHub::~OpenStageHub()
{
}

int OpenStageHub::Initialize()
{

	// init device and core
	device_ = this;
	core_ = GetCoreCallback();

	// read stage information
	std::string answer;
	rx(answer, "\r\n");
	logMessage(answer);
	rx(answer, "\r\n");
	logMessage(answer);

	double s;
	getStepSizeValue(s);
	char stepsizestr[64];
	sprintf(stepsizestr,"%d",microstepping_idx_);

	updatePosition();

	// add properties
	CPropertyAction* pAct1 = new CPropertyAction (this, &OpenStageHub::OnCommand);
	CreateProperty("Command", "b", MM::String, false, pAct1); 

	CPropertyAction* pAct2 = new CPropertyAction (this, &OpenStageHub::OnStepSize);
	CreateProperty("Step size index", stepsizestr, MM::Float, false, pAct2); 

	initialized_ = true;
	return DEVICE_OK;
}

int OpenStageHub::Shutdown()
{
	initialized_ = false;
	return DEVICE_OK;
}

void OpenStageHub::GetName(char* name) const 
{
	CDeviceUtils::CopyLimitedString(name, g_OpenStageHub);
}

bool OpenStageHub::Busy()
{
	return false;
}

bool OpenStageHub::SupportsDeviceDetection(void)
{
	return true;
}

MM::DeviceDetectionStatus OpenStageHub::DetectDevice(void)
{
	if (initialized_) 
	{
		return MM::CanCommunicate;
	}

	MM::DeviceDetectionStatus result = MM::Misconfigured;
	result = MM::CanCommunicate;
	// TODO: some test here
	return result;
}

int OpenStageHub::DetectInstalledDevices()
{
	if (MM::CanCommunicate == DetectDevice()) 
	{
		MM::Device* pDev;
		pDev = ::CreateDevice(g_OpenStageXY);
         if (pDev) 
         {
            AddInstalledDevice(pDev);
         }
		 pDev= ::CreateDevice(g_OpenStageZ);
         if (pDev) 
         {
            AddInstalledDevice(pDev);
		 }
   }
   return DEVICE_OK;
}

int OpenStageHub::tx(const std::string &command, const std::string &terminator)
{		
	int ret = SendSerialCommand(port_.c_str(), command.c_str(), terminator.c_str());
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__, "error while sending message", ret);
	}
	return DEVICE_OK;
}

int OpenStageHub::rx(std::string &answer, const std::string &terminator)  
{		
	int ret = GetSerialAnswer(port_.c_str(), terminator.c_str(), answer);
	PurgeComPort(port_.c_str());
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__, "error while receiving message", ret);
	}
	return DEVICE_OK;
}

void parsestr(const std::string & str, double &x, double &y, double &z) {
	char buf[4096];
	sprintf(buf,"%s",str.c_str());
	char * pch;
	pch = strtok (buf,",");
	x = atof(pch);
	printf("x=%f\n",x);
  
	pch = strtok (NULL,",");
	y = atof(pch);
	printf("y=%f\n",y);
  
	pch = strtok (NULL,",");
	z = atof(pch);
	printf("x=%f\n",z);
}

int OpenStageHub::get(const std::string & keyword, double &x, double &y, double &z) {
	logMessage("get<double> x,y,z");
	int ret = tx(keyword,"");
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__, "failed sending command" + keyword, ret);
	}
	
	std::string answer;
	ret = rx(answer,"$");
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__, "failed receiving answer for "+keyword, ret);
	}
	traceback(__FUNCTION__, answer, ret);

	float a,b,c;
	ret = sscanf(answer.c_str(),"%f,%f,%f", &a, &b, &c);
	if (ret != 3) 
	{
		char msg[1024];
		sprintf(msg, "[double] failed receiving correct number of values for command '%s' (got %d instead of 3) answer was '%s'", keyword.c_str(), ret, answer.c_str());
		return traceback(__FUNCTION__, msg, DEVICE_SERIAL_INVALID_RESPONSE);
	}
	x = (double) a;
	y = (double) b;
	z = (double) c;

	//parsestr(answer, x, y,  z);
	return DEVICE_OK;
}

int OpenStageHub::get(const std::string &keyword, long &x, long &y, long &z)
{
	logMessage("get<long> x,y,z");
	int ret = tx(keyword,"");
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__, "failed sending command" + keyword, ret);
	}
	
	std::string answer;
	ret = rx(answer,"$");
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__, "failed receiving answer for "+keyword, ret);
	}
	
	traceback(__FUNCTION__, answer, ret);

	ret = sscanf(answer.c_str(),"%ld,%ld,%ld", &x, &y, &z);
	if (ret != 3) 
	{
		char msg[1024];
		sprintf(msg, "[long] failed receiving correct number of values for command '%s' (got %d instead of 3) answer was '%s'", keyword.c_str(), ret, answer.c_str());
		return traceback(__FUNCTION__, msg, DEVICE_SERIAL_INVALID_RESPONSE);
	}
	return DEVICE_OK;
}

int OpenStageHub::set(const std::string &keyword, const long x, const long y, const long z) 
{
	char cmd[2048];
	snprintf(cmd,2048,"%s%ld,%ld,%ld",keyword.c_str(),x,y,z);
	logMessage("OpenStage::sending command");
	logMessage(cmd);
	int ret = tx(cmd, "$");
	if (ret != DEVICE_OK){
		return traceback(__FUNCTION__, "failed setting new values", ret);
	}
	boost::this_thread::sleep(delay_ms_);
	std::string answer;
	logMessage("OpenStage::receiving answer");

	ret = rx(answer,"$");
	
	logMessage(answer);
	if (ret != DEVICE_OK){
		return traceback(__FUNCTION__, "failed reading answer after set", ret);
	}
	return DEVICE_OK;
}

int OpenStageHub::goToAbsolutePositionUm(const double x, const double y, const double z) 
{
	int ret = 0;
	ret = updatePosition();
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"could not get position", ret);
	}
	
	if (!isPositionValid(x,y,z)) 
	{
		return traceback(__FUNCTION__,"invalid position", DEVICE_ERR);
	}
	ret = set("ga", (long)floor(1000.0*x), (long)floor(1000.0*y), (long)floor(1000.0*z));
	if( ret != DEVICE_OK )
	{
		return traceback(__FUNCTION__,"could not go to absolute position", ret);
	}

	ret = updatePosition();
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"could not get position", ret);
	}

	return DEVICE_OK;
}

int OpenStageHub::goToRelativePositionUm(const double dx, const double dy, const double dz) 
{
	logMessage("Start go to relative position um : " + std::to_string(dx) + "," + std::to_string(dy) + "," + std::to_string(dz));
	int ret = 0;
	ret = updatePosition();
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"could not get position", ret);
	}
	
	if (!isPositionValid(x_+dx,y_+dy,z_+dy)) 
	{
		return traceback(__FUNCTION__,"invalid position ", DEVICE_ERR);
	}

	ret = set("gr", (long)floor(1000.*dx), (long)floor(1000.0*dy), (long)floor(1000.0*dz));
	if ( ret != DEVICE_OK) 
	{
		return traceback(__FUNCTION__,"could not go to relative position", ret);
	}
	
	ret = updatePosition();
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"could not get position", ret);
	}
	
	return DEVICE_OK;
}

int OpenStageHub::updatePosition() {
	double x,y,z;
	int ret = get("p",x,y,z);
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"could not get position", ret);
	}
	char buf[1024];
	sprintf(buf,"## position is : x=%f, y=%f, z=%f",x,y,z);
	logMessage(buf);
	x_ = (double)x;
	y_ = (double)y;
	z_ = (double)z;
	step_x_ = (int) floor( (x_ - x0_) /  getStepTravelUm() );
	step_y_ = (int) floor( (y_ - y0_) /  getStepTravelUm() );
	step_z_ = (int) floor( (z_ - z0_) /  getStepTravelUm() );
	return DEVICE_OK;
}

int OpenStageHub::getPositionUm(double &x, double &y, double &z) {
	int ret = updatePosition();
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"could not get position", ret);
	}
	x = x_;
	y = y_;
	z = z_;
	return DEVICE_OK;
}

int OpenStageHub::zero() {
	int ret = tx("z","");
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"could not zero stage", ret);
	}
	x_ = 0;
	y_ = 0;
	z_ = 0;
	return DEVICE_OK;
}

int OpenStageHub::beep() {
	int ret = tx("b","");
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"could not beep", ret);
	}
	logMessage("OpenStage says bip.");
	return DEVICE_OK;
}

int OpenStageHub::setDelay(const long delay_ms) {
	this->delay_ms_ = boost::posix_time::milliseconds(delay_ms);
	logMessage("Set delay to "+ std::to_string(delay_ms));
	return DEVICE_OK;
}


std::string OpenStageHub::info() {
	std::string answer("No information..");
	//query("l", answer);
	return answer;
}

int OpenStageHub::setStepSizeIdx(int n) {
	if (n <= 1 || n >= 5)  {
		return traceback(__FUNCTION__, "index out of bound", DEVICE_ERR); 
	}
	char buf[256];
	snprintf(buf, 256, "ss%d", n);
	int ret = tx(buf,"");
	if (ret != DEVICE_OK) 
	{ 
		return traceback(__FUNCTION__, "could not set step size", ret); 
	}
	microstepping_idx_ = n - 1;
	return DEVICE_OK;
}

int OpenStageHub::getStepSizeValue(double & s) {
	
	int ret = tx("sr","");
	if (ret != DEVICE_OK) 
	{ 
		return traceback(__FUNCTION__, "could not ask for step size", ret); 
	}

	std::string answer;
	ret = rx(answer,"$");
	if (ret != DEVICE_OK) 
	{ 
		return traceback(__FUNCTION__, "could not read step size", ret); 
	}

	int k = sscanf(answer.c_str(),"%lf", &s);
	if (k!=1) 
	{ 
		return traceback(__FUNCTION__, "could not read step size", ret); 
	}

	for (int i = 0; i < 5; i++) {
		if (abs(s - microstepping_values_[i]) < 0.001)
		{
			microstepping_idx_ = i; 
			break;
		} 
	}
	return DEVICE_OK;
}

double OpenStageHub::getStepTravelUm() {
	return microstepping_values_[microstepping_idx_] * step_angle_ / 360.0 * gear_ratio_;
}

int OpenStageHub::setVelocity(const long x, const long y, const long z) 
{
	int ret = set("vs",x,y,z);
	if (ret != DEVICE_OK) 
	{ 
		return traceback(__FUNCTION__, "could not set velocity", ret); 
	}
	return DEVICE_OK;
}

int OpenStageHub::getVelocity(long &x, long &y, long &z) 
{
	int ret = get("vr",x,y,z);
	if (ret != DEVICE_OK) 
	{ 
		return traceback(__FUNCTION__, "could not get velocity", ret); 
	}
	return DEVICE_OK;
}

int OpenStageHub::setAcceleration(const long x, const long y, const long z) 
{
	int ret = set("as",x,y,z);
	if (ret != DEVICE_OK) 
	{ 
		return traceback(__FUNCTION__, "could not set acceleration", ret); 
	}
	return DEVICE_OK;
}

int OpenStageHub::getAcceleration(long &x, long &y, long &z) 
{
	int ret = get("ar",x,y,z);
	if (ret != DEVICE_OK) 
	{ 
		return traceback(__FUNCTION__, "could not get acceleration", ret); 
	}
	return DEVICE_OK;
}

void OpenStageHub::logMessage(const std::string & msg) const {
	if (core_) 
	{
		core_->LogMessage(device_, msg.c_str(), false);
	}
}

bool OpenStageHub::isPositionValid(const double x, const double y, const double z) 
{
	if (x >= xmin_ && x <= xmax_ && y >= ymin_ && y <= ymax_ && z >= zmin_ && z <= zmax_) 
	{
		return true;
	}
	else 
	{
		return false;
	}
}

int OpenStageHub::traceback(const std::string& identity, const std::string& msg, const int error) const
{
	logMessage(identity+"::"+msg);
	return error;
}


//////////////////////////////////////////////////////////////////////////////////
//                         OpenStageHub::Action Interface
//////////////////////////////////////////////////////////////////////////////////
int OpenStageHub::OnPort(MM::PropertyBase* pProp, MM::ActionType eAct)
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

int OpenStageHub::OnCommand(MM::PropertyBase* pProp, MM::ActionType eAct) 
{
	int ret = DEVICE_OK;
	if (eAct == MM::BeforeGet)
	{		
		pProp->Set(command_.c_str());
	}
	else if (eAct == MM::AfterSet)
	{
		pProp->Get(command_);     
		ret = tx(command_,"$");      
	}
	return ret;	
}

int OpenStageHub::OnStepSize(MM::PropertyBase* pProp, MM::ActionType eAct) 
{
	int ret = DEVICE_OK;
	if (eAct == MM::BeforeGet)
	{
		pProp->Set((double)microstepping_idx_);
	}
	else if (eAct == MM::AfterSet)
	{
		double val;
		pProp->Get(val);
		setStepSizeIdx((int)floor(val));
	}
	return ret;	
}

int OpenStageHub::OnDelay(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	int ret = DEVICE_OK;
	if (eAct == MM::BeforeGet)
	{
		pProp->Set(0.0);
	}
	else if (eAct == MM::AfterSet)
	{
		double val;
		pProp->Get(val);
		this->setDelay((long) floor(val));
		logMessage("set delay to " + std::to_string(val) + "ms\n");
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////////////////
//                         OpenStageXY::Device API
//////////////////////////////////////////////////////////////////////////////////

OpenStageXY::OpenStageXY()
{
	InitializeDefaultErrorMessages();
	CreateProperty(MM::g_Keyword_Name, g_OpenStageXY, MM::String, true);
	CreateProperty(MM::g_Keyword_Description, g_OpenStageXYDescription, MM::String, true);
}

OpenStageXY::~OpenStageXY() 
{
}

int OpenStageXY::Initialize() 
{

	// init device and core
	device_ = this;
	core_ = GetCoreCallback();

	// test the parent hub
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable()) 
	{
		return ERR_NO_PORT_SET;
	}

	// add properties

	CPropertyAction* pAct2 = new CPropertyAction (this, &OpenStageXY::OnSetRelativeXUm);
	CreateProperty("Go rel X um", "0", MM::Float, false, pAct2); 

	CPropertyAction* pAct3 = new CPropertyAction (this, &OpenStageXY::OnSetRelativeYUm);
	CreateProperty("Go rel Y um", "0", MM::Float, false, pAct3); 
	
	//CPropertyAction* pAct3 = new CPropertyAction (this, &OpenStageXY::OnGearRatio);
	//CreateProperty("GearRatio", "125.0", MM::Float, false, pAct3); 

	return DEVICE_OK;
}

int OpenStageXY::Shutdown() 
{
	return DEVICE_OK;
}

void OpenStageXY::GetName(char* name) const 
{
	CDeviceUtils::CopyLimitedString(name, g_OpenStageXY);
}

bool OpenStageXY::Busy() 
{
	return false;
}

//////////////////////////////////////////////////////////////////////////////////
//                         OpenStageXY:: XYStage API
//////////////////////////////////////////////////////////////////////////////////

int OpenStageXY::SetPositionUm(double x, double y) {
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}

	int ret = hub->goToAbsolutePositionUm(x,y,hub->z_);
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot move to absolute xy position", ret);
	}
	ret = hub->updatePosition();
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot read position", ret);
	}
	return DEVICE_OK;
}

int OpenStageXY::SetRelativePositionUm(double dx, double dy)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}

	int ret = hub->goToRelativePositionUm(dx,dy,0);
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot move relative to position", ret);
	}
	ret = hub->updatePosition();
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot read position", ret);
	}
	return DEVICE_OK;
}

int OpenStageXY::SetAdapterOriginUm(double x, double y)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	SetPositionUm(x,y);
	int ret = hub->zero();
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot set origin", ret);
	}
	return DEVICE_OK;
}

int OpenStageXY::GetPositionUm(double& x, double& y)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	double z;
	int ret = hub->getPositionUm(x,y,z);
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot get position", ret);
	}
	return DEVICE_OK;
}

int OpenStageXY::GetLimitsUm(double& xMin, double& xMax, double& yMin, double& yMax)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	xMin = hub->xmin_;
	xMax = hub->xmax_;
	yMin = hub->ymin_;
	yMax = hub->ymax_;
	return DEVICE_OK;
}

int OpenStageXY::Move(double vx, double vy)
{
	if (vx){}
	if (vy){}
	traceback(__FUNCTION__, "move is not implemented", 0);
	return DEVICE_OK;
}

int OpenStageXY::SetPositionSteps(long x, long y)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	// convert step to micron to feed the stage
	double 
		xr = hub->x0_ +  hub->getStepTravelUm() * x,
		yr = hub->y0_ +  hub->getStepTravelUm() * y;
	int ret = SetPositionUm(xr, yr);
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot move to position step", ret);
	}
	return DEVICE_OK;
}

int OpenStageXY::GetPositionSteps(long& x, long& y)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	hub->updatePosition();
	x = hub->step_x_;
	y = hub->step_y_;
	return DEVICE_OK;
}

int OpenStageXY::SetRelativePositionSteps(long x, long y)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	// convert step to micron to feed the stage
	double 
		xr =  hub->getStepTravelUm() * x,
		yr =  hub->getStepTravelUm() * y;
	int ret = SetRelativePositionUm(xr, yr);
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot move to position step", ret);
	}
	return DEVICE_OK;
}

int OpenStageXY::Home()
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	return SetPositionUm(hub->x0_,hub->y0_);
}

int OpenStageXY::Stop()
{
	return DEVICE_OK;
}

int OpenStageXY::SetOrigin()
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	double z;
	hub->getPositionUm(hub->x0_,hub->y0_,z);
	return DEVICE_OK;
}

int OpenStageXY::SetXOrigin()
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	double y,z;
	hub->getPositionUm(hub->x0_,y,z);
	return DEVICE_OK;
}

int OpenStageXY::SetYOrigin()
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	double x,z;
	hub->getPositionUm(x,hub->y0_,z);
	return DEVICE_OK;
}

int OpenStageXY::GetStepLimits(long& xMin, long& xMax, long& yMin, long& yMax)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	xMin = (long) floor( (hub->xmin_ - hub->x0_) / hub->getStepTravelUm() );
	xMax = (long) floor( (hub->xmax_ - hub->x0_) / hub->getStepTravelUm() );
	yMin = (long) floor( (hub->ymin_ - hub->y0_) / hub->getStepTravelUm() );
	yMax = (long) floor( (hub->ymax_ - hub->y0_) / hub->getStepTravelUm() );
	return DEVICE_OK;
}

double OpenStageXY::GetStepSizeXUm()
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	return hub->getStepTravelUm();
}

double OpenStageXY::GetStepSizeYUm()
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	return hub->getStepTravelUm();
}

int OpenStageXY::IsXYStageSequenceable(bool& isSequenceable) const
{
	isSequenceable = false;
	return DEVICE_OK;
}

int OpenStageXY::GetXYStageSequenceMaxLength(long& nrEvents) const
{
	nrEvents = 1;
	return DEVICE_OK;
}

int OpenStageXY::StartXYStageSequence()
{
	return DEVICE_OK;
}

int OpenStageXY::StopXYStageSequence()
{
	return DEVICE_OK;
}

int OpenStageXY::ClearXYStageSequence()
{
	return DEVICE_OK;
}

int OpenStageXY::AddToXYStageSequence(double positionX, double positionY)
{
	if (positionX){}
	if (positionY){}
	return DEVICE_OK;
}

int OpenStageXY::SendXYStageSequence()
{
	return DEVICE_OK;
}

int OpenStageXY::traceback(const std::string& identity, const std::string& msg, const int error) const
{
	logMessage(identity+"::"+msg);
	return error;
}

void OpenStageXY::logMessage(const std::string & msg) const {
	if (core_) {
		core_->LogMessage(device_, msg.c_str(), false);
	}
}


//////////////////////////////////////////////////////////////////////////////////
//                         OpenStageXY::Action interface
//////////////////////////////////////////////////////////////////////////////////



int OpenStageXY::OnGearRatio(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	int ret = DEVICE_OK;
	if (eAct == MM::BeforeGet)
	{		
		pProp->Set(hub->gear_ratio_);
	}
	else if (eAct == MM::AfterSet)
	{
		pProp->Get(hub->gear_ratio_);   
		// TODO set the gear ration
	}
	return ret;	
}

int OpenStageXY::OnSetRelativeXUm(MM::PropertyBase* pProp, MM::ActionType eAct) 
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	int ret = DEVICE_OK;
	if (eAct == MM::BeforeGet)
	{		
		pProp->Set(0.0);
	}
	else if (eAct == MM::AfterSet)
	{
		double val;
		pProp->Get(val);
		hub->goToRelativePositionUm(val,0,0);
	}
	return ret;	
}

int OpenStageXY::OnSetRelativeYUm(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	int ret = DEVICE_OK;
	if (eAct == MM::BeforeGet)
	{		
		pProp->Set(0.0);
	}
	else if (eAct == MM::AfterSet)
	{
		double val;
		pProp->Get(val);
		hub->goToRelativePositionUm(0,val,0);
	}
	return ret;	
}



//////////////////////////////////////////////////////////////////////////////////
//                         OpenStageZ::API
//////////////////////////////////////////////////////////////////////////////////
OpenStageZ::OpenStageZ()
{
	InitializeDefaultErrorMessages();
	CreateProperty(MM::g_Keyword_Name, g_OpenStageZ, MM::String, true);
	CreateProperty(MM::g_Keyword_Description, g_OpenStageZDescription, MM::String, true);
}

OpenStageZ::~OpenStageZ() 
{
}

int OpenStageZ::Initialize() 
{

	// init device and core
	device_ = this;
	core_ = GetCoreCallback();

	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}

	double s;
	hub->getStepSizeValue(s);
	hub->updatePosition();

	// add properties
	CPropertyAction* pAct1 = new CPropertyAction (this, &OpenStageZ::OnSetRelativeZUm);
	CreateProperty("Go rel Z um", "0", MM::Float, false, pAct1); 

	return DEVICE_OK;
}

int OpenStageZ::Shutdown() 
{
	return DEVICE_OK;
}

void OpenStageZ::GetName(char* name) const {
	CDeviceUtils::CopyLimitedString(name, g_OpenStageZ);
}

bool OpenStageZ::Busy() {
	return false;
}

//////////////////////////////////////////////////////////////////////////////////
//                         OpenStageZ::Stage API
//////////////////////////////////////////////////////////////////////////////////
int OpenStageZ::SetPositionUm(double pos)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	int ret = hub->goToAbsolutePositionUm(hub->x_,hub->y_,pos);
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot move to absolute z position", ret);
	}
	ret = hub->updatePosition();
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot read position", ret);
	}
	return DEVICE_OK;
}

int OpenStageZ::SetRelativePositionUm(double d)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	int ret = hub->goToRelativePositionUm(0,0,d);
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot move relative to position", ret);
	}
	ret = hub->updatePosition();
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot read position", ret);
	}
	return DEVICE_OK;
}

int OpenStageZ::Move(double velocity)
{
	if (velocity){}
	traceback(__FUNCTION__, "move is not implemented", 0);
	return DEVICE_OK;
}

int OpenStageZ::Stop()
{
	traceback(__FUNCTION__, "stop is not implemented", 0);
	return DEVICE_OK;
}

int OpenStageZ::Home()
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	return SetPositionUm(hub->z0_);
}

int OpenStageZ::SetAdapterOriginUm(double d)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	SetPositionUm(d);
	int ret = hub->zero();
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot set origin", ret);
	}
	return DEVICE_OK;
}

int OpenStageZ::GetPositionUm(double& pos)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	double x,y;
	int ret = hub->getPositionUm(x,y,pos);
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot get position", ret);
	}
	return DEVICE_OK;
}

int OpenStageZ::SetPositionSteps(long steps)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	double zr = hub->x0_ +  hub->getStepTravelUm() * steps;
	int ret = SetPositionUm(zr);
	if ( ret != DEVICE_OK ) 
	{
		return traceback(__FUNCTION__,"cannot move to position step", ret);
	}
	return DEVICE_OK;
}

int OpenStageZ::GetPositionSteps(long& steps)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	hub->updatePosition();
	steps = hub->step_z_;
	return DEVICE_OK;
}

int OpenStageZ::SetOrigin()
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	double x,y;
	hub->getPositionUm(x,y,hub->z0_);
	return DEVICE_OK;
}

int OpenStageZ::GetLimits(double& lower, double& upper)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	lower = hub->zmin_;
	upper = hub->zmax_;
	return DEVICE_OK;
}

//int OpenStageZ::GetFocusDirection(FocusDirection& direction)
//{
//	return DEVICE_OK;
//}

int OpenStageZ::IsStageSequenceable(bool& isSequenceable) const
{
	isSequenceable = false;
	return DEVICE_OK;
}

int OpenStageZ::IsStageLinearSequenceable(bool& isSequenceable) const
{
	isSequenceable = false;
	return DEVICE_OK;
}

bool OpenStageZ::IsContinuousFocusDrive() const
{
	return false;
}

int OpenStageZ::GetStageSequenceMaxLength(long& nrEvents) const
{
	nrEvents = 1;
	return DEVICE_OK;
}

int OpenStageZ::StartStageSequence()
{
	return DEVICE_OK;
}

int OpenStageZ::StopStageSequence()
{
	return DEVICE_OK;
}

int OpenStageZ::ClearStageSequence()
{
	return DEVICE_OK;
}

int OpenStageZ::AddToStageSequence(double position) 
{
	if (position){}
	return DEVICE_OK;
}

int OpenStageZ::SendStageSequence()
{
	return DEVICE_OK;
}

int OpenStageZ::SetStageLinearSequence(double dZ_um, long nSlices)
{
	if (dZ_um){}
	if (nSlices){}
	return DEVICE_OK;
}

void OpenStageZ::logMessage(const std::string & msg) const {
	if (core_) {
		core_->LogMessage(device_, msg.c_str(), false);
	}
}

int OpenStageZ::traceback(const std::string& identity, const std::string& msg, const int error) const
{
	logMessage(identity+"::"+msg);
	return error;
}


//////////////////////////////////////////////////////////////////////////////////
//                         OpenStageZ::Action interface
//////////////////////////////////////////////////////////////////////////////////
int OpenStageZ::OnSetRelativeZUm(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	OpenStageHub* hub = static_cast<OpenStageHub*>(GetParentHub());
	if (!hub || !hub->IsPortAvailable())
	{
		return traceback(__FUNCTION__,"no port available", ERR_NO_PORT_SET);
	}
	int ret = DEVICE_OK;
	if (eAct == MM::BeforeGet)
	{		
		pProp->Set(0.0);
	}
	else if (eAct == MM::AfterSet)
	{
		double val;
		pProp->Get(val);
		hub->goToRelativePositionUm(0,0,val);
	}
	return ret;	
}

