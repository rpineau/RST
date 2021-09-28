#pragma once
// C++ includes
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <ctime>
#include <cmath>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/mountdriverinterface.h"
#include "../../licensedinterfaces/mount/asymmetricalequatorialinterface.h"

#include "StopWatch.h"

#define PLUGIN_VERSION 1.0

#define PLUGIN_DEBUG 2   // define this to have log files, 1 = bad stuff only, 2 and up.. full debug

enum RSTErrors {PLUGIN_OK=0, NOT_CONNECTED, PLUGIN_CANT_CONNECT, PLUGIN_BAD_CMD_RESPONSE, COMMAND_FAILED, PLUGIN_ERROR, COMMAND_TIMEOUT};

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 1000
#define MAX_READ_WAIT_TIMEOUT 25
#define ND_LOG_BUFFER_SIZE 256
#define ERR_PARSE   1

#define PLUGIN_NB_SLEW_SPEEDS 4


// Define Class for Astrometric Instruments RST controller.
class RST
{
public:
	RST();
	~RST();
	
	int Connect(char *pszPort);
	int Disconnect();
	bool isConnected() const { return m_bIsConnected; }

    void setSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void setTSX(TheSkyXFacadeForDriversInterface *pTSX) { m_pTsx = pTSX;};

    int getFirmwareVersion(std::string &sFirmware);

    void setMountMode(MountTypeInterface::Type mountType);
    MountTypeInterface::Type mountType();

    int getRaAndDec(double &dRa, double &dDec);
    int syncTo(double dRa, double dDec);
    int isAligned(bool &bAligned);
    
    int setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dTrackRaArcSecPerHr, double dTrackDecArcSecPerHr);
    int getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerHr, double &dTrackDecArcSecPerHr);

    int startSlewTo(double dRa, double dDec);
    int isSlewToComplete(bool &bComplete);

    int startOpenLoopMove(const MountDriverInterface::MoveDir Dir, unsigned int nRate);
    int stopOpenLoopMove();
    int getNbSlewRates();
    int getRateName(int nZeroBasedIndex, std::string &sOut);
    
    int setSpeed(const int nSpeedId, const int nSpeed);
    int getSpeed(const int nSpeedId, int &nSpeed);

    int setGuideSpeed(const double dSpeed);
    int getGuideSpeed(double &dSpeed);

    int gotoPark(double dRa, double dDEc);
    int getAtPark(bool &bParked);
    int unPark();


    int getLimits(double &dHoursEast, double &dHoursWest);

    int Abort();

    int setSiteData(double dLongitude, double dLatitute, double dTimeZone);
    int getSiteData(std::string &sLongitude, std::string &sLatitude, std::string &sTimeZone);

    int getLocalTime(std::string &sTime);
    int getLocalDate(std::string &sDate);
    int syncTime();
    int syncDate();

private:

    SerXInterface                       *m_pSerx;
    TheSkyXFacadeForDriversInterface    *m_pTsx;

	bool    m_bIsConnected;                               // Connected to the mount?
    std::string m_sFirmwareVersion;

    MountTypeInterface::Type    m_mountType;
    
    std::string     m_sTime;
    std::string     m_sDate;

	double  m_dGotoRATarget;						  // Current Target RA;
	double  m_dGotoDECTarget;                      // Current Goto Target Dec;
	
    MountDriverInterface::MoveDir      m_nOpenLoopDir;

    // limits don't change mid-course so we cache them
    bool    m_bLimitCached;
    double  m_dHoursEast;
    double  m_dHoursWest;

    int     sendCommand(const std::string sCmd, std::string &sResp, int nTimeout = MAX_TIMEOUT);
    int     readResponse(std::string &sResp, int nTimeout = MAX_TIMEOUT);

    int     setSiteLongitude(const std::string sLongitude);
    int     setSiteLatitude(const std::string sLatitude);
    int     setSiteTimezone(const std::string sTimezone);

    int     getSiteLongitude(std::string &sLongitude);
    int     getSiteLatitude(std::string &sLatitude);
    int     getSiteTZ(std::string &sTimeZone);

    int     setTarget(double dRa, double dDec);
    int     slewTargetRA_DecEpochNow();

    int     getSoftLimitEastAngle(double &dAngle);
    int     getSoftLimitWestAngle(double &dAngle);

    void    convertDecDegToDDMMSS(double dDeg, std::string &sResult, char &cSign);

    int     convertDDMMSSToDecDeg(const std::string sStrDeg, double &dDecDeg);
    
    void    convertRaToHHMMSSt(double dRa, std::string &sResult);
    int     convertHHMMSStToRa(const std::string szStrRa, double &dRa);

    int     parseFields(const std::string sIn, std::vector<std::string> &svFields, char cSeparator);

    std::vector<std::string>    m_svSlewRateNames = {"Guide", "Centering", "Find", "Max"};
    CStopWatch      timer;

    
#ifdef PLUGIN_DEBUG
    // timestamp for logs
    const std::string getTimeStamp();
    std::ofstream m_sLogFile;
    std::string m_sLogfilePath;
#endif
	
};


