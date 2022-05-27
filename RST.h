#ifndef __RST__
#define __RST__

#pragma once
// C++ includes
#include <string>
#include <cstring>
#include <vector>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <ctime>
#include <cmath>
#include <algorithm>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/mountdriverinterface.h"
#include "../../licensedinterfaces/mount/asymmetricalequatorialinterface.h"

#include "StopWatch.h"

#define PLUGIN_VERSION 1.2

// #define PLUGIN_DEBUG 2   // define this to have log files, 1 = bad stuff only, 2 and up.. full debug

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

    int getRaAndDec(double &dRa, double &dDec);
    int syncTo(double dRa, double dDec);
    int isAligned(bool &bAligned);
    
    int setTrackingRates(bool bSiderialTrackingOn, bool bIgnoreRates, double dRaRateArcSecPerSec, double dDecRateArcSecPerSec);
    int getTrackRates(bool &bSiderialTrackingOn, double &dRaRateArcSecPerSec, double &dDecRateArcSecPerSec);

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

    int gotoPark(double dAlt, double dAz);
    int getAtPark(bool &bParked);
    int unPark();
    int isUnparkDone(bool &bcomplete);
    int isTrackingOn(bool &bTrakOn);

    int getLimits(double &dHoursEast, double &dHoursWest);

    int Abort();

    int setSiteData(double dLongitude, double dLatitute, double dTimeZone);
    int getSiteData(std::string &sLongitude, std::string &sLatitude, std::string &sTimeZone);
    void setSyncLocationDataConnect(bool bSync);

    int getLocalTime(std::string &sTime);
    int getLocalDate(std::string &sDate);
    int syncTime();
    int syncDate();

    int homeMount();
    int isHomingDone(bool &bIsHomed);

    int getInputVoltage(double &dVolts);

    int     IsBeyondThePole(bool &bBeyondPole);

#ifdef PLUGIN_DEBUG
    void log(std::string sLogEntry);
#endif
private:

    SerXInterface                       *m_pSerx;
    TheSkyXFacadeForDriversInterface    *m_pTsx;

	bool    m_bIsConnected;                               // Connected to the mount?
    std::string m_sFirmwareVersion;

    bool    m_bSyncLocationDataConnect;
    bool    m_bHomeOnUnpark;
    bool    m_bUnparking;
    int     m_nNbHomingTries;
    bool    m_bSyncDone;

    double m_dRaRateArcSecPerSec;
    double m_dDecRateArcSecPerSec;

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
    int     setTargetAltAz(double dAlt, double dAz);
    int     slewTargetRA_DecEpochNow();

    void    convertDecDegToDDMMSS(double dDeg, std::string &sResult);
    void    convertDecDegToDDMMSS_ForDecl(double dDeg, std::string &sResult);
    void    convertDecAzToDDMMSSs(double dDeg, std::string &sResult);

    int     convertDDMMSSToDecDeg(const std::string sStrDeg, double &dDecDeg);
    void    convertRaToHHMMSSt(double dRa, std::string &sResult);
    int     convertHHMMSStToRa(const std::string szStrRa, double &dRa);

    int     getDecAxisAlignmentOffset(double &dOffset);

    int     parseFields(const std::string sIn, std::vector<std::string> &svFields, char cSeparator);

    std::vector<std::string>    m_svSlewRateNames = {"Guide", "Centering", "Find", "Max"};

    
#ifdef PLUGIN_DEBUG
    // timestamp for logs
    const std::string getTimeStamp();
    std::ofstream m_sLogFile;
    std::string m_sLogfilePath;
#endif
	
};


#endif // __RST__

