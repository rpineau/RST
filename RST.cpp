#include "RST.h"

// Constructor for RST
RST::RST()
{

	m_bIsConnected = false;
    m_bLimitCached = false;

#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\RSTLog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/RSTLog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/RSTLog.txt";
#endif
    m_sLogFile.open(m_sLogfilePath, std::ios::out |std::ios::trunc);
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [RST] Version " << std::fixed << std::setprecision(2) << PLUGIN_VERSION << " build " << __DATE__ << " " << __TIME__ << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [RST] Constructor Called." << std::endl;
    m_sLogFile.flush();
#endif
}


RST::~RST(void)
{
#ifdef    PLUGIN_DEBUG
    // Close LogFile
    if(m_sLogFile.is_open())
        m_sLogFile.close();
#endif
}

int RST::Connect(char *pszPort)
{
    bool bIsParked;
    bool bIsAligned;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Connect Called." << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Trying to connect to port " << pszPort<< std::endl;
    m_sLogFile.flush();
#endif

    // 115.2K 8N1
    if(m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] m_mountType : " << m_mountType << std::endl;
    m_sLogFile.flush();
#endif

    // set mount type
    switch(m_mountType) {
        case MountTypeInterface::Symmetrical_Equatorial:
            break;

        case MountTypeInterface::AltAz :
            break;
            
        default :
            break;
    }

    return SB_OK;
}


int RST::Disconnect(void)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Disconnect] Disconnect Called." << std::endl;
    m_sLogFile.flush();
#endif
	if (m_bIsConnected) {
        if(m_pSerx){
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Disconnect] closing serial port." << std::endl;
            m_sLogFile.flush();
#endif
            m_pSerx->flushTx();
            m_pSerx->purgeTxRx();
            m_pSerx->close();
        }
    }
	m_bIsConnected = false;

	return SB_OK;
}




#pragma mark - RST communication
int RST::sendCommand(const std::string sCmd, std::string &sResp, int nTimeout)
{
    int nErr = PLUGIN_OK;
    unsigned long  ulBytesWrite;

    m_pSerx->purgeTxRx();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [domeCommand] sending : " << sCmd << std::endl;
    m_sLogFile.flush();
#endif

    nErr = m_pSerx->writeFile((void *)sCmd.c_str(), sCmd.size(), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;

    // read response
    if(nTimeout == 0) // no response expected
        return nErr;

    nErr = readResponse(sResp, nTimeout);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [domeCommand] ***** ERROR READING RESPONSE **** error = " << nErr << " , response : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [domeCommand] response : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}


int RST::readResponse(std::string &sResp, int nTimeout)
{
    int nErr = PLUGIN_OK;
    char pszBuf[SERIAL_BUFFER_SIZE];
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;
    int nBytesWaiting = 0 ;
    int nbTimeouts = 0;

    sResp.clear();
    memset(pszBuf, 0, SERIAL_BUFFER_SIZE);
    pszBufPtr = pszBuf;

    do {
        nErr = m_pSerx->bytesWaitingRx(nBytesWaiting);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] nBytesWaiting      : " << nBytesWaiting << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] nBytesWaiting nErr : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        if(!nBytesWaiting) {
            nbTimeouts += MAX_READ_WAIT_TIMEOUT;
            if(nbTimeouts >= nTimeout) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] bytesWaitingRx timeout, no data for " << nbTimeouts << " ms"<< std::endl;
                m_sLogFile.flush();
#endif
                nErr = COMMAND_TIMEOUT;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(MAX_READ_WAIT_TIMEOUT));
            continue;
        }
        nbTimeouts = 0;
        if(ulTotalBytesRead + nBytesWaiting <= SERIAL_BUFFER_SIZE)
            nErr = m_pSerx->readFile(pszBufPtr, nBytesWaiting, ulBytesRead, nTimeout);
        else {
            nErr = ERR_RXTIMEOUT;
            break; // buffer is full.. there is a problem !!
        }
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile error : " << nErr << std::endl;
            m_sLogFile.flush();
#endif
            return nErr;
        }

        if (ulBytesRead != nBytesWaiting) { // timeout
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] rreadFile Timeout Error." << std::endl;
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile nBytesWaiting : " << nBytesWaiting << std::endl;
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] readFile ulBytesRead   : " << ulBytesRead << std::endl;
            m_sLogFile.flush();
#endif
        }

        ulTotalBytesRead += ulBytesRead;
        pszBufPtr+=ulBytesRead;
    }  while (ulTotalBytesRead < SERIAL_BUFFER_SIZE  && *(pszBufPtr-1) != '#');

    if(!ulTotalBytesRead)
        nErr = COMMAND_TIMEOUT; // we didn't get an answer.. so timeout
    else
        *(pszBufPtr-1) = 0; //remove the #

    sResp.assign(pszBuf);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse] sResp : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int RST::getFirmwareVersion(std::string &sFirmware)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = sendCommand(":AV#", sResp);
    sFirmware.assign(sResp);
    return nErr;
}

#pragma mark - Mount Coordinates
void RST::setMountMode(MountTypeInterface::Type mountType)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setMountMode]  mountType = " << mountType << std::endl;
    m_sLogFile.flush();
#endif

    m_mountType = mountType;
}

MountTypeInterface::Type RST::mountType()
{
    return m_mountType;
}


int RST::getRaAndDec(double &dRa, double &dDec)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    // get RA
    nErr = sendCommand(":GR#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec]':GR#' ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec]  sResp : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

#pragma mark - FIX RA CONVERSION
    nErr = convertHHMMSStToRa(sResp.c_str(), dRa);
    if(nErr)
        return nErr;

    // get DEC
    nErr = sendCommand(":GD#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec]':GD#' ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
#pragma mark - FIX DEC CONVERSION
    nErr = convertDDMMSSToDecDeg(sResp.c_str(), dDec);

    return nErr;
}

int RST::setTarget(double dRa, double dDec)
{
    int nErr;
    std::stringstream ssTmp;
    std::string sResp;
    std::string sTemp;
    char cSign;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget]  Ra  : " << std::fixed << std::setprecision(2) << dRa << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget]  Dec : " << std::fixed << std::setprecision(2) << dDec << std::endl;
    m_sLogFile.flush();
#endif

    // convert Ra value to HH:MM:SS.T before passing them to the RST
    convertRaToHHMMSSt(dRa, sTemp);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget]  szTemp(Ra)  : " << sTemp << std::endl;
    m_sLogFile.flush();
#endif
    // set target Ra
    ssTmp<<":Sr"<<sTemp<<"#";
    nErr = sendCommand(ssTmp.str(), sResp);


    // set target dec
    convertDecDegToDDMMSS(dDec, sTemp, cSign);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget]  szTemp(Dec)  : " << cSign<<sTemp << std::endl;
    m_sLogFile.flush();
#endif
    std::stringstream().swap(ssTmp);

    ssTmp<<":Sd"<<cSign<<sTemp<<"#";
    nErr = sendCommand(ssTmp.str(), sResp);
    return nErr;
}

#pragma mark - Sync and Cal
int RST::syncTo(double dRa, double dDec)
{
    int nErr = PLUGIN_OK;
    bool bAligned;
    std::stringstream ssTmp;
    std::string sResp;
    char cSign;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo]  Ra  : " << std::fixed << std::setprecision(2) << dRa << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo]  Dec : " << std::fixed << std::setprecision(2) << dDec << std::endl;
    m_sLogFile.flush();
#endif


    if(dDec <0) {
        cSign = '-';
        dDec = -dDec;
    } else {
        cSign = '+';
    }
    ssTmp << ":Ck" << std::fixed << std::setprecision(2) << dRa << cSign << std::fixed << std::setprecision(2) << dDec << "#";
    nErr = sendCommand(ssTmp.str(), sResp);

    return nErr;
}

int RST::isAligned(bool &bAligned)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];


    return nErr;
}

#pragma mark - tracking rates
int RST::setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dTrackRaArcSecPerHr, double dTrackDecArcSecPerHr)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!bTrackingOn) { // stop tracking
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] setting to stopped" << std::endl;
        m_sLogFile.flush();
#endif
        nErr = sendCommand(":CtL#", sResp); // tracking off
    }
    else if(bTrackingOn && bIgnoreRates) { // sidereal
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] setting to Sidereal" << std::endl;
        m_sLogFile.flush();
#endif
        nErr = sendCommand(":CT0#", sResp);
    }
    else { // lunar, solar, ..
    }
    return nErr;
}

int RST::getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerHr, double &dTrackDecArcSecPerHr)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = sendCommand(":Ct?#", sResp);
    if(nErr)
        return nErr;
    switch(sResp.at(3)) {
        case '0' :  // Sidereal
            dTrackRaArcSecPerHr = 0;
            dTrackDecArcSecPerHr = 0;
            bTrackingOn = true;
            break;
        case '1' :  // Solar
            dTrackRaArcSecPerHr = 0;
            dTrackDecArcSecPerHr = 0;
            bTrackingOn = true;
            break;
        case '2' :  // Lunar
            dTrackRaArcSecPerHr = 0;
            dTrackDecArcSecPerHr = 0;
            bTrackingOn = true;
            break;
        case '3' :  //  Guide
            dTrackRaArcSecPerHr = 0;
            dTrackDecArcSecPerHr = 0;
            bTrackingOn = true;
            break;
        default:
            bTrackingOn = false;
            break;
    }
    bTrackingOn = true;

    return nErr;
}


#pragma mark - Limis
int RST::getLimits(double &dHoursEast, double &dHoursWest)
{
    int nErr = PLUGIN_OK;
    double dEast, dWest;

    if(m_bLimitCached) {
        dHoursEast = m_dHoursEast;
        dHoursWest = m_dHoursWest;
        return nErr;
    }

    nErr = getSoftLimitEastAngle(dEast);
    if(nErr)
        return nErr;

    nErr = getSoftLimitWestAngle(dWest);
    if(nErr)
        return nErr;

    dHoursEast = m_pTsx->hourAngle(dEast);
    dHoursWest = m_pTsx->hourAngle(dWest);

    m_bLimitCached = true;
    m_dHoursEast = dHoursEast;
    m_dHoursWest = dHoursWest;
    return nErr;
}

#pragma mark - TODO getSoftLimitEastAngle
int RST::getSoftLimitEastAngle(double &dAngle)
{
    int nErr;
    std::string sResp;
    nErr = sendCommand("", sResp);
    if(nErr)
        return nErr;
    try {
        dAngle = std::stof(sResp);
    }
    catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSoftLimitEastAngle] conversion exception : " << e.what() << std::endl;
        m_sLogFile.flush();
#endif
        return ERR_CMDFAILED;
    }

    return nErr;
}

#pragma mark - TODO getSoftLimitWestAngle
int RST::getSoftLimitWestAngle(double &dAngle)
{
    int nErr;
    std::string sResp;

    nErr = sendCommand("", sResp);
    if(nErr)
        return nErr;
    try {
        dAngle = std::stof(sResp);
    }
    catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSoftLimitWestAngle] conversion exception : " << e.what() << std::endl;
        m_sLogFile.flush();
#endif
        return ERR_CMDFAILED;
    }

    return nErr;
}

#pragma mark - Slew

int RST::startSlewTo(double dRa, double dDec)
{
    int nErr = PLUGIN_OK;
    bool bAligned;

    nErr = isAligned(bAligned);
    if(nErr)
        return nErr;

    // set sync target coordinate
    nErr = setTarget(dRa, dDec);
    if(nErr)
        return nErr;

    slewTargetRA_DecEpochNow();
    return nErr;
}

int RST::slewTargetRA_DecEpochNow()
{
    int nErr;
    std::string sResp;

    nErr = sendCommand(":MS#", sResp);
    // nErr = sendCommand(":MA#", sResp);   // AltAz

    timer.Reset();
    return nErr;

}

int RST::getNbSlewRates()
{
    return PLUGIN_NB_SLEW_SPEEDS;
}

// returns "Slew", "ViewVel4", "ViewVel3", "ViewVel2", "ViewVel1"
int RST::getRateName(int nZeroBasedIndex, std::string &sOut)
{
    if (nZeroBasedIndex > PLUGIN_NB_SLEW_SPEEDS)
        return PLUGIN_ERROR;

    sOut.assign(m_svSlewRateNames[nZeroBasedIndex]);
    return PLUGIN_OK;
}

int RST::startOpenLoopMove(const MountDriverInterface::MoveDir Dir, unsigned int nRate)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    m_nOpenLoopDir = Dir;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [startOpenLoopMove] setting dir to  : " << Dir << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [startOpenLoopMove] setting rate to : " << nRate << std::endl;
    m_sLogFile.flush();
#endif

    // select rate
    m_nOpenLoopDir = Dir;
    switch(nRate) {
        case 0:
            nErr = sendCommand(":RG#", sResp, 0);
            break;

        case 1:
            nErr = sendCommand(":RC#", sResp, 0);
            break;

        case 2:
            nErr = sendCommand(":RM#", sResp, 0);
            break;

        case 3:
            nErr = sendCommand(":RS#", sResp, 0);
            break;

        default :
            return COMMAND_FAILED;
            break;
    }
    
    // figure out direction
    switch(Dir){
        case MountDriverInterface::MD_NORTH:
            nErr = sendCommand(":Mn#", sResp, 0);
            break;
        case MountDriverInterface::MD_SOUTH:
            nErr = sendCommand(":Ms#", sResp, 0);
            break;
        case MountDriverInterface::MD_EAST:
            nErr = sendCommand(":Me#", sResp, 0);
            break;
        case MountDriverInterface::MD_WEST:
            nErr = sendCommand(":Mw#", sResp, 0);
            break;
    }

    return nErr;
}

int RST::stopOpenLoopMove()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [stopOpenLoopMove] dir was  : " << m_nOpenLoopDir << std::endl;
    m_sLogFile.flush();
#endif

    switch(m_nOpenLoopDir){
        case MountDriverInterface::MD_NORTH:
            nErr = sendCommand(":Qn#", sResp, 0);
            break;
        case MountDriverInterface::MD_SOUTH:
            nErr = sendCommand(":Qs#", sResp, 0);
            break;
        case MountDriverInterface::MD_EAST:
            nErr = sendCommand(":Qe#", sResp, 0);
            break;
        case MountDriverInterface::MD_WEST:
            nErr = sendCommand(":Qw#", sResp, 0);
            break;
    }

    return nErr;
}


int RST::setSpeed(const int nSpeedId, const int nSpeed)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssTmp;
    std::string sResp;

    ssTmp << ":Cu" << nSpeedId << "=" << std::setfill('0') << std::setw(4) << nSpeed << "#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    return nErr;
}

#pragma mark - TODO : fix response parsing
int RST::getSpeed(const int nSpeedId, int &nSpeed)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssTmp;
    std::string sResp;

    ssTmp << ":CU" << nSpeedId << "#";
    nErr = sendCommand(ssTmp.str(), sResp);
    if(nErr)
        return nErr;

    // nSpeed = atoi(szResp+5);
    
    return nErr;
}

int RST::setGuideSpeed(const double dSpeed)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssTmp;
    std::string sResp;

    ssTmp << ":Cu0=" << std::fixed << std::setprecision(1) << dSpeed << "#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    return nErr;
}

#pragma mark - TODO : fix response parsing
int RST::getGuideSpeed(double &dSpeed)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = sendCommand(":CU0#", sResp);
    if(nErr)
        return nErr;
    
    // dSpeed = atof(szResp+5);

    return nErr;
}

#pragma mark - TODO : parse response and set bComplete
int RST::isSlewToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    bComplete = false;

    if(timer.GetElapsedSeconds()<2) {
        // we're checking for comletion to quickly, assume it's moving for now
        return nErr;
    }

    nErr = sendCommand(":CL#", sResp);
    if(nErr)
        return nErr;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isSlewToComplete] sResp : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int RST::gotoPark(double dRa, double dDec)
{
    int nErr = PLUGIN_OK;

    nErr = startSlewTo(dRa, dDec);
    return nErr;
}


int RST::getAtPark(bool &bParked)
{
    int nErr = PLUGIN_OK;

    return nErr;
}

int RST::unPark()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = sendCommand(":CtA#", sResp);
    return nErr;
}


int RST::Abort()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = sendCommand(":Q#", sResp);
    nErr |= sendCommand(":CtL#", sResp);
    return nErr;
}

#pragma mark - time and site methods
int RST::syncTime()
{
    int nErr = PLUGIN_OK;
    int yy, mm, dd, h, min, dst;
    double sec;
    std::string sResp;
    std::stringstream ssTmp;

    m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);

    ssTmp << ":SL" << std::setfill('0') << std::setw(2) << h << std::setfill('0') << std::setw(2) << min << std::setfill('0') << std::setw(2) << int(sec) << "#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    getLocalTime(m_sTime);

    return nErr;
}


int RST::syncDate()
{
    int nErr = PLUGIN_OK;
    int yy, mm, dd, h, min, dst;
    double sec;
    std::string sResp;
    std::stringstream ssTmp;


    m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);
    // yy is actually yyyy, need conversion to yy, 2017 -> 17
    yy = yy - (int(yy / 1000) * 1000);

    ssTmp << ":SC" << std::setfill('0') << std::setw(2) << mm << "/" << std::setfill('0') << std::setw(2) << dd << "/" << std::setfill('0') << std::setw(2) << yy << "#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);

    getLocalDate(m_sDate);
    return nErr;
}

#pragma mark - Check value format are correct
int RST::setSiteLongitude(const std::string sLongitude)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

    // :SgsDDD*MM'SS#
    nErr = sendCommand(ssTmp.str(), sResp, 0);

    return nErr;
}

int RST::setSiteLatitude(const std::string sLatitude)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

    // :StsDD*MM'SS#
    nErr = sendCommand(ssTmp.str(), sResp, 0);

    return nErr;
}

int RST::setSiteTimezone(const std::string sTimezone)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

    // :SGsHH#
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    return nErr;
}

#pragma mark - TODO : Parse response
int RST::getSiteLongitude(std::string &sLongitude)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = sendCommand(":Gg#", sResp);
    if(!nErr) {
        sLongitude.assign(sResp);
    }
    return nErr;
}

#pragma mark - TODO : Parse response
int RST::getSiteLatitude(std::string &sLatitude)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = sendCommand(":Gt#", sResp);
    if(!nErr) {
        sLatitude.assign(sResp);
    }

    return nErr;
}

#pragma mark - TODO : Parse response
int RST::getSiteTZ(std::string &sTimeZone)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = sendCommand(":GG#", sResp);
    if(!nErr) {
        sTimeZone.assign(sResp);
    }

    return nErr;
}

int RST::setSiteData(double dLongitude, double dLatitute, double dTimeZone)
{
    int nErr = PLUGIN_OK;
    std::string sLong;
    std::string sLat;
    std::stringstream ssTimeZone;
    std::stringstream ssTmp;

    std::stringstream  ssHH, ssMM;
    char cSignLong;
    char cSignLat;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dLongitude : " << std::fixed << std::setprecision(2) << dLongitude << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dLatitute : " << std::fixed << std::setprecision(2) << dLatitute << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dTimeZone : " << std::fixed << std::setprecision(2) << dTimeZone << std::endl;
    m_sLogFile.flush();
#endif

    convertDecDegToDDMMSS(dLongitude, sLong, cSignLong);
    convertDecDegToDDMMSS(dLatitute, sLat, cSignLat);
    
    ssHH << std::setfill('0') << std::setw(2) << int(std::fabs(dTimeZone));
    ssMM << std::setfill('0') << std::setw(2) << int((std::fabs(dTimeZone) - int(std::fabs(dTimeZone)))) * 100;

    if(dTimeZone<0) {
        ssTimeZone << ssHH.str() << ":" << ssMM.str() << "W";
    }
    else if (dTimeZone>0) {
        ssTimeZone << ssHH.str() << ":" << ssMM.str() << "E";
    }
    else
        ssTimeZone << "00:00";

    // Set the W/E
    if(dLongitude<0) {
        sLong+="W";
    }
    else {
        sLong+="E";
    }
    // convert signed latitude to N/S
    if(dLatitute>=0) {
        sLat+="N";
    }
    else {
        sLat+="S";
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] sLong      : " << sLong << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] sLat       : " << sLat<< std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] ssTimeZone : " << ssTimeZone.str() << std::endl;
    m_sLogFile.flush();
#endif

    nErr = setSiteLongitude(sLong);
    nErr |= setSiteLatitude(sLat);
    nErr |= setSiteTimezone(ssTimeZone.str());

    return nErr;
}

int RST::getSiteData(std::string &sLongitude, std::string &sLatitude, std::string &sTimeZone)
{
    int nErr = PLUGIN_OK;

    nErr = getSiteLongitude(sLongitude);
    nErr |= getSiteLatitude(sLatitude);
    nErr |= getSiteTZ(sTimeZone);
    return nErr;
}



#pragma mark  - Time and Date

#pragma mark - Parse result
int RST::getLocalTime(std::string &sTime)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = sendCommand(":GL#", sResp);
    if(nErr)
        return nErr;
    sTime.assign(sResp);
    return nErr;
}

int RST::getLocalDate(std::string &sDate)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    nErr = sendCommand(":GC#", sResp);
    if(nErr)
        return nErr;
    sDate.assign(sResp);
    return nErr;
}


void RST::convertDecDegToDDMMSS(double dDeg, std::string &sResult, char &cSign)
{
    int DD, MM, SS;
    double mm, ss;
    std::stringstream ssTmp;

    sResult.clear();
    // convert dDeg decimal value to sDD:MM:SS
    cSign = dDeg>=0?'+':'-';
    DD = int(std::fabs(dDeg));
    mm = dDeg - DD;
    MM = int(mm*60);
    ss = (mm*60) - MM;
    SS = int(std::ceil(ss*60));

    ssTmp << cSign << std::setfill('0') << std::setw(2) << DD << ":" << std::setfill('0') << std::setw(2) << MM << ":" << std::setfill('0') << std::setw(2) << SS;
    sResult.assign(ssTmp.str());
}


int RST::convertDDMMSSToDecDeg(const std::string sStrDeg, double &dDecDeg)
{
    int nErr = PLUGIN_OK;
    std::vector<std::string> vFieldsData;

    dDecDeg = 0;

    nErr = parseFields(sStrDeg, vFieldsData, ':');
    if(nErr)
        return nErr;

    if(vFieldsData.size() >= 3) {
        dDecDeg = atof(vFieldsData[0].c_str()) + atof(vFieldsData[1].c_str())/60 + atof(vFieldsData[1].c_str())/3600;
    }
    else
        nErr = ERR_PARSE;

    return nErr;
}

void RST::convertRaToHHMMSSt(double dRa, std::string &sResult)
{
    int HH, MM;
    double hh, mm, SSt;
    std::stringstream ssTmp;

    sResult.clear();
    // convert Ra value to HH:MM:SS.T before passing them to the RST
    HH = int(dRa);
    hh = dRa - HH;
    MM = int(hh*60);
    mm = (hh*60) - MM;
    SSt = mm * 60;

    ssTmp << std::setfill('0') << std::setw(2) << HH << ":" << std::setfill('0') << std::setw(2) << MM << ":" << std::setfill('0') << std::setw(2) << std::fixed << std::setprecision(1) << SSt;
    sResult.assign(ssTmp.str());
}


int RST::convertHHMMSStToRa(const std::string szStrRa, double &dRa)
{
    int nErr = PLUGIN_OK;
    std::vector<std::string> vFieldsData;

    dRa = 0;

    nErr = parseFields(szStrRa, vFieldsData, ':');
    if(nErr)
        return nErr;

    if(vFieldsData.size() >= 3) {
        try {
            dRa = std::stof(vFieldsData[0]) + std::stof(vFieldsData[1])/60 + std::stof(vFieldsData[2])/3600;
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertHHMMSStToRa] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_PARSE;
        }
    }
    else
        nErr = ERR_PARSE;

    return nErr;
}


int RST::parseFields(const std::string sIn, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = PLUGIN_OK;
    std::string sSegment;
    std::stringstream ssTmp(sIn);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = ERR_PARSE;
    }
    return nErr;
}

#ifdef PLUGIN_DEBUG
const std::string RST::getTimeStamp()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    std::strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}
#endif
