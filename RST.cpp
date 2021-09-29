#include "RST.h"

// Constructor for RST
RST::RST()
{

	m_bIsConnected = false;
    m_bLimitCached = false;

    m_dHoursEast = 8.0;
    m_dHoursWest = 8.0;

    m_bSyncTimeAndDateOnConnect = false;
    m_bSyncLocationOnConnect = false;

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
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] m_mountType is Equatorial " << std::endl;
            m_sLogFile.flush();
#endif
            break;

        case MountTypeInterface::AltAz :
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] m_mountType is AltAz " << std::endl;
            m_sLogFile.flush();
#endif
            break;
            
        default :
            break;
    }

    if(m_bSyncTimeAndDateOnConnect) {
        syncTime();
        syncDate();
    }

    if(m_bSyncLocationOnConnect) {
        setSiteData(m_pTsx->longitude(),
                    m_pTsx->latitude(),
                    m_pTsx->timeZone());
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
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [domeCommand] sending "<< sCmd<< std::endl;
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
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [domeCommand] response " << sResp <<  std::endl;
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
    else if(*(pszBufPtr-1) == '#')
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
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getFirmwareVersion] Called." << std::endl;
    m_sLogFile.flush();
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // get RA
    nErr = sendCommand(":GR#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] :GR# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec]  sResp : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    nErr = convertHHMMSStToRa(sResp.substr(3), dRa);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] :GD# convertHHMMSStToRa error : " << nErr << " , sResp.substr(3) : " << sResp.substr(3) << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec]  dRa : " << std::fixed << std::setprecision(8) << dRa << std::endl;
    m_sLogFile.flush();
#endif

    // get DEC
    nErr = sendCommand(":GD#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] :GD# ERROR : " << nErr << " , sResp : " << sResp << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    nErr = convertDDMMSSToDecDeg(sResp.substr(3), dDec);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] :GD# convertDDMMSSToDecDeg error : " << nErr << " , sResp.substr(3) : " << sResp.substr(3) << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] dDec : " << std::fixed << std::setprecision(8) << dDec << std::endl;
    m_sLogFile.flush();
#endif

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
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] Ra  : " << std::fixed << std::setprecision(8) << dRa << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] Dec : " << std::fixed << std::setprecision(8) << dDec << std::endl;
    m_sLogFile.flush();
#endif

    // convert Ra value to HH:MM:SS.T before passing them to the RST
    convertRaToHHMMSSt(dRa, sTemp);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] szTemp(Ra)  : " << sTemp << std::endl;
    m_sLogFile.flush();
#endif
    // set target Ra
    ssTmp<<":Sr"<<sTemp<<"#";
    nErr = sendCommand(ssTmp.str(), sResp, 100); // need a fast error as this command doesn't follow the usual format and doesn't end with #
    if(sResp.size() && sResp.at(0)=='1') {
        nErr = PLUGIN_OK;
    }
    else if(nErr)
        return nErr;

    // set target dec
    convertDecDegToDDMMSS_ForDecl(dDec, sTemp, cSign);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget]  szTemp(Dec)  : " << cSign<<sTemp << std::endl;
    m_sLogFile.flush();
#endif
    std::stringstream().swap(ssTmp);

    ssTmp<<":Sd"<<cSign<<sTemp<<"#";
    nErr = sendCommand(ssTmp.str(), sResp, 100); // need a fast error as this command doesn't follow the usual format and doesn't end with #
    if(sResp.size() && sResp.at(0)=='1')
        nErr = PLUGIN_OK;
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
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo]  Ra  : " << std::fixed << std::setprecision(5) << dRa << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo]  Dec : " << std::fixed << std::setprecision(5) << dDec << std::endl;
    m_sLogFile.flush();
#endif


    if(dDec <0) {
        cSign = '-';
        dDec = -dDec;
    } else {
        cSign = '+';
    }
    ssTmp << ":Ck" << std::setfill('0') << std::setw(7) << std::fixed << std::setprecision(3) << dRa << cSign << std::setfill('0') << std::setw(6)<< std::fixed << std::setprecision(3) << dDec << "#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return nErr;
}

int RST::isAligned(bool &bAligned)
{
    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isAligned] Called." << std::endl;
    m_sLogFile.flush();
#endif
    // for now
    bAligned = true;
    return nErr;
}

#pragma mark - tracking rates
int RST::setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dTrackRaArcSecPerHr, double dTrackDecArcSecPerHr)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] Called." << std::endl;
    m_sLogFile.flush();
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getTrackRates] Called." << std::endl;
    m_sLogFile.flush();
#endif

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


#pragma mark - Limits
int RST::getLimits(double &dHoursEast, double &dHoursWest)
{
    int nErr = PLUGIN_OK;
    double dEast, dWest;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLimits] Called." << std::endl;
    m_sLogFile.flush();
#endif
    return nErr;

}

#pragma mark - Slew

int RST::startSlewTo(double dRa, double dDec)
{
    int nErr = PLUGIN_OK;
    bool bAligned;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [startSlewTo] Called." << std::endl;
    m_sLogFile.flush();
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [slewTargetRA_DecEpochNow] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = sendCommand(":MS#", sResp, 0);
    // nErr = sendCommand(":MA#", sResp, 0);   // AltAz

    timer.Reset();
    return nErr;

}

int RST::getNbSlewRates()
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getNbSlewRates] Called : PLUGIN_NB_SLEW_SPEEDS = " << PLUGIN_NB_SLEW_SPEEDS << std::endl;
    m_sLogFile.flush();
#endif
    return PLUGIN_NB_SLEW_SPEEDS;
}

// returns "Slew", "ViewVel4", "ViewVel3", "ViewVel2", "ViewVel1"
int RST::getRateName(int nZeroBasedIndex, std::string &sOut)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRateName] Called." << std::endl;
    m_sLogFile.flush();
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSpeed] Called." << std::endl;
    m_sLogFile.flush();
#endif


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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSpeed] Called." << std::endl;
    m_sLogFile.flush();
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setGuideSpeed] Called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << ":Cu0=" << std::fixed << std::setprecision(1) << dSpeed << "#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    return nErr;
}

#pragma mark - TODO : fix response parsing
int RST::getGuideSpeed(double &dSpeed)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getGuideSpeed] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = sendCommand(":CU0#", sResp);
    if(nErr)
        return nErr;
    
    // dSpeed = atof(szResp+5);

    return nErr;
}

int RST::isSlewToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isSlewToComplete] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bComplete = false;

    if(timer.GetElapsedSeconds()<2) {
        // we're checking for comletion to quickly, assume it's moving for now
        return nErr;
    }

    nErr = sendCommand(":CL#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isSlewToComplete] error " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isSlewToComplete] sResp : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    if(sResp.at(3)=='0')
        bComplete = true;
    return nErr;
}

int RST::gotoPark(double dRa, double dDec)
{
    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [gotoPark] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = startSlewTo(dRa, dDec);
    return nErr;
}


int RST::getAtPark(bool &bParked)
{
    int nErr = PLUGIN_OK;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAtPark] Called." << std::endl;
    m_sLogFile.flush();
#endif
    bParked = false;

    return nErr;
}

int RST::unPark()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [unPark] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = sendCommand(":CtA#", sResp, 0);
    return nErr;
}


int RST::Abort()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Abort] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = sendCommand(":Q#", sResp, 0);
    nErr |= sendCommand(":CtL#", sResp, 0);
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTime] Called." << std::endl;
    m_sLogFile.flush();
#endif

    m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);

    ssTmp << ":SL" << std::setfill('0') << std::setw(2) << h << ":" << std::setfill('0') << std::setw(2) << min << ":" << std::setfill('0') << std::setw(2) << int(sec) << "#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncDate] Called." << std::endl;
    m_sLogFile.flush();
#endif

    m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);
    // yy is actually yyyy, need conversion to yy, 2017 -> 17
    yy = yy - (int(yy / 1000) * 1000);

    ssTmp << ":SC" << std::setfill('0') << std::setw(2) << mm << "/" << std::setfill('0') << std::setw(2) << dd << "/" << std::setfill('0') << std::setw(2) << yy << "#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    getLocalDate(m_sDate);
    return nErr;
}

void RST::setSyncDateTimeOnConnect(bool bSync)
{
    m_bSyncTimeAndDateOnConnect = bSync;
}
#pragma mark - Check value format are correct
int RST::setSiteLongitude(const std::string sLongitude)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteLongitude] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // :SgsDDD*MM'SS#
    ssTmp << ":Sg" << sLongitude << "#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return nErr;
}

int RST::setSiteLatitude(const std::string sLatitude)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteLatitude] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // :StsDD*MM'SS#
    ssTmp << ":St" << sLatitude << "#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return nErr;
}

int RST::setSiteTimezone(const std::string sTimezone)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteTimezone] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // :SGsHH#
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return nErr;
}

#pragma mark - TODO : Parse response
int RST::getSiteLongitude(std::string &sLongitude)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteLongitude] Called." << std::endl;
    m_sLogFile.flush();
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteLatitude] Called." << std::endl;
    m_sLogFile.flush();
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteTZ] Called." << std::endl;
    m_sLogFile.flush();
#endif

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
    std::string sTimeZone;
    std::stringstream ssTmp;

    std::stringstream  ssHH, ssMM;
    char cSignLong;
    char cSignLat;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] Called." << std::endl;
    m_sLogFile.flush();
#endif


#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dLongitude : " << std::fixed << std::setprecision(5) << dLongitude << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dLatitute : " << std::fixed << std::setprecision(5) << dLatitute << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dTimeZone : " << std::fixed << std::setprecision(2) << dTimeZone << std::endl;
    m_sLogFile.flush();
#endif

    convertDecDegToDDMMSS(dLongitude, sLong, cSignLong);
    convertDecDegToDDMMSS(dLatitute, sLat, cSignLat);
    
    ssHH << std::setfill('0') << std::setw(2) << int(std::fabs(dTimeZone));
    ssMM << std::setfill('0') << std::setw(2) << int((std::fabs(dTimeZone) - int(std::fabs(dTimeZone)))) * 100;

    // longitude    :SgsDDD*MM'SS #    ->   :Sg-127*30'20#
    // latitude     :StsDD*MM'SS #     ->   :St+37*20'30#

    sTimeZone = std::to_string(-dTimeZone);

    ssTmp<< cSignLong << sLong;
    sLong.assign(ssTmp.str());

    std::stringstream().swap(ssTmp);
    ssTmp<< cSignLat << sLat;
    sLat.assign(ssTmp.str());

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] sLong      : " << sLong << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] sLat       : " << sLat<< std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] ssTimeZone : " << sTimeZone << std::endl;
    m_sLogFile.flush();
#endif

    nErr = setSiteLongitude(sLong);
    nErr |= setSiteLatitude(sLat);
    nErr |= setSiteTimezone(sTimeZone);

    return nErr;
}

int RST::getSiteData(std::string &sLongitude, std::string &sLatitude, std::string &sTimeZone)
{
    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteData] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = getSiteLongitude(sLongitude);
    nErr |= getSiteLatitude(sLatitude);
    nErr |= getSiteTZ(sTimeZone);
    return nErr;
}

void RST::setSyncLocationConnect(bool bSync)
{
    m_bSyncLocationOnConnect = bSync;
}

#pragma mark  - Time and Date

#pragma mark - Parse result
int RST::getLocalTime(std::string &sTime)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLocalTime] Called." << std::endl;
    m_sLogFile.flush();
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLocalDate] Called." << std::endl;
    m_sLogFile.flush();
#endif

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
    double dNewDeg;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDecDegToDDMMSS] Called." << std::endl;
    m_sLogFile.flush();
#endif

    sResult.clear();
    // convert dDeg decimal value to sDD:MM:SS
    dNewDeg = std::fabs(dDeg);
    cSign = dDeg>=0?'+':'-';
    DD = int(dNewDeg);
    mm = dNewDeg - DD;
    MM = int(mm*60);
    ss = (mm*60) - MM;
    SS = int(std::ceil(ss*60));

    ssTmp << DD << "*" << std::setfill('0') << std::setw(2) << MM << "'" << std::setfill('0') << std::setw(2) << SS;
    sResult.assign(ssTmp.str());
}

void RST::convertDecDegToDDMMSS_ForDecl(double dDeg, std::string &sResult, char &cSign)
{
    int DD, MM;
    double mm, ss, SS;
    double dNewDeg;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDecDegToDDMMSS_ForDecl] Called." << std::endl;
    m_sLogFile.flush();
#endif

    sResult.clear();
    // convert dDeg decimal value to sDD:MM:SS
    dNewDeg = std::fabs(dDeg);
    cSign = dDeg>=0?'+':'-';
    DD = int(dNewDeg);
    mm = dNewDeg - DD;
    MM = int(mm*60);
    ss = (mm*60) - MM;
    SS = ss*60;

    ssTmp << DD << "*" << std::setfill('0') << std::setw(2) << MM << ":" << std::setfill('0') << std::setw(4) << std::fixed << std::setprecision(1)<< SS;
    sResult.assign(ssTmp.str());
}

int RST::convertDDMMSSToDecDeg(const std::string sStrDeg, double &dDecDeg)
{
    int nErr = PLUGIN_OK;
    std::vector<std::string> vFieldsData;
    std::string newDec;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDDMMSSToDecDeg] Called." << std::endl;
    m_sLogFile.flush();
#endif

    dDecDeg = 0;
    // dec is in a weird format.
    newDec.assign(sStrDeg);

    std::replace(newDec.begin(), newDec.end(), '*', ':' );
    std::replace(newDec.begin(), newDec.end(), '\'', ':' );
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDDMMSSToDecDeg] newDec = " << newDec << std::endl;
    m_sLogFile.flush();
#endif

    nErr = parseFields(newDec, vFieldsData, ':');
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDDMMSSToDecDeg] parseFields error " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    if(vFieldsData.size() >= 3) {
        try {
            dDecDeg = atof(vFieldsData[0].c_str()) + atof(vFieldsData[1].c_str())/60 + atof(vFieldsData[1].c_str())/3600;
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDDMMSSToDecDeg] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
            return ERR_PARSE;
        }

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertRaToHHMMSSt] Called." << std::endl;
    m_sLogFile.flush();
#endif

    sResult.clear();
    // convert Ra value to HH:MM:SS.T before passing them to the RST
    HH = int(dRa);
    hh = dRa - HH;
    MM = int(hh*60);
    mm = (hh*60) - MM;
    SSt = mm * 60;

    ssTmp << std::setfill('0') << std::setw(2) << HH << ":" << std::setfill('0') << std::setw(2) << MM << ":" << std::setfill('0') << std::setw(4) << std::fixed << std::setprecision(1) << SSt;
    sResult.assign(ssTmp.str());
}


int RST::convertHHMMSStToRa(const std::string szStrRa, double &dRa)
{
    int nErr = PLUGIN_OK;
    std::vector<std::string> vFieldsData;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertHHMMSStToRa] Called." << std::endl;
    m_sLogFile.flush();
#endif

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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [parseFields] Called." << std::endl;
    m_sLogFile.flush();
#endif

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
void RST::log(std::string sLogEntry)
{
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [log] " << sLogEntry << std::endl;
    m_sLogFile.flush();

}

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
