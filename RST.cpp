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
    m_bHomeOnUnpark = false;

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
    std::string sResp;

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

    // usb mode on
    sendCommand(":AU#", sResp, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
    // request protocol Rainbow
    sendCommand(":AR#", sResp, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

    if(m_bSyncTimeAndDateOnConnect) {
        syncTime();
        syncDate();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
    }

    if(m_bSyncLocationOnConnect) {
        setSiteData(m_pTsx->longitude(),
                    m_pTsx->latitude(),
                    m_pTsx->timeZone());
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
    }

    sendCommand(":GA#", sResp);
    sendCommand(":GZ#", sResp);
    sendCommand(":GR#", sResp);
    sendCommand(":GD#", sResp);

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
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec]  dRa : " << std::fixed << std::setprecision(12) << dRa << std::endl;
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
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] dDec : " << std::fixed << std::setprecision(12) << dDec << std::endl;
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

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] Ra  : " << std::fixed << std::setprecision(8) << dRa << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] Dec : " << std::fixed << std::setprecision(8) << dDec << std::endl;
    m_sLogFile.flush();
#endif

    // convert Ra value to HH:MM:SS.S before passing them to the RST
    convertRaToHHMMSSt(dRa, sTemp);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] Converted Ra : " << sTemp << std::endl;
    m_sLogFile.flush();
#endif
    // set target Ra
    ssTmp<<":Sr"<<sTemp<<"#";
    nErr = sendCommand(ssTmp.str(), sResp, 100); // need a fast error as this command doesn't follow the usual format and doesn't end with #
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
    if(sResp.size() && sResp.at(0)=='1') {
        nErr = PLUGIN_OK;
    }
    else if(nErr)
        return nErr;


    // convert target dec to sDD*MM:SS.S
    convertDecDegToDDMMSS_ForDecl(dDec, sTemp);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget] Converted Dec : " <<sTemp << std::endl;
    m_sLogFile.flush();
#endif
    std::stringstream().swap(ssTmp);
    // set target Dec
    ssTmp<<":Sd"<<sTemp<<"#";
    nErr = sendCommand(ssTmp.str(), sResp, 100); // need a fast error as this command doesn't follow the usual format and doesn't end with #
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
    if(sResp.size() && sResp.at(0)=='1')
        nErr = PLUGIN_OK;

    return nErr;
}

int RST::setTargetAltAz(double dAlt, double dAz)
{
    int nErr;
    std::stringstream ssTmp;
    std::string sResp;
    std::string sTemp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTargetAltAz] Az  : " << std::fixed << std::setprecision(8) << dAz << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTargetAltAz] Alt : " << std::fixed << std::setprecision(8) << dAlt << std::endl;
    m_sLogFile.flush();
#endif

    // convert Az value to DDD*MM:SS.S
    convertDecAzToDDMMSSs(dAz, sTemp);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTargetAltAz] szTemp(Az)  : " << sTemp << std::endl;
    m_sLogFile.flush();
#endif
    // set target Az
    ssTmp<<":Sz"<<sTemp<<"#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
    if(nErr)
        return nErr;


    // convert Alt value sDD*MM:SS.S
    convertDecDegToDDMMSS_ForDecl(dAlt, sTemp);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTargetAltAz]  szTemp(Alt)  : " <<sTemp << std::endl;
    m_sLogFile.flush();
#endif
    // set target Alt
    std::stringstream().swap(ssTmp);
    ssTmp<<":Sa"<<sTemp<<"#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
    if(nErr)
        return nErr;

    return nErr;
}

#pragma mark - Sync and Cal
int RST::syncTo(double dRa, double dDec)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssTmp;
    std::string sResp;
    char cSign;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo]  Ra Hours   : " << std::fixed << std::setprecision(5) << dRa << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo]  Ra Degrees : " << std::fixed << std::setprecision(5) << dRa*15.0 << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo]  Dec        : " << std::fixed << std::setprecision(5) << dDec << std::endl;
    m_sLogFile.flush();
#endif


    if(dDec <0) {
        cSign = '-';
        dDec = -dDec;
    } else {
        cSign = '+';
    }
    ssTmp << ":Ck" << std::setfill('0') << std::setw(7) << std::fixed << std::setprecision(3) << dRa*15.0 << cSign << std::setfill('0') << std::setw(6)<< std::fixed << std::setprecision(3) << dDec << "#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

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
            break;
        case '1' :  // Solar
            dTrackRaArcSecPerHr = 0;
            dTrackDecArcSecPerHr = 0;
            break;
        case '2' :  // Lunar
            dTrackRaArcSecPerHr = 0;
            dTrackDecArcSecPerHr = 0;
            break;
        case '3' :  //  Guide
            dTrackRaArcSecPerHr = 0;
            dTrackDecArcSecPerHr = 0;
            break;
        default:
            dTrackRaArcSecPerHr = 0;
            dTrackDecArcSecPerHr = 0;
            break;
    }

    isTrackingOn(bTrackingOn);
    if(!bTrackingOn)
        dTrackRaArcSecPerHr = 15.0410681; // Convention to say tracking is off - see TSX documentation

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

    nErr = slewTargetRA_DecEpochNow();
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

    nErr = sendCommand(":MS#", sResp, 100);
    if(sResp.size() && nErr)
        return nErr;

    nErr = PLUGIN_OK;

    if(sResp.size()>=4) {
        if(sResp.at(3) == 'L') {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [slewTargetRA_DecEpochNow] Limit error ? => " << sResp << std::endl;
            m_sLogFile.flush();
#endif
            nErr = ERR_MKS_SLEW_PAST_LIMIT;
        }
    }
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

int RST::getSpeed(const int nSpeedId, int &nSpeed)
{
    int nErr = PLUGIN_OK;
    std::stringstream ssTmp;
    std::string sResp;
    std::vector<std::string> vFieldsData;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSpeed] Called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << ":CU" << nSpeedId << "#";
    nErr = sendCommand(ssTmp.str(), sResp);
    if(nErr)
        return nErr;

    parseFields(sResp, vFieldsData, '=');
    if(vFieldsData.size() >1) {
        try {
            nSpeed = std::stoi(vFieldsData[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSpeed] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
        }
    }
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

int RST::getGuideSpeed(double &dSpeed)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::vector<std::string> vFieldsData;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getGuideSpeed] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = sendCommand(":CU0#", sResp);
    if(nErr)
        return nErr;

    parseFields(sResp, vFieldsData, '=');
    if(vFieldsData.size() >1) {
        try {
            dSpeed = std::stod(vFieldsData[1]);
        }
        catch(const std::exception& e) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setGuideSpeed] conversion exception : " << e.what() << std::endl;
            m_sLogFile.flush();
#endif
        }
    }
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


int RST::gotoPark(double dAlt, double dAz)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [gotoPark] Called." << std::endl;
    m_sLogFile.flush();
#endif

    // set target
    nErr = setTargetAltAz(dAlt, dAz);
    if(nErr)
        return nErr;

    // goto in Az mode
    nErr = sendCommand(":MA#", sResp, 0);   // AltAz

    return nErr;
}


int RST::getAtPark(bool &bParked)
{
    int nErr = PLUGIN_OK;
    bool bTracking;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAtPark] Called." << std::endl;
    m_sLogFile.flush();
#endif
    isTrackingOn(bTracking);

    bParked = bTracking?false:true;

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
    m_bUnparking = true;

    nErr = homeMount();
    return nErr;
}

int RST::isUnparkDone(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    bool bIsHomed = false;
    bool bAtPArk;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] Called." << std::endl;
    m_sLogFile.flush();
#endif

    bComplete = false;
    if(!m_bUnparking) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] not unparking, checking at park state " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        nErr = getAtPark(bAtPArk);
        if(bAtPArk)
            bComplete = true;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] bAtPArk   " << (bAtPArk?"Yes":"No") << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] bComplete " << (bComplete?"Yes":"No") << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] Chcking if homing is done" << nErr << std::endl;
    m_sLogFile.flush();
#endif
    nErr = isHomingDone(bIsHomed);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] error " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] bIsHomed   " << (bIsHomed?"Yes":"No") << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] bComplete " << (bComplete?"Yes":"No") << std::endl;
    m_sLogFile.flush();
#endif

    if(!bIsHomed)
        return nErr;

    // unparking and homing is done, enable tracking a sidereal rate
    m_bUnparking = false;
    bComplete = true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] bIsHomed   " << (bIsHomed?"Yes":"No") << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isUnparkDone] bComplete " << (bComplete?"Yes":"No") << std::endl;
    m_sLogFile.flush();
#endif

    nErr = sendCommand(":CtA#", sResp); // unpark, tracking on
    std::this_thread::sleep_for(std::chrono::milliseconds(250)); // need to give time to the mount to process the command
    nErr |= sendCommand(":CtR#", sResp); // set tracking to sidereal
    std::this_thread::sleep_for(std::chrono::milliseconds(250)); // need to give time to the mount to process the command

    return nErr;
}


int RST::homeMount()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [homeMount] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = sendCommand(":Ch#", sResp, 0);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [homeMount] error " << nErr << std::endl;
        m_sLogFile.flush();
#endif
    }

    return nErr;
}


int RST::isHomingDone(bool &bIsHomed)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isHomingDone] Called." << std::endl;
    m_sLogFile.flush();
#endif
    bIsHomed = false;

    nErr = sendCommand(":AH#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isHomingDone] error " << nErr << std::endl;
        m_sLogFile.flush();
#endif
    }

    if(sResp.size() >= 3 && sResp.at(3) == '0') {
            bIsHomed = true;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
       m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isHomingDone] bIsHomed : " << (bIsHomed?"Yes":"No") <<  std::endl;
       m_sLogFile.flush();
#endif

    return nErr;
}


int RST::isTrackingOn(bool &bTrakOn)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isTrackingOn] Called." << std::endl;
    m_sLogFile.flush();
#endif
    bTrakOn = false;
    nErr = sendCommand(":AT#", sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isTrackingOn] error " << nErr << std::endl;
        m_sLogFile.flush();
#endif
    }

    if(sResp.size() >= 3 && sResp.at(3) == '1')
        bTrakOn = true;

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
    m_bUnparking = false;
    
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
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
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
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
    getLocalDate(m_sDate);
    return nErr;
}

void RST::setSyncDateTimeOnConnect(bool bSync)
{
    m_bSyncTimeAndDateOnConnect = bSync;
}

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
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

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
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

    return nErr;
}

int RST::setSiteTimezone(const std::string sTimezone)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;
    std::string sCurrentTimeZone;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteTimezone] Called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = sendCommand(":GG#", sResp);
    if(!nErr) {
        sCurrentTimeZone.assign(sResp.substr(3));
    }
    else {
        sCurrentTimeZone.clear();
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteTimezone] sCurrentTimeZone = '" << sCurrentTimeZone << "'" << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteTimezone] sTimezone        = '" << sTimezone << "'" << std::endl;
    m_sLogFile.flush();
#endif

    if(sCurrentTimeZone == sTimezone)
        return nErr;

    ssTmp << ":SG" << sTimezone << "#";
    nErr = sendCommand(ssTmp.str(), sResp, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
    return nErr;
}

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
        sLongitude.assign(sResp.substr(3));
    }
    return nErr;
}

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
        sLatitude.assign(sResp.substr(3));
    }

    return nErr;
}

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
        sTimeZone.assign(sResp.substr(3));
        if(sTimeZone.size() && sTimeZone.at(0) == '-') {
            sTimeZone[0] = '+';
        }
        else {
            sTimeZone[0] = '-';
        }
    }

    return nErr;
}

int RST::setSiteData(double dLongitude, double dLatitute, double dTimeZone)
{
    int nErr = PLUGIN_OK;
    std::string sLong;
    std::string sLat;
    std::stringstream ssTimeZone;
    std::stringstream  ssHH, ssMM;
    char cSign;
    double dTimeZoneNew;

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

    convertDecDegToDDMMSS(dLongitude, sLong);
    convertDecDegToDDMMSS(dLatitute, sLat);

    dTimeZoneNew = -dTimeZone;
    cSign = dTimeZoneNew>=0?'+':'-';
    dTimeZoneNew=std::fabs(dTimeZone);

    ssTimeZone << cSign << std::setfill('0') << std::setw(2) << dTimeZoneNew;

    sLong.assign(sLong);

    sLat.assign(sLat);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] sLong      : " << sLong << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] sLat       : " << sLat<< std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] ssTimeZone : " << ssTimeZone.str() << std::endl;
    m_sLogFile.flush();
#endif

    nErr = setSiteLongitude(sLong);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
    nErr |= setSiteLatitude(sLat);
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command
    nErr |= setSiteTimezone(ssTimeZone.str());
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // need to give time to the mount to process the command

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
    sTime.assign(sResp.substr(3));
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
    sDate.assign(sResp.substr(3));
    return nErr;
}


void RST::convertDecDegToDDMMSS(double dDeg, std::string &sResult)
{
    int DD, MM, SS;
    double mm, ss;
    double dNewDeg;
    std::stringstream ssTmp;
    char cSign;

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
    SS = int(std::roundf(ss*60));

    ssTmp << cSign << DD << "*" << std::setfill('0') << std::setw(2) << MM << "'" << std::setfill('0') << std::setw(2) << SS;
    sResult.assign(ssTmp.str());
}

void RST::convertDecAzToDDMMSSs(double dDeg, std::string &sResult)
{
    int DD, MM;
    double mm, ss, SS;
    double dNewDeg;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDecAzToDDMMSSs] Called." << std::endl;
    m_sLogFile.flush();
#endif

    sResult.clear();
    // convert dDeg decimal value to DDD*MM:SS.S 
    dNewDeg = std::fabs(dDeg);
    DD = int(dNewDeg);
    mm = dNewDeg - DD;
    MM = int(mm*60);
    ss = (mm*60) - MM;
    SS = ss*60;

    ssTmp << std::setfill('0') << std::setw(3) << DD << "*" << std::setfill('0') << std::setw(2) << MM << "'" << std::setfill('0') << std::setw(4) << std::fixed << std::setprecision(1) << SS;
    sResult.assign(ssTmp.str());
}

void RST::convertDecDegToDDMMSS_ForDecl(double dDeg, std::string &sResult)
{
    int DD, MM;
    double mm, ss, SS;
    double dNewDeg;
    char cSign;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDecDegToDDMMSS_ForDecl] Called." << std::endl;
    m_sLogFile.flush();
#endif

    sResult.clear();
    // convert dDeg decimal value to sDD*MM:SS.S
    dNewDeg = std::fabs(dDeg);
    cSign = dDeg>=0?'+':'-';
    DD = int(dNewDeg);
    mm = dNewDeg - DD;
    MM = int(mm*60);
    ss = (mm*60) - MM;
    SS = ss*60;

    ssTmp << cSign << std::setfill('0') << std::setw(2) << DD << "*" << std::setfill('0') << std::setw(2) << MM << ":" << std::setfill('0') << std::setw(4) << std::fixed << std::setprecision(1)<< SS;
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
            dDecDeg = std::stod(vFieldsData[0]);
            if(dDecDeg <0) {
                dDecDeg = dDecDeg - std::stod(vFieldsData[1])/60.0 - std::stod(vFieldsData[2])/3600.0;
            }
            else {
                dDecDeg = dDecDeg + std::stod(vFieldsData[1])/60.0 + std::stod(vFieldsData[2])/3600.0;
            }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDDMMSSToDecDeg] dDecDeg = " << std::fixed << std::setprecision(12) << dDecDeg << std::endl;
            m_sLogFile.flush();
#endif
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
    // convert Ra value to HH:MM:SS.S
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
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertHHMMSStToRa] szStrRa = '" <<  szStrRa << "'" << std::endl;
    m_sLogFile.flush();
#endif

    dRa = 0;

    nErr = parseFields(szStrRa, vFieldsData, ':');
    if(nErr)
        return nErr;

    if(vFieldsData.size() >= 3) {
        try {
            dRa = std::stod(vFieldsData[0]) + std::stod(vFieldsData[1])/60.0 + std::stod(vFieldsData[2])/3600.0;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [convertDDMMSSToDecDeg] dRa = " << std::fixed << std::setprecision(12) << dRa << std::endl;
            m_sLogFile.flush();
#endif
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
