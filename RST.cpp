#include "RST.h"

// Constructor for RST
RST::RST()
{

	m_bIsConnected = false;

    m_bDebugLog = true;
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
	Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [RST::RST] Version %3.2f build 2020_09_05_1140.\n", timestamp, DRIVER_VERSION);
	fprintf(Logfile, "[%s] RST New Constructor Called\n", timestamp);
    fflush(Logfile);
#endif

}


RST::~RST(void)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] RST Destructor Called\n", timestamp );
    fflush(Logfile);
#endif
#ifdef PLUGIN_DEBUG
    // Close LogFile
    if (Logfile) fclose(Logfile);
#endif
}

int RST::Connect(char *pszPort)
{
    bool bIsParked;
    bool bIsAligned;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] RST::Connect Called %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif

    // 115.2K 8N1
    if(m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;


#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] RST::Connect m_mountType %d\n", timestamp, m_mountType);
    fflush(Logfile);
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
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] RST::Disconnect Called\n", timestamp);
    fflush(Logfile);
#endif
	if (m_bIsConnected) {
        if(m_pSerx){
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] RST::Disconnect closing serial port\n", timestamp);
            fflush(Logfile);
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

int RST::RSTSendCommand(const char *pszCmd, char *pszResult, unsigned int nResultMaxLen)
{
    int nErr = PLUGIN_OK;
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    unsigned long ulBytesWrite;

    m_pSerx->purgeTxRx();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [RST::RSTSendCommand] Sending %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;
    // read response

    if(pszResult) {
        nErr = RSTreadResponse(szResp, SERIAL_BUFFER_SIZE);
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [RST::RSTSendCommand] error %d reading response : %s\n", timestamp, nErr, szResp);
            fflush(Logfile);
#endif
            return nErr;
        }
        strncpy(pszResult, (const char *)szResp, nResultMaxLen);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [RST::RSTSendCommand] got response : '%s'\n", timestamp, szResp);
        fflush(Logfile);
#endif
    }
    return nErr;
}


int RST::RSTreadResponse(unsigned char *pszRespBuffer, unsigned int nBufferLen)
{
    int nErr = PLUGIN_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    unsigned char *pszBufPtr;

    memset(pszRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = pszRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
        if(nErr) {
            return nErr;
        }

 #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [RST::readResponse] *pszBufPtr = 0x%02X\n", timestamp, *pszBufPtr);
        fflush(Logfile);
#endif

        if (ulBytesRead !=1) {// timeout
            nErr = PLUGIN_BAD_CMD_RESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;

    } while (*pszBufPtr++ != '#' && ulTotalBytesRead < nBufferLen );

    if(ulTotalBytesRead && *(pszBufPtr-1) == '#')
        *(pszBufPtr-1) = 0; //remove the # to zero terminate the string

    return nErr;
}


#pragma mark - dome controller informations

int RST::getFirmwareVersion(std::string &sFirmware)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = RSTSendCommand(":AV#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    sFirmware.assign(szResp+3);
    m_sFirmwareVersion.assign(szResp+3);
    return nErr;
}

#pragma mark - Mount Coordinates
void RST::setMountMode(MountTypeInterface::Type mountType)
{
    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [RST::setMountMode] mountType = %d\n", timestamp, mountType);
        fflush(Logfile);
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
    char szResp[SERIAL_BUFFER_SIZE];

    // get RA
    nErr = RSTSendCommand(":GR#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [RST::getRaAndDec] szResp = %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    nErr = convertHHMMSStToRa(szResp+3, dRa);
    if(nErr)
        return nErr;

    // get DEC
    nErr = RSTSendCommand(":GD#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    nErr = convertDDMMSSToDecDeg(szResp+3, dDec);

    return nErr;
}

int RST::setTarget(double dRa, double dDec)
{
    int nErr;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    char szTemp[SERIAL_BUFFER_SIZE];
    char cSign;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [RST::setTarget] Ra : %f\n", timestamp, dRa);
    fprintf(Logfile, "[%s] [RST::setTarget] Dec : %f\n", timestamp, dDec);
    fflush(Logfile);
#endif

    // convert Ra value to HH:MM:SS.T before passing them to the RST
    convertRaToHHMMSSt(dRa, szTemp, SERIAL_BUFFER_SIZE);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [RST::setTarget] Ra : %s\n", timestamp, szTemp);
    fflush(Logfile);
#endif
    // set target Ra
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":Sr%s#", szTemp);
    nErr = RSTSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    convertDecDegToDDMMSS(dDec, szTemp, cSign, SERIAL_BUFFER_SIZE);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [RST::setTarget] Dec : %c%s\n", timestamp, cSign, szTemp);
    fflush(Logfile);
#endif
    // set target dec
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":Sd%c%s#", cSign,szTemp);
    nErr = RSTSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

#pragma mark - Sync and Cal
int RST::syncTo(double dRa, double dDec)
{
    int nErr = PLUGIN_OK;
    bool bAligned;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    char cSign;
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [RST::syncTo] Ra : %f\n", timestamp, dRa);
    fprintf(Logfile, "[%s] [RST::syncTo] Dec : %f\n", timestamp, dDec);
    fflush(Logfile);
#endif
    if(dDec <0) {
        cSign = '-';
        dDec = -dDec;
    } else {
        cSign = '+';
    }
    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":Ck%03.3f%c%02.3f#", dRa, cSign, dDec);
    nErr = RSTSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

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
    char szResp[SERIAL_BUFFER_SIZE];

    if(!bTrackingOn) { // stop tracking
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [RST::setTrackingRates] setting to stopped\n", timestamp);
        fflush(Logfile);
#endif
        nErr = RSTSendCommand(":CtL#", szResp, SERIAL_BUFFER_SIZE); // tracking off
    }
    else if(bTrackingOn && bIgnoreRates) { // sidereal
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [RST::setTrackingRates] setting to Sidereal\n", timestamp);
        fflush(Logfile);
#endif
        nErr = RSTSendCommand(":CT0#", szResp, SERIAL_BUFFER_SIZE);
    }
    else { // lunar, solar, ..
    }
    return nErr;
}

int RST::getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerHr, double &dTrackDecArcSecPerHr)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand(":Ct?#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    switch(szResp[3]) {
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

int RST::getSoftLimitEastAngle(double &dAngle)
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand("!NGle;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    dAngle = atof(szResp);

    return nErr;
}

int RST::getSoftLimitWestAngle(double &dAngle)
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand("!NGlw;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    dAngle = atof(szResp);

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
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand(":MS#", szResp, SERIAL_BUFFER_SIZE); // Equatorial
    // nErr = RSTSendCommand(":MA#", szResp, SERIAL_BUFFER_SIZE);   // AltAz

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

    m_nOpenLoopDir = Dir;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [RST::startOpenSlew] setting to Dir %d\n", timestamp, Dir);
    fprintf(Logfile, "[%s] [RST::startOpenSlew] Setting rate to %d\n", timestamp, nRate);
    fflush(Logfile);
#endif

    // select rate
    m_nOpenLoopDir = Dir;
    switch(nRate) {
        case 0:
            nErr = RSTSendCommand(":RG#", NULL, SERIAL_BUFFER_SIZE);
            break;

        case 1:
            nErr = RSTSendCommand(":RC#", NULL, SERIAL_BUFFER_SIZE);
            break;

        case 2:
            nErr = RSTSendCommand(":RM#", NULL, SERIAL_BUFFER_SIZE);
            break;

        case 3:
            nErr = RSTSendCommand(":RS#", NULL, SERIAL_BUFFER_SIZE);
            break;

        default :
            return COMMAND_FAILED;
            break;
    }
    
    // figure out direction
    switch(Dir){
        case MountDriverInterface::MD_NORTH:
            nErr = RSTSendCommand(":Mn#", NULL, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_SOUTH:
            nErr = RSTSendCommand(":Ms#", NULL, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_EAST:
            nErr = RSTSendCommand(":Me#", NULL, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_WEST:
            nErr = RSTSendCommand(":Mw#", NULL, SERIAL_BUFFER_SIZE);
            break;
    }

    return nErr;
}

int RST::stopOpenLoopMove()
{
    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [RST::stopOpenLoopMove] Dir was %d\n", timestamp, m_nOpenLoopDir);
    fflush(Logfile);
#endif

    switch(m_nOpenLoopDir){
        case MountDriverInterface::MD_NORTH:
            nErr = RSTSendCommand(":Qn#", NULL, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_SOUTH:
            nErr = RSTSendCommand(":Qs#", NULL, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_EAST:
            nErr = RSTSendCommand(":Qe#", NULL, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_WEST:
            nErr = RSTSendCommand(":Qw#", NULL, SERIAL_BUFFER_SIZE);
            break;
    }

    return nErr;
}


int RST::setMaxSpeed(const int nSpeed)
{
    int nErr = PLUGIN_OK;
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":Cu3=%04d#;", nSpeed);
    nErr = RSTSendCommand(szCmd, NULL, SERIAL_BUFFER_SIZE);
    return nErr;
}

int RST::getMaxSpeed(int &nSpeed)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand(":CU3#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
    nSpeed = atoi(szResp+5);
    
    return nErr;
}

int RST::setFindSpeed(const int nSpeed)
{
    int nErr = PLUGIN_OK;
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":Cu2=%04d#;", nSpeed);
    nErr = RSTSendCommand(szCmd, NULL, SERIAL_BUFFER_SIZE);
    return nErr;
}

int RST::getFindSpeed(int &nSpeed)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand(":CU2#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
    nSpeed = atoi(szResp+5);

    return nErr;
}

int RST::setCenteringSpeed(const int nSpeed)
{
    int nErr = PLUGIN_OK;
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":Cu1=%04d#;", nSpeed);
    nErr = RSTSendCommand(szCmd, NULL, SERIAL_BUFFER_SIZE);
    return nErr;
}

int RST::getCenteringSpeed(int &nSpeed)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand(":CU21#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
    nSpeed = atoi(szResp+5);

    return nErr;
}

int RST::setGuideSpeed(const int nSpeed)
{
    int nErr = PLUGIN_OK;
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":Cu0=%04d#;", nSpeed);
    nErr = RSTSendCommand(szCmd, NULL, SERIAL_BUFFER_SIZE);
    return nErr;
}

int RST::getGuideSpeed(int &nSpeed)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand(":CU0#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
    nSpeed = atoi(szResp+5);

    return nErr;
}


int RST::isSlewToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nPrecentRemaining;

    bComplete = false;

    if(timer.GetElapsedSeconds()<2) {
        // we're checking for comletion to quickly, assume it's moving for now
        return nErr;
    }

    nErr = RSTSendCommand("!GGgr;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [RST::isSlewToComplete] szResp : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    // remove the %
    szResp[strlen(szResp) -1 ] = 0;
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [RST::isSlewToComplete] szResp : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    nPrecentRemaining = atoi(szResp);
    if(nPrecentRemaining == 0)
        bComplete = true;

    return nErr;
}

int RST::gotoPark(double dRa, double dDec)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    // set park position ?
    // or goto ?
    // goto park
    nErr = RSTSendCommand("!GTop;", szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int RST::markParkPosition()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand("!AMpp;", szResp, SERIAL_BUFFER_SIZE);

    return nErr;

}

int RST::getAtPark(bool &bParked)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bParked = false;
    nErr = RSTSendCommand("!AGak;", szResp, SERIAL_BUFFER_SIZE);
    if(strncmp(szResp,"Yes",SERIAL_BUFFER_SIZE) == 0) {
        bParked = true;
    }
    return nErr;
}

int RST::unPark()
{
    int nErr = PLUGIN_OK;
    bool bAligned;

    return nErr;
}


int RST::Abort()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand(":Q#", szResp, SERIAL_BUFFER_SIZE);
    nErr |= RSTSendCommand(":CtL#", szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}

#pragma mark - time and site methods
int RST::syncTime()
{
    int nErr = PLUGIN_OK;
    int yy, mm, dd, h, min, dst;
    double sec;

    char szCmd[SERIAL_BUFFER_SIZE];

    m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SL%02d:%02d:%02d#",  h, min, int(sec));
    nErr = RSTSendCommand(szCmd, NULL, SERIAL_BUFFER_SIZE);
    getStandardTime(m_sTime);

    return nErr;
}


int RST::syncDate()
{
    int nErr = PLUGIN_OK;
    int yy, mm, dd, h, min, dst;
    double sec;

    char szCmd[SERIAL_BUFFER_SIZE];

    m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);
    // yy is actually yyyy, need conversion to yy, 2017 -> 17
    yy = yy - (int(yy / 1000) * 1000);

    snprintf(szCmd, SERIAL_BUFFER_SIZE, ":SC%02d/%02d/%02d#", mm, dd, yy);
    nErr = RSTSendCommand(szCmd, NULL, SERIAL_BUFFER_SIZE);

    getStandardDate(m_sDate);
    return nErr;
}


int RST::setSiteLongitude(const char *szLongitude)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    // snprintf(szCmd, SERIAL_BUFFER_SIZE, "!SSo%d%s;",szLongitude);
    nErr = RSTSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int RST::setSiteLatitude(const char *szLatitude)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    // snprintf(szCmd, SERIAL_BUFFER_SIZE, ":StsDD*MM'SS#", nSiteNb, szLatitude);
    nErr = RSTSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int RST::setSiteTimezone(const char *szTimezone)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    // snprintf(szCmd, SERIAL_BUFFER_SIZE, "!SSz%d%s;", nSiteNb, szTimezone);
    nErr = RSTSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int RST::getSiteLongitude(std::string &sLongitude)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    // snprintf(szCmd, SERIAL_BUFFER_SIZE, "!SGo%d;", nSiteNb);
    nErr = RSTSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(!nErr) {
        sLongitude.assign(szResp);
    }
    return nErr;
}

int RST::getSiteLatitude(std::string &sLatitude)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    // snprintf(szCmd, SERIAL_BUFFER_SIZE, "!SGa%d;", nSiteNb);
    nErr = RSTSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(!nErr) {
        sLatitude.assign(szResp);
    }

    return nErr;
}

int RST::getSiteTZ(std::string &sTimeZone)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    // snprintf(szCmd, SERIAL_BUFFER_SIZE, "!SGz%d;", nSiteNb);
    nErr = RSTSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(!nErr) {
        sTimeZone.assign(szResp);
    }

    return nErr;
}

int RST::setSiteData(double dLongitude, double dLatitute, double dTimeZone)
{
    int nErr = PLUGIN_OK;
    char szLong[SERIAL_BUFFER_SIZE];
    char szLat[SERIAL_BUFFER_SIZE];
    char szTimeZone[SERIAL_BUFFER_SIZE];
    char szHH[3], szMM[3];
    char cSignLong;
    char cSignLat;

#if defined ATCS_DEBUG && ATCS_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::setSiteData] dLongitude : %f\n", timestamp, dLongitude);
    fprintf(Logfile, "[%s] [ATCS::setSiteData] dLatitute : %f\n", timestamp, dLatitute);
    fprintf(Logfile, "[%s] [ATCS::setSiteData] szTimeZone : %f\n", timestamp, dTimeZone);
    fflush(Logfile);
#endif

    convertDecDegToDDMMSS(dLongitude, szLong, cSignLong, SERIAL_BUFFER_SIZE);
    convertDecDegToDDMMSS(dLatitute, szLat, cSignLat, SERIAL_BUFFER_SIZE);
    snprintf(szHH,3,"%02d", int(fabs(dTimeZone)));
    snprintf(szMM,3,"%02d", int((fabs(dTimeZone) - int(fabs(dTimeZone)))) * 100);
    
    if(dTimeZone<0) {
        snprintf(szTimeZone, SERIAL_BUFFER_SIZE, "%s:%sW", szHH, szMM);
    }
    else if (dTimeZone>0) {
        snprintf(szTimeZone, SERIAL_BUFFER_SIZE, "%s:%sE", szHH, szMM);
    }
    else
        snprintf(szTimeZone, SERIAL_BUFFER_SIZE, "00:00");

    // Set the W/E
    if(dLongitude<0) {
        snprintf(szLong, SERIAL_BUFFER_SIZE, "%sW", szLong);
    }
    else {
        snprintf(szLong, SERIAL_BUFFER_SIZE, "%sE", szLong);
    }
    // convert signed latitude to N/S
    if(dLatitute>=0) {
        snprintf(szLat, SERIAL_BUFFER_SIZE, "%sN", szLat);
    }
    else {
        snprintf(szLat, SERIAL_BUFFER_SIZE, "%sS", szLat);
    }

#if defined ATCS_DEBUG && ATCS_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::setSiteData] szLong : %s\n", timestamp, szLong);
    fprintf(Logfile, "[%s] [ATCS::setSiteData] szLat : %s\n", timestamp, szLat);
    fprintf(Logfile, "[%s] [ATCS::setSiteData] szTimeZone : %s\n", timestamp, szTimeZone);
    fflush(Logfile);
#endif

    nErr = setSiteLongitude(szLong);
    nErr |= setSiteLatitude(szLat);
    nErr |= setSiteTimezone(szTimeZone);

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


int RST::getLocalTimeFormat(bool &b24h)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand("!TGlf;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    b24h = false;
    if(strncmp(szResp,"24hr",SERIAL_BUFFER_SIZE) == 0) {
        b24h = true;
    }

    return nErr;
}

int RST::getDateFormat(bool &bDdMmYy )
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand("!TGdf;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    bDdMmYy = false;
    if(strncmp(szResp,"dd/mm/yy",SERIAL_BUFFER_SIZE) == 0) {
        bDdMmYy = true;
    }
    return nErr;
}

int RST::getStandardTime(std::string &sTime)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand("!TGst;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    sTime.assign(szResp);
    return nErr;
}

int RST::getStandardDate(std::string &sDate)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = RSTSendCommand("!TGsd;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    sDate.assign(szResp);
    return nErr;
}



void RST::convertDecDegToDDMMSS(double dDeg, char *szResult, char &cSign, unsigned int size)
{
    int DD, MM, SS;
    double mm, ss;

    // convert dDeg decimal value to sDD:MM:SS

    cSign = dDeg>=0?'+':'-';
    dDeg = fabs(dDeg);
    DD = int(dDeg);
    mm = dDeg - DD;
    MM = int(mm*60);
    ss = (mm*60) - MM;
    SS = int(ceil(ss*60));
    snprintf(szResult, size, "%02d:%02d:%02d", DD, MM, SS);
}

int RST::convertDDMMSSToDecDeg(const char *szStrDeg, double &dDecDeg)
{
    int nErr = PLUGIN_OK;
    std::vector<std::string> vFieldsData;

    dDecDeg = 0;

    nErr = parseFields(szStrDeg, vFieldsData, ':');
    if(nErr)
        return nErr;

    if(vFieldsData.size() >= 3) {
        dDecDeg = atof(vFieldsData[0].c_str()) + atof(vFieldsData[1].c_str())/60 + atof(vFieldsData[1].c_str())/3600;
    }
    else
        nErr = ERR_PARSE;

    return nErr;
}

void RST::convertRaToHHMMSSt(double dRa, char *szResult, unsigned int size)
{
    int HH, MM;
    double hh, mm, SSt;

    // convert Ra value to HH:MM:SS.T before passing them to the RST
    HH = int(dRa);
    hh = dRa - HH;
    MM = int(hh*60);
    mm = (hh*60) - MM;
    SSt = mm * 60;
    snprintf(szResult,SERIAL_BUFFER_SIZE, "%02d:%02d:%02.1f", HH, MM, SSt);
}

int RST::convertHHMMSStToRa(const char *szStrRa, double &dRa)
{
    int nErr = PLUGIN_OK;
    std::vector<std::string> vFieldsData;

    dRa = 0;

    nErr = parseFields(szStrRa, vFieldsData, ':');
    if(nErr)
        return nErr;

    if(vFieldsData.size() >= 3) {
        dRa = atof(vFieldsData[0].c_str()) + atof(vFieldsData[1].c_str())/60 + atof(vFieldsData[1].c_str())/3600;
    }
    else
        nErr = ERR_PARSE;

    return nErr;
}


int RST::parseFields(const char *pszIn, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = PLUGIN_OK;
    std::string sSegment;
    std::stringstream ssTmp(pszIn);

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
