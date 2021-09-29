#include "x2mount.h"

X2Mount::X2Mount(const char* pszDriverSelection,
				 const int& nInstanceIndex,
				 SerXInterface					* pSerX,
				 TheSkyXFacadeForDriversInterface	* pTheSkyX,
				 SleeperInterface					* pSleeper,
				 BasicIniUtilInterface			* pIniUtil,
				 LoggerInterface					* pLogger,
				 MutexInterface					* pIOMutex,
				 TickCountInterface				* pTickCount)
{

	m_nPrivateMulitInstanceIndex	= nInstanceIndex;
	m_pSerX							= pSerX;
	m_pTheSkyXForMounts				= pTheSkyX;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;

	m_bSynced = false;
	m_bParked = false;
    m_bLinked = false;

    mRST.setSerxPointer(m_pSerX);
    mRST.setTSX(m_pTheSkyXForMounts);

    m_CurrentRateIndex = 0;

	// Read the current stored values for the settings
	if (m_pIniUtil)
	{
	}

    // set mount alignement type and meridian avoidance mode.
    if(strstr(pszDriverSelection,"Fork")) {
        mRST.setMountMode(MountTypeInterface::Symmetrical_Equatorial);
    }
    else if(strstr(pszDriverSelection,"Equatorial")) {
         mRST.setMountMode(MountTypeInterface::Asymmetrical_Equatorial);
     }
     else {
         mRST.setMountMode(MountTypeInterface::AltAz);
     }

    mRST.setSyncDateTimeOnConnect(true);
    mRST.setSyncLocationConnect(true);
}

X2Mount::~X2Mount()
{
	// Write the stored values

    if(m_bLinked)
        mRST.Disconnect();
    
    if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyXForMounts)
		delete m_pTheSkyXForMounts;
	if (m_pSleeper)
		delete m_pSleeper;
	if (m_pIniUtil)
		delete m_pIniUtil;
	if (m_pLogger)
		delete m_pLogger;
	if (m_pIOMutex)
		delete m_pIOMutex;
	if (m_pTickCount)
		delete m_pTickCount;
	

}

int X2Mount::queryAbstraction(const char* pszName, void** ppVal)
{
	*ppVal = NULL;
	
	if (!strcmp(pszName, SyncMountInterface_Name))
	    *ppVal = dynamic_cast<SyncMountInterface*>(this);
	if (!strcmp(pszName, SlewToInterface_Name))
		*ppVal = dynamic_cast<SlewToInterface*>(this);
	else if (!strcmp(pszName, AsymmetricalEquatorialInterface_Name))
		*ppVal = dynamic_cast<AsymmetricalEquatorialInterface*>(this);
	else if (!strcmp(pszName, OpenLoopMoveInterface_Name))
		*ppVal = dynamic_cast<OpenLoopMoveInterface*>(this);
	else if (!strcmp(pszName, NeedsRefractionInterface_Name))
		*ppVal = dynamic_cast<NeedsRefractionInterface*>(this);
	else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
		*ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
	else if (!strcmp(pszName, X2GUIEventInterface_Name))
		*ppVal = dynamic_cast<X2GUIEventInterface*>(this);
	else if (!strcmp(pszName, TrackingRatesInterface_Name))
		*ppVal = dynamic_cast<TrackingRatesInterface*>(this);
	else if (!strcmp(pszName, ParkInterface_Name))
		*ppVal = dynamic_cast<ParkInterface*>(this);
	else if (!strcmp(pszName, UnparkInterface_Name))
		*ppVal = dynamic_cast<UnparkInterface*>(this);
	else if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = GetLogger();
    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);
    else if (!strcmp(pszName, DriverSlewsToParkPositionInterface_Name))
        *ppVal = dynamic_cast<DriverSlewsToParkPositionInterface*>(this);

    return SB_OK;
}

#pragma mark - OpenLoopMoveInterface

int X2Mount::startOpenLoopMove(const MountDriverInterface::MoveDir& Dir, const int& nRateIndex)
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());


	m_CurrentRateIndex = nRateIndex;
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::startOpenLoopMove");
#endif

    nErr = mRST.startOpenLoopMove(Dir, nRateIndex);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::startOpenLoopMove error " + std::to_string(nErr));
#endif
        return ERR_CMDFAILED;
    }
    return SB_OK;
}

int X2Mount::endOpenLoopMove(void)
{
	int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::endOpenLoopMove");
#endif


    nErr = mRST.stopOpenLoopMove();
    if(nErr) {
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::endOpenLoopMove error " + std::to_string(nErr));
#endif
        return ERR_CMDFAILED;
    }
    return nErr;
}

int X2Mount::rateCountOpenLoopMove(void) const
{
    X2Mount* pMe = (X2Mount*)this;

    X2MutexLocker ml(pMe->GetMutex());
	return pMe->mRST.getNbSlewRates();
}

int X2Mount::rateNameFromIndexOpenLoopMove(const int& nZeroBasedIndex, char* pszOut, const int& nOutMaxSize)
{
    int nErr = SB_OK;
    std::string sTmp;

    X2MutexLocker ml(GetMutex());

#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::rateNameFromIndexOpenLoopMove");
#endif

    nErr = mRST.getRateName(nZeroBasedIndex, sTmp);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::rateNameFromIndexOpenLoopMove error " + std::to_string(nErr));
#endif
        return ERR_CMDFAILED;
    }
    strncpy(pszOut, sTmp.c_str(), nOutMaxSize);
    return nErr;
}

int X2Mount::rateIndexOpenLoopMove(void)
{
	return m_CurrentRateIndex;
}

#pragma mark - UI binding

int X2Mount::execModalSettingsDialog(void)
{
	int nErr = SB_OK;
	X2ModalUIUtil uiutil(this, m_pTheSkyXForMounts);
	X2GUIInterface*					ui = uiutil.X2UI();
	X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
	bool bPressedOK = false;
    std::string sTmp;
    std::string sTime;
    std::string sDate;
    std::string sLongitude;
    std::string sLatitude;
    std::string sTimeZone;
	if (NULL == ui) return ERR_POINTER;
	
	if ((nErr = ui->loadUserInterface("RST.ui", deviceType(), m_nPrivateMulitInstanceIndex)))
		return nErr;
	
	if (NULL == (dx = uiutil.X2DX())) {
		return ERR_POINTER;
	}

    X2MutexLocker ml(GetMutex());

	// Set values in the userinterface
    if(m_bLinked) {
        dx->setEnabled("pushButton",true);
        dx->setEnabled("pushButton_2",true);
        dx->setEnabled("pushButton_3",true);
        dx->setEnabled("alignmentType",true);
        dx->setEnabled("pushButton_4",true);

        nErr = mRST.getLocalTime(sTime);
        nErr |= mRST.getLocalDate(sDate);
        if(!nErr) {
            sTmp =sDate + " - " + sTime;
            dx->setText("time_date", sTmp.c_str());
        }

    }
    else {
        dx->setText("time_date", "");
        dx->setText("siteName", "");
        dx->setText("longitude", "");
        dx->setText("latitude", "");
        dx->setText("timezone", "");
        dx->setEnabled("pushButton",false);
        dx->setEnabled("pushButton_2",false);
        dx->setEnabled("pushButton_3",false);
        dx->setEnabled("alignmentType",false);
        dx->setEnabled("pushButton_4",false);
    }
	//Display the user interface
	if ((nErr = ui->exec(bPressedOK)))
		return nErr;
	
	//Retreive values from the user interface
	if (bPressedOK) {
	}
	return nErr;
}

void X2Mount::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr = SB_OK;
    std::string sTmpBuf;
    std::string sTime;
    std::string sDate;

    if(!m_bLinked)
        return ;

#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::uiEvent : " + std::string(pszEvent));
#endif

	if (!strcmp(pszEvent, "on_timer")) {

	}

    if (!strcmp(pszEvent, "on_pushButton_clicked")) {
        // TSX longitude is + going west and - going east, same as the RST
        mRST.setSiteData( m_pTheSkyXForMounts->longitude(),
                          m_pTheSkyXForMounts->latitude(),
                          m_pTheSkyXForMounts->timeZone());
    }
    

    if (!strcmp(pszEvent, "on_pushButton_2_clicked")) {
    }

    if (!strcmp(pszEvent, "on_pushButton_3_clicked")) {
    }

	return;
}

#pragma mark - LinkInterface
int X2Mount::establishLink(void)
{
    int nErr;
    char szPort[DRIVER_MAX_STRING];

	X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::establishLink");
#endif

    // get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);

	nErr =  mRST.Connect(szPort);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::establishLink error " + std::to_string(nErr));
#endif
        m_bLinked = false;
    }
    else {
        m_bLinked = true;
    }
    return nErr;
}

int X2Mount::terminateLink(void)
{
    int nErr = SB_OK;

	X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::terminateLink");
#endif

    nErr = mRST.Disconnect();
    m_bLinked = false;

    return nErr;
}

bool X2Mount::isLinked(void) const
{
	return mRST.isConnected();
}

bool X2Mount::isEstablishLinkAbortable(void) const
{
    return false;
}

#pragma mark - AbstractDriverInfo

void	X2Mount::driverInfoDetailedInfo(BasicStringInterface& str) const
{
	str = "RST X2 plugin by Rodolphe Pineau";
}

double	X2Mount::driverInfoVersion(void) const
{
	return PLUGIN_VERSION;
}

void X2Mount::deviceInfoNameShort(BasicStringInterface& str) const
{
    if(m_bLinked) {
        str = "RST";
    }
    else
        str = "Not connected1";
}
void X2Mount::deviceInfoNameLong(BasicStringInterface& str) const
{
	str = "RST Mount";
	
}
void X2Mount::deviceInfoDetailedDescription(BasicStringInterface& str) const
{
	str = "Astrometric Instruments Telescope Control System";
	
}
void X2Mount::deviceInfoFirmwareVersion(BasicStringInterface& str)
{
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::deviceInfoFirmwareVersion");
#endif

    if(m_bLinked) {
        std::string sFirmware;
        X2MutexLocker ml(GetMutex());
        mRST.getFirmwareVersion(sFirmware);
        str = sFirmware.c_str();
    }
    else
        str = "Not connected";
}
void X2Mount::deviceInfoModel(BasicStringInterface& str)
{
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::deviceInfoModel");
#endif

    if(m_bLinked) {
        str = "RST";
    }
    else
        str = "Not connected";
}

#pragma mark - Common Mount specifics
int X2Mount::raDec(double& ra, double& dec, const bool& bCached)
{
	int nErr = 0;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

	// Get the RA and DEC from the mount
	nErr = mRST.getRaAndDec(ra, dec);
    if(nErr)
        nErr = ERR_CMDFAILED;

	return nErr;
}

int X2Mount::abort()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::abort");
#endif

    nErr = mRST.Abort();
    if(nErr) {
        nErr = ERR_CMDFAILED;

#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::abort error " + std::to_string(nErr));
#endif
    }
    return nErr;
}

int X2Mount::startSlewTo(const double& dRa, const double& dDec)
{
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    std::stringstream ssTmp;
    ssTmp << "X2Mount::startSlewTo Ra : " << std::fixed << std::setprecision(2) << dRa << " , Dec: " << std::fixed << std::setprecision(2) << dDec;
    mRST.log(ssTmp.str());
#endif
    nErr = mRST.startSlewTo(dRa, dDec);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::startSlewTo error " + std::to_string(nErr));
#endif
        return ERR_CMDFAILED;
    }

    return nErr;
}

int X2Mount::isCompleteSlewTo(bool& bComplete) const
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;
    X2MutexLocker ml(pMe->GetMutex());
#ifdef PLUGIN_DEBUG
    pMe->mRST.log("X2Mount::isCompleteSlewTo");
#endif

    nErr = pMe->mRST.isSlewToComplete(bComplete);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        pMe->mRST.log("X2Mount::isCompleteSlewTo error " + std::to_string(nErr));
#endif
    }

	return nErr;
}

int X2Mount::endSlewTo(void)
{
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::endSlewTo");
#endif
    return SB_OK;
}


int X2Mount::syncMount(const double& ra, const double& dec)
{
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::syncMount");
#endif

    nErr = mRST.syncTo(ra, dec);
    if(nErr) {
        nErr = ERR_CMDFAILED;
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::syncMount error " + std::to_string(nErr));
#endif
    }
    return nErr;
}

bool X2Mount::isSynced(void)
{
    int nErr;

    if(!m_bLinked)
        return false;

    X2MutexLocker ml(GetMutex());

#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::isSynced");
#endif

   nErr = mRST.isAligned(m_bSynced);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::isSynced error " + std::to_string(nErr));
#endif
    }

    return m_bSynced;
}

#pragma mark - TrackingRatesInterface
int X2Mount::setTrackingRates(const bool& bTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec)
{
    int nErr = SB_OK;
    double dTrackRaArcSecPerHr;
    double dTrackDecArcSecPerHr;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::setTrackingRates");
#endif

    dTrackRaArcSecPerHr = dRaRateArcSecPerSec * 3600;
    dTrackDecArcSecPerHr = dDecRateArcSecPerSec * 3600;

#ifdef PLUGIN_DEBUG
    std::stringstream ssTmp;
    ssTmp << "X2Mount::setTrackingRates Tracking On: " << (bTrackingOn?"true":"false") <<"Ra rate : " << std::fixed << std::setprecision(2) << dRaRateArcSecPerSec << " , Dec rate: " << std::fixed << std::setprecision(2) << dDecRateArcSecPerSec;
    mRST.log(ssTmp.str());
#endif

    nErr = mRST.setTrackingRates(bTrackingOn, bIgnoreRates, dTrackRaArcSecPerHr, dTrackDecArcSecPerHr);

#ifdef PLUGIN_DEBUG
    if(nErr) {
        mRST.log("X2Mount::setTrackingRates error " + std::to_string(nErr));
    }
#endif

    return nErr;
}

int X2Mount::trackingRates(bool& bTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec)
{
    int nErr = SB_OK;
    double dTrackRaArcSecPerHr;
    double dTrackDecArcSecPerHr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::trackingRates");
#endif

    nErr = mRST.getTrackRates(bTrackingOn, dTrackRaArcSecPerHr, dTrackDecArcSecPerHr);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::trackingRates error " + std::to_string(nErr));
#endif
        return ERR_CMDFAILED;
    }
    dRaRateArcSecPerSec = dTrackRaArcSecPerHr / 3600;
    dDecRateArcSecPerSec = dTrackDecArcSecPerHr / 3600;

#ifdef PLUGIN_DEBUG
    std::stringstream ssTmp;
    ssTmp << "X2Mount::trackingRates Tracking On: " << (bTrackingOn?"true":"false") <<" , Ra rate : " << std::fixed << std::setprecision(2) << dRaRateArcSecPerSec << " , Dec rate: " << std::fixed << std::setprecision(2) << dDecRateArcSecPerSec;
    mRST.log(ssTmp.str());
#endif

	return nErr;
}

int X2Mount::siderealTrackingOn()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::siderealTrackingOn");
#endif
    nErr = setTrackingRates( true, true, 0.0, 0.0);
#ifdef PLUGIN_DEBUG
    if(nErr) {
        mRST.log("X2Mount::siderealTrackingOn error " + std::to_string(nErr));
    }
#endif

    return nErr;
}

int X2Mount::trackingOff()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::trackingOff");
#endif

    nErr = setTrackingRates( false, true, 0.0, 0.0);
#ifdef PLUGIN_DEBUG
    if(nErr) {
        mRST.log("X2Mount::trackingOff error " + std::to_string(nErr));
    }
#endif

    return nErr;
}

#pragma mark - NeedsRefractionInterface
bool X2Mount::needsRefactionAdjustments(void)
{

    if(!m_bLinked)
        return false;

    return true;
}

#pragma mark - Parking Interface
bool X2Mount::isParked(void)
{
    int nErr;
    bool bTrackingOn;
    bool bIsPArked;
    double dTrackRaArcSecPerHr, dTrackDecArcSecPerHr;

    if(!m_bLinked)
        return false;

    X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::isParked");
#endif

    nErr = mRST.getAtPark(bIsPArked);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::isParked error " + std::to_string(nErr));
#endif
        return false;
    }
    if(!bIsPArked) // not parked
        return false;

    // get tracking state.
    nErr = mRST.getTrackRates(bTrackingOn, dTrackRaArcSecPerHr, dTrackDecArcSecPerHr);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::isParked -> getTrackRates error " + std::to_string(nErr));
#endif
        return false;
    }
    // if AtPark and tracking is off, then we're parked, if not then we're unparked.
    if(bIsPArked && !bTrackingOn)
        m_bParked = true;
    else
        m_bParked = false;
    return m_bParked;
}

int X2Mount::startPark(const double& dAz, const double& dAlt)
{
	double dRa, dDec;
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;
	
	X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::startPark");
#endif

	nErr = m_pTheSkyXForMounts->HzToEq(dAz, dAlt, dRa, dDec);
    if (nErr) {
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::startPark error " + std::to_string(nErr));
#endif
        return nErr;
    }


#ifdef PLUGIN_DEBUG
    std::stringstream ssTmp;
    ssTmp << "X2Mount::startPark Alt : " << std::fixed << std::setprecision(2) << dAlt << " , Az: " << std::fixed << std::setprecision(2) << dAz << "[ Ra : " << std::fixed << std::setprecision(2) << dRa << " , Dec: " << std::fixed << std::setprecision(2) << dDec <<"]";
    mRST.log(ssTmp.str());
#endif
    // goto park
    nErr = mRST.gotoPark(dRa, dDec);
    if (nErr) {
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::startPark error " + std::to_string(nErr));
#endif
        nErr = ERR_CMDFAILED;
    }
	return nErr;
}


int X2Mount::isCompletePark(bool& bComplete) const
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;

    X2MutexLocker ml(pMe ->GetMutex());

#ifdef PLUGIN_DEBUG
    pMe->mRST.log("X2Mount::isCompletePark");
#endif

    nErr = pMe->mRST.getAtPark(bComplete);
    if(nErr)
        nErr = ERR_CMDFAILED;

    if (nErr) {
#ifdef PLUGIN_DEBUG
        pMe->mRST.log("X2Mount::isCompletePark error " + std::to_string(nErr));
#endif
    }
	return nErr;
}

int X2Mount::endPark(void)
{
    return SB_OK;
}

int X2Mount::startUnpark(void)
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::startUnpark");
#endif

    nErr = mRST.unPark();
    if(nErr) {
#ifdef PLUGIN_DEBUG
        mRST.log("X2Mount::startUnpark error " + std::to_string(nErr));
#endif
        nErr = ERR_CMDFAILED;
    }
    m_bParked = false;
    return nErr;
}

/*!Called to monitor the unpark process.
 \param bComplete Set to true if the unpark is complete, otherwise set to false.
*/
int X2Mount::isCompleteUnpark(bool& bComplete) const
{
    int nErr;
    bool bIsParked;
    bool bTrackingOn;
    double dTrackRaArcSecPerHr, dTrackDecArcSecPerHr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;

    X2MutexLocker ml(pMe ->GetMutex());
#ifdef PLUGIN_DEBUG
    pMe->mRST.log("X2Mount::isCompleteUnpark");
#endif

    bComplete = false;

    nErr = pMe->mRST.getAtPark(bIsParked);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        pMe->mRST.log("X2Mount::isCompleteUnpark error " + std::to_string(nErr));
#endif
        return ERR_CMDFAILED;
    }
    if(!bIsParked) { // no longer parked.
        bComplete = true;
        pMe->m_bParked = false;
        return nErr;
    }

    // if we're still at the park position
    // get tracking state. If tracking is off, then we're parked, if not then we're unparked.
    nErr = pMe->mRST.getTrackRates(bTrackingOn, dTrackRaArcSecPerHr, dTrackDecArcSecPerHr);
    if(nErr)
        nErr = ERR_CMDFAILED;
#ifdef PLUGIN_DEBUG
    pMe->mRST.log("X2Mount::isCompleteUnpark getTrackRates error " + std::to_string(nErr));
#endif

    if(bTrackingOn) {
        bComplete = true;
        pMe->m_bParked = false;
    }
    else {
        bComplete = false;
        pMe->m_bParked = true;
    }
	return SB_OK;
}

/*!Called once the unpark is complete.
 This is called once for every corresponding startUnpark() allowing software implementations of unpark.
 */
int X2Mount::endUnpark(void)
{
	return SB_OK;
}

#pragma mark - AsymmetricalEquatorialInterface

bool X2Mount::knowsBeyondThePole()
{
    X2MutexLocker ml(GetMutex());
   return true;
}

int X2Mount::beyondThePole(bool& bYes) {
    X2MutexLocker ml(GetMutex());
    // “beyond the pole” =  “telescope west of the pier”,
	// bYes = mRST.GetIsBeyondThePole();
	return SB_OK;
}


double X2Mount::flipHourAngle()
{
    X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::flipHourAngle");
#endif

	return 0.0;
}


int X2Mount::gemLimits(double& dHoursEast, double& dHoursWest)
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::gemLimits");
#endif

    nErr = mRST.getLimits(dHoursEast, dHoursWest);

#ifdef PLUGIN_DEBUG
    if(nErr) {
        mRST.log("X2Mount::gemLimits error " + std::to_string(nErr));
    }
    std::stringstream ssTmp;
    ssTmp << "X2Mount::gemLimits dHoursEast : " << std::fixed << std::setprecision(2) << dHoursEast << " , dHoursWest: " << std::fixed << std::setprecision(2) << dHoursWest;
    mRST.log(ssTmp.str());
#endif
    // temp debugging.
	dHoursEast = 0.0;
	dHoursWest = 0.0;
#ifdef PLUGIN_DEBUG
    std::stringstream().swap(ssTmp);
    ssTmp << "X2Mount::gemLimits dHoursEast : " << std::fixed << std::setprecision(2) << dHoursEast << " , dHoursWest: " << std::fixed << std::setprecision(2) << dHoursWest;
    mRST.log(ssTmp.str());
#endif

    return SB_OK;
}

MountTypeInterface::Type X2Mount::mountType()
{
#ifdef PLUGIN_DEBUG
    mRST.log("X2Mount::mountType");
#endif

    return  mRST.mountType();
}


#pragma mark - SerialPortParams2Interface

void X2Mount::portName(BasicStringInterface& str) const
{
    char szPortName[DRIVER_MAX_STRING];

    portNameOnToCharPtr(szPortName, DRIVER_MAX_STRING);

    str = szPortName;

}

void X2Mount::setPortName(const char* pszPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORT_NAME, pszPort);

}


void X2Mount::portNameOnToCharPtr(char* pszPort, const unsigned int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize,DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORT_NAME, pszPort, pszPort, nMaxSize);

}




