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
    m_bSyncOnConnect = false;
    m_nParkingPosition = 1;

    mRST.setSerxPointer(m_pSerX);
    mRST.setTSX(m_pTheSkyXForMounts);

    m_CurrentRateIndex = 0;

	// Read the current stored values for the settings
	if (m_pIniUtil)
	{
        m_bSyncOnConnect = (m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SYNC_TIME, 0) == 0 ? false : true);
        m_nParkingPosition = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_PARK_POS, 1);
        
	}

    mRST.setSyncLocationDataConnect(m_bSyncOnConnect);
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
    nErr = mRST.startOpenLoopMove(Dir, nRateIndex);
    if(nErr) {
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

    nErr = mRST.stopOpenLoopMove();
    if(nErr) {
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

    nErr = mRST.getRateName(nZeroBasedIndex, sTmp);
    if(nErr) {
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
    double dVolts;

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
        dx->setEnabled("pushButton_3",true);

        nErr = mRST.getLocalTime(sTime);
        nErr |= mRST.getLocalDate(sDate);
        if(!nErr) {
            sTmp =sDate + " - " + sTime;
            dx->setText("time_date", sTmp.c_str());
        }
        mRST.getSiteData(sLongitude, sLatitude, sTimeZone);
        sTimeZone = std::string("GMT ") + sTimeZone;

        dx->setText("longitude", sLongitude.c_str());
        dx->setText("latitude", sLatitude.c_str());
        dx->setText("timezone", sTimeZone.c_str());
        dx->setCurrentIndex("comboBox", m_nParkingPosition-1);

        mRST.getInputVoltage(dVolts);
        dx->setText("voltage", (std::string("Input volatage : ") + std::to_string(dVolts)).c_str());
    }
    else {
        dx->setText("time_date", "");
        dx->setText("siteName", "");
        dx->setText("longitude", "");
        dx->setText("latitude", "");
        dx->setText("timezone", "");
        dx->setEnabled("pushButton",false);
        dx->setEnabled("pushButton_3",false);
    }

    dx->setChecked("checkBox", (m_bSyncOnConnect?1:0));

    //Display the user interface
	if ((nErr = ui->exec(bPressedOK)))
		return nErr;
	
	//Retreive values from the user interface
	if (bPressedOK) {
        m_bSyncOnConnect = (dx->isChecked("checkBox")==1?true:false);
        m_nParkingPosition = dx->currentIndex("comboBox") + 1;
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_SYNC_TIME, (m_bSyncOnConnect?1:0));
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_PARK_POS, m_nParkingPosition);
	}
	return nErr;
}

void X2Mount::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr = SB_OK;
    std::string sLongitude;
    std::string sLatitude;
    std::string sTimeZone;
    std::string sTime;
    std::string sDate;
    std::string sTmp;
    double dVolts;

    if(!m_bLinked)
        return ; 

	if (!strcmp(pszEvent, "on_timer")) {
        nErr = mRST.getLocalTime(sTime);
        nErr |= mRST.getLocalDate(sDate);
        if(!nErr) {
            sTmp =sDate + " - " + sTime;
            uiex->setText("time_date", sTmp.c_str());
        }
        mRST.getInputVoltage(dVolts);
        uiex->setText("voltage", (std::string("Input volatage : ") + std::to_string(dVolts)).c_str());
	}

    if (!strcmp(pszEvent, "on_pushButton_clicked")) {
        mRST.syncDate();
        mRST.syncTime();
        nErr = mRST.getLocalTime(sTime);
        nErr |= mRST.getLocalDate(sDate);
        if(!nErr) {
            sTmp =sDate + " - " + sTime;
            uiex->setText("time_date", sTmp.c_str());
        }

        mRST.setSiteData( m_pTheSkyXForMounts->longitude(),
                          m_pTheSkyXForMounts->latitude(),
                          m_pTheSkyXForMounts->timeZone());
        mRST.getSiteData(sLongitude, sLatitude, sTimeZone);
        sTimeZone = std::string("GMT ") + sTimeZone;

        uiex->setText("longitude", sLongitude.c_str());
        uiex->setText("latitude", sLatitude.c_str());
        uiex->setText("timezone", sTimeZone.c_str());
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

    // get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);

	nErr =  mRST.Connect(szPort);
    if(nErr) {
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
#ifdef PLUGIN_DEBUG
    str = "RST X2 plugin by Rodolphe Pineau [DEBUG]";
#else
	str = "RST X2 plugin by Rodolphe Pineau";
#endif
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
	str = "Rainbow Astro RST mount";
	
}
void X2Mount::deviceInfoFirmwareVersion(BasicStringInterface& str)
{
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

    nErr = mRST.Abort();
    if(nErr) {
        nErr = ERR_CMDFAILED;
    }
    return nErr;
}

int X2Mount::startSlewTo(const double& dRa, const double& dDec)
{
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    nErr = mRST.startSlewTo(dRa, dDec);
    if(nErr) {
        return nErr;
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
    nErr = pMe->mRST.isSlewToComplete(bComplete);
	return nErr;
}

int X2Mount::endSlewTo(void)
{
    return SB_OK;
}


int X2Mount::syncMount(const double& ra, const double& dec)
{
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    nErr = mRST.syncTo(ra, dec);
    if(nErr) {
        nErr = ERR_CMDFAILED;
    }
    return nErr;
}

bool X2Mount::isSynced(void)
{
    int nErr;

    if(!m_bLinked)
        return false;

    X2MutexLocker ml(GetMutex());

    nErr = mRST.isAligned(m_bSynced);

    return m_bSynced;
}

#pragma mark - TrackingRatesInterface
int X2Mount::setTrackingRates(const bool& bSiderialTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec)
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = mRST.setTrackingRates(bSiderialTrackingOn, bIgnoreRates, dRaRateArcSecPerSec, dDecRateArcSecPerSec);

    return nErr;
}

int X2Mount::trackingRates(bool& bSiderialTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec)
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    
    nErr = mRST.getTrackRates(bSiderialTrackingOn, dRaRateArcSecPerSec, dDecRateArcSecPerSec);
    if(nErr) {
        return ERR_CMDFAILED;
    }

    return nErr;
}

int X2Mount::siderealTrackingOn()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    nErr = setTrackingRates( true, true, 0.0, 0.0);
    return nErr;
}

int X2Mount::trackingOff()
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    nErr = setTrackingRates( false, true, 0.0, 0.0);
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

    if(!m_bLinked)
        return false;

    X2MutexLocker ml(GetMutex());
    nErr = mRST.getAtPark(m_bParked);
    if(nErr) {
        return false;
    }
    return m_bParked;
}

int X2Mount::startPark(const double& dAz, const double& dAlt)
{
	double dParkAz, dPArkAlt;
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;
	
	X2MutexLocker ml(GetMutex());
    // for now TSX pass 0.00 for both values.
    // so we overrides this
    switch (m_nParkingPosition) {
        case 1:
            dParkAz = 270.00;
            dPArkAlt = 0.00;
            break;

        case 2:
            dParkAz = 180.00;
            dPArkAlt = 0.00;
            break;

        case 3:
            dParkAz = 90.00;
            dPArkAlt = 0.00;
            break;

        default:
            dParkAz = 270.00;
            dPArkAlt = 0.00;
            break;
    }

    nErr = mRST.gotoPark(dPArkAlt, dParkAz);
    if (nErr) {
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

    nErr =  pMe->mRST.isSlewToComplete(bComplete);
    if(nErr)
        return nErr;

    if(bComplete) {
        // stop tracking
        nErr = pMe->setTrackingRates( false, true, 0.0, 0.0);
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
    nErr = mRST.unPark();
    if(nErr) {
        nErr = ERR_CMDFAILED;
    }

    return nErr;
}

/*!Called to monitor the unpark process.
 \param bComplete Set to true if the unpark is complete, otherwise set to false.
*/
int X2Mount::isCompleteUnpark(bool& bComplete) const
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;

    X2MutexLocker ml(pMe ->GetMutex());
    bComplete = false;

    nErr = pMe->mRST.isUnparkDone(bComplete);

    if(bComplete) { // no longer parked.
        pMe->m_bParked = false;
    }
    else
        pMe->m_bParked = true;

    return nErr;
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
    int nErr = SB_OK;
    X2MutexLocker ml(GetMutex());
    // “beyond the pole” =  “telescope west of the pier”,
    nErr = mRST.IsBeyondThePole(bYes);
	return nErr;
}


double X2Mount::flipHourAngle()
{
    X2MutexLocker ml(GetMutex());
	return 0.0;
}

MountTypeInterface::Type X2Mount::mountType()
{
    return  MountTypeInterface::Symmetrical_Equatorial;
}

int X2Mount::gemLimits(double& dHoursEast, double& dHoursWest)
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    nErr = mRST.getLimits(dHoursEast, dHoursWest);

    // temp debugging.
	dHoursEast = 0.0;
	dHoursWest = 0.0;
    return SB_OK;
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




