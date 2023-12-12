#pragma once

#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>

#include "doosanapi/include/DRFLEx.h"
using namespace DRAFramework;

#undef NDEBUG
#include <assert.h>

extern CDRFLEx Drfl;

extern void OnTpInitializingCompleted();
extern void OnHommingCompleted();
extern void OnProgramStopped(const PROGRAM_STOP_CAUSE);
extern void OnMonitoringDataCB(const LPMONITORING_DATA pData);
extern void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData);
extern void OnMonitoringCtrlIOCB(const LPMONITORING_CTRLIO pData);
extern void OnMonitoringCtrlIOExCB(const LPMONITORING_CTRLIO_EX pData);
extern void OnMonitoringStateCB(const ROBOT_STATE eState);
extern void OnMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL eTrasnsitControl);
extern void OnLogAlarm(LPLOG_ALARM tLog);
extern void OnTpPopup(LPMESSAGE_POPUP tPopup);
extern void OnTpLog(const char *strLog);
extern void OnTpProgress(LPMESSAGE_PROGRESS tProgress);
extern void OnTpGetuserInput(LPMESSAGE_INPUT tInput);
extern void OnRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData);
extern void OnDisConnected();

extern bool g_bHasControlAuthority;
extern bool g_TpInitailizingComplted;
extern bool g_mStat;
extern bool g_Stop;
extern bool moving;
