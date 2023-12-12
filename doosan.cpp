#include "doosan.h"

bool g_bHasControlAuthority = FALSE;
bool g_TpInitailizingComplted = FALSE;
bool g_mStat = FALSE;
bool g_Stop = FALSE;
bool moving = FALSE;

CDRFLEx Drfl;

void OnTpInitializingCompleted()
{
	// Tp ???? ???? ????? ???.
	g_TpInitailizingComplted = TRUE;
	Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

void OnHommingCompleted()
{
	// 50msec ??? ????? ?????? ??.
	cout << "homming completed" << endl;
}

void OnProgramStopped(const PROGRAM_STOP_CAUSE)
{
	assert(Drfl.PlayDrlStop(STOP_TYPE_SLOW));
	// 50msec ??? ????? ?????? ??.
	// assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
	cout << "program stopped" << endl;
}

void OnMonitoringDataCB(const LPMONITORING_DATA pData)
{
	// 50msec ??? ????? ?????? ??.

	return;
	cout << "# monitoring 0 data " << pData->_tCtrl._tTask._fActualPos[0][0]
		 << pData->_tCtrl._tTask._fActualPos[0][1]
		 << pData->_tCtrl._tTask._fActualPos[0][2]
		 << pData->_tCtrl._tTask._fActualPos[0][3]
		 << pData->_tCtrl._tTask._fActualPos[0][4]
		 << pData->_tCtrl._tTask._fActualPos[0][5] << endl;
}

void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData)
{
	return;
	cout << "# monitoring 1 data " << pData->_tCtrl._tWorld._fTargetPos[0]
		 << pData->_tCtrl._tWorld._fTargetPos[1]
		 << pData->_tCtrl._tWorld._fTargetPos[2]
		 << pData->_tCtrl._tWorld._fTargetPos[3]
		 << pData->_tCtrl._tWorld._fTargetPos[4]
		 << pData->_tCtrl._tWorld._fTargetPos[5] << endl;
}

void OnMonitoringCtrlIOCB(const LPMONITORING_CTRLIO pData)
{
	return;
	cout << "# monitoring ctrl 0 data" << endl;
	for (int i = 0; i < 16; i++)
	{
		cout << (int)pData->_tInput._iActualDI[i] << endl;
	}
}

void OnMonitoringCtrlIOExCB(const LPMONITORING_CTRLIO_EX pData)
{
	return;
	cout << "# monitoring ctrl 1 data" << endl;
	for (int i = 0; i < 16; i++)
	{
		cout << (int)pData->_tInput._iActualDI[i] << endl;
	}
	for (int i = 0; i < 16; i++)
	{
		cout << (int)pData->_tOutput._iTargetDO[i] << endl;
	}
}

void OnMonitoringStateCB(const ROBOT_STATE eState)
{
	// 50msec ??? ????? ?????? ??.
	switch ((unsigned char)eState)
	{
#if 0 // TP ?????? ?????? ?????????? API ?????????? ??????? ????.(TP????
	  // ??? ????? ???, ???)
	case STATE_NOT_READY:
		if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_INIT_CONFIG);
		break;
	case STATE_INITIALIZING:
		// add initalizing logic
		if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_ENABLE_OPERATION);
		break;
#endif
	case STATE_EMERGENCY_STOP:
		// popup
		break;
	case STATE_STANDBY:
	case STATE_MOVING:
	case STATE_TEACHING:
		break;
	case STATE_SAFE_STOP:
		if (g_bHasControlAuthority)
		{
			Drfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
			Drfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
		}
		break;
	case STATE_SAFE_OFF:
		// cout << "STATE_SAFE_OFF1" << endl;
		if (g_bHasControlAuthority)
		{
			// cout << "STATE_SAFE_OFF2" << endl;
			Drfl.SetRobotControl(CONTROL_SERVO_ON);
		}
		break;
	case STATE_SAFE_STOP2:
		if (g_bHasControlAuthority)
			Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
		break;
	case STATE_SAFE_OFF2:
		if (g_bHasControlAuthority)
		{
			Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
		}
		break;
	case STATE_RECOVERY:
		// Drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
		break;
	default:
		break;
	}
	return;
//	cout << "current state: " << (int)eState << endl;
}

void OnMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL eTrasnsitControl)
{
	// 50msec ??? ????? ?????? ??.

	switch (eTrasnsitControl)
	{
	case MONITORING_ACCESS_CONTROL_REQUEST:
		assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO));
		// Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_YES);
		break;
	case MONITORING_ACCESS_CONTROL_GRANT:
		g_bHasControlAuthority = TRUE;
		// cout << "GRANT1" << endl;
		// cout << "MONITORINGCB : " << (int)Drfl.GetRobotState() << endl;
		OnMonitoringStateCB(Drfl.GetRobotState());
		// cout << "GRANT2" << endl;
		break;
	case MONITORING_ACCESS_CONTROL_DENY:
	case MONITORING_ACCESS_CONTROL_LOSS:
		g_bHasControlAuthority = FALSE;
		if (g_TpInitailizingComplted)
		{
			// assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST));
			Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
		}
		break;
	default:
		break;
	}
}

void OnLogAlarm(LPLOG_ALARM tLog)
{
	g_mStat = true;
	cout << "Alarm Info: "
		 << "group(" << (unsigned int)tLog->_iGroup << "), index("
		 << tLog->_iIndex << "), param(" << tLog->_szParam[0] << "), param("
		 << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << endl;
}

void OnTpPopup(LPMESSAGE_POPUP tPopup)
{
	cout << "Popup Message: " << tPopup->_szText << endl;
	cout << "Message Level: " << tPopup->_iLevel << endl;
	cout << "Button Type: " << tPopup->_iBtnType << endl;
}

void OnTpLog(const char *strLog)
{
	cout << "Log Message: " << strLog << endl;
}

void OnTpProgress(LPMESSAGE_PROGRESS tProgress)
{
	cout << "Progress cnt : " << (int)tProgress->_iTotalCount << endl;
	cout << "Current cnt : " << (int)tProgress->_iCurrentCount << endl;
}

void OnTpGetuserInput(LPMESSAGE_INPUT tInput)
{
	cout << "User Input : " << tInput->_szText << endl;
	cout << "Data Type : " << (int)tInput->_iType << endl;
}

void OnRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData)
{
	//    static int td = 0;
	//    if (td++ == 1000) {
	//    	td = 0;
	//    	printf("timestamp : %.3f\n", tData->time_stamp);
	//    	printf("joint : %f %f %f %f %f %f\n", tData->actual_joint_position[0], tData->actual_joint_position[1], tData->actual_joint_position[2], tData->actual_joint_position[3], tData->actual_joint_position[4], tData->actual_joint_position[5]);
	//		printf("q = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
	//				tData->actual_joint_position[0], tData->actual_joint_position[1], tData->actual_joint_position[2],
	//				tData->actual_joint_position[3], tData->actual_joint_position[4], tData->actual_joint_position[5]);
	//		printf("q_dot = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
	//				tData->actual_joint_velocity[0], tData->actual_joint_velocity[1], tData->actual_joint_velocity[2],
	//				tData->actual_joint_velocity[3], tData->actual_joint_velocity[4], tData->actual_joint_velocity[5]);
	//		printf("trq_g = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
	//				tData->gravity_torque[0], tData->gravity_torque[1], tData->gravity_torque[2],
	//				tData->gravity_torque[3], tData->gravity_torque[4], tData->gravity_torque[5]);
	//    }
}

void OnDisConnected()
{

}
