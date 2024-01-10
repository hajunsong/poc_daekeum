#include "m1013_v3.h"
// mm, deg

bool g_bHasControlAuthority = FALSE;
bool g_TpInitailizingComplted = FALSE;
bool g_mStat = FALSE;
bool g_Stop = FALSE;
bool moving = FALSE;

CDRFLEx Drfl;

m1013::m1013()
{
	g_bHasControlAuthority = FALSE;
	g_TpInitailizingComplted = FALSE;
	g_mStat = FALSE;
	g_Stop = FALSE;
	moving = FALSE;

	Drfl.set_on_homming_completed(OnHommingCompleted);
	Drfl.set_on_monitoring_data(OnMonitoringDataCB);
	Drfl.set_on_monitoring_data_ex(OnMonitoringDataExCB);
	Drfl.set_on_monitoring_ctrl_io(OnMonitoringCtrlIOCB);
	Drfl.set_on_monitoring_ctrl_io_ex(OnMonitoringCtrlIOExCB);
	Drfl.set_on_monitoring_state(OnMonitoringStateCB);
	Drfl.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
	Drfl.set_on_tp_initializing_completed(OnTpInitializingCompleted);
	Drfl.set_on_log_alarm(OnLogAlarm);
	Drfl.set_on_tp_popup(OnTpPopup);
	Drfl.set_on_tp_log(OnTpLog);
	Drfl.set_on_tp_progress(OnTpProgress);
	Drfl.set_on_tp_get_user_input(OnTpGetuserInput);
	Drfl.set_on_rt_monitoring_data(OnRTMonitoringData);

	Drfl.set_on_program_stopped(OnProgramStopped);
	Drfl.set_on_disconnected(OnDisConnected);
}

void m1013::InitSocket(TCPClient *sock1, TCPClient *sock2, ReciveData *Info, int cord_type)
{
	// connect
//	InitSocket(sock1,Info,cord_type);
	(void)sock1;
	(void)sock2;
	(void)Info;
}

bool m1013::RobotConnect(std::string ip, int port, ReciveData *Info){
	std::cout << "robot ip : " << ip << std::endl;
	std::cout << "robot port : " << (int)port << std::endl;
	// ???? ????
	assert(Drfl.open_connection(ip.c_str(), port));

	// ???? ???? ???
	SYSTEM_VERSION tSysVerion = {
		'\0',
	};
	Drfl.get_system_version(&tSysVerion);
	// ?????? ?????? ???? ????
	Drfl.setup_monitoring_version(1);
	Drfl.set_robot_control(CONTROL_SERVO_ON);
	cout << "System version: " << tSysVerion._szController << endl;
	cout << "Library version: " << Drfl.get_library_version() << endl;

	while ((Drfl.get_robot_state() != STATE_STANDBY) || !g_bHasControlAuthority)
		// Sleep(1000);
		this_thread::sleep_for(std::chrono::milliseconds(1000));

	assert(Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS));
	assert(Drfl.set_robot_system(ROBOT_SYSTEM_REAL));

	sdata=Info;

	Conf.rottype = zyz_deg;

	return true;
}

void m1013::RobotDisconnect(){
	Drfl.close_connection();
}

void m1013::MoveL(WayPoints *data)
{
	double dpos[6];
	for(unsigned int i = 0; i < 6; i++){
		dpos[i] = data->Pnt[0].position[i]*1000;
	}

	Conf.InverseRot(data->Pnt[0].R, &dpos[3], &dpos[4], &dpos[5]);

	float fpos[6];
	for(unsigned int i = 0; i < 6; i++){
		fpos[i] = static_cast<float>(dpos[i]);
	}

	float acc = (float)vel_l*1.5;
	float vel1[2] = {(float)vel_l,};
	float acc1[2] = {acc,};

	Drfl.amovel(fpos, vel1, acc1, 0.f, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);
}

void m1013::MoveJ(WayPoints *data,double acc)
{
	data->Jnt[0].value[0] = data->Jnt[0].value[0]*180/M_PI;
	data->Jnt[0].value[1] = data->Jnt[0].value[1]*180/M_PI;
	data->Jnt[0].value[2] = data->Jnt[0].value[2]*180/M_PI;
	data->Jnt[0].value[3] = data->Jnt[0].value[3]*180/M_PI;
	data->Jnt[0].value[4] = data->Jnt[0].value[4]*180/M_PI;
	data->Jnt[0].value[5] = data->Jnt[0].value[5]*180/M_PI;

	float jnt[6];
	for(unsigned int i = 0; i < 6; i++){
		jnt[i] = static_cast<float>(data->Jnt[0].value[i]);
	}

	Drfl.amovej(jnt, (float)vel_j, (float)acc);
}

void m1013::MoveJ(WayPoints *data)
{
	data->Jnt[0].value[0] = data->Jnt[0].value[0]*180/M_PI;
	data->Jnt[0].value[1] = data->Jnt[0].value[1]*180/M_PI;
	data->Jnt[0].value[2] = data->Jnt[0].value[2]*180/M_PI;
	data->Jnt[0].value[3] = data->Jnt[0].value[3]*180/M_PI;
	data->Jnt[0].value[4] = data->Jnt[0].value[4]*180/M_PI;
	data->Jnt[0].value[5] = data->Jnt[0].value[5]*180/M_PI;

	float jnt[6];
	for(unsigned int i = 0; i < 6; i++){
		jnt[i] = static_cast<float>(data->Jnt[0].value[i]);
	}

	Drfl.amovej(jnt, (float)vel_j, (float)vel_j*1.5);
}

void m1013::MoveB(WayPoints *data)
{
	unsigned char num = static_cast<unsigned char>(data->NUM_PNT);
	MOVE_POSB *pos = new MOVE_POSB[num];

	for(unsigned int j = 0; j < num; j++){
		double dpos[6];
		for(unsigned int i = 0; i < 6; i++){
			dpos[i] = data->Pnt[j].position[i]*1000;
		}

		Conf.InverseRot(data->Pnt[j].R, &dpos[3], &dpos[4], &dpos[5]);

		for(unsigned int i = 0; i < 6; i++){
			pos[j]._fTargetPos[0][i] = static_cast<float>(dpos[i]);
		}
		pos[j]._iBlendType = 0;
		pos[j]._fBlendRad = 50;
	}

	float acc = (float)vel_l*1.5;
	float vel1[2] = {(float)vel_l,};
	float acc1[2] = {acc,};

	Drfl.amoveb(pos, num, vel1, acc1, 0.f, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);
}

void m1013::MoveC(WayPoints *data)
{
	unsigned char num = static_cast<unsigned char>(data->NUM_PNT);
	float pos[2][6] = {{0,}, {0,}};

	for(unsigned int j = 0; j < num; j++){
		double dpos[6];
		for(unsigned int i = 0; i < 6; i++){
			dpos[i] = data->Pnt[j].position[i]*1000;
		}

		Conf.InverseRot(data->Pnt[j].R, &dpos[3], &dpos[4], &dpos[5]);

		for(unsigned int i = 0; i < 6; i++){
			pos[j][i] = static_cast<float>(dpos[i]);
		}
	}

	float acc = (float)vel_l*1.5;
	float vel1[2] = {(float)vel_l,};
	float acc1[2] = {acc,};

	Drfl.amovec(pos, vel1, acc1, 0.f, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);
}

void m1013::RobotInfo()
{
	int state = Drfl.check_motion();
	if(state == 0) sdata->RobotState = 1;
	else if(state == 2) sdata->RobotState = 2;
	else sdata->RobotState = 0;

	float pose[6], jnt[6];
	memcpy(jnt, Drfl.get_current_posj()->_fPosition, sizeof(float)*6);
	memcpy(pose, Drfl.get_current_posx()->_fTargetPos, sizeof(float)*6);

	sdata->Jnt[0] = static_cast<double>(jnt[0])*M_PI/180.0;
	sdata->Jnt[1] = static_cast<double>(jnt[1])*M_PI/180.0;
	sdata->Jnt[2] = static_cast<double>(jnt[2])*M_PI/180.0;
	sdata->Jnt[3] = static_cast<double>(jnt[3])*M_PI/180.0;
	sdata->Jnt[4] = static_cast<double>(jnt[4])*M_PI/180.0;
	sdata->Jnt[5] = static_cast<double>(jnt[5])*M_PI/180.0;

	sdata->TCPpos[0] = static_cast<double>(pose[0])/1000.0;
	sdata->TCPpos[1] = static_cast<double>(pose[1])/1000.0;
	sdata->TCPpos[2] = static_cast<double>(pose[2])/1000.0;
	sdata->TCPpos[3] = static_cast<double>(pose[3])*M_PI/180.0;
	sdata->TCPpos[4] = static_cast<double>(pose[4])*M_PI/180.0;
	sdata->TCPpos[5] = static_cast<double>(pose[5])*M_PI/180.0;
}

void m1013::Stop()
{
	Drfl.stop();
}

void m1013::SetVelocity(double v)
{
	if(v<0){
		v=0;
	}
	else if(v>100){
		v=100;
	}

	vel_j = 224.0*((v/100.0));
	vel_l = 2000.0*((v/100.0));

	if(vel_j >= 225.0){
		vel_j = 225.0;
		std::cout << "JOINT VEL LIMIT(225)" << std::endl;
	}
}

void m1013::settcp(bool on){
}

bool m1013::WaitMove(){
	int cnt = 0;
	while(sdata->RobotState == 1){
		usleep(1000);
		cnt++;
		if(cnt > 500) break;
	}

	while(sdata->RobotState == 2 && cnt < 500){
		usleep(1000);
	}

	return true;
}

void m1013::ControlBoxDigitalOut(int out)
{
	GPIO_CTRLBOX_DIGITAL_INDEX dio_indx[16] = {
		GPIO_CTRLBOX_DIGITAL_INDEX_1,
		GPIO_CTRLBOX_DIGITAL_INDEX_2,
		GPIO_CTRLBOX_DIGITAL_INDEX_3,
		GPIO_CTRLBOX_DIGITAL_INDEX_4,
		GPIO_CTRLBOX_DIGITAL_INDEX_5,
		GPIO_CTRLBOX_DIGITAL_INDEX_6,
		GPIO_CTRLBOX_DIGITAL_INDEX_7,
		GPIO_CTRLBOX_DIGITAL_INDEX_8,
		GPIO_CTRLBOX_DIGITAL_INDEX_9,
		GPIO_CTRLBOX_DIGITAL_INDEX_10,
		GPIO_CTRLBOX_DIGITAL_INDEX_11,
		GPIO_CTRLBOX_DIGITAL_INDEX_12,
		GPIO_CTRLBOX_DIGITAL_INDEX_13,
		GPIO_CTRLBOX_DIGITAL_INDEX_14,
		GPIO_CTRLBOX_DIGITAL_INDEX_15,
		GPIO_CTRLBOX_DIGITAL_INDEX_16,
	};
	bool dout[16] = {0,};
	int temp = 1;
	for(int i = 0; i < 16; i++){
		dout[i] = out&(temp << i);
		Drfl.set_digital_output(dio_indx[i], dout[i]);
	}
}

int m1013::ControlBoxDigitalIn()
{
	GPIO_CTRLBOX_DIGITAL_INDEX dio_indx[16] = {
		GPIO_CTRLBOX_DIGITAL_INDEX_1,
		GPIO_CTRLBOX_DIGITAL_INDEX_2,
		GPIO_CTRLBOX_DIGITAL_INDEX_3,
		GPIO_CTRLBOX_DIGITAL_INDEX_4,
		GPIO_CTRLBOX_DIGITAL_INDEX_5,
		GPIO_CTRLBOX_DIGITAL_INDEX_6,
		GPIO_CTRLBOX_DIGITAL_INDEX_7,
		GPIO_CTRLBOX_DIGITAL_INDEX_8,
		GPIO_CTRLBOX_DIGITAL_INDEX_9,
		GPIO_CTRLBOX_DIGITAL_INDEX_10,
		GPIO_CTRLBOX_DIGITAL_INDEX_11,
		GPIO_CTRLBOX_DIGITAL_INDEX_12,
		GPIO_CTRLBOX_DIGITAL_INDEX_13,
		GPIO_CTRLBOX_DIGITAL_INDEX_14,
		GPIO_CTRLBOX_DIGITAL_INDEX_15,
		GPIO_CTRLBOX_DIGITAL_INDEX_16,
	};
	bool din[16] = {0,};
	int temp = 0;
	for(int i = 0; i < 16; i++){
		din[i] = Drfl.get_digital_input(dio_indx[i]);
		temp += ((int)din[i]) << i;
	}

	return temp;
}

void m1013::RobotComplianceCtrlOn(float stpx, float stpy, float stpz, float strx, float stry, float strz)
{
	float stx[6] = {stpx, stpy, stpz, strx, stry, strz};

	Drfl.task_compliance_ctrl(stx);
	usleep(50000);
}

void m1013::RobotComplianceCtrlOff()
{
	Drfl.release_compliance_ctrl();
	usleep(50000);
}







/*
 * Doosan API callback function (?)
*/
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
