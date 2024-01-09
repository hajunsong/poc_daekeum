#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    gripper = new ZimmerGripper();
    memset(gripper_write_reg, 0, sizeof(uint16_t)*NUM_SEND_REG);
    memset(gripper_read_reg, 0, sizeof(uint16_t)*NUM_RECV_REG);

	QObject::connect(ui->btnRobotConnect, SIGNAL(clicked()), this, SLOT(btnRobotConnectClicked()));
    connect(ui->btnGripperConnect, SIGNAL(clicked()), this, SLOT(btnGripperConnectClicked()));
    connect(ui->btnInit, SIGNAL(clicked()), this, SLOT(btnInitClicked()));
    connect(ui->btnRun, SIGNAL(clicked()), this, SLOT(btnRunClicked()));

    connect(ui->btnPickObj, SIGNAL(clicked()), this, SLOT(btnPickObjClicked()));
    connect(ui->btnChuckObj, SIGNAL(clicked()), this, SLOT(btnChuckObjClicked()));
    connect(ui->btnUnChuckObj, SIGNAL(clicked()), this, SLOT(btnUnChuckObjClicked()));
    connect(ui->btnDoorSwitch, SIGNAL(clicked()), this, SLOT(btnDoorSwitchClicked()));
    connect(ui->btnPlaceObj, SIGNAL(clicked()), this, SLOT(btnPlaceObjClicked()));
	connect(ui->btnLatheStart, SIGNAL(clicked()), this, SLOT(btnLatheStartClicked()));
	connect(ui->btnLatheWait, SIGNAL(clicked()), this, SLOT(btnLatheWaitClicked()));

    connect(ui->btnPrint, SIGNAL(clicked()), this, SLOT(btnPrintClicked()));

    connect(ui->btnGripperInit, SIGNAL(clicked()), this, SLOT(btnGripperInitClicked()));
    connect(ui->btnGripperGrip, SIGNAL(clicked()), this, SLOT(btnGripperGripClicked()));
	connect(ui->btnGripperRelease, SIGNAL(clicked()), this, SLOT(btnGripperReleaseClicked()));

    connect(this, SIGNAL(robotUpdate()), this, SLOT(robotStateUpdate()));
    connect(this, SIGNAL(gripperUpdate()), this, SLOT(gripperStateUpdate()));

    connect(ui->btnTest, SIGNAL(clicked()), this, SLOT(btnTestClicked()));

    txtJoint = {ui->txtJoint1, ui->txtJoint2, ui->txtJoint3, ui->txtJoint4, ui->txtJoint5, ui->txtJoint6};
    txtPose = {ui->txtPose1, ui->txtPose2, ui->txtPose3, ui->txtPose4, ui->txtPose5, ui->txtPose6};

    mainState = -1;
    pocState = -1;
    pocSubState = -1;

    robot_init = false;
    gripper_init = false;
    robot_connected = false;
    gripper_connected = false;

    move_complete = false;
	robot_moving = false;
    cmd_flag = false;
    chuck_moving = false;

    door_cnt = 0;
    obj_cnt = 0;

    mainTimer = new QTimer(this);
	mainTimer->setInterval(50);
    connect(mainTimer, SIGNAL(timeout()), this, SLOT(update()));
    mainTimer->start();

	tcpSocket = new TCP::TcpSocket();
	connect(this, SIGNAL(visionUpdate()), this, SLOT(visionStateUpdate()));
	connect(ui->btnVisionListen, SIGNAL(clicked()), this, SLOT(btnVisionListenClicked()));

	customSettings = new CustomSettings(ui);
	customSettings->loadConfigFile();

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

MainWindow::~MainWindow()
{
	customSettings->saveConfigFile();
	Drfl.close_connection();
	delete tcpSocket;
    delete customSettings;
    delete ui;
}

void MainWindow::update()
{
	if(robot_connected){
		emit robotUpdate();
	}

	if(gripper_connected){
		emit gripperUpdate();
	}

	if(vision_connected){
//		emit visionUpdate();
	}

    if(robot_connected && gripper_connected){
        switch(mainState){
            case Wait:
            {
                if(pocState > 0) {
                   mainState = Start;
                   // mainState = TestStart;
                }
                break;
            }
            case Init:
            {
				if(!robot_init){
//                    moveJoint(JS_ready, duration_slow);
					robot_init = true;
				}
				if(!gripper_init && robot_init){
					gripper->gripper_init();
					gripper_init = true;
				}
				pocState = 0;
                pocSubState = 0;
                ui->rbInit->setChecked(true);
				obj_cnt = 0;
                break;
            }
            case Start:
            {
                POCControlFunc();
//                std::cout << (int)obj_cnt << std::endl;
                break;
            }
            case TestStart:
            {
                POCTestFunc();
                break;
            }
            default:
                break;
        }
    }
}

void MainWindow::POCControlFunc()
{
//	std::cout << "pocState : " << (int)pocState << std::endl;
//	std::cout << "pocSubState : " << (int)pocSubState << std::endl;
    switch(pocState){
        case Ready:
        {
            if(POCReady()){
                pocState = Ready;
                pocSubState = 0;
            }
            break;
        }
        case Pick:
        {
            if(POCPickObj()){
                pocState = Chuck1;
                pocSubState = 0;
            }
            ui->rbPick->setChecked(true);
            break;
        }
        case Chuck1:
        {
            if(POCChuck1Obj()){
                pocState = Door1;
                pocSubState = 0;
            }
            ui->rbChuck1->setChecked(true);
            break;
        }
        case Door1:
        {
            if(POCDoorSwitch1()){
                if(door_cnt%2 == 0) {
                    pocState = UnChuck1;
                }
                else{
					pocState = LatheStart1;
                }
                pocSubState = 0;
            }
            ui->rbDoor1->setChecked(true);
            break;
        }
		case LatheStart1:
		{
			if(POCLatheStart1()){
				pocState = LatheWait1;
				pocSubState = 0;
			}
			ui->rbLatheStart->setChecked(true);
			break;
		}
        case LatheWait1:
        {
			if(POCLatheWait1()){
				pocState = Door1;
				pocSubState = 0;
			}
			ui->rbLatheWait->setChecked(true);
            break;
        }
        case UnChuck1:
        {
            if(POCUnChuck1Obj()){
                pocState = Place;
                pocSubState = 0;
            }
            ui->rbChuck1->setChecked(true);
            break;
        }
        case Place:
        {
            if(POCPlaceObj()){
				obj_cnt++;
				if(obj_cnt == 1) obj_cnt = 2;
				else if(obj_cnt == 3) obj_cnt = 9;
				else if(obj_cnt == 10) obj_cnt = 11;

				pocState = obj_cnt < 12 ? Pick : Ready;
				pocSubState = 0;
            }
            ui->rbPlace->setChecked(true);
            break;
        }
        default:
            break;
    }
}

bool MainWindow::POCReady(){
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
                moveJoint(JS_ready, duration_fast);
                break;
            default:
                finish = true;
                break;
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCPickObj()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
                moveJoint(JS_ready2pick2, duration_fast);
                break;
            }
            case 1:
            {
                moveJoint(JS_ready2pick3, duration_slow);
                break;
            }
            case 2:
            {
                moveGripperOff();
                break;
            }
            case 3:
            {
                double offset[6] = {0, 0, 0, 0, 0, 0};
				offset[1] = -Pick_obj_offset_x*(obj_cnt%3);
				offset[0] = Pick_obj_offset_y*(obj_cnt/3);
				movePose(offset, duration_super_fast, "rel");
                break;
            }
            case 4:
            {
                double offset[6] = {0, 0, WS_to_obj_z, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 5:
            {
                moveGripperOn();
                break;
            }
            case 6:
            {
                double offset[6] = {0, 0, -WS_to_obj_z, 0, 0, 0};
                movePose(offset, duration_slow, "rel");
                break;
            }
            case 7:
            {
                moveJoint(JS_ready2pick2, duration_fast);
                break;
            }
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCChuck1Obj()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
                moveJoint(JS_to_chuck1, duration_fast);
                break;
            }
            case 1:
            {
//                movePose(WS_to_chuck3, duration_slow);
                break;
            }
            case 2:
            {
                moveJoint(JS_to_chuck3, duration_slow);
                break;
            }
            case 3:
            {
				moveSetForceCtrlOn();
				double offset[6] = {WS_insert_x, 0, 0, 0, 0, 0};
				movePose(offset, duration_super_slow, "rel");

                break;
            }
            case 4: // foot switch
            {
				moveChuckClose();
                break;
            }
            case 5:
            {
				moveGripperOff();
                break;
            }
            case 6:
            {
				moveSetForceCtrlOff();
				double offset[6] = {-WS_insert_x, 0, -5, 0, 0, 0};
				movePose(offset, duration_slow, "rel");
                break;
			}
			case 7:
			{
//				moveJoint(JS_to_chuck3, duration_slow);
				break;
			}
			case 8:
			{
				moveJoint(JS_to_chuck1, duration_slow);
				break;
			}
            default:
            {
				finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCDoorSwitch1()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
				if(door_cnt%2 == 0)
                    moveJoint(JS_to_door_SW2, duration_fast);
                break;
            }
            case 1:
            {
//                if(door_cnt%2 == 0)
                    moveJoint(JS_over_door_SW, duration_fast);
                break;
            }
            case 2:
            {
				double offset[6] = {0, WS_push_SW_y, 0, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 3:
            {
				double offset[6] = {0, -WS_push_SW_y, 0, 0, 0, 0};
				movePose(offset, duration_super_slow, "rel");
				break;
            }
            case 4:
			{
				moveJoint(JS_to_door_SW2, duration_fast);
                break;
            }
			case 5:
			{
				if(door_cnt%2 == 0){
					moveDoorClose();
				}
				if(door_cnt%2 == 1){
					moveDoorOpen();
				}
				break;
			}
            default:
            {
                door_cnt++;
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

	return finish;
}

bool MainWindow::POCLatheStart1()
{
	bool finish = false;

	if(!cmd_flag){
		switch(pocSubState){
			case 0:
			{
//				moveJoint(JS_over_MEM_SW, duration_fast);
				break;
			}
			case 1:
			{
				double offset[6] = {WS_push_MEM_SW_x, 0, 0, 0, 0, 0};
//				movePose(offset, duration_super_slow, "rel");
				break;
			}
			case 2:
			{
				double offset[6] = {-WS_push_MEM_SW_x, 0, 0, 0, 0, 0};
//				movePose(offset, duration_super_slow, "rel");
				break;
			}
			case 3:
			{
//				moveJoint(JS_over_start_SW, duration_fast);
				break;
			}
			case 4:
			{
				double offset[6] = {WS_push_start_SW_x, 0, 0, 0, 0, 0};
//				movePose(offset, duration_super_slow, "rel");
				break;
			}
			case 5:
			{
				double offset[6] = {-WS_push_start_SW_x, 0, 0, 0, 0, 0};
//				movePose(offset, duration_super_slow, "rel");
				break;
			}
			case 6:
			{
//				moveJoint(JS_ready2pick3, duration_fast);
				break;
			}
			default:
			{
				finish = true;
				break;
			}
		}
		pocSubState++;
	}

	return finish;
}

bool MainWindow::POCLatheWait1()
{
	bool finish = false;

	if(!cmd_flag){
		switch(pocSubState){
			case 0:
			{
//				movePose(WS_lathewait, duration_slow, "abs", "base");
//				moveJoint(JS_to_chuck1, duration_fast);
				break;
			}
			case 1:
			{
				moveJoint(JS_lathewait2, duration_slow);
//				tcpSocket->sendData('1');
				break;
			}
			case 2:
			{
				moveLathe();
				break;
			}
			case 3:
			{
//				moveJoint(JS_over_JOG_SW, duration_fast);
				break;
			}
			case 4:
			{
				double offset[6] = {0, WS_push_JOG_SW_y, 0, 0, 0, 0};
//				movePose(offset, duration_super_slow, "rel");
				break;
			}
			case 5:
			{
				double offset[6] = {0, -WS_push_JOG_SW_y, 0, 0, 0, 0};
//				movePose(offset, duration_super_slow, "rel");
				break;
			}
			case 6:
			{
				moveJoint(JS_to_door_SW2, duration_fast);
				break;
			}
			default:
			{
				finish = true;
				break;
			}
		}
		pocSubState++;
	}

	return finish;
}

bool MainWindow::POCUnChuck1Obj()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
                moveJoint(JS_to_unchuck1, duration_fast);
                break;
            }
            case 1:
            {
                moveJoint(JS_to_unchuck3, duration_fast);
                break;
            }
            case 2:
            {
				double offset[6] = {WS_remove_x, 0, 0, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 3:
            {
                moveGripperOn();
                break;
            }
            case 4: // foot switch
            {
				moveChuckOpen();
                break;
            }
            case 5:
            {
				double offset[6] = {-WS_remove_x, 0, 0, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");
                break;
			}
			case 6:
			{
				moveJoint(JS_to_unchuck1, duration_fast);
				break;
			}
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCPlaceObj()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
//                moveJoint(JS_withdraw_pick, duration_fast);
                break;
            }
            case 1:
            {
				moveJoint(JS_over_place, duration_fast);
                break;
            }
			case 2:
			{
				double offset[6] = {0, 0, 0, 0, 0, 0};
				offset[1] = -Place_obj_offset_x*(obj_cnt%3);
				offset[0] = Place_obj_offset_y*(obj_cnt/3);
				movePose(offset, duration_fast, "rel");
				break;
			}
			case 3:
            {
				moveSetForceCtrlOn();
				double offset[6] = {0, 0, WS_place_z, 0, 0, 0};
				movePose(offset, duration_super_slow, "rel");
                break;
            }
			case 4:
            {
                moveGripperOff();
                break;
            }
			case 5:
            {
				moveSetForceCtrlOff();
                double offset[6] = {0, 0, -WS_place_z, 0, 0, 0};
                movePose(offset, duration_slow, "rel");
                break;
            }
			default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

void MainWindow::moveJoint(double cmd[], double vel)
{
    if(!cmd_flag){
		float cmd_f[6];
		for(unsigned int i = 0; i < 6; i++){
			cmd_f[i] = (float)cmd[i];
		}
		Drfl.amovej(cmd_f, (float)vel, 30);
		cmd_type = MoveJ;
		memcpy(cmd_value, cmd, sizeof(double)*6);
		pthread_create(&move_wait_thread, NULL, move_wait_func, this);
    }
}

void MainWindow::movePose(double *cmd, double vel, std::string opt, std::string coord)
{
    if(!cmd_flag){
		float vel1[2] = {(float)vel, (float)vel};
		float acc1[2] = {500, 500};
		float cmd_f[6];
		for(unsigned int i = 0; i < 6; i++){
			cmd_f[i] = (float)cmd[i];
		}
		Drfl.amovel(cmd_f, vel1, acc1, 0.f, MOVE_MODE_RELATIVE, MOVE_REFERENCE_BASE);
		cmd_type = MoveL;
		pthread_create(&move_wait_thread, NULL, move_wait_func, this);
	}
}

void MainWindow::moveGripperOn()
{
	if(!cmd_flag){
        cmd_type = GripOn;
        usleep(10000);
        pthread_create(&move_wait_thread, NULL, move_wait_func, this);
    }
}

void MainWindow::moveGripperOff()
{
	if(!cmd_flag){
        cmd_type = GripOff;
        usleep(10000);
        pthread_create(&move_wait_thread, NULL, move_wait_func, this);
    }
}

void MainWindow::moveChuckOpen()
{
	if(!cmd_flag){
		Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_1, true);
		cmd_type = ChuckOpen;
		usleep(10000);
		pthread_create(&move_wait_thread, NULL, move_wait_func, this);
	}
}

void MainWindow::moveChuckClose()
{
	if(!cmd_flag){
		Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_1, true);
		cmd_type = ChuckClose;
		usleep(10000);
		pthread_create(&move_wait_thread, NULL, move_wait_func, this);
	}
}

void MainWindow::moveDoorOpen()
{
	if(!cmd_flag){
		cmd_type = DoorOpen;
		usleep(10000);
		pthread_create(&move_wait_thread, NULL, move_wait_func, this);
	}
}

void MainWindow::moveDoorClose()
{
	if(!cmd_flag){
		cmd_type = DoorClose;
		usleep(10000);
		pthread_create(&move_wait_thread, NULL, move_wait_func, this);
	}
}

void MainWindow::moveLathe()
{
	usleep(3000000);
}

void MainWindow::moveSetForceCtrlOn()
{
	float stx[6] = {3000, 3000, 3000, 200, 200, 200};
//	float des_force[6] = {-50, 0, 0, 0, 0, 0};
//	unsigned char dir_force[6] = {1, 0, 0, 0, 0, 0};

	Drfl.task_compliance_ctrl(stx);
//	Drfl.set_desired_force(des_force, dir_force);
	usleep(50000);
}

void MainWindow::moveSetForceCtrlOff()
{
//	Drfl.release_force();
	Drfl.release_compliance_ctrl();
	usleep(50000);
}

void MainWindow::btnRobotConnectClicked()
{
    if(ui->btnRobotConnect->text().compare("Connect")){
//        RobotDisconnect();

		Drfl.close_connection();

        ui->btnRobotConnect->setText("Connect");

		robot_connected = false;
	}
    else{
//        SetRobotConf(UR10, ui->txtRobotIP->text().toStdString().c_str(), ui->txtRobotPORT->text().toInt());
//        RobotConnect();

		// ???? ????
		assert(Drfl.open_connection(ui->txtRobotIP->text().toStdString().c_str(), ui->txtRobotPORT->text().toInt()));

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

        ui->btnRobotConnect->setText("Disconnect");

        robot_connected = true;
    }
}

void MainWindow::btnGripperConnectClicked()
{
    if(ui->btnGripperConnect->text().compare("Connect")){
        gripper->disconnect();

        ui->btnGripperConnect->setText("Connect");

        gripper_connected = false;
    }
    else{
        gripper->connect(ui->txtGripperIP->text().toStdString(), ui->txtGripperPORT->text().toInt());

        usleep(10000);

        ui->btnGripperConnect->setText("Disconnect");

        gripper_connected = true;
    }
}

void MainWindow::btnInitClicked()
{
    mainState = Init;
    door_cnt = 0;
	obj_cnt = 0;
}

void MainWindow::btnRunClicked()
{
    mainState = Start;
	pocState = Pick;
    door_cnt = 0;
    ui->rbRun->setChecked(true);
}

void MainWindow::btnPickObjClicked()
{
    mainState = Start;
    pocState = Pick;
    door_cnt = 0;
}

void MainWindow::btnChuckObjClicked()
{
    mainState = Start;
    pocState = Chuck1;
    door_cnt = 0;
}

void MainWindow::btnUnChuckObjClicked()
{
    mainState = Start;
    pocState = UnChuck1;
    door_cnt = 0;
}

void MainWindow::btnDoorSwitchClicked()
{
    mainState = Start;
    pocState = Door1;
    door_cnt = 0;
}

void MainWindow::btnPlaceObjClicked()
{
	mainState = Start;
	pocState = Place;
	door_cnt = 0;
}

void MainWindow::btnTestClicked()
{
    mainState = TestStart;
    pocState = TestPick;
	obj_cnt = 0;
}

void MainWindow::btnLatheStartClicked()
{
	mainState = Start;
	pocState = LatheStart1;
	obj_cnt = 0;
}

void MainWindow::btnLatheWaitClicked()
{
	mainState = Start;
	pocState = LatheWait1;
	obj_cnt = 0;
}

void MainWindow::btnVisionListenClicked()
{
	tcpSocket->setPort(ui->txtVisionPORT->text().toInt());
	tcpSocket->start();
}

void MainWindow::btnPrintClicked()
{
    for(unsigned int i = 0; i < 6; i++){
//        printf("%f\t", robotInfor.jnt[i]);
		std::cout << jnt[i] << ", ";
    }
//    printf("\n");
    std::cout << std::endl;
	for(unsigned int i = 0; i < 6; i++){
//        printf("%f\t", robotInfor.mat[i]);
		std::cout << pose[i] << ", ";
    }
    std::cout << std::endl;
    for(unsigned int i = 0; i < 3; i++){
        for(unsigned int j = 0; j < 3; j++){
			std::cout << mat[i*4 + j] << ",";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    //    printf("\n");
}

void MainWindow::btnGripperInitClicked()
{
    gripper->gripper_init();
    gripper_init = true;
}

void MainWindow::btnGripperGripClicked()
{
    gripper->gripper_grip();
}

void MainWindow::btnGripperReleaseClicked()
{
    gripper->gripper_release();
}

void MainWindow::robotStateUpdate()
{
	state = Drfl.check_motion();
	if(state == 0) state = 1;
	if(state == 2) {
		robot_moving = true;
	}
	else{
		robot_moving = false;
	}
	memcpy(jnt, Drfl.get_current_posj()->_fPosition, sizeof(float)*6);
	memcpy(pose, Drfl.get_current_posx()->_fTargetPos, sizeof(float)*6);
	memset(mat, 0, sizeof(float)*16);
	for(int i = 0; i < 3; i++){
		memcpy(mat + i*4, Drfl.get_current_rotm()[i], sizeof(float)*3);
		mat[i*4 + 3] = pose[i];
	}
	mat[15] = 1;

	ui->txtRobotState->setText(QString::number(state));

	for(unsigned int i = 0; i < 6; i++){
		txtJoint[i]->setText(QString::number(jnt[i]));
	}
	for(unsigned int i = 0; i < 6; i++){
		txtPose[i]->setText(QString::number(pose[i]));
	}

	door_close = Drfl.get_digital_input(GPIO_CTRLBOX_DIGITAL_INDEX_1);
	chuck_close = Drfl.get_digital_input(GPIO_CTRLBOX_DIGITAL_INDEX_3);
	chuck_open = Drfl.get_digital_input(GPIO_CTRLBOX_DIGITAL_INDEX_4);

	ui->txtStateDoor->setText(door_close ? "Close" : "Open");

	if(!chuck_open && !chuck_close){
		chuck_moving = true;
	}
	else{
		chuck_moving = false;
	}

	if(chuck_open && !chuck_close)
		ui->txtStateChuck->setText("Open");
	if(!chuck_open && chuck_close)
		ui->txtStateChuck->setText("Close");
	if(!chuck_open && !chuck_close)
		ui->txtStateChuck->setText("Open");
}

void MainWindow::gripperStateUpdate()
{
    gripper->get_write_reg(gripper_write_reg);
    gripper->get_read_reg(gripper_read_reg);

    ui->txtBasePosition->setText(QString::number(gripper_write_reg[4]));
    ui->txtShiftPosition->setText(QString::number(gripper_write_reg[5]));
    ui->txtWorkPosition->setText(QString::number(gripper_write_reg[7]));
    ui->txtCurrentPosition->setText(QString::number(gripper_read_reg[2]));

	ui->txtErrorNum->setText(QString::number(((gripper_read_reg[0]&0x0004) == 0x0004)) + "(" + QString::number(gripper_read_reg[1], 16) + ")");
}

void MainWindow::visionStateUpdate(){

}

void* MainWindow::move_wait_func(void *arg){
    MainWindow *pThis = static_cast<MainWindow*>(arg);

    pThis->move_complete = false;
    pThis->cmd_flag = true;

    unsigned int cnt = 0;
    bool run = true;
    while(true){
        if(!run) break;
        switch(pThis->cmd_type){
            case MoveJ:
            {
                double err_max = abs(pThis->cmd_value[0] - pThis->jnt[0]);
                double err = 0;
                for(unsigned int i = 1; i < 6; i++){
                    err = abs(pThis->cmd_value[i] - pThis->jnt[i]);
                    err_max = err > err_max ? err : err_max;
                }
				if(pThis->robot_moving && pThis->state == 1) {
                    std::cout << "move finish" << std::endl;
					pThis->robot_moving = false;
                    run = false;
					break;
                }
				if(cnt >= 300 && pThis->state == 1) {
                    std::cout << "move wait timeout " << cnt << std::endl;
                    run = false;
					break;
                }
                if(err_max < 1e-3) {
                    std::cout << "goal reach" << std::endl;
                    run = false;
					break;
                }
//                std::cout << err_max << std::endl;
                cnt++;

                break;
            }
            case MoveL:
            {
//				double err_max = abs(pThis->cmd_value[0] - pThis->pose[0]);
//                double err = 0;
//                for(unsigned int i = 1; i < 3; i++){
//					err = abs(pThis->cmd_value[i] - pThis->pose[i]);
//                    err_max = err > err_max ? err : err_max;
//                }
                //        std::cout << err_max << std::endl;
//                if(pThis->state == 2.0) {
//                    moving = true;
//                    pThis->robot_moving = true;
//                }
				if(pThis->robot_moving && pThis->state == 1) {
                    std::cout << "move finish" << std::endl;
                    run = false;
					break;
                }
				if(cnt >= 300 && pThis->state == 1) {
                    std::cout << "move wait timeout " << cnt << std::endl;
                    run = false;
					break;
                }
//                if(err_max < 1e-3) {
//                    std::cout << "goal reach" << std::endl;
//                    run = false;
//                }
                cnt++;
                break;
            }
            case GripOn:
            {
                pThis->gripper->gripper_grip();
				pThis->robot_moving = false;
                run = false;
                pThis->cmd_type = None;
                break;
            }
            case GripOff:
            {
                pThis->gripper->gripper_release();
				pThis->robot_moving = false;
                run = false;
                pThis->cmd_type = None;
                break;
            }
            case GripCustom:
            {
                pThis->gripper->gripper_custom(1100, 50, 50);
				pThis->robot_moving = false;
                run = false;
                pThis->cmd_type = None;
                break;
            }
            case ChuckOpen:
			{
				if(!pThis->chuck_moving && !pThis->chuck_open && pThis->chuck_close){
					Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_1, true);
				}
				if(!pThis->chuck_moving && pThis->chuck_open && !pThis->chuck_close){
					run = false;
					pThis->cmd_type = None;
					Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_1, false);
				}
                break;
            }
            case ChuckClose:
            {
				if(!pThis->chuck_moving && pThis->chuck_open && !pThis->chuck_close){
					Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_1, true);
				}
				if(!pThis->chuck_moving && !pThis->chuck_open && pThis->chuck_close){
					run = false;
					pThis->cmd_type = None;
					Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_1, false);
				}
                break;
            }
            case DoorOpen:
            {
                if(!pThis->door_close){
                    run = false;
                }
                break;
            }
            case DoorClose:
            {
                if(pThis->door_close){
                    run = false;
                }
                break;
            }
			case LatheWait:
			{
				if(pThis->tcpSocket->recvData()){
					run = false;
				}
				break;
			}
            default:
            {
                break;
            }
        }

        usleep(2000);
    }

    pThis->mainState = Wait;
    pThis->move_complete = true;
	pThis->robot_moving = false;
    pThis->cmd_flag = false;

    std::cout << "\nfinish move wait thread\n" << std::endl;

    return nullptr;
}

void MainWindow::POCTestFunc()
{
    switch(pocState){
        case TestPick:
        {
            if(POCTestPick()){
                pocState = TestPlace;
                pocSubState = 0;
            }
            break;
        }
//        case TestPlace:
//        {
//            if(POCTestPlace()){
//                obj_cnt++;
//                pocState = obj_cnt < 12 ? TestPick : TestFinish;
//                pocSubState = 0;
//            }
//            break;
//        }
//        case TestFinish:
//        {
//            POCReady();
//            pocSubState = 0;
//            break;
//        }
        default:
        {
            break;
        }
    }
}

bool MainWindow::POCTestPick()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
//                moveJoint(TJS_ready2pick2, duration_fast);
                break;
            }
            case 1:
            {
//                moveJoint(TJS_ready2pick3, duration_slow);
                break;
            }
            case 2:
            {
                moveGripperOff();
                break;
            }
            case 3:
            {
                double offset[6] = {0, 0, 0, 0, 0, 0};
                offset[0] = -TPick_obj_offset_x*(obj_cnt%3);
                offset[2] = TPick_obj_offset_y*(obj_cnt/3);
//                movePose(offset, duration_fast, "rel", "tcp");
                break;
            }
            case 4:
            {
//                double offset[6] = {0, 0, TWS_to_obj_z, 0, 0, 0};
//                movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 5:
            {
                moveGripperOn();
                break;
            }
            case 6:
            {
                double offset[6] = {0, 0, -TWS_to_obj_z, 0, 0, 0};
                movePose(offset, duration_slow, "rel");
                break;
            }
//            case 6:
//            {
//                moveJoint(TJS_withdraw_pick, duration_fast);
//                break;
//            }
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}

bool MainWindow::POCTestPlace()
{
    bool finish = false;

    if(!cmd_flag){
        switch(pocSubState){
            case 0:
            {
                moveJoint(TJS_withdraw_pick, duration_fast);
                break;
            }
            case 1:
            {
                moveJoint(TJS_over_place, duration_fast);
                break;
            }
            case 2:
            {
                double offset[6] = {0, 0, 0, 0, 0, 0};
                offset[0] = TPlace_obj_offset_x*(obj_cnt%3);
                offset[2] = -TPlace_obj_offset_y*(obj_cnt/3);
                movePose(offset, duration_fast, "rel", "tcp");
                break;
            }
            case 3:
            {
                double offset[6] = {0, 0, TWS_place_z, 0, 0, 0};
                movePose(offset, duration_super_slow, "rel");
                break;
            }
            case 4:
            {
                moveGripperOff();
                break;
            }
            case 5:
            {
                double offset[6] = {0, 0, -TWS_place_z, 0, 0, 0};
                movePose(offset, duration_slow, "rel");
                break;
            }
            default:
            {
                finish = true;
                break;
            }
        }
        pocSubState++;
    }

    return finish;
}
