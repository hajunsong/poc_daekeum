#include "rb10.h"


rb10::rb10()
{
    initFlag = false;
    cmdConfirmFlag = false;
    moveCmdFlag = false;
    moveCmdCnt = 0;

    memset(&systemStat, 0, sizeof(systemSTAT));
    systemStat.sdata.program_mode = -1;
    systemStat.sdata.robot_state = -1;
}

void rb10::InitSocket(QTcpSocket *sock1,QTcpSocket *sock2,ReciveData *Info,int cord_type){
    cmdSocket=sock1;
    dataSocket=sock2;
    Uidata=Info;


}
void rb10::SetTCP(cord TCPcord){
    QString text;
    text.sprintf("set_tcp_info(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)"
                 , TCPcord.value[0], TCPcord.value[1], TCPcord.value[2]
                 , TCPcord.value[3], TCPcord.value[4], TCPcord.value[5]);
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    cmdSocket->flush();
}
void rb10::CobotInit(){
    QString text;
    text.sprintf("mc jall init");
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
}
void rb10::ProgramMode_Real(){
    QString text;
    text.sprintf("pgmode real");
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
}
void rb10::ProgramMode_Simulation(){
    QString text;
    text.sprintf("pgmode simulation");
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
}
void rb10::MoveJoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd, float acc){
    QString text;
    text.sprintf("jointall %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, joint1, joint2, joint3, joint4, joint5, joint6);
    moveCmdFlag = true;
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    cmdSocket->flush();
    systemStat.sdata.robot_state = 3; //run
}
void rb10::MoveTCP(float x, float y, float z, float rx, float ry, float rz, float spd, float acc){
    QString text;
//    text.sprintf("movetcp %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, x, y, z, rx, ry, rz);
    text.sprintf("move_l(pnt[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],300,100,1)",x,y,z,rx,ry,rz);
    moveCmdFlag = true;
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    cmdSocket->flush();
    systemStat.sdata.robot_state = 3; //run
}
void rb10::MoveCircle_ThreePoint(int type, float x1, float y1, float z1, float rx1, float ry1, float rz1, float x2, float y2, float z2, float rx2, float ry2, float rz2, float spd, float acc){
    QString text;
    char buf[15];
    if(type == 0){
        sprintf(buf, "intended");
    }else if(type == 1){
        sprintf(buf, "constant");
    }else if(type == 2){
        sprintf(buf, "radial");
    }
    text.sprintf("movecircle threepoints %s %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                 buf, spd, acc, x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2);
    moveCmdFlag = true;
    cmdConfirmFlag =false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    systemStat.sdata.robot_state = 3;
}
void rb10::MoveCircle_Axis(int type, float cx, float cy, float cz, float ax, float ay, float az, float rot_angle, float spd, float acc){
    QString text;
    char buf[15];
    if(type == 0){
        sprintf(buf, "intended");
    }else if(type == 1){
        sprintf(buf, "constant");
    }else if(type == 2){
        sprintf(buf, "radial");
    }
    text.sprintf("movecircle axis %s %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
                 buf, spd, acc, rot_angle, cx, cy, cz, ax, ay, az);
    moveCmdFlag = true;
    cmdConfirmFlag =false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    systemStat.sdata.robot_state = 3;
}
void rb10::MoveJointBlend_Clear(){
    QString text;
    text.sprintf("blend_jnt clear_pt");
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    cmdSocket->flush();
}
void rb10::MoveJointBlend_AddPoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd, float acc){
    QString text;
    text.sprintf("blend_jnt add_pt %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, joint1, joint2, joint3, joint4, joint5, joint6);
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    systemStat.sdata.robot_state = 3; //run
}
void rb10::MoveJointBlend_MovePoint(){
    QString text;
    text.sprintf("blend_jnt move_pt");
    moveCmdFlag = true;
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    systemStat.sdata.robot_state = 3;
}
void rb10::MoveTCPBlend_Clear(){
    QString text;
//    text.sprintf("blend_tcp clear_pt");
    text.sprintf("move_lb_clear()");
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    cmdSocket->waitForBytesWritten(1);
    cmdSocket->flush();
}
void rb10::MoveTCPBlend_AddPoint(float radius, float x, float y, float z, float rx, float ry, float rz, float spd, float acc){
    QString text;
//    text.sprintf("blend_tcp add_pt %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, radius, x, y, z, rx, ry, rz);
    text.sprintf("move_lb_add(pnt[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],%.3f)",x,y,z,rx,ry,rz,radius);
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    cmdSocket->waitForBytesWritten(1);
    cmdSocket->flush();
    systemStat.sdata.robot_state = 3; //run
}
void rb10::MoveTCPBlend_MovePoint(){
    QString text;
//    text.sprintf("blend_tcp move_pt");
    text.sprintf("move_lb_run(100, 100, 1)");
    moveCmdFlag = true;
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    cmdSocket->waitForBytesWritten(1);
    cmdSocket->flush();
    systemStat.sdata.robot_state = 3;
}
void rb10::ControlBoxDigitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15){
    QString text;
    text.sprintf("digital_out %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15);
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    cmdSocket->flush();
}
void rb10::ControlBoxAnalogOut(float a0, float a1, float a2, float a3){
    QString text;
    text.sprintf("analog_out %.3f, %.3f, %.3f, %.3f", a0, a1, a2, a3);
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
}
void rb10::ToolOut(int volt, int d0, int d1){
    int temp_volt = volt;
    if((temp_volt != 12) && (temp_volt != 24))
        temp_volt = 0;

    QString text;
    text.sprintf("tool_out %d, %d, %d", temp_volt, d0, d1);
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
}
void rb10::BaseSpeedChange(float spd){
    QString text;
    if(spd > 1.0)
        spd = 1.0;
    if(spd < 0.0)
        spd = 0.0;
    text.sprintf("sdw default_speed %.3f", spd);
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
}
void rb10::MotionPause(){
    QString text;
    text.sprintf("task pause");
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
}
void rb10::MotionHalt(){
    QString text;
    text.sprintf("task stop");
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
}
void rb10::MotionResume(){
    QString text;
    text.sprintf("task resume_a");
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
}
void rb10::CollisionResume(){
    QString text;
    text.sprintf("task resume_b");
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
}

void rb10::MoveTCPitpl_Clear(){
    QString text;
    text.sprintf("move_itpl_clear()");
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
}
void rb10::MoveTCPitpl_AddPoint(float vel, float x, float y, float z, float rx, float ry, float rz, float spd, float acc){
    QString text;
//    text.sprintf("blend_tcp add_pt %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, radius, x, y, z, rx, ry, rz);
    text.sprintf("move_itpl_add(pnt[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],%.3f)",x,y,z,rx,ry,rz,vel);
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    systemStat.sdata.robot_state = 3; //run
}
void rb10::MoveTCPitpl_MovePoint(){
    QString text;
//    text.sprintf("blend_tcp move_pt");
    text.sprintf("move_itpl_run(300, 3, 1)");
    moveCmdFlag = true;
    cmdConfirmFlag = false;
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    systemStat.sdata.robot_state = 3;
}

void rb10::RobotInfo(){
    dataSocket->write("reqdata");
    dataSocket->waitForReadyRead(1);

    QByteArray datas = dataSocket->readAll();
    memcpy(&systemStat,datas.data(),sizeof(systemSTAT));

    for(int i=0;i<6;i++)
    {
        Uidata->Jnt[i]=systemStat.sdata.jnt_ang[i];
        Uidata->TCPpos[i]=systemStat.sdata.tcp_pos[i];

    }
    if(systemStat.sdata.robot_state==1){
        Uidata->RobotState=1; //Idle
    }
    else if(systemStat.sdata.robot_state==3){
        Uidata->RobotState=2;//move
    }
    else{
        Uidata->RobotState=0;//else
    }
//    systemStat.sdata.digital_in[0]

//    qDebug() <<datas;

}

//void rb10::ControlBoxDigitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15){
//    QString text;
//    text.sprintf("digital_out %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15);
//    cmdConfirmFlag = false;
//    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
//}

