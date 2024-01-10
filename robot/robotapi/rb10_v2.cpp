#include "rb10_v2.h"

/* mm,deg,rpy*/
rb10::rb10() : robot()
{

}

rb10::~rb10()
{

}



void rb10::ControlBoxDigitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15){

    char buf[200];
    sprintf(buf,"digital_out %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15);
    cmdSocket->Write(buf);
}

//vector<bool> rb10::ControlBoxDigitalIn()
//{
//    vector<bool> temp;
//    for(int i=0;i<16;i++){
//        temp.push_back(systemStat.sdata.digital_in[i]);
//    }
//    return temp;
//}
int rb10::ControlBoxDigitalIn()
{
    int temp;
    for(int i=0;i<16;i++){
        temp+=systemStat.sdata.digital_in[i]<<i;
    }
    return temp;
}

void rb10::InitSocket(TCPClient *sock1,TCPClient *sock2,ReciveData *Info,int cord_type){
    cmdSocket=sock1;
    dataSocket=sock2;
    Uidata=Info;
    Conf.rottype=cord_type;

}

//void rb10::InitSocket(TCPClient *sock1,TCPClient *sock2){
//    cmdSocket=sock1;
//    dataSocket=sock2;
//}

void rb10::MoveL(WayPoints *data)
{
    double x,y,z;
    double alpha,beta,gamma;

    x = data->Pnt[0].position[0]*1000;
    y = data->Pnt[0].position[1]*1000;
    z = data->Pnt[0].position[2]*1000;

    Conf.InverseRot(data->Pnt[0].R,&alpha,&beta,&gamma);

    if(movestate==0){
        MoveTCP(x,y,z,alpha,beta,gamma,vel_l);
        RecvSkipCnt=5;
        systemStat.sdata.robot_state=3;
        movestate=1;
    }

//    while(systemStat.sdata.robot_state==3){

//    }

}

void rb10::MoveL_TCP(){

}
void rb10::MoveJ(WayPoints *data)
{
    double j1,j2,j3,j4,j5,j6;

    j1 = data->Jnt[0].value[0]*180/M_PI;
    j2 = data->Jnt[0].value[1]*180/M_PI;
    j3 = data->Jnt[0].value[2]*180/M_PI;
    j4 = data->Jnt[0].value[3]*180/M_PI;
    j5 = data->Jnt[0].value[4]*180/M_PI;
    j6 = data->Jnt[0].value[5]*180/M_PI;


    if(movestate==0){
        MoveJoint(j1,j2,j3,j4,j5,j6,vel_j,acc_j);
        RecvSkipCnt=5;
        systemStat.sdata.robot_state=3;
        movestate=1;
    }
//    while(systemStat.sdata.robot_state==3){

//    }

}

void rb10::MoveJ(WayPoints *data, double acc)
{
    if(acc<0) acc=-1;
    double j1,j2,j3,j4,j5,j6;

    j1 = data->Jnt[0].value[0]*180/M_PI;
    j2 = data->Jnt[0].value[1]*180/M_PI;
    j3 = data->Jnt[0].value[2]*180/M_PI;
    j4 = data->Jnt[0].value[3]*180/M_PI;
    j5 = data->Jnt[0].value[4]*180/M_PI;
    j6 = data->Jnt[0].value[5]*180/M_PI;


    if(movestate==0){
        MoveJoint(j1,j2,j3,j4,j5,j6,vel_j,acc);
        RecvSkipCnt=5;
        systemStat.sdata.robot_state=3;
        movestate=1;
    }
}

void rb10::MoveB(WayPoints *data)
{
    double x,y,z;
    double alpha,beta,gamma;

    data->blend=data->blend*1000;
    MoveTCPBlend_Clear();
    for(int i=0;i<data->NUM_PNT;i++)
    {
        x = data->Pnt[i].position[0]*1000;
        y = data->Pnt[i].position[1]*1000;
        z = data->Pnt[i].position[2]*1000;



        Conf.InverseRot(data->Pnt[i].R,&alpha,&beta,&gamma);
       MoveTCPBlend_AddPoint(data->blend,x,y,z,alpha,beta,gamma,vel_l,-1);
    }
    MoveTCPBlend_MovePoint();
    RecvSkipCnt=5;
    systemStat.sdata.robot_state=3;
//    while((!RecvSkipCnt)&&((Uidata->RobotState)==2)){

//    }


}

void rb10::MoveC(WayPoints *data)
{
    double x1,y1,z1,x2,y2,z2;
    double alpha1,beta1,gamma1,alpha2,beta2,gamma2;

    x1 = data->Pnt[0].position[0]*1000;
    y1 = data->Pnt[0].position[1]*1000;
    z1 = data->Pnt[0].position[2]*1000;

    x2 = data->Pnt[1].position[0]*1000;
    y2 = data->Pnt[1].position[1]*1000;
    z2 = data->Pnt[1].position[2]*1000;

    Conf.InverseRot(data->Pnt[0].R,&alpha1,&beta1,&gamma1);
    Conf.InverseRot(data->Pnt[1].R,&alpha2,&beta2,&gamma2);

//    move_c_points(x1,y1,z1,alpha1,beta1,gamma1,x2,y2,z2,alpha2,beta2,gamma2,vel_l,-1,2);
    if(movestate==0){
        move_c_points(x1,y1,z1,alpha1,beta1,gamma1,x2,y2,z2,alpha2,beta2,gamma2,vel_l,-1,0);
        RecvSkipCnt=5;
        systemStat.sdata.robot_state=3;
        movestate=1;
    }
}

void rb10::move_c_points(double x1,double y1,double z1,
                         double rx1,double ry1,double rz1,
                         double x2,double y2,double z2,
                         double rx2,double ry2,double rz2,
                         double spd,double acc,int opt){
    char buf[1024];
//    cout<<"movetcp"<<endl;
    sprintf(buf,"move_c_points(pnt[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
                "pnt[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
                "%.3f,%.3f,%d)",x1,y1,z1,rx1,ry1,rz1,x2,y2,z2,rx2,ry2,rz2,spd,acc,opt);


    cmdSocket->Write(buf);
}
void rb10::CobotInit(){

    char buf[200];
    sprintf(buf,"mc jall init");
    cmdSocket->Write(buf);
    while(systemStat.sdata.init_state_info!=INIT_STAT_INFO_INIT_DONE)
    {

    }
    usleep(10000);
    sprintf(buf,"pgmode real");
    cmdSocket->Write(buf);
    while(systemStat.sdata.program_mode!=0);
    {
    }
    usleep(10000);
    if(systemStat.sdata.default_speed!=1);
    {
        sprintf(buf,"sdw default_speed 1");
        cmdSocket->Write(buf);
    }
    usleep(10000);
}

void rb10::MoveTCP(float x, float y, float z, float rx, float ry, float rz, float spd, float acc){

    char buf[200];
//    cout<<"movetcp"<<endl;
    sprintf(buf,"move_l(pnt[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],%.3f,%.3f)",x,y,z,rx,ry,rz,spd,acc);

    cmdSocket->Write(buf);
}

void rb10::ControlBoxDigitalOut(int out)
{
    int dout[16];
    int temp=1;
    for(int i=0;i<16;i++)
    {
        dout[i]=(out&(temp<<i))>i;
//        cout<<"dout"<<i<<":"<<dout[i]<<endl;
    }

    ControlBoxDigitalOut(dout[0],dout[1],dout[2],dout[3],
            dout[4],dout[5],dout[6],dout[7],
            dout[8],dout[9],dout[10],dout[11],
            dout[12],dout[13],dout[14],dout[15]);
}

void rb10::MoveJoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd, float acc){

    char buf[200];
    sprintf(buf,"jointall %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, joint1, joint2, joint3, joint4, joint5, joint6);

    cmdSocket->Write(buf);
}


void rb10::RobotInfo(){
//    cout<<"rb info"<<endl;
//    cout<<"rb recv skip"<<RecvSkipCnt<<endl;
    char *datas;
    dataSocket->Write("reqdata");
//    dataSocket->Write("give");
    datas=dataSocket->Read();


    if(RecvSkipCnt==0){
        memcpy(&systemStat,datas,sizeof(systemSTAT));
    }

    for(int i=0;i<6;i++)
    {
        Uidata->Jnt[i]=systemStat.sdata.jnt_ang[i]*M_PI/180;
    }
    Uidata->TCPpos[0]=systemStat.sdata.tcp_pos[0]/1000;
    Uidata->TCPpos[1]=systemStat.sdata.tcp_pos[1]/1000;
    Uidata->TCPpos[2]=systemStat.sdata.tcp_pos[2]/1000;
    Uidata->TCPpos[3]=systemStat.sdata.tcp_pos[3]*M_PI/180;
    Uidata->TCPpos[4]=systemStat.sdata.tcp_pos[4]*M_PI/180;
    Uidata->TCPpos[5]=systemStat.sdata.tcp_pos[5]*M_PI/180;
    if(systemStat.sdata.robot_state==1){
        Uidata->RobotState=1; //Idle
    }
    else if(systemStat.sdata.robot_state==3){
        Uidata->RobotState=2;//move
    }
    else{
        Uidata->RobotState=0;//else
    }

    if(RecvSkipCnt>0) { RecvSkipCnt--  ;}
    if(systemStat.sdata.robot_state!=3){
         movestate=0;
    }
}

void rb10::MoveTCPBlend_Clear(){

    char buf[200];
//    sprintf(buf,"move_lb_clear()");
    sprintf(buf,"move_pb_clear()");
    cmdSocket->Write(buf);
}


void rb10::MoveTCPBlend_AddPoint(float radius, float x, float y, float z, float rx, float ry, float rz, float spd, float acc){

    char buf[200];
//    sprintf(buf,"move_lb_add(pnt[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],%.3f)",x,y,z,rx,ry,rz,radius);
    sprintf(buf,"move_pb_add(pnt[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],%.3f,1,%.3f)",x,y,z,rx,ry,rz,spd,radius);

    systemStat.sdata.robot_state = 3; //run
    cmdSocket->Write(buf);


}

void rb10::MoveTCPBlend_MovePoint(){
    char buf[200];
//    sprintf(buf,"move_lb_run(100, 100, 1)");
    sprintf(buf,"move_pb_run(%.3f, 0)",vel_l*2);
    systemStat.sdata.robot_state = 3; //run
    cmdSocket->Write(buf);
}

void rb10::SetVelocity(double v){
//    if(v<0){
//        v=0;
//    }
//    else if(v>100){
//        v=100;
//    }
//    vel_j=191*v/100;
    acc_j=v/400;
    vel_j=v/5;
    vel_l=300*v/50;
}

void rb10::Stop(){ 
    char buf[200];
    sprintf(buf,"halt");
    systemStat.sdata.robot_state = 3; //run
    cmdSocket->Write(buf);
}
bool rb10::WaitMove(){
    Uidata->RobotState=2;

    while(Uidata->RobotState!=1){

    }
//    if(systemStat.sdata.robot_state!=3){
//         movestate=0;
//    }
    movestate=0;
//    cout<<"wait move "<<movestate<<endl;
    return movestate;
}
