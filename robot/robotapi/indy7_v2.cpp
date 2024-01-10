//#include "indy7_v2.h"

//indy7::indy7() : robot()
//{

//    invoke=0;
//    strcpy(msg.DCP.header.robotName,"NRMK-Indy7");

//    msg.DCP.header.stepInfo=0x02;
//    msg.DCP.header.sof=0x34;

//    strcpy(Extmsg.value.header.robotName,"NRMK-Indy7");
//    Extmsg.value.header.stepInfo=0;
//    Extmsg.value.header.sof=0x34;

//}

//indy7::~indy7()
//{

//}

//void indy7::InitSocket(TCPClient *sock1, TCPClient *sock2, ReciveData *Info, int cord_type)
//{
//    InitSocket(sock1,Info,cord_type);
//}

//void indy7::InitSocket(TCPClient *sock,ReciveData *Info,int cord_type){

//    IndyTCP=sock;
//    RecvData=Info;

//}

//void indy7::MoveL(WayPoints *data)
//{
//    double x,y,z;
//    double alpha,beta,gamma;

//    x = data->Pnt[0].position[0];
//    y = data->Pnt[0].position[1];
//    z = data->Pnt[0].position[2];
//    Conf.InverseRot(data->Pnt[0].R,&alpha,&beta,&gamma);
//    moveTaskTo(x,y,z,alpha,beta,gamma);


//}

//void indy7::MoveJ(WayPoints *data)
//{
//    double j1,j2,j3,j4,j5,j6;

//    j1 = data->Jnt[0].value[0]*180/M_PI;
//    j2 = data->Jnt[0].value[1]*180/M_PI;
//    j3 = data->Jnt[0].value[2]*180/M_PI;
//    j4 = data->Jnt[0].value[3]*180/M_PI;
//    j5 = data->Jnt[0].value[4]*180/M_PI;
//    j6 = data->Jnt[0].value[5]*180/M_PI;

//    JointTaskTo(j1,j2,j3,j4,j5,j6);

//}

//void indy7::MoveB(WayPoints *data)
//{
//    double x,y,z;
//    double alpha,beta,gamma;

//    ClearTaskWaypointSet();

//    for(int i=0;i<data->NUM_PNT;i++)
//    {

//        x = data->Pnt[i].position[0];
//        y = data->Pnt[i].position[1];
//        z = data->Pnt[i].position[2];

//        Conf.InverseRot(data->Pnt[i].R,&alpha,&beta,&gamma);
//        AddTaskWaypointSet(x,y,z,alpha,beta,gamma,data->blend);

//    }
//    ExecuteTaskWaypointSet();
//}

//void indy7::SetDefaultTCP(cord TCPcord){

//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=48;
//    msg.DCP.header.cmdId=100;

//    /*x y z scale : m*/
//    msg.DCP.data.doubleData[0]=TCPcord.value[0];
//    msg.DCP.data.doubleData[1]=TCPcord.value[1];
//    msg.DCP.data.doubleData[2]=TCPcord.value[2];
//    msg.DCP.data.doubleData[3]=TCPcord.value[3];
//    msg.DCP.data.doubleData[4]=TCPcord.value[4];
//    msg.DCP.data.doubleData[5]=TCPcord.value[5];

////    IndyTCP->Write(msg.byte,56+msg.DCP.header.dataSize);
////    IndyTCP->waitForReadyRead(10);
////    IndyTCP->readAll();
////    IndyTCP->waitForReadyRead(10);
////    IndyTCP->readAll();
////    IndyTCP->flush();
//    IndyTCP->Write(msg.byte);
//    IndyTCP->Read();
//    IndyTCP->Read();

//}

//void indy7::moveTaskTo(double x, double y, double z, double u, double v, double w){

//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=48;
//    msg.DCP.header.cmdId=11;

//    /*x y z scale : m*/
//    msg.DCP.data.doubleData[0]=x;
//    msg.DCP.data.doubleData[1]=y;
//    msg.DCP.data.doubleData[2]=z;
//    msg.DCP.data.doubleData[3]=u;
//    msg.DCP.data.doubleData[4]=v;
//    msg.DCP.data.doubleData[5]=w;

////    char _buf[8*480000];
////     memcpy(_buf,msg.byte,56+msg.DCP.header.dataSize);
//    IndyTCP->Write(msg.byte,56+msg.DCP.header.dataSize);
//    IndyTCP->Read();
//    IndyTCP->Read();
//}
//void indy7::JointTaskTo(double j1, double j2, double j3, double j4, double j5, double j6){

//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=48;
//    msg.DCP.header.cmdId=9;

//    /*x y z scale : m*/
//    msg.DCP.data.doubleData[0]=j1;
//    msg.DCP.data.doubleData[1]=j2;
//    msg.DCP.data.doubleData[2]=j3;
//    msg.DCP.data.doubleData[3]=j4;
//    msg.DCP.data.doubleData[4]=j5;
//    msg.DCP.data.doubleData[5]=j6;

//    IndyTCP->Write(msg.byte);
//    IndyTCP->Read();
//    IndyTCP->Read();
//}
//void indy7::AddTaskWaypointSet(double x, double y,double z,double u,double v,double w,double r ){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=64;
//    msg.DCP.header.cmdId=95;

//    msg.DCP.data.doubleData[0]=0;
//    msg.DCP.data.doubleData[1]=r;
//    msg.DCP.data.doubleData[2]=x;
//    msg.DCP.data.doubleData[3]=y;
//    msg.DCP.data.doubleData[4]=z;
//    msg.DCP.data.doubleData[5]=u;
//    msg.DCP.data.doubleData[6]=v;
//    msg.DCP.data.doubleData[7]=w;

////    IndyTCP->write(msg.byte,56+msg.DCP.header.dataSize);
////    IndyTCP->waitForBytesWritten(-1);
////    IndyTCP->flush();
////    IndyTCP->waitForReadyRead(-1);
////    IndyTCP->readAll();
//    IndyTCP->Write(msg.byte);
//    IndyTCP->Read();

//}

//void indy7::ClearTaskWaypointSet(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=0;
//    msg.DCP.header.cmdId=97;

//    IndyTCP->Write(msg.byte);
//    IndyTCP->Read();


//}


//void indy7::ExecuteTaskWaypointSet(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=16;
//    msg.DCP.header.cmdId=99;

//    msg.DCP.data.doubleData[0]=0;
//    msg.DCP.data.doubleData[1]=2;

//    IndyTCP->Write(msg.byte);
//    IndyTCP->Read();
//    IndyTCP->Read();


//}
//void indy7::SetTatskMoveBaseMode(int TaskMode){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=4;
//    msg.DCP.header.cmdId=113;

//    IndyTCP->Write(msg.byte);
//    IndyTCP->Read();
//    IndyTCP->Read();
//}

//void indy7::MoveTCP(float x, float y, float z, float rx, float ry, float rz, float spd, float acc)
//{
//    moveTaskTo(x,y,z,rx,ry,rz);
//}

//void indy7::SetServoOn(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=6;
//    msg.DCP.header.cmdId=3;

//    msg.DCP.data.charData[0]=1;
//    msg.DCP.data.charData[1]=1;
//    msg.DCP.data.charData[2]=1;
//    msg.DCP.data.charData[3]=1;
//    msg.DCP.data.charData[4]=1;
//    msg.DCP.data.charData[5]=1;

//    IndyTCP->Write(msg.byte);


//}
//void indy7::SetServoOff(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=6;
//    msg.DCP.header.cmdId=3;

//    msg.DCP.data.charData[0]=0;
//    msg.DCP.data.charData[1]=0;
//    msg.DCP.data.charData[2]=0;
//    msg.DCP.data.charData[3]=0;
//    msg.DCP.data.charData[4]=0;
//    msg.DCP.data.charData[5]=0;

//    IndyTCP->Write(msg.byte);

//}
//void indy7::SetBrakeOn(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=6;
//    msg.DCP.header.cmdId=4;

//    msg.DCP.data.charData[0]=1;
//    msg.DCP.data.charData[1]=1;
//    msg.DCP.data.charData[2]=1;
//    msg.DCP.data.charData[3]=1;
//    msg.DCP.data.charData[4]=1;
//    msg.DCP.data.charData[5]=1;


//    IndyTCP->Write(msg.byte);

//}
//void indy7::SetBrakeOff(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=6;
//    msg.DCP.header.cmdId=4;

//    msg.DCP.data.charData[0]=0;
//    msg.DCP.data.charData[1]=0;
//    msg.DCP.data.charData[2]=0;
//    msg.DCP.data.charData[3]=0;
//    msg.DCP.data.charData[4]=0;
//    msg.DCP.data.charData[5]=0;

//    IndyTCP->Write(msg.byte);
//}

//void indy7::JointMoveTo(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=48;
//    msg.DCP.header.cmdId=9;

//    msg.DCP.data.doubleData[0]=0;
//    msg.DCP.data.doubleData[1]=0;
//    msg.DCP.data.doubleData[2]=-90;
//    msg.DCP.data.doubleData[3]=0;
//    msg.DCP.data.doubleData[4]=-90;
//    msg.DCP.data.doubleData[5]=0;

//    IndyTCP->Write(msg.byte);
//}

////void indy7::ext_trajmove(WayPoints *data){

////    double a,b,c;

////    Extmsg.value.header.invokeId=++invoke;
////    Extmsg.value.header.dataSize=8;
////    Extmsg.value.header.cmdId=800;

////    Extmsg.value.data.intData[0]=12; //Extended CommandID
////    Extmsg.value.data.intData[1]=48*(data->NUM_PNT);

//////    Extmsg.value.data.intData[0]=1; //Extended CommandID
//////    Extmsg.value.data.intData[1]=48*3*cnt+20;


////    for(int i=0;i<data->NUM_PNT;i++){

////    Conf.InverseRot(data->Pnt[i].R,&a,&b,&c);
////    Extmsg.value.data.doubleData[1+6*i]=data->Pnt[i].position[0]/1000;
////    Extmsg.value.data.doubleData[2+6*i]=data->Pnt[i].position[1]/1000;
////    Extmsg.value.data.doubleData[3+6*i]=data->Pnt[i].position[2]/1000;
////    Extmsg.value.data.doubleData[4+6*i]=a;
////    Extmsg.value.data.doubleData[5+6*i]=b;
////    Extmsg.value.data.doubleData[6+6*i]=c;
////    }
//////    Extmsg.value.data.intData[2]=2;
//////    Extmsg.value.data.intData[3]=4000;
//////    Extmsg.value.data.intData[4]=1;
//////    Extmsg.value.data.intData[5]=6;
//////    Extmsg.value.data.intData[6]=cnt;

//////    for(int i=0;i<cnt;i++){

//////    Extmsg.value.data.intData[7+12*i]=(((double)550/1000)&&0xFFFFFFFF00000000)>>32;
//////    Extmsg.value.data.intData[8+12*i]=((double)550/1000);
//////    Extmsg.value.data.intData[9+12*i]=(((double)0/1000)&&0xFFFFFFFF00000000)>>32;
//////    Extmsg.value.data.intData[10+12*i]=((double)0/1000);
//////    Extmsg.value.data.intData[11+12*i]=(((double)(400+i*20)/1000)&&0xFFFFFFFF00000000)>>32;
//////    Extmsg.value.data.intData[12+12*i]=((double)(400+i*20)/1000);
//////    Extmsg.value.data.intData[13+12*i]=(((double)(0+i*20)/1000)&&0xFFFFFFFF00000000)>>32;
//////    Extmsg.value.data.intData[14+12*i]=((double)0/1000);
//////    Extmsg.value.data.intData[15+12*i]=(((double)180/1000)&&0xFFFFFFFF00000000)>>32;
//////    Extmsg.value.data.intData[16+12*i]=((double)180/1000);
//////    Extmsg.value.data.intData[17+12*i]=(((double)0/1000)&&0xFFFFFFFF00000000)>>32;
//////    Extmsg.value.data.intData[18+12*i]=((double)0/1000);

//////   /*vel*/
//////    Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
//////    Extmsg.value.data.intData[1+6*i]=((double)0/1000);
//////    Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
//////    Extmsg.value.data.intData[1+6*i]=((double)0/1000);
//////    Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
//////    Extmsg.value.data.intData[1+6*i]=((double)0/1000);
//////    Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
//////    Extmsg.value.data.intData[1+6*i]=((double)0/1000);
//////    Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
//////    Extmsg.value.data.intData[1+6*i]=((double)0/1000);
//////    Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
//////    Extmsg.value.data.intData[1+6*i]=((double)0/1000);

//////    /*acc*/
//////     Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
//////     Extmsg.value.data.intData[1+6*i]=((double)0/1000);
//////     Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
//////     Extmsg.value.data.intData[1+6*i]=((double)0/1000);
//////     Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
//////     Extmsg.value.data.intData[1+6*i]=((double)0/1000);
//////     Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
//////     Extmsg.value.data.intData[1+6*i]=((double)0/1000);
//////     Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
//////     Extmsg.value.data.intData[1+6*i]=((double)0/1000);
//////     Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
//////     Extmsg.value.data.intData[1+6*i]=((double)0/1000);
//////    }

////    IndyTCP->write(Extmsg.byte,56+8+48*(data->NUM_PNT));
////    IndyTCP->waitForBytesWritten(-1);
////    IndyTCP->waitForReadyRead(-1);
////    IndyTCP->readAll();
////    IndyTCP->waitForReadyRead(-1);
////    IndyTCP->readAll();
////    IndyTCP->flush();
////}


//void indy7::GetTaskPos(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=0;
//    msg.DCP.header.cmdId=322;

////    IndyTCP->write(msg.byte,56+msg.DCP.header.dataSize);
////    IndyTCP->waitForBytesWritten(-1);
////    IndyTCP->flush();

//    char buf[200];
////    sprintf(buf,"move_l(pnt[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],%.3f,%.3f,1)",x,y,z,rx,ry,rz,spd,acc);
//    memcpy(buf,msg.byte,56+msg.DCP.header.dataSize);

//    IndyTCP->Write(buf);

//}

//void indy7::GetJointPos(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=0;
//    msg.DCP.header.cmdId=320;

////    IndyTCP->write(msg.byte,56+msg.DCP.header.dataSize);
////    IndyTCP->waitForBytesWritten(-1);
////    IndyTCP->flush();
//}

//void indy7::RobotIsBusy(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=0;
//    msg.DCP.header.cmdId=35;



////    IndyTCP->write(msg.byte,56+msg.DCP.header.dataSize);
////    IndyTCP->waitForBytesWritten(-1);

////    IndyTCP->flush();

//}
//void indy7::RobotFinishMotion(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=0;
//    msg.DCP.header.cmdId=31;

////    IndyTCP->write(msg.byte,56+msg.DCP.header.dataSize);
////    IndyTCP->waitForBytesWritten(-1);
////    IndyTCP->flush();
//}

//void indy7::RobotInfo(){

//    char *datas,*datas2;
//    int out=0;
//    bool ok;

//    GetTaskPos();

//    ///out|=1*(IndyTCP->waitForReadyRead(-1));

//    ///datas = IndyTCP->readAll();///
//    /// \brief memcpy
//    datas=IndyTCP->Read();

//    memcpy(&RecvHeader,datas,sizeof(datas));

//    if(RecvHeader.invokeId==msg.DCP.header.invokeId)
//    {

//        ///out|=2*(IndyTCP->waitForReadyRead(-1));

//        ///datas = IndyTCP->readAll();
//        datas=IndyTCP->Read();
//        memcpy(RecvData->TCPpos,datas,sizeof(datas));
//        //qDebug()<<"ok1";

//    }
//    else
//    {
//        //datas = IndyTCP->readAll();
//        datas=IndyTCP->Read();

//    }

//    GetJointPos();

//    ///out|=2*(IndyTCP->waitForReadyRead(-1));

//    ///datas = IndyTCP->readAll();
//    datas=IndyTCP->Read();

//    memcpy(&RecvHeader,datas,sizeof(datas));

//    if(RecvHeader.invokeId==msg.DCP.header.invokeId)
//    {
//        ///out|=2*(IndyTCP->waitForReadyRead(-1));

//        ///datas = IndyTCP->readAll();
//        datas=IndyTCP->Read();
//        memcpy(RecvData->Jnt,datas,sizeof(datas));
//        //qDebug()<<"ok2";
//    }
//    else
//    {
//        //datas = IndyTCP->readAll();
//        datas=IndyTCP->Read();
//    }


//    RobotFinishMotion();

//    ///out|=2*(IndyTCP->waitForReadyRead(-1));

//    ///datas = IndyTCP->readAll();
//    datas=IndyTCP->Read();
//    memcpy(&RecvHeader,datas,sizeof(datas));
//    if(RecvHeader.invokeId==msg.DCP.header.invokeId)
//    {
//        ///out|=2*(IndyTCP->waitForReadyRead(-1));

//        ///datas = IndyTCP->readAll();
//        datas=IndyTCP->Read();
//        //qDebug()<<"ok3";

//    }
//    else
//    {
//        //datas = IndyTCP->readAll();
//        datas=IndyTCP->Read();
//    }



//    RobotIsBusy();

//    ///out|=2*(IndyTCP->waitForReadyRead(-1));

//    ///datas = IndyTCP->readAll();
//    datas=IndyTCP->Read();
//    memcpy(&RecvHeader,datas2,sizeof(datas2));
//    if(RecvHeader.invokeId==msg.DCP.header.invokeId)
//    {
//        ///out|=2*(IndyTCP->waitForReadyRead(-1));

//        ///datas = IndyTCP->readAll();
//        datas=IndyTCP->Read();
//        //qDebug()<<"ok4";
//    }
//    else
//    {
//        //datas = IndyTCP->readAll();
//        datas=IndyTCP->Read();
//    }

//    \

////    if((datas2==1)) RecvData->RobotState=2;
////    else if((datas==1)) RecvData->RobotState=1;
////    else RecvData->RobotState=0;



//}

#include "indy7_v2.h"

indy7::indy7() : robot()
{

//    invoke=0;
    strcpy(msg.DCP.header.robotName,"NRMK-Indy7");

    msg.DCP.header.stepInfo=0;
    msg.DCP.header.sof=0x34;

    strcpy(Extmsg.value.header.robotName,"NRMK-Indy7");
    Extmsg.value.header.stepInfo=0;
    Extmsg.value.header.sof=0x34;

}

indy7::~indy7()
{

}

void indy7::InitSocket(TCPClient *sock1, TCPClient *sock2, ReciveData *Info, int cord_type)
{
    InitSocket(sock1,Info,cord_type);
    invoke=0;
}

void indy7::InitSocket(TCPClient *sock,ReciveData *Info,int cord_type){

    IndyTCP=sock;
    RecvData=Info;
    Conf.rottype=cord_type;

}

void indy7::MoveL(WayPoints *data)
{
    double x,y,z;
    double alpha,beta,gamma;

    x = data->Pnt[0].position[0];
    y = data->Pnt[0].position[1];
    z = data->Pnt[0].position[2];
    Conf.InverseRot(data->Pnt[0].R,&alpha,&beta,&gamma);
    while(ReadState){

    }
    MoveState=true;
    moveTaskTo(x,y,z,alpha,beta,gamma);
    MoveState=false;


}
void indy7::MoveJ(WayPoints *data)
{
    double j1,j2,j3,j4,j5,j6;

    j1 = data->Jnt[0].value[0]*180/M_PI;
    j2 = data->Jnt[0].value[1]*180/M_PI;
    j3 = data->Jnt[0].value[2]*180/M_PI;
    j4 = data->Jnt[0].value[3]*180/M_PI;
    j5 = data->Jnt[0].value[4]*180/M_PI;
    j6 = data->Jnt[0].value[5]*180/M_PI;

    while(ReadState){

    }
    MoveState=true;
    JointTaskTo(j1,j2,j3,j4,j5,j6);
    MoveState=false;

}
void indy7::MoveJ(WayPoints *data,double acc)
{
    double j1,j2,j3,j4,j5,j6;

    j1 = data->Jnt[0].value[0]*180/M_PI;
    j2 = data->Jnt[0].value[1]*180/M_PI;
    j3 = data->Jnt[0].value[2]*180/M_PI;
    j4 = data->Jnt[0].value[3]*180/M_PI;
    j5 = data->Jnt[0].value[4]*180/M_PI;
    j6 = data->Jnt[0].value[5]*180/M_PI;

    while(ReadState){

    }
    MoveState=true;
    JointTaskTo(j1,j2,j3,j4,j5,j6);
    MoveState=false;

}

void indy7::MoveB(WayPoints *data)
{
    double x,y,z;
    double alpha,beta,gamma;

    ClearTaskWaypointSet();

    for(int i=0;i<data->NUM_PNT;i++)
    {

        x = data->Pnt[i].position[0];
        y = data->Pnt[i].position[1];
        z = data->Pnt[i].position[2];

        Conf.InverseRot(data->Pnt[i].R,&alpha,&beta,&gamma);
        AddTaskWaypointSet(x,y,z,alpha,beta,gamma,data->blend);

    }
    
    ExecuteTaskWaypointSet();
}

void indy7::ControlBoxDigitalOut(int out)
{
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=32;
    msg.DCP.header.cmdId=403;

    msg.DCP.data.doubleData[0]=out&0x000f;
    msg.DCP.data.doubleData[1]=out&0x00f0;
    msg.DCP.data.doubleData[2]=out&0x0f00;
    msg.DCP.data.doubleData[3]=out&0xf000;


    IndyTCP->Write(msg.byte);
    IndyTCP->Read();
}

void indy7::SetDefaultTCP(cord TCPcord){

    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=48;
    msg.DCP.header.cmdId=100;

    /*x y z scale : m*/
    msg.DCP.data.doubleData[0]=TCPcord.value[0];
    msg.DCP.data.doubleData[1]=TCPcord.value[1];
    msg.DCP.data.doubleData[2]=TCPcord.value[2];
    msg.DCP.data.doubleData[3]=TCPcord.value[3];
    msg.DCP.data.doubleData[4]=TCPcord.value[4];
    msg.DCP.data.doubleData[5]=TCPcord.value[5];

//    IndyTCP->Write(msg.byte,56+msg.DCP.header.dataSize);
//    IndyTCP->waitForReadyRead(10);
//    IndyTCP->readAll();
//    IndyTCP->waitForReadyRead(10);
//    IndyTCP->readAll();
//    IndyTCP->flush();
    IndyTCP->Write(msg.byte);
    //IndyTCP->WaitWrite();
    IndyTCP->Read();
    //IndyTCP->WaitRead();
    IndyTCP->Read();
    //IndyTCP->WaitRead();
}

void indy7::moveTaskTo(double x, double y, double z, double u, double v, double w){

    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=48;
    msg.DCP.header.cmdId=11;

    /*x y z scale : m*/
    msg.DCP.data.doubleData[0]=x;
    msg.DCP.data.doubleData[1]=y;
    msg.DCP.data.doubleData[2]=z;
    msg.DCP.data.doubleData[3]=u;
    msg.DCP.data.doubleData[4]=v;
    msg.DCP.data.doubleData[5]=w;

//    char _buf[8*480000];
//     memcpy(_buf,msg.byte,56+msg.DCP.header.dataSize);
    cout<<"move Linear"<<endl;
    IndyTCP->Write(msg.byte,56+msg.DCP.header.dataSize);
    //IndyTCP->WaitWrite();
    IndyTCP->Read();
    //IndyTCP->WaitRead();

//    IndyTCP->Read();
//    //IndyTCP->WaitRead();
    cout<<"move end"<<endl;
}
void indy7::JointTaskTo(double j1, double j2, double j3, double j4, double j5, double j6){

    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=48;
    msg.DCP.header.cmdId=9;

    /*x y z scale : m*/
    msg.DCP.data.doubleData[0]=j1;
    msg.DCP.data.doubleData[1]=j2;
    msg.DCP.data.doubleData[2]=j3;
    msg.DCP.data.doubleData[3]=j4;
    msg.DCP.data.doubleData[4]=j5;
    msg.DCP.data.doubleData[5]=j6;
    cout<<"Move Joint"<<endl;
    IndyTCP->Write(msg.byte,56+msg.DCP.header.dataSize);
    //IndyTCP->WaitWrite();
    IndyTCP->Read();
    //IndyTCP->WaitRead();
//    IndyTCP->Read();
//    //IndyTCP->WaitRead();
}
void indy7::AddTaskWaypointSet(double x, double y,double z,double u,double v,double w,double r ){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=64;
    msg.DCP.header.cmdId=95;

    msg.DCP.data.doubleData[0]=0;
    msg.DCP.data.doubleData[1]=r;
    msg.DCP.data.doubleData[2]=x;
    msg.DCP.data.doubleData[3]=y;
    msg.DCP.data.doubleData[4]=z;
    msg.DCP.data.doubleData[5]=u;
    msg.DCP.data.doubleData[6]=v;
    msg.DCP.data.doubleData[7]=w;

//    IndyTCP->write(msg.byte,56+msg.DCP.header.dataSize);
//    IndyTCP->waitForBytesWritten(-1);
//    IndyTCP->flush();
//    IndyTCP->waitForReadyRead(-1);
//    IndyTCP->readAll();
    IndyTCP->Write(msg.byte);
    //IndyTCP->WaitWrite();
    IndyTCP->Read();
    //IndyTCP->WaitRead();

}

void indy7::ClearTaskWaypointSet(){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=0;
    msg.DCP.header.cmdId=97;

    IndyTCP->Write(msg.byte);
    //IndyTCP->WaitWrite();
    IndyTCP->Read();
    //IndyTCP->WaitRead();

}


void indy7::ExecuteTaskWaypointSet(){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=16;
    msg.DCP.header.cmdId=99;

    msg.DCP.data.doubleData[0]=0;
    msg.DCP.data.doubleData[1]=2;

    IndyTCP->Write(msg.byte);
    //IndyTCP->WaitWrite();
    IndyTCP->Read();
    //IndyTCP->WaitRead();
    IndyTCP->Read();
    //IndyTCP->WaitRead();


}
void indy7::SetTatskMoveBaseMode(int TaskMode){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=4;
    msg.DCP.header.cmdId=113;

    IndyTCP->Write(msg.byte);
    //IndyTCP->WaitWrite();
    IndyTCP->Read();
    //IndyTCP->WaitRead();
    IndyTCP->Read();
    //IndyTCP->WaitRead();
}

void indy7::MoveTCP(float x, float y, float z, float rx, float ry, float rz, float spd, float acc)
{
    moveTaskTo(x,y,z,rx,ry,rz);
}

void indy7::SetServoOn(){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=6;
    msg.DCP.header.cmdId=3;

    msg.DCP.data.charData[0]=1;
    msg.DCP.data.charData[1]=1;
    msg.DCP.data.charData[2]=1;
    msg.DCP.data.charData[3]=1;
    msg.DCP.data.charData[4]=1;
    msg.DCP.data.charData[5]=1;

    IndyTCP->Write(msg.byte);
    //IndyTCP->WaitWrite();


}
void indy7::SetServoOff(){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=6;
    msg.DCP.header.cmdId=3;

    msg.DCP.data.charData[0]=0;
    msg.DCP.data.charData[1]=0;
    msg.DCP.data.charData[2]=0;
    msg.DCP.data.charData[3]=0;
    msg.DCP.data.charData[4]=0;
    msg.DCP.data.charData[5]=0;

    IndyTCP->Write(msg.byte);
    //IndyTCP->WaitWrite();

}
void indy7::SetBrakeOn(){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=6;
    msg.DCP.header.cmdId=4;

    msg.DCP.data.charData[0]=1;
    msg.DCP.data.charData[1]=1;
    msg.DCP.data.charData[2]=1;
    msg.DCP.data.charData[3]=1;
    msg.DCP.data.charData[4]=1;
    msg.DCP.data.charData[5]=1;


    IndyTCP->Write(msg.byte);
    //IndyTCP->WaitWrite();

}
void indy7::SetBrakeOff(){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=6;
    msg.DCP.header.cmdId=4;

    msg.DCP.data.charData[0]=0;
    msg.DCP.data.charData[1]=0;
    msg.DCP.data.charData[2]=0;
    msg.DCP.data.charData[3]=0;
    msg.DCP.data.charData[4]=0;
    msg.DCP.data.charData[5]=0;

    IndyTCP->Write(msg.byte);
    //IndyTCP->WaitWrite();
}

void indy7::JointMoveTo(){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=48;
    msg.DCP.header.cmdId=9;

    msg.DCP.data.doubleData[0]=0;
    msg.DCP.data.doubleData[1]=0;
    msg.DCP.data.doubleData[2]=-90;
    msg.DCP.data.doubleData[3]=0;
    msg.DCP.data.doubleData[4]=-90;
    msg.DCP.data.doubleData[5]=0;

    IndyTCP->Write(msg.byte);
    //IndyTCP->WaitWrite();
}

//void indy7::ext_trajmove(WayPoints *data){

//    double a,b,c;

//    Extmsg.value.header.invokeId=++invoke;
//    Extmsg.value.header.dataSize=8;
//    Extmsg.value.header.cmdId=800;

//    Extmsg.value.data.intData[0]=12; //Extended CommandID
//    Extmsg.value.data.intData[1]=48*(data->NUM_PNT);

////    Extmsg.value.data.intData[0]=1; //Extended CommandID
////    Extmsg.value.data.intData[1]=48*3*cnt+20;


//    for(int i=0;i<data->NUM_PNT;i++){

//    Conf.InverseRot(data->Pnt[i].R,&a,&b,&c);
//    Extmsg.value.data.doubleData[1+6*i]=data->Pnt[i].position[0]/1000;
//    Extmsg.value.data.doubleData[2+6*i]=data->Pnt[i].position[1]/1000;
//    Extmsg.value.data.doubleData[3+6*i]=data->Pnt[i].position[2]/1000;
//    Extmsg.value.data.doubleData[4+6*i]=a;
//    Extmsg.value.data.doubleData[5+6*i]=b;
//    Extmsg.value.data.doubleData[6+6*i]=c;
//    }
////    Extmsg.value.data.intData[2]=2;
////    Extmsg.value.data.intData[3]=4000;
////    Extmsg.value.data.intData[4]=1;
////    Extmsg.value.data.intData[5]=6;
////    Extmsg.value.data.intData[6]=cnt;

////    for(int i=0;i<cnt;i++){

////    Extmsg.value.data.intData[7+12*i]=(((double)550/1000)&&0xFFFFFFFF00000000)>>32;
////    Extmsg.value.data.intData[8+12*i]=((double)550/1000);
////    Extmsg.value.data.intData[9+12*i]=(((double)0/1000)&&0xFFFFFFFF00000000)>>32;
////    Extmsg.value.data.intData[10+12*i]=((double)0/1000);
////    Extmsg.value.data.intData[11+12*i]=(((double)(400+i*20)/1000)&&0xFFFFFFFF00000000)>>32;
////    Extmsg.value.data.intData[12+12*i]=((double)(400+i*20)/1000);
////    Extmsg.value.data.intData[13+12*i]=(((double)(0+i*20)/1000)&&0xFFFFFFFF00000000)>>32;
////    Extmsg.value.data.intData[14+12*i]=((double)0/1000);
////    Extmsg.value.data.intData[15+12*i]=(((double)180/1000)&&0xFFFFFFFF00000000)>>32;
////    Extmsg.value.data.intData[16+12*i]=((double)180/1000);
////    Extmsg.value.data.intData[17+12*i]=(((double)0/1000)&&0xFFFFFFFF00000000)>>32;
////    Extmsg.value.data.intData[18+12*i]=((double)0/1000);

////   /*vel*/
////    Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
////    Extmsg.value.data.intData[1+6*i]=((double)0/1000);
////    Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
////    Extmsg.value.data.intData[1+6*i]=((double)0/1000);
////    Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
////    Extmsg.value.data.intData[1+6*i]=((double)0/1000);
////    Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
////    Extmsg.value.data.intData[1+6*i]=((double)0/1000);
////    Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
////    Extmsg.value.data.intData[1+6*i]=((double)0/1000);
////    Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
////    Extmsg.value.data.intData[1+6*i]=((double)0/1000);

////    /*acc*/
////     Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
////     Extmsg.value.data.intData[1+6*i]=((double)0/1000);
////     Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
////     Extmsg.value.data.intData[1+6*i]=((double)0/1000);
////     Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
////     Extmsg.value.data.intData[1+6*i]=((double)0/1000);
////     Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
////     Extmsg.value.data.intData[1+6*i]=((double)0/1000);
////     Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
////     Extmsg.value.data.intData[1+6*i]=((double)0/1000);
////     Extmsg.value.data.intData[1+6*i]=((double)0/1000)>>4;
////     Extmsg.value.data.intData[1+6*i]=((double)0/1000);
////    }

//    IndyTCP->write(Extmsg.byte,56+8+48*(data->NUM_PNT));
//    IndyTCP->waitForBytesWritten(-1);
//    IndyTCP->waitForReadyRead(-1);
//    IndyTCP->readAll();
//    IndyTCP->waitForReadyRead(-1);
//    IndyTCP->readAll();
//    IndyTCP->flush();
//}


void indy7::GetTaskPos(){

    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=0;
    msg.DCP.header.cmdId=322;

//    IndyTCP->write(msg.byte,56+msg.DCP.header.dataSize);
//    IndyTCP->waitForBytesWritten(-1);
//    IndyTCP->flush();

    char buf[200];
//    sprintf(buf,"move_l(pnt[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],%.3f,%.3f,1)",x,y,z,rx,ry,rz,spd,acc);
//    memcpy(buf,msg.byte,56+msg.DCP.header.dataSize);
    cout<<"send id"<<msg.DCP.header.invokeId<<endl;
//    IndyTCP->Write(buf,56+msg.DCP.header.dataSize);
    IndyTCP->Write(msg.byte,56+msg.DCP.header.dataSize);
     //IndyTCP->WaitWrite();
}

void indy7::GetJointPos(){
//    invoke=invoke++;
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=0;
    msg.DCP.header.cmdId=320;

//    IndyTCP->write(msg.byte,56+msg.DCP.header.dataSize);
//    IndyTCP->waitForBytesWritten(-1);
//    IndyTCP->flush();
    char buf[200];
//    sprintf(buf,"move_l(pnt[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],%.3f,%.3f,1)",x,y,z,rx,ry,rz,spd,acc);
    memcpy(buf,msg.byte,56+msg.DCP.header.dataSize);

    IndyTCP->Write(buf,56+msg.DCP.header.dataSize);
     //IndyTCP->WaitWrite();
}

void indy7::RobotIsBusy(){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=0;
    msg.DCP.header.cmdId=35;



    IndyTCP->Write(msg.byte,56+msg.DCP.header.dataSize);
    //IndyTCP->WaitWrite();
//    IndyTCP->waitForBytesWritten(-1);

//    IndyTCP->flush();

}
void indy7::RobotFinishMotion(){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=0;
    msg.DCP.header.cmdId=31;

    IndyTCP->Write(msg.byte,56+msg.DCP.header.dataSize);
    //IndyTCP->WaitWrite();
//    IndyTCP->waitForBytesWritten(-1);
//    IndyTCP->flush();
}

void indy7::RobotInfo(){

    while(MoveState){

    }
    ReadState=true;
    char *datas,*datas2;
    int out=0;
    bool ok;

    cout<<"------------------"<<endl;
    GetTaskPos();



    datas=IndyTCP->Read();
    //IndyTCP->WaitRead();
    memcpy(&RecvHeader,datas,56);

//    cout<<RecvHeader.robotName<<endl;
//    cout<<RecvHeader.dataSize<<endl;
//    cout<<RecvHeader.invokeId<<endl;
//    cout<<msg.DCP.header.invokeId<<endl;
//    cout<<RecvHeader.robotVersion<<endl;

    if(RecvHeader.invokeId==msg.DCP.header.invokeId)
    {

        datas=IndyTCP->Read();
        //IndyTCP->WaitRead();
        memcpy(RecvData->TCPpos,datas,RecvHeader.dataSize);


    }
    else
    {

        datas=IndyTCP->Read();
        //IndyTCP->WaitRead();

    }
    cout<<"+++++++++++++++++++++"<<endl;
    GetJointPos();

    datas=IndyTCP->Read();
    //IndyTCP->WaitRead();
    memcpy(&RecvHeader,datas,56);
//    cout<<RecvHeader.robotName<<endl;
//    cout<<RecvHeader.dataSize<<endl;
//    cout<<RecvHeader.invokeId<<endl;
//    cout<<msg.DCP.header.invokeId<<endl;
//    cout<<RecvHeader.robotVersion<<endl;


    if(RecvHeader.invokeId==msg.DCP.header.invokeId)
    {

        datas=IndyTCP->Read();
        //IndyTCP->WaitRead();
        memcpy(RecvData->Jnt,datas,RecvHeader.dataSize);
        RecvData->Jnt[0]=RecvData->Jnt[0]*M_PI/180;
        RecvData->Jnt[1]=RecvData->Jnt[1]*M_PI/180;
        RecvData->Jnt[2]=RecvData->Jnt[2]*M_PI/180;
        RecvData->Jnt[3]=RecvData->Jnt[3]*M_PI/180;
        RecvData->Jnt[4]=RecvData->Jnt[4]*M_PI/180;
        RecvData->Jnt[5]=RecvData->Jnt[5]*M_PI/180;
    }
    else
    {

        datas=IndyTCP->Read();
        //IndyTCP->WaitRead();
    }

cout<<"+++++++++++++++++++++"<<endl;
    RobotFinishMotion();


    datas=IndyTCP->Read();
    //IndyTCP->WaitRead();
    memcpy(&RecvHeader,datas,sizeof(datas));
//    cout<<RecvHeader.robotName<<endl;
//    cout<<RecvHeader.dataSize<<endl;
//    cout<<RecvHeader.invokeId<<endl;
//    cout<<msg.DCP.header.invokeId<<endl;
//    cout<<RecvHeader.robotVersion<<endl;

    if(RecvHeader.invokeId==msg.DCP.header.invokeId)
    {

        datas=IndyTCP->Read();
        //IndyTCP->WaitRead();

    }
    else
    {
        datas=IndyTCP->Read();
        //IndyTCP->WaitRead();
    }


cout<<"+++++++++++++++++++++"<<endl;
    RobotIsBusy();

    ///out|=2*(IndyTCP->waitForReadyRead(-1));

    ///datas = IndyTCP->readAll();
    datas2=IndyTCP->Read();
    //IndyTCP->WaitRead();
    memcpy(&RecvHeader,datas2,sizeof(datas2));
//    cout<<RecvHeader.robotName<<endl;
//    cout<<RecvHeader.dataSize<<endl;
//    cout<<RecvHeader.invokeId<<endl;
//    cout<<msg.DCP.header.invokeId<<endl;
//    cout<<RecvHeader.robotVersion<<endl;
    if(RecvHeader.invokeId==msg.DCP.header.invokeId)
    {
        ///out|=2*(IndyTCP->waitForReadyRead(-1));

        ///datas = IndyTCP->readAll();
        datas2=IndyTCP->Read();
        //IndyTCP->WaitRead();
        //qDebug()<<"ok4";
    }
    else
    {
        //datas = IndyTCP->readAll();
        datas2=IndyTCP->Read();
        //IndyTCP->WaitRead();
    }

    \

//    if((datas2==1)) RecvData->RobotState=2;
//    else if((datas==1)) RecvData->RobotState=1;
//    else RecvData->RobotState=0;


    ReadState=false;
}
