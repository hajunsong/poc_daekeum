#include "m1013_v2.h"
// mm, deg

m1013::m1013()
{

}

void m1013::InitSocket(TCPClient *sock1, TCPClient *sock2, ReciveData *Info, int cord_type)
{
    InitSocket(sock1,Info,cord_type);

}

void m1013::MoveL(WayPoints *data)
{
    char byte[1024];

    data->Pnt[0].position[0]=data->Pnt[0].position[0]*1000;
    data->Pnt[0].position[1]=data->Pnt[0].position[1]*1000;
    data->Pnt[0].position[2]=data->Pnt[0].position[2]*1000;

  \
    byte[0] ='2';
    memcpy(byte+1,data,sizeof(TMatrixInfo)*data->NUM_PNT+2*sizeof(double));
    m1013CmdTCP->Write(byte,sizeof(TMatrixInfo)*data->NUM_PNT+2*sizeof(double)+1);
    usleep(50000);
//    cout<<"movel"<<endl;
}

void m1013::MoveJ(WayPoints *data,double acc)
{
     char byte[1024];

    data->Jnt[0].value[0] = data->Jnt[0].value[0]*180/M_PI;
    data->Jnt[0].value[1] = data->Jnt[0].value[1]*180/M_PI;
    data->Jnt[0].value[2] = data->Jnt[0].value[2]*180/M_PI;
    data->Jnt[0].value[3] = data->Jnt[0].value[3]*180/M_PI;
    data->Jnt[0].value[4] = data->Jnt[0].value[4]*180/M_PI;
    data->Jnt[0].value[5] = data->Jnt[0].value[5]*180/M_PI;

    byte[0] ='3';
    memcpy(byte+1,data->Jnt,sizeof(JointInfo)*data->NUM_PNT+sizeof(double));
    m1013CmdTCP->Write(byte,sizeof(JointInfo)*data->NUM_PNT+sizeof(double)+1);
    usleep(30000);
//    cout<<byte<<endl;

}
void m1013::MoveJ(WayPoints *data)
{
     char byte[1024];

    data->Jnt[0].value[0] = data->Jnt[0].value[0]*180/M_PI;
    data->Jnt[0].value[1] = data->Jnt[0].value[1]*180/M_PI;
    data->Jnt[0].value[2] = data->Jnt[0].value[2]*180/M_PI;
    data->Jnt[0].value[3] = data->Jnt[0].value[3]*180/M_PI;
    data->Jnt[0].value[4] = data->Jnt[0].value[4]*180/M_PI;
    data->Jnt[0].value[5] = data->Jnt[0].value[5]*180/M_PI;

    byte[0] ='3';
    memcpy(byte+1,data->Jnt,48+sizeof(double));
    m1013CmdTCP->Write(byte,48+sizeof(double)+1);
    usleep(30000);
    cout<<*byte<<endl;

}
void m1013::MoveB(WayPoints *data)
{
//    char byte[1024];
    char *byte=new char[8*480000];

    for(int i=0;i<data->NUM_PNT;i++)
    {
        data->Pnt[i].position[0]=data->Pnt[i].position[0]*1000;
        data->Pnt[i].position[1]=data->Pnt[i].position[1]*1000;
        data->Pnt[i].position[2]=data->Pnt[i].position[2]*1000;
    }
    byte[0] = '2';
    memcpy(byte+1,data,sizeof(TMatrixInfo)*data->NUM_PNT+2*sizeof(double));
    m1013CmdTCP->Write(byte,sizeof(TMatrixInfo)*data->NUM_PNT+2*sizeof(double)+1);

    byte=nullptr;
    delete[] byte;

}

void m1013::MoveC(WayPoints *data)
{
    char *byte=new char[8*480000];

    for(int i=0;i<data->NUM_PNT;i++)
    {
        data->Pnt[i].position[0]=data->Pnt[i].position[0]*1000;
        data->Pnt[i].position[1]=data->Pnt[i].position[1]*1000;
        data->Pnt[i].position[2]=data->Pnt[i].position[2]*1000;
    }
    byte[0] = '6';
    memcpy(byte+1,data,sizeof(TMatrixInfo)*data->NUM_PNT+2*sizeof(double));
    m1013CmdTCP->Write(byte,sizeof(TMatrixInfo)*data->NUM_PNT+2*sizeof(double)+1);
    usleep(50000);

    byte=nullptr;
    delete[] byte;
}
void m1013::InitSocket(TCPClient *cmdsock,ReciveData *Info,int cord_type){
    m1013CmdTCP = cmdsock;
    sdata=Info;

}

void m1013::RobotInfo(){

    char sendbyte;
    sendbyte='1';

    m1013CmdTCP->writeinfo=true;
    m1013CmdTCP->Write(&sendbyte,1);
    m1013CmdTCP->writeinfo=false;


    char *by =new char[4096];
    by=m1013CmdTCP->Read();

    memcpy((sdata),by,sizeof(ReciveData));
    if(sdata->RobotState==0) sdata->RobotState=1;
    else if(sdata->RobotState==2) sdata->RobotState=2;
    else sdata->RobotState=0;

    sdata->Jnt[0]=sdata->Jnt[0]*M_PI/180;
    sdata->Jnt[1]=sdata->Jnt[1]*M_PI/180;
    sdata->Jnt[2]=sdata->Jnt[2]*M_PI/180;
    sdata->Jnt[3]=sdata->Jnt[3]*M_PI/180;
    sdata->Jnt[4]=sdata->Jnt[4]*M_PI/180;
    sdata->Jnt[5]=sdata->Jnt[5]*M_PI/180;

    sdata->TCPpos[0]=sdata->TCPpos[0]/1000;
    sdata->TCPpos[1]=sdata->TCPpos[1]/1000;
    sdata->TCPpos[2]=sdata->TCPpos[2]/1000;
    sdata->TCPpos[3]=sdata->TCPpos[3]*M_PI/180;
    sdata->TCPpos[4]=sdata->TCPpos[4]*M_PI/180;
    sdata->TCPpos[5]=sdata->TCPpos[5]*M_PI/180;
//    cout<<"by"<<by<<"by size"<<sizeof(ReciveData)<<endl;
    by=nullptr;
    delete[] by;



}

void m1013::Stop()
{
    char sendbyte;
    sendbyte='5';

    m1013CmdTCP->Write(&sendbyte,1);
    usleep(50000);
}

void m1013::SetVelocity(double v)
{
    char *byte=new char[8*480000];

    if(v<0){
        v=0;
    }
    else if(v>100){
        v=100;
    }
    byte[0] = '4';
    memcpy(byte+1,&v,sizeof(double));
    m1013CmdTCP->Write(byte,sizeof(double)*2);
    usleep(50000);



    byte=nullptr;
    delete[] byte;
}

void m1013::settcp(bool on){
    char sendbyte;
    if (on==1){
        sendbyte='8';
    }
    else sendbyte='7';


    m1013CmdTCP->Write(&sendbyte,1);
    usleep(50000);
}
