#include "indy7.h"

indy7::indy7()
{

    invoke=0;
    strcpy(msg.DCP.header.robotName,"NRMK-Indy7");

    msg.DCP.header.stepInfo=0;
    msg.DCP.header.sof=0x34;

    strcpy(Extmsg.value.header.robotName,"NRMK-Indy7");
    Extmsg.value.header.stepInfo=0;
    Extmsg.value.header.sof=0x34;

}

void indy7::InitSocket(TCPClient *sock,ReciveData *Info,int cord_type){

    IndyTCP=sock;
    RecvData=Info;

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
    IndyTCP->Read();
    IndyTCP->Read();

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

    IndyTCP->Write(msg.byte);
    IndyTCP->Read();
    IndyTCP->Read();
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

    IndyTCP->Write(msg.byte);
    IndyTCP->Read();
    IndyTCP->Read();
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
    IndyTCP->Read();

}

void indy7::ClearTaskWaypointSet(){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=0;
    msg.DCP.header.cmdId=97;

    IndyTCP->Write(msg.byte);
    IndyTCP->Read();


}


void indy7::ExecuteTaskWaypointSet(){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=16;
    msg.DCP.header.cmdId=99;

    msg.DCP.data.doubleData[0]=0;
    msg.DCP.data.doubleData[1]=2;

    IndyTCP->Write(msg.byte);
    IndyTCP->Read();
    IndyTCP->Read();


}
void indy7::SetTatskMoveBaseMode(int TaskMode){
    msg.DCP.header.invokeId=++invoke;
    msg.DCP.header.dataSize=4;
    msg.DCP.header.cmdId=113;

    IndyTCP->Write(msg.byte);
    IndyTCP->Read();
    IndyTCP->Read();
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


//void indy7::GetTaskPos(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=0;
//    msg.DCP.header.cmdId=322;

//    IndyTCP->write(msg.byte,56+msg.DCP.header.dataSize);
//    IndyTCP->waitForBytesWritten(-1);
//    IndyTCP->flush();
//}

//void indy7::GetJointPos(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=0;
//    msg.DCP.header.cmdId=320;

//    IndyTCP->write(msg.byte,56+msg.DCP.header.dataSize);
//    IndyTCP->waitForBytesWritten(-1);
//    IndyTCP->flush();
//}

//void indy7::RobotIsBusy(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=0;
//    msg.DCP.header.cmdId=35;



//    IndyTCP->write(msg.byte,56+msg.DCP.header.dataSize);
//    IndyTCP->waitForBytesWritten(-1);

//    IndyTCP->flush();

//}
//void indy7::RobotFinishMotion(){
//    msg.DCP.header.invokeId=++invoke;
//    msg.DCP.header.dataSize=0;
//    msg.DCP.header.cmdId=31;

//    IndyTCP->write(msg.byte,56+msg.DCP.header.dataSize);
//    IndyTCP->waitForBytesWritten(-1);
//    IndyTCP->flush();
//}

//void indy7::RobotInfo(){
//    QByteArray datas,datas2;
//    int out=0;
//    bool ok;

//    GetTaskPos();
//    out|=1*(IndyTCP->waitForReadyRead(-1));

//    datas = IndyTCP->readAll();

//    memcpy(&RecvHeader,datas,datas.size());

//    if(RecvHeader.invokeId==msg.DCP.header.invokeId)
//    {

//        out|=2*(IndyTCP->waitForReadyRead(-1));

//        datas = IndyTCP->readAll();
//        memcpy(RecvData->TCPpos,datas,datas.size());
//        //qDebug()<<"ok1";

//    }
//    else
//    {
//        datas = IndyTCP->readAll();
//        //qDebug()<<"fault1";
//    }

//    GetJointPos();

//    out|=4*(IndyTCP->waitForReadyRead(-1));

//    datas = IndyTCP->readAll();
//    memcpy(&RecvHeader,datas,datas.size());

//    if(RecvHeader.invokeId==msg.DCP.header.invokeId)
//    {
//        out|=8*(IndyTCP->waitForReadyRead(-1));

//        datas = IndyTCP->readAll();
//        memcpy(RecvData->Jnt,datas,datas.size());
//        //qDebug()<<"ok2";
//    }
//    else
//    {
//        datas = IndyTCP->readAll();
//        //qDebug()<<"fault2";
//    }


//    RobotFinishMotion();

//    IndyTCP->waitForReadyRead(-1);
//    datas = IndyTCP->readAll();
//    memcpy(&RecvHeader,datas,datas.size());
//    if(RecvHeader.invokeId==msg.DCP.header.invokeId)
//    {
//        IndyTCP->waitForReadyRead(-1);
//        datas = IndyTCP->readAll();
//        //qDebug()<<"ok3";

//    }
//    else
//    {
//        datas = IndyTCP->readAll();
//        //qDebug()<<"fault3";
//    }



//    RobotIsBusy();

//    IndyTCP->waitForReadyRead(-1);
//    datas2 = IndyTCP->readAll();
//    memcpy(&RecvHeader,datas2,datas2.size());
//    if(RecvHeader.invokeId==msg.DCP.header.invokeId)
//    {
//        IndyTCP->waitForReadyRead(-1);
//        datas2 = IndyTCP->readAll();
//        //qDebug()<<"ok4";
//    }
//    else
//    {
//        datas2 = IndyTCP->readAll();
//        //qDebug()<<"fault4";
//    }

//    \

//    if((datas2.toHex().toInt(&ok,16)==1)) RecvData->RobotState=2;
//    else if((datas.toHex().toInt(&ok,16)==1)) RecvData->RobotState=1;
//    else RecvData->RobotState=0;



//}
