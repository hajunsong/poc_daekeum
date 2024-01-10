#include "ur10.h"
#define MAINWINDOW_H
ur10::ur10()
{

}
void ur10::InitSocket(QTcpSocket *sock1,QTcpSocket *sock2,ReciveData *Info,int cord_type){

    cmdSocket=sock1;
    dataSocket=sock2;
    RecvData=Info;

}

void ur10::movel(float x, float y, float z, float rx, float ry, float rz,float acc,float vel, float time){
    QString text,str;
    text.sprintf("def motion():\n");
    text.append(str.sprintf("set_tcp(p[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f])\n"
                                ,TCPcord.value[0],TCPcord.value[1],TCPcord.value[2]
                                ,TCPcord.value[3],TCPcord.value[4],TCPcord.value[5]));
    text.append(str.sprintf("movel(p[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],%.3f,%.3f,%.3f,0)\n",x,y,z,rx,ry,rz,acc,vel,time));
    text.append("end\n");
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    cmdSocket->flush();
}

void ur10::movej(float j1, float j2, float j3, float j4, float j5, float j6, float acc, float vel, float time){
    QString text;
     text.sprintf("movej([%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],%.3f,%.3f,%.3f)\n",j1,j2,j3,j4,j5,j6,acc,vel,time);
    cmdSocket->write(text.toStdString().c_str(), text.toStdString().length());
    cmdSocket->flush();
}

void ur10::moveb_clear(void){
    QString str;
    blendMsg.clear();
    blendMsg.append("def motion():\n");
    blendMsg.append(str.sprintf("set_tcp(p[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f])\n"
                                ,TCPcord.value[0],TCPcord.value[1],TCPcord.value[2]
                                ,TCPcord.value[3],TCPcord.value[4],TCPcord.value[5]));
}

void ur10::moveb_addpoint(float r,float x, float y, float z, float rx, float ry, float rz,float acc,float vel){
    QString str;
    blendMsg.append(str.sprintf("movel(p[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],a=%.3f,v=%.3f,r=%.3f)\n",x,y,z,rx,ry,rz,acc,vel,r));
}

void ur10::moveb_move(){
    blendMsg.append("end\n");
    cmdSocket->write(blendMsg.toStdString().c_str(),blendMsg.toStdString().length());
    cmdSocket->flush();
}

void ur10::RobotInfo(){
     QByteArray temp;
     QByteArray by;
    char by2[1140];

    dataSocket->read(by2,1140);
    by=dataSocket->readAll();

    temp.append(by.data()[3]);
    temp.append(by.data()[2]);
    temp.append(by.data()[1]);
    temp.append(by.data()[0]);




    for(int i=4;i<1140;i+=8){
        temp.append(by.data()[i+7]);
        temp.append(by.data()[i+6]);
        temp.append(by.data()[i+5]);
        temp.append(by.data()[i+4]);
        temp.append(by.data()[i+3]);
        temp.append(by.data()[i+2]);
        temp.append(by.data()[i+1]);
        temp.append(by.data()[i]);
    }

    memcpy(&urdata,temp,sizeof(urdata));
    qDebug()<<urdata.MessageSize<<urdata.ProgramState;
    if(urdata.MessageSize==1140){
        RecvData->TCPpos[0]=urdata.Tool_vector_act[0];
        RecvData->TCPpos[1]=urdata.Tool_vector_act[1];
        RecvData->TCPpos[2]=urdata.Tool_vector_act[2];
        RecvData->TCPpos[3]=urdata.Tool_vector_act[3];
        RecvData->TCPpos[4]=urdata.Tool_vector_act[4];
        RecvData->TCPpos[5]=urdata.Tool_vector_act[5];
        RecvData->Jnt[0]=urdata.q_actual[0];
        RecvData->Jnt[1]=urdata.q_actual[1];
        RecvData->Jnt[2]=urdata.q_actual[2];
        RecvData->Jnt[3]=urdata.q_actual[3];
        RecvData->Jnt[4]=urdata.q_actual[4];
        RecvData->Jnt[5]=urdata.q_actual[5];
        if(urdata.ProgramState==1) RecvData->RobotState=1;
        else if(urdata.ProgramState==2) RecvData->RobotState=2;
        else RecvData->RobotState=0;
    }

}
