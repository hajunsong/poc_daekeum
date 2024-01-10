#include "m1013.h"

m1013::m1013()
{

}
void m1013::InitSocket(QTcpSocket *cmdsock,ReciveData *Info,int cord_type){
    m1013CmdTCP = cmdsock;
    sdata=Info;

}

void m1013::RobotInfo(){
    QByteArray sendbyte;
    sendbyte="1";

    m1013CmdTCP->write(sendbyte);
    m1013CmdTCP->waitForReadyRead(-1);
    QByteArray by=m1013CmdTCP->readAll();



    memcpy((sdata),by,by.size());
    if(sdata->RobotState==0) sdata->RobotState=1;
    else if(sdata->RobotState==2) sdata->RobotState=2;
    else sdata->RobotState=0;

//    qDebug()<<"by"<<by;



}
