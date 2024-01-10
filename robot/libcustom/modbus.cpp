#include "modbus.h"

modbus::modbus(QWidget *parent):
    QMainWindow(parent)
{
    connect(&modbustimer,SIGNAL(timeout()),this,SLOT(test()));
    modbustimer.start(10);
   qDebug()<<"modbusmake";

}


modbus::~modbus()
{

}


void modbus::test(){
    a+=1;
    qDebug()<<"timer"<<a;
}
int modbus::test2(){

    qDebug()<<"timer2";
    return 0;;
}

void modbus::run(){


}


