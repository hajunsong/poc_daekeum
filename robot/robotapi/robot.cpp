#include "robot.h"


robot::robot()
{

}

robot::~robot()
{

}

void robot::InitSocket(TCPClient *sock1,TCPClient *sock2,ReciveData *Info,int cord_type){

}

void robot::MoveJ(WayPoints *data, double acc)
{

}

void robot::MoveJ(WayPoints *data)
{

}
void robot::MoveL(WayPoints *data)
{

}
void robot::MoveB(WayPoints *data)
{

}

void robot::SetVelocity(double v)
{

}

void robot::CobotInit()
{

}

void robot::RobotInfo()
{

}

void robot::ControlBoxDigitalOut(int out)
{

}
array<bool,8> robot::ControlBoxDigitalInput(void)
{

}
//vector<bool> robot::ControlBoxDigitalIn(void){

//}
int robot::ControlBoxDigitalIn(void){

}

void robot::testf(int i)
{
    test_value=i;
    cout<< " test value :  " << test_value<<endl;
}

void robot::check()
{
	cout<< " test value :  " << test_value <<endl;
}

void robot::Stop()
{

}
void robot::MoveC(WayPoints *data)
{

}
bool robot::WaitMove(){

}

//void robot::set1(T *source)
//{
//    test =source;
//    cout<<typeid(T).name()<<endl;


//}

//void robot::set2(TCPClient *src1,TCPClient *src2){
//    test->InitSocket2(src1,src2);
//    test->CobotInit();
//}

void robot::settcp(bool on){

}

bool robot::RobotConnect(std::string ip, int port, ReciveData *Info){

}

void robot::RobotDisconnect(){

}


void robot::RobotComplianceCtrlOn(float stpx, float stpy, float stpz, float strx, float stry, float strz)
{

}

void robot::RobotComplianceCtrlOff()
{

}
