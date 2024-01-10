#include "panda.h"

panda::panda()
{

}

void panda::InitSocket(TCPClient *sock1, TCPClient *sock2, ReciveData *Info, int cord_type)
{
    tcpClient=sock1;
    recvsock=Info;
}

void panda::MoveL(WayPoints *data)
{

}

void panda::MoveJ(WayPoints *data)
{

    memset(cmdData.data.joint, 0, sizeof(double)*35);
    memcpy(cmdData.data.joint, recvRobotState.robot_state.q, sizeof(double)*7);
    memcpy(cmdData.data.pose, recvRobotState.robot_state.O_T_EE, sizeof(double)*16);
    memset(cmdData.data.vel, 0, sizeof(double)*5);
    memset(cmdData.data.time, 0, sizeof(double)*5);

    cmdData.data.joint[button_num] = recvRobotState.robot_state.q[button_num] + 90*M_PI/180.0*(button_sign ? 1 : -1);
    cmdData.data.joint[button_num] = data->Jnt[0];
    cmdData.data.cmd = CMD_JOINT_MOVE;
    cmdData.data.num = 1;
    cmdData.data.vel[0] = vel;
    cmdData.data.time[0] = 0;
    cmdData.data.acc_time[0] = 0.3;

    memset(bufSend, 0, SENDBUFSIZE);
    bufSend[0] = SOP_TX;
    memcpy(bufSend + 1, cmdData.char_data, sizeof(cmdData.char_data));
    bufSend[SENDBUFSIZE - 1] = EOP;

    tcpClient->Write(bufSend, SENDBUFSIZE);
}

void panda::MoveB(WayPoints *data)
{

}

void panda::MoveC(WayPoints *data)
{

}

void panda::RobotInfo()
{

}

void panda::Stop()
{

}

void panda::SetVelocity(double v)
{
    vel=0.6*(v/100.0);
}

void panda::settcp(bool on)
{

}
