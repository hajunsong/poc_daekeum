#include "camsdk.h"

CamSDK::CamSDK()
{
    CAMSDKSocket=new TCPClient;
}

CamSDK::~CamSDK()
{
    delete CAMSDKSocket;
}

void CamSDK::Connect(const string& _ip, uint16_t _port,int _timeout)
{
    CAMSDKSocket->Connect(_ip,_port,_timeout);
}

void CamSDK::Disconect()
{
    CAMSDKSocket->Disconnect();
}

void CamSDK::SendMsg(char Msg[]){

    CAMSDKSocket->Write(Msg);
}

void CamSDK::SendMsg(char *Msg,int len){

    CAMSDKSocket->Write(Msg,len);
}

char *CamSDK::ReadMsg(){
    char *buf;
    buf=CAMSDKSocket->Read();

    if (CAMSDKSocket->recvByteLen==-1) buf= "0";
    return buf;

}
