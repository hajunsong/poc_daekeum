
#include "tcpclient.h"

TCPClient::TCPClient()
{
    comm_thread_run=false;
    server_connected=false;
    Cmd=TCPCmd::Idle;
//    pthread_create(&comm_rx_thread,nullptr,run,this);
        bufSend=new char[8*480000];

}
TCPClient::~TCPClient(){
   if(server_connected)
   {
       pthread_join(comm_rx_thread,nullptr);
   }
   delete[] bufSend;
}

int TCPClient::Connect(const string& _ip, uint16_t _port,int _timeout){

    if(server_connected){
      cout<<"socket already connected"<<endl;
    }

    else{
        comm_thread_run=true;
        pthread_create(&comm_rx_thread,nullptr,run,this);
        pthread_detach(comm_rx_thread);


        ip=  _ip;
        port= _port;

        Cmd=TCPCmd::Connection;

        std::chrono::system_clock::time_point start,end;

        if (_timeout<0) {
            timeout=std::chrono::milliseconds(1000);
        }
        else{
            timeout=std::chrono::milliseconds(_timeout);
        }
        start = std::chrono::system_clock::now();

        while(ret<0){
//            cout<<"connecting"<<endl;
            Cmd=TCPCmd::Connection;
            end = std::chrono::system_clock::now();
            dt = end-start;

            if(timeout<dt){
                Cmd=TCPCmd::Idle;
                comm_thread_run=false;

                break;
            }

        }

    }
//    cout<<"connect , CMD state : "<<Cmd <<endl;

    return ret;
}

void TCPClient::Disconnect(){

    if(server_connected==false){
        cout<<"socket not connected"<<endl;
    }
    else{
        comm_thread_run=false;
        server_connected=false;
        Cmd=TCPCmd::Disconnction;

        cout<<"socket disconnet"<<endl;
//         pthread_join(comm_rx_thread,nullptr);
       shutdown(SockFD,SHUT_RDWR);
       pthread_cancel(comm_rx_thread);
//

    }


    ret=-1;
}

void TCPClient::Write(char* _buf,int _len)
{
    sendByteLen=-2;
    if(server_connected){

        while (Cmd!=TCPCmd::Idle) {

        }
        string text;
        text=_buf;
        len= (_len < 0 ) ? text.size(): _len;
        Cmd=TCPCmd::Write;
        memcpy(bufSend,_buf,len);
    }

}


char* TCPClient::Read()
{
    if(server_connected){
         Cmd=TCPCmd::Read;
//         memset(buf,0,sizeof(buf));

//         while(recvByteLen<0){
//         recvByteLen = recv(SockFD,buf,sizeof(buf),0);
//        }

         recvByteLen=-1;
//         while(recvByteLen<0){
//             recvByteLen = recv(SockFD,buf,sizeof(buf),0);
//         }
         while (Cmd!=TCPCmd::Idle) {
            recvByteLen = recv(SockFD,buf,sizeof(buf),0);
         }



//         cout <<buf<<endl;
         return buf;
    }
    else{
        return "error";
    }
}

bool TCPClient::IsConnected()
{
    if(ret==-1)     return false;
    else return true;
}

void* TCPClient::run(void *arg)
{
    TCPClient *tThis =static_cast<TCPClient*>(arg);
//    cout<<"TCP THr Ok;"<<endl;

    while ((tThis->comm_thread_run)==true) {
        switch (tThis->Cmd) {
        case TCPCmd::Connection:
                tThis->Connect();
                tThis->Cmd=Idle;
            break;
        case TCPCmd::Disconnction:
                tThis->Cmd=Idle;
            break;
        case TCPCmd::Write:
                tThis->Write();
                tThis->Cmd=Idle;
            break;
        case TCPCmd::Read:
//                tThis->recvByteLen = recv(tThis->SockFD,tThis->buf,sizeof(tThis->buf),0);

                if(tThis->recvByteLen>0){
                    tThis->Cmd=Idle;
                }
            break;
        default:
            break;
        }
//    cout<<errno<<endl;
    }
}

void TCPClient::Connect()
{
    SockFD=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);

    if(SockFD < 0){
           cout << endl << "socket create error" << endl;
           return;
       }

    int on = 1;
    if(setsockopt(SockFD, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }
    if(setsockopt(SockFD, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&on), sizeof(on))){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }

//    if(setsockopt(SockFD,IPPROTO_TCP,TCP_QUICKACK,reinterpret_cast<const char*>(&on), sizeof(on))<0){
//        cout << endl << "set option curLen = 0; error!!" << endl;
//        return;
//    }

//    if(setsockopt(SockFD,IPPROTO_TCP,TCP_CORK,reinterpret_cast<const char*>(&on), sizeof(on))<0){
//        cout << endl << "set option curLen = 0; error!!" << endl;
//        return;
//    }

    if(setsockopt(SockFD,IPPROTO_TCP,TCP_NODELAY,reinterpret_cast<const char*>(&on), sizeof(on))<0){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }

    timeval time ={0,10000};
    if(setsockopt(SockFD,SOL_SOCKET,SO_RCVTIMEO,reinterpret_cast<const char*>(&time), sizeof(time))<0){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }

    //    int t=1000;
    //    if(setsockopt(SockFD,IPPROTO_TCP,TCP_TX_DELAY,reinterpret_cast<const char*>(&t), sizeof(t))<0){
    //        cout << endl << "set option curLen = 0; error!!" << endl;
    //        return;
    //    }





    memset(&server_addr, 0x00, sizeof(sockaddr_in));

    server_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);

    //Connection//
    curLen = 0;
    memset(bufWait, 0, MAXWAITBUFSIZE);
    ptrRecvBufIndx = bufWait;

    while ((ret = connect(SockFD, (struct sockaddr*)&server_addr, sizeof(server_addr)))<0)
    {
            server_connected = false;

            if(timeout<dt){
                comm_thread_run=false;
                cout << endl << "connect error : time out" << endl;
                pthread_join(comm_rx_thread,nullptr);
                break;
            }
    }

    if(ret>=0) {
        server_connected=true;
        cout<<"connetcion Ok"<<endl;
    }

}
void TCPClient::Write()
{
    if(server_connected==true){
//        cout<<"Write Real"<<endl;
        sendByteLen =-1;
        sendByteLen = send(SockFD, bufSend, len, MSG_WAITFORONE);

        if(!writeinfo){
            if(sendByteLen < 0){
//                printf("Send Error");
            }
            else{
//                printf("Send Success");
            }
        }
    }

}

void TCPClient::WaitWrite(){
    while (sendByteLen<0) {

    }
    cout<<"Write Complete"<<endl;
}
void TCPClient::WaitRead(){
    while (recvByteLen<0) {

    }
//    Cmd=TCPCmd::Idle;
    cout<<"read byte"<<recvByteLen<<endl;
    cout<<"Read Complete"<<endl;
}




/*

#include "tcpclient.h"

TCPClient::TCPClient()
{
    comm_thread_run=false;
    server_connected=false;
    Cmd=TCPCmd::Idle;
//    pthread_create(&comm_rx_thread,nullptr,run,this);
        bufSend=new char[8*480000];

}
TCPClient::~TCPClient(){
   if(server_connected)
   {
       pthread_join(comm_rx_thread,nullptr);
   }
   delete[] bufSend;
}

int TCPClient::Connect(const string& _ip, uint16_t _port,int _timeout){


    if(server_connected){
      cout<<"socket already connected"<<endl;
    }

    else{
        comm_thread_run=true;
        pthread_create(&comm_rx_thread,nullptr,run,this);
        pthread_detach(comm_rx_thread);


        ip=  _ip;
        port= _port;

        Cmd=TCPCmd::Connection;

        std::chrono::system_clock::time_point start,end;

        if (_timeout<0) {
            timeout=std::chrono::milliseconds(1000);
        }
        else{
            timeout=std::chrono::milliseconds(_timeout);
        }
        start = std::chrono::system_clock::now();

        while(ret<0){
//            cout<<"connecting"<<endl;
            Cmd=TCPCmd::Connection;
            end = std::chrono::system_clock::now();
            dt = end-start;

            if(timeout<dt){
                Cmd=TCPCmd::Idle;
                comm_thread_run=false;

                break;
            }

        }

    }
//    cout<<"connect , CMD state : "<<Cmd <<endl;

    return ret;
}

void TCPClient::Disconnect(){

    if(server_connected==false){
        cout<<"socket not connected"<<endl;
    }
    else{
        comm_thread_run=false;
        server_connected=false;
        Cmd=TCPCmd::Disconnction;

        cout<<"socket disconnet"<<endl;
//         pthread_join(comm_rx_thread,nullptr);
       shutdown(SockFD,SHUT_RDWR);
       pthread_cancel(comm_rx_thread);
//

    }


    ret=-1;
}

void TCPClient::Write(char* _buf,int _len)
{

    if(server_connected){

        while (Cmd!=TCPCmd::Idle) {

        }
        string text;
        text=_buf;
        len= (_len < 0 ) ? text.size(): _len;
        Cmd=TCPCmd::Write;
        memcpy(bufSend,_buf,len);
    }

}


char* TCPClient::Read()
{
    if(server_connected){
         Cmd=TCPCmd::Read;
//         memset(buf,0,sizeof(buf));
         recvByteLen = recv(SockFD,buf,sizeof(buf),0);
//         recvByteLen = recv(SockFD,buf,sizeof(buf),MSG_DONTWAIT);

         while (Cmd!=TCPCmd::Idle) {

         }
//         cout <<"recive Len :"<< recvByteLen<<endl;
//         cout <<buf<<endl;
         return buf;
    }
    else{
        return "-1";
    }
}

bool TCPClient::IsConnected()
{
    if(ret==-1)     return false;
    else return true;
}

void* TCPClient::run(void *arg)
{
    TCPClient *tThis =static_cast<TCPClient*>(arg);
//    cout<<"TCP THr Ok;"<<endl;

    while ((tThis->comm_thread_run)==true) {
        switch (tThis->Cmd) {
        case TCPCmd::Connection:
                tThis->Connect();
                tThis->Cmd=Idle;
            break;
        case TCPCmd::Disconnction:
                tThis->Cmd=Idle;
            break;
        case TCPCmd::Write:
                tThis->Write();
                tThis->Cmd=Idle;
            break;
        case TCPCmd::Read:
//                tThis->recvByteLen = recv(tThis->SockFD,tThis->buf,sizeof(tThis->buf),0);

                tThis->Cmd=Idle;
            break;
        default:
            break;
        }
//    cout<<errno<<endl;
    }
}

void TCPClient::Connect()
{
    SockFD=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);

    if(SockFD < 0){
           cout << endl << "socket create error" << endl;
           return;
       }

    int on = 1;
    if(setsockopt(SockFD, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&on), sizeof(on)) < 0){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }
    if(setsockopt(SockFD, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&on), sizeof(on))){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }

//    if(setsockopt(SockFD,IPPROTO_TCP,TCP_QUICKACK,reinterpret_cast<const char*>(&on), sizeof(on))<0){
//        cout << endl << "set option curLen = 0; error!!" << endl;
//        return;
//    }

//    if(setsockopt(SockFD,IPPROTO_TCP,TCP_CORK,reinterpret_cast<const char*>(&on), sizeof(on))<0){
//        cout << endl << "set option curLen = 0; error!!" << endl;
//        return;
//    }

    if(setsockopt(SockFD,IPPROTO_TCP,TCP_NODELAY,reinterpret_cast<const char*>(&on), sizeof(on))<0){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }

    timeval time ={0,10000};
    if(setsockopt(SockFD,SOL_SOCKET,SO_RCVTIMEO,reinterpret_cast<const char*>(&time), sizeof(time))<0){
        cout << endl << "set option curLen = 0; error!!" << endl;
        return;
    }

    //    int t=1000;
    //    if(setsockopt(SockFD,IPPROTO_TCP,TCP_TX_DELAY,reinterpret_cast<const char*>(&t), sizeof(t))<0){
    //        cout << endl << "set option curLen = 0; error!!" << endl;
    //        return;
    //    }





    memset(&server_addr, 0x00, sizeof(sockaddr_in));

    server_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);

    //Connection//
    curLen = 0;
    memset(bufWait, 0, MAXWAITBUFSIZE);
    ptrRecvBufIndx = bufWait;

    while ((ret = connect(SockFD, (struct sockaddr*)&server_addr, sizeof(server_addr)))<0)
    {
            server_connected = false;

            if(timeout<dt){
                comm_thread_run=false;
                cout << endl << "connect error : time out" << endl;
                pthread_join(comm_rx_thread,nullptr);
                break;
            }
    }

    if(ret>=0) {
        server_connected=true;
        cout<<"connetcion Ok"<<endl;
    }

}
void TCPClient::Write()
{
    if(server_connected==true){
//        cout<<"Write Real"<<endl;
        sendByteLen = send(SockFD, bufSend, len, MSG_WAITFORONE);

//        if(!writeinfo){
//            if(sendByteLen < 0){
//                printf("Send Error");
//            }
//            else{
//                printf("Send Success");
//            }
//        }
    }

}

*/
