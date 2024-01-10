#include "ur10_v2.h"
#define MAINWINDOW_H

/* m,rad,rotation_Vector*/
ur10::ur10()
{

}

ur10::~ur10()
{

}
void ur10::InitSocket(TCPClient *sock1,TCPClient *sock2,ReciveData *Info,int cord_type){
    cmdSocket=sock1;
    dataSocket=sock2;
    RecvData=Info;

}

void ur10::movel(float x, float y, float z, float rx, float ry, float rz,float vel,float acc, float time){
    char buf[500];
    sprintf(buf,"def motion():\n"
                "set_tcp(p[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f])\n"
                "vec=rpy2rotvec([%.3f,%.3f,%.3f])\n"
                "movel(p[%.3f,%.3f,%.3f,vec[0],vec[1],vec[2]],%.3f,%.3f,%.3f,0)\n"
                "end\n"
            ,TCPcord.value[0],TCPcord.value[1],TCPcord.value[2],TCPcord.value[3],TCPcord.value[4],TCPcord.value[5]
            ,rx,ry,rz,x,y,z,acc,vel,time);

    cmdSocket->Write(buf);

}

void ur10::movej(float j1, float j2, float j3, float j4, float j5, float j6, float vel, float acc, float time){

     char buf[500];
     sprintf(buf,"movej([%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],%.3f,%.3f,%.3f)\n",j1,j2,j3,j4,j5,j6,vel,acc,time);
     cmdSocket->Write(buf);
}

void ur10::moveb_clear(void){
    char buf[500];
    sprintf(buf,"def motion():\n"
                "set_tcp(p[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f])\n"
            ,TCPcord.value[0],TCPcord.value[1],TCPcord.value[2]
            ,TCPcord.value[3],TCPcord.value[4],TCPcord.value[5]);
    string text;
    text=buf;
    memcpy(blendMsg,buf,text.size());
    blendMsgSize+=text.size();
}

void ur10::moveb_addpoint(float r,float x, float y, float z, float rx, float ry, float rz,float vel,float acc){
    char buf[500];
//    sprintf(buf,"vec=rpy2rotvec([%.3f,%.3f,%.3f])\n"
//                "movel(p[%.3f,%.3f,%.3f,vec[0],vec[1],vec[2]],%.3f,%.3f,0,%.3f)\n",rx,ry,rz,x,y,z,acc,vel,r);
    sprintf(buf,"vec=rpy2rotvec([%.3f,%.3f,%.3f])\n"
                "movep(p[%.3f,%.3f,%.3f,vec[0],vec[1],vec[2]],%.3f,%.3f,%.3f)\n",rx,ry,rz,x,y,z,acc,vel,r);

    string text;
    text=buf;
    memcpy(blendMsg+blendMsgSize,buf,text.size());
    blendMsgSize+=text.size();

}

void ur10::moveb_move(){
    char buf[500];
    sprintf(buf,"end\n");
    string text;
    text=buf;
    memcpy(blendMsg+blendMsgSize,buf,text.size());
    blendMsgSize+=text.size();
    cmdSocket->Write(blendMsg,blendMsgSize);
}

void ur10::RobotInfo(){

     char *temp =new char[1220];
     char *by=new char[1220];

    by=dataSocket->Read();

    temp[0]=by[3];
    temp[1]=by[2];
    temp[2]=by[1];
    temp[3]=by[0];




    for(int i=4;i<1140;i+=8){
        temp[i]=by[i+7];
        temp[i+1]=by[i+6];
        temp[i+2]=by[i+5];
        temp[i+3]=by[i+4];
        temp[i+4]=by[i+3];
        temp[i+5]=by[i+2];
        temp[i+6]=by[i+1];
        temp[i+7]=by[i];
    }

    memcpy(&urdata,temp,sizeof(urdata));

    double angle;
    double vec[3],mat[9],ori[3];
    angle = sqrt(urdata.Tool_vector_act[3]*urdata.Tool_vector_act[3]
                + urdata.Tool_vector_act[4]*urdata.Tool_vector_act[4]
                + urdata.Tool_vector_act[5]*urdata.Tool_vector_act[5]);

        vec[0] = urdata.Tool_vector_act[3]/angle;
        vec[1] = urdata.Tool_vector_act[4]/angle;
        vec[2] = urdata.Tool_vector_act[5]/angle;


        double c = cos(angle);
        double s = sin(angle);
        double t = 1.0 - c;
        double x = vec[0];
        double y = vec[1];
        double z = vec[2];
        double deg_x,deg_y,deg_z;

        mat[0] = t*x*x + c;     mat[1] = t*x*y - z*s;   mat[2] = t*x*z + y*s;
        mat[3] = t*x*y + z*s;   mat[4] = t*y*y + c;     mat[5] = t*y*z - x*s;
        mat[6] = t*x*z - y*s;   mat[7] = t*y*z + x*s;   mat[8] = t*z*z + c;

        Conf.rottype=rpy_rad;
        Conf.InverseRot(mat,&deg_x,&deg_y,&deg_z);

        ori[0] = deg_x;
        ori[1] = deg_y;
        ori[2] = deg_z;

//        cout<<"UR INFO"<<endl;
//        cout<<urdata.Tool_vector_act[3]<<"  "<<urdata.Tool_vector_act[4]<<"  "<<urdata.Tool_vector_act[5]<<endl;
//        cout<<ori[0]<<"  "<<ori[1]<<"  "<<ori[2]<<endl;

    if((urdata.MessageSize==1140)||(urdata.MessageSize==1220)||(urdata.MessageSize==1116)){
        RecvData->TCPpos[0]=urdata.Tool_vector_act[0];
        RecvData->TCPpos[1]=urdata.Tool_vector_act[1];
        RecvData->TCPpos[2]=urdata.Tool_vector_act[2];
        RecvData->TCPpos[3]=ori[0];
        RecvData->TCPpos[4]=ori[1];
        RecvData->TCPpos[5]=ori[2];
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

    temp=nullptr;
    delete []temp;
    by=nullptr;
    delete []by;
}

void ur10::MoveL(WayPoints *data)
{
    double x,y,z;
    double alpha,beta,gamma;

    x = data->Pnt[0].position[0];
    y = data->Pnt[0].position[1];
    z = data->Pnt[0].position[2];

    Conf.rottype=rpy_rad;
    Conf.InverseRot(data->Pnt[0].R,&alpha,&beta,&gamma);
    alpha=alpha;
    beta=beta;
    gamma=gamma;
    movel(x,y,z,alpha,beta,gamma,vel_l);
}

void ur10::MoveJ(WayPoints *data)
{
    double j1,j2,j3,j4,j5,j6;

    j1 = data->Jnt[0].value[0];
    j2 = data->Jnt[0].value[1];
    j3 = data->Jnt[0].value[2];
    j4 = data->Jnt[0].value[3];
    j5 = data->Jnt[0].value[4];
    j6 = data->Jnt[0].value[5];


    movej(j1,j2,j3,j4,j5,j6,vel_j);
}
void ur10::MoveJ(WayPoints *data,double acc)
{
    if (acc<0) acc=30;
    double j1,j2,j3,j4,j5,j6;

    j1 = data->Jnt[0].value[0];
    j2 = data->Jnt[0].value[1];
    j3 = data->Jnt[0].value[2];
    j4 = data->Jnt[0].value[3];
    j5 = data->Jnt[0].value[4];
    j6 = data->Jnt[0].value[5];


    movej(j1,j2,j3,j4,j5,j6,vel_j,acc);
}
void ur10::MoveB(WayPoints *data)
{


    double x,y,z;
    double alpha,beta,gamma;

    moveb_clear();

    for(int i=0;i<data->NUM_PNT;i++)
    {

        x = data->Pnt[i].position[0];
        y = data->Pnt[i].position[1];
        z = data->Pnt[i].position[2];

        Conf.InverseRot(data->Pnt[i].R,&alpha,&beta,&gamma);
        if(i==(data->NUM_PNT)-1) moveb_addpoint(0,x,y,z,alpha,beta,gamma,vel_l,0.2);
        else moveb_addpoint(data->blend,x,y,z,alpha,beta,gamma,vel_l,0.2);

    }

    moveb_move();

}

void ur10::MoveC(WayPoints *data)
{
    double x1,y1,z1,x2,y2,z2;
    double alpha1,beta1,gamma1,alpha2,beta2,gamma2;

    x1 = data->Pnt[0].position[0];
    y1 = data->Pnt[0].position[1];
    z1 = data->Pnt[0].position[2];

    x2 = data->Pnt[1].position[0];
    y2 = data->Pnt[1].position[1];
    z2 = data->Pnt[1].position[2];

    Conf.InverseRot(data->Pnt[0].R,&alpha1,&beta1,&gamma1);
    Conf.InverseRot(data->Pnt[1].R,&alpha2,&beta2,&gamma2);



    char buf[500];
    sprintf(buf,"def motion():\n"
                "set_tcp(p[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f])\n"
                "vec1=rpy2rotvec([%.3f,%.3f,%.3f])\n"
                "vec2=rpy2rotvec([%.3f,%.3f,%.3f])\n"
                "movec(p[%.3f,%.3f,%.3f,vec1[0],vec1[1],vec1[2]],p[%.3f,%.3f,%.3f,vec2[0],vec2[1],vec2[2]],1.2,%.3f,%.3f,%.3f)\n"
                "end\n"
            ,TCPcord.value[0],TCPcord.value[1],TCPcord.value[2],TCPcord.value[3],TCPcord.value[4],TCPcord.value[5]
            ,alpha1,beta1,gamma1
            ,alpha2,beta2,gamma2
            ,x1,y1,z1,x2,y2,z2,vel_l,0,0);

    cmdSocket->Write(buf);

}

void ur10::SetVelocity(double v)
{
    if(v<0) v=0;
    else if(v>100) v=100;
    vel_l=v*0.5/100;
    vel_j=v*3.33/100;
}



void ur10::stopj()
{
    char buf[500];
    sprintf(buf,"stopj(1)\n");

    cmdSocket->Write(buf);
}

void ur10::stopl()
{
    char buf[500];
    sprintf(buf,"stopl(1, 5)\n");

    cmdSocket->Write(buf);
}

void ur10::Stop()
{
    stopj();
    stopl();
}

void ur10::ControlBoxDigitalOut(int out)
{

    bool v1,v2,v3,v4,v5,v6,v7,v8;
    v1=out&0x01;
    v2=out&0x02;
    v3=out&0x04;
    v4=out&0x08;
    v5=out&0x10;
    v6=out&0x20;
    v7=out&0x40;
    v8=out&0x80;

    char buf[500];
    sprintf(buf,"def motion():\n"
                "set_digital_out(0,%s)\n"
            "set_digital_out(1,%s)\n"
            "set_digital_out(2,%s)\n"
            "set_digital_out(3,%s)\n"
            "set_digital_out(4,%s)\n"
            "set_digital_out(5,%s)\n"
            "set_digital_out(6,%s)\n"
            "set_digital_out(7,%s)\n"
            "end\n"
            ,v1? "True":"False",v2? "True":"False",v3? "True":"False",v4? "True":"False"
            ,v5? "True":"False",v6? "True":"False",v7? "True":"False",v8? "True":"False");

    cmdSocket->Write(buf);
}

array<bool,8> ur10::ControlBoxDigitalInput()
{
    array<bool,8> DI;
    int bits=urdata.DI_bits;
    DI[0]=bits&0x01;
    DI[1]=bits&0x02;
    DI[2]=bits&0x04;
    DI[3]=bits&0x08;
    DI[4]=bits&0x10;
    DI[5]=bits&0x20;
    DI[6]=bits&0x40;
    DI[7]=bits&0x80;


    return DI;
}

int ur10::ControlBoxDigitalIn()
{
    int temp =urdata.DI_bits;
    return temp;
}

bool ur10::WaitMove()
{
//    urdata.ProgramState==2;
    while(RecvData->RobotState!=2){

    }
    while(RecvData->RobotState!=1){

    }

            return movestate;
}
