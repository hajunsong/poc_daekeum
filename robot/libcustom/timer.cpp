#include "timer.h"

timer::timer(){
    timer_start=false;
}

timer::~timer(){

}

void *timer::run(void *arg){

        timer *pThis = static_cast<timer*>(arg);

        while (pThis->timer_start) {


            std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

            pThis->start();
//            cout<<"loop-2"<<endl;
            std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
            std::this_thread::sleep_for(std::chrono::microseconds(pThis->interval*1000)-(end-start));
//            cout<<"loop-3"<<endl;
        }

}

void timer::start(int _interval,robot *C){
//    rb10 *THr;

    interval=_interval;
    THr=C;

    timer_start=true;
    pthread_create(&thread,nullptr,run,this);
    pthread_detach(thread);
//    cout<<"Timer Interval"<<interval<<endl;
}
void timer::start(int _interval){
//    rb10 *THr;

    interval=_interval;

    timer_start=true;
    pthread_create(&thread,nullptr,run,this);
    pthread_detach(thread);
//    cout<<"Timer Interval"<<interval<<endl;
}
void timer::start(){
//    cout<<"timer Thr run"<<endl;
    TimerCon.run();

}
void timer::stop(){

    if(timer_start){
        timer_start=false;
        pthread_join(thread,nullptr);
//        pthread_cancel(thread);
    }
//    pthread_cancel(thread);


}
//template<typename F,typename... T,class C>
//void sync(bool i, void (*func)(T...)){

////    F (*func2)(T... arg)=&func;


//}

//template<typename F,typename... T,class C>
//void sync(bool i, void (MainWindow::)(*func)(int,int)){

//}



