#include "mbed.h"
#include "m3508.h"

Timer t;
InterruptIn userbutton(PC_13);
CAN can1(PA_11, PA_12, 1000000);
m3508 _m3508;
int id = 1;

void delay(int time)//delay in micro sec
{
    t.start();
    while(t.elapsed_time() <= std::chrono::microseconds(time));
    t.stop();
    t.reset();
}
int main()
{
    //m3508 setting, don' change the order if you don't wanna fuck up
    _m3508.m3508_init(&can1);

    //**Important
    // changing this may result it lost control, please ensure safty and clear the site before power up the device 
    
    // PID for position
    _m3508.set_p_pid_param(id, 6.8,  0.195, 0.00055);
    // PID for velocity for position loop
    _m3508.set_v_pid_param(id, 2, 0.2, 0.00049);

    // velocity set value, try to change it if the speed in not encough
    //** very sensitive 
    _m3508.set_velocity(id,12000);
    //inital start up position
    _m3508.set_position(id,0);

    while(1) {
        if(!userbutton) {
            //set position to +1400
            _m3508.set_position(id,1400);
        } else {
            //set position back to zero
            _m3508.set_position(id,0);
        }
        //read and write for the PID, check on the m3508.cpp for more detail
        _m3508.c620_read();
        _m3508.c620_write();
        
        //control the frequncy of for the PID controller 
        delay(50);
    }
}