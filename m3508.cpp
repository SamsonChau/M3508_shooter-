#include <stdint.h>
#include <stdint.h>
#include "mbed.h"
#include "m3508.h"
/////User Define/////Herman T(HT)
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#ifdef UART_Debug
//Serial pc(USBTX, USBRX,115200);
#endif

typedef enum {
    m3508_vel_loop = 0,
    m3508_pos_loop
} m3508_mode;

CANMessage receive_msg;

void m3508::m3508_init(CAN* _CAN)
{
    can1 = _CAN;
    can1->frequency(1000000);
}

int m3508::utils_truncate_number(float *number, float min, float max)
{
    int did_trunc = 0;
    if (*number > max) {
        *number = max;
        did_trunc = 1;
    } else if (*number < min) {
        *number = min;
        did_trunc = 1;
    }
    return did_trunc;
}
int m3508::utils_truncate_number_abs(float *number, float max)
{
    int did_trunc = 0;
    if (*number > max) {
        *number = max;
        did_trunc = 1;
    } else if (*number < -max) {
        *number = -max;
        did_trunc = 1;
    }
    return did_trunc;
}

void m3508::CAN_Send(int16_t current1, int16_t current2, int16_t current3, int16_t current4)    //CAN 发送 一标准帧数据
{


    s_current = current1;
    CANMessage TxMessage;
    TxMessage.id=0x200;
    TxMessage.format=CANStandard;
    TxMessage.type=CANData;
    TxMessage.len=8;
    TxMessage.data[0]=current1>>8;           //data1;
    TxMessage.data[1]=current1;           //data2;
    TxMessage.data[2]=current2>>8;            //data3;
    TxMessage.data[3]=current2;           //data4;
    TxMessage.data[4]=current3>>8;            //data5;
    TxMessage.data[5]=current3;          //data6;
    TxMessage.data[6]=current4>>8;            //data7;
    TxMessage.data[7]=current4;          //data8;
    can1->write(TxMessage);
}

void m3508::c620_read()
{
    can1->read(receive_msg);
    uint16_t actual_local_position = (uint16_t)(receive_msg.data[0]<<8)|receive_msg.data[1];
    int16_t  actual_velocity       =  (int16_t)(receive_msg.data[2]<<8)|receive_msg.data[3];
    int16_t  actual_current        =  (int16_t)(receive_msg.data[4]<<8)|receive_msg.data[5];
    //change it your id has been change
    //driver id 1 = 0
    //driver id 2 = 1
    //driver id 3 = 2
    //driver id 4 = 3 etc
    int motor_index = 1;
    switch(receive_msg.id) {
        case Motor_1_RevID: {
            motor_index = 0;
            break;
        }
        case Motor_2_RevID: {
            motor_index = 1;
            break;
        }
        case Motor_3_RevID: {
            motor_index = 2;
            break;
        }
        case Motor_4_RevID: {
            motor_index = 3;
            break;
        }
        default:
            return;
    }
    read_velocity[motor_index] = actual_velocity;
    read_position[motor_index] = actual_local_position;
    read_current[motor_index]  = actual_current;

    //To record the starting position
    if(pos_init[motor_index]) {
        start_pos[motor_index] = read_position[motor_index];
        pos_init[motor_index] = false;
    }

    //Accumulate the round count
    if (read_position[motor_index] - last_pos[motor_index] > 4096) {
        round_cnt[motor_index]--;
    } else if (read_position[motor_index] - last_pos[motor_index] < -4096) {
        round_cnt[motor_index]++;
    } else {}

    //Multi-turn global position
    global_pos[motor_index] = round_cnt[motor_index] * 8192 + read_position[motor_index] - start_pos[motor_index];
    //Map to degree, 0-360
    global_angle[motor_index] = global_pos[motor_index] * 360.0 / 8192.0;

    //After using the value read_position, assign its value to last_pos
    last_pos[motor_index] = read_position[motor_index];//used for round count
}
void m3508::c620_write()//Execution frequency 100Hz
{
    t_pid.stop();
    dt = (float)std::chrono::duration_cast<std::chrono::microseconds>(t_pid.elapsed_time()).count()/1000000;
    int motor_index = 1; // same as read
    if(loop_mode[motor_index] == m3508_vel_loop) {
        //velocity loop
        v_pid[motor_index].P = required_velocity[motor_index]- read_velocity[motor_index];
        v_pid[motor_index].I += v_pid[motor_index].P * dt;
        v_pid[motor_index].D = (v_pid[motor_index].P - v_pid[motor_index].prev_err) / dt;
        v_pid[motor_index].prev_err = v_pid[motor_index].P;
        //Calculate output current
        motor_out[motor_index] = (int16_t)floor((v_pid[motor_index].P*v_pid[motor_index].kP +v_pid[motor_index].I*v_pid[motor_index].kI + v_pid[motor_index].D*v_pid[motor_index].kD));
    } else if(loop_mode[motor_index] == m3508_pos_loop) {
        //position loop
        p_pid[motor_index].P = required_position[motor_index] - global_angle[motor_index];
        p_pid[motor_index].P=constrain(p_pid[motor_index].P, -4096, 4096);
        p_pid[motor_index].I += p_pid[motor_index].P * dt;
        p_pid[motor_index].D = (p_pid[motor_index].P - p_pid[motor_index].prev_err) / dt;

        // Store previous error
        p_pid[motor_index].prev_err = p_pid[motor_index].P;

        // Sum up the terms
        required_velocity[motor_index] = (int16_t) floor(p_pid[motor_index].kP*p_pid[motor_index].P + p_pid[motor_index].kI*p_pid[motor_index].I + p_pid[motor_index].kD*p_pid[motor_index].D);
        //printf("%d\r\n",required_velocity[motor_index]);

        //velocity loop cal veloncity to the desire position
        v_pid[motor_index].P = required_velocity[motor_index] - read_velocity[motor_index];
        // constrain(v_pid[motor_index].P, -100, 100);
        v_pid[motor_index].I += v_pid[motor_index].P * dt;
        v_pid[motor_index].D = (v_pid[motor_index].P - v_pid[motor_index].prev_err) / dt;
        v_pid[motor_index].prev_err = v_pid[motor_index].P;

        //Calculate output current
        motor_out[motor_index] = (int16_t)floor((v_pid[motor_index].P*v_pid[motor_index].kP + v_pid[motor_index].I*v_pid[motor_index].kI + v_pid[motor_index].D*v_pid[motor_index].kD));
        //printf("%d\r\n",motor_out[motor_index]);
    }
    //Set current constaint according to m3508 & c620 official manual
    //(m3508 rated 10A, c620 rated 0 - 20A)
    // max current for m3508 = 16384 / 2 = 8192
    motor_out[motor_index]=constrain(motor_out[i],-7000,7000);
    t_pid.reset();
    CAN_Send(motor_out[0],motor_out[1],motor_out[2],motor_out[3]);
    t_pid.start();
}
void m3508::set_velocity(int motor_index, int speed)
{
    loop_mode[motor_index] = m3508_vel_loop;
    required_velocity[motor_index] = speed;
}
//pos loop input degree value (float)
void m3508::set_position(int motor_index, float pos)
{
    loop_mode[motor_index] = m3508_pos_loop;
    required_position[motor_index] = pos;
}
void m3508::set_v_pid_param(int ID, float kp, float ki, float kd)
{
    v_pid[ID].kP = kp;
    v_pid[ID].kI = ki;
    v_pid[ID].kD = kd;
}
void m3508::set_p_pid_param(int ID, float kp, float ki, float kd)
{
    p_pid[ID].kP = kp;
    p_pid[ID].kI = ki;
    p_pid[ID].kD = kd;
}