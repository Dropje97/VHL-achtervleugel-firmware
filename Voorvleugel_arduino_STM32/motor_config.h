#include "DualVNH5019MotorShield.h"


DualVNH5019MotorShield motor_shield;
const int offset_fixed_global = 6;

struct Motor {
    //TODO organize this struct
    void (*set_speed)(int speed);
    int (*get_current)();
        
    ulong millis_last = 0;
    volatile int encoder_pulses = 0;
    int encoder_pulses_prev = 0;
    
    float p_term = 2;
    float i_term = 0.00001;
    float d_term = 0.000000;

    int setpoint = 0;
    int offset = 0;
    int offset_fixed;

    int pwm_min = 40;
    float out_min = -360, out_max = 360;

    float i_term_result = 0;
    float pref_error = 0;
    float pid_out = 0;
    float pwm = 0;
    
    int home_pwm_high = 400;
    int home_pwm_low = 70;
    
    int current_offset;
};


enum Motor_Num {M1, M2};


void set_m1_speed(int speed){motor_shield.setM1Speed(speed);}
void set_m2_speed(int speed){motor_shield.setM2Speed(speed);}
int get_m1_current(){return motor_shield.getM1CurrentMilliamps();}
int get_m2_current(){return motor_shield.getM2CurrentMilliamps();}


Motor get_motor(Motor_Num m){
    Motor result;
    if (m == M1){
        result.offset_fixed = -1 * offset_fixed_global;
        result.current_offset = 30;
        result.set_speed = &set_m1_speed;
        result.get_current = &get_m1_current;
    }else if (m == M2){
        result.offset_fixed = offset_fixed_global;
        result.current_offset = 200; // was 408
        result.set_speed = &set_m2_speed;
        result.get_current = &get_m2_current;
    }
    return result;
}
