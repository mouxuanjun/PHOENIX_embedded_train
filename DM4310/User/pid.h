#ifndef PID_H
#define PID_H
#ifdef __cplusplus
extern "C"
{
#endif


#ifdef __cplusplus
}
#include "math.h"

class PID
{
public:

    PID(float kp ,float ki , float kd ,float dt ,float target);
    void Pid_Update(float current_value , float target_value);
    void Set_Max_Output(float max_output);
    void Set_Max_Integral_Output(float Max_Integral_Limit);
    void Set_Parameters(float kp ,float ki , float kd ,float dt );
    //积分分离段
    void Enable_Integral_Separation(float Integral_Separation_Threshold);
    void Disable_Integral_Separation();
    //死区段
    void Enable_DeadZone(float DeadZone_Threshold);
    void Disable_DeadZone();
    //前馈段
    void Enable_Feedforward();
    void Disable_Feedforward();
    //变速积分段
    void Enable_Variable_Speed_Integral(float Variable_Speed_Integral_Lower_Limit , float Variable_Speed_Integral_Upper_Limit , float alpha_e);
    void Disable_Variable_Speed_Integral();
    //过零保护段
    void Enable_ZeroCrossingProtection(float ZeroCrossingProtection_Threshold = 3.1415926f);
    void Disable_ZeroCrossingProtection();
    //微分先行段
    void Enable_Differential_First();
    void Disable_Differential_First();


    float Get_Output() const;
    ~PID() = default;

private:
    float target;
    float kp;
    float ki;
    float Kd; //微分系数
    float dt; //采样时间

    float error; //当前误差
    float error_last; //上次误差
    float error_sum; //误差和
    float p_output;
    float i_output;
    float d_output;
    float output; //输出值
    float output_last;//上一次的输出值

    float max_output;  //最大输出限幅
    float max_Integral_Limit; // 最大积分限幅

    //积分分离段
    bool enable_integral_separation; //积分分离
    float integral_separation_threshold; //积分分离阈值

    //死区段
    bool enable_dead_zone; //死区
    float dead_zone_threshold; //死区阈值

    //前馈段
    bool enable_feedforward;
    float feedforward_compensation_value;


    //变速积分段
    bool enable_variable_speed_integral;
    float variable_speed_integral_lower_limit;  //变速积分下限
    float variable_speed_integral_upper_limit;  //变速积分上限
    float alpha_e; //变速积分影响因子

    //过零保护段
    bool  enable_zero_crossing_protection;
    float  zero_crossing_protection_threshold;

    //微分先行段
    bool enable_differential_first;

};


#endif
#endif //PID_H
