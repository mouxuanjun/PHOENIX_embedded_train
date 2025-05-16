#include "pid.h"

PID::PID(const float kp, const float ki, const float kd, const float dt, const float target)
{
    this->kp = kp;
    this->ki = ki;
    this->Kd = kd; //微分系数
    this->dt = dt; //采样时间
    this->target = target;
    this->error = 0.0f;
    this->error_last = 0.0f;
    this->error_sum = 0.0f;
    this->p_output = 0.0f;
    this->i_output = 0.0f;
    this->d_output = 0.0f;
    this->output_last = 0.0f;
    this->output = 0.0f;
    this->max_output = 25000.0f; // 默认无限幅
    this->max_Integral_Limit = 25000.0f;  // 默认无限幅

    this->enable_integral_separation = false; //积分分离
    this->integral_separation_threshold = 0.0f;

    this->enable_dead_zone = false; //死区
    this->dead_zone_threshold = 0.0f; //死区阈值

    this->enable_feedforward = false; //前馈段
    this->feedforward_compensation_value = 0.0f; //前馈补偿值

    this->enable_variable_speed_integral = false; //变速积分段
    this->variable_speed_integral_lower_limit = 0.0f;  //变速积分下限
    this->variable_speed_integral_upper_limit = 0.0f;  //变速积分上限
    this->alpha_e = 0.0f; //变速积分影响因子

    this->enable_zero_crossing_protection = false; //过零保护段
    this->zero_crossing_protection_threshold = 0.0f;//过零保护阈值

    this->enable_differential_first = false;
}



// 前馈段  微分先行段
void PID::Pid_Update(float current_value, float target_value)
{
    this->error_last = this->error;
    this->target = target_value;

    //过零保护段
    if (this->enable_zero_crossing_protection) {
        if (target_value - current_value >= this->zero_crossing_protection_threshold) {
            this->error = target_value - current_value - 2 * this->zero_crossing_protection_threshold;
        }
        else if (target_value - current_value <= -this->zero_crossing_protection_threshold) {
            this->error = target_value - current_value + 2 * this->zero_crossing_protection_threshold;
        }
        else {
            this->error = target_value - current_value;
        }
    }
    else {
        this->error = target_value - current_value;
    }

    //死区段
    if (this->enable_dead_zone)
    {
        if (error<=this->dead_zone_threshold && error>=-this->dead_zone_threshold)
        {
            this->error = 0;
        }
    }

    this->error_sum += this->error;
    //p_out计算
    this->p_output = this->kp * this->error;
    //i_out计算
    //变速积分段
    if (this->enable_variable_speed_integral)
    {
        if (this->error > this->variable_speed_integral_upper_limit || this->error < this->variable_speed_integral_lower_limit){
            this->i_output =(1/(1+this->alpha_e*abs(this->error)))* this->error_sum *this->ki * this->dt ;
        }
        else
        {
            this->i_output = this->error_sum *this->ki * this->dt;
        }
    }
    else
    {
        this->i_output = this->error_sum *this->ki * this->dt ;
    }

    //积分分离段
    if (this->enable_integral_separation)
    {
        if (this->error > this->integral_separation_threshold || this->error < -this->integral_separation_threshold) {
            this->i_output = 0;
            this->error_sum -=this->error;
        }
    }
    //积分限幅段
    if (this->i_output > this->max_Integral_Limit)
    {
        this->i_output = this->max_Integral_Limit;
    }
    else if (this->i_output < -this->max_Integral_Limit)
    {
        this->i_output = -this->max_Integral_Limit;
    }



    //微分先行段
    if (this->enable_differential_first)
    {
        this->d_output = this->Kd * (this->output - this->output_last) / this->dt;
    }
    else
    {
        this->d_output = this->Kd * (this->error - this->error_last) / this->dt;
    }


    this->output_last = this->output;
    this->output = this->p_output + this->i_output + this->d_output;
    //输出限幅段
    if (this->output>this->max_output)
    {
        this->output = this->max_output;
    }
    else if (this->output<-this->max_output)
    {
        this->output = -this->max_output;
    }



}

void PID::Set_Parameters(const float kp, const float ki, const float kd, const float dt)
{
    this->kp = kp;
    this->ki = ki;
    this->Kd = kd; //微分系数
    this->dt = dt; //采样时间
}

void PID::Set_Max_Output(const float max_output)
{
    this->max_output = max_output;
}

void PID::Set_Max_Integral_Output(const float Max_Integral_Limit)
{
    this->max_Integral_Limit = Max_Integral_Limit;
}

void PID::Enable_Integral_Separation(const float Integral_Separation_Threshold)
{
    this->enable_integral_separation = true;
    this->integral_separation_threshold = Integral_Separation_Threshold;
}

void PID::Disable_Integral_Separation()
{
    this->enable_integral_separation = false;
}

void PID::Enable_DeadZone(const float DeadZone_Threshold)
{
    this->enable_dead_zone = true;
    this->dead_zone_threshold = DeadZone_Threshold;
}

void PID::Disable_DeadZone()
{
    this->enable_dead_zone = false;

}

void PID::Enable_Feedforward()
{
    this->enable_feedforward = true;
    this->feedforward_compensation_value = 0;
}

void PID::Disable_Feedforward()
{
    this->enable_feedforward = false;
}


void PID::Enable_Variable_Speed_Integral(const float Variable_Speed_Integral_Lower_Limit , const float Variable_Speed_Integral_Upper_Limit , const float alpha_e)
{
    this->enable_variable_speed_integral = true;
    this->variable_speed_integral_lower_limit = Variable_Speed_Integral_Lower_Limit;
    this->variable_speed_integral_upper_limit = Variable_Speed_Integral_Upper_Limit;
    this->alpha_e = alpha_e;
}

void PID::Disable_Variable_Speed_Integral()
{
    this->enable_variable_speed_integral = false;
}

void PID::Enable_ZeroCrossingProtection(const float ZeroCrossingProtection_Threshold)
{
    this->enable_zero_crossing_protection = true;
    this->zero_crossing_protection_threshold = ZeroCrossingProtection_Threshold;
}

void PID::Disable_ZeroCrossingProtection()
{
    this->enable_zero_crossing_protection = false;
}

void PID::Enable_Differential_First()
{
    this->enable_differential_first = true;
}

void PID::Disable_Differential_First()
{
    this->enable_differential_first = false;
}

float PID::Get_Output() const
{
    return this->output;
}



