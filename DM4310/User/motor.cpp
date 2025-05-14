//
// Created by 29812 on 25-5-6.
//
#include "motor.h"

Motor::Motor()
{
    pid_type = NONE_PID;
    pid_speed = nullptr;
    pid_angle = nullptr;
}

void Motor::Set_Pid_Type(Pid_Type pid_type)
{
    this->pid_type = pid_type;
}

Motor::~Motor()
{

}
