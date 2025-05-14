#ifndef MOTOR_H
#define MOTOR_H
#include "pid.h"
#include "cstdint"
#include "can.h"
#ifdef __cplusplus
extern "C"
{
#endif
#ifdef __cplusplus
}
#endif


typedef enum
{
    NONE_PID,
    SPEED_LOOP,
    POSITION_LOOP,
}Pid_Type;



class Motor
{
public:
    Motor();
    virtual void Set_Angle(float angle , float speed = 0) = 0;
    virtual void Set_Speed(float speed) = 0;
    virtual void Pid_Update() = 0;

    void Set_Pid_Type(Pid_Type pid_type);

    virtual ~Motor() = 0;




private:
    Pid_Type pid_type;

    PID* pid_speed = nullptr;
    PID* pid_angle = nullptr;


};





#endif //MOTOR_H
