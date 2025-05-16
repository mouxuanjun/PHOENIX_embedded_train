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
    virtual void Pid_Update(float target) = 0;

    virtual void Set_Pid_Type(Pid_Type pid_type);

    virtual ~Motor() = 0;




private:
    Pid_Type pid_type;




};





#endif //MOTOR_H
