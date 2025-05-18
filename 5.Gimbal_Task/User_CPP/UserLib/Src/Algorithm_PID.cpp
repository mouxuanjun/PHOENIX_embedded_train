//
// Created by 22560 on 25-4-22.
//

#include "Algorithm_PID.hpp"
#include "Algorithm_Math.hpp"
#include "Device_DM4310.hpp"

void Class_PID::Init(CPP_Enum_PID_Type pid_type, float CPP_K_P, float CPP_K_I, float CPP_K_D, float CPP_K_F,
                     float CPP_I_Out_Max, float CPP_Out_Max,
                     float CPP_D_T, float CPP_Dead_Zone,
                     float CPP_I_Variable_Speed_A,
                     float CPP_I_Variable_Speed_B, float CPP_I_Separate_Threshold,
                     CPP_Enum_DFirst CPP_D_First) {
    PID_Type = pid_type;
    PID_Params.K_P = CPP_K_P;
    PID_Params.K_I = CPP_K_I;
    PID_Params.K_D = CPP_K_D;
    PID_Params.K_F = CPP_K_F;
    PID_Params.I_Out_Max = CPP_I_Out_Max;
    PID_Params.Out_Max = CPP_Out_Max;
    PID_Params.D_T = CPP_D_T;
    PID_Params.Dead_Zone = CPP_Dead_Zone;
    PID_Params.I_Variable_Speed_A = CPP_I_Variable_Speed_A;
    PID_Params.I_Variable_Speed_B = CPP_I_Variable_Speed_B;
    PID_Params.I_Separate_Threshold = CPP_I_Separate_Threshold;
    PID_Params.D_First = CPP_D_First;
}

void Class_PID::Set_params(CPP_PID_Params_t* Params) {
    PID_Params.K_P = Params->K_P;
    PID_Params.K_I = Params->K_I;
    PID_Params.K_D = Params->K_D;
    PID_Params.K_F = Params->K_F;
    PID_Params.I_Out_Max = Params->I_Out_Max;
    PID_Params.Out_Max = Params->Out_Max;
    PID_Params.D_T = Params->D_T;
    PID_Params.Dead_Zone = Params->Dead_Zone;
    PID_Params.D_First = Params->D_First;
}


//以下函数用于PID类的输出
float Class_PID::Get_Integral_Error() {
    return (PID_Info.Integral_Error);
}

float Class_PID::Get_Out() {
    return (PID_Info.Out);
}

//以下函数用于PID输入
void Class_PID::Set_K_P(float CPP_K_P) {
    PID_Params.K_P = CPP_K_P;
}

void Class_PID::Set_K_I(float CPP_K_I) {
    PID_Params.K_I = CPP_K_I;
}

void Class_PID::Set_K_D(float CPP_K_D) {
    PID_Params.K_D = CPP_K_D;
}

void Class_PID::Set_K_F(float CPP_K_F) {
    PID_Params.K_F = CPP_K_F;
}

void Class_PID::Set_I_Out_Max(float CPP_I_Out_Max) {
    PID_Params.I_Out_Max = CPP_I_Out_Max;
}

void Class_PID::Set_Out_Max(float CPP_Out_Max) {
    PID_Params.Out_Max = CPP_Out_Max;
}

void Class_PID::Set_I_Separate_Threshold(float CPP_I_Separate_Threshold) {
    PID_Params.I_Separate_Threshold = CPP_I_Separate_Threshold;
}

void Class_PID::Set_I_Variable_Speed_A(float CPP_I_Variable_Speed_A) {
    PID_Params.I_Variable_Speed_A = CPP_I_Variable_Speed_A;
}

void Class_PID::Set_Target(float CPP_Target) {
    PID_Info.Target = CPP_Target;
}

void Class_PID::Set_Now(float CPP_Now) {
    PID_Info.Now = CPP_Now;
}

void Class_PID::Set_Integral_Error(float CPP_Integral_Error) {
    PID_Info.Integral_Error = CPP_Integral_Error;
}

void Class_PID::Set_DFirst(CPP_Enum_DFirst DFirst) {
    PID_Params.D_First = DFirst;
}


void Class_PID::CPP_TIM_Adjust_PeriodElapsedCallback() {
    // P输出
    float p_out = 0.0f;
    // I输出
    float i_out = 0.0f;
    // D输出
    float d_out = 0.0f;
    // F输出
    float f_out = 0.0f;
    //误差
    float error;
    //绝对值误差
    float abs_error;
    //线性变速积分
    float speed_ratio;

    //判断是否需要过圈保护
    switch (PID_Type) {
    case speed: break;
    case GM6020_angle: {
        //过圈保护
        if (PID_Info.Target - PID_Info.Now > 4096) {
            PID_Info.Now += 8192;
        }
        else if (PID_Info.Target - PID_Info.Now < -4096) {
            PID_Info.Target += 8192;
        }
        break;
    }
    case DM4310_angle: {
        //过圈保护
        if (PID_Info.Target - PID_Info.Now > PI) {
            PID_Info.Now += 2*PI;
        }
        else if (PID_Info.Target - PID_Info.Now < -PI) {
            PID_Info.Target += 2*PI;
        }
        break;
    }
    }

    error = PID_Info.Target - PID_Info.Now;
    abs_error = Math_Abs(error);
    PID_Info.Error = error;

    //判断死区
    if (abs_error < PID_Params.Dead_Zone) {
        PID_Info.Target = PID_Info.Now;
        error = 0.0f;
        abs_error = 0.0f;
        PID_Info.Integral_Error = 0.0f;
    }

    //计算p项

    p_out = PID_Params.K_P * error;
    PID_Info.P_Out = p_out;

    //计算i项

    if (PID_Params.I_Variable_Speed_A == 0.0f && PID_Params.I_Variable_Speed_A == 0.0f) {
        //非变速积分
        speed_ratio = 1.0f;
    }
    else {
        //变速积分
        if (abs_error <= PID_Params.I_Variable_Speed_B) {
            speed_ratio = 1.0f;
        }
        else if (PID_Params.I_Variable_Speed_B < abs_error && abs_error < PID_Params.I_Variable_Speed_A + PID_Params.
            I_Variable_Speed_B) {
            speed_ratio = (PID_Params.I_Variable_Speed_A + PID_Params.I_Variable_Speed_B - abs_error) / PID_Params.
                I_Variable_Speed_A;
        }
        if (abs_error >= PID_Params.I_Variable_Speed_B) {
            speed_ratio = 0.0f;
        }
    }
    //积分限幅
    if (PID_Params.I_Out_Max != 0.0f) {
        Math_Constrain(&PID_Info.Integral_Error, -PID_Params.I_Out_Max / PID_Params.K_I,
                       PID_Params.I_Out_Max / PID_Params.K_I);
    }
    if (PID_Params.I_Separate_Threshold == 0.0f) {
        //没有积分分离
        PID_Info.Integral_Error += speed_ratio * PID_Params.D_T * error;
        i_out = PID_Params.K_I * PID_Info.Integral_Error;
    }
    else {
        //积分分离使能
        if (abs_error < PID_Params.I_Separate_Threshold) {
            PID_Info.Integral_Error += speed_ratio * PID_Params.D_T * error;
            i_out = PID_Params.K_I * PID_Info.Integral_Error;
            PID_Info.I_Out = i_out;
        }
        else {
            PID_Info.Integral_Error = 0.0f;
            i_out = 0.0f;
            PID_Info.I_Out = i_out;
        }
    }

    //计算d项

    if (PID_Params.D_First == CPP_PID_DFirst_Disable) {
        //没有微分先行
        d_out = PID_Params.K_D * (error - PID_Info.Pre_Error) / PID_Params.D_T;
        PID_Info.D_Out = d_out;
    }
    else {
        //微分先行使能
        d_out = PID_Params.K_D * (PID_Info.Out - PID_Info.Pre_Out) / PID_Params.D_T;
        PID_Info.D_Out = d_out;
    }

    //计算前馈

    //判断是否需要前馈过圈保护
    float temp_Pre_target = PID_Info.Pre_Target;
    float temp_Target = PID_Info.Target;
    switch (PID_Type) {
    case speed: break;
    case GM6020_angle: {
        //过圈保护
        if (temp_Target - temp_Pre_target > 4096) {
            temp_Pre_target += 8192;
        }
        else if (temp_Target - temp_Pre_target < -4096) {
            temp_Target += 8192;
        }
        break;
    }
    case DM4310_angle: {
        //过圈保护
        if (temp_Target - temp_Pre_target > PI) {
            temp_Pre_target += 2*PI;
        }
        else if (temp_Target - temp_Pre_target < -PI) {
            temp_Target += 2*PI;
        }
        break;
    }
    }
    f_out = (temp_Target - temp_Pre_target) * PID_Params.K_F;
    PID_Info.F_Out = f_out;

    //计算总共的输出

    PID_Info.Out = p_out + i_out + d_out + f_out;
    //输出限幅
    if (PID_Params.Out_Max != 0.0f) {
        Math_Constrain(&PID_Info.Out, -PID_Params.Out_Max, PID_Params.Out_Max);
    }

    //善后工作
    PID_Info.Pre_Now = PID_Info.Now;
    PID_Info.Pre_Target = PID_Info.Target;
    PID_Info.Pre_Out = PID_Info.Out;
    PID_Info.Pre_Error = error;
}
