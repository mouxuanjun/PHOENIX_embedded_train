//
// Created by 22560 on 25-4-22.
//

#ifndef ALGORITHM_PID_HPP
#define ALGORITHM_PID_HPP

enum CPP_Enum_DFirst {
    CPP_PID_DFirst_Disable = 0,
    CPP_PID_DFirst_Enable = 1,
};

enum CPP_Enum_PID_Type {
    speed = 1,
    GM6020_angle =2,
    DM4310_angle = 3,
};

typedef struct {
    float K_P = 0.0f;
    float K_I = 0.0f;
    float K_D = 0.0f;
    float K_F = 0.0f;

    //积分限幅, 0为不限制
    float I_Out_Max = 0;
    //输出限幅, 0为不限制
    float Out_Max = 0;

    //变速积分定速内段阈值, 0为不限制
    float I_Variable_Speed_A = 0.0f;
    //变速积分变速区间, 0为不限制
    float I_Variable_Speed_B = 0.0f;
    //积分分离阈值，需为正数, 0为不限制
    float I_Separate_Threshold = 0.0f;

    // PID计时器周期, s
    float D_T = 0.001f;
    //死区, Error在其绝对值内不输出
    float Dead_Zone = 0.0f;
    //Bang-bang控制区间
    float BangBang_Threshold = 0.0f;
    //微分先行
    CPP_Enum_DFirst D_First = CPP_PID_DFirst_Disable;
} CPP_PID_Params_t;

typedef struct {
    //之前的当前值
    float Pre_Now = 0.0f;
    //之前的目标值
    float Pre_Target = 0.0f;
    //之前的输出值
    float Pre_Out = 0.0f;
    //前向误差
    float Pre_Error = 0.0f;

    float Error = 0.0f;
    float P_Out = 0.0f;
    float I_Out = 0.0f;
    float D_Out = 0.0f;
    float F_Out = 0.0f;

    //目标值
    float Target = 0.0f;
    //当前值
    float Now = 0.0f;
    //积分值
    float Integral_Error = 0.0f;

    float Out = 0.0f;
} CPP_PID_Info_t;

class Class_PID {
public:
    void Init(CPP_Enum_PID_Type pid_type = speed,
              float CPP_K_P = 0.0f, float CPP_K_I = 0.0f, float CPP_K_D = 0.0f, float CPP_K_F = 0.0f,
              float CPP_I_Out_Max = 0.0f, float CPP_Out_Max = 0.0f,
              float CPP_D_T = 0.001f, float CPP_Dead_Zone = 0.0f,
              float CPP_I_Variable_Speed_A = 0.0f,
              float CPP_I_Variable_Speed_B = 0.0f, float CPP_I_Separate_Threshold = 0.0f,
              CPP_Enum_DFirst CPP_D_First = CPP_PID_DFirst_Disable);

    float Get_Integral_Error();
    float Get_Out();
    //pid基础值设置
    void Set_params(CPP_PID_Params_t* Params);
    void Set_K_P(float CPP_K_P);
    void Set_K_I(float CPP_K_I);
    void Set_K_D(float CPP_K_D);
    void Set_K_F(float CPP_K_F);
    //pid输出限幅
    void Set_I_Out_Max(float CPP_I_Out_Max);
    void Set_Out_Max(float CPP_Out_Max);
    //变速积分设置
    void Set_I_Variable_Speed_A(float CPP_Variable_Speed_I_A);
    void Set_I_Variable_Speed_B(float CPP_Variable_Speed_I_B);
    //积分分离阈值
    void Set_I_Separate_Threshold(float CPP_I_Separate_Threshold);
    //设置目标/当前值
    void Set_Target(float CPP_Target);
    void Set_Now(float CPP_Now);
    //设置积分误差：在控制器断电的时候防止误差累计在通电的时候产生一个很大的I
    void Set_Integral_Error(float CPP_Integral_Error);
    //设置微分先行
    void Set_DFirst(CPP_Enum_DFirst DFirst);

    //用定时器保证PID的时间片精确
    void CPP_TIM_Adjust_PeriodElapsedCallback();

protected:
    CPP_PID_Params_t PID_Params;
    CPP_PID_Info_t PID_Info;
    CPP_Enum_PID_Type PID_Type;
};

#endif //ALGORITHM_PID_HPP
