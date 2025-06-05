/**
 * @file Quaternions.c
 * @brief 四元数计算
 * @param Quater(四元数结构体)
 * @param 
 * @author CGH
 * @date 2025/5/26
 */
 #include "Quaternions.h"
 #include "arm_math.h"
 #include "math.h"//没有办法，只能用原生库了
 
 #define M_PI 3.1415926
quaternions_struct_t Quater;







/**
* @brief 初始化姿态（东北天坐标系）
*
* @param gyro 陀螺仪输出数值[3x1]
* @param origin_quater 初始姿态四元数[w,x,y,z]
* @param acc 加速度输出数值[3x1]
* @param magne 磁力计输出值[3x1]

* @return 0: 成功, -1: 失败, 1:妙妙成功
*/
uint8_t Quater_Init(const float* gyro, const float* magne,const float* acc,const float* origin_quater,uint8_t check){
	if(check==1){
	}else{
		
	}
	
	
	
}




/**
* @brief 四元数乘法 q1 * q2
* @param q1 第一个四元数 [w, x, y, z]
* @param q2 第二个四元数 [w, x, y, z]  
* @param result 结果四元数 [w, x, y, z]
*/
void quaternion_multiply(const float* q1, const float* q2, float* result)
{
    float w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
    
    result[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;  // w
    result[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;  // x
    result[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;  // y
    result[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;  // z
}
/**
* @brief 四元数转欧拉角
* @param quaternion 输入四元数 [w, x, y, z]
* @param roll 输出横滚角 (弧度)
* @param pitch 输出俯仰角 (弧度)
* @param yaw 输出偏航角 (弧度)
* @author Claude Sonnet4
*/
void quaternion_to_euler(const float* quaternion, float* roll, float* pitch, float* yaw)
{
    float w = quaternion[0], x = quaternion[1], y = quaternion[2], z = quaternion[3];
    
    // Roll (x轴旋转)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    *roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (y轴旋转)
    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(M_PI / 2.0f, sinp);  // 万向节锁情况
    } else {
        *pitch = asinf(sinp);
    }
    
    // Yaw (z轴旋转)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

//计算模长（的平方）（只对xyz有效）
float calculate_norm(const float *arr){
		float arr_norm;
		arm_sqrt_f32(arr[0]*arr[0] + arr[1]*arr[1] + arr[2]*arr[2], &arr_norm);//求根号
    return arr_norm;
    
}
/**
* @brief 根据加速度计数据计算横滚角(roll)和俯仰角(pitch)
*
* @param g0 初始重力向量 [3x1]
* @param g1 当前重力向量 [3x1]
* @param quaternion 输出四元数 [w, x, y, z]
* @return 0: 成功, -1: 失败, 1:妙妙成功
*/
uint8_t calculate_quaternion_from_gravity(const float* g0, const float* g1, float* quaternion)
{
	float32_t g0_norm, g1_norm;
	float32_t g0_normalized[3], g1_normalized[3];
  float32_t cross_product[3];
  float32_t dot_product;
	
	
	
	//归一化g0和g1
	g0_norm=calculate_norm(g0);
	
	g1_norm=calculate_norm(g1);

	
	// 检查向量是否为零向量    
  if (g0_norm < 1e-6f || g1_norm < 1e-6f) {        
		return -1; //表示失败
  }
	   // 归一化向量
    arm_scale_f32(g0, 1.0f/g0_norm, g0_normalized, 3);
    arm_scale_f32(g1, 1.0f/g1_norm, g1_normalized, 3);
    
    // 计算向量点积（cosθ）
	  arm_dot_prod_f32(g0_normalized, g1_normalized, 3, &dot_product);//点积，因为是归一化的，所以输出就直接是cosθ值
    
    // 限制数值范围
    dot_product = fmaxf(-1.0f, fminf(1.0f, dot_product));
	  //根据cos值反解θ
		acosf(dot_product);
    
	// 检查向量是否已经对齐（这个AI写的，我不清楚为什么要加进来）
    if (dot_product > 0.999999f) {
        quaternion[0] = 1.0f;  // w
        quaternion[1] = 0.0f;  // x
        quaternion[2] = 0.0f;  // y
        quaternion[3] = 0.0f;  // z
        return 0;
    }
    
    // 处理相反向量的情况（180度旋转）
    if (dot_product < -0.999999f) {
        // 找一个垂直轴进行180度旋转
        float32_t axis[3];
        if (fabsf(g0_normalized[0]) < 0.9f) {
            axis[0] = 1.0f; axis[1] = 0.0f; axis[2] = 0.0f;
        } else {
            axis[0] = 0.0f; axis[1] = 1.0f; axis[2] = 0.0f;
        }
        
        // 正交化处理
        float32_t dot_temp;
        arm_dot_prod_f32(axis, g0_normalized, 3, &dot_temp);  // 计算投影
        float32_t temp_vec[3];
        arm_scale_f32(g0_normalized, dot_temp, temp_vec, 3);  // 计算投影向量
        arm_sub_f32(axis, temp_vec, axis, 3);                // 去除投影分量
        
        // 归一化正交轴
        float32_t axis_norm;
        arm_power_f32(axis, 3, &axis_norm);
        arm_sqrt_f32(axis_norm, &axis_norm);
        arm_scale_f32(axis, 1.0f/axis_norm, axis, 3);
        
        // 构造180度旋转的四元数
        quaternion[0] = 0.0f;     // w = cos(π/2) = 0
        quaternion[1] = axis[0];  // x分量
        quaternion[2] = axis[1];  // y分量
        quaternion[3] = axis[2];  // z分量
        return 0;
		}
		// 计算叉积
    cross_product[0] = g0_normalized[1]*g1_normalized[2] - g0_normalized[2]*g1_normalized[1];
    cross_product[1] = g0_normalized[2]*g1_normalized[0] - g0_normalized[0]*g1_normalized[2];
    cross_product[2] = g0_normalized[0]*g1_normalized[1] - g0_normalized[1]*g1_normalized[0];
    
    // 计算四元数
    quaternion[0] = sqrtf((1.0f + dot_product) / 2.0f);  // w（用cos反解θ）
    float32_t w_inv = 1.0f / (2.0f * quaternion[0]);//方便后面计算
		
    //计算四元数（根据w计算因子（好高大上的名字）
		quaternion[1] = cross_product[0]*w_inv;  // x
    quaternion[2] = cross_product[1]*w_inv;  // y
    quaternion[3] = cross_product[2]*w_inv;  // z
		

		return 1;
}

/**
* @brief 四元数归一化
* @param quaternion 输入输出四元数 [w, x, y, z]
*/
void quaternion_normalize(float* quaternion)
{
    float norm_squared = quaternion[0]*quaternion[0] + quaternion[1]*quaternion[1] + 
                        quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3];
    
    if (norm_squared < 1e-6f) {
        // 设置为单位四元数
        quaternion[0] = 1.0f;
        quaternion[1] = quaternion[2] = quaternion[3] = 0.0f;
        return;
    }
    
    float norm;
    arm_sqrt_f32(norm_squared, &norm);
    arm_scale_f32(quaternion, 1.0f/norm, quaternion, 4);
}

/**
* @brief 对陀螺仪进行积分来获取四元数角度更新（一阶龙格库塔法）
*
* @param gyro 陀螺仪输出数值[3x1]
* @param origin_quater 初始姿态四元数[4x1]
* @param dt 运行间隔
* @param quaternion 输出四元数 [w, x, y, z]
* @return 0: 成功, -1: 失败, 1:妙妙成功
*/
uint8_t caculate_angle(const float* gyro, const float* origin_quater, float* quaternion, float dt)
{
    // 检查输入参数
    if (gyro == NULL || origin_quater == NULL || quaternion == NULL || dt <= 0) {
        return -1;
    }
    
    // 构造角速度四元数 [0, wx, wy, wz]
    float omega_q[4] = {0.0f, gyro[0], gyro[1], gyro[2]};
    float temp_q[4];//导数
		float delta_q[4];
    // 计算四元数导数：dq/dt = 0.5 * w*q （右乘，作用于物体坐标系）
  
    quaternion_multiply(origin_quater, omega_q, temp_q);//qw
    
    // 除以二
    arm_scale_f32(temp_q, 0.5f, temp_q, 4);
    
    // 一阶积分：q(t+dt) = q(t) + dq/dt * dt
    
    arm_scale_f32(temp_q, dt, delta_q, 4);
    arm_add_f32(origin_quater, delta_q, quaternion, 4);
    
    // 归一化保持单位四元数性质
    quaternion_normalize(quaternion);
		
    
    return 1;
}

