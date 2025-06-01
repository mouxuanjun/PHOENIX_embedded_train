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
quaternions_struct_t Quater;

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
int calculate_quaternion_from_gravity(const float* g0, const float* g1, float* quaternion)
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
    
    // 计算向量点积（cos(θ)）
	  arm_dot_prod_f32(g0_normalized, g1_normalized, 3, &dot_product);//点积，因为是归一化的，所以输出就直接是cosθ值
    
    // 限制数值范围
    dot_product = fmaxf(-1.0f, fminf(1.0f, dot_product));
	  //根据cos值反解θ
		acosf(dot_product);
    
    // 检查向量是否已经对齐
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
* @brief 对陀螺仪进行积分来获取四元数角度
*
* @param 陀螺仪输出数值[3x1]
* @param g1 当前重力向量 [3x1]
* @param quaternion 输出四元数 [w, x, y, z]
* @return 0: 成功, -1: 失败, 1:妙妙成功
*/
uint8_t caculate_angle(){
	
	return 1;
}
