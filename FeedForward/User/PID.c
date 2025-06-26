#include "PID.h"
#include "math.h"
/**************************¼¸¸öPID³ıÁË¹ıÁã±£»¤ÍâÃ»ÓĞÈÎºÎÇø±ğ**********************************/
/********************************************************************************************
*½Ç¶È»·-KI¸ø0
*ËÙ¶È»·-KD¸ø0
*ÏÈµ÷ËÙ¶È»·£¬µÈËÙ¶È»·ÄÜÊµÏÖÌø±ä¸úËæÔÙµ÷½Ç¶È¡£
********************************************************************************************/

float Limit_Min_Max(float value,float min,float max);

/**
 * @brief PIDÊı×é³õÊ¼»¯
 * @param PID PIDÊı×é
 * @param kp 
 * @param ki 
 * @param kd 
 * @param i_max 
 * @param out_max 
 */
void PID_init(PID_struct_t *PID,
              float kp,
              float ki,
              float kd,
              float kf,
              float i_max,
              float out_max)//PID³õÊ¼»¯º¯Êı
{
  PID->kp      = kp;
  PID->ki      = ki;
  PID->kd      = kd;
  PID->kf      = kf;
  PID->i_max   = i_max;//»ı·ÖÏŞ·ù
  PID->out_max = out_max;//Êä³öÏŞ·ù
}

/**
 * @brief PID¹ıÁã±£»¤
 * @param pid PID½á¹¹Ìå
 * @param angle_max ½Ç¶ÈÉÏÏŞ
 */
void PID_Protect(PID_struct_t *pid,float angle_max)
{
  float half_angle = angle_max/2;
	if(pid->ref[0] - pid->fdb > half_angle)
	{
		pid->fdb+=angle_max;
	}
	else if(pid->ref[0] - pid->fdb < -half_angle)
	{
		pid->fdb-=angle_max;
	}
}

/**
 * @brief PID½Ç¶È»·¼ÆËãº¯Êı
 * @param PID PID½á¹¹Ìå
 * @param ref Éè¶¨Öµ
 * @param fdb Êµ¼ÊÖµ
 * @param angle_max ½Ç¶ÈÉÏÏŞ
 * @param i_out »ı·Ö·ÖÀë²ÎÊı£¨Îª0Ê±ÎŞĞ§£©
 * @return PID¼ÆËã½á¹û
 */
float PID_Calc_Angle(PID_struct_t *PID, float ref, float fdb, float angle_max, float integral_threshold)
{
    PID->ref[0] = ref;
    PID->fdb = fdb;
    PID->f_out = PID->kf * (PID->ref[0] - PID->ref[1]);
    PID_Protect(PID, angle_max); // è¿‡è½½ä¿æŠ¤

    PID->err[0] = PID->ref[0] - PID->fdb;

    // æ¯”ä¾‹é¡¹
    PID->p_out = PID->kp * PID->err[0];
    
    // ç§¯åˆ†åˆ†ç¦»ï¼šåªæœ‰å½“è¯¯å·®å°äºé˜ˆå€¼æ—¶æ‰ç´¯ç§¯ç§¯åˆ†
    if (fabs(PID->err[0]) < integral_threshold) {
        PID->i_out += PID->ki * PID->err[0];
    }
    
    // å¾®åˆ†é¡¹
    PID->d_out = PID->kd * (PID->err[0] - PID->err[1]);
    
    // ç§¯åˆ†é™å¹…
    PID->i_out = Limit_Min_Max(PID->i_out, -PID->i_max, PID->i_max);
    
    // æ€»è¾“å‡º
    PID->output = PID->p_out + PID->i_out + PID->d_out + PID->f_out;
    PID->output = Limit_Min_Max(PID->output, -PID->out_max, PID->out_max);

    // æ›´æ–°å†å²å€¼
    PID->err[1] = PID->err[0];
    PID->ref[1] = PID->ref[0];
    
    return PID->output;
}
/**
 * @brief ËÙ¶È»·PID
 * @param PID PID½á¹¹Ìå
 * @param ref Éè¶¨Öµ
 * @param fdb Êµ¼ÊÖµ
 * @return PID¼ÆËã½á¹û
 */
float PID_Calc_Speed(PID_struct_t *PID, float ref, float fdb)
{
  PID->ref[0] = ref;
  PID->fdb = fdb;

  PID->f_out = PID->kf * (PID->ref[0] - PID->ref[1]);

  PID->err[0] = PID->f_out + PID->ref[0] - PID->fdb;

  PID->p_out  = PID->kp * PID->err[0];
  PID->i_out += PID->ki * PID->err[0];
  PID->d_out  = PID->kd * (PID->err[0] - PID->err[1]);
  PID->i_out=Limit_Min_Max(PID->i_out, -PID->i_max, PID->i_max);
  
  PID->output = PID->p_out + PID->i_out + PID->d_out;
  PID->output=Limit_Min_Max(PID->output, -PID->out_max, PID->out_max);

  PID->ref[1] = PID->ref[0];
  PID->err[1] = PID->err[0];

  return PID->output;
}

/**
 * @brief ÏŞÖÆÒ»¸öÕûÊı±äÁ¿ value ÔÚÖ¸¶¨µÄ×îĞ¡Öµ min ºÍ×î´óÖµ max Ö®¼ä
 * @param value ÊäÈëÖµ
 * @param min ×îĞ¡Öµ
 * @param max ×î´óÖµ
 * @return 
 */
float Limit_Min_Max(float value,float min,float max)
{
	if(value<min)
		return min;
	else if(value>max)
		return max;
	else return value;
}

