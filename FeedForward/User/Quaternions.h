#include "math.h"
//CMSIS好像要求用float32_t，可能是为了可移植性考虑。我这里就不管了
typedef struct
{
	
  float A_x;
	float A_y;
	float A_z;
}acceleration;
typedef struct
{
	float w;
  float x;
	float y;
	float z;
	acceleration Acc;
}quaternions_struct_t;
