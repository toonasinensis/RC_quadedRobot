#ifndef __GENERAL_MATH__
#define __GENERAL_MATH__

#include "main.h"

#define REAL   float
#define TAN_MAP_RES      0.003921569f     /* (smallest non-zero value in table) */
#define RAD_PER_DEG      0.017453293f
#define TAN_MAP_SIZE     256
#define MY_PPPIII        3.14159f
#define MY_PPPIII_HALF   1.570796f

#define my_3_norm(x,y,z) (my_sqrt(my_pow((x)) + my_pow((y)) + my_pow((z))))
#define my_2_norm(x,y) (my_sqrt(my_pow((x)) + my_pow((y))))

#define my_pow(a) ((a)*(a))
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )
#define ABS(x) ( (x)>0?(x):-(x) )

#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
#define LIMIT_ZERO( x,abs ) ( (((x) >= (-abs)) &((x) <= (abs)))  ?  0.0f : x )

#define _MIN(a, b) ((a) < (b) ? (a) : (b))
#define _MAX(a, b) ((a) > (b) ? (a) : (b))

#define my_pow_2_curve(in,a,max) (((1.0f - (a)) + (a) *LIMIT(ABS((in) / (max)),0,1)) * in)

#define To_180_degrees range_to_180deg
#define range_to_180deg(a) ( (a)>180 ? (a - 360) : ((a)<-180 ? (a + 360) : (a)) )

bool Is_Float_Equal(float a, float b);
float my_abs(float f);
REAL fast_atan2(REAL y, REAL x);
float my_sqrt(float number);
float my_sqrt_reciprocal(float number);
double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float invSqrt(float x);
float my_deadzone(float x,float,float zoom);
float my_deadzone_2(float x,float,float zoom);

float my_deadzone_p(float x,float zone);
float my_deadzone_n(float x,float zone);

double To_180_degrees_db(double x);
float fifo(u8 arr_num,u8 *cnt,float *arr,float in);
float linear_interpolation_5(float range[5],float interpolation[5],float in);//range ??????
void length_limit(float *in1,float *in2,float limit,float out[]);
SCHAR8 Int_Sign_Judge(SINT32 s32_Judge_Number);
#endif

