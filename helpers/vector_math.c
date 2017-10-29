/*
 * vector_math.c
 *
 *  Created on: 18 мар. 2017 г.
 *      Author: frost
 */

#include "vector_math.h"

void vector_cross(const floatvector_t *a, const floatvector_t *b, floatvector_t *out)
{
	out->x = a->y*b->z - a->z*b->y;
	out->y = a->z*b->x - a->x*b->z;
	out->z = a->x*b->y - a->y*b->x;
}

float32_t vector_dot(const floatvector_t *a, const floatvector_t *b)
{
	return a->x*b->x + a->y*b->y + a->z*b->z;
}

void vector_normalize(floatvector_t *a)
{
	float32_t magnitude = __builtin_sqrtf(vector_dot(a,a));
	a->x /= magnitude;
	a->y /= magnitude;
	a->z /= magnitude;
}

void quaternion_normalize(quaternion_t *a)
{
	float32_t magnitude = __builtin_sqrtf( (a->q0 * a->q0) + (a->q1 * a->q1) + (a->q2 * a->q2) + (a->q3 * a->q3) );
	a->q0 /= magnitude;
	a->q1 /= magnitude;
	a->q2 /= magnitude;
	a->q3 /= magnitude;
}

//// 1/sqrt(number)
//float32_t invSqrt(float32_t number)
//{
//	volatile int32_t i;
//	volatile float32_t x, y;
//	volatile const float32_t f = 1.5F;
//
//	x = number * 0.5F;
//	y = number;
//	i = * (( int32_t * ) &y);
//	i = 0x5f375a86 - ( i >> 1 );
//	y = * (( float32_t * ) &i);
//	y = y * ( f - ( x * y * y ) );
//	return y;
//}

