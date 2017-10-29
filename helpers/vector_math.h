/*
 * vector_math.h
 *
 *  Created on: 18 мар. 2017 г.
 *      Author: frost
 */

#ifndef VECTOR_MATH_H_
#define VECTOR_MATH_H_

#include <stdint.h>
#include "arm_math.h"

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} int16vector_t;

typedef struct
{
	int32_t x;
	int32_t y;
	int32_t z;
} int32vector_t;


typedef struct tFloatVector
{
	float32_t x;
	float32_t y;
	float32_t z;
} floatvector_t;

typedef struct tQuaternion
{
	float32_t q0;
	float32_t q1;
	float32_t q2;
	float32_t q3;
} quaternion_t;

void vector_cross(const floatvector_t *a, const floatvector_t *b, floatvector_t *out);
float32_t vector_dot(const floatvector_t *a, const floatvector_t *b);
void vector_normalize(floatvector_t *a);
float32_t invSqrt(float32_t number);

#endif /* VECTOR_MATH_H_ */

