/*
 * movingaverage.h
 *
 *  Created on: 11 февр. 2017 г.
 *      Author: frost
 */

#ifndef MOVINGAVERAGE_H_
#define MOVINGAVERAGE_H_

#include <stdint.h>

typedef struct
{
	int32_t *buf;
	uint32_t idx;
	int32_t sum;
	uint32_t size;
} movingaverage_t;

movingaverage_t* ma_init_dynamic(const int32_t buf_size, const int32_t initial_value);

void ma_deinit_dynamic(movingaverage_t *ma);

int32_t ma_add_new_value(movingaverage_t *ma, const int32_t new_value);

int32_t ma_get_average(const movingaverage_t *ma);


#endif /* MOVINGAVERAGE_H_ */
