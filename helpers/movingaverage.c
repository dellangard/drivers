/*
 * movingaverage.c
 *
 *  Created on: 11 февр. 2017 г.
 *      Author: frost
 */

#include <stddef.h>

#include "movingaverage.h"

int32_t ma_add_new_value(movingaverage_t *ma, const int32_t new_value)
{
	//Substract oldest array element value from sum
	ma->sum -= ma->buf[ma->idx];

	//Add new element to array at current index
	ma->buf[ma->idx] = new_value;

	//Add most recent element's value to sum
	(ma->sum) += new_value;

	//Increment index
	(ma->idx)++;

	if(ma->idx == ma->size)
	{
		//Move index to start of the array
		ma->idx = 0;
	}

	//Calculate and return average value
	return (ma->sum / ma->size);
}

int32_t ma_get_average(const movingaverage_t *ma)
{
	//Calculate and return average value
	return (ma->sum / ma->size);
}

void ma_fill(movingaverage_t *ma, const int32_t fill_value)
{
	uint8_t i;
	i = 0;

	ma->sum = ma->size*fill_value;

	while(i < ma->size)
	{
		ma->buf[i] = fill_value;
		i++;
	}
}

void ma_init(movingaverage_t *ma, int32_t *buf, const int32_t size, const int32_t initial_value)
{
	//Initial fill of averaging array and sum calculation
	ma->buf = buf;
	ma->idx = 0;
	ma->size = size;
	ma->sum = 0;

	ma_fill(ma, initial_value);
}


//Create and initialize MA structure with dynamic buffer
movingaverage_t* ma_init_dynamic(const int32_t buf_size, const int32_t initial_value)
{
    movingaverage_t *ma;

    //Allocate memory for moving average control structure
    ma = (movingaverage_t*)pvPortMalloc(sizeof(movingaverage_t));

    //Exit with error if memory allocation failed
    if(ma == NULL)
        return NULL;

    //Initial fill of averaging array and sum calculation
    ma->buf = (int32_t*)pvPortMalloc( buf_size*sizeof(int32_t) );

    //Exit with error if memory allocation failed
    if(ma->buf == NULL)
        return NULL;

    ma->idx = 0;
    ma->size = buf_size;
    ma->sum = 0;

    ma_fill(ma, initial_value);

    return ma;
}

//De-initialize MA buffer, control structure and free memory
void ma_deinit_dynamic(movingaverage_t *ma)
{
    if(ma != NULL)
    {
        //Free buffer memory
        vPortFree(ma->buf);

        //Free control structure memory
        vPortFree(ma);
    }
}



