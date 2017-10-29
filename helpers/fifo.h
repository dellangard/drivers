/*
 * fifo.h
 *
 *  Created on: 7 февр. 2017 г.
 *      Author: frost
 */

#ifndef FIFO_H_
#define FIFO_H_

#include <stdint.h>

typedef struct
{
	char * buf;
	int32_t head;
	int32_t tail;
	int32_t size;
} fifo_t;


void fifo_init(fifo_t * f, char * buf, int32_t size);
fifo_t* fifo_create_dynamic(int32_t size);
int32_t fifo_read(fifo_t * f, void * buf, int32_t nbytes);
int32_t fifo_read_str(fifo_t * f, void * buf);
int8_t fifo_read_byte(fifo_t * f, char *c);
int32_t fifo_write(fifo_t * f, const void * buf, int32_t nbytes);
int8_t fifo_write_byte(fifo_t * f, const char byte);

void fifo_set_head(fifo_t * f, int32_t new_head);

int32_t fifo_get_head(const fifo_t *f);
int32_t fifo_get_tail(const fifo_t *f);

#endif /* FIFO_H_ */
