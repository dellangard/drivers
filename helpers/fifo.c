/*
 * fifo.c
 *
 *  Created on: 7 февр. 2017 г.
 *      Author: frost
 *
 *      Grabbed from https://stratifylabs.co/embedded%20design%20tips/2013/10/02/Tips-A-FIFO-Buffer-Implementation/
 */

#include "fifo.h"
#include <stddef.h>

//This initializes the FIFO structure with the given buffer and size
void fifo_init(fifo_t * f, char * buf, int32_t size)
{
	f->head = 0;
	f->tail = 0;
	f->size = size;
	f->buf = buf;
}

//This creates and initializes the FIFO structure with the given buffer size
fifo_t* fifo_create_dynamic(int32_t size)
{
    //Allocate memory for FIFO control structure
    fifo_t* fifo = (fifo_t*)pvPortMalloc(sizeof(fifo_t));

    if(fifo == NULL)
        return 0;

    fifo->head = 0;
    fifo->tail = 0;
    fifo->size = size;
    fifo->buf = (char*)pvPortMalloc( size*sizeof(char) );

    if(fifo->buf == NULL)
    {
        vPortFree(fifo);

        return NULL;
    }

    return fifo;
}



//This reads nbytes bytes from the FIFO
//The number of bytes read is returned
int32_t fifo_read(fifo_t * f, void * buf, int32_t nbytes)
{
	int32_t i;
	char * p;
	p = buf;

	for(i=0; i < nbytes; i++)
	{
		if( f->tail != f->head )
		{
			//see if any data is available
			*p++ = f->buf[f->tail];  //grab a byte from the buffer
			f->tail++;  //increment the tail

			if( f->tail == f->size )
			{  //check for wrap-around
				f->tail = 0;
			}
		} else
		{
			return i; //number of bytes read
		}
	}

	return nbytes;
}

//This reads string from the FIFO
//Read is performed until '\0' is found or FIFO is empty.
//If FIFO end reached before '\0', then 0 is returned
//Otherwise the number of bytes read is returned
int32_t fifo_read_str(fifo_t * f, void * buf)
{
	int32_t i = 0;
	char *p;
	p = buf;

	do
	{
		//Read data from tail
		*p = f->buf[f->tail];  //grab a byte from the buffer

		//If FIFO is empty (tail reaches head)
		if(f->tail == f->head)
		{
			return 0;	//Return zero if FIFO becomes empty and end of string was not found
		}

		f->tail++;  //increment the tail

		if( f->tail == f->size )
		{  //check for wrap-around
			f->tail = 0;
		}

		i++;

	} while(*p++ != '\0');	//While end of string is not reached

	return i;	//Return number of chars that actually were read
}

int8_t fifo_read_byte(fifo_t * f, char *c)
{

	if( f->tail != f->head )
	{
		*c = f->buf[f->tail];  //grab a byte from the buffer
		f->tail++;  //increment the tail

		if( f->tail == f->size )
		{  //check for wrap-around
			f->tail = 0;
		}

		return 1;
	} else
		return 0;
}

//This writes up to nbytes bytes to the FIFO
//If the head runs in to the tail, not all bytes are written
//The number of bytes written is returned
int32_t fifo_write(fifo_t * f, const void * buf, int32_t nbytes)
{
	int32_t i;
	const char * p;
	p = buf;

	for(i=0; i < nbytes; i++)
	{
		//first check to see if there is space in the buffer
		if( (f->head + 1 == f->tail) || ( (f->head + 1 == f->size) && (f->tail == 0) ))
		{
			return i; //no more room
		} else
		{
			f->buf[f->head] = *p++;
			f->head++;  //increment the head

			if( f->head == f->size )
			{  //check for wrap-around
				f->head = 0;
			}
		}
	}

	return nbytes;
}

int8_t fifo_write_byte(fifo_t * f, const char byte)
{
	//first check to see if there is space in the buffer
	if( (f->head + 1 == f->tail) || ( (f->head + 1 == f->size) && (f->tail == 0) ))
	{
		return 0; //no more room
	} else
	{
		f->buf[f->head] = byte;
		f->head++;  //increment the head

		if( f->head == f->size )
		{  //check for wrap-around
			f->head = 0;
		}
	}

	return 1;
}

void fifo_set_head(fifo_t * f, int32_t new_head)
{
	f->head = new_head;
}

int32_t fifo_get_head(const fifo_t *f)
{
	return f->head;
}

int32_t fifo_get_tail(const fifo_t *f)
{
	return f->tail;
}


