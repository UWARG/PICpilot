/**.
 * @file ByteQueue.c
 * @author Serge Babayan
 * @date Feb 19, 2017
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#include "ByteQueue.h"
#include <stddef.h>
#include <stdlib.h>

void initBQueue(ByteQueue* queue, unsigned int initial_size, unsigned int max_size)
{
    queue->_data = malloc(initial_size);
    queue->size = 0;
    queue->_total_size = initial_size;
    queue->_start_index = 0;
    queue->_max_size = max_size;
}

unsigned char popBQueue(ByteQueue* queue)
{
    if (queue->size == 0) {
        return -1;
    } else {
        queue->size--;
        unsigned char to_return = queue->_data[queue->_start_index];

        //prevent any overflow
        queue->_start_index = (queue->_start_index + 1) % queue->_total_size;
        
        return to_return;
    }
}

unsigned char pushBQueue(ByteQueue* queue, unsigned char byte)
{
    //if the queue is full
    if (queue->size == queue->_total_size) {
        //if we cant double the size without hitting the max limit, return 0
        if (queue->_total_size*2 > queue->_max_size){
            return 0;
        }
        //otherwise double the size of the queue
        unsigned char* new_data = realloc(queue->_data, queue->_total_size * 2);
        
        if (new_data == 0){ //if we couldn't reallocate more data
            return 0;
        }
        queue->_data = new_data;
        queue->_total_size *= 2;
    }

    queue->_data[(queue->_start_index + queue->size) % queue->_total_size] = byte;
    queue->size++;
    return 1;
}

void deleteBQueue(ByteQueue* queue)
{
    free((void*) queue->_data);
    queue->size = 0;
}