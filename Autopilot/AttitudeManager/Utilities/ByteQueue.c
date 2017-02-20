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

void initBQueue(ByteQueue* queue, unsigned int initial_size)
{
    queue->_data = malloc(initial_size);
    queue->size = 0;
    queue->_max_size = initial_size;
    queue->_start_index = 0;
}

unsigned char popBQueue(ByteQueue* queue){
    if (queue->size == 0){
        return -1;
    } else {
        queue->size--;
        unsigned char to_return = queue->_data[queue->_start_index];
        
        //prevent any overflow
        queue->_start_index = (queue->_start_index + 1) % queue->_max_size;
        return to_return;
    }
}

void popAllBQueue(ByteQueue* queue,unsigned char* array){
    if (queue->size != 0){
        unsigned int i;
        for(i = 0; i<queue->size;i++){
            array[i] = queue->_data[queue->_start_index];
            queue->_start_index = (queue->_start_index + 1) % queue->_max_size;
        }
        queue->size = 0;
        queue->_start_index = 0;
    }
}

void pushBQueue(ByteQueue* queue, unsigned char byte){
    //if the queue is full, create a new one double the size to ensure we have enough storage
    if (queue->size == queue->_max_size){
        unsigned int old_size = queue->size;
        unsigned char* new_data = malloc(queue->_max_size*2);
        popAllBQueue(queue, new_data);
        free((void*)queue->_data); //free the old data
        queue->_data = new_data;
        queue->size = old_size;
        queue->_max_size *= 2;
    }
    
    queue->_data[(queue->_start_index + queue->size)%queue->_max_size] = byte;
    queue->size++;
}

void deleteBQueue(ByteQueue* queue){
    free((void*)queue->_data);
    queue->size = 0;
}