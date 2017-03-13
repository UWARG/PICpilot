/**
 * @file ByteQueue.h
 * @author Serge Babayan
 * @date Feb 19, 2017
 * @brief
 * Contains generic implementation of a byte queue. This is used extensively where
 * a character buffer is required, such as in the UART implementation. This is an 
 * array based queue that will increase in size by 2x if it becomes full and if
 * its max size allows it. It will shrink in half if its a quarter full. 
 * 
 * @copyright Waterloo Aerial Robotics Group 2017 \n
 *   https://raw.githubusercontent.com/UWARG/PICpilot/master/LICENCE 
 */

#ifndef BYTEQUEUE_H
#define BYTEQUEUE_H

/**
 * ByteQueue struct. Use the size property to get the size of the queue
 * All of the fields are declared as volatile as it is expected this class
 * is used within ISR's
 */
typedef volatile struct _ByteQueue {
    unsigned char * _data;
    unsigned int _size; //current size of the queue. Can be accessed directly for convenience
    unsigned int _total_size; //current total size of the queue
    unsigned int _initial_size; //Used to make sure that shrink resizing doesn't go below this value
    unsigned int _max_size; //max size of the queue. It cannot be resized to anything larger than this value
    unsigned int _start_index;
} ByteQueue;

/**
 * Initializes the byte queue given the initial data size. If this number is 
 * exceeded, the queue will try to double in size. The max size defines the limit
 * to which the queue can grow. Note that because the queue will also shrink by 1/2
 * if its 1/4 full, its best to define the initial size and max size to be even 
 * multiples of each other. If initial size and max size are the same, the queue
 * will not grow or shrink.
 * @param queue
 */
void initBQueue(ByteQueue* queue, unsigned int initial_size, unsigned int max_size);

/**
 * Pop an element from the queue. This will decrease the queue size by 1.Make sure
 * to check the size of the queue before calling this! Unexpected results will occur
 * if the you call this on an empty queue
 * @param queue
 * @return -1 or 256 if the queue is empty, or the value of the next byte if not
 */
unsigned char popBQueue(ByteQueue* queue);

/**
 * Push/add a byte onto the end of the queue. If the size of the queue is exceeded
 * by this operation, the size of the queue will double
 * @param queue
 * @param byte The byte to add to the queue
 * @return 1 if successfully pushed to queue, 0 otherwise. Will fail if queue is full and can't be resized
 */
unsigned char pushBQueue(ByteQueue* queue, unsigned char byte);

/**
 * This function returns the number of bytes that is guaranteed to be pushed to the queue.
 * This is different from size, as it takes into account what the max size and current
 * size of the queue is.
 * 
 * This is useful for functions where partial data sent isnt useful at all, such
 * as loggers.  You can use this to check if all the data you want to enqueue can fit
 * @param queue
 * @return Bytes that can be guaranteed to be pushed to the queue
 */
unsigned int getBQueueSpace(ByteQueue* queue);

/**
 * Returns the current size of the byte queue (amount of data currently stored on there)
 * @param queue
 * @return size 
 */
unsigned int getBQueueSize(ByteQueue* queue);

/**
 * Cleans up the byte queue (deallocated its data correctly). If you want to reuse
 * the queue after, make sure to call the initBQueue function on it after words
 * @param queue
 */
void deleteBQueue(ByteQueue* queue);

#endif