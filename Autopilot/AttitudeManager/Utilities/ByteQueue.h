/**
 * @file ByteQueue.h
 * @author Serge Babayan
 * @date Feb 19, 2017
 * @brief
 * Contains generic implementation of a byte queue. This is used extensively where
 * a character buffer is required, such as in the UART implementation. This is an 
 * array based queue that will increase in size by 2x if the size is reached, but
 * will not shrink. Since this is an array based queue, there is little overhead
 * in terms of memory, and methods have been provided to copy the data over for
 * mass consumption to avoid the need for many function calls.
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
typedef volatile struct _ByteQueue{
    unsigned char * _data;
    unsigned int size; //current size of the queue. Can be accessed directly for convenience
    unsigned int _max_size; //max size of the queue
    unsigned int _start_index;
} ByteQueue;

/**
 * Initializes the byte queue given the initial data size. If this number is 
 * exceeded, the queue will try to double in size. It will never shrink in size
 * @param queue
 */
void initBQueue(ByteQueue* queue, unsigned int initial_size);

/**
 * Pop an element from the queue. This will decrease the queue size by 1.Make sure
 * to check the size of the queue before calling this! Unexpected results will occur
 * if the you call this on an empty queue
 * @param queue
 * @return -1 or 256 if the queue is empty, or the value of the next byte if not
 */
unsigned char popBQueue(ByteQueue* queue);

/**
 * Pops all the elements in the queue and copies them into a provided array. Make
 * sure this array is at least the size of the queue, otherwise you'll get data
 * corruption. This method is useful if you want to read the entire queue all at once,
 * which will avoid the overhead of a function call
 * @param queue
 * @param array Byte array that is at least the size of the queue
 */
void popAllBQueue(ByteQueue* queue, unsigned char* array);

/**
 * Push/add a byte onto the end of the queue. If the size of the queue is exceeded
 * by this operation, the size of the queue will double
 * @param queue
 * @param byte The byte to add to the queue
 */
void pushBQueue(ByteQueue* queue, unsigned char byte);

/**
 * Cleans up the byte queue (deallocated its data correctly). If you want to reuse
 * the queue after, make sure to call the initBQueue function on it after words
 * @param queue
 */
void deleteBQueue(ByteQueue* queue);

#endif