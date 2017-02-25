/*******************************************************************************
 *    INCLUDED FILES
 ******************************************************************************/
 
//-- unity: unit test framework
#include "unity.h"
#include <stdlib.h>
 
//-- module being tested
#include "../../../Common/Utilities/ByteQueue.h"
 
/*******************************************************************************
 *    DEFINITIONS
 ******************************************************************************/
#define QUEUE_SIZE 10

/*******************************************************************************
 *    PRIVATE TYPES
 ******************************************************************************/
 
/*******************************************************************************
 *    PRIVATE DATA
 ******************************************************************************/
 static ByteQueue bqueue;
 
/*******************************************************************************
 *    PRIVATE FUNCTIONS
 ******************************************************************************/
 
 
/*******************************************************************************
 *    SETUP, TEARDOWN
 ******************************************************************************/


void setUp(void)
{
    initBQueue(&bqueue, QUEUE_SIZE);
}
 
void tearDown(void)
{
    deleteBQueue(&bqueue);
}
 
/*******************************************************************************
 *    TESTS
 ******************************************************************************/
 
void test_bQueueInitialSizeZero(void)
{
    TEST_ASSERT_EQUAL_INT(0 ,bqueue.size);
}

void test_bQueueMaxSizeEqualToInitialSize(void)
{
    TEST_ASSERT_EQUAL_INT(QUEUE_SIZE ,bqueue._max_size);
}

void test_bQueuePushIncreasesSize(void)
{
    pushBQueue(&bqueue, 'a');
    TEST_ASSERT_EQUAL_INT(1 ,bqueue.size);
}

void test_bQueuePushPop(void)
{
    pushBQueue(&bqueue, 'a');
    TEST_ASSERT_EQUAL_INT(1 ,bqueue.size);
    unsigned char val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(0 ,bqueue.size);
    TEST_ASSERT_EQUAL_UINT8('a' ,val);
}

void test_bQueueMultiplePushPop(void){
    pushBQueue(&bqueue, 'a');
    TEST_ASSERT_EQUAL_INT(1 ,bqueue.size);
    pushBQueue(&bqueue, 'b');
    TEST_ASSERT_EQUAL_INT(2 ,bqueue.size);
    pushBQueue(&bqueue, 'c');
    TEST_ASSERT_EQUAL_INT(3 ,bqueue.size);
    pushBQueue(&bqueue, 'd');
    TEST_ASSERT_EQUAL_INT(4 ,bqueue.size);
    unsigned char val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(3 ,bqueue.size);
    TEST_ASSERT_EQUAL_UINT8('a' ,val);
    val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(2 ,bqueue.size);
    TEST_ASSERT_EQUAL_UINT8('b' ,val);
    val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(1 ,bqueue.size);
    TEST_ASSERT_EQUAL_UINT8('c' ,val);
    val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(0 ,bqueue.size);
    TEST_ASSERT_EQUAL_UINT8('d' ,val);
}

void test_bQueueEmptyPop(void){
    unsigned char val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(0 ,bqueue.size);
    TEST_ASSERT_EQUAL_UINT8(255 ,val);
}

void test_bQueueEndQueuePushPop(void){
    deleteBQueue(&bqueue);
    initBQueue(&bqueue, 4);
    pushBQueue(&bqueue, 'a');
    pushBQueue(&bqueue, 'a');
    pushBQueue(&bqueue, 'a');
    pushBQueue(&bqueue, 'a');
    popBQueue(&bqueue);
    popBQueue(&bqueue);
    popBQueue(&bqueue);
    popBQueue(&bqueue);
    
    //should be empty now, do the same thing again
    pushBQueue(&bqueue, 'b');
    pushBQueue(&bqueue, 'c');
    pushBQueue(&bqueue, 'd');
    pushBQueue(&bqueue, 'e');
    unsigned char val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_UINT8('b' ,val);
    val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_UINT8('c' ,val);
    val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(1 ,bqueue.size);
    TEST_ASSERT_EQUAL_UINT8('d' ,val);
    val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_UINT8('e' ,val);
}

void test_bQueueSizeDouble(void){
    int i;
    unsigned char* array = malloc(QUEUE_SIZE + 1);
    for (i = 0; i< QUEUE_SIZE + 1; i++){
        pushBQueue(&bqueue, i + 64);
        array[i] = i + 64;
    }
    TEST_ASSERT_EQUAL_INT(QUEUE_SIZE + 1 ,bqueue.size);
    TEST_ASSERT_EQUAL_INT(QUEUE_SIZE*2 ,bqueue._max_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(bqueue._data,array , QUEUE_SIZE + 1);
}

void test_bQueuePopAllQueue(void){
    //do a few push pops so that the start index is non-zero
    pushBQueue(&bqueue, 'a');
    pushBQueue(&bqueue, 'a');
    pushBQueue(&bqueue, 'a');
    pushBQueue(&bqueue, 'a');
    popBQueue(&bqueue);
    popBQueue(&bqueue);
    popBQueue(&bqueue);
    popBQueue(&bqueue);
    
    int i;
    unsigned char* to_copy = malloc(QUEUE_SIZE);
    unsigned char* expected = malloc(QUEUE_SIZE);
    //have the queue go full
    for (i = 0; i< QUEUE_SIZE; i++){
        pushBQueue(&bqueue, i + 64);
        expected[i] = i + 64;
    }
    
    popAllBQueue(&bqueue, to_copy);
    TEST_ASSERT_EQUAL_INT(0 ,bqueue.size); //size should be 0
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected,to_copy , QUEUE_SIZE);
}

