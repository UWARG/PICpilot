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

#define QUEUE_MAX_SIZE 100

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
    initBQueue(&bqueue, QUEUE_SIZE, QUEUE_MAX_SIZE);
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

void test_bQueueAvailableSpaceShouldBeSetCorrectly(void)
{
    TEST_ASSERT_EQUAL_INT(QUEUE_MAX_SIZE ,getBQueueSpace(&bqueue));
}

void test_bQueueMaxSizeAndTotalSizeShouldBeSetCorrectly(void)
{
    TEST_ASSERT_EQUAL_INT(QUEUE_SIZE ,bqueue._total_size);
    TEST_ASSERT_EQUAL_INT(QUEUE_MAX_SIZE ,bqueue._max_size);
    TEST_ASSERT_EQUAL_INT(QUEUE_SIZE ,bqueue._initial_size);
}

void test_bQueuePushIncreasesSize(void)
{
    pushBQueue(&bqueue, 'a');
    TEST_ASSERT_EQUAL_INT(1 ,getBQueueSize(&bqueue));
}

void test_bQueueSuccessfulPushReturnsTrue(void)
{
    unsigned char status = pushBQueue(&bqueue, 'a');
    TEST_ASSERT_EQUAL_INT(1 ,status);
}

void test_bQueuePushPop(void)
{
    pushBQueue(&bqueue, 'a');
    TEST_ASSERT_EQUAL_INT(1 ,getBQueueSize(&bqueue));
    unsigned char val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(0 ,getBQueueSize(&bqueue));
    TEST_ASSERT_EQUAL_UINT8('a' ,val);
}

void test_bQueueMultiplePushPop(void){
    pushBQueue(&bqueue, 'a');
    TEST_ASSERT_EQUAL_INT(1 ,getBQueueSize(&bqueue));
    pushBQueue(&bqueue, 'b');
    TEST_ASSERT_EQUAL_INT(2 ,getBQueueSize(&bqueue));
    pushBQueue(&bqueue, 'c');
    TEST_ASSERT_EQUAL_INT(3 ,getBQueueSize(&bqueue));
    pushBQueue(&bqueue, 'd');
    TEST_ASSERT_EQUAL_INT(4 ,getBQueueSize(&bqueue));
    unsigned char val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(3 ,getBQueueSize(&bqueue));
    TEST_ASSERT_EQUAL_UINT8('a' ,val);
    val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(2 ,getBQueueSize(&bqueue));
    TEST_ASSERT_EQUAL_UINT8('b' ,val);
    val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(1 ,getBQueueSize(&bqueue));
    TEST_ASSERT_EQUAL_UINT8('c' ,val);
    val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(0 ,getBQueueSize(&bqueue));
    TEST_ASSERT_EQUAL_UINT8('d' ,val);
}

void test_bQueueEmptyPop(void){
    unsigned char val = popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(0 ,getBQueueSize(&bqueue));
    TEST_ASSERT_EQUAL_UINT8(255 ,val);
}

void test_bQueueEndQueuePushPop(void){
    deleteBQueue(&bqueue);
    initBQueue(&bqueue, 4, QUEUE_MAX_SIZE);
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
    TEST_ASSERT_EQUAL_INT(1 ,getBQueueSize(&bqueue));
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
    TEST_ASSERT_EQUAL_INT(QUEUE_SIZE + 1 ,getBQueueSize(&bqueue));
    TEST_ASSERT_EQUAL_INT(QUEUE_SIZE*2 ,bqueue._total_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(bqueue._data,array , QUEUE_SIZE + 1);
}

void test_bQueueShouldNotDoublePastMaxSize(void){
    deleteBQueue(&bqueue);
    initBQueue(&bqueue, 4, 5);
    int i;
    unsigned char* array = malloc(5);
    unsigned char last_status_value = 1;
    for (i = 0; i< 10; i++){
        last_status_value = pushBQueue(&bqueue, i + 64);
        array[i] = i + 64;
    }
    TEST_ASSERT_EQUAL_INT(5 ,getBQueueSize(&bqueue));
    TEST_ASSERT_EQUAL_INT(5 ,bqueue._total_size);
    TEST_ASSERT_EQUAL_INT(0 ,last_status_value);
}

void test_bQueueShouldHalfSizeIfQuarterFull(void){
    deleteBQueue(&bqueue);
    initBQueue(&bqueue, 4, 8);
    int i;
    for (i = 0; i< 8; i++){ //fill up the queue to the max size
        pushBQueue(&bqueue, i + 64);
    }
     for (i = 0; i< 6; i++){ //remove all but 2 values, which should make the size back to 4
        popBQueue(&bqueue);
    }
    
    TEST_ASSERT_EQUAL_INT(4 ,bqueue._total_size);
    
    //remove the last 2, the size shouldnt go below the initial size of 4
    popBQueue(&bqueue);
    popBQueue(&bqueue);
    TEST_ASSERT_EQUAL_INT(4 ,bqueue._total_size);
}