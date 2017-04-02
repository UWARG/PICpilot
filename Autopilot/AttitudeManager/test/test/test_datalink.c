/*******************************************************************************
 *    INCLUDED FILES
 ******************************************************************************/
 
//-- unity: unit test framework
#include "unity.h"
#include "mock_Radio.h"
#include "../../Network/Datalink.h"
#include <stdlib.h>
 
//-- module being tested
//   TODO
// you would include the header of the module being tested here
 
 
/*******************************************************************************
 *    DEFINITIONS
 ******************************************************************************/
 
/*******************************************************************************
 *    PRIVATE TYPES
 ******************************************************************************/
 
/*******************************************************************************
 *    PRIVATE DATA
 ******************************************************************************/
int i;
uint8_t test_data[20];
uint8_t test_data2[34];
uint16_t test_data_length;
uint16_t test_data2_length;
/*******************************************************************************
 *    PRIVATE FUNCTIONS
 ******************************************************************************/
 
 
/*******************************************************************************
 *    SETUP, TEARDOWN
 ******************************************************************************/
 
void setUp(void)
{
    for(i = 0; i < 20; i ++){
        test_data[i] = i + 1;
    }
    test_data_length = 20;
    
    for(i = 0; i < 34; i ++){
        test_data2[i] = i + 1;
    }
    test_data2_length = 34;
}
 
void tearDown(void)
{
    
}
 
/*******************************************************************************
 *    TESTS
 ******************************************************************************/
 
void test_initDatalinkShouldInitializeRadio(void)
{
    initRadio_Expect();
    initDatalink();
}

void test_parseDatalinkBufferShouldDoNothingWithNoIncomingData(void)
{
    uint16_t length;
    parseUplinkPacket_ExpectAndReturn(&length, NULL);
    parseDatalinkBuffer();
}

uint8_t* parseDatalinkBufferValidDataMock(uint16_t* length, int NumCalls){
    NumCalls++; //so compiler doesn't complain
    *length = test_data_length;
    return test_data;
}

uint8_t* parseDatalinkBufferValidDataMock2(uint16_t* length, int NumCalls){
    NumCalls++; //so compiler doesn't complain
    *length = test_data2_length;
    return test_data2;
}

void test_parseDatalinkBufferShouldAddToCommandQueueWithIncomingData(void)
{
    parseUplinkPacket_StubWithCallback((CMOCK_parseUplinkPacket_CALLBACK) parseDatalinkBufferValidDataMock);
    parseDatalinkBuffer();
    DatalinkCommand* command = popDatalinkCommand();
    
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(test_data[0], command->cmd, "Command ID should be first byte of received uplink");
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(&test_data[1], command->data, test_data_length - 1, "Command payload should be copied correctly");
    TEST_ASSERT_NULL(command->next);
    free(command);
}

void test_parseDatalinkBufferPopCommandMultiple(void)
{
    parseUplinkPacket_StubWithCallback((CMOCK_parseUplinkPacket_CALLBACK) parseDatalinkBufferValidDataMock);
    parseDatalinkBuffer();
    parseUplinkPacket_StubWithCallback((CMOCK_parseUplinkPacket_CALLBACK) parseDatalinkBufferValidDataMock2);
    parseDatalinkBuffer();
    
    DatalinkCommand* command1 = popDatalinkCommand();
    DatalinkCommand* command2 = popDatalinkCommand();
    
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(test_data[0], command1->cmd, "Command ID should be first byte of received uplink");
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(&test_data[1], command1->data, test_data_length - 1, "Command payload should be copied correctly");
    TEST_ASSERT_NULL(command1->next);
  
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(test_data2[0], command2->cmd, "Command ID should be first byte of received uplink");
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(&test_data2[1], command2->data, test_data2_length - 1, "Command payload should be copied correctly");
    TEST_ASSERT_NULL(command2->next);
    free(command1);
    free(command2);
}

void test_popDatalinkCommandShouldReturnNullIfNoCommands(void)
{
    TEST_ASSERT_NULL(popDatalinkCommand());
}

void test_freeDatalinkCommandShouldDeleteCommandWithoutErrors(void)
{
    
    DatalinkCommand* command = malloc(sizeof(DatalinkCommand));
    command->data = malloc(12) + 1;
    command->data_length = 12;
    command->next = NULL;
    freeDatalinkCommand(command);
}

void test_createTelemetryBlockShouldReturnWithoutErrors(void)
{
    TelemetryBlock* block = createTelemetryBlock(PRIORITY0);
    free(block);
}

void test_queueTelemetryBlockShouldCallRadioQueueMethodAndReturnTrue(void)
{
    TelemetryBlock* block = createTelemetryBlock(PRIORITY0);
    queueDownlinkPacket_IgnoreAndReturn(true);
    TEST_ASSERT_TRUE(queueTelemetryBlock(block));
    free(block);
}

void test_queueTelemetryBlockShouldCallRadioQueueMethodAndReturnFalse(void)
{
    TelemetryBlock* block = createTelemetryBlock(PRIORITY0);
    queueDownlinkPacket_IgnoreAndReturn(false);
    TEST_ASSERT_TRUE(queueTelemetryBlock(block));
    free(block);
}