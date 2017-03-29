/*******************************************************************************
 *    INCLUDED FILES
 ******************************************************************************/
 
//-- unity: unit test framework
#include "unity.h"
#include <stdlib.h>
 
//-- module being tested
#include "../../../Common/Utilities/Logger.h"

 // Mocked modules
 #include "mock_UART.h"
 
/*******************************************************************************
 *    DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 *    PRIVATE TYPES
 ******************************************************************************/
 
/*******************************************************************************
 *    PRIVATE DATA
 ******************************************************************************/
 
/*******************************************************************************
 *    PRIVATE FUNCTIONS
 ******************************************************************************/
 
 
/*******************************************************************************
 *    SETUP, TEARDOWN
 ******************************************************************************/


void setUp(void)
{
}
 
void tearDown(void)
{
}
 
/*******************************************************************************
 *    TESTS
 ******************************************************************************/
 
 /*
 * Note that this test must come first! As the module does not provide a deInitialize
 * method, we need to call this test first before initLogger is called anywhere else!
 */
 void test_LoggerShouldDoNothingIfNotInitialized(void){
    debug("hello");
    error("hello");
    warning("hello");
}

void test_initLoggerShouldCallInitUART(void)
{
    initUART_Expect(LOGGER_UART_INTERFACE, LOGGER_UART_BAUD_RATE, LOGGER_BUFFER_INITIAL_LENGTH, LOGGER_BUFFER_MAX_LENGTH, 0b01);
    initLogger();
}

void test_debugShouldCallQueueUartWithCorrectMessage(void){
    getTXSpace_ExpectAndReturn(LOGGER_UART_INTERFACE,100);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)DEBUG_TAG_STRING, DEBUG_TAG_STRING_LENGTH, DEBUG_TAG_STRING_LENGTH);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)"hello", 5, 5);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)"\r\n\0", 3, 3);

    debug("hello");
}

void test_warningShouldCallQueueUartWithCorrectMessage(void){
    getTXSpace_ExpectAndReturn(LOGGER_UART_INTERFACE,100);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)WARNING_TAG_STRING, WARNING_TAG_STRING_LENGTH, WARNING_TAG_STRING_LENGTH);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)"hello", 5, 5);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)"\r\n\0", 3, 3);

    warning("hello");
}

void test_errorShouldCallQueueUartWithCorrectMessage(void){
    getTXSpace_ExpectAndReturn(LOGGER_UART_INTERFACE,100);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)ERROR_TAG_STRING, ERROR_TAG_STRING_LENGTH,ERROR_TAG_STRING_LENGTH);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)"hello", 5, 5);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)"\r\n\0", 3, 3);

    error("hello");
}

void test_writeMessageWithOneCharacterShouldCallQueueWithCorrectMessage(void){
    getTXSpace_ExpectAndReturn(LOGGER_UART_INTERFACE,100);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)ERROR_TAG_STRING, ERROR_TAG_STRING_LENGTH,ERROR_TAG_STRING_LENGTH);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)"r", 1, 1);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)"\r\n\0", 3, 3); 

    error("r");
}

void test_writeMessageThatIsTooLongShouldNotQueueUART(void){
    getTXSpace_ExpectAndReturn(LOGGER_UART_INTERFACE,1);

    error("thishsouldnotprint");
}

void test_infoShouldCallQueueUartWithCorrectMessage(void){
    getTXSpace_ExpectAndReturn(LOGGER_UART_INTERFACE,100);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)INFO_TAG_STRING, INFO_TAG_STRING_LENGTH, INFO_TAG_STRING_LENGTH);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)"hello", 5, 5);
    queueTXData_ExpectWithArray(LOGGER_UART_INTERFACE, (unsigned char*)"\r\n\0", 3, 3);

    info("hello");
}