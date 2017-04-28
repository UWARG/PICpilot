/**
 * @file
 * @author
 * 
 * @date
 */

#include <xc.h>
#include "Logger.h"

void __attribute__((__interrupt__, no_auto_psv)) _AddressError(void) {
    error("Address Error");
    INTCON1bits.ADDRERR = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _StackError(void) {
    error("Stack Error Occurred");
    INTCON1bits.STKERR = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _MathError(void) {
    error("Math Error Occurred");
    if (INTCON1bits.DIV0ERR) {
        error("Divide by zero");
    }
    INTCON1bits.MATHERR = 0;
}
