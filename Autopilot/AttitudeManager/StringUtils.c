/*
 * File:   StringUtils.c
 *
 * Created on August 24, 2014, 06:12 PM
 */
#include <stdlib.h>
#include <string.h>

char* concat(char* str1, char* str2){
    size_t len1 = strlen(str1);
    size_t len2 = strlen(str2);
    char *result = malloc(len1+len2+1);//+1 for the zero-terminator
    //Check for malloc errors
    if (result == 0)
        return 0;
    memcpy(result, str1, len1);
    memcpy(result+len1, str2, len2+1);//+1 to copy the null-terminator
    return result;
}