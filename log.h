#ifndef LOG_H
#define LOG_H

#include <windows.h>
#include <stdio.h>
#include <stdarg.h>

inline void apiLog(const char* const formatString, ...)
{
    va_list argumentList;
    va_start(argumentList, formatString);
    static char buffer[4096];
    vsnprintf(buffer, 4095, formatString, argumentList);
    OutputDebugString(buffer);
    va_end(argumentList);
}

#endif // LOG_H
