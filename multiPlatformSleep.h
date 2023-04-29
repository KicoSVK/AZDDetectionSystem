#ifndef MULTIPLATFORMSLEEP_H_
#define MULTIPLATFORMSLEEP_H_

#if __has_include("cameraGrabModule.h")    
#include <unistd.h>
#define LINUX
#else
#define NOMINMAX
#include <windows.h>
#endif

/**
* Function for sleep in ms
*
* @param int sleepMs is time in ms for which program will sleep
*/
inline void sleep(int sleepMs) 
{
    #ifdef LINUX
        usleep(sleepMs * 1000);
    #else
        Sleep(sleepMs);
    #endif
}
#endif