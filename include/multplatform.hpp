#ifndef _MULTPLATFORM
#define _MULTPLATFORM

#include <iostream>
#include <vector>

#include <thread>

#include <cstdlib>
#include <cstdlib>
#include <ctime>
#include <cmath>


#if defined (WIN32) || defined (_WIN32) || defined (WIN64) || defined (_WIN64)
#include "windows.h"
#include "time.h"
#define sleep(x) Sleep((x))



#elif defined (__unix__) || defined(__linux__)
#include "unistd.h"


#endif




#endif
