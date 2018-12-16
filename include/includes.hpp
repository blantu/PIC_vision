#ifndef _INCLUDES
#define _INCLUDES

#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <iomanip> 
#include <sstream> 

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <cstdlib>
#include <ctime>
#include <cmath>

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include "sl_zed/Camera.hpp"


#if defined (WIN32) || defined (_WIN32) || defined (WIN64) || defined (_WIN64)
#include "windows.h"
#include "time.h"
#define sleep(x) Sleep((x))



#elif defined (__unix__) || defined(__linux__)
#include "unistd.h"


#endif




#endif
