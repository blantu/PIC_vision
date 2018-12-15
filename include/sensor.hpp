#ifndef _SENSOR
#define _SENSOR

#include "sl_zed/Camera.hpp"
#include "opencv2/opencv.hpp"


class Sensor
{
public:
	Sensor(int argc, char **argv);
	bool Start();
	void Stop() { zed_grab_thread.detach(); };

private:
	void GrabImage();

	sl::Camera zed;
	sl::InitParameters zed_init_params;
	sl::ERROR_CODE zed_error;
	std::thread zed_grab_thread;
};



#endif
