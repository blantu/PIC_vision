#ifndef _SENSOR
#define _SENSOR

#include <vector>
#include "sl_zed/Camera.hpp"
#include "opencv2/opencv.hpp"


class Sensor
{
public:
	Sensor(int argc, char **argv);
	void SetColorRange(int low=0, int high=360) { _h_low = low; _h_high = high; };
	void SetDistanceRange(float low = 0.5, float high = 10) { _d_low = low; _d_high = high; };
	bool Start();
	void Stop() { zed_grab_thread.detach(); };

private:
	void GrabImage();
	std::vector<cv::Rect> ImageAnalyse(cv::Mat&, cv::Mat&, sl::Mat&);

	sl::Mat image_zed;
	sl::Mat depth_image_zed;
	sl::Mat point_cloud;
		
	cv::Mat image_ocv;
	cv::Mat depth_image_ocv;

	sl::Camera zed;
	sl::Resolution zed_image_size;
	sl::InitParameters zed_init_params;
	sl::ERROR_CODE zed_error;
	std::thread zed_grab_thread;

	int _h_low,_h_high;
	float _d_low = 0.5;
	float _d_high = 10;
};



#endif
