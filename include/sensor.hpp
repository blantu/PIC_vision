#ifndef _SENSOR
#define _SENSOR

#include <vector>
#include "includes.hpp"
#include "planning.hpp"

class Sensor
{
public:
	enum CalcMode
	{
		BIGGEST,
		BIGGEST2,
		ALL,
	};

	enum ReSize
	{
		FULL=1,
		HALF,
		THRID,
		QUARTERN,
	};

	Sensor(int argc, char **argv);
	void SetColorRange(int low=0, int high=360) { _h_low = low; _h_high = high; };
	void SetDistanceRange(float low = 0.5, float high = 10) { _d_low = low; _d_high = high; };
	void SetMinROI(int width, int height) { _min_rct = cv::Size2i(width, height); };
	void SetMinROI(cv::Size2i rct) { _min_rct = rct; };
	void SetRecord(bool re = false) { _record = re; };

	bool Start(ReSize size = FULL, CalcMode mode = BIGGEST);
	void Stop();

	void Display(cv::Mat&, std::vector<std::string>&, std::vector<cv::Rect>&);
	void Display(cv::Mat&, std::vector<std::string>&, cv::Rect&);
	void Display(cv::Mat&, std::string&, cv::Rect&);
	void Display(cv::Mat&);
	void Display();

	cv::Mat ImageAnalyse(cv::Mat&, cv::Mat&);
	std::vector<Obstacle> DistanceCalc(cv::Mat&, cv::Mat&, CalcMode);

private:
	const int IMG_MAX_ERROR = 10;
	const int REFESH_GAP = 100;

	void GrabImage();
	void CheckObj(CalcMode);

	sl::Mat image_zed;
	sl::Mat depth_image_zed;
	sl::Mat point_cloud_zed;
		
	cv::Mat image_ocv;
	cv::Mat image_bw_ocv;
	cv::Mat depth_image_ocv;
	cv::Mat point_cloud_ocv;

	cv::VideoWriter outputVideo;

	sl::Camera zed;
	float zed_fps;
	sl::Resolution zed_image_size;
	sl::InitParameters zed_init_params;
	sl::ERROR_CODE zed_error;

	std::mutex mutex_img;
	std::condition_variable cond_img;
	std::thread zed_grab_thread;
	std::thread obj_get_thread;

	std::vector<Obstacle> obstacle_objs;

	cv::Size2i _min_rct = cv::Size2i(0, 0);
	CalcMode mode;
	int _h_low = 0,_h_high = 360;
	float _d_low = 0.5;
	float _d_high = 10;
	bool _record = false;
	std::atomic_int _grab_error_counts;
};



#endif
