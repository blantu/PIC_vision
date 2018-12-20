#include "sensor.hpp"
#include "zedtools.hpp"
#include "planning.hpp"
#include "includes.hpp"

using namespace sl;
void printHelp();

Sensor::Sensor(int argc, char **argv)
{
	zed_init_params.camera_resolution = RESOLUTION_HD720;
	zed_init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
	zed_init_params.coordinate_units = UNIT_METER;
	if (argc > 1)
		zed_init_params.svo_input_filename.set(argv[1]);
}

bool Sensor::Start(ReSize size, CalcMode mode)
{
	zed_error = zed.open(zed_init_params);
	
	if (zed_error != SUCCESS) {
		printf("%s\n", toString(zed_error).c_str());
		zed.close();
		return false; // Quit if an error occurred
	}

	zed_image_size.height = zed.getResolution().height / size;
	zed_image_size.width = zed.getResolution().width / size;
	zed_fps = zed.getCameraFPS();

	if (_record)
	{
		if (outputVideo.open("./record.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), zed_fps, cv::Size(zed_image_size.width, zed_image_size.height), true) == false)
			std::cout << "video output failed" << std::endl;
		else
			std::cout << "recording..." << std::endl;
	}

	zed_grab_thread = std::thread([this] { GrabImage(); });
	obj_get_thread = std::thread([this, mode]() {CheckObj(mode); });

	printHelp();

	return true;
}

void Sensor::CheckObj(CalcMode mode)
{
	int error_counts = 0;
	while (true)
	{
		static std::vector<Obstacle> obstacle_objs_old;
		std::unique_lock<std::mutex> lock(mutex_img);
		auto wait = cond_img.wait_for(
			lock,
			std::chrono::milliseconds(REFESH_GAP));

		if (wait == std::cv_status::timeout)
		{
			++error_counts;
			obstacle_objs = obstacle_objs_old;
		}
		else
		{
			error_counts = 0;
			obstacle_objs = DistanceCalc(image_bw_ocv, point_cloud_ocv, mode);
		}

		if (error_counts >= IMG_MAX_ERROR)
			break;

		obstacle_objs_old = obstacle_objs;
	}
}

void Sensor::GrabImage()
{
	// Set runtime parameters after opening the camera
	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

	// To share data between sl::Mat and cv::Mat, use slMat2cvMat()
	// Only the headers and pointer to the sl::Mat are copied, not the data itself
	image_zed.alloc(zed_image_size, MAT_TYPE_8U_C4);
	image_ocv = slMat2cvMat(image_zed);
	depth_image_zed.alloc(zed_image_size, MAT_TYPE_32F_C1);
	depth_image_ocv = slMat2cvMat(depth_image_zed);
	point_cloud_zed.alloc(zed_image_size, MAT_TYPE_32F_C4);
	point_cloud_ocv = slMat2cvMat(point_cloud_zed);

	while (true) 
	{
		if (_grab_error_counts >= IMG_MAX_ERROR)
		{
			zed.close();
			break;
		}

		if (zed.grab(runtime_parameters) == SUCCESS) {
			_grab_error_counts = _grab_error_counts >= IMG_MAX_ERROR ? IMG_MAX_ERROR : 0;

			std::lock_guard<std::mutex> lock(mutex_img);
			// Retrieve the left image, depth image in half-resolution
			zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, zed_image_size.width, zed_image_size.height);
			zed.retrieveMeasure(depth_image_zed, MEASURE_DEPTH, MEM_CPU, zed_image_size.width, zed_image_size.height);

			// Retrieve the RGBA point cloud in half-resolution
			// To learn how to manipulate and display point clouds, see Depth Sensing sample
			zed.retrieveMeasure(point_cloud_zed, MEASURE_XYZ, MEM_CPU, zed_image_size.width, zed_image_size.height);

			image_bw_ocv = ImageAnalyse(image_ocv, depth_image_ocv);

			cond_img.notify_one();
		}
		else { ++_grab_error_counts; }
	}
}

cv::Mat Sensor::ImageAnalyse(cv::Mat& color, cv::Mat& depth)
{
	cv::Mat img_hsv, img_color_bw, img_depth_bw, img_dst_bw;

	//change image into hsv mode
	cvtColor(color, img_hsv, CV_BGR2HSV);
	//set ROI range, color and depth
	cv::inRange(img_hsv, cv::Scalar(_h_low, 43, 46), cv::Scalar(_h_high, 255, 255), img_color_bw); //Threshold the image
	cv::inRange(depth, cv::Scalar(_d_low), cv::Scalar(_d_high), img_depth_bw); //Threshold the image
	//"and" ROI regions
	cv::bitwise_and(img_color_bw, img_depth_bw, img_dst_bw);
	//morphology "close"
	cv::morphologyEx(img_dst_bw, img_dst_bw, cv::MORPH_CLOSE, cv::Mat());
	
	return img_dst_bw;
}

std::vector<Obstacle> Sensor::DistanceCalc(cv::Mat& mask, cv::Mat& cloud, CalcMode mode)
{
	std::vector<Obstacle> obs;

	std::vector<cv::Rect> ROI;
	std::vector<std::vector<cv::Point> > contours;

	//find contours
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	//input rcts into vector ROI
	for (auto contour : contours)
	{
		auto rct = cv::boundingRect(contour);
		if (rct.area() > _min_rct.area())
			ROI.push_back(rct);
	}

	if (!ROI.empty())
	{
		//sort ROI from big to small
		std::sort(ROI.begin(), ROI.end(),
			[](const cv::Rect &a, const cv::Rect &b) -> bool
		{
			return a.area() > b.area();
		});

		switch (mode)
		{
		case Sensor::BIGGEST:
			if (ROI.size() > 1)
			{
				if (ROI[0].area() != ROI[1].area())
					ROI.resize(1);
				else
					ROI.resize(2);
			}
			break;
		case Sensor::BIGGEST2:
			if (ROI.size() > 2)
			{
				if (ROI[2].area() != ROI[1].area())
					ROI.resize(2);
				else
					ROI.resize(3);
			}
			break;
		case Sensor::ALL:
		default:
			break;
		}

		for (auto roi : ROI)
		{
			std::vector<cv::Vec4f> real_pos;
			cv::Point pixel = roi.tl();
			cv::Point3f m_avr_pos = cv::Point3f(0, 0, 0);
			cv::Point3f m_near_pos = cv::Point3f(0, 0, _d_high);

			for (; pixel.x < roi.br().x; ++pixel.x)
			{
				for (; pixel.y < roi.br().y; ++pixel.y)
				{
					if (mask.at<uchar>(pixel) == 0 && cloud.at<cv::Vec4f>(pixel)[2] > _d_low)
					{
						auto point = cloud.at<cv::Vec4f>(pixel);
						real_pos.emplace_back(point);
						m_avr_pos.x += point[0];
						m_avr_pos.y += point[1];
						m_avr_pos.z += point[2];
						if (point[3] < m_near_pos.z)
						{
							m_near_pos = cv::Point3f(point[0], point[1], point[2]);
						}
					}
				}
			}
			int size = real_pos.size();
			if (size > 0)
			{
				obs.push_back(Obstacle(
					cv::Point3f(m_avr_pos.x / size, m_avr_pos.y / size, m_avr_pos.z / size),
					m_near_pos,
					roi));
			}
		}
	}

	return obs;
}

void Sensor::Display(cv::Mat& image, const std::vector<std::string>& string, const std::vector<cv::Rect>& ROIs)
{
	cv::Size resolution = image.size();

	for (auto i = 0; i < string.size(); ++i)
	{
		putText(image, string[i], cv::Point(5, i*30+20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
	}
	
	for (auto rct : ROIs)
	{
		cv::rectangle(image, rct, cv::Scalar(0, 255, 0));
	}
	
	cv::imshow("display", image);
	
	if (_record)
	{
		outputVideo << image;
	}	
}

void Sensor::Display(cv::Mat& image, const std::vector<std::string>& string, const cv::Rect& ROI)
{
	std::vector<cv::Rect> roi;
	roi.emplace_back(ROI);

	Display(image, string, roi);
}

void Sensor::Display(cv::Mat& image, const std::string& string, const cv::Rect& ROI)
{
	std::vector<std::string> str;
	std::vector<cv::Rect> roi;

	str.emplace_back(string);
	roi.emplace_back(ROI);

	Display(image, str, roi);
}

void Sensor::Display(cv::Mat& image)
{
	std::vector<std::string> string;
	std::vector<cv::Rect> rect;
	Display(image, string, rect);
}

void Sensor::Display()
{
	while (true)
	{
		if (!obstacle_objs.empty())
			Display(image_ocv, obstacle_objs[0].ToString(), obstacle_objs[0].GetROI());
		else
			Display(image_ocv);

		auto key = cv::waitKey(10);
		if (key != -1)
			break;
	}
}

void Sensor::Stop()
{
	_grab_error_counts = IMG_MAX_ERROR;

	if(zed_grab_thread.joinable())
		zed_grab_thread.join();
	if(obj_get_thread.joinable())
		obj_get_thread.join();
}

void printHelp() {
	std::cout << "Welcome" << std::endl;
}
