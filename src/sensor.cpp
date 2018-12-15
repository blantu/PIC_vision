#include "sensor.hpp"
#include "zedtools.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/cuda.hpp"
#include "opencv2/imgproc.hpp"

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

bool Sensor::Start()
{
	zed_error = zed.open(zed_init_params);
	
	if (zed_error != SUCCESS) {
		printf("%s\n", toString(zed_error).c_str());
		zed.close();
		return false; // Quit if an error occurred
	}

	zed_image_size.height = zed.getResolution().height / 2;
	zed_image_size.width = zed.getResolution().width / 2;

	zed_grab_thread = std::thread([this] { GrabImage(); });

	printHelp();

	return true;
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
	cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
	point_cloud.alloc(zed_image_size, MAT_TYPE_32F_C4);
	cv::Mat point_cloud_ocv = slMat2cvMat(point_cloud);

	while (true) 
	{
		if (zed.grab(runtime_parameters) == SUCCESS) {

			// Retrieve the left image, depth image in half-resolution
			zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, zed_image_size.width, zed_image_size.height);
			zed.retrieveMeasure(depth_image_zed, MEASURE_DEPTH, MEM_CPU, zed_image_size.width, zed_image_size.height);

			// Retrieve the RGBA point cloud in half-resolution
			// To learn how to manipulate and display point clouds, see Depth Sensing sample
			zed.retrieveMeasure(point_cloud, MEASURE_XYZ, MEM_CPU, zed_image_size.width, zed_image_size.height);

			// Display image and depth using cv:Mat which share sl:Mat data
			//cv::imshow("Image", image_ocv);
			//cv::imshow("Depth", depth_image_ocv);
			//std::cout << depth_image_ocv.at<float>(360, 360) << std::endl;

			//cv::waitKey(1);
			ImageAnalyse(image_ocv, depth_image_ocv, point_cloud);
		}
	}
}

std::vector<cv::Rect> Sensor::ImageAnalyse(cv::Mat& color, cv::Mat& depth, sl::Mat& cloud)
{
	std::vector<cv::Rect> ROI;
	std::vector<std::vector<cv::Point> > contours;
	cv::Mat img_hsv, img_color_bw, img_depth_bw, img_dst_bw;

	cvtColor(color, img_hsv, CV_BGR2HSV);
	cv::inRange(img_hsv, cv::Scalar(_h_low, 43, 46), cv::Scalar(_h_high, 255, 255), img_color_bw); //Threshold the image
	cv::inRange(depth, cv::Scalar(_d_low), cv::Scalar(_d_high), img_depth_bw); //Threshold the image
	cv::bitwise_and(img_color_bw, img_depth_bw, img_dst_bw);

	//cv::fastNlMeansDenoising(img_dst_bw, img_dst_bw, 3, 7, 21);
	cv::findContours(img_dst_bw, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	for (auto contour : contours)
	{
		ROI.push_back(cv::boundingRect(contour));
		cv::rectangle(color, ROI.back(), cv::Scalar(0,255,0));
		cv::imshow("dst", color);
		cv::waitKey(1);
	}
		
	


	return ROI;
}

void printHelp() {
	std::cout << " Press 's' to save Side by side images" << std::endl;
	std::cout << " Press 'p' to save Point Cloud" << std::endl;
	std::cout << " Press 'd' to save Depth image" << std::endl;
	std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
	std::cout << " Press 'n' to switch Depth format" << std::endl;
}


