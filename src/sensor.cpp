#include "sensor.hpp"
#include "zedtools.hpp"

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

	zed_grab_thread = std::thread([this] { GrabImage(); });

	printHelp();

	return true;
}

void Sensor::GrabImage()
{
	// Set runtime parameters after opening the camera
	RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

	Resolution image_size = zed.getResolution();

	// To share data between sl::Mat and cv::Mat, use slMat2cvMat()
	// Only the headers and pointer to the sl::Mat are copied, not the data itself
	Mat image_zed(image_size, MAT_TYPE_8U_C4);
	cv::Mat image_ocv = slMat2cvMat(image_zed);
	Mat depth_image_zed(image_size, MAT_TYPE_8U_C4);
	cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
	Mat point_cloud(image_size, MAT_TYPE_32F_C4);
	cv::Mat point_cloud_ocv = slMat2cvMat(point_cloud);

	while (true) 
	{
		if (zed.grab(runtime_parameters) == SUCCESS) {

			// Retrieve the left image, depth image in half-resolution
			zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, image_size.width, image_size.height);
			zed.retrieveImage(depth_image_zed, VIEW_DEPTH, MEM_CPU, image_size.width, image_size.height);

			// Retrieve the RGBA point cloud in half-resolution
			// To learn how to manipulate and display point clouds, see Depth Sensing sample
			zed.retrieveMeasure(point_cloud, MEASURE_XYZ, MEM_CPU, image_size.width, image_size.height);

			// Display image and depth using cv:Mat which share sl:Mat data
			cv::imshow("Image", image_ocv);
			cv::imshow("Depth", depth_image_ocv);

			cv::waitKey(1);
		}
	}
}

void printHelp() {
	std::cout << " Press 's' to save Side by side images" << std::endl;
	std::cout << " Press 'p' to save Point Cloud" << std::endl;
	std::cout << " Press 'd' to save Depth image" << std::endl;
	std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
	std::cout << " Press 'n' to switch Depth format" << std::endl;
}


