#include "instruction.hpp"
#include "planning.hpp"
#include "sensor.hpp"
#include "multplatform.hpp"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv)
{
	Sensor sensor(argc, argv);
	if (sensor.Start())
		while (true);
		
	return 0;
}