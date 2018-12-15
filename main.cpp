#include "instruction.hpp"
#include "planning.hpp"
#include "sensor.hpp"
#include "multplatform.hpp"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv)
{
	Sensor sensor(argc, argv);
	if (sensor.Start())
	{
		sensor.SetColorRange(100, 124);
		sensor.SetDistanceRange(0.5, 10);
		while (true);
	}
		
		
	return 0;
}