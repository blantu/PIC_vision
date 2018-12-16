#include "instruction.hpp"
#include "planning.hpp"
#include "sensor.hpp"
#include "includes.hpp"

int main(int argc, char** argv)
{
	Sensor sensor(argc, argv);
	sensor.SetColorRange(100, 124);
	//sensor.SetColorRange();
	sensor.SetDistanceRange(0.5, 10);
	sensor.SetMinROI(10, 10);
	sensor.SetRecord(true);

	if (sensor.Start(sensor.HALF))
	{
		sensor.Display();
	}

	sensor.Stop();
	return 0;
}