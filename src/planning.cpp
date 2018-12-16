#include "planning.hpp"
#include "includes.hpp"

std::vector<std::string> Obstacle::ToString()
{
	std::vector<std::string> string;
	string.push_back(ToString(this->_min_pos));
	string.push_back(ToString(this->_avr_pos));
	string.push_back(ToString(this->_region));
	return string;
}

std::string Obstacle::ToString(cv::Point3f &point)
{
	std::stringstream stream;
	std::string pos_string = "x:";

	stream << std::fixed << std::setprecision(2) << point.x << "m y:" << point.y << "m z:" << point.z << "m";
	pos_string += stream.str();

	return pos_string;
}

std::string Obstacle::ToString(cv::Rect &rct)
{
	std::stringstream stream;
	std::string pos_string;

	stream << rct.x << " y:" << rct.y << " width:" << rct.width << " height:" << rct.height;
	pos_string += stream.str();

	return pos_string;
}


void Planning::Plan()
{
	Direction target;
	while (true)
	{
		sleep(100);
		int seconds = time((time_t*)NULL);

		target._x = -sqrt(seconds%100)/5;
		_instruction.MoveTO(target);

		if (seconds % 7 == 0)
		{
			_instruction.ChangeMode();
		}
		else if (seconds % 11 == 0)
		{
			_instruction.ChangeMode(MoveInstruction::MANUAL);
		}
	}
}
