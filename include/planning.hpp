#ifndef _PLANNING
#define _PLANNING
#include "instruction.hpp"
#include "includes.hpp"

class Obstacle
{
public:
	Obstacle(cv::Point3f &avr, cv::Point3f &min, cv::Rect &region) :_avr_pos(avr),_min_pos(min),_region(region) {};
	cv::Point3f GetAverage() { return _avr_pos; }
	cv::Point3f GetMin() { return _min_pos; }
	cv::Rect GetROI() { return _region; }

	std::vector<std::string> ToString();
	std::string ToString(cv::Point3f &);
	std::string ToString(cv::Rect &);
	Obstacle operator =(const Obstacle& ob)
	{
		_avr_pos = ob._avr_pos;
		_min_pos = ob._min_pos;
		_region = ob._region;
		return ob;
	}

	bool operator ==(const Obstacle& ob)
	{
		return _avr_pos == ob._avr_pos && _min_pos == ob._min_pos && _region == ob._region;
	}

private:
	cv::Point3f _avr_pos;
	cv::Point3f _min_pos;
	cv::Rect _region;
};

//todo:thread safety
class Planning
{
public:
	void Start() {
		planning_thread = std::thread([this] { Plan(); });
	};
	void Stop() {
		planning_thread.detach();
	};

	MoveInstruction GetMoveInstr() { return _instruction; };

private:
	void Plan();

	MoveInstruction _instruction;
	std::thread planning_thread;
};

#endif
