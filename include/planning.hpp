#ifndef _PLANNING
#define _PLANNING
#include "instruction.hpp"
#include "zedtools.hpp"

class Planning
{
public:
	//todo:thread safety

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
