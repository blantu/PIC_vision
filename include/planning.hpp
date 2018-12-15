#ifndef _PLANNING
#define _PLANNING
#include "instruction.hpp"

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
