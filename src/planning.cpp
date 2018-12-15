#include "planning.hpp"
#include "multplatform.hpp"

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
