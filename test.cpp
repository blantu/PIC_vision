#include "instruction.hpp"
#include "planning.hpp"
#include "multplatform.hpp"

int main()
{
	MoveInstruction instruction;
	Planning *plan = new Planning();
	plan->Start();

	while (true)
	{
		//get instruction per 0.5s
		sleep(500);
		instruction = plan->GetMoveInstr();
		if (instruction.GetMode() == instruction.AUTO)
		{
			std::cout << "AUTO ";
			instruction.GetDirection().PrintXYZ();
		}
		else
			std::cout << "MANUAL" << std::endl;
	}
	
	return 0;
}