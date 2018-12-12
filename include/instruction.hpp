#ifndef _INSTRUCTION
#define _INSTRUCTION

#include <utility>
#include "multplatform.hpp"

class Direction
{
private:
    void Reset(){
        _x = _y = _z = 0.0f;
        _angle = _distance = 0.0f;
    };
public:
    Direction(float x=0.0f, float y=0.0f, float z=0.0f):
        _x(x),_y(y),_z(z){};

    Direction(float angle, float distance):
        _angle(angle), _distance(distance){};

    Direction ChangAxise();
	void PrintXYZ() { std::cout << "direction:\nx£º" << _x << std::endl << "y£º" << _y << std::endl << "z£º" << _z << std::endl; };

    float _x,_y,_z;
    float _angle,_distance;
};


class MoveInstruction
{
public:
    enum MODE{
        MANUAL=0,
        AUTO,
    };

    MoveInstruction(MODE mode = MANUAL):_mode(mode),_move_direction(*std::move(new Direction())){};

	void MoveTO(Direction d = *std::move(new Direction())) { _move_direction = d; };
	bool ChangeMode(MODE mode = AUTO) {
		_mode = mode; 
		return true;
	}

	Direction GetDirection() { return _move_direction; };
	MODE GetMode() { return _mode; };

private:
	Direction _move_direction;
    MODE _mode;	
};


#endif
