#ifndef _INSTRUCTION
#define _INSTRUCTION

#include <utility>

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

    Direction GetDirection();
    MODE GetMode();

private:
	Direction _move_direction;
    MODE _mode;	
};


#endif
