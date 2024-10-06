#ifndef CONSTANTS_H
#define CONSTANTS_H

#define STEPS 100

#define topOpenPin 27;
#define topClosePin 45;
#define bottomOpenPin 44;
#define bottomClosePin 26;

#define topOpen false;
#define topClose false;
#define bottomOpen false;
#define bottomClose false;

enum RobotState
{
    DISABLED,
    IDLE,
    GLIDING,
    SURFACING
};

enum Direction
{
    FORWARD,
    BACKWARD
};

#endif // CONSTANTS_H