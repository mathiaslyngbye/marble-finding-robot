#include "controller.h"

#include "math.h"

controller::controller()
{

}

float controller::getDir()
{
    return dir;
}

float controller::getSpeed()
{
    return speed;
}

void controller::setDir(float dirr)
{
    currDir = roundf(dirr * 10) / 10;
}

void controller::setPosX(double curreX)
{
    currX = roundf(curreX * 10) / 10;
}

void controller::setPosY(double curreY)
{
    currY = roundf(curreY * 10) / 10;
}

void controller::movePoint(int x, int y)
{
    int mode = 0;
    float thetaHat = std::atan2((y-currY),(x-currX));

    float goalDir = roundf(thetaHat * 10) / 10;

    if (currDir != goalDir)
    {
        mode = 1;
        speed = 0.15;
        if (currDir > goalDir+0.3)
        {
            dir = 0.4;
        }
        else if (currDir < goalDir-0.3)
        {
            dir = -0.4;
        }
        else if (currDir > goalDir)
        {
            dir = 0.1;
        }
        else if (currDir < goalDir)
        {
            dir = -0.1;
        }
    }

    if (mode == 0)
    {
        if ((currX != x && currY != y) || currX != x || currY != y)
        {
            dir = 0;
            speed = 0.5;
        }
    }
}
