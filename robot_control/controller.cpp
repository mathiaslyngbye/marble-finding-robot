#include "controller.h"

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
    currDir = dirr;
}

void controller::setSpeed(float speedy)
{
    currSpeed = speedy;
}

void controller::movePoint(int x, int y)
{
    float goalDir = 0.f;     //Calculate dir
    while (currDir != goalDir)
    {
        speed = 0.1;
        if (currDir > goalDir)
        {
            dir = 0.4;
        }
        else
        {
            dir = -0.4;
        }
    }
    /*
    while((locator.getLocationX() != x) && (locator.getLocationY() != y))
    {
        dir = 0;
        speed = 1;
    }
    */
}
