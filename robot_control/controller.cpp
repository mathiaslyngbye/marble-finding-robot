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

void controller::movePoint(double x, double y)
{
    float thetaHat = std::atan2((y-currY),(x-currX));

    float goalDir = (roundf(thetaHat * 10) / 10);

    double dirChange = getDifference(currDir + 3.14, goalDir);
    speed = 0.40;
    if (dirChange > 0)
    {
        dir = 0.4;
    }
    else if (dirChange < 0)
    {
        dir = -0.4;
    }
    std::cout << "goal dir: " << goalDir << " current dir: " << currDir << std::endl;
    std::cout << "x: " << currX << ", " << x << " y: " << currY << ", " << y << std::endl;
}

double controller::getDifference(double b1, double b2)
{
    double r = fmod(b2 - b1, 6.28);
    if (r < -3.14)
        r += 6.28;
    if (r >= 3.14)
        r -= 6.28;
    return r;
}
