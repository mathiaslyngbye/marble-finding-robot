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

int controller::getActive()
{
    return active;
}

void controller::setDir(float dirr)
{
    currDir = roundf(dirr * 10) / 10;
}

void controller::setPosX(double curreX)
{
    currX = curreX;
}

void controller::setPosY(double curreY)
{
    currY = curreY;
}

void controller::movePoint(std::array<double, 2> point)
{
    double x = point[0];
    double y = point[1];
    float thetaHat = std::atan2((y-currY),(x-currX));

    float goalDir = (roundf(thetaHat * 10) / 10);

    double dirChange = getDifference(currDir + 3.14, goalDir);
    //speed = 0.20;
    if (dirChange > 0)
    {
        dir = 0.4;
    }
    else if (dirChange < 0)
    {
        dir = -0.4;
    }
    std::cout << "Current d: " << currDir << "\t Goal d: " << goalDir << std::endl;
    std::cout << "Current x: " << currX << "\tGoal x: " << x << std::endl;
    std::cout << "Current y: " << currY << "\tGoal y: " << y << std::endl;
    std::cout << std::endl;
}

void controller::moveVector(std::vector<std::array<double, 2>> points)
{
    std::vector<std::array<double, 2>> localPoints;
    localPoints = points;
    if (!localPoints.empty())
    {
        movePoint(localPoints[0]);
        if (currX == localPoints[0][0] && currY == localPoints[0][1])
        {
            localPoints.erase(localPoints.begin());
            localPoints.erase(localPoints.begin());
        }
    }
    if (localPoints.empty())
    {
        active = 0;
    }
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
