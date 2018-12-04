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

void controller::setCollected()
{
    recentlyCollected = true;
}

bool controller::getCollected()
{
    return recentlyCollected;
}

void controller::movePoint(std::array<double, 2> point)
{
    double x = point[0];
    double y = point[1];
    float thetaHat = std::atan2((y-currY),(x-currX));

    float goalDir = (roundf(thetaHat * 10) / 10);

    double dirChange = getDifference(currDir, goalDir);

    /*
    if (dirChange < 0.7)
    {
        dir = 0.4;
    }
    else if (dirChange > 0.7)
    {
        dir = -0.4;
    }
    */

    if ((dirChange<0.2) && (dirChange>-0.2))
    {
        if(dirChange < 0)
        {
            dir = 0.1;
        }
        else
        {
            dir = -0.1;
        }
    }
    else
    {
        dir = -dirChange * 0.50;
    }
    if (dir > 3.14)
    {
        dir = 0.4;
    }
    else if (dir < -3.14)
    {
        dir = -0.4;
    }

    /*
    std::cout << "Current d: " << currDir << "\t Goal d: " << goalDir << std::endl;
    std::cout << "Current x: " << currX << "\tGoal x: " << x << std::endl;
    std::cout << "Current y: " << currY << "\tGoal y: " << y << std::endl;
    std::cout << std::endl;
    */
}

void controller::moveVector(std::vector<std::array<double, 2>> points)
{
    if (active == false)
    {
        localPoints = points;
        localPoints.erase(localPoints.begin());
        localPoints.erase(localPoints.begin());
    }

    if (!localPoints.empty())
    {
        active = true;
        movePoint(localPoints[0]);
        if (isClose(localPoints[0]))
        {
            localPoints.erase(localPoints.begin());
            recentlyCollected = false;
        }
    }
    if (localPoints.empty())
    {
        active = false;
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

bool controller::isClose(std::array<double, 2> point)
{
    distancePath = std::sqrt(pow((point[0] - currX),2) + pow((point[1] - currY),2));
    //std::cout << "Distance to next point is:  " << distancePath << std::endl;
    if (distancePath < 0.8)
    {
        return true;
    }
    return false;
}
