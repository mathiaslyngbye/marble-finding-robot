#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "localization.h"
#include <vector>

class controller
{
public:
    controller();
    void movePoint(std::array<double, 2>);
    void moveVector(std::vector<std::array<double, 2>> points);
    int getActive();
    float getDir();
    float getSpeed();
    void setDir(float dir);
    void setPosX(double posX);
    void setPosY(double posY);
    double getDifference(double b1, double b2);
    bool isClose(std::array<double, 2> point);
    void setCollected();
    bool getCollected();
private:
    float distancePath = false;
    float speed;
    float dir;
    float currDir;
    double currX;
    double currY;
    int active = false;
    std::vector<std::array<double, 2>> localPoints;
    bool recentlyCollected = false;
};

#endif // CONTROLLER_H
