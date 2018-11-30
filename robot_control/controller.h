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
private:
    float speed;
    float dir;
    float currDir;
    double currX;
    double currY;
    int active = 0;
    std::vector<std::array<double, 2>> localPoints;
};

#endif // CONTROLLER_H
