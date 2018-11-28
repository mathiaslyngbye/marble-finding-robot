#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "localization.h"
#include <vector>

class controller
{
public:
    controller();
    void movePoint(double x, double y);
    void moveVector(std::vector<cv::Point> points);
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
};

#endif // CONTROLLER_H
