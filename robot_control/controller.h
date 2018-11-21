#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "localization.h"

class controller
{
public:
    controller();
    void movePoint(int x, int y);
    float getDir();
    float getSpeed();
    void setDir(float dir);
    void setSpeed(float speed);
private:
    float speed;
    float dir;
    float currDir;
    float currSpeed;
};

#endif // CONTROLLER_H
