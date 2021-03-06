#include "localization.h"

#include <iostream>

Localization::Localization()
{

}

void Localization::poseCallBack(ConstPosesStampedPtr &msg)
{
    int pose_size = msg->pose_size();

    for (int i = 0; i < pose_size; i++)
    {
        if (msg->pose(i).name() == "pioneer2dx")
        {
            posArr[0] = msg->pose(i).position().x();
            posArr[1] = msg->pose(i).position().y();

            double w = msg->pose(i).orientation().w();
            double x = msg->pose(i).orientation().x();
            double y = msg->pose(i).orientation().y();
            double z = msg->pose(i).orientation().z();

            dir = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        }
    }
}

float Localization::getDir()
{
    return dir;
}

double Localization::getLocationX()
{
    posArr[0] = roundf(posArr[0] * 10) / 10;
    return posArr[0];
}

double Localization::getLocationY()
{
    posArr[1] = roundf(posArr[1] * 10) / 10;
    return posArr[1];
}
