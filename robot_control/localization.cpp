#include "localization.h"

#include <iostream>

Localization::Localization()
{

}

void Localization::poseCallBack(ConstPosesStampedPtr &msg)
{
    std::cout << "Works" << std::endl;

    int pose_size = msg->pose_size();

    for (int i = 0; i < pose_size; i++)
    {
        if (msg->pose(i).name() == "pioneer2dx")
        {
            posf.x = msg->pose(i).position().x();
            posf.y = msg->pose(i).position().y();

            double w = msg->pose(i).orientation().w();
            double x = msg->pose(i).orientation().x();
            double y = msg->pose(i).orientation().y();
            double z = msg->pose(i).orientation().z();

            dir = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

            std::cout << dir << std::endl;
        }
    }
}
