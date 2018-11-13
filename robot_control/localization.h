#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <gazebo/msgs/msgs.hh>
#include <opencv2/opencv.hpp>

class Localization
{
public:
    Localization();
    void poseCallBack(ConstPosesStampedPtr &msg);
private:
    float dir;
    cv::Point2f posf; //Position in meters from centre. Taken directly from poseCallback
};

#endif // LOCALIZATION_H
