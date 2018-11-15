#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <opencv2/opencv.hpp>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

class Localization
{
public:
    Localization();
    void poseCallBack(ConstPosesStampedPtr &msg);
    double getLocation();
    float getDir();
private:
    float dir;
    //cv::Point2f posf; //Position in meters from centre. Taken directly from poseCallback
    double posArr[2];
};

#endif // LOCALIZATION_H
