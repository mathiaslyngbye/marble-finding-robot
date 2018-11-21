#ifndef MARBLEDETECT_H
#define MARBLEDETECT_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>
#include "buffer.h"

class marbleDetect
{
public:
    marbleDetect();
    void drawCircle(ConstImageStampedPtr &msg);
    int getMarb();
    float getBlue();
    cv::Mat getCirc();
    bool marbleClose();
private:
    cv::Mat cameraImage;
    cv::Mat CircImage;
    buffer circBuffer;
};

#endif // MARBLEDETECT_H
