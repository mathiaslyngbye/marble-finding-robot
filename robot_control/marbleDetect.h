#ifndef MARBLEDETECT_H
#define MARBLEDETECT_H

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

class marbleDetect
{
public:
    marbleDetect();
    void drawCircle(ConstImageStampedPtr &msg);
    int getMarb(cv::Mat &image);
    float getBlue();
private:
    cv::Mat cameraImage;
};

#endif // MARBLEDETECT_H
