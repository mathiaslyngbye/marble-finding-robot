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
    int getMarb();
    float getBlue();
    cv::Mat getCirc();
private:
    cv::Mat cameraImage;
    cv::Mat CircImage;
};

#endif // MARBLEDETECT_H
