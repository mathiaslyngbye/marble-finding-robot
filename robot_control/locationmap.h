#ifndef LOCATIONMAP_H
#define LOCATIONMAP_H

#include <iostream>
#include <opencv2/opencv.hpp>

class LocationMap
{
public:
    LocationMap();
    LocationMap(cv::Mat);
    cv::Mat myLocation(double, double, double);
private:
    cv::Mat image;
};

#endif // LOCATIONMAP_H
