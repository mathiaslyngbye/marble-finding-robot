#ifndef LOCATIONMAP_H
#define LOCATIONMAP_H

#include <iostream>
#include <opencv2/opencv.hpp>

class LocationMap
{
public:
    LocationMap();
    LocationMap(cv::Mat);
    void setImage(cv::Mat);
    cv::Mat myLocation(double, double, double);
    std::array<double,2> getCoordsXY(cv::Point);
    cv::Point getCoordsPoint(double, double);
private:
    cv::Mat image;
    double scale;
};

#endif // LOCATIONMAP_H
