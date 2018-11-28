#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <iostream>
//#include <vector>
#include <opencv2/opencv.hpp>

class Pathplanner
{
public:
    Pathplanner();
    Pathplanner(cv::Mat);
    void calculatePath();
    cv::Mat getPathImage();
    std::vector<cv::Point> getEndPoints();
    std::vector<cv::Point> getPathPoints();
    std::vector<cv::Point> getPath(cv::Point,cv::Point);
private:
    void pathFinder(cv::Point,int);
    void brushfire();
    void pathLocalMaxima();
    void pathPreClean();
    void pathConnect();
    void pathPostClean();
    void pathEnds();
    bool hasWhite();
    void storePoints();
    cv::Mat image;
    cv::Mat image_brushfire;
    cv::Mat image_path;
    bool isCalculated;
    std::vector<cv::Point> endPoints;
    std::vector<cv::Point> pathPoints;
};

#endif // PATHPLANNER_H
