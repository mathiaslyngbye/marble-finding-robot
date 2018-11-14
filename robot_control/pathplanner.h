#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Pathplanner
{
public:
    Pathplanner();
    Pathplanner(Mat);
    void calculatePath();
    Mat getPath();
private:
    void brushfire();
    void pathLocalMaxima();
    void pathPreClean();
    void pathConnect();
    void pathPostClean();
    void pathEnds();
    bool hasWhite();
    Mat image;
    Mat image_brushfire;
    Mat image_path;
    bool isCalculated;
};

#endif // PATHPLANNER_H
