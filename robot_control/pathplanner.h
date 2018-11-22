#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <iostream>
//#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Pathplanner
{
public:
    Pathplanner();
    Pathplanner(Mat);
    void calculatePath();
    Mat getPathImage();
    vector<Point> getEndPoints();
    vector<Point> getPathPoints();
    vector<Point> getPath(Point,Point);
private:
    void pathFinder(Point,int);
    void brushfire();
    void pathLocalMaxima();
    void pathPreClean();
    void pathConnect();
    void pathPostClean();
    void pathEnds();
    bool hasWhite();
    void storePoints();
    Mat image;
    Mat image_brushfire;
    Mat image_path;
    bool isCalculated;
    vector<Point> endPoints;
    vector<Point> pathPoints;
};

#endif // PATHPLANNER_H
