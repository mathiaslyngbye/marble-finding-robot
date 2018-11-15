#include "marbleDetect.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <math.h>

marbleDetect::marbleDetect()
{

}

cv::Mat marbleDetect::getCirc()
{
    return CircImage;
}

int marbleDetect::getMarb()
{
    if (getBlue() != 0)
    {
        cv::Mat clone = cameraImage.clone();
        cv::Mat grayMat;
        cv::cvtColor(clone, grayMat, CV_BGR2GRAY);
        cv::GaussianBlur(grayMat, grayMat, cv::Size(9,9), 2, 2);
        std::vector<cv::Vec3f> circles;
        circles.clear();
        cv::HoughCircles(grayMat, circles, cv::HOUGH_GRADIENT, 1, grayMat.rows/8, 50, 25, 0, 0);

        float dist = 0.f;
        int rad = 0;
        int marbNum = 0;
        if (circles.size() > 0)
        {
            for (u_int i = 0; i < circles.size(); i++)
            {
                if (circles[i][2] > rad)
                {
                    rad = circles[i][2];
                    marbNum = i;
                }
            }
        }

        //find dist
        dist = circles[marbNum][0]; //- clone.cols/2;
        std::cout << "dist is " << dist << std::endl;
        return dist;
    }
    else
        return 0.f;
}


void marbleDetect::drawCircle(ConstImageStampedPtr &msg)
{
    std::size_t width = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat ima(int(height), int(width), CV_8UC3, const_cast<char *>(data));
    cv::cvtColor(ima, ima, CV_RGB2BGR);
    cameraImage = ima.clone();

    cv::Mat im = cameraImage.clone();

    /*
    cv::Mat hlsChannels[3];
    cv::split(im, hlsChannels);
    */
    cv::Mat grayMat;
    cv::cvtColor(im, grayMat, CV_BGR2GRAY);
    cv::GaussianBlur(grayMat, grayMat, cv::Size(9,9), 2, 2);

    //cv::threshold(hlsChannels[2], grayMat, 125, 255, cv::THRESH_BINARY);
    //cv::GaussianBlur(grayMat, grayMat, cv::Size(9, 9), 2, 2);

    std::vector<cv::Vec3f> circles;
    circles.clear();
    cv::HoughCircles(grayMat, circles, cv::HOUGH_GRADIENT, 1, grayMat.rows/8, 50, 25, 0, 0);
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        cv::circle( im, center, 1, cv::Scalar(0,255,255), 1, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        cv::circle( im, center, radius, cv::Scalar(0,0,255), 1, cv::LINE_AA);
    }

    int center_x = im.cols/2;
    int center_y = im.rows/2;
    cv:: circle(im, cv::Point(center_x, center_y), 2, cv::Scalar(150), -1, 8, 0);
    CircImage = im.clone();
    //cv::imshow("detected circles", im);
}

float marbleDetect::getBlue()
{
    cv::Mat im = cameraImage.clone();
    int blueLeft = 0;
    int blueRight = 0;
    for (int rows = im.rows*0.4; rows < im.rows*0.6; rows++)
    {
        for (int cols = 0; cols < im.cols; cols++)
        {
            int threshold = std::max(static_cast<int>(im.at<cv::Vec3b>(cv::Point(cols,rows))[1]), static_cast<int>(im.at<cv::Vec3b>(cv::Point(cols,rows))[2]))*1.8;
            if(static_cast<int>(im.at<cv::Vec3b>(cv::Point(cols,rows))[0])>threshold)
            {
                if(cols<im.cols*0.5)
                {
                    blueLeft++;
                }
                else
                {
                    blueRight++;
                }
                //image.at<Vec3b>(Point(cols,rows)) = color_blue;
            }
        }
    }

    int pixel_threshold = 500;
    float pixel_difference =  blueRight - blueLeft;

    if(pixel_difference < -pixel_threshold)
    {
        pixel_difference = -pixel_threshold;
    }
    else if(pixel_difference > pixel_threshold)
    {
        pixel_difference = pixel_threshold;
    }

    return pixel_difference / 500;
}
