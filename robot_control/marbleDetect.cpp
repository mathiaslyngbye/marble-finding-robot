#include "marbleDetect.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <math.h>

marbleDetect::marbleDetect()
{

}

void marbleDetect::drawCircle(ConstImageStampedPtr &msg)
{
    std::size_t width = msg->image().width();
    std::size_t height = msg->image().height();
    const char *data = msg->image().data().c_str();
    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));
    cv::cvtColor(im, im, CV_RGB2BGR);
    cameraImage = im.clone();

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

    cv::imshow("detected circles", im);
}

float marbleDetect::getBlue()
{
    int blueLeft = 0;
    int blueRight = 0;
    for (int rows = cameraImage.rows*0.4; rows < cameraImage.rows*0.6; rows++)
    {
        for (int cols = 0; cols < cameraImage.cols; cols++)
        {
            int threshold = std::max(static_cast<int>(cameraImage.at<cv::Vec3b>(cv::Point(cols,rows))[1]), static_cast<int>(cameraImage.at<cv::Vec3b>(cv::Point(cols,rows))[2]))*1.8;
            if(static_cast<int>(cameraImage.at<cv::Vec3b>(cv::Point(cols,rows))[0])>threshold)
            {
                if(cols<cameraImage.cols*0.5)
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
