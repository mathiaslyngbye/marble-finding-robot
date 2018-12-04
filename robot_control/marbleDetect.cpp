#include "marbleDetect.h"

#include <vector>
#include <iostream>
#include <math.h>
#include "buffer.h"

marbleDetect::marbleDetect()
{

}

cv::Mat marbleDetect::getCirc()
{
    return CircImage;
}

cv::Mat marbleDetect::getDeafult()
{
    return cameraImage;
}

bool marbleDetect::marbleClose()
{
    cv::Mat im = cameraImage.clone();
    int blue = 0;
    for (int rows = im.rows*0.4; rows < im.rows*0.6; rows++)
    {
        for (int cols = 30; cols < im.cols-30; cols++)
        {
            int threshold = std::max(static_cast<int>(im.at<cv::Vec3b>(cv::Point(cols,rows))[1]), static_cast<int>(im.at<cv::Vec3b>(cv::Point(cols,rows))[2]))*1.8;
            if(static_cast<int>(im.at<cv::Vec3b>(cv::Point(cols,rows))[0])>threshold)
            {
                blue += 1;
            }
        }
    }
    if (blue > 8000)
    {
        return true;
    }
    else
    {
        return false;
    }
}

int marbleDetect::getMarb()
{
    if (getBlue() != 0)
    {
        cv::Mat clone = cameraImage.clone();
        cv::Mat grayMat;
        cv::cvtColor(clone, grayMat, CV_BGR2GRAY);
        cv::GaussianBlur(grayMat, grayMat, cv::Size(9,9), 2, 2);

        int center_x = clone.cols/2;
        int center_y = clone.rows/2;

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(grayMat, circles, cv::HOUGH_GRADIENT, 1, clone.rows/8, 50, 25, 0, 0);

        circles.size() ? circBuffer.addToBuf(true) : circBuffer.addToBuf(false);

        float length = 0;
        if (!circBuffer.isEmpty() && circles.size())
        {
            cv::Vec3f marb = {0, 0, 0};
            for (int i = 0; i < circles.size(); i++)
            {
                marb = (circles[i][2] > marb[2]) ? circles[i] : marb;
            }
            length = (marb[0]-center_x > 0) ? std::sqrt(std::pow(marb[0]-center_x, 2) + std::pow(marb[1]-center_y, 2)) : -1*std::sqrt(std::pow(marb[0]-center_x, 2) + std::pow(marb[1]-center_y, 2));
        }
        return length;
    }
    else
    {
        return 0.f;
    }
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

    cv::Mat grayMat;
    cv::cvtColor(im, grayMat, CV_BGR2GRAY);
    cv::GaussianBlur(grayMat, grayMat, cv::Size(9,9), 2, 2);

    std::vector<cv::Vec3f> circles;
    circles.clear();
    cv::HoughCircles(grayMat, circles, cv::HOUGH_GRADIENT, 1, grayMat.rows/8, 50, 25, 0, 0);
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        cv::circle( im, center, 1, cv::Scalar(0,255,255), 1, cv::LINE_AA);
        int radius = c[2];
        cv::circle( im, center, radius, cv::Scalar(0,0,255), 1, cv::LINE_AA);
    }

    int center_x = im.cols/2;
    int center_y = im.rows/2;
    cv:: circle(im, cv::Point(center_x, center_y), 2, cv::Scalar(150), -1, 8, 0);
    CircImage = im.clone();
}

float marbleDetect::getBlue()
{
    lastBlue = currBlue;
    cv::Mat im = cameraImage.clone();
    int blueLeft = 0;
    int blueRight = 0;
    for (int rows = im.rows*0.4; rows < im.rows*0.6; rows++)
    {
        for (int cols = 30; cols < im.cols-30; cols++)
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
            }
        }
    }
    float pixel_sum = blueRight + blueLeft;
    currBlue = marbleClose();
    return pixel_sum;
}

bool marbleDetect::collected()
{
    if ((lastBlue == true) && (currBlue == false))
    {
        return true;
    }
    return false;
}
