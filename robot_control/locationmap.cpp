#include "locationmap.h"

#include <math.h>


LocationMap::LocationMap()
{

}

LocationMap::LocationMap(cv::Mat _image)
{
    image = _image;
}

void LocationMap::setImage(cv::Mat _image)
{
    image = _image;
}
cv::Mat LocationMap::myLocation(double x,double y, double theta)
{
    // Get scale
    double scale = image.cols / 86.0;

    // Clone image.
    cv::Mat image_location = image.clone();

    // Calculate offset coordinates.
    double local_x = scale*x+(0.5*image.cols);
    double local_y = scale*(-y)+(0.5*image.rows);

    // Determine arrow scale.
    double arrow_length = 18;

    // Calculate arrow tail coordinates.
    double arrow_x1 = local_x-(arrow_length/2)*cos(theta);
    double arrow_y1 = local_y+(arrow_length/2)*sin(theta);

    // Calculate arrow head coordinates.
    double arrow_x2 = local_x+(arrow_length/2)*cos(theta);
    double arrow_y2 = local_y-(arrow_length/2)*sin(theta);

    // Draw arrow
    cv::arrowedLine(    image_location,                 // Image
                        cv::Point(arrow_x1,arrow_y1),   // Point1
                        cv::Point(arrow_x2,arrow_y2),   // Point2
                        {0,0,255},                      // Color
                        1,                              // Thickness
                        8,                              // LineType
                        0,                              // Shift
                        0.3                             // TipLength
    );

    // Return image with arrow.
    return image_location;
}

std::array<double,2> LocationMap::getCoordsXY(cv::Point _point)
{
    // Define source width and height.
    int height = 80;
    int width = 120;

    // Determine scale.
    double scale = width / 86.0;

    // Calculate offset coordinates.
    double local_x = ((_point.x)-(0.5*width))/scale;
    double local_y = ((0.5*height)-(_point.y))/scale;

    // Create Array of calculated coordiantes.
    std::array<double,2> myCoords = {local_x, local_y};

    // Return array of coordinates.
    return myCoords;
}

cv::Point LocationMap::getCoordsPoint(double x,double y)
{
    // Define source width and height.
    int height = 80;
    int width = 120;

    // Determine scale.
    double scale = width / 86.0;

    // Calculate offset coordinates.
    double local_x = scale*x+(0.5*width);
    double local_y = scale*(-y)+(0.5*height);

    // Rerturn calculated point.
    return cv::Point(local_x, local_y);
}
