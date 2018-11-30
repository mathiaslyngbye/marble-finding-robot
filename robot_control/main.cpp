#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "fl/Headers.h"

#include <opencv2/opencv.hpp>

#include "pathplanner.h"
#include "controller.h"
#include "localization.h"
#include "FuzzyDefault.h"
#include "marbleDetect.h"
#include "locationmap.h"
#include <iostream>

// Common variables
float lidar_ranges[200];
float lidar_angles[200];
float w_lidar_ranges[200];
float w_lidar_angles[200];

static boost::mutex mutex;

// Gazebo Stat Callback Function
void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
    //std::cout << _msg->DebugString();
    //std::cout << std::flush;
}

// Gazebo Lidar Callback Function
void lidarCallback(ConstLaserScanStampedPtr &msg) {

    //std::cout << ">> " << msg->DebugString() << std::endl;
    float angle_min = float(msg->scan().angle_min());
    //double angle_max = msg->scan().angle_max();
    float angle_increment = float(msg->scan().angle_step());

    float range_min = float(msg->scan().range_min());
    float range_max = float(msg->scan().range_max());

    int sec = msg->time().sec();
    int nsec = msg->time().nsec();

    int nranges = msg->scan().ranges_size();
    int nintensities = msg->scan().intensities_size();

    assert(nranges == nintensities);

    int width = 400;
    int height = 400;
    float px_per_m = 200 / range_max;

    float lidar_ranges_tmp[200];
    float lidar_angles_tmp[200];
    //float w_lidar_ranges_tmp[200];

    cv::Mat im(height, width, CV_8UC3);
    im.setTo(0);
    for (int i = 0; i < nranges; i++)
    {
        float angle = angle_min + i * angle_increment;
        float range = std::min(float(msg->scan().ranges(i)), range_max);
            //double intensity = msg->scan().intensities(i);
        cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                            200.5f - range_min * px_per_m * std::sin(angle));
        cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                          200.5f - range * px_per_m * std::sin(angle));
        cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
                 cv::LINE_AA, 4);

        //std::cout << angle << " " << range << " " << intensity << std::endl;

        //std::cout << "Sensor: " << i << "\tAngle: " << angle << "\tRange: " << range << std::endl;

        lidar_ranges_tmp[i] = range;
        lidar_angles_tmp[i] = angle;
        //w_lidar_ranges_tmp[i] = (0.5+abs(i-100)*0.015)*range; //*range;

    }

    mutex.lock();
    for(int i = 0; i < 200; i++)
    {
        lidar_ranges[i] = lidar_ranges_tmp[i];
        lidar_angles[i] = lidar_angles_tmp[i];
    }
    mutex.unlock();

    cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
    cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

    mutex.lock();
    cv::imshow("lidar", im);
    mutex.unlock();
}

int main(int _argc, char **_argv)
{
    //Declare classes to read call backs from Gazebo
    marbleDetect marbDetect;
    Localization locator;

    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr statSubscriber =
      node->Subscribe("~/world_stats", statCallback);

    gazebo::transport::SubscriberPtr poseSubscriber =
      node->Subscribe("~/pose/info", &Localization::poseCallBack, &locator);

    gazebo::transport::SubscriberPtr circleDetection =
          node->Subscribe("~/pioneer2dx/camera/link/camera/image", &marbleDetect::drawCircle, &marbDetect);

    gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

    // Publish to the robot vel_cmd topic
    gazebo::transport::PublisherPtr movementPublisher =
      node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    // Publish a reset of the world
    gazebo::transport::PublisherPtr worldPublisher =
      node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);

    //=============================================

    enum driveMode
    {
        fuzzyObstacle,
        fuzzyMarble,
        pointsDriver,
        endPointsDriver,
        pathLocator,
        pathCalc
    };

    //Declare floats for controlling robot
    float speed = 0.0;
    float dir = 0.0;

    // Import image: floor plan of big worlds
    std::string floor_plan_JPG("../models/bigworld/meshes/floor_plan.png");
    cv::Mat floor_plan;
    floor_plan = cv::imread(floor_plan_JPG, cv::IMREAD_COLOR);

    // Create window and image for testing purposes
    cv::namedWindow("Test Space", cv::WINDOW_AUTOSIZE); // Create a window.
    cv::Mat testImage = floor_plan.clone();             // Generate test image of floor_plan.png

    // Initiate location map
    cv::Mat floor_plan_location = floor_plan.clone();                         // Duplicate floor_plan for location map.
    cv::resize(floor_plan_location, floor_plan_location, cv::Size(), 3, 3);   // Resize picture.
    LocationMap myMap(floor_plan_location);                                   // Create map based on resized
    cv::Mat myLocationMap = myMap.myLocation(0,0,0);                          // Initiate map image with position 0,0,0.
    cv::namedWindow("Location Map", cv::WINDOW_AUTOSIZE);                     // Create a window for image.

    // Initiate pathplanner
    Pathplanner pathplan(floor_plan);
    pathplan.calculatePath();
    std::vector<cv::Point> pathPoints = pathplan.getPathPoints();
    std::vector<cv::Point> endPoints = pathplan.getEndPoints();

    // Initiate controllers
    controller control;
    obstacle_avoidance controller1;
    controller1.init_controller();
    take_marble controller2;
    controller2.init_controller();

    //Declare enum used to switch between modes
    driveMode driver = pathLocator;
    int endPointNumber = 0;

    // Establish variables used for finding closest point.
    double currentX = locator.getLocationX();   // Current X location
    double currentY = locator.getLocationY();   // Current Y location
    double distance = 100;                      // Initiate distance
    double testDist = 0;                        // Initiate distance test variable.
    std::array<double,2> testPoint;             // Initiate cartesian point test variable.
    std::array<double,2> closePoint;            // Initiate cartesian point storing closest point.

    // Find closest path point.
    for (uint i = 0; i < pathPoints.size(); i++)
    {
        // Extract and translate point from path.
        testPoint = myMap.getCoordsXY(pathPoints[i]);

        // Calculate distance form current location to point location; sqrt((x2-x1)^2 + (y2-y1)^2)
        testDist = std::sqrt(pow((testPoint[0] - currentX),2) + pow((testPoint[1] - currentY),2));

        // If tested distance is shorter than shortest distance; update.
        if (testDist < distance)
        {
            distance = testDist;
            closePoint = testPoint;
        }
    }

    // Shit pile of debugging output by Malytm
    //std::cout << currentX << ':' << currentY << std::endl;
    //std::cout << "Path point: " << pathPoints[0] << std::endl;
    //testPoint = myMap.getCoordsXY(pathPoints[0]);
    //std::cout << testPoint[0] << ':' << testPoint[1] << std::endl;
    //std::cout << closePoint[0] << ':' << closePoint[1] << std::endl;
    //return 0;


    // Round x and y values of closest point to 1 decimal.
    closePoint[0] = roundf(closePoint[0] * 10) / 10;
    closePoint[1] = roundf(closePoint[1] * 10) / 10;

    // Establish vectors for containing paths.
    std::vector<cv::Point> drivePathPoints;          // Vector containing Points
    std::vector<std::array<double,2>> drivePathXY;  // Vector containing x and y values.

    // Loop
    while (true)
    {
        // Run loop in specific time intervals.
        gazebo::common::Time::MSleep(10);

        // Gazebo magic
        mutex.lock();
        cv::waitKey(1);
        mutex.unlock();

        // Initialize driving; go to closes path point.
        if (driver == pathLocator)
        {
            // Update robot location and direction in controller.
            control.setPosX(locator.getLocationX());
            control.setPosY(locator.getLocationY());
            control.setDir(locator.getDir());

            // Initiate movement towards closes path point.
            control.movePoint(closePoint);

            // Set direction and speed accoridng to controller output.
            dir = control.getDir();
            //speed = control.getSpeed();
            speed = 0.05;

            // If point is reached, shift into "follow path" mode.
            if ((locator.getLocationX() == closePoint[0]) && (locator.getLocationY() == closePoint[1]))
            {
                driver = pathCalc;   // Mode = calculate path.
                std::cout << "Arrived at path point..." << std::endl;
            }
        }

        // Drive mode; Calculate path
        if (driver == pathCalc)
        {
            cv::Point startPos = myMap.getCoordsPoint(locator.getLocationX(),locator.getLocationY());

            // Generate path from current point to 'some' endpoint.
            drivePathPoints = pathplan.getPath(startPos,endPoints[endPointNumber]);
            endPointNumber += 1;
            drivePathXY.clear();

            // Convert path to XY coordinates.
            for (uint i = 0; i < drivePathPoints.size(); i++)
            {
                //drivePathXY.insert(drivePathXY.begin(), myMap.getCoordsXY(drivePathPoints[i]));
                drivePathXY.push_back(myMap.getCoordsXY(drivePathPoints[i]));
            }

            //Round elements in array
            for (uint i = 0; i < drivePathXY.size(); i++)
            {
                drivePathXY[i][0] = roundf(drivePathXY[i][0] * 10) / 10;
                drivePathXY[i][1] = roundf(drivePathXY[i][1] * 10) / 10;
            }

            // Clear path image
            testImage = floor_plan.clone();

            // Draw path for testing purposes
            for(uint i = 0; i<drivePathPoints.size();i++)
            {
                testImage.at<cv::Vec3b>(drivePathPoints[i]) = {0,0,255};
            }

            // Show test image in test space frame.
            mutex.lock();
            cv::imshow("Test Space", testImage);
            mutex.unlock();

            // Set mode; follow path.
            driver = endPointsDriver;
            std::cout << "Path calculated succesfully..." << std::endl;
        }

        if (driver == endPointsDriver)
        {
            // Update robot location and direction in controller.
            control.setPosX(locator.getLocationX());
            control.setPosY(locator.getLocationY());
            control.setDir(locator.getDir());

            // Start controller.
            std::cout << "Driving path..." << std::endl;
            control.moveVector(drivePathXY);

            if (control.getActive() == 0)
            {
                driver = pathCalc;
            }

            // Set static speed.
            speed = 0.13;
            dir = control.getDir();
        }

        //Move between endpoints with vector
        /*
        if (control.getActive() == 0)
        {
            control.moveVector();
        }
        */

        // POINT TO POINT NAVIGATION
        /*
        control.setPosX(locator.getLocationX());
        control.setPosY(locator.getLocationY());
        control.setDir(locator.getDir());

        control.movePoint(3, 2);
        dir = control.getDir();
        speed = control.getSpeed();
        */


        // Fuzzy logic driving + marbCollection
        /*

        // Control
        sm_dist = 10;
        sm_angl = 0;

        // Fuzzy Controller variables
        float sm_dist = 10;
        float sm_angl = 0;

        // Find smallest angle
        for (int i = 30; i<170;i++)
        {
            if (lidar_ranges[i]<sm_dist)
            {
                sm_dist = lidar_ranges[i];
                sm_angl = lidar_angles[i];
            }
        }

        mutex.lock();
        if (marbDetect.marbleClose()) //Marble is right infront, just has to be collected... had issues with weird behavior on fuzzymarb close, can explain in report
        {
            controller2.setValues(sm_dist, sm_angl, marbDetect.getMarb());
            controller2.process();
            dir = controller2.getOutput().direction;
            speed = 0.5;
        }
        else if (marbDetect.getBlue() > 200 || marbDetect.getBlue() < -200) //marble collection controller
        {
            controller2.setValues(sm_dist, sm_angl, marbDetect.getMarb());
            controller2.process();
            speed = controller2.getOutput().speed;
            dir = controller2.getOutput().direction;
        }
        else //obstacle avoidance controller
        {
            controller1.setValues(sm_dist, sm_angl);
            controller1.process();
            speed = controller1.getOutput().speed;
            dir = controller1.getOutput().direction;
        }
        mutex.unlock();
        */

        // Generate location map
        myLocationMap = myMap.myLocation(locator.getLocationX(),locator.getLocationY(),locator.getDir());

        // Show images
        mutex.lock();
        cv::imshow("detected circles", marbDetect.getCirc());
        cv::imshow("Location Map", myLocationMap);
        mutex.unlock();

        // Generate a pose
        ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

        // Convert to a pose message
        gazebo::msgs::Pose msg;
        gazebo::msgs::Set(&msg, pose);
        movementPublisher->Publish(msg);
    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}

