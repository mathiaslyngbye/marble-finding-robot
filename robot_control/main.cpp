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

  float speed = 0.0;
  float dir = 0.0;

  int mode = 0;

  // Import image
  std::string floor_plan_JPG("../models/bigworld/meshes/floor_plan.png");
  cv::Mat floor_plan;
  floor_plan = cv::imread(floor_plan_JPG, cv::IMREAD_COLOR);

  // Create location map
  cv::Mat floor_plan_location = floor_plan.clone();
  //cv::resize(floor_plan, floor_plan_location, cv::Size(), 3, 3);
  LocationMap myMap(floor_plan_location);
  cv::Mat myLocationMap = myMap.myLocation(0,0,0);

  Pathplanner pathplan(floor_plan);

  pathplan.calculatePath();
  controller control;
  obstacle_avoidance controller1;
  controller1.init_controller();
  take_marble controller2;
  controller2.init_controller();

  // Create Window for LocationMap
  cv::namedWindow("Location Map", cv::WINDOW_AUTOSIZE);	// Create a window.

  // Fuzzy Controller variables
  float sm_dist = 10;
  float sm_angl = 0;

  std::vector<cv::Point> firstPath;
  firstPath = pathplan.getPathPoints();
  double locaX = locator.getLocationX();
  double locaY = locator.getLocationY();
  double distance = 100;
  double tempDist = 0;
  double startX = 0;
  double startY = 0;

  // MALYS TEST SPACE
   cv::namedWindow("Test Space", cv::WINDOW_AUTOSIZE);	// Create a window.
   cv::Mat testImage = floor_plan.clone();
  /* for(int index=0; index < firstPath.size(); index++)
   {
       testImage.at<cv::Vec3b>(firstPath[index]) = {0,0,255};
       std::array<double,2> testPoint = myMap.getCoordsXY(firstPath[index]);
       std::cout <<"Index: " << index <<"\tX: " << testPoint[0]  << ",\tY: " << testPoint[1] << std::endl;
       std::cout <<"Index: " << index <<"\tPint: " << firstPath[index] << std::endl;
   }

   std::cout << "Path size: "<< firstPath.size() << std::endl;
   mutex.lock();
   cv::imshow("Test Space", testImage);
   mutex.unlock();
*/
   // END OF MALYS TEST SPACE
  std::array<double,2> myPoint;

  for (int ii = 0; ii < firstPath.size(); ii++)
  {
      myPoint = myMap.getCoordsXY(firstPath[ii]);
      tempDist = std::sqrt(pow((myPoint[0] - locaX),2) + pow((myPoint[1] - locaY),2));
      //std::cout << tempDist << "  x:  " << myPoint[0] << "  y:  " << myPoint[0] << std::endl;
      if (tempDist < distance)
      {
          distance = tempDist;
          //startX = myPoint[0];
          //startY = myPoint[1];
      }
  }
  myPoint[0] = roundf(myPoint[0] * 10) / 10;
  myPoint[1] = roundf(myPoint[1] * 10) / 10;

  std::vector<cv::Point> drivePath;
  std::vector<std::array<double,2>> drivePathXY;

  // Loop
  while (true)
  {
    gazebo::common::Time::MSleep(10);

    mutex.lock();
    cv::waitKey(1);
    mutex.unlock();

    // Control
    sm_dist = 10;
    sm_angl = 0;

    // Find smallest angle
    for (int i = 30; i<170;i++)
    {
        if (lidar_ranges[i]<sm_dist)
        {
            sm_dist = lidar_ranges[i];
            sm_angl = lidar_angles[i];
        }
    }

    if (mode == 0)
    {
        control.setPosX(locator.getLocationX());
        control.setPosY(locator.getLocationY());
        control.setDir(locator.getDir());

        control.movePoint(myPoint);
        dir = control.getDir();
        //speed = control.getSpeed();
        speed = 0.05;
        //std::cout << startX << " : " << startY << std::endl;
        if ((locator.getLocationX() == startX) && (locator.getLocationY() == startY))
        {
            mode = 1;
            std::cout << "Found the path" << std::endl;
        }
    }

    if (mode == 1)
    {
        std::vector<cv::Point> endPoints = pathplan.getEndPoints();
        cv::Point startPos;
        startPos.x = startX;
        startPos.y = startY;
        startPos = myMap.getCoordsPoint(startX,startY);
        drivePath = pathplan.getPath(startPos,endPoints[2]);

        for (int p = 0; p < drivePath.size(); p++)
        {
            drivePathXY.insert(drivePathXY.begin(), myMap.getCoordsXY(drivePath[p]));
        }

        // DRAW TEST PATH
        for(uint i = 0; i<drivePath.size();i++)
        {
            testImage.at<cv::Vec3b>(drivePath[i]) = {0,0,255};
        }
        mutex.lock();
        cv::imshow("Test Space", testImage);
        mutex.unlock();
        // END OF DRAW TEST PATH

        mode = 2;
        std::cout << "calculated path" << std::endl;
    }

    if (mode == 2)
    {
        control.setPosX(locator.getLocationX());
        control.setPosY(locator.getLocationY());
        control.setDir(locator.getDir());
        speed = 0.2;
        std::cout << "Driving path" << std::endl;
        if (control.getActive() == 0)
        {
            control.moveVector(drivePathXY);
        }
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

