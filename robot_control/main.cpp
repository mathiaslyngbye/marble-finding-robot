#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "fl/Headers.h"

#include <opencv2/opencv.hpp>


#include "localization.h"
#include "FuzzyDefault.h"
#include "marbleDetect.h"
#include <iostream>

// Common variables
float lidar_ranges[200];
float lidar_angles[200];
float w_lidar_ranges[200];
float w_lidar_angles[200];
cv::Mat camera;

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

  float speed = 0.0;
  float dir = 0.0;

  obstacle_avoidance controller;
  controller.init_controller();
  take_marble controller2;
  controller2.init_controller();

  // Fuzzy Controller variables
  float sm_dist = 10;
  float sm_angl = 0;

  // Loop
  while (true) {
    gazebo::common::Time::MSleep(10);

//    DrawCircle(camera);

    mutex.lock();
    cv::waitKey(1);
    mutex.unlock();

    // Control
    sm_dist = 10;
    sm_angl = 0;

    for (int i = 30; i<170;i++)
    {
        if (lidar_ranges[i]<sm_dist)
        {
            sm_dist = lidar_ranges[i];
            sm_angl = lidar_angles[i];
        }
    }

    if (true) //marble collection controller
    {
        controller.setValues(sm_dist, sm_angl);
        controller.process();

        speed = controller.getOutput().speed;

        dir = controller.getOutput().direction;
    }
    else //obstacle avoidance controller
    {
        controller.setValues(sm_dist, sm_angl);
        controller.process();

        speed = controller.getOutput().speed;

        dir = controller.getOutput().direction;
    }

    if (locator.getDir() > 0.0)
    {
        std::cout << "works" << std::endl;
    }

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

