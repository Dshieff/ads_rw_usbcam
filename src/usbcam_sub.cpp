#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int16.h>

#include "AdsLib/AdsLib.h"
#include "AdsLib/AdsNotificationOOI.h"
#include "AdsLib/AdsVariable.h"

#include <thread>
#include <functional>
 
// Author: Addison Sears-Collins 
// Modified by: Dasha Shieff to include beckhoff/ADS
// Website: https://automaticaddison.com
// Description: A basic image subscriber + ADS link for ROS in C++
// Date: June 27, 2020, June 16 2022
 
 //declaring a pointer to member so the msg can be called in a parallel loop
 typedef const boost::function< void(const std_msgs::Int16::ConstPtr& msg)> callback;


//member function, the call back is in here
class subscriberReader {
  public:
    int avg_frame_col;
    void avgColourCallback(const std_msgs::Int16::ConstPtr& msg);
 };


void subscriberReader::avgColourCallback(const std_msgs::Int16::ConstPtr& msg)
{
  this->avg_frame_col = msg->data;
}


static void writeAvgColour(int avgColourIn,const AdsDevice& route)
{
    AdsVariable<uint8_t> avgColourOut {route, "MAIN.writeVar"};
    avgColourOut = avgColourIn;
    

    ROS_INFO("Average Colour of Camera Frame [%d] is written to Main.writeVar", avgColourIn);
}

//set rate to be slow enough so that TwinCAT is happy
void sendToADSThread(int const & avgColour)
{
  ros::Rate rate(10);

  // AMS net ID of the TwinCAT route on the target
  static const AmsNetId remoteNetId { 10, 199, 109, 140, 1, 1 };
  //IP address of the target (TC computer)
  static const char remoteIpV4[] = "192.168.0.2";

  // uncomment and adjust if automatic AmsNetId does not work (for the client, which is this computer)
  bhf::ads::SetLocalAddress({192, 168, 0, 1, 1, 1});

  AdsDevice route {remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};

  while (ros::ok()) {

      writeAvgColour(avgColour,route);

      rate.sleep();
  }

}

int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "avgColour_listener");
   
  // Default handler for nodes in ROS
  ros::NodeHandle nh;
   
  // Used to publish and subscribe to images
  image_transport::ImageTransport it(nh);
  
  subscriberReader subscriberInstance;

  //using boost bind to point to a member function
  callback boundAvgColourCallback = boost::bind( &subscriberReader::avgColourCallback, &subscriberInstance, _1);


  // Subscribe to the /camera topic
  ros::Subscriber sub = nh.subscribe("averageColour", 10, boundAvgColourCallback);

  //runs the ads in a different loop 
  std::thread worker(sendToADSThread, std::ref(subscriberInstance.avg_frame_col));


  ros::spin();
  worker.join();
   
  // Close down OpenCV
  //cv::destroyWindow("view");
}
