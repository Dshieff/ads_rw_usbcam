#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include <iostream>
using namespace std;
//using namespace bhf;

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
 typedef const boost::function< void(const std_msgs::Int32::ConstPtr& msg)> callback;


//member function, the call back is in here
class subscriberReader {
  public:
    int avg_frame_col;
    void avgColourCallback(const std_msgs::Int32::ConstPtr& msg);
 };


void subscriberReader::avgColourCallback(const std_msgs::Int32::ConstPtr& msg)
{
  this->avg_frame_col = msg->data;
}


static void writeAvgColour(int avgColourIn,const AdsDevice& route)
{
    uint8_t frameR = (int)(avgColourIn/1000000); 
    uint8_t frameG = (int)((avgColourIn - frameR*1000000)/1000);
    uint8_t frameB = (int)(avgColourIn - frameR*1000000 - frameG*1000);
    std::array<uint8_t,3> avgFrameCol = {frameR,frameG,frameB};
    
    //the variable is written to a zero offset in the main plc code
    long error = AdsSyncWriteReqEx(route.GetLocalPort(),&route.m_Addr,0x4020,0,3,&avgFrameCol);
    ROS_INFO("Average Colour of Camera Frame [%d] is written to Main.writeVar", avgColourIn);
}

//set rate to be slow enough so that TwinCAT is happy
void sendToADSThread(int const & avgColour)
{
  ros::Rate rate(50);

  // AMS net ID of the TwinCAT route on the target
  static const AmsNetId remoteNetId { 5, 71, 63, 181, 1, 1 };
  //IP address of the target (TC computer)
  static const char remoteIpV4[] = "192.168.1.8";
  // uncomment and adjust if automatic AmsNetId does not work (for the client, which is this computer)
  bhf::ads::SetLocalAddress({192, 168, 1, 4, 1, 1});
  AdsDeviceState AdsState;

  char buffer;
  uint32_t bytesRead;
  while (ros::ok()) {
     AdsDevice route {remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};
     //dummy function to detect errors in the connection between client and target
     uint16_t state[2];
     long error = AdsSyncReadStateReqEx(route.GetLocalPort(),&route.m_Addr, &state[0], &state[1]);
	  
     if (error == 0) {
        writeAvgColour(avgColour,route);
     }
	  
     ROS_INFO("error [%ld]", error);
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
  ros::Subscriber sub = nh.subscribe("averageColour", 1, boundAvgColourCallback);

  //runs the ads in a different loop 
  std::thread worker(sendToADSThread, std::ref(subscriberInstance.avg_frame_col));


  ros::spin();
  worker.join();
}
