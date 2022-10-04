#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int16.h>
 
// Author: Addison Sears-Collins 
// Modified by: Dasha Shieff
// Website: https://automaticaddison.com
// Description: A basic image publisher for ROS in C++
// Date: June 27, 2020, June 16 2022


int main(int argc, char** argv)
{
    //initialise ros
    ros::init(argc, argv, "webcam_pub_cpp");

    // Default handler for nodes in ROS
    ros::NodeHandle nh;  
 
    // 0 reads from your default camera
    const int CAMERA_INDEX = 0;
    cv::VideoCapture capture(CAMERA_INDEX); 
    if (!capture.isOpened()) {
      ROS_ERROR_STREAM("Failed to open camera with index " << CAMERA_INDEX << "!");
      ros::shutdown();
    }
     
    // Image_transport is responsible for publishing and subscribing to Images
    image_transport::ImageTransport it(nh);
     
    // Publish to the /camera topic
    ros::Publisher avg_colour = nh.advertise<std_msgs::Int16>("averageColour", 10);
     
    //Mat is the image class defined in OpenCV
    cv::Mat frame;
 
    ros::Rate loop_rate(10);
 
    while (nh.ok()) {
 
      // Load image (type, CV_8UC3)
      capture >> frame; 
    
      // Check if grabbed frame has content
      if (frame.empty()) {
        ROS_ERROR_STREAM("Failed to capture image!");
        ros::shutdown();
      }

      
      // get the pointer (cast to data type of Mat)
      uint8_t *pImgData = (uint8_t *)frame.data;
      //find length of the Mat structure
      int length = frame.total()*frame.channels();

      //before finding average, sum all pixels
      float sumPixel = 0;

      //iterate over all pixels of the image
      for(int row = 0; row < frame.rows; row++) {
          for(int column = 0; column < frame.cols; column++) {
            for (int channel = 0; channel < frame.channels(); ++channel)
            {
                sumPixel = (int)pImgData[frame.channels() * (frame.cols * row + column) + channel] + sumPixel;
            }
          }
      }
      //find average colour value
      int avg_frame_col = sumPixel/length;
 
      std_msgs::Int16 msg;

      msg.data = avg_frame_col;


      //outputs this data into the console
      avg_colour.publish(msg);

      //cv::waitKey(1); // Display image for 1 millisecond
 
      ros::spinOnce();
      loop_rate.sleep();
    }  
 
    // Shutdown the camera
    //capture.release();
}
