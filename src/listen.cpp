// Librerias para incluir
#include <iostream>
#include <vector>

// Includes to PID
#include <ctime>

// ROS includes
#include "ros/ros.h"

#include "geometry_msgs/Point.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#include <std_msgs/Int32.h>

// Subscriber funcion
void PanAndTiltValues(const std_msgs::Int32MultiArray& array)
{

  int yaw, pitch;

  std::cout << "He entrado!!\n";

  // Set the values
  yaw = array.data[0];
  pitch = array.data[1];
  std::cout << "Yaw = " << yaw << " Pitch = " << pitch << std::endl;

  return;
}

// Subscriber funcion
void YawValues(const std_msgs::Int32& array)
{

  std::cout << "He entrado!!\n";

  // Set the values
  std::cout << "Yaw = " << array << std::endl;

  return;
}

int main(int argc, char **argv)
{
	// Módulos ROS
	ros::init(argc, argv, "Pat_Receiver");
	ros::NodeHandle n;


	//ros::Subscriber sub = n.subscribe("PAT", 1000, PanAndTiltValues);
	ros::Subscriber sub2 = n.subscribe("chatter", 1000, YawValues );
	ros::Rate loop_rate(1000);

	while (ros::ok())
    {

		ros::spinOnce();
		loop_rate.sleep();
    } 
	return 0;
}