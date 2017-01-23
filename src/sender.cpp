/*
 * Sender.cpp
 *
 *		Fichero para probar las transmisiones entre el módulo de detección de 
 *		cara y el módulo de control
 *  Created on: 23 ene. 2017
 *      Author: David
 */

//std
#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Point.h"


int main(int argc, char **argv)
{
	// Módulos ROS
	ros::init(argc, argv, "point_publisher");
	ros::NodeHandle n;


	ros::Publisher point_pub = n.advertise<geometry_msgs::Point>("pointpub", 1000);
	ros::Rate loop_rate(1000);

	geometry_msgs::Point p;
	
	p.x = 0;
	p.y = 0;

	while (ros::ok())
    {

	    point_pub.publish(p);

		ros::spinOnce();
		loop_rate.sleep();
		//p.x++;
		//p.y++;

    } 
	return 0;
}



