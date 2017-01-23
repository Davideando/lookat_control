/*
*
*   LookAt Arduino control.- 
*       This program get the value from the control module, 
*       and the servos to adjust the pan & Tilt
*       Authors: Sandra & David
*/


// Includes

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

// Ros includes
#include <ros.h>

// Arduino servo library
#include <Servo.h>

// Data msg libraries
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

// Prototype of the functions
void PanAndTiltValues(const std_msgs::Int32MultiArray& array);
void ServoPosition(int yaw, int Pitch);

//ROS variables
ros::NodeHandle nh;
// Subscriber definition
ros::Subscriber<std_msgs::Int32MultiArray> sub("PAT", PanAndTiltValues);

// Define the update period
const unsigned int period = 20;

// Servo associated pins
const int PitchPin = 8;
const int YawPin = 9;

// Servo variables
Servo servoYaw; // Yaw servo
Servo servoPitch; // Pitch servo

void setup() 
{
  //ROS init
  nh.initNode();
  // Associate the subscriber to the function
  nh.subscribe(sub);

  // Link the servos to their pins
  servoYaw.attach(YawPin);
  servoPitch.attach(PitchPin);
  
  // Init the servo position
  servoYaw.write(90);
  servoPitch.write(90);

}

void loop() 
{
  // Wait for new messages
  // spin (ros sync and attend callbacks, if any ...)
  nh.spinOnce();
  
  //relax
  delay(period); 
}

// Servo update position function
void ServoPosition(int yaw, int Pitch)
{
  // Update the yaw position
  servoYaw.write(yaw);
  
  // Update the tilt position
  servoPitch.write(Pitch);  
}

// Subscriber funcion
void PanAndTiltValues(const std_msgs::Int32MultiArray& array)
{

  int yaw, pitch;

  // Set the values
  yaw = array.data[0];
  pitch = array.data[1];

  // Set the new values to the Servos
  ServoPosition(yaw, pitch);
  return;
}
