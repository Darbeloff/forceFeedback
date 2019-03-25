#include "motorClass.h"

#include <ros.h>
#include <std_msgs/Float32.h>

// Robotzone motor
float gearRatio = 721;
float EncCntsPerRev = 48.0;

// create motor object
motorClass forceMotor =  motorClass(3,4,8,gearRatio,EncCntsPerRev);

// ros stuff
ros::NodeHandle arduino2Motor;

// publisher for testing
std_msgs::Float32 testing;
ros::Publisher testingPub("testing", &testing);

void commandCallback(const std_msgs::Float32& command)
{
    // set desired motor force command
    forceMotor.setMotorForceD(command.data);
}

void measurementCallback(const std_msgs::Float32& measurement)
{
    // set desired motor force command
    forceMotor.sentMotorForce = measurement.data;
}


// subscriber
ros::Subscriber<std_msgs::Float32> commandSub("command2Arduino", &commandCallback);
ros::Subscriber<std_msgs::Float32> measurementSub("measurement2Arduino", &measurementCallback );


void setup () 
{ 
  // initialize ros
  arduino2Motor.initNode();
  
  // subscriber
  arduino2Motor.subscribe(commandSub);
  arduino2Motor.subscribe(measurementSub);

  // advertise testing publisher
  arduino2Motor.advertise(testingPub);

//  Serial.begin(57600);  
//  forceMotor.setMotorForce(1);

  delay(1000);
}

int printing = 0;
void loop ()
{
  // testing
//  forceMotor.sentMotorForce = random(10);
  
  if (printing > 1000)
  {  
  // send testing
  testing.data = forceMotor.currentCommandfd;
  testingPub.publish(&testing);
  
  // Serial.println(forceMotor.MotorForce);

  // reset
  printing = 0; 
  }
  
  // control motor
  forceMotor.force_d_closedLoopController();

  printing = printing + 1;

  arduino2Motor.spinOnce();
}
