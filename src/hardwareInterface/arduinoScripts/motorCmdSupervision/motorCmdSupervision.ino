#include "motorClass.h"

// vibration motor
#include <Wire.h>
#include "Adafruit_DRV2605.h"

// ros
#include <ros.h>
#include <std_msgs/Float32.h>

// Robotzone motor
float gearRatio = 721;
float EncCntsPerRev = 48.0;

// create motor object
motorClass forceMotor =  motorClass(3, 4, 8, gearRatio, EncCntsPerRev);

// ros stuff
ros::NodeHandle arduino2Motor;

// publisher for testing
std_msgs::Float32 testing;
ros::Publisher testingPub("testing", &testing);

// vibration motor
Adafruit_DRV2605 drv;

void commandCallback(const std_msgs::Float32& command)
{
  // set desired motor force command
  forceMotor.setMotorForce(command.data);
}

void measurementCallback(const std_msgs::Float32& measurement)
{
  // set desired motor force command
  forceMotor.sentMotorForce = measurement.data;
}

void buzzCallback(const std_msgs::Float32& buzz)
{
  // set the effect to play
  drv.setWaveform(0, 118);  // play effect
  drv.setWaveform(1, 0);    // end waveform

  // play the effect!
  drv.go();
}


// subscriber
ros::Subscriber<std_msgs::Float32> commandSub("command2Arduino", &commandCallback);
ros::Subscriber<std_msgs::Float32> measurementSub("measurement2Arduino", &measurementCallback );
ros::Subscriber<std_msgs::Float32> buzzSub("buzz2Arduino", &buzzCallback );


void setup ()
{
  // initialize ros
  arduino2Motor.initNode();

  // subscriber
  arduino2Motor.subscribe(commandSub);
  arduino2Motor.subscribe(measurementSub);
  arduino2Motor.subscribe(buzzSub);

  // advertise testing publisher
  arduino2Motor.advertise(testingPub);

  // vibration motor
  drv.begin();
  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_INTTRIG);

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
    testing.data = forceMotor.currentCommandf;
    testingPub.publish(&testing);

    // Serial.println(forceMotor.MotorForce);

    // reset
    printing = 0;
  }

  // control motor
  forceMotor.force_closedLoopController();

  printing = printing + 1;

  arduino2Motor.spinOnce();
}
