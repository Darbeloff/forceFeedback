#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Int16.h> 
#include <std_msgs/Int16MultiArray.h>  

#include <Wire.h>

// ros stuff
ros::NodeHandle forceGloveADC;
std_msgs::Int16MultiArray forceGloveSignal;
ros::Publisher forceGlovePub("forceGlovePub", &forceGloveSignal);

// sampe rate ros
//std_msgs::Int16 stSignal;
//ros::Publisher stPub("stPub", &stSignal);

// pins for fsr
int fsrPinRing = 5;     // the FSR and 10K pulldown are connected to a0
int fsrPinThumb = 7;     // the FSR and 10K pulldown are connected to a0
int fsrPinPointer = 6;     // the FSR and 10K pulldown are connected to a0
int fsrPinMiddle = 4;

int prevTime = 0;
int currTime = 0;
int num_sensors = 2;

void setup(void) 
{
  // initiate ros 
  forceGloveADC.initNode();
  
  // define message
  forceGloveSignal.layout.dim[0].size = num_sensors;
  forceGloveSignal.layout.dim[0].stride = 1*num_sensors;
  forceGloveSignal.data = (int *)malloc(sizeof(int)*num_sensors);
  forceGloveSignal.data_length = num_sensors;
  
  //advertise signal
  forceGloveADC.advertise(forceGlovePub);

  //advertise sample rate pub
//  forceGloveA/C.advertise(stPub);
}
 
void loop(void) 
{
  // read force sensing resistor signal
  forceGloveSignal.data[0] = analogRead(6); 
  forceGloveSignal.data[1] = analogRead(5);  

  // publish that value
  forceGlovePub.publish(&forceGloveSignal);

  // ros stuff
  forceGloveADC.spinOnce();

  //check sample rate
//  currTime = millis();
//  stSignal.data = currTime-prevTime;
//  prevTime = currTime;
//  stPub.publish(&stSignal);
} 
