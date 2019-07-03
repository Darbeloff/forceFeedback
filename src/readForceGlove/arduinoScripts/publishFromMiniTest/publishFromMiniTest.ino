#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Int16.h> 
#include <std_msgs/Int16MultiArray.h>  

#include <Wire.h>

// ros stuff
ros::NodeHandle forceGloveADC;
std_msgs::Int16MultiArray forceGloveSignal;
ros::Publisher forceGlovePub("forceGlovePub", &forceGloveSignal);

// pins for fsr
int fsrPinThumb = 4;     // the FSR and 10K pulldown are connected to a0
int fsrPinMiddle = 5;     // the FSR and 10K pulldown are connected to a0
int fsrPinPointer = 6;     // the FSR and 10K pulldown are connected to a0
int fsrPinKnuckle = 7;

void setup(void) 
{
  // initiate ros 
  forceGloveADC.initNode();

  // define message
  forceGloveSignal.layout.dim[0].size = 4;
  forceGloveSignal.layout.dim[0].stride = 1*4;
  forceGloveSignal.data = (int *)malloc(sizeof(int)*4);
  forceGloveSignal.data_length = 4;
  
  //advertise signal
  forceGloveADC.advertise(forceGlovePub);

  //advertise sample rate pub
//  forceGloveA/C.advertise(stPub);
}
 
void loop(void) 
{
  // read force sensing resistor signal
  forceGloveSignal.data[0]=analogRead(fsrPinThumb);
  forceGloveSignal.data[1]=analogRead(fsrPinMiddle);
  forceGloveSignal.data[2]=analogRead(fsrPinPointer);
  forceGloveSignal.data[3]=analogRead(fsrPinKnuckle);

  // publish that value
  forceGlovePub.publish(&forceGloveSignal);

  // ros stuff
  forceGloveADC.spinOnce();
} 
