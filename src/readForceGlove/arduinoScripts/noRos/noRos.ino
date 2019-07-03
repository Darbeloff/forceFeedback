#include <Wire.h>

// pins for fsr
int fsrPin2 = 4;     // the FSR and 10K pulldown are connected to a0
int fsrPinThumb = 5;     // the FSR and 10K pulldown are connected to a0
int fsrPinPointer = 6;     // the FSR and 10K pulldown are connected to a0
int fsrPinMiddle = 7;

void setup(void) 
{
  Serial.begin(9600);
}
 
void loop(void) 
{
  // read force sensing resistor signal
  //int thumb = analogRead(fsrPinThumb); 
  //int pointer = analogRead(fsrPinPointer);  

  Serial.print(analogRead(fsrPinThumb));
  Serial.print(",");
  Serial.print(analogRead(fsrPinPointer));
  Serial.print(",");
  Serial.print(analogRead(fsrPinMiddle));
  Serial.print(",");
  Serial.println(analogRead(fsrPin2));
} 
