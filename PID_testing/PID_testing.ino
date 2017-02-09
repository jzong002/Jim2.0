#include "PID_v1.h"
//#include "PID_AutoTune_v0.h"

// My variables
double setpoint = 50.0;
double input, output;
const float Kp = 10;
const float Ki = 0;
const float Kd = 100;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

int inputArray[20] = {49,50,50,51,51,52,51,50,50,50,49,48,47,48,48,48,49,50,50,51};
int i = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("PID controller Demo");
  Serial.println("Input\tOutput");
  myPID.SetOutputLimits(0,10000); //the output will be a decimal between 0 and 1
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
}

void loop() {

  input = inputArray[i];

  // compute PID, in theory it should be a smoother ride than just turning on and off wheels based on boolean inputs    
  myPID.Compute();
  
  // The PID controller will output between 0 and 1.
  /*Serial.print(leftSpeedMultiplier * topSpeed * output);
  Serial.print("\t");
  Serial.print(rightSpeedMultiplier * topSpeed * (1 - output));
  Serial.print("\t");*/
  Serial.print(input);
  Serial.print("\t");
  Serial.print(output);
  Serial.print("\n");

  //increment sensor counter
  i = i+1;
  if (i > 20) {
    Serial.println("finished");
    while(true)
    {}
   }
  // Delay and then loop
  delay(5);
}
