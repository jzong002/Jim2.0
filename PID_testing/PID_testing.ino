#include "PID_v1.h"
#include "PID_AutoTune_v0.h"

// Motion speeds
const int topSpeed=90;
const int lowSpeed=75;

//Multiplied by the speed of each individual motor to ensure they spin at the same rate when we give them the same number. 
//Needs to be tuned for your motors on a case by case basis
//Remember, if you multiply a float by an int, you will result with a float. You need to recast using (int) to get an integer back out.
const float rightSpeedMultiplier=.97;
const float leftSpeedMultiplier=1;

// His variables
int sensorValueR = 0;  // right sensor value =0   
int sensorValueL = 0; // left sensor value=0
int Sum=0;         // integral correction value
const int Igain =1;      // Integral gaining value
const int maxS=12;        //Maximum Sum value
const int scale=50;
const int defsped=100;

// My variables
double setpoint = 50.0;
double input, output;
const float Kp = 3;
const float Ki = 7;
const float Kd = 2;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

int inputArray[20] = {5,10,15,20,25,30,50,50,50,50,75,80,60,50,45,30,20,45,50,50};
int i = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("PID controller Demo");
  Serial.println("Input\tOutput");
  myPID.SetOutputLimits(0,100); //the output will be a decimal between 0 and 1
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
