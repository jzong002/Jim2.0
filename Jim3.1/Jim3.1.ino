#include <PID_v1.h>

// Enumeration for the motor function.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

// Motor initialization:
#define LEFT_MOTOR_NUM 4
#define RIGHT_MOTOR_NUM 3

// Motor Limit and Speed variables and scalars
const int topSpeed = 255; 
int leftSpeed = topSpeed; 
int rightSpeed = topSpeed;
const float rightSpeedMultiplier=.97;
const float leftSpeedMultiplier=1;

// Sensor things
#define NUM_OF_SENSORS 2
const int IR_Sensors[NUM_OF_SENSORS] = {A14,A15}; // Left, Right
int sensorLimit = 512;  // threshold between white and black for reflectance sensors
int errorValues[NUM_OF_SENSORS] = {-10, 10}; // error values for sensors

// positioning and control values
double desPos = 0; //desired position/angle
double curPos; // current position
double dxSpeed; // change in speed
double Kp = 5.75; // proportionality constant
double Ki = 0.75; // integration constant
double Kd = 1.1; // differentiation constant

// initialize PID class with given varriables and constants
PID pidExecutive(&curPos, &dxSpeed, &desPos, Kp, Ki, Kd, DIRECT); 

void setup(){
  // Setup debugging
  Serial.begin(9600);
  Serial.println("Line Sensing Robot Demo");
  //printMotorsHeader();
  
  //Setup PID
  pidExecutive.SetMode(AUTOMATIC); 
  pidExecutive.SetOutputLimits(-255, 255); 
  pidExecutive.SetSampleTime(1); //default is 100, might be fast enough

  //Setup sensor pins
  for(int i = 0; i < NUM_OF_SENSORS; i++){
    pinMode(IR_Sensors[i], INPUT);
  }

  //Start Motors
  updateMotors(leftSpeed,rightSpeed); 

}


void loop(){
  curPos = calcCurPos();
  pidExecutive.Compute();
  adjustSpeed();
  updateMotors(leftSpeed,rightSpeed); 
  //printMotors();
  //printSensors();
  delay(10); 
}

/* double calcCurPos
 * For each sensor that is higher then the sensor limit the
 * associated error is summed giving a current position.
 * The Current position will then be 0 for no sensors or
 * both sensors.
 */
double calcCurPos(){
  double pos = 0; 
  for(int i = 0; i < NUM_OF_SENSORS; i++){
    if(analogRead(IR_Sensors[i]) <= sensorLimit)
    {
      pos += errorValues[i];
    } 
  }
  return pos; 
}

/* void adjustSpeed
 * based on dxSpeed the motors will be adjusted to a new speed
 * and limitted to their motor speed limits.
 * positive dxSpeed cause the robot to turn left
 */
void adjustSpeed(){
  leftSpeed -= dxSpeed; 
  rightSpeed += dxSpeed; 
  if(leftSpeed > topSpeed){leftSpeed = topSpeed;} 
  if(leftSpeed < 0){leftSpeed = 0;} 
  if(rightSpeed > topSpeed){rightSpeed = topSpeed;} 
  if(rightSpeed < 0){rightSpeed = 0;} 
}

/* void updateMotors
 * Changes motor speed based on the paramaters, left and right
 */
void updateMotors(double ls, double rs){ 
  motor(LEFT_MOTOR_NUM, FORWARD, (int)ls);
  motor(RIGHT_MOTOR_NUM, FORWARD, (int)rs);
}

//////////////////////////////////////////////////////////////////////////
// Supplied Debug Code
//////////////////////////////////////////////////////////////////////////

/* void printMotorsHeader
 * To be called in setup in conjuntion with print motors.
 * Gives column names at the start of running
 */
void printMotorsHeader(){
  Serial.print("curPos\tdxSpeed\tleftSpeed\trightSpeed\n"); 
}

/* void printMotors
 * prints out values to see whats going on with the motors
 */
void printMotors(){
  Serial.print(curPos); 
  Serial.print("\t");
  Serial.print(dxSpeed); 
  Serial.print("\t");
  Serial.print(leftSpeed); 
  Serial.print("\t");
  Serial.println(rightSpeed); 
}

/* void printSensors
 * prints out raw sensor inputs 
 */
void printSensors(){
  for(int i = 0; i < NUM_OF_SENSORS; i++)
  {
    Serial.print(analogRead(IR_Sensors[i]));
    Serial.print("\t");
  }
  Serial.println();
}

//////////////////////////////////////////////////////////////////////////
// Code to make the motors work
//////////////////////////////////////////////////////////////////////////
// Arduino pins for the shift register
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8
// 8-bit bus after the 74HC595 shift register
// (not Arduino pins)
// These are used to set the direction of the bridge driver.
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6
// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3    //Right Motor
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5   //Left Motor
#define SERVO1_PWM 10
#define SERVO2_PWM 9
// Initializing
// ------------
// There is no initialization function.
//
// The shiftWrite() has an automatic initializing.
// The PWM outputs are floating during startup,
// that's okay for the Motor Shield, it stays off.
// Using analogWrite() without pinMode() is valid.
//


// ---------------------------------
// motor
//
// Select the motor (1-4), the command,
// and the speed (0-255).
// The commands are: FORWARD, BACKWARD, BRAKE, RELEASE.
//
void motor(int nMotor, int command, int speed)
{
  int motorA, motorB;

  if (nMotor >= 1 && nMotor <= 4)
  {  
    switch (nMotor)
    {
    case 1:
      motorA   = MOTOR1_A;
      motorB   = MOTOR1_B;
      break;
    case 2:
      motorA   = MOTOR2_A;
      motorB   = MOTOR2_B;
      break;
    case 3:
      motorA   = MOTOR3_A;
      motorB   = MOTOR3_B;
      break;
    case 4:
      motorA   = MOTOR4_A;
      motorB   = MOTOR4_B;
      break;
    default:
      break;
    }

    switch (command)
    {
    case FORWARD:
      motor_output (motorA, HIGH, speed);
      motor_output (motorB, LOW, -1);     // -1: no PWM set
      break;
    case BACKWARD:
      motor_output (motorA, LOW, speed);
      motor_output (motorB, HIGH, -1);    // -1: no PWM set
      break;
    case BRAKE:
      // The AdaFruit library didn't implement a brake.
      // The L293D motor driver ic doesn't have a good
      // brake anyway.
      // It uses transistors inside, and not mosfets.
      // Some use a software break, by using a short
      // reverse voltage.
      // This brake will try to brake, by enabling
      // the output and by pulling both outputs to ground.
      // But it isn't a good break.
      motor_output (motorA, LOW, 255); // 255: fully on.
      motor_output (motorB, LOW, -1);  // -1: no PWM set
      break;
    case RELEASE:
      motor_output (motorA, LOW, 0);  // 0: output floating.
      motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
    default:
      break;
    }
  }
}


// ---------------------------------
// motor_output
//
// The function motor_ouput uses the motor driver to
// drive normal outputs like lights, relays, solenoids,
// DC motors (but not in reverse).
//
// It is also used as an internal helper function
// for the motor() function.
//
// The high_low variable should be set 'HIGH'
// to drive lights, etc.
// It can be set 'LOW', to switch it off,
// but also a 'speed' of 0 will switch it off.
//
// The 'speed' sets the PWM for 0...255, and is for
// both pins of the motor output.
//   For example, if motor 3 side 'A' is used to for a
//   dimmed light at 50% (speed is 128), also the
//   motor 3 side 'B' output will be dimmed for 50%.
// Set to 0 for completelty off (high impedance).
// Set to 255 for fully on.
// Special settings for the PWM speed:
//    Set to -1 for not setting the PWM at all.
//
void motor_output (int output, int high_low, int speed)
{
  int motorPWM;

  switch (output)
  {
  case MOTOR1_A:
  case MOTOR1_B:
    motorPWM = MOTOR1_PWM;
    break;
  case MOTOR2_A:
  case MOTOR2_B:
    motorPWM = MOTOR2_PWM;
    break;
  case MOTOR3_A:
  case MOTOR3_B:
    motorPWM = MOTOR3_PWM;
    break;
  case MOTOR4_A:
  case MOTOR4_B:
    motorPWM = MOTOR4_PWM;
    break;
  default:
    // Use speed as error flag, -3333 = invalid output.
    speed = -3333;
    break;
  }

  if (speed != -3333)
  {
    // Set the direction with the shift register
    // on the MotorShield, even if the speed = -1.
    // In that case the direction will be set, but
    // not the PWM.
    shiftWrite(output, high_low);

    // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)    
    {
      analogWrite(motorPWM, speed);
    }
  }
}


// ---------------------------------
// shiftWrite
//
// The parameters are just like digitalWrite().
//
// The output is the pin 0...7 (the pin behind
// the shift register).
// The second parameter is HIGH or LOW.
//
// There is no initialization function.
// Initialization is automatically done at the first
// time it is used.
//
void shiftWrite(int output, int high_low)
{
  static int latch_copy;
  static int shift_register_initialized = false;

  // Do the initialization on the fly,
  // at the first time it is used.
  if (!shift_register_initialized)
  {
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);

    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);

    // start with all outputs (of the shift register) low
    latch_copy = 0;

    shift_register_initialized = true;
  }

  // The defines HIGH and LOW are 1 and 0.
  // So this is valid.
  bitWrite(latch_copy, output, high_low);

  // Use the default Arduino 'shiftOut()' function to
  // shift the bits with the MOTORCLK as clock pulse.
  // The 74HC595 shiftregister wants the MSB first.
  // After that, generate a latch pulse with MOTORLATCH.
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5);    // For safety, not really needed.
  digitalWrite(MOTORLATCH, LOW);
}
