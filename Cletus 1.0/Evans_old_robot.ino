#include <QTRSensors.h>
#include <Servo.h>

// Codes for the motor function.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

// enum Steering
  const int straight = 0;
  const int turnLeftHard = 1;
  const int turnLeft = 2;
  const int turnLeftSoft = 3;
  const int turnRightSoft = 4;
  const int turnRight = 5;
  const int turnRightHard = 6;
  const int reverse = 7;
  const int fullStop = 8;
  
//                              Hard is full and reverse, plain is one stop, soft is one slowed
//Servo Speeds                  S     TLH   TL  TLS   TRS   TR    TRH   Rev   Stop
const int leftServoSpeed[] =  { 180,  0,    90, 95,   180,  180,  180,  0,    90 };
const int rightServoSpeed[] = { 0,    0,    0,  0,    85,   90,   180,  180,  91 };

// Turn speeds
const int left90 = 560; // number of milliseconds that the servos need to turn hard left for a 90 degree turn
const int right90 = 560; // number of milliseconds that the servos need to turn hard right for a 90 degree turn

//Object size times
const int strafeTime = 1000; // time spent traveling left or right to avoid an object
const int forwardTime = 2000; // time spent traveling parallel to the line to pass the object

//pins and objects
const int SonicSensorPin = 11;
const int leftServoPin = 12;
const int rightServoPin = 13;
const int goButtonPin = 1;
const int ObjectDistance = 10; // number of cm to consider an object seen within
const int scanTime = 100; //ms
const bool avoidObjects = false; //when it is true it will take sonic sensor input.

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {3, 4, 5, 6, 7, 8, 9, 10}, NUM_SENSORS, TIMEOUT, EMITTER_PIN); 

const int sensorThreasholds[] = {1800,1500,1500,1500,1500,1500,1500,1500};
unsigned int sensorValues[NUM_SENSORS];
boolean sensorSeen[NUM_SENSORS];

Servo leftServo;
Servo rightServo;

//--------------------------------------------------//
// Setup function
//--------------------------------------------------//
// prepare all the sensors and wait on a button 
// press, perhaps a switch position to enable and
// disable the object sensor
//--------------------------------------------------//
void setup()
{
  // prep the sensors that we need
  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
  turn(fullStop);

  // Wait on button press before moving on to loop function
  while(digitalRead(goButtonPin) != HIGH) { } // do nothing
  
  //testing object avoidance
  if(objectSeen())
  {
    driveAroundObject();
  } 
  else
  {
    turn(straight);  
    delay(250);
  }
  turn(fullStop);    
  
  while(1) {}
}

//--------------------------------------------------//
// Main Loop function
//--------------------------------------------------//
// Check if an object is seen, if it is avoid it, if
// not find the line and adjust based on it's 
// position
//--------------------------------------------------//
void loop()
{
  // Read values from sonic sensor to see if there is an object to avoid
  if (avoidObjects)
  {
    if (objectSeen())
    {
      //turn to avoid it
      driveAroundObject();
    }
  }
  // read values from line sensor and correct
  followLine();
  delay(scanTime);
}

//--------------------------------------------------//
// driveAroundObject function
//--------------------------------------------------//
// Move the robot orthagonally then parallel, then 
// orthagonally again to avoid the object in the path
//--------------------------------------------------//
void driveAroundObject()
{
  // turn to the left and drive straight to avoid the object
  turn(turnLeftHard);
  delay(left90);
  turn(straight);
  delay(strafeTime);
  //turn right and go straight to pass the object
  turn(turnRightHard);
  delay(right90);
  turn(straight);
  delay(forwardTime);
  //turn right and go straight to get back to the line
  turn(turnRightHard);
  delay(right90);
  turn(straight);
  delay(strafeTime);
  //turn left to re align with the line
  turn(turnLeftHard);
  delay(left90);
  turn(straight);
}

//--------------------------------------------------//
// followLine function
//--------------------------------------------------//
// Take line position input from the line sensor and
// turn accordingly to remain on the line
//--------------------------------------------------//
void followLine()
{
  qtrrc.read(sensorValues);
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    if (sensorValues[i] < sensorThreasholds[i])
    { sensorSeen[i] = true; }
    else
    { sensorSeen[i] = false; }
  }
  // find the line, call the appropriate turn
  // this method checks outsides first and then moves in
  if (sensorSeen[0])
  { turn(turnLeft); }
  else if (sensorSeen[7])
  { turn(turnRight); }
  else if (sensorSeen[1])
  { turn(turnLeftSoft); }
  else if (sensorSeen[6])
  { turn(turnRightSoft); }
  else if (sensorSeen[2] || sensorSeen[3] || sensorSeen[4] || sensorSeen[5])
  { turn(straight); }
  else // nothing seen, turn left
  { turn(turnLeft); }
}

//--------------------------------------------------//
// objectSeen function
//--------------------------------------------------//
// Check if an object is seen by comparing the sonic
// sensor's value to the object distance value
// Return Boolean
//--------------------------------------------------//
boolean objectSeen()
{
  double duration, cm;
  // The PING is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(SonicSensorPin, OUTPUT);
  digitalWrite(SonicSensorPin, LOW);
  delayMicroseconds(2);
  digitalWrite(SonicSensorPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(SonicSensorPin, LOW);
  // The same pin is used to read the signal from the PING: a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(SonicSensorPin, INPUT);
  duration = pulseIn(SonicSensorPin, HIGH);
  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  return (cm < ObjectDistance);
}

//--------------------------------------------------//
// microsecondsToCentimeters function
//--------------------------------------------------//
// Calculate the distance of the object based on the 
// duration given.
// The speed of sound is 340 m/s or 29 microseconds 
// per centimeter. The ping travels out and back, so
// to find the distance of the object we take half 
// of the distance travelled.
//--------------------------------------------------//
long microsecondsToCentimeters(double microseconds) 
{
  return microseconds / 29.0 / 2.0;
}

//--------------------------------------------------//
// turn function
//--------------------------------------------------//
// given a direction call the servo writes that assign
// the coresponding value from the servo speed arrays
//--------------------------------------------------//
void turn(int Direction)
{
  leftServo.write(leftServoSpeed[Direction]);
  rightServo.write(rightServoSpeed[Direction]);
}


//==================================================//
// Code to make the motors work
//==================================================//
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