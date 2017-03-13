#include <Wire.h>
#define uchar unsigned char

//led pins for testing
const int leftPower = 35;
const int leftGround = 34;
const int rightPower = 49;
const int rightGround = 48;

// Enumeration for the motor function.
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
//motor Speeds                  S    TLH  TL   TLS  TRS  TR   TRH  Rev  Stop
const int leftServoSpeed[] =  { 68,  0,   0,   65,  70,  65,  65,  65,  0 };
const int rightServoSpeed[] = { 66,  68,  68,  68,  65,  0,   0,   63,  0 };
const int leftMotor = 3;
const int rightMotor = 4;
const int kickTime = 100; //ms
const int kickSpeed = 100;

//number of miiliseconds to stall each iteration
const int scanTime = 0;

//First sensor is left, last is right
const uchar sensorThreasholds[] = {200,200,200,200,200,200,200,200};
uchar sensorValues[16]; //only uses even bytes, so 0,2, ...14
boolean sensorSeen[8];  //used to simplify the logic
uchar t; //iterator for sensor reading

//Xbee char buffer
byte xbIn;

//--------------------------------------------------//
// Setup function
//--------------------------------------------------//
// prepare all the sensors and wait on a button 
// press, perhaps a switch position to enable and
// disable the object sensor
//--------------------------------------------------//
void setup()
{
  kickMotors();
  //turn(straight);
  //while (true){}
  // prep the sensors that we need
  Serial1.begin(9600);
  
  pinMode(52, OUTPUT);
  digitalWrite(52,HIGH);

  pinMode(leftPower, OUTPUT);
  pinMode(leftGround, OUTPUT);
  pinMode(rightPower, OUTPUT);
  pinMode(rightGround, OUTPUT);
  digitalWrite(leftPower, LOW);
  digitalWrite(leftGround, LOW);
  digitalWrite(rightPower, LOW);
  digitalWrite(rightGround, LOW);
  
  Wire.begin();  //join the i2c bus
  t=0;
  //turn(fullStop);
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
  // Here is where the code for XBee and stoplights will go:
  if (Serial1.available())
    xbIn=Serial1.read();
  if (xbIn=='a')
  {
    
  }
  else if (xbIn=='s')
  {
    
  }
  else if (xbIn=='d')
  {
    
  }
  else if (xbIn=='f')
  {
    
  }
  // read values from line sensor and correct
  followLine();
  delay(scanTime);
}

//--------------------------------------------------//
// followLine function
//--------------------------------------------------//
// Take line position input from the line sensor and
// turn accordingly to remain on the line
//--------------------------------------------------//
void followLine()
{
  //read the sensors off the i2c
  Wire.requestFrom(9,16);   //request 16 bytes from slave device #9
  while(Wire.available())   //slave device may send less than requested
  {
    sensorValues[t] = Wire.read();  //read the sensor value byte as a character
    if (t < 15)
    {
      t++;
    }
    else
    {
      t=0;
    }
  }

  //transfer sensors to the boolean array
  for (uchar i = 0; i < 8; i++)
  {
    if (sensorValues[2*i] < sensorThreasholds[i])  //weird but multiply by 2 so that we only use the even bytes: 0 -> 0, 2 -> 1 ... 14 -> 7
    { sensorSeen[i] = true; }
    else
    { sensorSeen[i] = false; }
  }
  // find the line, call the appropriate turn
  // this method checks outsides first and then moves in
  if (sensorSeen[0])
  { 
    turn(turnLeft); 
    digitalWrite(rightPower, LOW);
    digitalWrite(leftPower, HIGH);
  }
  else if (sensorSeen[7])
  { 
    turn(turnRight); 
    digitalWrite(rightPower, HIGH);
    digitalWrite(leftPower, LOW);
  }
  else if (sensorSeen[1])
  {
    turn(turnLeftSoft); 
    digitalWrite(rightPower, LOW);
    digitalWrite(leftPower, HIGH);
  }
  else if (sensorSeen[6])
  { 
    turn(turnRightSoft);
    digitalWrite(rightPower, HIGH);
    digitalWrite(leftPower, LOW);
  }
  else if (sensorSeen[2] || sensorSeen[3] || sensorSeen[4] || sensorSeen[5])
  { 
    turn(straight);
    digitalWrite(rightPower, LOW);
    digitalWrite(leftPower, LOW);
  }
  else // nothing seen, turn left
  { turn(turnLeft); }
}

//--------------------------------------------------//
// turn function
//--------------------------------------------------//
// given a direction call the servo writes that assign
// the coresponding value from the servo speed arrays
//--------------------------------------------------//
void turn(int Direction)
{
  if (Direction == reverse)
  {
    motor(leftMotor,BACKWARD,leftServoSpeed[Direction]);
    motor(rightMotor,BACKWARD,rightServoSpeed[Direction]);
  }
  else
  {
    motor(leftMotor,FORWARD,leftServoSpeed[Direction]);
    motor(rightMotor,FORWARD,rightServoSpeed[Direction]);
  }
}

void kickMotors()
{
  motor(leftMotor,FORWARD,kickSpeed);
  motor(rightMotor,FORWARD,kickSpeed);
  delay(kickTime);
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
