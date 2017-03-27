#include <Wire.h>
#include <Servo.h>
#define uchar unsigned char

//led pins for testing
const int leftPower = 35;
const int leftGround = 34;
const int rightPower = 49;
const int rightGround = 48;

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
const int rightServoSpeed[] = { 0,    180,  90,   94,   0,    0,    0,    180,  90 };
const int leftServoSpeed[] =  { 180,  180,  180,  180,  95,   91,   0,    0,    91 };
Servo leftServo;
Servo rightServo;

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

  leftServo.attach(10);
  rightServo.attach(11); 
  
  turn(fullStop);
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
  leftServo.write(leftServoSpeed[Direction]);
  rightServo.write(rightServoSpeed[Direction]);
}
