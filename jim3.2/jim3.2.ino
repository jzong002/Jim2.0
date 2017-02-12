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

// Codes for the motor function.
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

int sensorValueR = 0;  // right sensor value =0   
int sensorValue2L = 0; // left sensor value=0
int left=4;
int right=3;

//mess with these
int Pgain=0;         // integral correction value
float Igain =.8;      // Integral gaining value //turning speed  was .5
int maxS=5;        //Maximum Sum value //max turning speed    was 10
int scale=0;
int defsped=40; //base speed...kinda  was 30
int lastmove=0;


// Pins for Left and Right light sensors respectively

int lSens=A14;
int rSens=A15;

//Multiplied by the speed of each individual motor to ensure they spin at the same rate when we give them the same number. 
//Needs to be tuned for your motors on a case by case basis
//Remember, if you multiply a float by an int, you will result with a float. You need to recast using (int) to get an integer back out.
const float k=.5;
float rConst=.76; //.76
float lConst=1.8;//1.8


void setup()
{
  Serial.begin(9600);
  Serial.println("Line Sensing Robot Demo");
  pinMode(lSens, INPUT);
  pinMode(rSens, INPUT);  //Sets pins for line sensors to inputs
  motor(left, FORWARD, 88);
  motor(right, FORWARD, 90);
delay(150);
}

void loop()
{
 // motor_output(MOTOR4_A, HIGH, 200);
 // delay(2000);
 // motor_output(MOTOR4_A, LOW, 255);
  
      // read the digital in value:
  sensorValueR = analogRead(rSens);       // read the right sensor value
  sensorValue2L = analogRead(lSens);    // read the left sensors value
Serial.print(sensorValue2L);
Serial.print("\t");
Serial.print(sensorValueR);
Serial.print("\t");


  if (sensorValueR < 750) // change the value of the sensors to 0 and 1 
    {sensorValueR =0;}
  else 
    {sensorValueR=1;}
  if (sensorValue2L < 750)
    {sensorValue2L =0;}
  else 
    {sensorValue2L =1;}   
    
  if (sensorValueR==1 && sensorValue2L==1 )
  { 
 //   motor(right,FORWARD,(int)((defsped+k*20+Igain*Pgain)-scale)*rConst);
   // motor(left,FORWARD,(int)((defsped+k*20-Igain*Pgain)-scale)*lConst);   

   }
   else if (sensorValueR==0 && sensorValue2L==1 )
   {
      Pgain= Pgain -1;
      motor(left,FORWARD,(int)((defsped+k*40+Igain*Pgain)-scale));
      Serial.print((int)((defsped+k*40+Igain*Pgain)-scale));
      Serial.print("\t");
      motor(right,FORWARD,(int)((defsped+k*40-Igain*Pgain)-scale));
      Serial.println((int)((defsped+k*40-Igain*Pgain)-scale)); 
      lastmove=-1;
    }
    else if (sensorValueR==1 && sensorValue2L==0 )
    {   // right sensor is on
      Pgain= Pgain +1;
      motor(left,FORWARD,(int)((defsped+k*40+Igain*Pgain)+5));
      Serial.print((int)((defsped+k*40+Igain*Pgain)+5));
      Serial.print("\t");
      motor(right,FORWARD,(int)((defsped+k*40-Igain*Pgain)-scale));
      Serial.println((int)((defsped+k*40-Igain*Pgain)+5));
      lastmove=1;
    }
    else if (sensorValueR==0 && sensorValue2L==0)
    { 
      if (lastmove==1)
      {
        Pgain=1;
        lastmove=0;
      }
      else if (lastmove==-1)
      {
        Pgain=-1;
        lastmove=0;
      }
      else
      {
        Pgain=0;
      }
      Pgain=0;
    motor(right,FORWARD,(int)((defsped+k*40+Igain*Pgain)-scale));
    motor(left,FORWARD,(int)((defsped+k*40-Igain*Pgain)-scale));
    Serial.print((int)((defsped+k*40-Igain*Pgain)-scale));
    Serial.print("\t");
    Serial.println((int)((defsped+k*40+Igain*Pgain)-scale));
     
    }
    else
    {// this commands is use during testing the code
  
 
  
    /////////////////// PI control
    if ( Pgain > maxS )     // auto correction max value  
    Pgain=maxS;
    else if (Pgain <-maxS)  
    Pgain=-maxS;
          
  
  }
}


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
