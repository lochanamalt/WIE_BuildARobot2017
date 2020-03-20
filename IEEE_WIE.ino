#include <QTRSensors.h>
#include <Servo.h>

Servo servo1;

float Kp=0.1;// experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
float Kd=1; // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
int rightMaxSpeed=250;// max speed of the robot
int leftMaxSpeed=250;// max speed of the robot
int rightMinSpeed=20;// min speed of the robot
int leftMinSpeed=20;// min speed of the robot

int rightBaseSpeed = 200; // this is the speed at which the motors should spin when the robot is perfectly on the line
int leftBaseSpeed = 200; // this is the speed at which the motors should spin when the robot is perfectly on the line

#define NUM_SENSORS  8    // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN
#define buzzer 35

//================= ultra sonic==================================

const int serialPeriod = 250;       // only print to the serial console every 1/4 second
unsigned long timeSerialDelay = 0;

const int loopPeriod = 1;         // a period of 100ms = a frequency of 10Hz
unsigned long timeLoopDelay   = 0;

// specify the trig & echo pins used for the ultrasonic sensors
const int ultrasonic2TrigPin = 53;
const int utlrasonic2EchoPin = 52;

int ultrasonic2Distance;
int ultrasonic2Duration;

//***************MOTORS********************


#define motorL_enb 48
#define motorL_dirP 4
#define motorL_dirN 5
#define motorL_PWM 6

#define motorR_enb 50
#define motorR_dirP 9
#define motorR_dirN 10
#define motorR_PWM 11

int lastError = 0;
int position;
float error;


//QTR Sensor

QTRSensorsRC qtrrc((unsigned char[]) { 51, 49, 47, 45, 43, 41, 39, 37} , NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];
bool dval[NUM_SENSORS];

//  Encoders functions ------------------------------------------

volatile long leftCount = 0, rightCount = 0;

void leftISR() //ISR for left motor count
{
  leftCount++;
}
void rightISR() //ISR for right motor count
{
  rightCount++;
}


//===================================================



void setup()
{

  Serial.begin(9600);
  delay(500);

  pinMode(motorR_dirP, OUTPUT);
  pinMode(motorR_dirN, OUTPUT);

  pinMode(motorL_dirP, OUTPUT);
  pinMode(motorL_dirN, OUTPUT);

  pinMode(motorR_PWM, OUTPUT);
  pinMode(motorL_PWM, OUTPUT);

  pinMode(motorL_enb, OUTPUT);
  pinMode(motorR_enb, OUTPUT);

  digitalWrite(motorR_enb, HIGH);
  digitalWrite(motorL_enb, HIGH);

  attachInterrupt(digitalPinToInterrupt(2), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), rightISR, CHANGE);


   // ultrasonic sensor pin configurations
    pinMode(ultrasonic2TrigPin, OUTPUT);
    pinMode(utlrasonic2EchoPin, INPUT);

  pinMode(buzzer, OUTPUT);

  //--------------------SERVOS--------------------------
  servo1.attach(27);
  servo1.write(90);

  //-------------CALIBRATE---------------------------
  tone(buzzer, 200, 1000);
  for (int i = 0; i < 200; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
    qtrrc.calibrate();

  tone(buzzer, 200, 1000);
  delay(1000);

  //------------------------------------------------
  float lastError = 0;
  int errorSum = 0;
}


int n = 0;
int w=0;
void loop()

{ 
  servo1.write(38);
  while (n == 0)
  { 
    qtrRead();
   if((dval[7]==1 && dval[6]==1 && dval[5]==1 && dval[4]==1 && dval[3]==1 && dval[2]==1 && dval[1]==1 && dval[0]==1))
      {
      brake();
      delay(500);
      readUltrasonicSensors();
      if(ultrasonic2Distance<350)
        {

         while(1)
          {  
              wall_code(); 
              if(ultrasonic2Distance>400)
              {
              brake();
              delay(100);
              go(100,100);
              break;
              }
           }
        }
       else
       {
        check_line();
       }
}
else
{

 while(1)
  {
    qtrRead();
    if(dval[0]==0 && dval[7]==0 && dval[1]==0 && dval[6]==0 && dval[5]==0 && dval[4]==0 && dval[3]==0 && dval[2]==0)
    {   
      if(w==0)
      {       
       Serial.print("rtr");
        w=1;
        brake();
        ontoLine(150);
        break;
      }
      else
      {
        Serial.print("rtr2");
        brake();
        ontoLine(150);
        n=1;
        break; 
       
      }
    }
  else
    { 
      forward();
      PID(180);
      if((dval[7]==1 && dval[6]==1 && dval[5]==1 && dval[4]==1 && dval[3]==1 && dval[2]==1 && dval[1]==1 && dval[0]==1))
      {
        brake();
        break;
      }
      
    }
}
}  

}}


//PID Algorithm -------------------------------------------------------------------------------------------------------------------------------------------------------------------

void PID(int pidspeed)
{

  position = qtrrc.readLine(sensorValues,QTR_EMITTERS_ON,1); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  error = position - 3500;

  int motorSpeed = (Kp * error + Kd * (error - lastError));
  
  lastError = error;
  rightBaseSpeed = pidspeed;
  leftBaseSpeed = pidspeed;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = rightMinSpeed; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = leftMinSpeed; // keep the motor speed positive
  analogWrite(motorR_PWM, rightMotorSpeed);
  analogWrite(motorL_PWM, leftMotorSpeed);
}


void PIDB(int pidspeed)
{
  position = qtrrc.readLine(sensorValues,QTR_EMITTERS_ON,1); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  error = position - 3500;
  
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
 

  rightBaseSpeed = pidspeed;
  leftBaseSpeed = pidspeed;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;


  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = rightMinSpeed; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = leftMinSpeed; // keep the motor speed positive
  analogWrite(motorR_PWM, rightMotorSpeed);
  analogWrite(motorL_PWM, leftMotorSpeed);
}



//=====================================================================================================================================================================================
