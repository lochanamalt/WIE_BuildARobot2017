void readUltrasonicSensors()
{
    // ultrasonic 2
    digitalWrite(ultrasonic2TrigPin, HIGH);
    delayMicroseconds(10);                  // must keep the trig pin high for at least 10us
    digitalWrite(ultrasonic2TrigPin, LOW);
    
    ultrasonic2Duration = pulseIn(utlrasonic2EchoPin, HIGH);
    ultrasonic2Distance = (ultrasonic2Duration/2)/3;
}


void Wall_PID(int pidspeed)
{

  Kp=1;// experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
  Kd=5; // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd)
  rightMaxSpeed=250;// max speed of the robot
  leftMaxSpeed=250;// max speed of the robot
  rightMinSpeed=20;// min speed of the robot
  leftMinSpeed=20;// min speed of the robot
  
error=ultrasonic2Distance-150;
 Serial.print(ultrasonic2Distance);
 Serial.print('\t');
 Serial.print(error);
 Serial.print('\t');
  Serial.print(lastError);
  Serial.print('\t');
   Serial.print(Kp*error);
   Serial.print('\t');
    Serial.print(error - lastError);
   Serial.print('\t');
int motorSpeed = Kp * error + Kd * (error - lastError);
lastError = error;
Serial.print(motorSpeed);
Serial.print('\t'); 
  
  rightBaseSpeed = pidspeed;
  leftBaseSpeed = pidspeed;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;

 Serial.print(rightMotorSpeed);
  Serial.print('\t'); 
  Serial.print(leftMotorSpeed);
  Serial.println();
  
   if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = rightMinSpeed; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = leftMinSpeed; // keep the motor speed positive
  analogWrite(motorR_PWM, rightMotorSpeed);
  analogWrite(motorL_PWM, leftMotorSpeed);
}



//--------------------wall code---------------------------------
void wall_code()
{
debugOutput(); // prints debugging messages to the serial console
    
    if(millis() - timeLoopDelay >= loopPeriod)
    {
        readUltrasonicSensors(); // read and store the measured distances
    
        Wall_PID(110);
        forward();
        timeLoopDelay = millis();
    }
}
//=============== stop and go  when no line===============================================================

void deadend()
{
 while(1)
  {
    qtrRead();
    if(dval[7]==1 && dval[6]==1 && dval[5]==1 && dval[4]==1 && dval[3]==1 && dval[2]==1 && dval[1]==1 && dval[0]==1)
      {
        brake();
        delay(50);
        break;
      }
     else
     {
      forward();
      PID(180);
     }
       
  }
 }

//=================================================================

void deadend_white()
{
 while(1)
  {
    qtrRead();
    if(dval[5]==0 && dval[4]==0 && dval[3]==0 && dval[2]==0)
      {
        brake();
        delay(500);
        break;
      }
     else
     {
      forward();
      PID(180);
     }
       
  }
 }


void ontoLine(int val)
{
  forward();
  for (int i = 0; i < val; i++) PID(180);
  brake();
 
}

void check_line()
{ 
  int y=0,x=0;
while(1)
      {       
      y=y+1;
      qtrRead();
      leftTurn(80,80);
      if(dval[4]==0 && dval[5]==0)
      {
        brake();
        break;
      }
      else if(y==200)
      {
        brake();
        while(1)
        {          
            qtrRead();
            rightTurn(80,80);
            x=x+1;
            if(dval[4]==0 && dval[5]==0)
            {
            brake();
            break;
            }
            else if(x==400)
            {
              brake();
              break;
            }
        }
        break;
      }

    }
  }


void qtrRead()
{
  position = qtrrc.readLine(sensorValues);
  for(int i=0; i< NUM_SENSORS ; i++ )
  {
  if(sensorValues[i] > 600) dval[i] = 1;
    else dval[i] = 0;
  }
}

void go(unsigned int turns, int Speed)
{
  leftCount=0;
  rightCount=0;
  analogWrite(motorL_PWM,Speed);
  analogWrite(motorR_PWM,Speed);
  while(leftCount<turns || rightCount<turns){
    
    if(leftCount<turns) Lforward();
    else{
      digitalWrite(motorL_dirP,HIGH);
      digitalWrite(motorL_dirN,HIGH);
    }
    if(rightCount<turns) Rforward();
    else{
      digitalWrite(motorR_dirP,HIGH);
      digitalWrite(motorR_dirN,HIGH);
    }
  }
  brake();
  delay(50);
  
  leftCount=0;
  rightCount=0;
}



void debugOutput()
{
    if((millis() - timeSerialDelay) > serialPeriod)
    {
        timeSerialDelay = millis();
    }
}


