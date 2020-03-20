//Basic Motion Functions --------------------------------------------
void start()
{
  digitalWrite(motorL_PWM, LOW);
  digitalWrite(motorR_PWM, LOW);

  digitalWrite(motorR_dirP, LOW);
  digitalWrite(motorR_dirN, LOW);
  digitalWrite(motorL_dirP, LOW);
  digitalWrite(motorL_dirN, LOW);
}


//*********************BRAKE **********************
void brake()
{
  digitalWrite(motorR_dirP,HIGH);
  digitalWrite(motorR_dirN,HIGH);
  digitalWrite(motorL_dirP,HIGH);
  digitalWrite(motorL_dirN,HIGH);
}
//*******************   ADJUST SPEED ****************

void motorSpeed(bool state)
{
  digitalWrite(motorL_PWM, state);
  digitalWrite(motorR_PWM, state);
}


void motorSpeed(int leftSpeed, int rightSpeed)
{
  analogWrite(motorR_PWM,rightSpeed);
  analogWrite(motorL_PWM,leftSpeed);
}
//**********************  GO FORWARD MARVIC *************************************

void forward()
{
Lforward();
Rforward();
}

void forward(int LSpeed, int RSpeed)
{
 Lforward();
 Rforward();
 motorSpeed(LSpeed,RSpeed);
}

//********************** REVERSE MARVIC **********************************************

void back()
{
 Lbackward();
 Rbackward();
}

void back(int LSpeed, int RSpeed)
{
 Lbackward();
 Rbackward();
 motorSpeed(LSpeed,RSpeed);
}

//*******************************  TURN LEFT    ***************************************
void leftTurn()
{
 Lbackward();
 Rforward();
}
void leftTurn(int LSpeed, int RSpeed)
{
 Lbackward();
 Rforward();
 motorSpeed(LSpeed,RSpeed);
}

//********************* TURN   RIGHT   *******************************************************************************************
void rightTurn()
{
 Rbackward();
 Lforward();
}

void rightTurn(int LSpeed, int RSpeed)
{
  Rbackward();
  Lforward();
  motorSpeed(LSpeed,RSpeed);
}

//==============MOTOR Directions==================================

void Rforward()
{
  digitalWrite(motorR_dirP,HIGH);
  digitalWrite(motorR_dirN,LOW);
}
void Rbackward()
{
  digitalWrite(motorR_dirP,LOW);
  digitalWrite(motorR_dirN,HIGH);
}
void Lforward()
{
  digitalWrite(motorL_dirP,HIGH);
  digitalWrite(motorL_dirN,LOW);
}
void Lbackward()
{
  digitalWrite(motorL_dirP,LOW);
  digitalWrite(motorL_dirN,HIGH);
}

