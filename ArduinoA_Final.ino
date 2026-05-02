#include <Arduino.h>
#include <Wire.h>
#include <IRremote.hpp>

//Arduino A = Master.
//Arduino B = Slave.

// For forward rotation, the back wheels need to have this syntax.
//digitalWrite(IN1_2,LOW);
//digitalWrite(IN2_2,HIGH);

// For forward rotation, the front wheels need to have this syntax.
//digitalWrite(IN1,HIGH);
//digitalWrite(IN2,LOW);

//Pins for Motor Controller A --> Controls the front two wheels 
// IN1 & IN 2 are for the LHS motor and IN3 & IN4 are for the RHS motor
int IN1 = 4;
int IN2 = 3;
int IN3 = 2;
int IN4 = 8;

//Pins for Motor Controller B --> Controls the back two wheels 
// IN1_2 & IN2_2 are for the RHS motor and IN3_2 & IN4_2 are for the LHS motor
int IN1_2 = 5;
int IN2_2 = 6;
int IN3_2 = 7;
int IN4_2 = A0;

// Pins for the IR Sensor Array
int IR_Left = 11;
int IR_Mid = 12;
int IR_Right = 13;

//Pin & Variable for the IR Remote & Receiver
int IR_Pin = 0;
long IR_Value = 0;
int Robot_State = 0; // 0 - Paused | 1 - Running

//Variable That indicates what phase the robot is in.
int Loop_Num = 0;
// 1: Vert Calibration & Oscillation --> Soap applied to side of vehicle.  
// 2: Vert Oscillation --> Water applied to side of vehicle.
// 3: Horizontal Calibration & Oscillation --> Soap applied to roof of vehicle.
// 4: Horizontal Oscillation --> Water applied to roof of vehicle.

//Pins for the water and soap pumps
int Pump_Water = 10;
int Pump_Soap = 9;

int Slave_Address = 8; //The address of the slave Arduino

//Status of calibration
bool Calibration_Vert_OnGoing = false;
bool Calibration_Hori_OnGoing = false;

void setup() 
{
  //Pin initialization for Motor Controller A 
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  //Pin initialization for Motor Controller B 
  pinMode(IN1_2,OUTPUT);
  pinMode(IN2_2,OUTPUT);
  pinMode(IN3_2,OUTPUT);
  pinMode(IN4_2,OUTPUT);

  //Pin initialization for the IR Sensor Array 
  pinMode(IR_Left, INPUT);
  pinMode(IR_Mid, INPUT);
  pinMode(IR_Right, INPUT);
  
  //Pin initialization for the Pumps
  pinMode(Pump_Water, OUTPUT);
  pinMode(Pump_Soap, OUTPUT);
  digitalWrite(Pump_Water, LOW);
  digitalWrite(Pump_Soap, LOW);

  //Beginning the Serial Monitor
  Serial.begin(9600); 

  //Beginning the remote control
  IrReceiver.begin(IR_Pin,ENABLE_LED_FEEDBACK);

  //Beginning the I2C Communication
  Wire.begin(); 
}

//Used to stop robot from moving 
void Stop_Robot() 
{
  //Front LHS Motor - Stop
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,HIGH);

  //Front RHS Motor - Stop
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,HIGH);

  //Back RHS Motor - Stop
  digitalWrite(IN1_2,HIGH);
  digitalWrite(IN2_2,HIGH);

  //Back LHS Motor - Stop
  digitalWrite(IN3_2,HIGH);
  digitalWrite(IN4_2,HIGH);
}

//Used to move the robot forward
void forward() 
{
  //Front LHS Motor - Forward
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);

  //Front RHS Motor - Forward
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);

  //Back RHS Motor - Forward
  digitalWrite(IN1_2,LOW);
  digitalWrite(IN2_2,HIGH);

  //Back LHS Motor - Forward
  digitalWrite(IN3_2,LOW);
  digitalWrite(IN4_2,HIGH);
 
}

//Used to turn the robot to the left
void left()
{
  //Front LHS Motor - Backward
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);

  //Front RHS Motor - Forward
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);

  //Back RHS Motor - Forward
  digitalWrite(IN1_2,LOW);
  digitalWrite(IN2_2,HIGH);

  //Back LHS Motor - Backward
  digitalWrite(IN3_2,HIGH);
  digitalWrite(IN4_2,LOW);
  
}

//Used to turn the robot to the right
void right ()
{
  //Front LHS Motor - Forward
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);

  //Front RHS Motor - Backward
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);

  //Back RHS Motor - Backward
  digitalWrite(IN1_2,HIGH);
  digitalWrite(IN2_2,LOW);

  //Back LHS Motor - Forward
  digitalWrite(IN3_2,LOW);
  digitalWrite(IN4_2,HIGH);
}

//Used to send a vertical calibration request to the slave Arduino 
void Calibration_Vert_Request() 
{
  Wire.beginTransmission(Slave_Address); //Begins communication with the specified slave Arduino
  Wire.write(9);// Sending the number 9 | 9 = Start Vertical Calibration
  Wire.endTransmission();
}

//Used to send a Horizontal calibration request to the slave Arduino
void Calibration_Hori_Request() 
{
  Wire.beginTransmission(Slave_Address); //Begins communication with the specified slave Arduino
  Wire.write(11);// Sending the number 11 | 11 = Start Horizontal Calibration
  Wire.endTransmission();
}

//Checks if Vertical calibration is finished
bool Is_Calibration_Vert_Done () // Return true if calibration is done and false if it is not.
{
  Wire.requestFrom(Slave_Address, 1);// Asks for one byte from the slave arduino

  if (Wire.available())  //Checks if there is data available
  {
    int Status_Vert_Calibration = Wire.read(); // Stores the received data in status
    return Status_Vert_Calibration == 1; // Checks whether Status is equal to 1. If it is, then we know calibration is done. Therefore the function returns true.
  }
  return false; //No data has been received, calibration is not done, return false.
}

//Checks if Horizontal calibration is finished
bool Is_Calibration_Hori_Done () // Return true if calibration is done and false if it is not.
{
  Wire.requestFrom(Slave_Address, 1);// Asks for one byte from the slave arduino

  if (Wire.available())  //Checks if there is data available
  {
    int Status_Hori_Calibration = Wire.read();
    return Status_Hori_Calibration == 2; // Compares if Status is equal to 1. If it is, then we know calibration is done. Therefore the function returns true.
  }
  return false; //No data has been received, calibration is not done, return false. 
}

//Used to receive a signal from the IR remote
void Receive_Signal_From_Remote() 
{

  if (IrReceiver.decode()) // Checks if a signal is received
  {
    IR_Value = IrReceiver.decodedIRData.decodedRawData; //stores the received signal in the variable
    
    //Will set the robot state to "Paused"
    if (IR_Value == 0xBA45FF00 ) 
    {
      Robot_State = 0;
    }

    //Will set the robot state to "Running".
    if (IR_Value == 0xB946FF00 )
    {
      Robot_State = 1;
    }

    //Will reset the Robot 
    if (IR_Value == 0xB847FF00)
    {
      Robot_State = 2;
    }

    IrReceiver.resume();
  }
}

//Used to perform some action depending on the current robot state
void Handle_Robot_State()
{

  if (Robot_State == 0) // Make the robot stop. 
  {
    //Keeps the robot stopped until the user sends the required IR signal
    while (Robot_State == 0) 
    {
    Stop_Robot();
    Receive_Signal_From_Remote();
    }
  }

  if (Robot_State == 2) // Reset the robot
  {
    Loop_Num = 0; //Reset the Loop Number
    Robot_State = 1; // Set the the robot to running 
    Calibration_Vert_OnGoing = false; //Make sure that calibration is not requested/ongoing when we reset the robot
    Calibration_Hori_OnGoing = false; //Make sure that calibration is not requested/ongoing when we reset the robot
    digitalWrite(Pump_Water, LOW); // Ensure that the pump is turned off
    digitalWrite(Pump_Soap, LOW); // Ensure that the pump is turned off

    Wire.beginTransmission(Slave_Address); //Begins communication with the specified slave arduino
    Wire.write(10); // Sending the number 10 | 10 = Make sure calibration is not labelled as done in Arduino B
    Wire.endTransmission();
  }
  
}

//Used to send the current loop number to Arduino B.
void Transmit_LoopNum()
{
  Wire.beginTransmission(Slave_Address);
  Wire.write(Loop_Num);
  Wire.endTransmission();
}

void tapefollowing ()
{

  //Will contain the real-time state of the sensor (1 or 0). The sensors read 1 when over black tape.
  int IR_Left_Output;
  int IR_Mid_Output;
  int IR_Right_Output;
  
  //Reading the state of each sensor in the Array
  IR_Left_Output = digitalRead(IR_Left);
  IR_Mid_Output = digitalRead(IR_Mid);
  IR_Right_Output = digitalRead(IR_Right);


  if (IR_Left_Output == 1 && IR_Mid_Output == 1 && IR_Right_Output == 1) //All Sensors see black, move robot forward
  {
    forward();
  } 

  else if (IR_Left_Output == 1 && IR_Mid_Output == 1 && IR_Right_Output == 0) // Left sensor & Mid sensor are over the black tape. Turn the robot to the left.
  {
    left();
  }

  else if (IR_Left_Output == 1 && IR_Mid_Output == 0 && IR_Right_Output == 0) // Left sensor is over the black tape. Turn the robot to the left.
  {
    left();
  }

  else if (IR_Left_Output == 0 && IR_Mid_Output == 1 && IR_Right_Output == 1) // Right sensor & Mid sensor are over the black tape. Turn the robot to the right.
  {
    right();
  }

  else if (IR_Left_Output == 0 && IR_Mid_Output == 0 && IR_Right_Output == 1) // Right sensor is over the black tape. Turn the robot to the right. 
  {
    right(); 
  }

  else if (IR_Left_Output == 0 && IR_Mid_Output == 1 && IR_Right_Output == 0) // Only the middle sensor is over the black tape. Move the robot forward.
  {
    forward();
  }


  else if (IR_Right_Output == 0 && IR_Mid_Output == 0 && IR_Left_Output == 0) // The robot sees the gap in the line.  
  {
    delay(150); // Debounce for gap detection

    IR_Left_Output = digitalRead(IR_Left);
    IR_Mid_Output = digitalRead(IR_Mid);
    IR_Right_Output = digitalRead(IR_Right);

    if (IR_Right_Output == 0 && IR_Mid_Output == 0 && IR_Left_Output == 0) //After the debounce, the robot still detects the gap, therefore, stop the robot and increment the loop number.
    {
      Stop_Robot(); // Stop robot
      delay(2500); //No matter what, stop for 2.5 seconds
      Loop_Num++; //Indicate that we have completed one loop.
      Transmit_LoopNum();//Send the slave the current loop number

     if (Loop_Num == 1 ) //If the Loop_Num == 1, then we want to run the vertical calibration code.
      {
        Calibration_Vert_Request();
        Calibration_Vert_OnGoing = true; //Indicate that vertical calibration is currently ongoing.
      }

      if (Loop_Num == 3) //If the Loop_Num == 3, then we want to run the horizontal calibration code.
      {
        Calibration_Hori_Request();
        Calibration_Hori_OnGoing = true; //Indicate that Horizontal calibration is currently ongoing.
        digitalWrite(Pump_Water, LOW);
      }

      if (Loop_Num == 2 || Loop_Num == 4)
      {
        analogWrite(Pump_Water, 100); // Turns on water pump 
        digitalWrite(Pump_Soap, LOW); // Turns off soap pump  
      }

      if (Loop_Num >=  5)
      {
        Stop_Robot();
        digitalWrite(Pump_Water, LOW); // Turns off water pump 
        digitalWrite(Pump_Soap, LOW); // Turns off soap pump 
        
        while (Loop_Num>=5) //Having finished the wash cycle, wait for an input from the user. 
        {
          Receive_Signal_From_Remote(); 
          Handle_Robot_State();
        }
      }

      while (Calibration_Vert_OnGoing == true || Calibration_Hori_OnGoing == true) // While calibration is ongoing, keep the robot stationary.
      {
        Stop_Robot();
        Receive_Signal_From_Remote(); //In case the user wants to perform some action at this stage.
        Handle_Robot_State();
        
        if (Is_Calibration_Vert_Done() && Calibration_Vert_OnGoing == true ) //Checks if vertical calibration is done by communicating with Arduino B.
        {
          Calibration_Vert_OnGoing = false; //Set to false to exit the while loop.
          analogWrite(Pump_Soap, 100); // Start spraying soap
          digitalWrite(Pump_Water, LOW); // Stop spraying water
        }

        else if (Is_Calibration_Hori_Done() && Calibration_Hori_OnGoing == true) //Checks if Horizontal calibration is done by communicating with Arduino B.
        {
          Calibration_Hori_OnGoing = false; //Set to false to exit the while loop.
          analogWrite(Pump_Soap, 100); // Start spraying soap
          digitalWrite(Pump_Water, LOW); // Stop spraying water
        }
      
      }

      while (IR_Right_Output == 0 && IR_Left_Output == 0 && IR_Mid_Output == 0) // Keep moving forward until the robot clears the gap and finds the black tape again.
      {
        forward(); 
        IR_Left_Output = digitalRead(IR_Left);
        IR_Mid_Output = digitalRead(IR_Mid);
        IR_Right_Output = digitalRead(IR_Right); 
      }
    } 
  }
}


void loop() 
{
  Receive_Signal_From_Remote(); //Receive input from user 
  Handle_Robot_State();
  
  if (Robot_State == 1) 
  {
    tapefollowing(); // Calling the tape following function 
  }

}