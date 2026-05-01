#include <Arduino.h>
#include <Wire.h>
#include <IRremote.hpp>

// For forward rotation, the back wheels need to have this syntax.
//digitalWrite(IN1_2,LOW);
//digitalWrite(IN2_2,HIGH);

// For forward rotation, the front wheels need to have this syntax.
//digitalWrite(IN1,HIGH);
//digitalWrite(IN2,LOW);

//Pins for Motor Controller A. Controls the front two wheels 
// IN1 & IN 2 are for the LHS motor and IN3 & IN4 are for the RHS motor
int IN1 = 4;
int IN2 = 3;
int IN3 = 2;
int IN4 = 8;

//Pints for Motor Controller B. Controls the back two wheels 
// IN1 & IN 2 are for the RHS motor and IN3 & IN4 are for the LHS motor
int IN1_2 = 5;
int IN2_2 = 6;
int IN3_2 = 7;
int IN4_2 = A0;

// Pins for the IR Sensor Array
int IR_Left = 11;
int IR_Mid = 12;
int IR_Right = 13;

//Pin & Variable for Remote
int IR_Pin = 0;
long IR_Value = 0;
int Robot_State = 0; // 0 - Paused | 1 - Running

//Variable That indicates what phase the robot is in.
int Loop_Num = 0;
// 0: Calibration
// 1: Side of car with Soap  | 2: Side of car with Soap
// 3: Side of car with Water | 4: Side of car with Water
// 5: Top of car with Soap   | 6: Top of car with Soap
// 7: Top of car with Water  | 8: Top of car with Water

int Pump_Water = 10;
int Pump_Soap = 9;

int Slave_Address = 8; //The Addres of the slave arduino
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
  
  ////Pin initialization for the Pumps
  pinMode(Pump_Water, OUTPUT);
  pinMode(Pump_Soap, OUTPUT);
  digitalWrite(Pump_Water, LOW);
  digitalWrite(Pump_Soap, LOW);

  //Begining the Serial Monitor
  Serial.begin(9600);
   Serial.println("MASTER BOOTED"); 
  //Begining the remote control
  IrReceiver.begin(IR_Pin,ENABLE_LED_FEEDBACK);

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

//Used to send a vertical calibration request to the slave Arduino
void Calibration_Vert_Request() 
{
  Wire.beginTransmission(Slave_Address); //Begins communication with the specifed slave arduino
  Wire.write(9);// Sending the number 9 | 9 = Start Vetical Calibration
  Wire.endTransmission();
}

//Used to send a Horizontal calibration request to the slave Arduino
void Calibration_Hori_Request() 
{
  Wire.beginTransmission(Slave_Address); //Begins communication with the specifed slave arduino
  Wire.write(11);// Sending the number 11 | 11 = Start Horizontal Calibration
  Wire.endTransmission();
}

//Check If calibration is finished
bool Is_Calibration_Vert_Done () // Return true if calibration is done and false if it is not.
{
  Wire.requestFrom(Slave_Address, 1);// Asks for one byte from the slave arduino

  if (Wire.available())  //Checks if there is data available
  {
    int Status_Vert_Calibration = Wire.read(); // Stores the recived data in status
    Serial.println ("Status Of Vert Calibration: ");
    Serial.println (Status_Vert_Calibration); 
    return Status_Vert_Calibration == 1; // Compares if Status is equal to 1. If it is, then we know calibration is done. Therefore the function returns true.
  }
  return false; //No data has been recived, calibration is not done, return false. 

}

bool Is_Calibration_Hori_Done () // Return true if calibration is done and false if it is not.
{
  Wire.requestFrom(Slave_Address, 1);// Asks for one byte from the slave arduino

  if (Wire.available())  //Checks if there is data available
  {
    int Status_Hori_Calibration = Wire.read();
    Serial.println ("Status Of Hori Calibration: ");
    Serial.println (Status_Hori_Calibration); // Stores the recived data in status
    return Status_Hori_Calibration == 2; // Compares if Status is equal to 1. If it is, then we know calibration is done. Therefore the function returns true.
  }
  return false; //No data has been recived, calibration is not done, return false. 

}

void Receive_Signal_From_Remote() 
{
  
  if (IrReceiver.decode()) 
  {
    IR_Value = IrReceiver.decodedIRData.decodedRawData;
    
    //Will set the robot stated to Paused 
    if (IR_Value == 0xBA45FF00 ) 
    {
      Robot_State = 0;
    }

    //Resume the Robot
    if (IR_Value == 0xB946FF00 )
    {
      Robot_State = 1;
    }

    //Reset the Robot 
    if (IR_Value == 0xB847FF00)
    {
      Robot_State = 2;
    }
    IrReceiver.resume();
  }
}

//Need to communicate the robot status to the slave still
void Handle_Robot_State()
{
  if (Robot_State == 0)
  {
    Serial.println("HANDLE: Robot paused, stuck in pause loop");
    //Keeps the robot stopped until the user sends the right IR signal
    while (Robot_State == 0) 
    {
    Stop_Robot();
    Receive_Signal_From_Remote();
    }

  }

  if (Robot_State == 2) 
  {
    Serial.println("HANDLE: Reset triggered");
    Loop_Num = 0;
    Robot_State = 1;
    Calibration_Vert_OnGoing = false;
    Calibration_Hori_OnGoing = false; //Make sure that calibration is not requeste/ongoing when we reset the robot
    digitalWrite(Pump_Water, LOW);
    digitalWrite(Pump_Soap, LOW);
    Wire.beginTransmission(Slave_Address); //Begins communication with the specifed slave arduino
    Wire.write(10);// Sending the number 10 | 10 = Make sure calibration is not labled as done in Arduino B
    Wire.endTransmission();
  }
  
}


void tapefollowing ()
{


  //Will contain the real-time state of the sensor (1 or 0) - Reads 1 when over blakc tape.
  int IR_Left_Output;
  int IR_Mid_Output;
  int IR_Right_Output;
  
  //Reading the state of each sensor in the Array
  IR_Left_Output = digitalRead(IR_Left);
  IR_Mid_Output = digitalRead(IR_Mid);
  IR_Right_Output = digitalRead(IR_Right);

   //Indicates If the robot is in calibration mode or not

  if (IR_Left_Output == 1 && IR_Mid_Output == 1 && IR_Right_Output == 1) //All Sensors see black, move all motors forward
  {
    forward();
    
  } 

  else if (IR_Left_Output == 1 && IR_Mid_Output == 1 && IR_Right_Output == 0) // Left sensor & Mid sensor are over the black tape. So the car must turn left. Turn the LHS wheels backwards and the RHS wheels forward.
  {
    left();
  
  }

  else if (IR_Left_Output == 1 && IR_Mid_Output == 0 && IR_Right_Output == 0) // Left sensor is over the black tape. So the car must turn left. Turn both LHS wheels backwwards and both RHS wheels forwards.
  {
    left();
   
  }

  else if (IR_Left_Output == 0 && IR_Mid_Output == 1 && IR_Right_Output == 1) // Right sensor & Mid sensor are over the black tape. So the car must turn right. Turn the RHS wheels backwards and the LHS wheels forward.
  {
    right();
    
  }

  else if (IR_Left_Output == 0 && IR_Mid_Output == 0 && IR_Right_Output == 1) // Right sensor is over the black tape. So the car must turn right. Turn the RHS wheels backwards and the LHS wheels forward.
  {
    right();
    
  }

  else if (IR_Left_Output == 0 && IR_Mid_Output == 1 && IR_Right_Output == 0) // Only the middle sensors sees black, therefore, move the robot forward
  {
    forward();
    
  }

    
  else if (IR_Right_Output == 0 && IR_Mid_Output == 0 && IR_Left_Output == 0) // Right sensor is over the black tape. So the robot must STOP. 
  {
    delay(150); // Debounce for gap detection

    IR_Left_Output = digitalRead(IR_Left);
    IR_Mid_Output = digitalRead(IR_Mid);
    IR_Right_Output = digitalRead(IR_Right);
    if (IR_Right_Output == 0 && IR_Mid_Output == 0 && IR_Left_Output == 0) 
    {
      Stop_Robot();
      delay(2500); //No matter what, stop for 1 second
      Loop_Num++; //Indicate that we have completed one loop.
      Serial.print("MASTER Loop_Num = ");
      Serial.println(Loop_Num);
      Transmit_LoopNum();//SEND THE SLAVE ARDUINO THE LOOP

     if (Loop_Num == 1 ) //If the Loop_Num == 0, then we want to run the vertical calibration code.
      {
        Calibration_Vert_Request();
        Calibration_Vert_OnGoing = true;
      }

      if (Loop_Num == 3) //If the Loop_Num == 2, then we want to run the horizontal calibration code.
      {
        Calibration_Hori_Request();
        Calibration_Hori_OnGoing = true;
        digitalWrite(Pump_Water, LOW);
      }

      if (Loop_Num == 2 || Loop_Num == 4)
      {
         analogWrite(Pump_Water, 100); // Turns on water pump when 
         digitalWrite(Pump_Soap, LOW); // Turns off soap pump when 
      }

      if (Loop_Num >=  5)
      {
        Stop_Robot();
        digitalWrite(Pump_Water, LOW); // Turns off water pump when 
        digitalWrite(Pump_Soap, LOW); // Turns off soap pump when 
        
        while (Loop_Num>=5)
        {
        Receive_Signal_From_Remote();
        Handle_Robot_State();
        }

      }


      while (Calibration_Vert_OnGoing == true || Calibration_Hori_OnGoing == true) // While calibration is on going (Calibration_OnGoing), keep the robot stationary.
      {
        Serial.println("Entering the calibration while loop");
        Stop_Robot();
        Receive_Signal_From_Remote();
        Handle_Robot_State();
        
        if (Is_Calibration_Vert_Done() && Calibration_Vert_OnGoing == true )
        {
          Calibration_Vert_OnGoing = false;
          analogWrite(Pump_Soap, 100); // Turns on soap pump when vertical calibrasiton is done
          digitalWrite(Pump_Water, LOW); 
        }

        
        else if (Is_Calibration_Hori_Done() && Calibration_Hori_OnGoing == true)
        {
          Calibration_Hori_OnGoing = false;
          analogWrite(Pump_Soap, 100); // Turns on soap pump when vertical calibrasiton is done 
          digitalWrite(Pump_Water, LOW);
          Serial.println("MASTER: hori calibration finished");
        }
        
         delay(80);
        //Check to see if calibration is done. Read the value sent by the Slave Arduino

       }
      while (IR_Right_Output == 0 && IR_Left_Output == 0 && IR_Mid_Output == 0 )
      {
        Serial.println("Done with calibration loop moving forward");
        forward(); 
        IR_Left_Output = digitalRead(IR_Left);
        IR_Mid_Output = digitalRead(IR_Mid);
        IR_Right_Output = digitalRead(IR_Right); 
      }
    } 
  }
}

    //moves the car from the area with no black tape 
    

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

void Transmit_LoopNum()
{
  Wire.beginTransmission(Slave_Address);
  Wire.write(Loop_Num);
  Wire.endTransmission();
}

void loop() 
{
  
  Receive_Signal_From_Remote();
  Handle_Robot_State();
  
  if (Robot_State == 1) 
  {
    tapefollowing(); 
  }

  // Calling the tape following function 
  //Send the slave arduino the current loop number Wire.write(Loop_Num)
}