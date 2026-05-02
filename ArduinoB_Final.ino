#include "A4988.h"
#include <Arduino.h>
#include <Stepper.h>
#include <Wire.h>
//Pins for Horizontal Stepper Motor
int Step_Vert = 9; 
int Dir_Vert = 8;

//Pins for the Vertical ultrasonic sensor
int Trig_Vert = 10;
int Echo_Vert = 11;

//Pins for the Horizontal ultrasonic sensor
int Trig_Hori = 12;
int Echo_Hori = 13;

//Vertical Ultrasonic Sensor Variables
float Distance_Hori = 0; // Contains the measured horizontal distance
float Time_Hori;
int Count_Vert=0; //Contains the number of steps taken to reach the top of the vehicle.

//Horizontal Ultrasonic Sensor Variables  
float Distance_Vert = 0; // Contains the measured vertical distance
float Time_Vert;
int Count_To_Roof; // Contains the number of steps to reach the edge of the vehicle roof
int Count_Hori=0; //Contains the number of steps taken to cover the vehicle roof.

int Orient = 0; //Indicates the current orientation of the nozzles
//if 0 --> Parallel with the ground. 
//if 1 --> Perpendicular with the ground 

int i;


int StepsPerRev = 2048; // For Horizontal and Orientation Stepper motors

//Initializing both stepper motors.
Stepper Hori_Stepper(StepsPerRev,4,6,5,7);
Stepper Orient_Stepper(StepsPerRev,A0,A2,A1,A3); 
//-ve steps moves the motor shaft CW 

//Constants For Vertical Stepper Motor
int spr = 200; //The number of steps per revolution
int stepMode = 1; //Defining the step mode - Using Full Step
int RPM = 400; //Defining the motor speed

// Creating the Vertical stepper motor object
A4988 Vertical_Stepper(spr,Dir_Vert,Step_Vert);
// CW --> +ve
//CCW --> -ve

int My_Address = 8; //Address of the slave (This arduino)

//Calibration Variables
volatile bool Calibration_Vert_Requested = false; //Ensures that when we reset this Arduino, calibration is not requested
volatile bool Calibration_Vert_Done = false; //Ensures that when we reset this Arduino, calibration has not been completed yet
volatile bool Calibration_Hori_Requested = false;
volatile bool Calibration_Hori_Done = false;

//Variables that contain values sent from master
volatile int Signal = 0;
volatile int Loop_Num = 0;

//Used to de-energize the orientation stepper motor
void Turn_Off_Orient_Motor() 
{
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
}

//Used to de-energize the horizontal stepper motor
void Turn_Off_Hori_Motor()
{
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}


void Receive_Event(int howMany) //Used to read values sent by the master.
{
  while (Wire.available()) 
  {
    Signal = Wire.read(); //Store values in the "signal" variable when values are sent over.
  }

  if (Signal == 9 ) //If the value sent is 9, then vertical calibration is requested
  {
    Calibration_Vert_Requested = true;
    Calibration_Vert_Done = false;
  }

  else if (Signal == 10) //If the value sent is 10, then a reset of the robot is requested -- > Reset Func/Stop Button was pressed.
  {
    Calibration_Vert_Requested = false;
    Calibration_Vert_Done = false;
    Calibration_Hori_Requested = false;
    Calibration_Hori_Done = false;
  }
  else if (Signal == 11) //If the value sent is 11, then horizontal calibration is requested.
  {
    Calibration_Hori_Requested = true;
    Calibration_Hori_Done = false;
  }

  else if (Signal>0 && Signal <= 8) //If the value sent is in between 0 and 8, then the number transmitted is the current loop number. 
  {
    Loop_Num = Signal;
  }
}

void Request_Event() //Used to send a value to the master when the master requests it. 
{
  if (Calibration_Vert_Done == true) //Checks if vertical calibration has been completed
  {
    Wire.write(1); // If vertical calibration is done, notify the master by sending a value of 1.
  }

  else if (Calibration_Hori_Done == true) //Checks if horizontal calibration has been completed.
  {
    Wire.write(2);  // If horizontal calibration is done, notify the master by sending a value of 2.
  }
  
  else
  {
    Wire.write(0);
  }
  
}

//Measures the horizontal distance from the robot to the vehicle.
void Horizontal_Distance_Measure ()
{
  digitalWrite(Trig_Vert,LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_Vert,HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Vert,LOW);
  Time_Hori = pulseIn(Echo_Vert,HIGH);
  Distance_Hori = Time_Hori * 0.0343 / 2.0;
}

//Performs Vertical Calibration 
void Calibration_Vert ()
{
  int Threshold_Vert = 25; //Once the sensor detects a distance greater than this threshold --> Vertical Calibration is completed 
  Count_Vert = 0;
  if (Orient != 0) //If the nozzles aren't parallel
  {
    Orient_Stepper.step(-512); //rotate 90 degrees CCW
    Orient = 0; //Indicate that the nozzles are now parallel
  }

  Horizontal_Distance_Measure(); //Measure the horizontal distance using ultrasonic sensor

  //This while loop will keep moving the rack up until the top of the vehicle is reached --> the ultrasonic sensor detects a distance greater than the threshold.
  while (Distance_Hori < Threshold_Vert) 
  {
    Vertical_Stepper.rotate(-50); //Move the Rack up
    Count_Vert++; 
    Horizontal_Distance_Measure();
  }
   
  Calibration_Vert_Requested = false; //Calibration is no longer requested since it has just been completed.
  Calibration_Vert_Done = true; //Indicate that vertical calibration has been completed.
}

void Vertical_Distance_Measure () 
{
  digitalWrite(Trig_Hori,LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_Hori,HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Hori,LOW);
  Time_Vert = pulseIn(Echo_Hori,HIGH);
  Distance_Vert = Time_Vert * 0.0343 / 2.0; 
}

void Calibration_Hori()
{
  int Threshold_Hori = 15;
  Count_Hori = 0;
  if (Orient != 1) //If the nozzles arent perpendicular with the ground
  {
    Orient_Stepper.step(512); //rotate 90 degrees CW such that they are perpendicular
    Orient = 1; //Indicate that the nozzles are now perpendicular.
    Turn_Off_Orient_Motor(); 
  }

  Vertical_Distance_Measure(); //Measure the vertical distance using ultrasonic sensor

   //This while loop will keep moving the horizontal rack forward until the edge of the vehicle roof is reached --> the ultrasonic sensor detects a distance less than the threshold.
  while (Distance_Vert > 15) 
  {
    Hori_Stepper.step(-500);
    Count_Hori++;
    Vertical_Distance_Measure();
  }

  Count_To_Roof = Count_Hori;
  Count_Hori = 0;

  //Now that the rack is at the edge of the vehicle roof, this while loop will determine the number of steps to the opposite end of the roof.
  while (Distance_Vert < Threshold_Hori)  //Keeps looping until the distance measured is greater than the threshold
  {
    Hori_Stepper.step(-500);
    Count_Hori++;
    Vertical_Distance_Measure(); 
  }

  //Now Count_Hori contains the required number of steps to oscillate about the car roof.

  Calibration_Hori_Requested = false; //Horizontal calibration is no longer requested since it has just been completed.
  Calibration_Hori_Done = true; //Indicate that horizontal calibration has been completed.
}

void setup()
{
  //Pin initialization for the Vertical Stepper Motor
  pinMode(Step_Vert,OUTPUT);
  pinMode(Dir_Vert,OUTPUT);
  digitalWrite(Dir_Vert,HIGH);

  // Pins initialization for the Vertical Ultrasonic
  pinMode(Trig_Vert, OUTPUT);
  pinMode(Echo_Vert, INPUT);

  // Pins initialization for the Horizontal Ultrasonic
  pinMode(Trig_Hori, OUTPUT);
  pinMode(Echo_Hori, INPUT);

  //Initialization for Master Slave Communication
  Wire.begin(My_Address);
  Wire.onReceive(Receive_Event);
  Wire.onRequest(Request_Event);

  Serial.begin(9600);

  //Starting the stepper motor object
  Vertical_Stepper.begin(RPM,stepMode);

  //Setting the speed for the horizontal and orientation stepper motors.
  Hori_Stepper.setSpeed(10);
  Orient_Stepper.setSpeed(10);
}


void loop() 
{
  if (Calibration_Vert_Requested)
  {
    Calibration_Vert(); //Undergo vertical calibration  
  }

  if (Calibration_Vert_Done == true) //If vertical calibration has been completed then oscillate the vertical rack up & down with respect to the determined vehicle height. 
  {
    while (Loop_Num<=2 && Calibration_Vert_Done == true) 
    {
      for (i=1; i<=Count_Vert; i++)
      {
        Vertical_Stepper.rotate(50);
      }
      
      for (i=1; i<=Count_Vert; i++)
      {
        Vertical_Stepper.rotate(-50);
      }

     }
     Calibration_Vert_Done = false;
  }

  if (Calibration_Hori_Requested)
  {
    Calibration_Hori(); //Undergo horizontal calibration  
  }

  if (Calibration_Hori_Done == true) //If horizontal calibration has been completed then oscillate the horizontal rack forward & backwards with respect to the determined roof width. 
  {
   
    while (Loop_Num >= 3 && Loop_Num <= 4 && Calibration_Hori_Done == true) 
    {  
      for (i=1; i<=Count_Hori; i++)
      {
        Hori_Stepper.step(500);
      }
      
      for (i=1; i<=Count_Hori; i++)
      {
        Hori_Stepper.step(-500);
      }
    
    }
    Calibration_Hori_Done = false;

    //This for loop is used to return the horizontal rack to the original position after completing the wash cycle. 
    for (i=1; i<=Count_Hori+Count_To_Roof; i++)
    {
      Hori_Stepper.step(500);
    }

    Turn_Off_Hori_Motor();

    //This for loop is used to return the vertical rack to the original position after completing the wash cycle. 
    for (i=1; i<=Count_Vert; i++)
    {
      Vertical_Stepper.rotate(50);
    }

    if (Orient != 0) //Ensures that the nozzles are parallel with the ground after completing the wash cycle.
    {
      Orient_Stepper.step(-512); //rotate 90degrees CCW
      Orient = 0;
    }
    Turn_Off_Orient_Motor();
  }
}