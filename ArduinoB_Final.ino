#include "A4988.h"
#include <Arduino.h>
#include <Stepper.h>
#include <Wire.h>
//Pins for Horiztonal Stepper Motor 
int Step_Vert = 9; 
int Dir_Vert = 8;
//Pins for the Verttical Ultrasonic Sensors
int Trig_Vert = 10;
int Echo_Vert = 11;

//Pins for the Horiztonal Ultrasonic Sensors
int Trig_Hori = 12;
int Echo_Hori = 13;

//Vertical Ultrasonic Variables
float Distance_Hori = 0;
float Time_Hori;
int Count_Vert=0; // Indicates how many steps it took the motor for the rack to reach the top of the car

//Horizontal Ultrasonic Variables 
int Count_Hori=0;
float Distance_Vert = 0;
float Time_Vert;
int Count_To_Roof; 

int Orient = 0; 
//if 0 --> Parallel with the ground. 
//if 1 --> Perpendircular with the ground 


int i;

//Horizontal Stepper
int StepsPerRev = 2048;
Stepper Hori_Stepper(StepsPerRev,4,6,5,7);
Stepper Orient_Stepper(StepsPerRev,A0,A2,A1,A3); //-ve is CW

//Constants For Vertical Stepper Motor
int spr = 200; //The number of steps per revolution
int stepMode = 1; //Defining the step mode - Using Full Step
int RPM = 400; //Defining the motor speed


int My_Address = 8; //Address of the slave (this device)

//Calibration Variables
volatile bool Calibration_Vert_Requested = false; //Esnures that when we reset this arduino that calibration is not requested
volatile bool Calibration_Vert_Done = false; //Esnusres that when we reset this arduino, calibration has not been completed yet
volatile bool Calibration_Hori_Requested = false;
volatile bool Calibration_Hori_Done = false;

//Variables that contain vaules sent from master
volatile int Signal = 0;
volatile int Loop_Num = 0;

int Target_Hori_Distance = 8;
int Hori_Distance_Tolerance = 2;

// Creating the Vertical stepper motor object
A4988 Vertical_Stepper(spr,Dir_Vert,Step_Vert);
// CW --> +ve
//CCW --> -ve

void Turn_Off_Orient_Motor()
{
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
}

void Turn_Off_Hori_Motor()
{
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}


void Receive_Event(int howMany) 
{
  while (Wire.available())
  {
    Signal = Wire.read();
  }

  if (Signal == 9 ) 
  {
    Calibration_Vert_Requested = true;
    Calibration_Vert_Done = false;
  }

  else if (Signal == 10) //Reset Func/Stop Button
  {
    Calibration_Vert_Requested = false;
    Calibration_Vert_Done = false;
    Calibration_Hori_Requested = false;
    Calibration_Hori_Done = false;
  }
  else if (Signal == 11)
  {
    Calibration_Hori_Requested = true;
    Calibration_Hori_Done = false;
  }
  else if (Signal>0 && Signal <= 8)
  {
    Loop_Num = Signal;
  }
}

void Request_Event()
{
  if (Calibration_Vert_Done == true) //Checks if calibration has been completed
  {
    Wire.write(1); // If calibration is done, notift the master
  }
  else if (Calibration_Hori_Done == true)
  {
    Wire.write(2);
  }
    else
  {
    Wire.write(0);
  }
  
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

  //Starting the Horizontal Stepper
  Hori_Stepper.setSpeed(10);
  Orient_Stepper.setSpeed(10);
}

//Perform the calibration 
void Calibration_Vert ()
{
  if (Orient != 0) //If the nozzles arent parrallel
  {
    Orient_Stepper.step(-512); //rotate 90degrees CCW
    Orient = 0;
  }
  int Threshold_Vert = 25;
  Count_Vert = 0;
  Horizontal_Distance_Measure();//Measuing the distance to the car 

  //This while loop will keep moving the rack up until the top of the car is reached
  while (Distance_Hori < Threshold_Vert) 
  {
    Vertical_Stepper.rotate(-50); //Move the Rack up
    Count_Vert++; 
    Horizontal_Distance_Measure();
   
  }
   
  Calibration_Vert_Requested = false; //Calibration is no longer requested
  Calibration_Vert_Done = true; //set the calibration variable back to since we finished calibration
}

//Measures the horiztonal distance from the robot to the car.
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

void Calibration_Hori()
{
    if (Orient != 1) //If the nozzles arent perpendicular with the ground
  {
    Orient_Stepper.step(512); //rotate 90degrees Cw to be perpendicular
    Orient = 1;
    Turn_Off_Orient_Motor();
    delay(300);
  }
  Count_Hori = 0;
  Vertical_Distance_Measure();

  //This While loop finds the start of the car roof.
  while (Distance_Vert > 15) //Keeps looping until the distance measured is less than 15cm
  {
    Serial.println(Distance_Vert);
    Hori_Stepper.step(-500);
    Count_Hori++;
    Vertical_Distance_Measure();
    
  }
  Count_To_Roof = Count_Hori;
  Count_Hori = 0;
  //Now that we are at the start of the car roof, this While loop finds the end of the car roof.
  while (Distance_Vert < 15)  //Keeps looping until the distance measured is greater than 15
  {
    
    Serial.println(Distance_Vert);
    Hori_Stepper.step(-500);
    Count_Hori++;
    Vertical_Distance_Measure(); 
   
  }

  Calibration_Hori_Requested = false; //Calibration is no longer requested
  Calibration_Hori_Done = true;
 
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

void loop() 
{
  if (Calibration_Vert_Requested)
  {
    Calibration_Vert(); //run the vertical calibration code 
  }

  if (Calibration_Vert_Done == true)
  {

    while (Loop_Num<=2 && Calibration_Vert_Done == true) // This while loop will chaneg to while the robot is still in the first stage (will have to recive a signal from the other arduino)
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
    Calibration_Hori();
  }

  if (Calibration_Hori_Done == true) 
  {
    //This while loop is used to oscillate the horizontal rack
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

    //This for loop is used to return the horizontal rack to the orginal position. 
    for (i=1; i<=Count_Hori+Count_To_Roof; i++)
    {
      Hori_Stepper.step(500);
    }
      Turn_Off_Hori_Motor();
    //This for loop is used to return the vertical rack to the orginal position. 
    for (i=1; i<=Count_Vert; i++)
    {
       Vertical_Stepper.rotate(50);
    }

    if (Orient != 0) //If the nozzles arent parrallel
    {
      
      Orient_Stepper.step(-512); //rotate 90degrees CCW
      Orient = 0;
    }
    Turn_Off_Orient_Motor();
  }
  
}