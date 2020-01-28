#include <MegunoLink.h>
#include <CommandHandler.h>
#include <TCPCommandHandler.h>
#include <ArduinoTimer.h>
#include <CircularBuffer.h>
#include <EEPROMStore.h>
#include <Filter.h>

//#include <EEPROMex.h>
//#include <EEPROMVar.h>

#include <Servo.h>
#include <math.h>
//#include <EEPROMEx.h> //EEPROM write read
//#include <Filters.h>
//#include <Wire.h>
//#include <Adafruit_MLX90614.h>
#include "Wire.h"
#include "Adafruit_MLX90614.h"
#include <SoftwareSerial.h>
#include <SerialCommand.h>
//#include "CommandHandler.h"
#include <string.h>
#include <stdio.h>
using namespace std;

#define MAX_LENGTH 0.05          // [m] Maximum stroke of the motors
#define MAX_VEL 0.04             // [m/s] Maximum velocity of the motors; 0,04normal "CAPTURING HUMAN HAND KINEMATICS FOR OBJECT GRASPING AND MANIPULATION A Thesis by SHRAMANA GHOSH "
#define MAX_ACC 0.5              // [m/s^2] Maximum acceleration of the motors; 1 normal "CAPTURING HUMAN HAND KINEMATICS FOR OBJECT GRASPING AND MANIPULATION A Thesis by SHRAMANA GHOSH "
#define MIN_SIGNAL_DURATION 1000 // [microseconds]
#define MAX_SIGNAL_OFFSET 1000   // [microseconds]

//bugfix for the reset of the arduino at every game start from Unity
int addr0 = 0;
double value;

SerialCommand sCmd;
CommandHandler<> SerialCommandHandler;
// Physical contraints //
double mass = 1;                 //1 [kg]  Virtual mass of the admittance control scheme (otherwise motor drives continuously back, does not hold a position)
double damping = 100.0;         //100 [N*s/m] Virtual damping of the admittance control scheme
double g = 0.00981;              // [N/g] Gravity

// motor 1 and 4 (0 and 3 here)
float MCP_MIN_RANGE = 0;             // Minimum positon of the motors (can be modified from Unity3D)
float MCP_MAX_RANGE = 0.05 ;         //[m] Max positon of the motors is 50mm when full extended P 16-12-50-64-12-P (can be modified from Unity3D)
// mototr 2 and 3 (1 and 2 here)
float PIP_MIN_RANGE = 0;             // Minimum positon of the motors (can be modified from Unity3D)
float PIP_MAX_RANGE = 0.05 ;         //[m] Max positon of the motors is 50mm when full extended P 16-12-50-64-12-P (can be modified from Unity3D)
float control_mode = 0; //determins control mode, 0 for admittance control, 1 for sine wave feedforward control
float timeslot = 0;
float MAX_FORCE = 0;             // Maximum force of the motors
float stepsize = 0.001;
float foo[4] = {stepsize, stepsize, stepsize, stepsize}; // step size motor movement per loop

// Pins on the Arduino //
int Motor_Pin[4] = {2, 3, 4, 5};                        // Arduino Pins for the motors
int Force_Sensor_Pin[4] = {A0, A1, A3, A2};             // Arduino Pins for the force sensors
int Potentiometer_Pin = A4;           // The analog input pin reading the linear potentiometer
int GSR_Pin = A5;                     // The analog input pin reading the GSR sensor

// Motor and force varibles //
struct s_Kinetics                // Struct that stores kinetic data of every joint
{
  double x = 0;                  // [m] Position
  double dx = 0;                 // [m/s] Velocity
  double ddx = 0;                // [m/s^2] Acceleration
};

struct s_Assistive_Force         // Struct that stores information about the assistive force
{
  double Force_Level = 0;        // [N] Current force level
  double Direction = 0.5;        // [] Direction flag of the assistive force
  double Threshold_Position = 0.2 * (MCP_MAX_RANGE - MCP_MIN_RANGE); // [] Relative threshold position from lower and upper end of the range of the motor
};

struct s_Assistive_Force Assistive_Force[4];            // Assistive forces for all joints
struct s_Kinetics Kinetics[4];                          // Kinetic parameters for all joints
Servo Motor[4];                                         // Handle to the motors
double Offset_Force_Sensor[4] = {689, 671, 598, 632};   // Measured offsets of the force sensors
double Calib_Force_Sensor[4] = { -1.47, -1.45, -1.54, -1.54}; // Measured calibration factors of the force sensors
double Force_Force_Sensor[4];                           // Measured force from the force sensors
double Force_Unfilt_Sensor[4];                          // unfiltered force data for the output

// Sensor box variables //
float fMAX_Assistance_Force = 0;       // [N] Maximum assistance force, set by the potentiometer
float fGSR_Value = 0;                 // [] Value of the GSR
int sensorValue = 0;
// low pass filter variables
float EMA_a = 0.4;    //initialization of EMA alpha (cutoff-frequency)
float EMA_S[4] = {0, 0, 0, 0}; ;        //initialization of EMA S
float fTemperature_Value = 0;         // [C] Measured Temperature

// Communication variables //
float timestep = 0;                   // [s] Time for one iteration of the main loop
float feedbackFreq = 1;               // [1/s] Data is sent to the PC this many times per seconds; Unity reads once per second
float feedbackTime = 0;               // [s] Time since the last feedback was sent to the PC
double start = 0;                     // [s] Measures the time instant for the start of each iteration
double stp = 0;                       // [s] Measures the time instand for the end of each iteration
String outString;                     // [] The string to be sent to the PC
String printout;                      // [] The string to be sent to the Console

//Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_MLX90614 mlx;                // Set up IR Thermometer

// SETUP ROUTINE //
void setup() {
  Serial.begin(9600);                 // Start Serial Communication and set Analog reference
  mlx.begin();                        // Start IR Thermometer readouts
  analogReference(INTERNAL2V56);      // Set the internal reference of the Arduino to 2.56V (necessary for the force sensors)
  readUnityInput(); //setValues();                        // Obtain Settings for all the joints from Unity3D

  //Attach servos
  for (int iJoints = 0; iJoints < 4; iJoints++) {
    Motor[iJoints].attach(Motor_Pin[iJoints]);
    Motor[iJoints].writeMicroseconds(1000); // drive the motors to medium position
  }

  // Calculate offset of force sensors
  for (int i = 0; i < 5000; i++) {
    for (int iJoints = 0; iJoints < 4; iJoints++) {
      Offset_Force_Sensor[iJoints] += analogRead(Force_Sensor_Pin[iJoints]);
    }
  }

  // Take the mean value of the previous measurements to obtain an adequate offset for the force sensors
  for (int iJoints = 0; iJoints < 4; iJoints++) {
    Offset_Force_Sensor[iJoints] /= 5000;
  }
}



//// MAIN LOOP ////
void loop() {
  if (Serial.available() > 0)
  timestep = (stp - start) / 1000000; // [s] Calculate the timestep for the last iteration
  //Serial.print(timestep); Serial.println();

  feedbackTime += timestep;       // [s]Sum up the time since the last time data was sent to the PC
  start = micros();               // [mus] Measure the current time

  // Read force sensors
  for (int iJoints = 0; iJoints < 4; iJoints++) {
    double filt = 0.9990;  // Set only between 1.0 and 0.0.  Higher value filters more. Set to 0.0 to get original, unfiltered version.
    double f = (analogRead(Force_Sensor_Pin[iJoints])
                                   - Offset_Force_Sensor[iJoints]) * Calib_Force_Sensor[iJoints] * g;
    Force_Unfilt_Sensor[iJoints] = Force_Force_Sensor[iJoints];
    Force_Force_Sensor[iJoints] = filt*Force_Force_Sensor[iJoints] + (1.0-filt)*f;          
    EMA_S[iJoints] = (EMA_a*Force_Force_Sensor[iJoints]) + ((1-EMA_a)*EMA_S[iJoints]);  //run the EMA 
  }

  // Generate an assistive force
  Generate_Assistive_Force(Kinetics[0].x, 0);
  Generate_Assistive_Force(Kinetics[1].x, 1);
  Generate_Assistive_Force(Kinetics[2].x, 2);
  Generate_Assistive_Force(Kinetics[3].x, 3);

  if (control_mode == 0){
    // Run the admittance control scheme; here, the forces on one finger are added up, which makes the motors move synchron
    Admittance_Control(EMA_S[0], Assistive_Force[0].Force_Level, 0);
    Admittance_Control(EMA_S[1], Assistive_Force[1].Force_Level, 1);
    Admittance_Control(EMA_S[2], Assistive_Force[2].Force_Level, 2);
    Admittance_Control(EMA_S[3], Assistive_Force[3].Force_Level, 3);
  }
  else{
    timeslot = micros();
    Sin_feedforward(Assistive_Force[0].Force_Level, 0, MCP_MIN_RANGE, MCP_MAX_RANGE, timeslot);
    Sin_feedforward(Assistive_Force[1].Force_Level, 1, PIP_MIN_RANGE, PIP_MAX_RANGE, timeslot);
    Sin_feedforward(Assistive_Force[2].Force_Level, 2, PIP_MIN_RANGE, PIP_MAX_RANGE, timeslot);
    Sin_feedforward(Assistive_Force[3].Force_Level, 3, MCP_MIN_RANGE, MCP_MAX_RANGE, timeslot);
  }
 
  // Whenever Feedbacktime = 1/FeedbackFrequency read the sensor box and send data to the PC
  if (feedbackTime > (1 / feedbackFreq)) {
    readSensorBox();          // Read the sensors from the sensor box
    outputData();             // Send data for Unity
    readUnityInput(); //read min max values from Unity
    feedbackTime = 0;
  }
  stp = micros();               // Measure the current time
}
//------------------------------------------------------------------------------------------------
// ------------------------------FUNCTIONS--------------------------------------------------------
//------------------------------------------------------------------------------------------------
void readUnityInput(){
       if(Serial.available()>0){  
            MCP_MIN_RANGE = (Serial.parseFloat()+10)/100*MAX_LENGTH;
            MCP_MAX_RANGE = (Serial.parseFloat()+10)/100*MAX_LENGTH;
            PIP_MIN_RANGE = (Serial.parseFloat()+10)/100*MAX_LENGTH;
            PIP_MAX_RANGE = (Serial.parseFloat()+10)/100*MAX_LENGTH;
            control_mode = Serial.parseFloat();
            Serial.flush();
       }
     }


// FUNCTION : Reads the sensors from the sensor box
void readSensorBox() {
  int scale_force = 200;
  // Temperature
  //Serial.print(mlx.readObjectTempC());
  fTemperature_Value = mlx.readObjectTempC();
  
  //0-1000Ohm Assistance Force / Potentiometer  changed from 100 to 200 so F assistive max is 15N
  fMAX_Assistance_Force = analogRead(Potentiometer_Pin) / scale_force;
  // adapts the damping to the additional force in the system
  damping = 1000 + (analogRead(Potentiometer_Pin)/ scale_force) * 1000;
  //vs. 1000 + analogRead(Potentiometer_Pin)* scale_force / 10; poti high fine poti 0 too much damping

  // GSR - sweat sensor - resistance
  long sum = 0;
  for (int i = 0; i < 10; i++)    //Average the 10 measurements to remove the glitch
  {
    sensorValue = analogRead(GSR_Pin);
    sum += sensorValue;
  }
  fGSR_Value = sum / 10;
  //fGSR_Value = ((1024+2*(fGSR_Value-(992-512)))*10000)/(512-(fGSR_Value-(992-512)));
  //Human Resistance = ((1024+2*Serial_Port_Reading)*10000)/(512-Serial_Port_Reading),
  //unit is ohm, Serial_Port_Reading is the value display on Serial Port(between 0~1023)
}

// FUNCTION : Runs an admittance control scheme and sets the respective motor position
void Admittance_Control(double Force_Sensor, double Assistive_Force, int num)
{ double Force;
  // Admittance control basic equation: F = m*ddx + d*dx
  // smooth sigmoid 'step function' for cutting off low forces
  Force = Force_Sensor + Assistive_Force;
  Force = Force / (1. + exp(-10*(fabs(Force_Sensor) - 1)));

  Kinetics[num].ddx = 1 / mass * (Force  - damping * Kinetics[num].dx);
//Kinetics[num].ddx = 1 / mass * (Force + Assistive_Force - damping * Kinetics[num].dx);

  // Delimit the acceleration
  if (Kinetics[num].ddx < -MAX_ACC) Kinetics[num].ddx = -MAX_ACC;
  if (Kinetics[num].ddx > MAX_ACC) Kinetics[num].ddx = MAX_ACC;

  // Then, the obtained acceleration ddx has to be integrated twice in order to obtain x.
  Kinetics[num].dx += Kinetics[num].ddx * timestep;

  // Delimit the velocity
  if (Kinetics[num].dx < -MAX_VEL) Kinetics[num].dx = -MAX_VEL;
  if (Kinetics[num].dx > MAX_VEL) Kinetics[num].dx = MAX_VEL;

  Kinetics[num].x = Kinetics[num].x + Kinetics[num].dx * timestep + 0.5 * Kinetics[num].ddx * timestep * timestep;

  // Delimit the positon
  if (num == 0 || num==3){ //MCP
    if (Kinetics[num].x < MCP_MIN_RANGE) Kinetics[num].x = MCP_MIN_RANGE;
    if (Kinetics[num].x > MCP_MAX_RANGE) Kinetics[num].x = MCP_MAX_RANGE;
  }
  else{//PIP and DIP
    if (Kinetics[num].x < PIP_MIN_RANGE) Kinetics[num].x = PIP_MIN_RANGE;
    if (Kinetics[num].x > PIP_MAX_RANGE) Kinetics[num].x = PIP_MAX_RANGE;
    }

  Motor[num].writeMicroseconds( Kinetics[num].x / MAX_LENGTH * MAX_SIGNAL_OFFSET + MIN_SIGNAL_DURATION);
}


// FUNCTION : Runs a feedforward control sin wave to set the motor position
void Sin_feedforward(double Assistive_Force, int num, float mini, float maxi, float timeslot)
{ float omega=6.28;
  // Y= Amplitude*sin((2*pi*X/Wavelength)+PhaseShift) + Baseline
  //// position_x   = range_x    *sin(angular_frequency dep. on F     *omega*milli/1000   )+offset_x;
  ////Kinetics[num].x = (maxi-mini)*sin(omega*millis()*0.0001/(Assistive_Force+1))+mini+(maxi-mini)/2;
  //Kinetics[num].x = (maxi-mini)*sin(omega*timeslot*0.0000001/(Assistive_Force+1))+mini+(maxi-mini)/2; // also WORKING
  
  if  (Kinetics[num].x < mini)
  {foo[num] = stepsize;}
  else
  {if (Kinetics[num].x > maxi)
  {foo[num] = -stepsize;}
  }
  Kinetics[num].x += foo[num];
  // write position to motor
  Motor[num].writeMicroseconds( Kinetics[num].x / MAX_LENGTH * MAX_SIGNAL_OFFSET + MIN_SIGNAL_DURATION);
}

// FUNCTION : Generates an assistive force, when position thresholds are reached by the operator of the exoskeleton
void Generate_Assistive_Force(double x_a, int num)
{ 
  if (Force_Force_Sensor[num] > 0.0)
    Assistive_Force[num].Force_Level = fMAX_Assistance_Force * 10; //3,  with 10 the reaction of the system is much better and quicker 
  if (Force_Force_Sensor[num] < -0.0)
    Assistive_Force[num].Force_Level = -fMAX_Assistance_Force * 10;
}

// FUNCTION : Sends data via the serial port
void outputData() {
  printout = String();
  printout = fTemperature_Value;
  printout += ",";
  printout += fGSR_Value;
  printout += ",";
  printout += fMAX_Assistance_Force;
  printout += ",";
  printout += Force_Unfilt_Sensor[0];
  printout += ",";
  printout += Force_Unfilt_Sensor[1];
  printout += ",";
  printout += Force_Unfilt_Sensor[2];
  printout += ",";
  printout += Force_Unfilt_Sensor[3];
  printout += ",";
  //printout += Assistive_Force[0].Force_Level; //test output assistance force
  Serial.flush();
  Serial.println(printout);
}
