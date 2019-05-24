#include <Servo.h>
#include <math.h>
//#include <Filters.h>
#include "Wire.h"
#include "Adafruit_MLX90614.h"
#include <SoftwareSerial.h>
#include <SerialCommand.h>
#include <string.h>
#include <stdio.h>
using namespace std;

#define MAX_LENGTH 0.05          // [m] Maximum stroke of the motors
#define MAX_VEL 0.04             // [m/s] Maximum velocity of the motors; 0,04normal "CAPTURING HUMAN HAND KINEMATICS FOR OBJECT GRASPING AND MANIPULATION A Thesis by SHRAMANA GHOSH "
#define MAX_ACC 0.5              // [m/s^2] Maximum acceleration of the motors; 1 normal "CAPTURING HUMAN HAND KINEMATICS FOR OBJECT GRASPING AND MANIPULATION A Thesis by SHRAMANA GHOSH "
#define MIN_SIGNAL_DURATION 1000 // [microseconds]
#define MAX_SIGNAL_OFFSET 1000   // [microseconds]

SerialCommand sCmd;
// Physical contraints //
double mass = 2;                 // [kg]  Virtual mass of the admittance control scheme
double damping = 1000.0;         // [N*s/m] Virtual damping of the admittance control scheme
double g = 0.00981;              // [N/g] Gravity

float MIN_RANGE = 0;             // Minimum positon of the motors (can be modified from Unity3D)
float MAX_RANGE = 0.05 ;         //[m] Max positon of the motors is 50mm when full extended P 16-12-50-64-12-P (can be modified from Unity3D)
float MAX_FORCE = 0;             // Maximum force of the motors (can be modified from Unity3D)

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
  double Threshold_Position = 0.2 * (MAX_RANGE - MIN_RANGE); // [] Relative threshold position from lower and upper end of the range of the motor
};

struct s_Assistive_Force Assistive_Force[4];            // Assistive forces for all joints
struct s_Kinetics Kinetics[4];                          // Kinetic parameters for all joints
Servo Motor[4];                                         // Handle to the motors
double Offset_Force_Sensor[4] = {689, 671, 598, 632};   // Measured offsets of the force sensors
double Calib_Force_Sensor[4] = { -1.47, -1.45, -1.54, -1.54}; // Measured calibration factors of the force sensors
double Force_Force_Sensor[4];                           // Measured force from the force sensors

// Sensor box variables //
float fMAX_Assistance_Force = 0;       // [N] Maximum assistance force, set by the potentiometer
float fGSR_Value = 0;                 // [] Value of the GSR
int sensorValue = 0;
float fTemperature_Value = 0;         // [C] Measured Temperature

// Communication variables //
float timestep = 0;                   // [s] Time for one iteration of the main loop
float feedbackFreq = 1;               // [1/s] Data is sent to the PC this many times per seconds; Unity reads once per second
float feedbackTime = 0;               // [s] Time since the last feedback was sent to the PC
double start = 0;                     // [s] Measures the time instant for the start of each iteration
double stp = 0;                       // [s] Measures the time instand for the end of each iteration
String outString;                     // [] The string to be sent to the PC
String printout;                      // [] The string to be sent to the Console

//TEST VALUES-------------------------------------------------------------------
int incomingByte = 0;   // for incoming serial data
float test = 99;// TEST VALUE SERIAL READ
//------------------------------------------------------------------------------

Adafruit_MLX90614 mlx;                // Set up IR Thermometer

// SETUP ROUTINE //
void setup() {
  Serial.begin(9600);                 // Start Serial Communication and set Analog reference
  //-------------
  while (!Serial);
  sCmd.addCommand("PING", pingHandler);
  //-------------
  mlx.begin();                        // Start IR Thermometer readouts
  analogReference(INTERNAL2V56);      // Set the internal reference of the Arduino to 2.56V (necessary for the force sensors)
  setValues();                        // Obtain Settings for all the joints from Unity3D

  //Attach servos
  for (int iJoints = 0; iJoints < 4; iJoints++) {
    Motor[iJoints].attach(Motor_Pin[iJoints]);
    Motor[iJoints].writeMicroseconds(1000);
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
  //---------------
  //Serial.println("here");
  if (Serial.available() > 0)
    Serial.println("next");
    sCmd.readSerial();
  //-----------
  timestep = (stp - start) / 1000000; // [s] Calculate the timestep for the last iteration
  feedbackTime += timestep;       // [s]Sum up the time since the last time data was sent to the PC
  start = micros();               // [mus] Measure the current time

  // Read force sensors
  for (int iJoints = 0; iJoints < 4; iJoints++) {

    Force_Force_Sensor[iJoints] = (analogRead(Force_Sensor_Pin[iJoints])
                                   - Offset_Force_Sensor[iJoints]) * Calib_Force_Sensor[iJoints] * g;
  }

  //    // Generate an assistive force; here, the force is the same for each finger       // Generate an assistive force; here, the force is the same for each finger
  //    Generate_Assistive_Force(Kinetics[0].x, Kinetics[1].x, 0);          Generate_Assistive_Force(Kinetics[0].x, Kinetics[1].x, 0);
  //    Assistive_Force[1] = Assistive_Force[0];      Assistive_Force[1] = Assistive_Force[0];
  //    Generate_Assistive_Force(Kinetics[2].x, Kinetics[3].x, 2);          Generate_Assistive_Force(Kinetics[2].x, Kinetics[3].x, 2);
  //    Assistive_Force[3] = Assistive_Force[2];      Assistive_Force[3] = Assistive_Force[2];

  // Generate an assistive force; here, the force is the same for each finger
  Generate_Assistive_Force(Kinetics[0].x, 0);
  Generate_Assistive_Force(Kinetics[1].x, 1);
  Generate_Assistive_Force(Kinetics[2].x, 2);
  Generate_Assistive_Force(Kinetics[3].x, 3);

  // Run the admittance control scheme; here, the forces on one finger are added up, which makes the motors move synchron
  // Admittance_Control(Force_Force_Sensor[0] + Force_Force_Sensor[1], Assistive_Force[0].Force_Level, 0);
  // Admittance_Control(Force_Force_Sensor[0] + Force_Force_Sensor[1], Assistive_Force[1].Force_Level, 1);
  Admittance_Control(Force_Force_Sensor[0] , Assistive_Force[0].Force_Level, 0);
  Admittance_Control(Force_Force_Sensor[1], Assistive_Force[1].Force_Level, 1);
  //   Admittance_Control(Force_Force_Sensor[2] + Force_Force_Sensor[3], Assistive_Force[2].Force_Level, 2);
  //   Admittance_Control(Force_Force_Sensor[2] + Force_Force_Sensor[3], Assistive_Force[3].Force_Level, 3);
  Admittance_Control(Force_Force_Sensor[2], Assistive_Force[2].Force_Level, 2);
  Admittance_Control(Force_Force_Sensor[3], Assistive_Force[3].Force_Level, 3);
  // Whenever Feedbacktime = 1/FeedbackFrequency read the sensor box and send data to the PC
  if (feedbackTime > (1 / feedbackFreq)) {
    readSensorBox();          // Read the sensors from the sensor box
    outputData();             // Send data for Unity
    //Print_Data2Console();     // Print the data to console on Laptop
    feedbackTime = 0;
  }

  stp = micros();               // Measure the current time
}
//------------------------------------------------------------------------------------------------
// ------------------------------FUNCTIONS--------------------------------------------------------
//------------------------------------------------------------------------------------------------
void pingHandler (const char *command) {
  Serial.println("PONG");
}

// FUNCTION : After starting the serial connection read the settings done by therapeut from Unity3D
void setValues() {
  //  //char rc[35];
  //  //rc = Serial.readString();
  ////----std::string::c_str().
  ////  std::string rc = Serial.readString();
  ////  char *tempChars = new char[rc.length() + 1];
  ////  strcpy(tempChars, rc.c_str());
  //// or in modern c++
  ////  std::vector<char> tempChars(rc.c_str(), rc.c_str() + rc.size() + 1);
  ////-----
  //  char rc;
  //  rc = Serial.read();
  //  char tempChars[35];        // temporary array for use by strtok() function
  //  // split the data into its parts
  //  strcpy(tempChars, rc);
  ////----
  //  char *strtokIndx; // this is used by strtok() as an index
  //
  //  strtokIndx = strtok(tempChars,",");      // get the first part - the string
  //  feedbackFreq = atof(strtokIndx); // copy it to feedbackFreq
  //
  //  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  //  MIN_RANGE = atof(strtokIndx)/90*0.05;     // convert this part to a float & 90deg to 0.05m on the motors
  //
  //  strtokIndx = strtok(NULL, ",");
  //  MAX_RANGE = atof(strtokIndx)/90*0.05;     // convert this part to a float & 90deg to 0.05m on the motors
  //
  //  strtokIndx = strtok(NULL, ",");
  //  MAX_FORCE = atof(strtokIndx);     // convert this part to a float
  //   ------------------------------------------
  Serial.println("I am in ");
  test = Serial.read();
  // say what you got:
  Serial.print("I received: ");
  Serial.println(test, DEC); // -1 means no serial connection established
  //test = Serial.parseFloat();
  
  // serial connection is on:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
  }

  while (Serial.available() > 0) {
    Serial.println("I am listning");
    MAX_RANGE = Serial.parseFloat();
  }
  //  while(Serial.available()<=0){}      // Do nothing until  serial connection is established
  //
  //  // Read general Settings
  //  feedbackFreq = Serial.parseFloat(); // 1 dataset from unity to arduino
  //  MIN_RANGE = Serial.parseFloat(); // 0-90 deg
  //  MAX_RANGE = Serial.parseFloat(); // 0-90 deg
  //  MAX_FORCE = Serial.parseFloat();
  //  Serial.flush();
}
//--END OF CONSTRUCTION SITE---------------------

// FUNCTION : Reads the sensors from the sensor box
void readSensorBox() {
  int scale_force = 200,
  // Temperature
  fTemperature_Value = mlx.readObjectTempC();
  //0-1000Ohm Assistance Force / Potentiometer  changed from 100 to 200 so F assistive max is 5N
  fMAX_Assistance_Force = analogRead(Potentiometer_Pin) / scale_force;
  // adapts the damping to the additional force in the system
  damping = 1000 + analogRead(Potentiometer_Pin) * scale_force / 200;
  // GSR - sweat sensor - resistance
  //fGSR_Value = analogRead(GSR_Pin);
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
  if (Force_Sensor < 1.3) {
    Force = Force_Sensor; //0;
  } else {
    Force = Force_Sensor;
  }

  Kinetics[num].ddx = 1 / mass * (Force + Assistive_Force - damping * Kinetics[num].dx);

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
  if (Kinetics[num].x < MIN_RANGE) Kinetics[num].x = MIN_RANGE;
  if (Kinetics[num].x > MAX_RANGE) Kinetics[num].x = MAX_RANGE;

  Motor[num].writeMicroseconds( Kinetics[num].x / MAX_LENGTH * MAX_SIGNAL_OFFSET + MIN_SIGNAL_DURATION);
}

// FUNCTION : Generates an assistive force, when position thresholds are reached by the operator of the exoskeleton
void Generate_Assistive_Force(double x_a, int num)
{
  if (Force_Force_Sensor[num] > 0.5)
    Assistive_Force[num].Force_Level = fMAX_Assistance_Force * 3;
  if (Force_Force_Sensor[num] < -0.5)
    Assistive_Force[num].Force_Level = -fMAX_Assistance_Force * 3;
  //     if( x_a > (MIN_RANGE+Assistive_Force[num].Threshold_Position) && Assistive_Force[num].Direction == 0.5 )
  //        Assistive_Force[num].Direction = 1;
  //
  //     if( x_a < (MAX_RANGE-Assistive_Force[num].Threshold_Position) && Assistive_Force[num].Direction == -0.5 )
  //        Assistive_Force[num].Direction = -1;
  //
  //     if(x_a > (MAX_RANGE-Assistive_Force[num].Threshold_Position) && Assistive_Force[num].Direction == 1)
  //        Assistive_Force[num].Direction = -0.5;
  //
  //     if (x_a < (MIN_RANGE+Assistive_Force[num].Threshold_Position) && Assistive_Force[num].Direction == -1)
  //        Assistive_Force[num].Direction = 0.5;
  //
  //     if(Assistive_Force[num].Force_Level < fMAX_Assistance_Force  && Assistive_Force[num].Direction ==  1)
  //        Assistive_Force[num].Force_Level = fMAX_Assistance_Force;//Assistive_Force[num].Force_Level + 0.001;
  //
  //     if(Assistive_Force[num].Force_Level > -fMAX_Assistance_Force && Assistive_Force[num].Direction == -1)
  //        Assistive_Force[num].Force_Level = -fMAX_Assistance_Force;//Assistive_Force[num].Force_Level - 0.001;
  //
  //     if(Assistive_Force[num].Direction == 0.5 || Assistive_Force[num].Direction == -0.5)
  //        Assistive_Force[num].Force_Level = 0;
}

// FUNCTION : Sends data via the serial port
void outputData() {
  printout = String();
  printout = fTemperature_Value;
  printout += ",";
  printout += "'";
  printout += test;
  printout += "'";
  printout += fGSR_Value;
  printout += ",";
  printout += MAX_RANGE; //fMAX_Assistance_Force;
  printout += ",";
  printout += Force_Force_Sensor[0];
  printout += ",";
  printout += Force_Force_Sensor[1];
  printout += ",";
  printout += Force_Force_Sensor[2];
  printout += ",";
  printout += Force_Force_Sensor[3];
  printout += ",";
  Serial.flush();
  Serial.println(printout);
}

// FUNCTION : Prints data to console
void Print_Data2Console()
{
  printout = "F1[N]: ";
  printout += Force_Force_Sensor[0];
  printout += " , F2[N]: ";
  printout += Force_Force_Sensor[1];
  //  printout += " , F3: ";
  //  printout += Force_Force_Sensor[2];
  //  printout += " , F4: ";
  //  printout += Force_Force_Sensor[3];
  printout += " , FAssist[N] :";
  printout += Assistive_Force[0].Force_Level;
  printout += ", FassDir:";
  printout += Assistive_Force[0].Direction;
  //  printout += " , GSR:";
  //  printout += fGSR_Value;
  //  printout += " , T[C]:";
  //  printout += fTemperature_Value;
  printout += " , dt[ms]:";
  printout += timestep * 1000;
  printout += " , tloop[ms] ";
  printout += feedbackTime;
  // printout += " , Acc:";
  //printout += Kinetics[0].ddx;
  // printout += ", Vel:";
  // printout += Kinetics[0].dx;
  printout += ", X[mm]:";
  printout += Kinetics[0].x * 100;
  Serial.flush();
  Serial.println(printout);
}
