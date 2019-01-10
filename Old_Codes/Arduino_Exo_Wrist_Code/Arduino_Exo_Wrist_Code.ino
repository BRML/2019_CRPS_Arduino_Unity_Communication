///////////////////////////////////////////////////////
//			Technische Universität München			         //
//					Januar 2019						                   //
// Daniel Angloher / Negar Namdarian / Markus Kühne  //
//		 Exoskeleton control scheme and Unity3D        //
//                communication						           //
//                                                   //
// Description: Runs an admittance control scheme    //
// with optional directed force support. Settings    //
// are received over serial connection from unity    //
// and data outputs are send to unity for post       //
// processing.                                       //
///////////////////////////////////////////////////////

///// HARDWARE SETUP /////
#define num_Joints 3    
//////////////////////////

#include <Servo.h>
#include <math.h>
#include "Adafruit_MLX90614.h"

#define pi 3.14159265
#define g  0.00981    // [N/g]

// Define Pins of the Arduino
int potentiometerPin = A4;
int GSRPin = A5;
int CalibrationPin = A6;

// Define Joints and their variables as a struct. Note: One joint can be moved by up to two motors
struct sJoint{
  Servo motor[2];
  int isOn;
  int actuatorPin[2];
  int forcePin[2];
  float sensor_offset[2] = {500, 500};
  int   sensor_dir[2]; //direction of the attached sensor
  float mass;          //[kg]  Virtual mass of the admittance control scheme
  float damping;       //[N*s/(deg or m)] Virtual damping of the admittance control scheme
  float force = 0;     //[N] Force, measured at the force sensor
  float ddx = 0;       //[(deg or m)/s2] Acceleration of the virtual mass
  float dx = 0;        //[(deg or m)/s] Velocity of the virtual mass
  float x;             //[deg or m] position of the motor
  float MIN_Pos;       //[deg or m] min position of the motor
  float MAX_Pos;       //[deg or m] max position of the motor
  float MAX_Force;
  float MIN_Vel;
  float assistance_force = 0;
  float desired_pos;
  float assistance_speed;
  float radius = 1;
  int   DOUBLE_actuated = 0;              // TWO motors at one joint or finger
  int   NONLINEAR_DOUBLE_actuated = 0;    // TWO motors at one joint or finger, but moving non-linearly          
  
  //Average variables for output to Unity3D
  float AverageForce = 0;
  float Average_dx = 0;
  float Average_ddx = 0;
  float AverageAssistanceForce = 0;
};

struct sJoint joints[num_Joints];

// Hardware settings
float stroke_length = 0.1;        // [m]
float min_signal_duration = 1000; // [microseconds]
float max_signal_offset = 1000;   // [microseconds]
float calib_fact = 1.5;           // []
Adafruit_MLX90614 mlx;            // Handle to the IR Thermometer

// Communication variables
int counter = 0;
double stp = 0;
double start = 0;
float feedbackFreq = 1;
float timestep = 0;
String outString;
String s1;
int force_raw = 0;
float feedbackTime = 0;
float potentiometer_MAX_Force = 0; // assistance force scalable with slider
float assist_scale;                // maximum assistance force when slider is at 100%
int controllerAttached = 1;
int runCalib = 0;

// Measurement variables
float fTemperature_Value = 0;
float fGSR_Value = 0;
float AverageTemp = 0;
float AverageGSR = 0;

// SETUP ROUTINE //
void setup()
{ 
  Serial.begin(9600);             // Start the serial port
  while(!Serial);                 // Wait for serial port to run
  analogReference(INTERNAL2V56);  // Change internal voltage reference for force sensors
  mlx.begin();                    // Start the IR Thermometer
  setValues();                    // Obtain Settings for all the joints from Unity3D
  
  for(int i = 0; i < num_Joints; i++){  // Attach servos and bring them into starting position
    joints[i].motor[0].attach(joints[i].actuatorPin[0]);
    joints[i].motor[0].writeMicroseconds(1500);
    
    if(joints[i].DOUBLE_actuated){
      joints[i].motor[1].attach(joints[i].actuatorPin[1]);
      joints[i].motor[1].writeMicroseconds(1500);
    }
  }  
  
  pinMode(CalibrationPin,INPUT);  // Set the calibration pin as simple input pin
  
  // wait for patient to put on exoskeleton
  delay(10000);
}

//// MAIN LOOP ////
void loop()
{
  timestep = (stp-start)/1000000;
  feedbackTime += timestep;
  start = micros();
 
  counter += 1;
  
  if(controllerAttached){
    runCalib = digitalRead(CalibrationPin);                                         // Check if calibration button was pushed 
    potentiometer_MAX_Force=-(analogRead(potentiometerPin)/1023.0-1)*assist_scale;  // Read Potentiometer Value 
    fTemperature_Value = mlx.readObjectTempC();                                     // Read Temperature
    fGSR_Value = analogRead(GSRPin);                                                // Read GSR Value
  }

  if(runCalib){
    runCalibration();
    runCalib = 0;  
    start = micros();
  }
  
  readForces();           // Read out forces from the force sensors 
                          // Calculate Assitance Force
  runAdmittance();        // Run the Admittance Controller
  sumMeasurements();      // Sums up the measurements from the read data

  if (feedbackTime>(1/feedbackFreq)){
    outputData();
    resetAnalysis();
  }
  stp = micros();
}

// FUNCTION : After starting the serial connection all the settings are read in from Unity3D
void setValues(){
    
  while(Serial.available()<=0){}      // Wait for the serial connection to become available

  // Read general Settings
  feedbackFreq = Serial.parseFloat();
  controllerAttached = Serial.parseInt();
  runCalib = Serial.parseInt();
  assist_scale = Serial.parseFloat();

  // Read joints specific settings
  for(int i = 0; i < num_Joints; i++){
    joints[i].isOn = Serial.parseInt();
    joints[i].mass = Serial.parseFloat(); 
    joints[i].damping = Serial.parseFloat();  
    joints[i].MIN_Pos = Serial.parseFloat();
    joints[i].MAX_Pos = Serial.parseFloat();
    joints[i].sensor_dir[0] = Serial.parseInt();
    joints[i].MAX_Force = Serial.parseFloat();
    joints[i].MIN_Vel = Serial.parseFloat();
    joints[i].assistance_speed = Serial.parseFloat();
    joints[i].DOUBLE_actuated = Serial.parseInt();
    joints[i].NONLINEAR_DOUBLE_actuated = Serial.parseInt();            

    if (joints[i].DOUBLE_actuated){
      joints[i].radius = Serial.parseFloat();
      joints[i].sensor_dir[1] = Serial.parseInt();
    }
  }
  Serial.flush();
}

// FUNCTION : Calculates and writes the motor positions using the measured forces as well as the calculated force assistance
void runAdmittance(){
  for(int i = 0; i < num_Joints; i++){
    if (joints[i].isOn){   // Can be used to turn certain joints off
      if(abs(joints[i].force+joints[i].assistance_force)<joints[i].MAX_Force){  // Check if we are using more than max force, if we do stop

        joints[i].ddx = 1/joints[i].mass * ((joints[i].force+joints[i].assistance_force)*joints[i].radius - joints[i].damping * joints[i].dx);
        joints[i].dx += joints[i].ddx * timestep;
        joints[i].x = joints[i].x + joints[i].dx * timestep + 0.5 * joints[i].ddx * timestep * timestep;
        
        // Stop if the actuator is in max position, set velocity and accelaration to zero to allow quick turnaround and to avoid getting stuck in extreme position
        if (joints[i].x > joints[i].MAX_Pos){   
          joints[i].x = joints[i].MAX_Pos;
          joints[i].dx = 0;
          joints[i].ddx = 0;
        }
        // Stop if the actuator is in min position, set velocity and accelaration to zero to allow quick turnaround and to avoid getting stuck in extreme position 
        if (joints[i].x < joints[i].MIN_Pos){   
          joints[i].x = joints[i].MIN_Pos;
          joints[i].dx = 0;
          joints[i].ddx = 0;
        }
      }
      // Stop if there is too much force
      else{                               
        joints[i].dx = 0;
        joints[i].ddx = 0;
      }
      //calculate actuator positioning for double and nonlinearly actuated joints, this is different because they are coupled and not linear, also x,dx,ddx are in degrees so there needs to be 
      // a conversion to actual actuator position
      if(joints[i].DOUBLE_actuated == true && joints[i].NONLINEAR_DOUBLE_actuated == true){               
        // 0.028 and 0.03 are offsets that can be adjusted based on the bowden cable lengths, so that there is the right amount of tension in
        // the two cables responsible for supination/pronation
        joints[i].motor[0].writeMicroseconds((2*joints[i].radius*sin(pi*((90+joints[i].x)/360))-0.028)/stroke_length*max_signal_offset+min_signal_duration);
        joints[i].motor[1].writeMicroseconds((2*joints[i].radius*sin(pi*((90-joints[i].x)/360))-0.03)/stroke_length*max_signal_offset+min_signal_duration);
      }
      // The position for double actuated joints is just the same. This applies e.g. to fingers that are supported by two motors that move simoultaneously
      else if(joints[i].DOUBLE_actuated == true && joints[i].NONLINEAR_DOUBLE_actuated == false){               
        joints[i].motor[0].writeMicroseconds(joints[i].x/stroke_length*max_signal_offset+min_signal_duration);
        joints[i].motor[1].writeMicroseconds(joints[i].x/stroke_length*max_signal_offset+min_signal_duration);
      }
    else{                               // If its not supination, set normal actuator position
        joints[i].motor[0].writeMicroseconds(joints[i].x/stroke_length*max_signal_offset+min_signal_duration);
      }
    }
  }
}

// FUNCTION : Calculates averages from measurements
void sumMeasurements(){
  AverageTemp += fTemperature_Value;
  AverageGSR += fGSR_Value;
  
  for(int i = 0; i < num_Joints; i++){
    joints[i].AverageForce += joints[i].force;
    joints[i].AverageAssistanceForce += joints[i].assistance_force;
  }  
}

// FUNCTION : Resets averages from measurements
void resetAnalysis(){
  AverageTemp = 0;
  AverageGSR = 0;
  for(int i = 0; i < num_Joints; i++){
    joints[i].AverageForce = 0;
    joints[i].AverageAssistanceForce = 0;
  }
  outString = "";
  feedbackTime = 0;
  counter = 0;
}

// FUNCTION : Extends the string to be sent via the serial port
void addOutString(float inp){
  s1=String(inp/counter);             // Divide the measured value by the value of the counter -> average
  s1=String(s1+",");
  outString=String(outString+s1);
}

// FUNCTION : Sends data via the serial port
void outputData(){
  addOutString(AverageTemp);
  addOutString(AverageGSR);
  
  for(int i = 0; i < num_Joints; i++){
    addOutString(joints[i].AverageForce);
    addOutString(joints[i].AverageAssistanceForce);
  }
  Serial.flush();
  Serial.println(outString);
}

// FUNCTION : Run calibration procedure to zero force sensors
void runCalibration(){
  
  delay(2000); // Wait for exo user to hold still
  
   for(int i = 0; i < num_Joints; i++){
    
    // Take 5000 measurements and average them and use this as a new offset
    force_raw=0;  
    for (int j = 0; j < 5000; j++){
      force_raw += analogRead(joints[i].forcePin[0]);
    }
    joints[i].sensor_offset[0]=force_raw/5000;

    // If a joint is double actuated, there is a second sensor to read
    if(joints[i].DOUBLE_actuated){
      force_raw = 0;
      for (int j = 0; j < 5000; j++){
        force_raw += analogRead(joints[i].forcePin[1]);
      }
      joints[i].sensor_offset[1] = force_raw/5000;
    }
  }
}

// FUNCTION : Read the forces from the sensors and convert raw values to N
void readForces(){
  for(int i = 0; i < num_Joints; i++){
    force_raw = analogRead(joints[i].forcePin[0]);

    // (raw - offset) * calibrationfactor * gravity * direction of sensor
    joints[i].force = (force_raw - joints[i].sensor_offset[0]) * calib_fact * g * joints[i].sensor_dir[0];
    
    // If a joint is double actuated, there is a second sensor to read
    if(joints[i].DOUBLE_actuated){
      force_raw = analogRead(joints[i].forcePin[1]);
      joints[i].force += (force_raw - joints[i].sensor_offset[1]) * calib_fact * g * joints[i].sensor_dir[1];
    }
  }
}

