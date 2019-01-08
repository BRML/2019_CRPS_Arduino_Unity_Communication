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
#define WRIST_EXO 1
#define FINGER_EXO 0
#define num_Joints 3
//////////////////////////

#include <Servo.h>
#include <math.h>
#include "Libraries/Adafruit_MLX90614.h"

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
  int   NONLINEAR_DOUBLE_actuated = 0;    // TWO motors at one joint or finger, but moving non-linearly           // Markus to Daniel: This has changed
  
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

// Variables used for data output
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

// Additional measurement variables
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
  setValues(joints);              // Obtain Settings from Unity3D
  
  for(int i=0;i<num_Joints;i++){  // Attach servos and bring them into starting position
    joints[i].motor[0].attach(joints[i].actuatorPin[0]);
    joints[i].motor[0].writeMicroseconds(1500);
    
    if(joints[i].DOUBLE_actuated){
      joints[i].motor[1].attach(joints[i].actuatorPin[1]);
      joints[i].motor[1].writeMicroseconds(1500);
    }
  }  
  
  pinMode(CalibrationPin,INPUT);  // Set the calibration pin as simple input pin
  
  // wait for patient to put on exoskeleton
  //delay(10000);
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
    runCalibration(joints);
    runCalib = 0;  
    start = micros();
  }
  
  readForces(joints);  
  runVelAssistance(joints);
  runAdmittance(joints);
  calculateAverages(joints);

  if (feedbackTime>(1/feedbackFreq)){
    outputData(joints);
    resetAnalysis(joints);
    readUnityInput(joints);
  }
  stp = micros();
}

// FUNCTION : After starting the serial connection all the settings are read in from Unity3D
void setValues(struct sJoint *jts){
    
  while(Serial.available()<=0){}

  // General Settings
  feedbackFreq = Serial.parseFloat();
  potentiometerPin = Serial.parseInt();
  GSRPin = Serial.parseInt();
  controllerAttached = Serial.parseInt();
  CalibrationPin = Serial.parseInt();
  runCalib = Serial.parseInt();
  assist_scale = Serial.parseFloat();

  // Joints specific settings
  for(int i = 0; i < num_Joints; i++){
    jts[i].isOn = Serial.parseInt();;
    jts[i].actuatorPin[0] = Serial.parseInt();
    jts[i].forcePin[0] = Serial.parseInt();
    jts[i].mass = Serial.parseFloat(); 
    jts[i].damping = Serial.parseFloat();  
    jts[i].MIN_Pos = Serial.parseFloat();
    jts[i].MAX_Pos = Serial.parseFloat();
    jts[i].sensor_dir[0] = Serial.parseInt();
    jts[i].MAX_Force = Serial.parseFloat();
    jts[i].MIN_Vel = Serial.parseFloat();
    jts[i].assistance_speed = Serial.parseFloat();
    jts[i].DOUBLE_actuated = Serial.parseInt();
    jts[i].NONLINEAR_DOUBLE_actuated = Serial.parseInt();             // Markus: This has to be added

    if (jts[i].DOUBLE_actuated){
      jts[i].actuatorPin[1] = Serial.parseInt();
      jts[i].forcePin[1] = Serial.parseInt();
      jts[i].radius = Serial.parseFloat();
      jts[i].sensor_dir[1] = Serial.parseInt();
    }
  }
  Serial.flush();
}

// FUNCTION : Calculates and writes the motor positions using the measured forces as well as the calculated force assistance
void runAdmittance(struct sJoint *jts){
  
  for(int i=0;i<num_Joints;i++){
    if (jts[i].isOn){   // Can be used to turn certain joints off
      if(abs(jts[i].force+jts[i].assistance_force)<jts[i].MAX_Force){  // Check if we are using more than max force, if we do stop

        jts[i].ddx = 1/jts[i].mass * ((jts[i].force+jts[i].assistance_force)*jts[i].radius - jts[i].damping * jts[i].dx);
        jts[i].dx += jts[i].ddx * timestep;
        jts[i].x = jts[i].x + jts[i].dx * timestep + 0.5 * jts[i].ddx * timestep * timestep;
        
        // Stop if the actuator is in max position, set velocity and accelaration to zero to allow quick turnaround and to avoid getting stuck in extreme position
        if (jts[i].x > jts[i].MAX_Pos){   
          jts[i].x = jts[i].MAX_Pos;
          jts[i].dx = 0;
          jts[i].ddx = 0;
        }
        // Stop if the actuator is in min position, set velocity and accelaration to zero to allow quick turnaround and to avoid getting stuck in extreme position 
        if (jts[i].x < jts[i].MIN_Pos){   
          jts[i].x = jts[i].MIN_Pos;
          jts[i].dx = 0;
          jts[i].ddx = 0;
        }
      }
      // Stop if there is too much force
      else{                               
        jts[i].dx = 0;
        jts[i].ddx = 0;
      }
      //calculate actuator positioning for double actuated joints, this is different because they are coupled and not linear, also x,dx,ddx are in degrees so there needs to be 
      // a conversion to actual actuator position
      if(jts[i].DOUBLE_actuated == true && jts[i].NONLINEAR_DOUBLE_actuated == true){               
        // 0.028 and 0.03 are offsets that can be adjusted based on the bowden cable lengths, so that there is the right amount of tension in
        // the two cables responsible for supination/pronation
        jts[i].motor[0].writeMicroseconds((2*jts[i].radius*sin(pi*((90+jts[i].x)/360))-0.028)/stroke_length*max_signal_offset+min_signal_duration);
        jts[i].motor[1].writeMicroseconds((2*jts[i].radius*sin(pi*((90-jts[i].x)/360))-0.03)/stroke_length*max_signal_offset+min_signal_duration);
      }
      else if(jts[i].DOUBLE_actuated == true && jts[i].NONLINEAR_DOUBLE_actuated == false){               
        jts[i].motor[0].writeMicroseconds(jts[i].x/stroke_length*max_signal_offset+min_signal_duration);
        jts[i].motor[1].writeMicroseconds(jts[i].x/stroke_length*max_signal_offset+min_signal_duration);
      }
      else{                               // If its not supination, set normal actuator position
        jts[i].motor[0].writeMicroseconds(jts[i].x/stroke_length*max_signal_offset+min_signal_duration);
      }
    }
  }
}

// FUNCTION : Calculates the assistance force for the joints
void runVelAssistance(struct sJoint *jts){
  for(int i=0;i<num_Joints;i++){
    if((jts[i].desired_pos-jts[i].x)>0 && jts[i].dx<jts[i].MIN_Vel && jts[i].assistance_force<potentiometer_MAX_Force){
      //if i want to move up, but am not moving fast enough and assistance force is not max yet, increase force
      if(jts[i].assistance_force<0){
        // if direction of my target movement just changed, start again with 0 assistive force
        jts[i].assistance_force=0;
      }
      jts[i].assistance_force+=jts[i].assistance_speed;
    }
    else if((jts[i].desired_pos-jts[i].x)<0 && jts[i].dx>-jts[i].MIN_Vel && jts[i].assistance_force>-potentiometer_MAX_Force){
      //if i want to move down, but am not moving fast enough and assistance force is not -max yet, decrease force
      if(jts[i].assistance_force>0){
        // if direction of my target movement just changed, start again with 0 assistive force
        jts[i].assistance_force=0;
      }
      jts[i].assistance_force-=jts[i].assistance_speed;
    }
    else{
      //if assistance force is max hold it at that position through repeated increase decrease steps, if i am moving into the right direction fast enough decrease the magnitude of the assistane force
      if(jts[i].assistance_force>0){
        jts[i].assistance_force-=jts[i].assistance_speed;
      }
      else{
        jts[i].assistance_force+=jts[i].assistance_speed;
      }
    }
  }
}

// FUNCTION : Calculates averages from measurements
void calculateAverages(struct sJoint *jts){
  AverageTemp += fTemperature_Value;
  AverageGSR += fGSR_Value;
  
  for(int i=0;i<num_Joints;i++){
    jts[i].AverageForce += jts[i].force;
    jts[i].Average_dx += jts[i].dx;
    jts[i].Average_ddx += jts[i].ddx;
    jts[i].AverageAssistanceForce += jts[i].assistance_force;
  }  
}

// FUNCTION : Resets averages from measurements
void resetAnalysis(struct sJoint *jts){
  AverageTemp = 0;
  AverageGSR = 0;
  for(int i = 0; i < num_Joints; i++){
    jts[i].AverageForce = 0;
    jts[i].Average_dx = 0;
    jts[i].Average_ddx = 0;
    jts[i].AverageAssistanceForce = 0;
  }
  outString = "";
  feedbackTime = 0;
  counter = 0;
}

// FUNCTION : Extends the string to be sent via the serial port
void addOutString(float inp){
  s1=String(inp/counter);
  s1=String(s1+",");
  outString=String(outString+s1);
}

// FUNCTION : Sends data via the serial port
void outputData(struct sJoint *jts){
  addOutString(AverageTemp);
  addOutString(AverageGSR);
  
  for(int i = 0; i < num_Joints; i++){
    addOutString(jts[i].x*counter);         // is not an average value
    addOutString(jts[i].Average_dx);
    addOutString(jts[i].Average_ddx);
    addOutString(jts[i].AverageForce);
    addOutString(jts[i].AverageAssistanceForce);
  }
  Serial.flush();
  Serial.println(outString);
}

// FUNCTION : Run calibration procedure to zero out force sensors
void runCalibration(struct sJoint *jts){
  
  delay(2000); // Wait for exo user to hold still
  
   for(int i = 0; i < num_Joints; i++){
    
    // Take 5000 measurements and average them and use this as new offset
    force_raw=0;  
    for (int j = 0; j < 5000; j++){
      force_raw += analogRead(jts[i].forcePin[0]);
    }
    jts[i].sensor_offset[0]=force_raw/5000;

    //supination has 2 sensors
    if(jts[i].DOUBLE_actuated){
      force_raw = 0;
      for (int j = 0; j < 5000; j++){
        force_raw += analogRead(jts[i].forcePin[1]);
      }
      jts[i].sensor_offset[1] = force_raw/5000;
    }
  }
}

// FUNCTION : Read in the forces from the sensors and convert raw values to N
void readForces(struct sJoint *jts){
  for(int i=0;i<num_Joints;i++){
    force_raw=analogRead(jts[i].forcePin);

    //(raw -offset)*calibrationfactor * gravity *direction of sensor
    jts[i].force=(force_raw-jts[i].sensor_offset[0])*calib_fact*g*jts[i].sensor_dir[0];
    
    //supination has 2 sensors
    if(jts[i].DOUBLE_actuated){
      force_raw=analogRead(jts[i].forcePin[1]);
      jts[i].force+=(force_raw-jts[i].sensor_offset[1])*calib_fact*g*jts[i].sensor_dir[1];
    }
  }
}

// FUNCTION : Read desired positions from Unity3D
void readUnityInput(struct sJoint *jts){
  
  if(Serial.available()>0){   // dont wait if values are not there otherwise the control will be slow
    for(int i = 0; i < num_Joints; i++){
      jts[i].desired_pos = Serial.parseFloat();
    }
  }
  Serial.flush();
}

   /*
  
  if(WRIST_EXO == true)
  {
    // Define three rotary joints of the human wrist
    struct sJoint flexion; 
    struct sJoint radial;
    struct sJoint supin;
  
    //put all the rotations into array
    struct sJoint joints[num_Joints]={flexion,radial,supin};
    };

    if(FINGER_EXO == true)
   {
   // Define three rotary joints of the human wrist
    struct sJoint Index; 
    struct sJoint Thumb;
  
    //put all the rotations into array
    struct sJoint joints[num_Joints]={Index, Thumb};
    };
  */
