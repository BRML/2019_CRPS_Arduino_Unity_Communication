#include <Servo.h>
#include <math.h>
#include "Libraries/Adafruit_MLX90614.h"

#define pi 3.14159265
#define g  0.00981 //[N/g]
#define num_rotations 3

////////////////////////////////////////////////
//      Technische Universität München        //
//              Dezember 2018                 //
//             Daniel Angloher                //
// Wrist exoskeleton control scheme and unity //
//                communication               //
////////////////////////////////////////////////

//Runs an admittance control scheme with optional directed force support.
//Settings are received over serial connection from unity and data outputs 
//are send to unity for post processing.

//define pins
int potentiometer_pin;
int calibration_pin;
int gsr_pin;

Adafruit_MLX90614 mlx;

//Define Rotations
struct rotation{
  Servo motor;
  int is_on;
  int actuator_pin;
  int force_pin;
  float mass; // [kg]  Virtual mass of the admittance control scheme
  float damping; // [N*s/(deg or m)] Virtual damping of the admittance control scheme
  float force=0; // [N] Force, measured at the force sensor
  float ddx=0; // [(deg or m)/s2] Acceleration of the virtual mass
  float dx=0; // [(deg or m)/s] Velocity of the virtual mass
  float x; //[deg or m] position of the motor
  float min_pos;//[deg or m] min position of the motor
  float max_pos;//[deg or m] max position of the motor
  int   sensor_dir; //direction of the attached sensor
  float assistance_force=0;
  float desired_pos;
  float sensor_offset=500;
  float max_force;
  float min_vel;
  float assist_speed;

  //average variables for unity output
  float force_average=0;
  float dx_average=0;
  float ddx_average=0;
  float assistance_force_average=0;

  //additional variables for supination/pronation (these are for the right sensor)
  int   is_supin=0;
  Servo motor2;
  int   actuator_pin2;
  int   force_pin2;
  float sensor_offset2=500;
  float radius=1;
  int   sensor_dir2;
};

//create rotations
struct rotation flexion;
struct rotation radial;
struct rotation supin;

//put all the rotations into array
struct rotation rotations[num_rotations]={flexion,radial,supin};

//general hardware settings
float stroke_length=0.1; //[m]
float min_signal_duration=1000;//[microseconds]
float max_signal_offset=1000;//[microseconds]
float calib_fact=1.5;

//init variables used for time keeping and output
int counter=0;
double stp=0;
double start=0;
float feedbackFreq=1;
float timestep=0;
String outString;
String s1;
int force_raw = 0;
float feedbackTime=0;
float potentiometer_max_force=0; // assistance force scalable with slider
float assist_scale; // maximum assistance force when slider is at 100%
int controllerAttached=1;
int runCalib=0;

//additional measurement variables
float temp=0;
float gsr=0;
float temp_average=0;
float gsr_average=0;

void setValues(struct rotation *rots){
  //after the begin of serial connection all the settings are read in from unity
  
  while(Serial.available()<=0){}

  //general settings
  feedbackFreq=Serial.parseFloat();
  potentiometer_pin=Serial.parseInt();
  gsr_pin=Serial.parseInt();
  controllerAttached= Serial.parseInt();
  calibration_pin=Serial.parseInt();
  runCalib=Serial.parseInt();
  assist_scale=Serial.parseFloat();

  //rotation specific settings
  for(int i=0;i<num_rotations;i++){
    rots[i].is_on=Serial.parseInt();;
    rots[i].actuator_pin=Serial.parseInt();
    rots[i].force_pin=Serial.parseInt();
    rots[i].mass=Serial.parseFloat(); 
    rots[i].damping=Serial.parseFloat();  
    rots[i].min_pos=Serial.parseFloat();
    rots[i].max_pos=Serial.parseFloat();
    rots[i].sensor_dir=Serial.parseInt();
    rots[i].max_force=Serial.parseFloat();
    rots[i].min_vel=Serial.parseFloat();
    rots[i].assist_speed=Serial.parseFloat();
    rots[i].is_supin=Serial.parseInt();

    if (rots[i].is_supin){
      rots[i].actuator_pin2=Serial.parseInt();
      rots[i].force_pin2=Serial.parseInt();
      rots[i].radius=Serial.parseFloat();
      rots[i].sensor_dir2=Serial.parseInt();
    }
  }
  Serial.flush();
}

void runAdmittance(struct rotation *rots){
  //calculates and writes the motor positions using the measured forces as well as the calculated force assistance
  
  for(int i=0;i<num_rotations;i++){
    if (rots[i].is_on){   //can be used to turn certain rotations off
      if(abs(rots[i].force+rots[i].assistance_force)<rots[i].max_force){  //check if we are using more than max force, if we do stop


        //F = m*ddx + d*dx      overall force F consists of measured force + assistance force 
        //ddx=1/m*(F-d*dx)      for supination the forces become torques and accelartion, velocity and position are angular this is done by multiplying by the moment arm. For the other rotations the momentarm is set to 1 so this has no effect on them
        //dx=ddx*t
        //x=x+dx*t+0.5*ddx*t*t
        
        rots[i].ddx = 1/rots[i].mass * ((rots[i].force+rots[i].assistance_force)*rots[i].radius - rots[i].damping * rots[i].dx);
        rots[i].dx += rots[i].ddx * timestep;
        rots[i].x = rots[i].x + rots[i].dx * timestep + 0.5 * rots[i].ddx * timestep * timestep;
        
        
        if (rots[i].x>rots[i].max_pos){   // stop if the actuator is in max position, set velocity and accelaration to zero to allow quick turnaround and to avoid getting stuck in extreme position
          rots[i].x=rots[i].max_pos;
          rots[i].dx=0;
          rots[i].ddx=0;
        }
        if (rots[i].x<rots[i].min_pos){   //same as for max pos
          rots[i].x=rots[i].min_pos;
          rots[i].dx=0;
          rots[i].ddx=0;
        }
      }
      else{                               //stop if there is too much force
        rots[i].dx=0;
        rots[i].ddx=0;
      }
      if(rots[i].is_supin){               //calculate actuator positioning for supination, this is different because they are coupled and not linear, also x,dx,ddx are in degrees so there needs to be a conversion to actual actuator position
        // 0.028 and 0.03 are offsets that can be adjusted based on the bowden cable lengths, so that there is the right amount of tension in
        // the two cables responsible for supination/pronation
        rots[i].motor.writeMicroseconds((2*rots[i].radius*sin(pi*((90+rots[i].x)/360))-0.028)/stroke_length*max_signal_offset+min_signal_duration);
        rots[i].motor2.writeMicroseconds((2*rots[i].radius*sin(pi*((90-rots[i].x)/360))-0.03)/stroke_length*max_signal_offset+min_signal_duration);
      }
      else{                               //if its not supination, set normal actuator position
        rots[i].motor.writeMicroseconds(rots[i].x/stroke_length*max_signal_offset+min_signal_duration);
      }
    }
  }
}

void runVelAssistance(struct rotation *rots){
  //calculates the assistance force for the rotations
  for(int i=0;i<num_rotations;i++){
    if((rots[i].desired_pos-rots[i].x)>0 && rots[i].dx<rots[i].min_vel && rots[i].assistance_force<potentiometer_max_force){
      //if i want to move up, but am not moving fast enough and assistance force is not max yet, increase force
      if(rots[i].assistance_force<0){
        // if direction of my target movement just changed, start again with 0 assistive force
        rots[i].assistance_force=0;
      }
      rots[i].assistance_force+=rots[i].assist_speed;
    }
    else if((rots[i].desired_pos-rots[i].x)<0 && rots[i].dx>-rots[i].min_vel && rots[i].assistance_force>-potentiometer_max_force){
      //if i want to move down, but am not moving fast enough and assistance force is not -max yet, decrease force
      if(rots[i].assistance_force>0){
        // if direction of my target movement just changed, start again with 0 assistive force
        rots[i].assistance_force=0;
      }
      rots[i].assistance_force-=rots[i].assist_speed;
    }
    else{
      //if assistance force is max hold it at that position through repeated increase decrease steps, if i am moving into the right direction fast enough decrease the magnitude of the assistane force
      if(rots[i].assistance_force>0){
        rots[i].assistance_force-=rots[i].assist_speed;
      }
      else{
        rots[i].assistance_force+=rots[i].assist_speed;
      }
    }
  }
}

void runAnalysis(struct rotation *rots){
  //add up average for unity output
  temp_average+=temp;
  gsr_average+=gsr;
  for(int i=0;i<num_rotations;i++){
    rots[i].force_average+=rots[i].force;
    rots[i].dx_average+=rots[i].dx;
    rots[i].ddx_average+=rots[i].ddx;
    rots[i].assistance_force_average+=rots[i].assistance_force;
  }  
}

void resetAnalysis(struct rotation *rots){
  //reset variables for next feedback cycle
  temp_average=0;
  gsr_average=0;
  for(int i=0;i<num_rotations;i++){
    rots[i].force_average=0;
    rots[i].dx_average=0;
    rots[i].ddx_average=0;
    rots[i].assistance_force_average=0;
  }
  outString="";
  feedbackTime=0;
  counter=0;
}

void addOutString(float inp){
  // add averaged outputs to one single comma delimted output string
  s1=String(inp/counter);
  s1=String(s1+",");
  outString=String(outString+s1);
}

void outputData(struct rotation *rots){
  //add output data to output string and send to unity through serial connection
  addOutString(temp_average);
  addOutString(gsr_average);
  for(int i=0;i<num_rotations;i++){
    addOutString(rots[i].x*counter);    //is not an average value
    addOutString(rots[i].dx_average);
    addOutString(rots[i].ddx_average);
    addOutString(rots[i].force_average);
    addOutString(rots[i].assistance_force_average);
  }
  Serial.flush();
  Serial.println(outString);
}

void runCalibration(struct rotation *rots){
  //run calibration procedure to zero out force sensors
  delay(2000);
  //weigth for patient to hold still
  for(int i=0;i<num_rotations;i++){
    
    //take 5000 measurements and average them and use this as new offset
    force_raw=0;  
    for (int j=0;j<5000;j++){
      force_raw += analogRead(rots[i].force_pin);
    }
    rots[i].sensor_offset=force_raw/5000;

    //supination has 2 sensors
    if(rots[i].is_supin){
      force_raw=0;
      for (int j=0;j<5000;j++){
        force_raw += analogRead(rots[i].force_pin2);
      }
      rots[i].sensor_offset2=force_raw/5000;
    }
    
  }
}

void readForces(struct rotation *rots){
  // read in the forces from the sensors and convert raw values to N
  for(int i=0;i<num_rotations;i++){
    force_raw=analogRead(rots[i].force_pin);

    //(raw -offset)*calibrationfactor * gravity *direction of sensor
    rots[i].force=(force_raw-rots[i].sensor_offset)*calib_fact*g*rots[i].sensor_dir;
    
    //supination has 2 sensors
    if(rots[i].is_supin){
      force_raw=analogRead(rots[i].force_pin2);
      rots[i].force+=(force_raw-rots[i].sensor_offset2)*calib_fact*g*rots[i].sensor_dir2;
    }
  }
}

void readUnityInput(struct rotation *rots){
  //read desired positions from unity
  if(Serial.available()>0){   //dont wait if values are not there otherwise the control will be slow
    for(int i=0;i<num_rotations;i++){
      rots[i].desired_pos=Serial.parseFloat();
    }
  }
  Serial.flush();
}

void setup()
{ 
  //open Serial
  Serial.begin(9600);
  while(!Serial);
  analogReference(INTERNAL2V56);
  mlx.begin();
  
  //get settings from unity
  setValues(rotations);
  
  //attach_servos and bring into starting position
  for(int i=0;i<num_rotations;i++){
    rotations[i].motor.attach(rotations[i].actuator_pin);
    rotations[i].motor.writeMicroseconds(1500);
    if(rotations[i].is_supin){
      rotations[i].motor2.attach(rotations[i].actuator_pin2);
      rotations[i].motor2.writeMicroseconds(1500);
    }
  }  
  pinMode(calibration_pin,INPUT);
  // wait for patient to put on exoskeleton
  delay(10000);
}


void loop()
{
  timestep=(stp-start)/1000000;
  feedbackTime+=timestep;
  start=micros();
 
  counter+=1;
  
  if(controllerAttached){
    runCalib=digitalRead(calibration_pin);
    // check if sign flip is needed with new controler wireing
    potentiometer_max_force=-(analogRead(potentiometer_pin)/1023.0-1)*assist_scale; 
    temp=mlx.readObjectTempC();
    gsr=analogRead(gsr_pin);
  }

  if(runCalib){
    runCalibration(rotations);
    runCalib=0;  
    start=micros();
  }
  
  readForces(rotations);  
  runVelAssistance(rotations);
  runAdmittance(rotations);
  runAnalysis(rotations);

  if (feedbackTime>(1/feedbackFreq)){
    outputData(rotations);
    resetAnalysis(rotations);
    readUnityInput(rotations);
  }
  stp=micros();
}
