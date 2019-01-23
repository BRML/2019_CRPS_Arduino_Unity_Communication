#include <Servo.h>
#include <math.h>
#include <Filters.h>
#include <Arduino.h>
#include <include/twi.h>

#define MAX_VEL 0.2              // [m/s] Maximum velocity of the motors
#define MAX_ACC 0.5              // [m/s^2] Maximum acceleration of the motors
#define MIN_SIGNAL_DURATION 1000 // [microseconds]
#define MAX_SIGNAL_OFFSET 1000   // [microseconds]

// Definitions for communication with the IR Thermometer //
#define ADDR      0x5A

//EEPROM 32x16
#define TO_MAX    0x00
#define TO_MIN    0x01
#define PWM_CTRL  0x02

//RAM 32x16
#define RAW_IR_1  0x04
#define RAW_IR_2  0x05
#define TA        0x06
#define TOBJ_1    0x07
#define TOBJ_2    0x08

#define SYNC_PIN  2

// Physical contraints //
double mass = 5;                 // [kg]  Virtual mass of the admittance control scheme
double damping = 10.0;           // [N*s/m] Virtual damping of the admittance control scheme
double g = 0.00981;              // [N/g] Gravity

float MIN_RANGE = 0;             // Minimum positon of the motors (can be modified from Unity3D)
float MAX_RANGE = 0.5;           // Maximum positon of the motors (can be modified from Unity3D)
float MAX_FORCE = 0;             // Maximum force of the motors (can be modified from Unity3D)

// Pins on the Arduino //
int Motor_Pin[4] = {2,3,4,5};                           // Arduino Pins for the motors
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
  double Threshold_Position = 0.15; // [] Relative threshold position from lower and upper end of the range of the motor
};

struct s_Assistive_Force Assistive_Force[4];            // Assistive forces for all joints
struct s_Kinetics Kinetics[4];                          // Kinetic parameters for all joints
Servo Motor[4];                                         // Handle to the motors
double Offset_Force_Sensor[4] = {689, 671, 598, 632};   // Measured offsets of the force sensors
double Calib_Force_Sensor[4] = {-1.47, -1.45, -1.54, -1.54}; // Measured calibration factors of the force sensors
double Force_Force_Sensor[4];                           // Measured force from the force sensors

// Sensor box variables //
float fMAX_Assistance_Force = 0;      // [N] Maximum assistance force, set by the potentiometer    
float fGSR_Value = 0;                 // [] Value of the GSR
float fTemperature_Value = 0;         // [C] Measured Temperature

// Communication variables //
float timestep = 0;                   // [s] Time for one iteration of the main loop
float feedbackFreq = 4.0;             // [1/s] Data is sent to the PC this many times per seconds
float feedbackTime = 0;               // [s] Time since the last feedback was sent to the PC
double start = 0;                     // [s] Measures the time instant for the start of each iteration
double stp = 0;                       // [s] Measures the time instand for the end of each iteration
String outString;                     // [] The string to be sent to the PC
String printout;                      // [] The string to be sent to the Console

static const uint32_t TWI_CLOCK = 100000;     // IR Thermometer Communication Variable
static const uint32_t RECV_TIMEOUT = 100000;  // IR Thermometer Communication Variable
static const uint32_t XMIT_TIMEOUT = 100000;  // IR Thermometer Communication Variable

Twi *pTwi = WIRE_INTERFACE;           // Communication handle to the IR Thermometer

//FilterOnePole lowpassFilter( LOWPASS, 1000 );  // create a one pole (RC) lowpass filter

// SETUP ROUTINE //
void setup() {
  Serial.begin(9600);                 // Start Serial Communication and set Analog reference

  //setValues();                        // Obtain Settings for all the joints from Unity3D           

  // Set up communication with the IR Thermometer
  pinMode(SYNC_PIN, OUTPUT);          
  digitalWrite(SYNC_PIN, LOW);
  Wire_Init();
  pTwi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
  TWI_ConfigureMaster(pTwi, TWI_CLOCK, VARIANT_MCK);
  
  //Attach servos
  for(int iJoints = 0; iJoints<4; iJoints++){
    Motor[iJoints].attach(Motor_Pin[iJoints]);
    Motor[iJoints].writeMicroseconds(1000);
  }

  // Calculate offset of force sensors  
  for(int i=0;i<5000;i++){
      for(int iJoints = 0; iJoints<4; iJoints++){
        Offset_Force_Sensor[iJoints] += analogRead(Force_Sensor_Pin[iJoints]); 
      }
  }

  // Take the mean value of the previous measurements to obtain an adequate offset for the force sensors
  for(int iJoints = 0; iJoints<4; iJoints++){
    Offset_Force_Sensor[iJoints] /= 5000;
  }
}

//// MAIN LOOP ////
void loop() {
  timestep = (stp-start)/1000000; // [s] Calculate the timestep for the last iteration
  feedbackTime += timestep;       // [s]Sum up the time since the last time data was sent to the PC
  start = micros();               // [mus] Measure the current time
  
    // Read force sensors
    for(int iJoints = 0; iJoints<4; iJoints++){
      Force_Force_Sensor[iJoints] = ((analogRead(Force_Sensor_Pin[iJoints])) 
                                    - Offset_Force_Sensor[iJoints]) * Calib_Force_Sensor[iJoints] * g;  
    } 

    // Generate an assistive force; here, the force is the same for all the joints of each finger 
    Generate_Assistive_Force(Kinetics[0].x, Kinetics[1].x, 0);    
    Assistive_Force[1] = Assistive_Force[0];
    Generate_Assistive_Force(Kinetics[2].x, Kinetics[3].x, 2);    
    Assistive_Force[3] = Assistive_Force[2];

    // Run the admittance control scheme; here, the forces on one finger are added up, which makes the motors move unisono
    Admittance_Control(Force_Force_Sensor[0] + Force_Force_Sensor[1], Assistive_Force[0].Force_Level, 0);
    Admittance_Control(Force_Force_Sensor[0] + Force_Force_Sensor[1], Assistive_Force[1].Force_Level, 1);
    Admittance_Control(Force_Force_Sensor[2] + Force_Force_Sensor[3], Assistive_Force[2].Force_Level, 2);
    Admittance_Control(Force_Force_Sensor[2] + Force_Force_Sensor[3], Assistive_Force[3].Force_Level, 3);
    
    // Whenever Feedbacktime = 1/FeedbackFrequency read the sensor box and send data to the PC
    if (feedbackTime>(1.0/feedbackFreq)){
      readSensorBox();          // Read the sensors from the sensor box
      //outputData();             // Send data to the PC
      Print_Data2Console();     // Print the data to console (also on the PC)
      
      feedbackTime = 0;
    } 
    
  stp = micros();               // Measure the current time   
}

// FUNCTION : Reads the IR Temperature Sensor; Note: This is special for the Arduino DUE
float readIRThermometer()
{
  uint16_t tempUK;
  float tempC;
  uint8_t hB, lB, pec;

  digitalWrite(SYNC_PIN, HIGH);
  //delayMicroseconds(10);
  digitalWrite(SYNC_PIN, LOW);

  TWI_StartRead(pTwi, ADDR, TOBJ_1, 1);

  lB = readByte();
  hB = readByte();
  
  //last read
  TWI_SendSTOPCondition(pTwi);
  pec = readByte();
  
  while (!TWI_TransferComplete(pTwi)) 
    ;

  tempUK = (hB << 8) | lB;
  tempC = ((float)tempUK * 2) / 100 - 273.15 ;
  
  return tempC; 
}


// FUNCTION : Reads the sensors from the sensor box
void readSensorBox(){
      fTemperature_Value = readIRThermometer();                   // Temperature
      fMAX_Assistance_Force = analogRead(Potentiometer_Pin)/100;  // Assistance Force / Potentiometer  
      fGSR_Value = analogRead(GSR_Pin);                           // GSR       
}

// FUNCTION : Runs an admittance control scheme and sets the respective motor position
void Admittance_Control(double Force_Sensor, double Assistive_Force, int num) 
{
    // Admittance control basic equation: F = m*ddx + d*dx
    Kinetics[num].ddx = 1/mass * (Force_Sensor + Assistive_Force - damping * Kinetics[num].dx);

    // Delimit the acceleration
    if(Kinetics[num].ddx < -MAX_ACC) Kinetics[num].ddx = -MAX_ACC;
    if(Kinetics[num].ddx > MAX_ACC) Kinetics[num].ddx = MAX_ACC;
    
    // Then, the obtained acceleration ddx has to be integrated twice in order to obtain x.
    Kinetics[num].dx += Kinetics[num].ddx * timestep;

    // Delimit the velocity
    if(Kinetics[num].dx < -MAX_VEL) Kinetics[num].dx = -MAX_VEL;
    if(Kinetics[num].dx > MAX_VEL) Kinetics[num].dx = MAX_VEL;
    
    Kinetics[num].x = Kinetics[num].x + Kinetics[num].dx * timestep + 0.5 * Kinetics[num].ddx * timestep * timestep;

    // Delimit the positon 
    if(Kinetics[num].x < MIN_RANGE) Kinetics[num].x = MIN_RANGE;
    if(Kinetics[num].x > MAX_RANGE) Kinetics[num].x = MAX_RANGE;

    // Set the motor position
    Motor[num].writeMicroseconds( Kinetics[num].x / MAX_RANGE * MAX_SIGNAL_OFFSET + MIN_SIGNAL_DURATION);
}

// FUNCTION : Generates an assistive force, when position thresholds are reached by the operator of the exoskeleton
void Generate_Assistive_Force(double x_a, double x_b, int num)       
{
     if( (x_a > Assistive_Force[num].Threshold_Position * MAX_RANGE || x_b > Assistive_Force[num].Threshold_Position * MAX_RANGE) && Assistive_Force[num].Direction == 0.5 )
        Assistive_Force[num].Direction = 1;       // Assistive Force pushing forward
    
     if( (x_a < (1-Assistive_Force[num].Threshold_Position) * MAX_RANGE || x_b < (1-Assistive_Force[num].Threshold_Position) * MAX_RANGE) && Assistive_Force[num].Direction == -0.5 ) 
        Assistive_Force[num].Direction = -1;      // Assistive Force pushing backward
    
     if(x_a == MAX_RANGE && x_b == MAX_RANGE && Assistive_Force[num].Direction == 1) 
        Assistive_Force[num].Direction = -0.5;    // Assistive Force ready to push backward
      
     if (x_a == MIN_RANGE && x_b == MIN_RANGE && Assistive_Force[num].Direction == -1)
        Assistive_Force[num].Direction = 0.5;     // Assistive Force ready to push forward
    
     if(Assistive_Force[num].Force_Level < fMAX_Assistance_Force && Assistive_Force[num].Direction ==  1)
        Assistive_Force[num].Force_Level = Assistive_Force[num].Force_Level + 0.001;  // Assistive Force Level incremented to push further forward
    
     if(Assistive_Force[num].Force_Level > -fMAX_Assistance_Force && Assistive_Force[num].Direction == -1)
        Assistive_Force[num].Force_Level = Assistive_Force[num].Force_Level - 0.001;  // Assistive Force Level incremented to push further backward
    
     if(Assistive_Force[num].Direction == 0.5 || Assistive_Force[num].Direction == -0.5)
        Assistive_Force[num].Force_Level = 0;                                         // Assistive Force Level set to zero

     if(fMAX_Assistance_Force ==  0)      // Emergency shut off
        Assistive_Force[num].Force_Level = 0;   
}
    
// FUNCTION : After starting the serial connection all the settings are read in from Unity3D
void setValues(){
    
  while(Serial.available()<=0){}      // Wait for the serial connection to become available

  // Read general Settings
  feedbackFreq = Serial.parseFloat();
  MIN_RANGE = Serial.parseFloat(); 
  MAX_RANGE = Serial.parseFloat();
  MAX_FORCE = Serial.parseFloat();
  
  Serial.flush();
}

// FUNCTION : Extends the string to be sent via the serial port
void addOutString(float inp){
  String addString = String(inp);     
  addString = String(addString + ",");
  outString = String(outString + addString);
}

// FUNCTION : Sends data via the serial port
void outputData(){
  addOutString(fTemperature_Value);
  addOutString(fGSR_Value);
  addOutString(fMAX_Assistance_Force);
  for(int iJoints = 0; iJoints<4; iJoints++){
    addOutString(Force_Force_Sensor[iJoints]);
  }
  Serial.flush();
  Serial.println(outString);
  outString = "";
}

// FUNCTION : Prints data to console
void Print_Data2Console()
{
    printout = "Force Sensor 1: ";
    printout += Force_Force_Sensor[0];
    printout += " N, Force Sensor 2: ";
    printout += Force_Force_Sensor[1];
    printout += " N, Force Sensor 3: ";
    printout += Force_Force_Sensor[2];
    printout += " N, Force Sensor 4: ";
    printout += Force_Force_Sensor[3];
    printout += " N, Assistive Force :";
    printout += fMAX_Assistance_Force;
    printout += " N, GSR:";
    printout += fGSR_Value;
    printout += " [], Thermometer:";
    printout += fTemperature_Value;
    printout += " C, Timestep:";
    printout += timestep * 1000;
    printout += " ms, Feedbacktime ";
    printout += feedbackTime;
    printout += " s, Assistive Force Level ";
    printout +=  Assistive_Force[0].Force_Level;
   
    Serial.println(printout);   
}

// Funtions for communication with the IR Thermometer //
static void Wire_Init(void) {
  pmc_enable_periph_clk(WIRE_INTERFACE_ID);
  PIO_Configure(
  g_APinDescription[PIN_WIRE_SDA].pPort,
  g_APinDescription[PIN_WIRE_SDA].ulPinType,
  g_APinDescription[PIN_WIRE_SDA].ulPin,
  g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
  PIO_Configure(
  g_APinDescription[PIN_WIRE_SCL].pPort,
  g_APinDescription[PIN_WIRE_SCL].ulPinType,
  g_APinDescription[PIN_WIRE_SCL].ulPin,
  g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

  NVIC_DisableIRQ(TWI1_IRQn);
  NVIC_ClearPendingIRQ(TWI1_IRQn);
  NVIC_SetPriority(TWI1_IRQn, 0);
  NVIC_EnableIRQ(TWI1_IRQn);
}

static void Wire1_Init(void) {
    pmc_enable_periph_clk(WIRE1_INTERFACE_ID);
  PIO_Configure(
      g_APinDescription[PIN_WIRE1_SDA].pPort,
      g_APinDescription[PIN_WIRE1_SDA].ulPinType,
      g_APinDescription[PIN_WIRE1_SDA].ulPin,
      g_APinDescription[PIN_WIRE1_SDA].ulPinConfiguration);
  PIO_Configure(
      g_APinDescription[PIN_WIRE1_SCL].pPort,
      g_APinDescription[PIN_WIRE1_SCL].ulPinType,
      g_APinDescription[PIN_WIRE1_SCL].ulPin,
      g_APinDescription[PIN_WIRE1_SCL].ulPinConfiguration);

  NVIC_DisableIRQ(TWI0_IRQn);
  NVIC_ClearPendingIRQ(TWI0_IRQn);
  NVIC_SetPriority(TWI0_IRQn, 0);
  NVIC_EnableIRQ(TWI0_IRQn);
}

uint8_t readByte() {
  //TWI_WaitByteReceived(pTwi, RECV_TIMEOUT);
  while (!TWI_ByteReceived(pTwi))
    ;
  return TWI_ReadByte(pTwi);
}

static inline bool TWI_WaitTransferComplete(Twi *_twi, uint32_t _timeout) {
  while (!TWI_TransferComplete(_twi)) {
    if (TWI_FailedAcknowledge(_twi))
      return false;
    if (--_timeout == 0)
      return false;
  }
  return true;
}

static inline bool TWI_WaitByteReceived(Twi *_twi, uint32_t _timeout) {
  while (!TWI_ByteReceived(_twi)) {
    if (TWI_FailedAcknowledge(_twi))
      return false;
    if (--_timeout == 0)
      return false;
  }
  return true;
}

static inline bool TWI_FailedAcknowledge(Twi *pTwi) {
  return pTwi->TWI_SR & TWI_SR_NACK;
}











