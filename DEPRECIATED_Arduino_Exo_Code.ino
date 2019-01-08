#include <math.h>
#include "Libraries/Adafruit_MLX90614.h"

// Define Pins of the Arduino //
int potentiometerPin = A0;
int GSRPin = A1;

// Define Communication Variables //
float feedbackFreq = 1;
float feedbackTime = 0;
String outString;

// Main variables //
float fVariable = 0;
int iVariable = 0;

float fPotentiometer_Value = 0;
float fGSR_Value = 0;
Adafruit_MLX90614 mlx;
float fTemperature = 0;

// FUNCTION that reveives data from the serial port and assigns it to main variables
void setValues(){
  while(Serial.available()<=0){}	// Wait for serial port to become available
  
  fVariable = Serial.parseFloat();
  iVariable = Serial.parseInt();

  Serial.flush();	// Clear the serial port buffer
}

// FUNCTION that extends the string to be sent via the serial port
void addOutString(float inp){
  String addString = String(inp);             // Create a string from the function input
  addString = String(addString + ",");        // Add a comma as a seperator
  outString = String(outString + addString);  // Extend the outString
}

// FUNCTION that sends data via the serial port
void outputData(){
  Serial.flush();             // Clear the buffer of the serial port
  Serial.println(outString);  // Send the outString via the Serial port
  outString = "";             // Clear the outString (otherwise it would be endlessly extended)
}

// SETUP ROUTINE //
void setup()
{ 
  // Settings 
  mlx.begin();
  Serial.begin(9600);				// Start the serial port
  while(!Serial);					// Wait for serial port to run
  analogReference(INTERNAL2V56);	// Change internal voltage reference for force sensors
  feedbackTime = micros();			// Record an initial time for the feedback timer
}
//////////////////

//// MAIN LOOP ////
void loop()
{
  // Read the sensor values and assign them to the main variables
  fPotentiometer_Value = analogRead(potentiometerPin);	
  fGSR_Value = analogRead(GSRPin);	
  Temperature = mlx.readObjectTempC();
  
  // Run this at the feedback frequency
  //if (((feedbackTime - micros()) / 1000000) > (1/feedbackFreq))
  {
  	feedbackTime = micros();	// Reset the timer
  
  	addOutString(fPotentiometer_Value); // Add Potentiometer value
  	addOutString(fGSR_Value);           // Add GSR measurement
    addOutString(fTemperature);         // Add Temperature measurement
  	
  	outputData();				// Send the data via the serial port
  }
}
//////////////////
