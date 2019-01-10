using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO.Ports;
using Ardunity;

public class SerialReceiverClass : MonoBehaviour
{
    // Main contraints for hardware 
    float MIN_RANGE = 0;    // Minimum positon of the motors 
    float MAX_RANGE = 0.5;  // Maximum positon of the motors 
    float MAX_FORCE = 10;   // Maximum force of the motors

    // Sensor variables 
    public float fGSRValue = 0;             // [] Value of the GSR
    public float fTemperature = 0;          // [C] Measured Temperature
    public float fMAX_Assitance_Force = 0;  // [N] Maximum assistance force, set by the potentiometer    
    public float[] Force = {0, 0, 0, 0};

    // Communication variables
    float feedbackFreq = 4; // [1/s] Data is sent to the PC this many times per seconds
    SerialPort serial = new SerialPort("COM3", 9600);  // Define COM Port and Baudrate
    int iReadtimeout = 100; // [ms]; sets the timeout for the readout

    void setupArduino()
    {
        // Send the main constraints
        serial.WriteLine(feedbackFreq);
        serial.WriteLine(MIN_RANGE);
        serial.WriteLine(MAX_RANGE);
        serial.WriteLine(MAX_FORCE);
    }

    // SETUP ROUTINE //
    void Start()
   {
        if (serial.IsOpen)
        {
            serial.Close();   // If the serial port is still open, close it first
        }
        serial.Open();                      // (Re-)open the serial port
        serial.ReadTimeout = iReadtimeout;  // sets the timeout for the readout

       while (!serial.IsOpen) { }  // Wait for the serial port to open
    }

    //// MAIN LOOP ////
    void Update()
    {
        if(serial.IsOpen)
        {
            try
            {
                string[] input = serial.ReadLine().Split(',');  // read the serial port and split the buffer 

                fTemperature = float.Parse(input[0]);           // Temperature
                fGSRValue = float.Parse(input[1]);              // GSR
                fMAX_Assitance_Force = float.Parse(input[2]);   // Assistance Force / Potentiometer
                Force[0] = float.Parse(input[3]);               // Force Sensor 1   
                Force[1] = float.Parse(input[4]);               // Force Sensor 2
                Force[2] = float.Parse(input[5]);               // Force Sensor 3
                Force[3] = float.Parse(input[6]);               // Force Sensor 4

                serial.BaseStream.Flush();                      // clear the serial port buffer
            }
            catch (System.Exception) { }
   
            }

    }

    // QUIT ROUTINE //
    void OnApplicationQuit()
    {
        serial.Close(); // Close the serial port
    }

    // FUNCTION : Prints the received data to Console
    void Print_Data2Console()
    {
        Debug.LogErrorFormat(fMAX_Assitance_Force.ToString());
        Debug.LogErrorFormat(fGSRValue.ToString());
        Debug.LogErrorFormat(fTemperature.ToString());
        Debug.LogErrorFormat(Force[0].ToString());
        Debug.LogErrorFormat(Force[1].ToString());
        Debug.LogErrorFormat(Force[2].ToString());
        Debug.LogErrorFormat(Force[3].ToString());
    }

}

