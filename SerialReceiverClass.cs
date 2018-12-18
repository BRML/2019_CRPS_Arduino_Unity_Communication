using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO.Ports;
using Ardunity;

public class SerialReceiverClass : MonoBehaviour
{

    // Main vairables //
    public float fPotentiometerValue = 0;
    public float fGSRValue = 0;
    public float Temperature =0;

    //// Define communication vairables( //
    SerialPort serial = new SerialPort("COM3", 9600);  // Define COM Port and Baudrate
    int iReadtimeout = 100; // [ms]; sets the timeout for the readout

    //// SETUP ROUTINE //
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
       // Debug.LogErrorFormat(fPotentiometerValue.ToString());
      //  Debug.LogErrorFormat(fGSRValue.ToString());

        
    if(serial.IsOpen)
    {
        try
        {
            string[] input = serial.ReadLine().Split(',');  // read the serial port and split the buffer 
            fPotentiometerValue = float.Parse(input[0]);    // assign variable 1
            fGSRValue = float.Parse(input[1]);
            Temperature = float.Parse(input[2]);
            // assign variable 2
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

}

