using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO.Ports;
using Ardunity;
using UnityEngine.UI;

public class SerialReceiverClass : MonoBehaviour
{
    public Slider max;
    public Slider min;
    // Main contraints for hardware 
    float MIN_RANGE = 0;    // Minimum positon of the motors 
    float MAX_RANGE = 0.05f;  // Maximum positon of the motors 
    float MAX_FORCE = 10;   // Maximum force of the motors

    // Sensor variables 
    public static float fGSRValue = 0;             // [] Value of the GSR
    public static float fTemperature = 0;          // [C] Measured Temperature
    public static float fMAX_Assitance_Force = 0;  // [N] Maximum assistance force, set by the potentiometer    
    public static float[] Force = { 0, 0, 0, 0 };

    // Communication variables
    float feedbackFreq = 1f; // [1/s] Data is sent to the PC this many times per seconds
    SerialPort stream = new SerialPort("COM3", 9600);  // Define COM Port and Baudrate
    int iReadtimeout = 40; // [ms]; sets the timeout for the readout

    public void Awake()
    {
        DontDestroyOnLoad(this);
    }

    
    public void WriteToArduino(string message)
    {
        stream.WriteLine(message);
        stream.BaseStream.Flush();
    }

    void setupArduino()
    {
        WriteToArduino("PING");
        // Send the main constraints
        // stream.WriteLine("PING ");
        //stream.WriteLine(feedbackFreq.ToString());
        //stream.WriteLine(MIN_RANGE.ToString());
        //stream.WriteLine(MAX_RANGE.ToString());
        //stream.WriteLine(MAX_FORCE.ToString());
    }

    // SETUP ROUTINE //
    void Start()
    {
        if (stream.IsOpen)
        {
            stream.Close();   // If the serial port is still open, close it first
        }
        stream.Open();                      // (Re-)open the serial port
        stream.ReadTimeout = iReadtimeout;  // sets the timeout for the readout

        //while (!stream.IsOpen) {
        //}  // Wait for the serial port to open

        setupArduino();

    }



    //// MAIN LOOP ////
    void Update()
    {
        //Debug.LogErrorFormat("running");
        if (stream.IsOpen)
            Debug.LogErrorFormat("stream is open");
            Debug.LogErrorFormat(stream.ReadLine());

        StartCoroutine
            (
                AsynchronousReadFromArduino
                ((string s) => Debug.LogErrorFormat("stream read:" + s),     // Callback
                   
                    () => Debug.LogError("Error!"), // Error callback
                    10000f                          // Timeout (milliseconds)
                )
            );
        setupArduino(); // test write the serial port
        {
            try
            {
             


                string[] input = stream.ReadLine().Split(',');  // read the serial port and split the buffer 

                fTemperature = float.Parse(input[0]);           // Temperature
                fGSRValue = float.Parse(input[1]);              // GSR
                fMAX_Assitance_Force = float.Parse(input[2]);   // Assistance Force / Potentiometer
                Force[0] = float.Parse(input[3]);               // Force Sensor 1   
                Force[1] = float.Parse(input[4]);               // Force Sensor 2
                Force[2] = float.Parse(input[5]);               // Force Sensor 3
                Force[3] = float.Parse(input[6]);               // Force Sensor 4

                stream.BaseStream.Flush();                      // clear the serial port buffer
            }
            catch (System.Exception) { }

        }
        Print_Data2Console();
        
    }

    public IEnumerator AsynchronousReadFromArduino(Action<string> callback, Action fail = null, float timeout = float.PositiveInfinity)
    {
        DateTime initialTime = DateTime.Now;
        DateTime nowTime;
        TimeSpan diff = default(TimeSpan);

        string dataString = null;

        do
        {
            try
            {
                dataString = stream.ReadLine();
            }
            catch (TimeoutException)
            {
                dataString = null;
            }

            if (dataString != null)
            {
                callback(dataString);
                yield break; // Terminates the Coroutine
            }
            else
                yield return null; // Wait for next frame

            nowTime = DateTime.Now;
            diff = nowTime - initialTime;

        } while (diff.Milliseconds < timeout);

        if (fail != null)
            fail();
        yield return null;
    }

    public void changeminvalue()
    {
        
        MIN_RANGE = min.value;
       // Debug.LogErrorFormat("Minrange:" + MIN_RANGE);
        setupArduino();
    }

    public void changemaxvalue()
    {
        MAX_RANGE = max.value;
       // Debug.LogErrorFormat("maxrange:" + MAX_RANGE);
        setupArduino();
    }

    // QUIT ROUTINE //
    void OnApplicationQuit()
    {
        stream.Close(); // Close the serial port
    }

    // FUNCTION : Prints the received data to Console
    void Print_Data2Console()
    {
        //Debug.LogErrorFormat("slider"+fMAX_Assitance_Force.ToString());
        //Debug.LogErrorFormat(fGSRValue.ToString());
       // Debug.LogErrorFormat("Temperature"+fTemperature.ToString());
       // Debug.LogErrorFormat(Force[0].ToString());
       // Debug.LogErrorFormat(Force[1].ToString());
       // Debug.LogErrorFormat(Force[2].ToString());
       // Debug.LogErrorFormat(Force[3].ToString());
    }

}

