using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO.Ports;

public class Exo_Wrist_Communication_Class : MonoBehaviour
{

    ///// HARDWARE SETUP /////
    int num_Joints = 3;         // Define the number of Joints of the exoskeleton, the wrist has three

    public float flexZero = 0.035f;
    public float radialZero = 0.045f;
    public float supinZero = 0.00f;
    public float strokeLength = 0.1f;
    public float speed = 4f;
    //////////////////////////

    public struct sJoint {
        public int isOn;
        public int[] sensor_dir; // direction of the attached sensor
        public float mass;       // [kg]  Virtual mass of the admittance control scheme
        public float damping;    // [N*s/(deg or m)] Virtual damping of the admittance control scheme
        public float x;          // [deg or m] position of the motor
        public float min_pos;    // [deg or m] min position of the motor
        public float max_pos;    // [deg or m] max position of the motor

        public float assistance_force;
        public float desired_pos;
        public float sensor_offset;
        public float max_force;
        public float min_vel;
        public float assist_speed;
        public float radius;
        public int DOUBLE_actuated;
        public int NONLINEAR_DOUBLE_actuated;

        // Average variables for unity output
        public float force_average;
        public float dx_average;
        public float ddx_average;
        public float assistance_force_average;
    }

    sJoint[] joints = new sJoint[num_Joints];
 
    // Communication variables
	float feedbackFreq = 1;
	float assist_scale = 8;           // maximum assistance force when slider is at 100%
	int controllerAttached = 1;
	int runCalib = 1;

    // Measurement variables
    float AverageTemp=0;
	float AverageGSR=0;

    SerialPort sp = new SerialPort("COM3",9600);

    // Use this for initialization
    void Start()
    {
        gameObject.GetComponent<Renderer>().material.color = new Color(0.0f, 0.0f, 0.0f);
        if (sp.IsOpen)
        {
            sp.Close();
        }
        sp.Open();
        sp.ReadTimeout = 100;
        //sp.BaseStream.Flush();
        while (!sp.IsOpen) { }
        setupVariables();
        setupArduino();
        runCalib = 0;
    }
	
	// Update is called once per frame
	void Update () {
	 
	if (sp.IsOpen){
            try
            {
                string[] input = sp.ReadLine().Split(',');

                AverageTemp = float.Parse(input[0]);
                AverageGSR = float.Parse(input[1]);

                joints[0].force_average = float.Parse(input[2]);
                joints[0].assistance_force_average = float.Parse(input[3]);

                joints[1].force_average = float.Parse(input[4]);
                joints[1].assistance_force_average = float.Parse(input[5]);

                joints[2].force_average = float.Parse(input[6]);
                joints[2].assistance_force_average = float.Parse(input[7]);

			    sp.BaseStream.Flush();			
		    }
		catch(System.Exception){	
		}		
	}
		
	}
	void setupVariables(){
	// define flexion values
		joints[0].isOn=1;
        joints[0].sensor_dir[0] = 1;  // direction of the attached sensor
        joints[0].mass=5;             // [kg]  Virtual mass of the admittance control scheme
		joints[0].damping=200;        // [N*s/(deg or m)] Virtual damping of the admittance control scheme
		joints[0].min_pos=0;          // [deg or m] min position of the motor
		joints[0].max_pos=0.1f;       // [deg or m] max position of the motor
		joints[0].max_force=25;
		joints[0].min_vel=0.01f;
		joints[0].assist_speed=0.01f;
		joints[0].assistance_force=0;
		joints[0].DOUBLE_actuated=0;
        joints[0].NONLINEAR_DOUBLE_actuated = 0;
        joints[0].radius=1;
	
	// define radial values
		joints[1].isOn=1;
        joints[1].sensor_dir[0] = 1; // direction of the attached sensor
        joints[1].mass=1;            // [kg]  Virtual mass of the admittance control scheme
		joints[1].damping=100;       // [N*s/(deg or m)] Virtual damping of the admittance control scheme
		joints[1].min_pos=0;         // [deg or m] min position of the motor
		joints[1].max_pos=0.07f;     // [deg or m] max position of the motor
		joints[1].max_force=25;
		joints[1].min_vel=0.01f;
		joints[1].assist_speed=0.01f;
		joints[1].assistance_force=0;
		joints[1].DOUBLE_actuated=0;
        joints[1].NONLINEAR_DOUBLE_actuated = 0;
        joints[1].radius=1;

	// define supination values
		supin.isOn=1;
        joints[2].sensor_dir[0] = 1; // direction of the attached sensor
        joints[2].mass=0.001f;       // [kg]  Virtual mass of the admittance control scheme
		joints[2].damping=0.005f;    // [N*s/(deg or m)] Virtual damping of the admittance control scheme
		joints[2].min_pos=-70;       // [deg or m] min position of the motor
		joints[2].max_pos=70;        // [deg or m] max position of the motor

		joints[2].max_force=25;
		joints[2].min_vel=0.1f;
		joints[2].assist_speed=0.01f;
		joints[2].DOUBLE_actuated=1;
        joints[2].NONLINEAR_DOUBLE_actuated = 1;
		joints[2].radius=0.0565f;
		joints[2].sensor_dir[1]=-1;
		joints[2].assistance_force=0;
	}

	void setupArduino(){
		sp.WriteLine(feedbackFreq.ToString());
		sp.WriteLine(controllerAttached.ToString());
		sp.WriteLine(runCalib.ToString());
		sp.WriteLine(assist_scale.ToString());

		// Flexion
		sp.WriteLine(joints[0].isOn.ToString());
		sp.WriteLine(joints[0].mass.ToString());
		sp.WriteLine(joints[0].damping.ToString());
		sp.WriteLine(joints[0].min_pos.ToString());
		sp.WriteLine(joints[0].max_pos.ToString());
		sp.WriteLine(joints[0].sensor_dir[0].ToString());
		sp.WriteLine(joints[0].max_force.ToString());
		sp.WriteLine(joints[0].min_vel.ToString());
		sp.WriteLine(joints[0].assist_speed.ToString());
		sp.WriteLine(joints[0].DOUBLE_actuated.ToString());
        sp.WriteLine(joints[0].NONLINEAR_DOUBLE_actuated.ToString());

        // Radial
        sp.WriteLine(joints[1].isOn.ToString());
		sp.WriteLine(joints[1].mass.ToString());
		sp.WriteLine(joints[1].damping.ToString());
		sp.WriteLine(joints[1].min_pos.ToString());
		sp.WriteLine(joints[1].max_pos.ToString());
		sp.WriteLine(joints[1].sensor_dir[0].ToString());
		sp.WriteLine(joints[1].max_force.ToString());
		sp.WriteLine(joints[1].min_vel.ToString());
		sp.WriteLine(joints[1].assist_speed.ToString());
		sp.WriteLine(joints[1].DOUBLE_actuated.ToString());
        sp.WriteLine(joints[1].NONLINEAR_DOUBLE_actuated.ToString());

        // Supination
        sp.WriteLine(joints[2].isOn.ToString());
		sp.WriteLine(joints[2].mass.ToString());
		sp.WriteLine(joints[2].damping.ToString());
		sp.WriteLine(joints[2].min_pos.ToString());
		sp.WriteLine(joints[2].max_pos.ToString());
		sp.WriteLine(joints[2].sensor_dir[0].ToString());
		sp.WriteLine(joints[2].max_force.ToString());
		sp.WriteLine(joints[2].min_vel.ToString());
		sp.WriteLine(joints[2].assist_speed.ToString());
		sp.WriteLine(joints[2].DOUBLE_actuated.ToString());
        sp.WriteLine(joints[2].NONLINEAR_DOUBLE_actuated.ToString());

		sp.WriteLine(joints[2].radius.ToString());
		sp.WriteLine(joints[2].sensor_dir[1].ToString());       
    }

	void OnApplicationQuit(){
		sp.Close();
	}

}

