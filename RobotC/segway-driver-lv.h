#include "RobotC_MPU.c"
#include "Kalman.h"

int steering = 0;
int acceleration = 2500;
int speed = 0;

int delta = 1000;
long g_module = 70600;

Kalman kalman_GyY;

//Bluetooth varible
const int kPacketSize = 14;  // Number of bytes in command
ubyte BytesRead[kPacketSize]; // Circular buffer of last bytes read.
int nNumbBytesRead;

byte motorA_speed = 0;
byte motorB_speed = 0;

//GLOBAL VARIABLE SETUP
float gn_dth_dt,gn_th,gn_y,gn_dy_dt,kp,ki,kd,gear_down_ratio,dt;
float AcX, AcY, AcZ, GyX, GyY, GyZ;

void readRawBluetoothData();

void balancing()
{
  // Set gearing down ratio between motor and wheels (e.g. 5x slow down: 40z / 8z = 5)
  // The default is 1, no gearing down.
  gear_down_ratio = 1;

  // Set the time each loop cycle should last. You can set it up to 0.03 seconds or even higher, if you really
  // need to. If you add code to the control loop below (such as to read another sensor), make sure that
  // all code can run in under dt seconds. If it takes more time, set dt to a higher value.
  // Default is 0.010 seconds (10 miliseconds).
  dt = 0.010;

  // Customize PID constants. These variables are global, so you can optionally dynamically change them in your main task.
  gn_dth_dt = 0.23;	//0.20 - low power; //0.28 - high power
  gn_th = 15.00; //25
  gn_y = 172.8;
  gn_dy_dt = 24.6; //24.6
  kp = 0.109146; 		//0.1346 - low power; //0.09146; - high power
  ki = 0.6188;	//0.8188;			//0.6688 - low power; //0.2188 - high power
  kd = 0.000984;		//0.001904 - low power; //0.000984 - high power

  //MOTOR SETUP
  nMotorPIDSpeedCtrl[motorB] = mtrNoReg;
  nMotorPIDSpeedCtrl[motorA] = mtrNoReg;
  nMotorEncoder[motorA] = 0;
  nMotorEncoder[motorB] = 0;

  //SENSOR SETUP
  int nSensorsDefined = 0;

	configMPU();
	wait1Msec(500);
	nSensorsDefined++;

	if(nSensorsDefined != 1){
	  nxtDisplayTextLine(0,"Check Sensor");
	  nxtDisplayTextLine(1,"definition!");
	  wait1Msec(5000);StopAllTasks();
	}

  //MATH CONSTANTS
  const float radius = your_wheel_diameter/1000;
  const float degtorad = PI/180;

  float GyY_prev = 0;
  float AcZ_prev = 0;
	float AcX_prev = 0;

  //SETUP VARIABLES FOR CALCULATIONS
  float th = 0,//Theta            // Angle of robot (degree)
  			th_kalman = 0,
  			th_complementary = 0,
  			th_prev = 0,
        dth_dt = 0;//dTheta/dt    // Angular velocity of robot (degree/sec)
  float e = 0,//Error             // Sum of four states to be kept zero: th, dth_dt, y, dy_dt.
        de_dt = 0,//dError/dt     // Change of above error
        _edt = 0,//Integral Error // Accumulated error in time
        e_prev = 0;//Previous Error/ Error found in previous loop cycle
  float pid = 0;                  // SUM OF PID CALCULATION
  float y = 0,//y                     // Measured Motor position (degrees)
        dy_dt = 0,//dy/dt             // Measured motor velocity (degrees/sec)
	      v = 0,//velocity          // Desired motor velocity (degrees/sec)
	      y_ref = 0;//reference pos // Desired motor position (degrees)
  int motorpower = 0,             // Power ultimately applied to motors
      last_steering = 0,          // Steering value in previous cycle
      straight = 0,               // Average motor position for synchronizing
      d_pwr = 0;                  // Change in power required for synchronizing
  const int n_max = 7;            // Number of measurement used for floating motor speed average
  int n = 0,n_comp = 0,           // Intermediate variables needed to compute measured motor speed
  encoder[n_max];                 // Array containing last n_max motor positions
  memset(&encoder[0],0,sizeof(encoder));
  //starting_balancing_task = false;// We're done configuring. Main task now resumes.


  ClearTimer(T4);                 // This timer is used in the driver. Do not use it for other purposes!

  kalman_init(kalman_GyY);

  while(true)
  {

    //READ GYRO SENSOR
  	getMotion(AcX, AcY, AcZ, GyX, GyY, GyZ);

  	//READ BLUETOOTH COMMAND
  	readRawBluetoothData();

  	//low pass filter
  	/*float alpha = 0.59;
  	GyY = GyY_prev + alpha*(GyY - GyY_prev);

  	AcZ = AcZ_prev + alpha*(AcZ - AcZ_prev);
  	AcX = AcX_prev + alpha*(AcX - AcX_prev);
  	AcZ_prev = AcZ;
  	AcX_prev = AcX;*/

		//COMPUTE GYRO ANGULAR VELOCITY AND ESTIMATE ANGLE
  	AcZ += 0.40; //offset
  	float th_accel = atan(AcZ/AcX) * 180.0 / PI;
  	th += 90.0*(GyY + GyY_prev)/25000.0;

    //th_kalman = getAngle(kalman_GyY, th_accel, GyY, dt); //kalman filter

    //Complementary filter. module=70155
    /*th_complementary += 90.0*(GyY + GyY_prev)/25000.0;
    long acc_module = AcZ*AcZ + AcX*AcX;
    if(acc_module > (g_module-delta) && acc_module < g_module+delta)
    	th_complementary = 0.99*th_complementary + 0.01*th_accel;*/

    dth_dt = GyY;//(th - th_prev) / dt;
    th_prev = th;
    GyY_prev = GyY;

    //compare filters
    //writeDebugStream("%f; %f; %f; %f\n", nSysTime, th, th_kalman, th_complementary);

    //ADJUST REFERENCE POSITION ON SPEED AND ACCELERATION
    if(v < speed*2){
    	v = v + acceleration*dt;
    }
    else if(v > speed*2){
    	v = v - acceleration*dt;
    }

    y_ref = y_ref + v*dt;
    //writeDebugStream("v=%f, speed=%d, yref=%f\n", v, speed, y_ref);

  	//COMPUTE MOTOR ENCODER POSITION AND SPEED
  	n++;if(n == n_max){n = 0;}
  	encoder[n] = nMotorEncoder[motorB] + nMotorEncoder[motorC] + y_ref;
  	n_comp = n+1;if(n_comp == n_max){n_comp = 0;}
  	y = encoder[n]*degtorad*radius/gear_down_ratio;
  	dy_dt = (encoder[n] - encoder[n_comp])/(dt*(n_max-1))*degtorad*radius/gear_down_ratio;

  	//COMPUTE COMBINED ERROR AND PID VALUES
  	e = gn_th*th + gn_dth_dt*dth_dt + gn_y*y + gn_dy_dt*dy_dt;
  	de_dt = (e - e_prev)/dt;
  	_edt = _edt + e*dt;
  	e_prev = e;

  	//writeDebugStream("th=%f dth_dt=%f y=%f dy_dt=%f\n", gn_th * th, gn_dth_dt * dth_dt, gn_y * y, gn_dy_dt * dy_dt );
  	//writeDebugStream("e=%f de_dt=%f\n", kp*e, kd*de_dt);
  	pid = (kp*e + ki*_edt + kd*de_dt)/radius*gear_down_ratio;

  	//time; gn_th*th; gn_dth_dt*dth_dt; gn_y*y; gn_dy_dt*dy_dt; kp*e; ki*_edt; kd*de_dt; pid;
  	//writeDebugStreamLine("%d; %f; %f; %f; %f; %f; %f; %f; %f", nSysTime, gn_th*th, gn_dth_dt*dth_dt, gn_y*y, gn_dy_dt*dy_dt, kp*e, ki*_edt, kd*de_dt, pid);

  	//ADJUST MOTOR SPEED TO STEERING AND SYNCHING
    if(steering == 0){
     	if(last_steering != 0){
	    	straight = nMotorEncoder[motorA] - nMotorEncoder[motorB];
	   	}
		  d_pwr = (nMotorEncoder[motorA] - nMotorEncoder[motorB] - straight)/(radius*10/gear_down_ratio);
		}
    else{
    	d_pwr = steering/(radius*10/gear_down_ratio);
    }
    last_steering = steering;

  	//CONTROL MOTOR POWER AND STEERING
  	motorpower = 	pid;
    motor[motorB] = motorpower + d_pwr;
    motor[motorA] = motorpower - d_pwr;

    //ERROR CHECKING OR SHUTDOWN
    if(abs(th)>60 || abs(motorpower) > 2000){
      StopAllTasks();}

    //WAIT THEN REPEAT
  	//while(time1[T4] < dt*1000){
  	//  wait1Msec(10);}
  	ClearTimer(T4);
  }
}


//Bluetooth functions
void readRawBluetoothData()
{
		if (nBTCurrentStreamIndex < 0) //// if there is no currently an open Bluetooth connection:
	  	return;

	  nNumbBytesRead = nxtReadRawBluetooth(BytesRead, kPacketSize);

	  if (nNumbBytesRead != 14)
	  	return;

	  byte motor_port = BytesRead[4];
	  if(motor_port == 0x01)
	  	motorA_speed = BytesRead[5];
	 	else if(motor_port == 0x02)
	  	motorB_speed = BytesRead[5];


	  if(motorA_speed < 0 && motorB_speed < 0){ //move forward
  		speed = motorA_speed;
  		steering = 0;
  	}
  	else if(motorA_speed > 0 && motorB_speed > 0) //move backward
  	{
  		speed = motorA_speed;
  		steering = 0;
  	}
  	else if(motorA_speed > 0 && motorB_speed < 0) //move left
  	{
  		speed = motorA_speed;
  		steering = -10;
  	}
  	else if(motorA_speed < 0 && motorB_speed > 0) //move right
  	{
  		speed = motorA_speed;
  		steering = 10;
  	}
  	else
  	{
  		speed = motorA_speed;
  		steering = 0;
  	}
}
