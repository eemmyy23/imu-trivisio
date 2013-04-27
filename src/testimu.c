#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "trivisio/TrivisioColibri.h"
#include "trivisio/TrivisioConfig.h"
#include "trivisio/TrivisioTypes.h"

#include <stdio.h>
#ifdef WIN32
#	include <windows.h>
#else
#	include <unistd.h>
#endif

#define M_PI 3.14159265358979323846

#include "MadgwickAHRS.c"
#define RATE 1000
float accel[3];
float vx, vy, vz;
float dx, dy, dz;
int counter = 1;
void accel_rotate(float a,float b,float c,float d, float ax,float ay,float az);
void velocity(float ax, float ay, float az);
void displacement(float vx, float vy, float vz);
float THRESHOLD = 1;
void calibrateGravity(float ax, float ay, double raz);
int numCalibrate = 300;
double calibrateMed[3];//x y z
double calibrateGravityRotMed = 0;
double calibrateAccelXMed = 0;
double calibrateAccelYMed = 0;
double calibrateAccelXMax = 0;
double calibrateAccelYMax = 0;
double calibrateAccelXMin = 0;
double calibrateAccelYMin = 0;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "testimu");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  //ros::Publisher m_joint_pub = n.advertise<sensor_msgs::JointState>("/ptu/cmd", 1);

  ros::Rate loop_rate(10);

  int count = 0;

/**********************/
	struct TrivisioSensor sensorList[10];
	int sensorCount = colibriGetDeviceList(sensorList, 10);

        void* imu = colibriCreate(RATE);
	struct ColibriConfig conf;
	char ID[8];

	/* Diagonal matrices with diagonal element .68 yields approx 20Hz
    bandwidth @ 100Hz */
	float Ka[9] = { 0.68f,    0.00f,    0.00f,
	                0.00f,    0.68f,    0.00f,
	                0.00f,    0.00f,    0.68f };
	float Kg[9] = { 0.68f,    0.00f,    0.00f,
	                0.00f,    0.68f,    0.00f,
	                0.00f,    0.00f,    0.68f };

	struct TrivisioIMUData data;
	double oldt = 0;
	int i;

	printf("Number of Colibris found: %d\n", sensorCount);
	if (sensorCount<0)
		sensorCount = 10;
	for (i=0; i<sensorCount; ++i)
		printf("%s:\t %s (FW %d.%d)\n", sensorList[i].dev, sensorList[i].ID,
           sensorList[i].FWver, sensorList[i].FWsubver);
	printf("\n\n");

	if (sensorCount<1) {
		fprintf(stderr, "No Colibri sensors found\n");
		return 0;
	}
	if (colibriOpen(imu, 0, 0) < 0) {
		fprintf(stderr, "Error while trying to access Colibri\n");
		return -1;
	}

	colibriGetConfig(imu, &conf);
	conf.raw = 0;
        conf.freq = RATE;
	conf.sensor = 1023;
	conf.ascii = 0;
	colibriSetConfig(imu, &conf);

	colibriSetKa(imu, Ka);
	colibriSetKaStatus(imu, 1);
	colibriSetKg(imu, Kg);
	colibriSetKgStatus(imu, 1);
	colibriSetJitterStatus(imu, 1);

	printf("Colibri IMU\n");
	colibriGetID(imu, ID);
  printf("Device ID:        %s\n", ID);
  printf("Sensor config:    %d\n", conf.sensor);
	printf("Magnetic div:     %d\n", (unsigned)conf.magDiv);
	printf("Frequency:        %d\n", conf.freq);
	printf("ASCII output:     %d\n", conf.ascii);
	printf("autoStart:        %d\n", conf.autoStart);
	printf("RAW mode:         %d\n", conf.raw);
	printf("Jitter reduction: %d\n", colibriGetJitterStatus(imu));
	printf("\n\n");
	colibriStart(imu);

	while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

/*****************************/
		colibriGetData(imu, &data);
		if (data.t > oldt) {

			float eul[3];
                       // printf("\n\n ---- Time: %6.2f\t", data.t*1e-4);
                        //printf("Temp: %6.2f\t", data.temp);
                        //printf("\nAcc: %6.2f, %6.2f, %6.2f\t", data.acc_x, data.acc_y, data.acc_z);
                       // printf("\nGyr: %6.2f, %6.2f, %6.2f\t", data.gyr_x, data.gyr_y, data.gyr_z);
                        //printf("\nMag: %6.2f, %6.2f, %6.2f\t", data.mag_x, data.mag_y, data.mag_z);

                       //printf("Quat: %f, %f, %f, %f\t", data.q_w, data.q_x, data.q_y, data.q_z);
                       // colibriEulerOri(&data, eul);
                       // printf("Euler: %10.4f, %10.4f, %10.4f\n",180/M_PI*eul[0], 180/M_PI*eul[1], 180/M_PI*eul[2]);

                        //MadgwickAHRSupdate(data.gyr_x, data.gyr_y, data.gyr_z, data.acc_x, data.acc_y, data.acc_z, data.mag_x, data.mag_y, data.mag_z);
                        // printf("\n\nq0:%f\t\tq1:%f\t\tq2:%f\t\tq3:%f\t\t",q0,q1,q2,q3);
                        //accel_rotate(data.q_w, data.q_x, data.q_y, data.q_z,data.acc_x, data.acc_y, data.acc_z);

                        //velocity(accel[0], accel[1], accel[2]);

                        oldt = data.t;
		}

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

	colibriStop(imu);
	colibriClose(imu);
  return 0;
}

//compensate the accelerometer readings from gravity.
//@param q the quaternion representing the orientation of a 9DOM MARG sensor array
//@param acc the readings coming from an accelerometer expressed in g
//
//@return a 3d vector representing dinamic acceleration expressed in g
//void accel_rotate(float a,float b,float c,float d, float ax,float ay,float az){
void accel_rotate(float q0,float q1,float q2,float q3, float ax,float ay,float az){

    double rax,ray,raz;

//    if(numCalibrate < 0){ //if the calibration si done (after 5 sec)
//    ax -= calibrateAccelXMed;
//    ay -= calibrateAccelYMed;
//    }


//    rax= (a*a+b*b-c*c-d*d) * ax + (2*b*c-2*a*d)     * ay + (2*b*d+2*a*c)     * az;
//    ray= (2*b*c+2*a*d)     * ax + (a*a-b*b+c*c-d*d) * ay + (2*c*d+2*a*b)     * az;
//    raz= (2*b*d-2*a*c)     * ax + (2*c*d-2*a*b)     * ay + (a*a-b*b-c*c+d*d) * az;

        rax= (2*q0*q0 - 1 + 2*q1*q1 ) * ax + 2 * (q1*q2+q0*q3)     * ay + 2* (q1*q3 - q0*q2)     * az;
        ray= 2*(q1*q2-q0*q3)     * ax + (2*q0*q0-1+2*q2*q2) * ay + 2*(q2*q3+q0*q1)     * az;
        raz= 2*(q1*q3+q0*q2)     * ax + 2*(q2*q3 - q0*q1)     * ay + (2*q0*q0-1+2*q3*q3) * az;

//calibrateGravity( rax,  ray,  raz);


if(rax < THRESHOLD && rax > -THRESHOLD) rax=0; else  printf("%+6.6f\n", rax);
//if(ray < THRESHOLD && ray > -THRESHOLD) ray=0;
//if(raz < THRESHOLD && raz > -THRESHOLD) raz=0;


//    if(numCalibrate < 0){ //if the calibration si done (after 5 sec)
//    //raz -= calibrateGravityRotMed;

//    }


//
        //printf("%+6.6f\n", rax);

//printf("\nRot: %6.4f, %6.4f, %6.4f\t", rax, ray, raz);
//velocity(rax, ray, raz);
//displacement_instant(rax, ray, raz);
    //rax -= calibrateMed[0];
   // ray -= calibrateMed[1]
    //raz -= calibrateMed[2]

}
void velocity(float ax, float ay, float az){

    counter+=1;

    float Dvx, Dvy, Dvz;
    Dvx = ax * 1/RATE;
    Dvy = ay * 1/RATE;
    Dvz = az * 1/RATE;
    printf("\n\ndelta vx:%f\t\tdelta vy:%f\t\tdelta vz:%f\t\t",Dvx,Dvy,Dvz);


    vx+= Dvx;
    vy+= Dvy;
    vz+= Dvz;
    //printf("\n\n vx:%f\t\tvy:%f\t\tvz:%f\t\t",vx,vy,vz);
    if (counter == RATE+1){
       // printf("accelllllllllllllll");
    //    displacement( vx,  vy,  vz);

        counter = 1;
    }



}

void displacement(float vx, float vy, float vz){

    float Ddx, Ddy, Ddz;
    Ddx = vx * 1;
    Ddy = vy * 1;
    Ddz = vz * 1;
    //printf("\n\ndelta dx:%f\t\tdelta dy:%f\t\tdelta dz:%f\t\t",Ddx,Ddy,Ddz);

    dx+= Ddx;
    dy+= Ddy;
    dz+= Ddz;

    printf("\n\ndelta dx:%f\t\tdelta dy:%f\t\tdelta dz:%f\t\t",dx,dy,dz);


}


void displacement_instant(float ax, float ay, float az){

    float Dvx, Dvy, Dvz;
    Dvx = ax * 1/RATE;
    Dvy = ay * 1/RATE;
    Dvz = az * 1/RATE;
    printf("\n\ndelta vx:%f\t\tdelta vy:%f\t\tdelta vz:%f\t\t",Dvx,Dvy,Dvz);
    vx+= Dvx;
    vy+= Dvy;
    vz+= Dvz;
    printf("\n\nvx:%f\t\t vy:%f\t\t vz:%f\t\t",Dvx,Dvy,Dvz);

    float Ddx, Ddy, Ddz;
    Ddx = vx * 1/RATE;
    Ddy = vy * 1/RATE;
    Ddz = vz * 1/RATE;
    //printf("\n\ndelta dx:%f\t\tdelta dy:%f\t\tdelta dz:%f\t\t",Ddx,Ddy,Ddz);

    dx+= Ddx;
    dy+= Ddy;
    dz+= Ddz;

    printf("\n\ndelta dx:%f\t\tdelta dy:%f\t\tdelta dz:%f\t\t",dx,dy,dz);

    printf("\n\n");
}

void calibrateGravity(float ax, float ay, double raz){

    if(numCalibrate > 0 ){
        //printf("calibrating");
        numCalibrate--;
        calibrateAccelXMed += ax;
        calibrateAccelYMed += ay;
        calibrateGravityRotMed += raz;
        if (ax > calibrateAccelXMax) calibrateAccelXMax = ax;
        if (ax < calibrateAccelXMin) calibrateAccelXMin = ax;
        if (ay > calibrateAccelYMax) calibrateAccelYMax = ay;
        if (ay < calibrateAccelYMin) calibrateAccelYMin = ay;
    }
    else if (numCalibrate == 0){
        printf("done calibrating");
        //numCalibrate--;
        numCalibrate=300;
        calibrateAccelXMed /= 500;
        calibrateAccelYMed /= 500;
        calibrateGravityRotMed /= 500;
        printf("axMax=%f\taxMin=%f\tayMax=%f\tayMin=%f\r\n",calibrateAccelXMax,calibrateAccelXMin,calibrateAccelYMax,calibrateAccelYMin);
         calibrateAccelXMax = 0;
         calibrateAccelYMax = 0;
         calibrateAccelXMin = 0;
         calibrateAccelYMin = 0;
    }

}
