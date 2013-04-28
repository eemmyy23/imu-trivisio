#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
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


#define RATE 100
float accel[3];
float vx, vy, vz;
float dx, dy, dz;
int counter = 1;
void accel_rotate(float a,float b,float c,float d, float ax,float ay,float az);
void velocity(float ax, float ay, float az);
void displacement(float vx, float vy, float vz);
float THRESHOLD = 0;
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


ros::Publisher imu_pub;
sensor_msgs::Imu imu_msg;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testimu");
  ros::NodeHandle n;
  imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 100);
  ros::Rate loop_rate(100);
  int count = 0;
  // Fill constant parts of IMU Message
  // No covariance yet for linear acceleration
  for (sensor_msgs::Imu::_angular_velocity_covariance_type::iterator ita = imu_msg.linear_acceleration_covariance.begin(); ita != imu_msg.linear_acceleration_covariance.end(); ita++)
  {
    *ita = 0.0;
  }

  // No covariance yet for rotation
  for (sensor_msgs::Imu::_orientation_covariance_type::iterator ito = imu_msg.orientation_covariance.begin(); ito != imu_msg.orientation_covariance.end(); ito++)
  {
   *ito = 0.0;
  }
  // No angular velocity at all
  imu_msg.angular_velocity_covariance[0] = -1.0;

/**********************/
  struct TrivisioSensor sensorList[10];
  int sensorCount = colibriGetDeviceList(sensorList, 10);
  void* imu = colibriCreate(RATE);
  struct ColibriConfig conf;
  char ID[8];
/* Diagonal matrices with diagonal element .68 yields approx 20Hz bandwidth @ 100Hz */
/*
	float Ka[9] = { 0.68f,    0.00f,    0.00f,
	                0.00f,    0.68f,    0.00f,
	                0.00f,    0.00f,    0.68f };
	float Kg[9] = { 0.68f,    0.00f,    0.00f,
	                0.00f,    0.68f,    0.00f,
	                0.00f,    0.00f,    0.68f };
*/
  struct TrivisioIMUData data;
  double oldt = 0;
  int i;
  printf("Number of Colibris found: %d\n", sensorCount);
  if (sensorCount<0)
    sensorCount = 10;
  for (i=0; i<sensorCount; ++i)
    printf("%s:\t %s (FW %d.%d)\n", sensorList[i].dev, sensorList[i].ID, sensorList[i].FWver, sensorList[i].FWsubver);
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
/*
  conf.raw = 0;
  conf.freq = RATE;
  conf.sensor = (ColibriConfig::Sensor) 1023;
  conf.ascii = 0;
  colibriSetConfig(imu, &conf);
  colibriSetKa(imu, Ka);
  colibriSetKaStatus(imu, 1);
  colibriSetKg(imu, Kg);
  colibriSetKgStatus(imu, 1);
  colibriSetJitterStatus(imu, 1);
*/
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
  
    colibriGetData(imu, &data);
    if (data.t > oldt) 
		{
/*
			float eul[3];
                        printf("\n\n ---- Time: %6.2f\t", data.t*1e-4);
                        printf("Temp: %6.2f\t", data.temp);
                        printf("\nAcc: %6.2f, %6.2f, %6.2f\t", data.acc_x, data.acc_y, data.acc_z);
                       	printf("\nGyr: %6.2f, %6.2f, %6.2f\t", data.gyr_x, data.gyr_y, data.gyr_z);
                       	printf("\nMag: %6.2f, %6.2f, %6.2f\t", data.mag_x, data.mag_y, data.mag_z);

                       	printf("Quat: %f, %f, %f, %f\t", data.q_w, data.q_x, data.q_y, data.q_z);
                        colibriEulerOri(&data, eul);
                        printf("Euler: %10.4f, %10.4f, %10.4f\n",180/M_PI*eul[0], 180/M_PI*eul[1], 180/M_PI*eul[2]);

*/
												

			/* IMU */
    	imu_msg.header.frame_id = "FrameIMU";
    	imu_msg.header.stamp = ros::Time::now();
			// IMU - Acc
    	imu_msg.linear_acceleration.x = data.acc_x;
    	imu_msg.linear_acceleration.y = data.acc_y;
    	imu_msg.linear_acceleration.z = data.acc_z;
			// IMU - Gyr
    	imu_msg.angular_velocity.x = data.gyr_x;
    	imu_msg.angular_velocity.y = data.gyr_y;
    	imu_msg.angular_velocity.z = data.gyr_z;
			// IMU - Quat
    	imu_msg.orientation.x = data.q_x;
    	imu_msg.orientation.y = data.q_y;
    	imu_msg.orientation.z = data.q_z;
    	imu_msg.orientation.w = data.q_w;
			//publish    	
			imu_pub.publish(imu_msg);
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

