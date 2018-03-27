#include<stdio.h> //printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include <sys/socket.h>
#include<arpa/inet.h>

typedef struct{
	int laser1;
	int laser2;
	int laser3;
	int laser4;
	int laser5;
	int laser6;
	int laser7;
	int laser8;
	int laser9;
	int laser10;
	int imu_yaw;
	int imu_pitch;
	int imu_roll;
}sensor_data;
