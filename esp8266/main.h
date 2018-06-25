#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <esp8266.h>
#include <stdio.h>
#include <string.h>
#include <semphr.h>
#include <esp/spi.h>
#include <time.h>
#include <timers.h>
#include "queue.h"
#include "lwip/api.h"
#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>
#include <ssid_config.h>
#include <math.h>

#define MQTT_HOST "192.168.1.85"
#define MQTT_PORT 1883
#define MQTT_USER NULL
#define MQTT_PASS NULL
#define PUB_MSG_LEN 16

typedef struct{
	int laser1;
	int laser2;
	int laser3;
	int laser4;
	int laser5;
	int laser6;
	int laser7;
	int laser8;
	float a[3];
	float g[3];
	float m[3];
}sensor_data;

typedef struct{
	int laser1;
	int laser2;
	int laser3;
	int laser4;
	int laser5;
	int laser6;
	int laser7;
	int laser8;
	int yaw;
}process_data;

typedef struct {
	double q; //process noise covariance
	double r; //measurement noise covariance
	int x; //value
	double p; //estimation error covariance
	double k; //kalman gain
} kalman_state;

extern volatile process_data data_processed;
extern volatile sensor_data data;
extern volatile int print_data, data_to_send, data_to_process;
extern QueueHandle_t publish_queue;
extern SemaphoreHandle_t wifi_alive, sem_print_data, sem_data, sem_processed_data;

float madgwick_invSqrt(float x);
void madgwick_init(void);
void madgwick_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void madgwick_computeAngles(void);
float getYaw(void); 

void convertRawAcceleration(float * buf,int aRaw[3]);
void convertRawGyro(float * buf, int gRaw[3]);
void convertRawMag(float * buf, int mRaw[3]);
void topic_received(mqtt_message_data_t *md);
const char *  get_my_id(void);
sensor_data get_sensor_data(void);
kalman_state kalman_init(double q, double r, double p, double initial_value);
kalman_state kalman_update(kalman_state state,double medida);

void  mqtt_task(void *pvParameters);
void  wifi_task(void *pvParameters);
void blink(void *pvParameters);
void get_data(void *pvParameters);
void show_data(void *pvParameters);
void processing(void *pvParameters);
void send_data(void *pvParameters);
