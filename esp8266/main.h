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

#define MQTT_HOST "192.168.1.40"
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
	int laser9;
	int laser10;
	int imu_yaw;
	int imu_pitch;
	int imu_roll;
}sensor_data;

extern volatile sensor_data data;
extern volatile int print_data;
extern QueueHandle_t publish_queue;
extern SemaphoreHandle_t wifi_alive, sem_print_data, sem_data;

void topic_received(mqtt_message_data_t *md);
const char *  get_my_id(void);
void  mqtt_task(void *pvParameters);
void  wifi_task(void *pvParameters);
void blink(void *pvParameters);
sensor_data get_sensor_data(void);
void get_data(void *pvParameters);
void show_data(void *pvParameters);
void slam_process(void *pvParameters);
void send_data(void *pvParameters);
