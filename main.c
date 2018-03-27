#include <main.h>
#include <CoreSLAM.h>

volatile sensor_data data;
volatile int print_data = 0;
SemaphoreHandle_t wifi_alive, sem_print_data, sem_data;
QueueHandle_t publish_queue;

void user_init(void)
{
    BaseType_t xReturned;
    TaskHandle_t xHandle_spi = NULL,xHandle_blink = NULL, xHandle_showdata = NULL, xHandle_wifi_task = NULL, xHandle_mqtt_task = NULL, xHandle_SLAM = NULL;
	vSemaphoreCreateBinary(sem_print_data);
	vSemaphoreCreateBinary(sem_data);
	vSemaphoreCreateBinary(wifi_alive);
    publish_queue = xQueueCreate(3, PUB_MSG_LEN);	

    uart_set_baud(0, 115200);
    spi_init(1, SPI_MODE0, SPI_FREQ_DIV_1M, 1, SPI_BIG_ENDIAN, true); // init SPI module

    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    xReturned = xTaskCreate(&get_data, "get_data", 1024, NULL, 4 | portPRIVILEGE_BIT, &xHandle_spi);
	if(xReturned != pdPASS)
	{
		printf("Error creating the get_data task!\n");
	}

	xReturned = xTaskCreate(&slam_process, "slam_process", 1024, NULL, 3, &xHandle_SLAM);
	if(xReturned != pdPASS)
	{
		printf("Error creating the slam_process task!\n");
	}

    xReturned = xTaskCreate(&blink, "blink", 256, NULL, 2, &xHandle_blink);
	if(xReturned != pdPASS)
	{
		printf("Error creating the blink task!\n");
	}

	xReturned = xTaskCreate(&show_data, "show_data", 1024, NULL, 2, &xHandle_showdata);
	if(xReturned != pdPASS)
	{
		printf("Error creating the show_data task!\n");
	}

	xReturned = xTaskCreate(&wifi_task, "wifi_task", 256, NULL, 2, &xHandle_wifi_task);
	if(xReturned != pdPASS)
	{
		printf("Error creating the wifi task!\n");
	}

	xReturned = xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 2, &xHandle_mqtt_task);
	if(xReturned != pdPASS)
	{
		printf("Error creating the mqqt task!\n");
	}
}
