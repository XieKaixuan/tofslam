#include <main.h>

void blink(void *pvParameters)
{
	const int gpio = 2;
	char msg[PUB_MSG_LEN];
    gpio_enable(gpio, GPIO_OUTPUT);
    while(1) {
        gpio_write(gpio, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_write(gpio, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
		snprintf(msg, PUB_MSG_LEN, "Hi!\r\n");
        if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) {
            printf("Publish queue overflow.\r\n");
        }
    }
}

