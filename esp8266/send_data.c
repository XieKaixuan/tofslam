//https://github.com/SuperHouse/esp-open-rtos/blob/master/examples/ds18b20_broadcaster/ds18b20_broadcaster.c

#include "main.h"



void send_data(void *pvParameters){

	TickType_t xLastWakeTime;
 	const TickType_t xFrequency = 30/ portTICK_PERIOD_MS;
	int number = 1;
	uint32_t now = 0, lastUpdate = 0;
    char msg[100];
    err_t err;

    // Send out some UDP data
    struct netconn* conn;

    // Create UDP connection
    conn = netconn_new(NETCONN_UDP);

    // Connect to local port
    err = netconn_bind(conn, IP_ADDR_ANY, 8004);
    if (err != ERR_OK) {
        netconn_delete(conn);
        printf("%s : Could not bind! (%s)\n", __FUNCTION__, lwip_strerr(err));
    }

    err = netconn_connect(conn, IP_ADDR_BROADCAST, 8005);
    if (err != ERR_OK) {
            netconn_delete(conn);
            printf("%s : Could not connect! (%s)\n", __FUNCTION__, lwip_strerr  (err));
    }

    printf("Socket created\n");
	xLastWakeTime = xTaskGetTickCount(); // Get current time

    while(1)
    {
		xSemaphoreTake(sem_processed_data,0);
		if(data_to_send == 1)
		{

			// To check periodicity
			//now = sdk_system_get_time();
			/*lastUpdate = now - lastUpdate;
			printf("%d since last enter\n",lastUpdate);	
			lastUpdate = now;*/

			data_to_send = 0;
			int imu_yaw = 0;
		    sprintf(msg,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",data_processed.laser1,data_processed.laser2,data_processed.laser3,data_processed.laser4,data_processed.laser5,data_processed.laser6,data_processed.laser7,data_processed.laser8,data_processed.yaw,number);        
			number += 1;
			printf("Number: %d\n",number);
		    struct netbuf* buf = netbuf_new();
		    void* data = netbuf_alloc(buf, strlen(msg));

		    memcpy (data, msg, strlen(msg));
		    err = netconn_send(conn, buf);

		    if (err != ERR_OK) {
		                //printf("%s : Could not send data!!! (%s)\n", __FUNCTION__, lwip_strerr(err));
		    }
		    //printf("Sensor data sent over UDP Wifi\n");
		    netbuf_delete(buf); // De-allocate packet buffer
		}
		xSemaphoreGive(sem_processed_data);	
		vTaskDelayUntil( &xLastWakeTime, xFrequency ); // Espera de 30 milisegundos para tener nuevos datos a una tasa de 33 Hz
    }

    err = netconn_disconnect(conn);
    printf("%s : Disconnected from IP_ADDR_BROADCAST port 12346 (%s)\n", __FUNCTION__, lwip_strerr(err));

    err = netconn_delete(conn);
    printf("%s : Deleted connection (%s)\n", __FUNCTION__, lwip_strerr(err));


}
