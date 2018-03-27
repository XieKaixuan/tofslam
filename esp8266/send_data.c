//https://github.com/SuperHouse/esp-open-rtos/blob/master/examples/ds18b20_broadcaster/ds18b20_broadcaster.c

#include "main.h"



void send_data(void *pvParameters){

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
    while(1)
    {

        sprintf(msg,"%d,%d,%d,%d,%d,%d,%d,%d\n",data.laser1,data.laser2,data.laser3,data.laser4,data.laser5,data.laser6,data.laser7,data.imu_yaw);        

        struct netbuf* buf = netbuf_new();
        void* data = netbuf_alloc(buf, strlen(msg));

        memcpy (data, msg, strlen(msg));
        err = netconn_send(conn, buf);

        if (err != ERR_OK) {
                    printf("%s : Could not send data!!! (%s)\n", __FUNCTION__, lwip_strerr(err));
        }
        printf("Sensor data sent over UDP Wifi\n");
        netbuf_delete(buf); // De-allocate packet buffer
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }

    err = netconn_disconnect(conn);
    printf("%s : Disconnected from IP_ADDR_BROADCAST port 12346 (%s)\n", __FUNCTION__, lwip_strerr(err));

    err = netconn_delete(conn);
    printf("%s : Deleted connection (%s)\n", __FUNCTION__, lwip_strerr(err));


}
