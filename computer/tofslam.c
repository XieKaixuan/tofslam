//http://www.chuidiang.org/clinux/sockets/udp/udp.php

#include "tofslam.h"

sem_t sem_data;
sensor_data data;

void main(void){

	// Create semaphore
	sem_init(&sem_data, 0, 1);

	// Create both threads
	pthread_t tid[2];

	pthread_create(&tid[0], NULL, get_udp, NULL); //Get data from esp8266
	sleep(1);
	pthread_create(&tid[1], NULL, slam, NULL); //Proccess SLAM
	
	// Wait for the threads
	pthread_join(tid[0],NULL);
    pthread_join(tid[1],NULL);
	
	// Destroy the sem
	sem_destroy(&sem_data);
    return;
	
}
