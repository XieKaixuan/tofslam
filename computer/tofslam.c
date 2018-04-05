//http://www.chuidiang.org/clinux/sockets/udp/udp.php

#include "tofslam.h"

sem_t sem_data;
sensor_data data;

void main(void){

	// Create semaphore
	sem_init(&sem_data, 0, 1);

	// Create both threads
	pthread_t tid[2];

	if(pthread_create(&tid[0], NULL, get_udp, NULL) != 0) //Get data from esp8266
	{
		printf("Error creating GET_UDP thread\n");
		return;
	}
	sleep(1);
	if(pthread_create(&tid[1], NULL, slam, NULL) != 0); //Proccess SLAM
	{
		printf("Error creating SLAM thread\n");
		return;
	}
	// Wait for the threads
	pthread_join(tid[0],NULL);
    pthread_join(tid[1],NULL);
	
	// Destroy the sem
	sem_destroy(&sem_data);
    return;
	
}
