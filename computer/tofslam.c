#include "tofslam.h"

// Variables to be used
sem_t sem_data;
sensor_data data;
int new_data = 0;

void main(void){

	// Create semaphore
	sem_init(&sem_data, 0, 1); // To protect data structure

	// Create both threads
	pthread_t tid[2];

	if(pthread_create(&tid[0], NULL, get_udp, NULL) != 0) //Get data from esp8266
	{
		printf("Error creating GET_UDP thread\n");
		return;
	}
	sleep(1);
	if(pthread_create(&tid[1], NULL, slam, NULL) != 0) //Proccess SLAM
	{
		printf("Error creating SLAM thread\n");
		return;
	}

	// Wait for the threads to finish
	pthread_join(tid[0],NULL);
    pthread_join(tid[1],NULL);
	
	// Destroy the sem
	sem_destroy(&sem_data);
    return;
	
}
