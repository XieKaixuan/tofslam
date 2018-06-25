#include "tofslam.h"

void *get_udp(void *vargp){
	
	printf("GET UDP thread started\n");	
	
	uint32_t now = 0, lastUpdate = 0;
    int sock;
    if ((sock = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        perror("socket");
        exit(1);
    }

    struct sockaddr_in dir;

    dir.sin_family = AF_INET;
    dir.sin_port = htons(8005);
    dir.sin_addr.s_addr = INADDR_ANY; 

    if(bind ( sock, (struct sockaddr *)&dir, sizeof (dir))==-1)
    {
        perror("bind");
        exit(1);
    }

    /* Contendrá los datos del que nos envía el mensaje */
    struct sockaddr_in client;

    /* Tamaño de la estructura anterior */    
    int lenclient = sizeof(client);  

    /* Nuestro mensaje es simplemente un entero, 4 bytes. */
    char buffer[100]; 
	int number_previous = 0;
    while(1){
        //printf("Waiting data\n");
        if((recvfrom (sock, (char *)&buffer, sizeof(buffer), 0, (struct sockaddr *)&client, &lenclient))==-1)
        {
            perror("receive");
            exit(1);
        }
        sem_wait(&sem_data);

		/*now = getMicrotime();
		lastUpdate = now - lastUpdate;
		printf("%d since last enter\n",lastUpdate);	
		lastUpdate = now;*/

        sscanf(buffer,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",&data.laser1,&data.laser2,&data.laser3,&data.laser4,&data.laser5,&data.laser6,&data.laser7,&data.laser8,&data.imu_yaw, &data.number);		

		/*if(number_previous+1 != data.number)
			printf("Error!\n");
		number_previous = data.number;
		*/
		new_data = 1;

		sem_post(&sem_data);
	}

	pthread_exit(NULL);


}

long getMicrotime(){
	struct timeval currentTime;
	gettimeofday(&currentTime, NULL);
	return currentTime.tv_sec * (int)1e6 + currentTime.tv_usec;
}
