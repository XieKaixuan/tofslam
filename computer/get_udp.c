#include "tofslam.h"

void *get_udp(void *vargp){

	printf("GET UDP thread started\n");	
	
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
    
	
    while(1){
        //printf("Waiting data\n");
        if((recvfrom (sock, (char *)&buffer, sizeof(buffer), 0, (struct sockaddr *)&client, &lenclient))==-1)
        {
            perror("receive");
            exit(1);
        }
        sem_wait(&sem_data);

        sscanf(buffer,"%d,%d,%d,%d,%d,%d,%d,%d\n",&data.laser1,&data.laser2,&data.laser3,&data.laser4,&data.laser5,&data.laser6,&data.laser7,&data.imu_yaw);	
	
		// Optional!!!!!
		//data.laser2 = data.laser3;
		//data.laser3 = data.laser5;

		new_data = 1;

		sem_post(&sem_data);
	}

	pthread_exit(NULL);


}
