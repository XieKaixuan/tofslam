//http://www.chuidiang.org/clinux/sockets/udp/udp.php

#include "tofslam.h"

void main(void){

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
    sensor_data data;
    while(1){
        printf("Waiting data\n");
        if((recvfrom (sock, (char *)&buffer, sizeof(buffer), 0, (struct sockaddr *)&client, &lenclient))==-1)
        {
            perror("receive");
            exit(1);
        }
        
        sscanf(buffer,"%d,%d,%d,%d,%d,%d,%d,%d\n",&data.laser1,&data.laser2,&data.laser3,&data.laser4,&data.laser5,&data.laser6,&data.laser7,&data.imu_yaw);
        
        
        ts_sensor_data_t *sd, sd_struct;
	    sd = &sd_struct;
	    ts_state_t *state, state_struct; 
	    state = &state_struct;
	    ts_map_t *map, map_struct;
	    map = &map_struct;
	    ts_laser_parameters_t *laser_params, laser_params_struct;
	    laser_params = &laser_params_struct;
	    laser_params->offset = 0; // in mm
	    laser_params->angle[0] = 0*M_PI/180;
	    laser_params->angle[1] = 0*M_PI/180;
	    laser_params->angle[2] = 0*M_PI/180;
	    laser_params->angle[3] = 0*M_PI/180;
	    laser_params->angle[4] = 0*M_PI/180;
	    laser_params->angle[5] = 0*M_PI/180;
	    laser_params->angle[6] = 0*M_PI/180;
	    laser_params->angle[7] = 0*M_PI/180;
	    laser_params->distance_no_detection = 2000;
	    ts_position_t *position, position_struct;
	    position = &position_struct;
	    position->x = 0;
	    position->y = 0;
	    position->theta = 0;	
            
        // Initialization of map, state and random struct
	    //ts_map_init(map);
	    //ts_state_init(state, map, laser_params, position, 100); //ts_state_init(state, map, laser_params, position, hole_width);
	    unsigned long jsrseed = 1;
	    //ts_random_init(&state->randomizer,jsrseed);

		// Get data and put it in the structure sd
		
		sd->d[0] = data.laser1;
		sd->d[1] = data.laser2;
		sd->d[2] = data.laser3;
		sd->d[3] = data.laser4;
		sd->d[4] = data.laser5;
		sd->d[5] = data.laser6;
		sd->d[6] = data.laser7;
		sd->d[7] = data.imu_roll;
		sd->timestamp += 1;

	}

}
