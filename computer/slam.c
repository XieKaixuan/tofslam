#include "tofslam.h"

void *slam(void *vargp){

	printf("SLAM thread started\n");

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
	while(1)
	{
	
		// Get data and put it in the structure sd
		sem_wait(&sem_data);

		sd->d[0] = data.laser1;
		sd->d[1] = data.laser2;
		sd->d[2] = data.laser3;
		sd->d[3] = data.laser4;
		sd->d[4] = data.laser5;
		sd->d[5] = data.laser6;
		sd->d[6] = data.laser7;
		sd->d[7] = data.imu_roll;
		sd->timestamp += 1;		

		sem_post(&sem_data);
		
	}

	pthread_exit(NULL);

}
