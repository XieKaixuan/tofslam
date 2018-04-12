#include "tofslam.h"

void *slam(void *vargp){

	printf("SLAM thread started\n");

	// Define variables
	ts_sensor_data_t *sd, sd_struct;
    sd = &sd_struct;
    ts_state_t *state, state_struct; 
    state = &state_struct;
    ts_map_t *map, map_struct;
    map = &map_struct;
    ts_laser_parameters_t *laser_params, laser_params_struct;
    laser_params = &laser_params_struct;
    ts_position_t *position, position_struct;
    position = &position_struct;
	ts_scan_t *scan, scan_struct;
	scan = &scan_struct;
    
        
    // Initialization of params, pos, map, state and random struct
	set_params(laser_params);
	set_init_pos(position);
    ts_map_init(map);
    ts_state_init(state, map, laser_params, position, 5); //ts_state_init(state, map, laser_params, position, hole_width);
    unsigned long jsrseed = 1 ;
    ts_random_init(&state->randomizer,jsrseed);
	while(1)
	{
		// Get data and put it in the structure sd
		sem_wait(&sem_data);
		if(new_data == 1)
		{
			new_data = 0;

			//Pass data from buffer to structure of data
			sd->d[0] = data.laser1;
			sd->d[1] = data.laser2;
			sd->d[2] = data.laser3;
			sd->d[3] = data.laser4;
			sd->d[4] = data.laser5;
			sd->d[5] = data.laser6;
			sd->d[6] = data.laser7;
			sd->theta = data.imu_yaw;
			sd->timestamp += 1;		
	
			/*
			printf("Laser 1: %d\n",sd->d[0]);
			printf("Laser 2: %d\n",sd->d[1]);
			printf("Laser 3: %d\n",sd->d[2]);
			printf("Laser 4: %d\n",sd->d[3]);
			printf("Laser 5: %d\n",sd->d[4]);
			printf("Laser 6: %d\n",sd->d[5]);
			printf("Laser 7: %d\n",sd->d[6]);
			printf("Yaw: %d\n",sd->theta);
			*/

			//ts_build_scan(sd,scan,state);
		
			/*printf("Scan 1: %d\n",scan->value[0]);
			printf("Scan 2: %d\n",scan->value[1]);
			printf("Scan 3: %d\n",scan->value[2]);
			printf("Scan 4: %d\n",scan->value[3]);
			printf("Scan 5: %d\n",scan->value[4]);
			printf("Scan 6: %d\n",scan->value[5]);
			printf("Scan 7: %d\n",scan->value[6]);	*/

			ts_iterative_map_building(sd, state);
			
			//if(sd->timestamp == 1)
			ts_save_map_pgm(state->map, state->map, "map", 500,500);			

			printf("Packet: %d\n",sd->timestamp);
		}
		sem_post(&sem_data);
		
	}

	pthread_exit(NULL);

}




