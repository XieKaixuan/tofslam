#include "tofslam.h"
#include <time.h>
#include <ncurses.h>

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
    ts_state_init(state, map, laser_params, position, 25, 0.000000002, 50); //ts_state_init(state, map, laser_params, position, hole_width, sigma_xy 0.000000003, memory);
    unsigned long jsrseed = 1;
    ts_random_init(&state->randomizer,jsrseed);
	int count = 0;

	clock_t t_frec = 0;	

	int print = 1;
	char buf;

	// Create a file to store trajectory
	FILE* output = fopen("pos", "w");
	fclose(output);

	/*initscr();
    cbreak();
    noecho();
    scrollok(stdscr, TRUE);
    nodelay(stdscr, TRUE);*/

	while(1)
	{	
		// Lock sem to check new data flag
		sem_wait(&sem_data);
		if(new_data == 1)
		{
			new_data = 0;
			
			t_frec = (clock()-t_frec);
			printf("%.16g milisegundos\n", t_frec * 1000.0/ CLOCKS_PER_SEC);
			t_frec = clock();
			
  			
			// Get the data
			ts_get_data(sd);
			
			// Unlock sem	
			sem_post(&sem_data);

			// Iterate the algorithm	
			ts_iterative_map_building(sd, state);
			

			
			//if(sd->timestamp == 1)
			if(print)
			{
				printf("Position: x: %d, y:%d, theta: %d\n",state->position.x,state->position.y,state->position.theta);
				ts_save_position(state->position.x,state->position.y,state->position.theta, state->map, "pos", TS_MAP_SIZE,TS_MAP_SIZE);
				ts_save_map(state->map, state->map, "map", TS_MAP_SIZE,TS_MAP_SIZE);			
			}
			
			// With this we can stop the adquisition
			/*if(getch()=='s')
			{
				printf("Pause, press a key to continue ... \n");
				getchar();
			}*/
		}
		else
		{
			// Unlock sem if there is no data
			sem_post(&sem_data);
		}

		
	}

	pthread_exit(NULL);

}




void ts_get_data(ts_sensor_data_t *sd)
{

	sd->d[0] = data.laser1;
	sd->d[1] = data.laser2;
	sd->d[2] = data.laser3;
	sd->d[3] = data.laser4;
	sd->d[4] = data.laser5;
	sd->d[5] = data.laser6;
	sd->d[6] = data.laser7;
	sd->d[7] = data.laser8;
	sd->theta = data.imu_yaw;
	sd->timestamp += 1;	

}
