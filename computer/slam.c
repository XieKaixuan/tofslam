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
    ts_state_init(state, map, laser_params, position, 25); //ts_state_init(state, map, laser_params, position, hole_width);
    unsigned long jsrseed = 1 ;
    ts_random_init(&state->randomizer,jsrseed);
	int count = 0;

	clock_t t_frec = 0;	

	ts_position_t trajectory[1000], last_position;
	int i = 0;
	char buf;


	initscr();
    cbreak();
    noecho();
    scrollok(stdscr, TRUE);
    nodelay(stdscr, TRUE);

	while(1)
	{	
		sem_wait(&sem_data);
		if(new_data == 1)
		{
			
			
			t_frec = (clock()-t_frec);
			//printf("%.16g milisegundos\n", t_frec * 1000.0/ CLOCKS_PER_SEC);
			t_frec = clock();
			
  			
			new_data = 0;

			//Pass data from buffer to structure of data
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
			sem_post(&sem_data);

	
			printf("Position: x: %d, y:%d, theta: %d\n",state->position.x,state->position.y,state->position.theta);
			if((state->position.x!=last_position.x)||(state->position.y!=last_position.y)||(state->position.theta!=last_position.theta))
			{
				trajectory[i++] = state->position;
				if(i>1000)
					i = 0;
			}
	
			last_position = state->position;

			/*printf("Data 1 %d\t",data.laser1);
			printf("Data 2 %d\t",data.laser2);
			printf("Data 3 %d\t",data.laser3);
			printf("Data 4 %d\t",data.laser4);
			printf("Data 5 %d\t",data.laser5);
			printf("Data 6 %d\t",data.laser6);
			printf("Data 7 %d\t",data.laser7);
			printf("Data 8 %d\t",data.laser8);
			printf("Yaw %d\n",data.imu_yaw);*/
			
				
			ts_iterative_map_building(sd, state);
		
			//if(sd->timestamp == 1)
			ts_save_map_pgm(state->map, state->map, "map", TS_MAP_SIZE,TS_MAP_SIZE);			
			
			if(getch()=='s')
			{
				printf("Pause, press a key to continue ... \n");
				getchar();
			}
		}
		else
		{
			sem_post(&sem_data);
		}

		
	}

	pthread_exit(NULL);

}




