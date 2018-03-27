#include <tofslam.h>

void ts_state_init(ts_state_t *state, ts_map_t *map, ts_laser_parameters_t *laser_params, ts_position_t *position, int hole_width){
    ts_random_init(&state->randomizer, 0xdead);
    state->map = map;
    state->laser_params = *laser_params;
    state->position = *position;
    state->timestamp = 0; // Indicating start
    state->distance = 0;
    state->done = 0;
    state->hole_width = hole_width;
}

void ts_build_scan(ts_sensor_data_t *sd, ts_scan_t *scan, ts_state_t *state)
{
	int i = 0;
	for(i=0;i<TS_SCAN_SIZE;i++)
	{
		if (sd->d[i] == 0)
		{
			scan->x[i] = state->laser_params.distance_no_detection * cos(state->laser_params.angle[i]);
		    scan->y[i] = state->laser_params.distance_no_detection * sin(state->laser_params.angle[i]);
			scan->value[i] = TS_NO_OBSTACLE;
		}
		if (sd->d[i] > state->hole_width / 2)
		{
		    scan->x[i] = sd->d[i] * cos(state->laser_params.angle[i]);
		    scan->y[i] = sd->d[i] * sin(state->laser_params.angle[i]);
		    scan->value[i] = TS_OBSTACLE;
		}
	}
}

void ts_iterative_map_building(ts_sensor_data_t *sd, ts_state_t *state)
{
    double psidot, v, d;
    ts_scan_t scan2map;
    double m, thetarad;
    ts_position_t position;

    ts_build_scan(sd, &scan2map, state);
    //ts_build_scan(sd, &state->scan, state);

	//Rotate scans to reference them to global map

    // Monte Carlo search
    position.x += state->laser_params.offset * cos(thetarad);
    position.y += state->laser_params.offset * sin(thetarad);
    position = 
        ts_monte_carlo_search(&state->randomizer, &state->scan, state->map, &position, state->sigma_xy, state->sigma_theta, 1000, NULL);
    position.x -= state->laser_params.offset * cos(position.theta * M_PI / 180);
    position.y -= state->laser_params.offset * sin(position.theta * M_PI / 180);
    d = sqrt((state->position.x -position.x) * (state->position.x - position.x) +
            (state->position.y - position.y) * (state->position.y - position.y));
    state->distance += d;

    // Map update
    ts_map_update(&scan2map, state->map, &position, 50, state->hole_width);

    // Prepare next step
    state->position = position;
    state->timestamp = sd->timestamp;
}


