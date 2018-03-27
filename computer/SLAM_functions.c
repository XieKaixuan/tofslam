#include <tofslam.h>

void ts_map_init(ts_map_t *map)
{
    int x, y, initval;
    ts_map_pixel_t *ptr;
    initval = (TS_OBSTACLE + TS_NO_OBSTACLE) / 2;
    for (ptr = map->map, y = 0; y < TS_MAP_SIZE; y++) {
		for (x = 0; x < TS_MAP_SIZE; x++, ptr++) {
			*ptr = initval;
			printf("Map init: %d\n",initval);
		}
    }
}

void ts_state_init(ts_state_t *state, ts_map_t *map, ts_laser_parameters_t *laser_params, ts_position_t *position, int hole_width)
{
	printf("Starting init state\n");
	state->map = map;
	state->laser_params = *laser_params;
	state->position = *position;
	state->timestamp = 0;
	state->distance = 0;
	state->done = 0;
	state->hole_width = hole_width;
	state->sigma_xy = 0;
	state->sigma_theta = 0;
	state->timestamp = 0;
}

// Convert sensor data to x and y position with respect the center of the robot and store in value OBSTACLE OR NO OBSTACLE
void ts_build_scan(ts_sensor_data_t *sd, ts_scan_t *scan, ts_state_t *state)
{
	int i=0;
	float offsetx, offsety;
	for(i=0;i<TS_SCAN_SIZE;i++)
	{
		
		if((double)sd->d[i] >= state->laser_params.distance_no_detection)
		{
			scan->x[i] = (state->laser_params.distance_no_detection * cos(state->laser_params.angle[i])) + state->laser_params.offset * cos(state->laser_params.angle[i]);
		    scan->y[i] = (state->laser_params.distance_no_detection * sin(state->laser_params.angle[i])) + state->laser_params.offset * sin(state->laser_params.angle[i]);
			scan->value[i] = TS_NO_OBSTACLE;
		}
		if (sd->d[i] > state->hole_width / 2)
		{
		    scan->x[i] = (sd->d[i] * cos(state->laser_params.angle[i])) + state->laser_params.offset * cos(state->laser_params.angle[i]);
		    scan->y[i] = (sd->d[i] * sin(state->laser_params.angle[i])) + state->laser_params.offset * sin(state->laser_params.angle[i]);
		    scan->value[i] = TS_OBSTACLE;
		}
	}
}

int ts_distance_scan_to_map(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos)
{
    double c, s;
    int i, x, y, nb_points = 0;
    int64_t sum;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    // Translate and rotate scan to robot position
    // and compute the distance
    for (i = 0, sum = 0; i != TS_SCAN_SIZE; i++){
        if (scan->value[i] != TS_NO_OBSTACLE) {
			//Translating into global x and y
            x = (int)floor((pos->x + c * scan->x[i] - s * scan->y[i]) * TS_MAP_SCALE + 0.5);
            y = (int)floor((pos->y + s * scan->x[i] + c * scan->y[i]) * TS_MAP_SCALE + 0.5);
            // Check boundaries
            if (x >= 0 && x < TS_MAP_SIZE && y >= 0 && y < TS_MAP_SIZE) {
                sum += map->map[y * TS_MAP_SIZE + x];
                nb_points++;
            }
        }
    }
    if (nb_points) 
	{
		sum = sum * 1024 / nb_points;
	}else{ 
		sum = 2000000000;
	}
    return (int)sum;
}

#define SWAP(x, y) (x ^= y ^= x ^= y)

void
ts_map_laser_ray(ts_map_t *map, int x1, int y1, int x2, int y2, int xp, int yp, int value, int alpha)
{
    int x2c, y2c, dx, dy, dxc, dyc, error, errorv, derrorv, x;
    int incv, sincv, incerrorv, incptrx, incptry, pixval, horiz, diago;
    ts_map_pixel_t *ptr;

    if (x1 < 0 || x1 >= TS_MAP_SIZE || y1 < 0 || y1 >= TS_MAP_SIZE)
        return; // Robot is out of map
    
    x2c = x2; y2c = y2;
    // Clipping
    if (x2c < 0) {
        if (x2c == x1) return;
        y2c += (y2c - y1) * (-x2c) / (x2c - x1);
        x2c = 0;
    }
    if (x2c >= TS_MAP_SIZE) {
        if (x1 == x2c) return;
        y2c += (y2c - y1) * (TS_MAP_SIZE - 1 - x2c) / (x2c - x1);
        x2c = TS_MAP_SIZE - 1;
    }
    if (y2c < 0) {
        if (y1 == y2c) return;
        x2c += (x1 - x2c) * (-y2c) / (y1 - y2c);
        y2c = 0;
    }
    if (y2c >= TS_MAP_SIZE) {
        if (y1 == y2c) return;
        x2c += (x1 - x2c) * (TS_MAP_SIZE - 1 - y2c) / (y1 - y2c);
        y2c = TS_MAP_SIZE - 1;
    }

    dx = abs(x2 - x1); dy = abs(y2 - y1);
    dxc = abs(x2c - x1); dyc = abs(y2c - y1);
    incptrx = (x2 > x1) ? 1 : -1;
    incptry = (y2 > y1) ? TS_MAP_SIZE : -TS_MAP_SIZE;
    sincv = (value > TS_NO_OBSTACLE) ? 1 : -1; 
    if (dx > dy) {
        derrorv = abs(xp - x2);
    } else {
        SWAP(dx, dy); SWAP(dxc, dyc); SWAP(incptrx, incptry);        
        derrorv = abs(yp - y2);
    }
    error = 2 * dyc - dxc;
    horiz = 2 * dyc;
    diago = 2 * (dyc - dxc);
    errorv = derrorv / 2;
    incv = (value - TS_NO_OBSTACLE) / derrorv;
    incerrorv = value - TS_NO_OBSTACLE - derrorv * incv;  
    ptr = map->map + y1 * TS_MAP_SIZE + x1;
    pixval = TS_NO_OBSTACLE;
    for (x = 0; x <= dxc; x++, ptr += incptrx) {
        if (x > dx - 2 * derrorv) {
            if (x <= dx - derrorv) {
                pixval += incv;
                errorv += incerrorv;
                if (errorv > derrorv) {
                    pixval += sincv;
                    errorv -= derrorv; 
                }
            } else {
                pixval -= incv;
                errorv -= incerrorv;
                if (errorv < 0) {
                    pixval -= sincv;
                    errorv += derrorv; 
                }
            }
        } 
        // Integration into the map
        *ptr = ((256 - alpha) * (*ptr) + alpha * pixval) >> 8; 	
        if (error > 0) {
            ptr += incptry;
            error += diago;
        } else error += horiz;
    }
}


void
ts_map_update(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos, int quality, int hole_width)
{
    double c, s;
    double x2p, y2p;
    int i, x1, y1, x2, y2, xp, yp, value, q;
    double add, dist;

    c = cos(pos->theta * M_PI / 180);
    s = sin(pos->theta * M_PI / 180);
    x1 = (int)floor(pos->x * TS_MAP_SCALE + 0.5); //Position of the robot in global system in number of "holes" and with respect to reference point
    y1 = (int)floor(pos->y * TS_MAP_SCALE + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i != TS_SCAN_SIZE; i++) {
        x2p = c * scan->x[i] - s * scan->y[i]; //Position of obstacles in global system but with respect to robot position
        y2p = s * scan->x[i] + c * scan->y[i];  
		xp = (int)floor((pos->x + x2p) * TS_MAP_SCALE + 0.5); //Position of obstacles in global system in number of "holes" and with respect to reference point
        yp = (int)floor((pos->y + y2p) * TS_MAP_SCALE + 0.5);
        dist = sqrt(x2p * x2p + y2p * y2p); //Distance from obstacles to robot position
        add = hole_width / 2 / dist; 
        x2p *= TS_MAP_SCALE * (1 + add); //Position of obstacles in global system and number of "holes" with respect to robot position
        y2p *= TS_MAP_SCALE * (1 + add); 
        x2 = (int)floor(pos->x * TS_MAP_SCALE + x2p + 0.5); //Position of obstacles in global system in number of "holes" with respect to reference point and adding some dependance with the distance from obstacle to robot
        y2 = (int)floor(pos->y * TS_MAP_SCALE + y2p + 0.5);
        if (scan->value[i] == TS_NO_OBSTACLE) { 
            q = quality / 4;
            value = TS_NO_OBSTACLE;
        } else {
            q = quality;
            value = TS_OBSTACLE;
        }
        printf("%d %d %d %d %d %d %d\n", i, x1, y1, x2, y2, xp, yp);
        ts_map_laser_ray(map, x1, y1, x2, y2, xp, yp, value, q);
    }
}

void ts_iterative_map_building(ts_sensor_data_t *sd, ts_state_t *state)
{
    
    state->position.theta = 0;//sd->d[7];

    ts_build_scan(sd, &state->scan, state);
	printf("Scan 1, x: %f, y: %f\n",state->scan.x[0],state->scan.y[0]);
    // Monte Carlo search
    
    state->position = ts_monte_carlo_search(&state->randomizer, &state->scan, state->map, &state->position, state->sigma_xy, state->sigma_theta, 1000, NULL);
	
	printf("Position: x: %f, y:%f, theta: %f\n",state->position.x,state->position.y,state->position.theta);  	
	
    // Map update
    ts_map_update(&state->scan, state->map, &state->position, 50, state->hole_width);

    // Prepare next step
    state->timestamp = sd->timestamp;
}



