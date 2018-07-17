#include <tofslam.h>


int first_yaw = 1000;

void set_params(ts_laser_parameters_t *laser_params)
{
	laser_params->offset = (double)25/2; // diameter/2 in mm
    laser_params->angle[0] = 90*M_PI/180;
    laser_params->angle[1] = 45*M_PI/180;
    laser_params->angle[2] = 0*M_PI/180;
    laser_params->angle[3] = -45*M_PI/180;
    laser_params->angle[4] = -90*M_PI/180;
    laser_params->angle[5] = -135*M_PI/180;
    laser_params->angle[6] = -180*M_PI/180;
    laser_params->angle[7] = 135*M_PI/180;
    laser_params->distance_no_detection = 2000;
	printf("Params set\n");
}

void set_init_pos(ts_position_t *position)
{
	position->x = TS_MAP_SIZE/2;
    position->y = TS_MAP_SIZE/2;
    position->theta = 0;	
	printf("Pos init\n");
}

void ts_map_init(ts_map_t *map)
{
    int x, y, initval;
    ts_map_pixel_t *ptr;
    initval = (TS_OBSTACLE + TS_NO_OBSTACLE) / 2;
    for (ptr = map->map, y = 0; y < TS_MAP_SIZE; y++) {
		for (x = 0; x < TS_MAP_SIZE; x++, ptr++) {
			*ptr = initval;
			//printf("Map init: %d\n",map->map[y*TS_MAP_SIZE+x]);
		}
    }
	//printf("Map initiated\n");
}

void ts_state_init(ts_state_t *state, ts_map_t *map, ts_laser_parameters_t *laser_params, ts_position_t *position, int hole_width, double sigma_xy, int memory)
{
	state->map = map;
	state->laser_params = *laser_params;
	state->position = *position;
	state->timestamp = 0;
	state->distance = 0;
	state->done = 0;
	state->hole_width = hole_width;
	state->sigma_xy = sigma_xy; //Check why I need to use such a little value
	state->memory = memory;
	printf("State initiated\n");
}

// Convert sensor data to x and y position with respect the center of the robot and store in value OBSTACLE OR NO OBSTACLE
void ts_build_scan(ts_sensor_data_t *sd, ts_scan_t *scan, ts_state_t *state)
{
	int i=0;

	for(i=0;i<TS_SCAN_SIZE;i++)
	{
		
		if(sd->d[i] >= state->laser_params.distance_no_detection) //in case we haven't a detection
		{
			scan->x[i] = ((state->laser_params.distance_no_detection + state->laser_params.offset) * cos(state->laser_params.angle[i]));
		    scan->y[i] = ((state->laser_params.distance_no_detection + state->laser_params.offset) * sin(state->laser_params.angle[i]));
			scan->value[i] = TS_NO_OBSTACLE;
		}
		else if(sd->d[i] > state->hole_width / 2) //in case we have a detection
		{
		    scan->x[i] = ((sd->d[i] + state->laser_params.offset) * cos(state->laser_params.angle[i]));
		    scan->y[i] = ((sd->d[i] + state->laser_params.offset) * sin(state->laser_params.angle[i]));
		    scan->value[i] = TS_OBSTACLE;
		}
		scan->x[i]/=10; // We don't need mm precision
		scan->y[i]/=10;
		
	}
}

// This function provide the likelyhood between position and map
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
		sum = sum / nb_points;//* 1024 / nb_points;
	}else{ 
		sum = 2000000000;
	}
    return (int)sum;
}

#define SWAP(x, y) (x ^= y ^= x ^= y)


// Implement bresenham's algorithm
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
    x1 = (int)floor(pos->x * TS_MAP_SCALE + 0.5); //Position of the robot in global system with respect to global system scaled
    y1 = (int)floor(pos->y * TS_MAP_SCALE + 0.5);
    // Translate and rotate scan to robot position
    for (i = 0; i != TS_SCAN_SIZE; i++) {
        x2p = c * scan->x[i] - s * scan->y[i]; //Position of obstacles in global system with respect to global system but from the robot
        y2p = s * scan->x[i] + c * scan->y[i];  
		xp = (int)floor((pos->x + x2p) * TS_MAP_SCALE + 0.5); //Position of obstacles in global system from reference point
        yp = (int)floor((pos->y + y2p) * TS_MAP_SCALE + 0.5);
        dist = sqrt(x2p * x2p + y2p * y2p); //Distance from obstacles to robot position
        add = hole_width / 2 / dist;
		//printf("ADD = %f\n", add);
        x2p *= TS_MAP_SCALE * (1 + add); //Position of obstacles in global system with respect to robot position
        y2p *= TS_MAP_SCALE * (1 + add); 
        x2 = (int)floor(pos->x * TS_MAP_SCALE + x2p + 0.5); //Position of obstacles in global system with respect to reference point and adding the half of the hole width
        y2 = (int)floor(pos->y * TS_MAP_SCALE + y2p + 0.5);
        if (scan->value[i] == TS_NO_OBSTACLE) { 
            q = quality / 4;
            value = TS_NO_OBSTACLE;
        } else {
            q = quality;
            value = TS_OBSTACLE;
        }
        /*printf("%d %d %d %d %d %d %d\n", i, x1, y1, x2, y2, xp, yp);*/
        ts_map_laser_ray(map, x1, y1, x2, y2, xp, yp, value, q);
    }
}

void ts_iterative_map_building(ts_sensor_data_t *sd, ts_state_t *state)
{
	
	if(first_yaw == 1000) //relative yaw
		first_yaw = sd->theta;
    
	//Update tetha
	//state->position.theta = 0;
    state->position.theta = -(sd->theta-first_yaw);
	//printf("Yaw: %d\n",sd->theta);
	
	// Translate distance detection to robot coordinate system
    ts_build_scan(sd, &state->scan, state);
 	
	// Estimate the position based on Monte Carlo search
    state->position = ts_monte_carlo_search(&state->randomizer, &state->scan, state->map, &state->position, state->sigma_xy, 1000, NULL);
		

  	
	
    // Map update
    ts_map_update(&state->scan, state->map, &state->position, state->memory, state->hole_width);

    // Prepare next step
    state->timestamp = sd->timestamp;
}



void
ts_save_map(ts_map_t *map, ts_map_t *overlay, char *filename, int width, int height) 
{
    int x, y, xp, yp;
    FILE *output;
   
    char buffer[1000500];
    int index = 0;
    //fprintf(output, "P2\n%d %d 255\n", width, height);
    y = (TS_MAP_SIZE - height) / 2;
    for (yp = 0; yp < height; y++, yp++) {
        x = (TS_MAP_SIZE - width) / 2; 
        for (xp = 0; xp < width; x++, xp++) {
            if (overlay->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x] == 0) 
            {            
                index += sprintf(&buffer[index], "0");
                //printf("0");
            }            
            else
            { 
                index += sprintf(&buffer[index], "%d", (int)(map->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8);
                //printf("%d",(int)(map->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 5);
            }
            index += sprintf(&buffer[index], "\n");  
        }
        index += sprintf(&buffer[index], "\n");
    }

    output = fopen(filename, "w");
    fwrite( buffer, index, 1, output );
    fclose(output);
}


void
ts_save_position(int x, int y, int theta, ts_map_t *map, char *filename, int width, int height)
{
    FILE *output;
    output = fopen(filename, "a");
	fprintf(output,"%d\t",x);	
	fprintf(output,"%d\t",TS_MAP_SIZE/2-(y-TS_MAP_SIZE/2));	
	fprintf(output,"%d\n",theta);	
    fclose(output);
}




/*void
ts_save_map_pgm(ts_map_t *map, ts_map_t *overlay, char *filename, int width, int height) 
{
    int x, y, xp, yp;
    FILE *output;
    output = fopen(filename, "wt");
    //fprintf(output, "P2\n%d %d 255\n", width, height);
    y = (TS_MAP_SIZE - height) / 2;
    for (yp = 0; yp < height; y++, yp++) {
        x = (TS_MAP_SIZE - width) / 2; 
        for (xp = 0; xp < width; x++, xp++) {
            if (overlay->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x] == 0) 
            {            
                fprintf(output, "0");
                //printf("0");
            }            
            else
            { 
                fprintf(output, "%d", (int)(map->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8);
                //printf("%d",(int)(map->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 5);
            }   
        fprintf(output, "\n");  
        }
        fprintf(output, "\n");
    }
    fclose(output);
}
*/