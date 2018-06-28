#ifndef _HEADERS_
#define _HEADERS_

#include <stdio.h> //printf
#include <string.h> //memset
#include <stdlib.h> //exit(0);
#include <sys/socket.h>
#include <arpa/inet.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

typedef struct{
	int laser1;
	int laser2;
	int laser3;
	int laser4;
	int laser5;
	int laser6;
	int laser7;
	int laser8;
	int imu_yaw;
	int imu_pitch;
	int imu_roll;
	int number;
}sensor_data;

extern sem_t sem_data;
extern sensor_data data;
extern int new_data;

void *get_udp(void *vargp);
void *slam(void *vargp);


#endif // _HEADERS

#ifndef _TINYSLAM_H_
#define _TINYSLAM_H_

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TS_SCAN_SIZE 8
#define TS_MAP_SIZE 500
#define TS_MAP_SCALE 1
#define TS_NO_OBSTACLE 65500
#define TS_OBSTACLE 0

typedef unsigned short ts_map_pixel_t;

typedef struct {
    ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE];    
} ts_map_t;

typedef struct {
    int x[TS_SCAN_SIZE], y[TS_SCAN_SIZE];
    int value[TS_SCAN_SIZE];
} ts_scan_t;

typedef struct {
    int x, y;    // in mm
    int theta;   // in degrees
} ts_position_t;

typedef struct {
    unsigned int timestamp;
    int d[TS_SCAN_SIZE];
	int theta;
} ts_sensor_data_t;

typedef struct {
    float offset;  // position of the laser wrt center of rotation
    float angle[8];  // angle for scan
    int distance_no_detection; // default value when the laser returns 0
} ts_laser_parameters_t;

typedef struct {
    unsigned long jz;
    unsigned long jsr;
    long hz;
    unsigned long iz;
    unsigned long kn[128];
    double wnt[128];
    double wn[128];
    double fn[128];
} ts_randomizer_t;

typedef struct {
	ts_randomizer_t randomizer;
    ts_map_t *map;
    ts_laser_parameters_t laser_params;
    ts_position_t position;
    unsigned int timestamp;
    double distance;
	int done;
    int hole_width;
    ts_scan_t scan;
	double sigma_xy;
    double sigma_theta;
} ts_state_t;

long getMicrotime();

void set_params(ts_laser_parameters_t *laser_params);
void set_init_pos(ts_position_t *position);

void ts_state_init(ts_state_t *state, ts_map_t *map, ts_laser_parameters_t *laser_params, ts_position_t *position, int hole_width);
void ts_build_scan(ts_sensor_data_t *sd, ts_scan_t *scan, ts_state_t *state);
void ts_map_init(ts_map_t *map);
int ts_distance_scan_to_map(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos);
void ts_iterative_map_building(ts_sensor_data_t *sd, ts_state_t *state);
void ts_map_update(ts_scan_t *scan, ts_map_t *map, ts_position_t *position, int quality, int hole_width);

void ts_random_init(ts_randomizer_t *d, unsigned long jsrseed);
ts_position_t ts_monte_carlo_search(ts_randomizer_t *randomizer, ts_scan_t *scan, ts_map_t *map, ts_position_t *start_pos, double sigma_xy, double sigma_theta, int stop, int *bestdist);
double ts_random_normal(ts_randomizer_t *d, double m, double s);
double ts_random_normal_fix(ts_randomizer_t *d);
static unsigned long SHR3(ts_randomizer_t *d);

void ts_save_map_pgm(ts_map_t *map, ts_map_t *overlay, char *filename, int width, int height);

#endif // _TINYSLAM_H_
