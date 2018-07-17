#include <main.h>

#define sampleFreqDef   100.0f          // sample frequency in Hz
#define betaDef         1.f            // 2 * proportional gain


float beta;				// algorithm gain
float q0;
float q1;
float q2;
float q3;	// quaternion of sensor frame relative to auxiliary frame
float invSampleFreq;
float roll;
float pitch;
float yaw;
char anglesComputed;

void processing(void *pvParameters)
{
	TickType_t xLastWakeTime;
 	const TickType_t xFrequency = 5/ portTICK_PERIOD_MS;

	uint32_t now = 0, lastUpdate = 0, count = 0, aux = 0, i = 0, max = 0;
	int yaw = 0;
	madgwick_init();

	kalman_state state[8];

	//Init kalman filters 
	state[0] = kalman_init(1,15,1,100); //(process, measurement, error, init_value) 10,170,50,100
	state[1] = kalman_init(1,15,1,100);
	state[2] = kalman_init(1,15,1,100);
	state[3] = kalman_init(1,15,1,100);
	state[4] = kalman_init(1,15,1,100);
	state[5] = kalman_init(1,15,1,100);
	state[6] = kalman_init(1,15,1,100);
	state[7] = kalman_init(1,15,1,100);

	xLastWakeTime = xTaskGetTickCount(); // Get current time
	while(1)
	{
		// To check periodicity
		/*now = sdk_system_get_time();
		lastUpdate = now - lastUpdate;
		printf("%d since last enter\n",lastUpdate);	
		lastUpdate = now;*/
		

		xSemaphoreTake(sem_data,0);
		if(data_to_process == 1)
		{
			
			count += 1;
			
			// Update Madgwick filter
			madgwick_update(data.g[0],data.g[1],data.g[2],data.a[1],-data.a[0],data.a[2],data.m[1],data.m[0],data.m[2]);
			yaw = getYaw();
			/*if(yaw%5!=0)
			{
				if(yaw%5<=2)
					yaw-=(yaw%5);
				else{
					yaw+=(5-(yaw%5));
				}
			}*/
			if(yaw == 360) yaw = 0;
				
				
			if(count >=6) // To achieve 30 ms between kalman compute
			{
				
				count = 0;				

				//Update kalman filters  
				state[0] =kalman_update(state[0],data.laser1);
				state[1] =kalman_update(state[1],data.laser2);
				state[2] =kalman_update(state[2],data.laser3);
				state[3] =kalman_update(state[3],data.laser4);
				state[4] =kalman_update(state[4],data.laser5);
				state[5] =kalman_update(state[5],data.laser6);
				state[6] =kalman_update(state[6],data.laser7);
				state[7] =kalman_update(state[7],data.laser8);
	
				// Pass data procesed into the send structure
				xSemaphoreTake(sem_processed_data,0);
			  	data_processed.laser1 = state[0].x;
			  	data_processed.laser2 = state[1].x;
			  	data_processed.laser3 = state[2].x;
			  	data_processed.laser4 = state[3].x;
			  	data_processed.laser5 = state[4].x;
			  	data_processed.laser6 = state[5].x;
			  	data_processed.laser7 = state[6].x;
			  	data_processed.laser8 = state[7].x;
				data_processed.yaw = yaw;
				data_to_send = 1;
				xSemaphoreGive(sem_processed_data);
				
				
			}
			
		}
		xSemaphoreGive(sem_data);
		vTaskDelayUntil( &xLastWakeTime, xFrequency ); // Espera de 5 milisegundos para tener nuevos datos a una tasa de 200 Hz
	}
}

kalman_state kalman_init(double q, double r, double p, double initial_value) //(process_noise, measurement noise, estimation error, init_value)
{
  kalman_state result;
  result.q = q;
  result.r = r;
  result.p = p;
  result.x = initial_value;

  return result;
}

kalman_state kalman_update(kalman_state state,double medida)
{
  //prediction update
  //omit x = x
  state.p = state.p + state.q;

  //measurement update
  state.k = state.p / (state.p + state.r);
  state.x = state.x + state.k * (medida - state.x);
  state.p = (1 - state.k) * state.p;

  return state;
}

void madgwick_init(void)
{
	beta = betaDef;
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	invSampleFreq = 1.0f / sampleFreqDef;
	anglesComputed = 0;
}

void madgwick_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	//if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
	//	updateIMU(gx, gy, gz, ax, ay, az);
	//	return;
	//}

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = madgwick_invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = madgwick_invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = madgwick_invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;

	// Normalise quaternion
	recipNorm = madgwick_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

float madgwick_invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void madgwick_computeAngles()
{
	roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	anglesComputed = 1;
}

float getYaw() {
        if (!anglesComputed) madgwick_computeAngles();
        return yaw * 57.29578f + 180.0f;
    }

