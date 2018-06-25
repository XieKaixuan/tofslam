#include <main.h>

uint32_t now = 0, lastUpdate = 0;

void get_data(void *pvParameters)
{
	TickType_t xLastWakeTime;
 	const TickType_t xFrequency = 5/ portTICK_PERIOD_MS;
	xSemaphoreTake(sem_print_data,0);
	print_data = 0;	
    xSemaphoreGive(sem_print_data);
	spi_init(1, SPI_MODE0, SPI_FREQ_DIV_1M, 1, SPI_BIG_ENDIAN, true); // init SPI module	

	xLastWakeTime = xTaskGetTickCount(); // Get current time
    while(1)
	{
		// To check periodicity
		//now = sdk_system_get_time();
		/*lastUpdate = now - lastUpdate;
		printf("%d \n",aux/1000);
		lastUpdate = now;*/
		
		xSemaphoreTake(sem_data,0);
		data = get_sensor_data();
		//printf("Laser 1: %d\n",data.laser1);
		data_to_process = 1; 
		xSemaphoreGive(sem_data);
		vTaskDelayUntil( &xLastWakeTime, xFrequency ); // Espera de 5 milisegundos para tener nuevos datos a una tasa de 200 Hz
    }
	
}

sensor_data get_sensor_data(){

	
	uint16_t received_data_msb,received_data_lsb;
	int ai[3], gi[3], mi[3], i, wait = 1;

	spi_transfer_8(1,80); //notify the slave the transaction is about to start
	//Start receiving data
		
	printf(" "); //Insert a little wait to avoid errors on SPI (Arduino doesn't have time to change SPI data register if I don't wait)
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser1=(int)((received_data_msb << 8) | received_data_lsb);

	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser2=(int)((received_data_msb << 8) | received_data_lsb);
	

	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser3=(int)((received_data_msb << 8) | received_data_lsb);

	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser4=(int)((received_data_msb << 8) | received_data_lsb);

	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser5=(int)((received_data_msb << 8) | received_data_lsb);

	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser6=(int)((received_data_msb << 8) | received_data_lsb);

	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser7=(int)((received_data_msb << 8) | received_data_lsb);

	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser8=(int)((received_data_msb << 8) | received_data_lsb);

	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	ai[0] = (((int)((received_data_msb << 8) | received_data_lsb)) > 32768) ? ((int)((received_data_msb << 8) | received_data_lsb)) - 65536 : ((int)((received_data_msb << 8) | received_data_lsb));

	
	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	ai[1] = (((int)((received_data_msb << 8) | received_data_lsb)) > 32768) ? ((int)((received_data_msb << 8) | received_data_lsb)) - 65536 : ((int)((received_data_msb << 8) | received_data_lsb));

	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	ai[2] = (((int)((received_data_msb << 8) | received_data_lsb)) > 32768) ? ((int)((received_data_msb << 8) | received_data_lsb)) - 65536 : ((int)((received_data_msb << 8) | received_data_lsb));

	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	gi[0] = (((int)((received_data_msb << 8) | received_data_lsb)) > 32768) ? ((int)((received_data_msb << 8) | received_data_lsb)) - 65536 : ((int)((received_data_msb << 8) | received_data_lsb));	

	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	gi[1] = (((int)((received_data_msb << 8) | received_data_lsb)) > 32768) ? ((int)((received_data_msb << 8) | received_data_lsb)) - 65536 : ((int)((received_data_msb << 8) | received_data_lsb));
	
	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	gi[2] = (((int)((received_data_msb << 8) | received_data_lsb)) > 32768) ? ((int)((received_data_msb << 8) | received_data_lsb)) - 65536 : ((int)((received_data_msb << 8) | received_data_lsb));
	
	printf(" ");
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	mi[0] = (((int)((received_data_msb << 8) | received_data_lsb)) > 32768) ? ((int)((received_data_msb << 8) | received_data_lsb)) - 65536 : ((int)((received_data_msb << 8) | received_data_lsb));

	printf(" ");	
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,80);
	mi[1] = (((int)((received_data_msb << 8) | received_data_lsb)) > 32768) ? ((int)((received_data_msb << 8) | received_data_lsb)) - 65536 : ((int)((received_data_msb << 8) | received_data_lsb));

	printf(" ");	
	received_data_msb=spi_transfer_8(1,80);
	received_data_lsb=spi_transfer_8(1,0);
	mi[2] = (((int)((received_data_msb << 8) | received_data_lsb)) > 32768) ? ((int)((received_data_msb << 8) | received_data_lsb)) - 65536 : ((int)((received_data_msb << 8) | received_data_lsb));
	//printf("%d \t %d\n",received_data_msb,received_data_lsb);	
	

	// Convert from raw data to g, deg/s and G
	convertRawAcceleration(data.a, ai);
	convertRawAcceleration(data.g, gi);
	convertRawAcceleration(data.m, mi);
	
	return data;
}
	
void convertRawAcceleration(float * buf,int aRaw[3]) 
{
	// since we are using 2G range
	// -2g maps to a raw value of -32768
	// +2g maps to a raw value of 32767
	
	buf[0] = ((float)aRaw[0] * 2.0) / 32768.0f;
	buf[1] = ((float)aRaw[1] * 2.0) / 32768.0f;
	buf[2] = ((float)aRaw[2] * 2.0) / 32768.0f;

}

void convertRawGyro(float * buf, int gRaw[3]) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  buf[0] = ((float)gRaw[0] * 250.0) / 32768.0;
  buf[1] = ((float)gRaw[1] * 250.0) / 32768.0;
  buf[2] = ((float)gRaw[2] * 250.0) / 32768.0;
 
}

void convertRawMag(float * buf, int mRaw[3]) {
   
  buf[0] = ((float)mRaw[0] * 8) / 2048.0;
  buf[1] = ((float)mRaw[1] * 8) / 2048.0;
  buf[2] = ((float)mRaw[2] * 8) / 2048.0;
  
}

void show_data(void *pvParameters)
{
    while(1)
    {
    	xSemaphoreTake(sem_print_data,0);
		if(print_data==1)
		{
			xSemaphoreGive(sem_print_data);
			xSemaphoreTake(sem_data,0);
        	printf("Received 1: %d\n",data.laser1);
        	printf("Received 2: %d\n",data.laser2);
        	printf("Received 3: %d\n",data.laser3);
        	printf("Received 4: %d\n",data.laser4);
    		printf("Received 5: %d\n",data.laser5);
    		printf("Received 6: %d\n",data.laser6);
    		printf("Received 7: %d\n",data.laser7);
			printf("Received 8: %d\n",data.laser8);
			printf("ax: %d\n",data.a[0]);	
			printf("ay: %d\n",data.a[1]);	
			printf("az: %d\n",data.a[2]);	
			printf("gx: %d\n",data.g[0]);	
			printf("gy: %d\n",data.g[1]);	
			printf("gz: %d\n",data.g[2]);	
			printf("mx: %d\n",data.m[0]);	
			printf("my: %d\n",data.m[1]);	
			printf("mz: %d\n",data.m[2]);	
			xSemaphoreGive(sem_data);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
		
	}
}
