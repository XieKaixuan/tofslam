#include <main.h>

void get_data(void *pvParameters)
{
	xSemaphoreTake(sem_print_data,0);
	print_data = 0;	
    xSemaphoreGive(sem_print_data);
	spi_init(1, SPI_MODE0, SPI_FREQ_DIV_1M, 1, SPI_BIG_ENDIAN, true); // init SPI module	
    while(1)
	{
		xSemaphoreTake(sem_data,0);
		data = get_sensor_data();
		data_to_send = 1;
		xSemaphoreGive(sem_data);
		vTaskDelay(1 / portTICK_PERIOD_MS);
    }
	
}	

sensor_data get_sensor_data(){

	uint8_t received_data_msb,received_data_lsb;

	spi_transfer_8(1,80); //notify the slave the transaction is about to start
	//Start receiving data
	vTaskDelay(1);
	received_data_msb=spi_transfer_8(1,80);
	vTaskDelay(1);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser1=(int)((received_data_msb << 8) | received_data_lsb);
	vTaskDelay(1);
	received_data_msb=spi_transfer_8(1,80);
	vTaskDelay(1);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser2=(int)((received_data_msb << 8) | received_data_lsb);
	vTaskDelay(1);
	received_data_msb=spi_transfer_8(1,80);
	vTaskDelay(1);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser3=(int)((received_data_msb << 8) | received_data_lsb);
	vTaskDelay(1);
	received_data_msb=spi_transfer_8(1,80);
	vTaskDelay(1);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser4=(int)((received_data_msb << 8) | received_data_lsb);
	vTaskDelay(1);
	received_data_msb=spi_transfer_8(1,80);
	vTaskDelay(1);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser5=(int)((received_data_msb << 8) | received_data_lsb);
	vTaskDelay(1);
	received_data_msb=spi_transfer_8(1,80);
	vTaskDelay(1);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser6=(int)((received_data_msb << 8) | received_data_lsb);
	vTaskDelay(1);
	received_data_msb=spi_transfer_8(1,80);
	vTaskDelay(1);
	received_data_lsb=spi_transfer_8(1,80);
	data.laser7=(int)((received_data_msb << 8) | received_data_lsb);
	vTaskDelay(1);
	received_data_msb=spi_transfer_8(1,80);
	vTaskDelay(1);
	received_data_lsb=spi_transfer_8(1,0);
	data.imu_yaw=((int)((received_data_msb << 8) | received_data_lsb));

	printf("Laser 1: %d\n",data.laser1);
	printf("Laser 2: %d\n",data.laser2);
	printf("Laser 3: %d\n",data.laser3);
	printf("Laser 4: %d\n",data.laser4);
	printf("Laser 5: %d\n",data.laser5);
	printf("Laser 6: %d\n",data.laser6);
	printf("Laser 7: %d\n",data.laser7);
	printf("Yaw: %d\n",data.imu_yaw);	

	return data;
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
        	printf("Received 1: %f\n",data.laser1);
        	printf("Received 2: %f\n",data.laser2);
        	printf("Received 3: %f\n",data.laser3);
        	printf("Received 4: %f\n",data.laser4);
    		printf("Received 5: %f\n",data.laser5);
    		printf("Received 6: %f\n",data.laser6);
    		printf("Received 7: %f\n",data.laser7);
    		printf("Received yaw: %f\n",data.imu_yaw);
    		printf("Received pitch: %f\n",data.imu_pitch);
    		printf("Received roll: %f\n",data.imu_roll);
			xSemaphoreGive(sem_data);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
		
	}
}
