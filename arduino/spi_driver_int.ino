#include <SPI.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <I2Cdev.h>
#include "HMC5883L.h"
#include "MPU6050.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"

#define XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD                 
#define YGOFFS_TC        0x01                                                                          
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E    
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  // Free-fall
#define FF_DUR           0x1E  // Free-fall
#define MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68
#define MPU6050_ADDRESS 0x68  // Device address when ADO = 0

#define HMC5883L_ADDRESS      0x1E
#define HMC5883L_CONFIG_A     0x00
#define HMC5883L_CONFIG_B     0x01
#define HMC5883L_MODE         0x02
#define HMC5883L_OUT_X_H      0x03
#define HMC5883L_OUT_X_L      0x04
#define HMC5883L_OUT_Z_H      0x05
#define HMC5883L_OUT_Z_L      0x06
#define HMC5883L_OUT_Y_H      0x07
#define HMC5883L_OUT_Y_L      0x08
#define HMC5883L_STATUS       0x09
#define HMC5883L_IDA          0x0A  // should return 0x48
#define HMC5883L_IDB          0x0B  // should return 0x34
#define HMC5883L_IDC          0x0C  // should return 0x33

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mrate { // set magnetometer ODR
  MRT_0075 = 0, // 0.75 Hz ODR
  MRT_015,      // 1.5 Hz
  MRT_030,      // 3.0 Hz
  MRT_075,      // 7.5 Hz
  MRT_15,       // 15 Hz
  MRT_30,       // 30 Hz
  MRT_75,       // 75 Hz ODR    
};

// Specify sensor full scale
int Gscale = GFS_250DPS;
int Ascale = AFS_2G;
uint8_t Mrate = MRT_75;        //  15 Hz ODR 

// Pin definitions
int intPin = 2;

int send_data = 0, ind=0, sending=0,word_part=0;
volatile uint8_t c=0, data=0;
uint8_t sensor_data[34];
int measure[8];
uint8_t *data_to_send;


// measure variance
unsigned long suma= 0;
int media, l = 0, cov = 0;

VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor3;
VL53L0X Sensor4;
VL53L0X Sensor5;
VL53L0X Sensor6;
VL53L0X Sensor7;
VL53L0X Sensor8;

MPU6050 accelgyro;
HMC5883L mag;
//Mahony filter;
Madgwick filter;

unsigned long microsPerReading, microsPrevious, microsNow;

int16_t ai[3];
float a;
int16_t gi[3];
float g[3];
int16_t mi[3];
float m[3];

float mag_offsets[3]    ={1.44F, -0.97F, 12.69F };

float mag_softiron_matrix[3][3] = { {0.950, 0.005, 0.031 },
                                    {0.007, 0.904, -0.013},
                                    {0.032, -0.013, 1.168} };

float mag_field_strength  = 49.03F;

void setup() {

  //Setting up serial
  Serial.begin(115200);

  //// Set up pins ////
  
  
  pinMode(9, OUTPUT);
  digitalWrite(9,LOW);
  pinMode(8, OUTPUT);
  digitalWrite(8,LOW);
  pinMode(7, OUTPUT);
  digitalWrite(7,LOW);
  pinMode(6, OUTPUT);
  digitalWrite(6,LOW);  
  pinMode(5, OUTPUT);
  digitalWrite(5,LOW);
  pinMode(4, OUTPUT);
  digitalWrite(4,LOW);
  pinMode(3, OUTPUT);
  digitalWrite(3,LOW);
  pinMode(2, OUTPUT);
  digitalWrite(2,LOW);
  
  ///////////////   Bus I2C   /////////////////////
  Wire.begin();
  Wire.setClock(400000);
  //accelgyro.setI2CMasterModeEnabled(false);
  //accelgyro.setI2CBypassEnabled(true) ;
  //accelgyro.setSleepEnabled(false);
  
  // initialize device
  Serial.println("Initializing I2C devices...");
  initMPU6050();
  initHMC5883L(); // Initialize and configure magnetometer
  //accelgyro.setRate(200);
  //mag.initialize();
  //mag.setDataRate(6); // 75 Hz

  //// Testing connections ////
  Serial.println("Testing device connections...");
  

  accelgyro.setXAccelOffset(-4383);
  accelgyro.setYAccelOffset(-770);
  accelgyro.setZAccelOffset(1263);
  
  accelgyro.setXGyroOffset(69);
  accelgyro.setYGyroOffset(23);
  accelgyro.setZGyroOffset(-25); 
  
  ///////////////////////////////////////////////// 
  Serial.print("Seting up ToF lasers...\n");

  digitalWrite(9, HIGH);
  Sensor8.init(true);
  Sensor8.setAddress(0x28);
  Sensor8.setSignalRateLimit(0.1); // lower the return signal rate limit (default is 0.25 MCPS)
  Sensor8.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18); // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  Sensor8.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  Sensor8.startContinuous();
  
  delay(100);

  digitalWrite(8, HIGH);
  Sensor7.init(true);
  Sensor7.setAddress(0x27);
  Sensor7.setSignalRateLimit(0.1); // lower the return signal rate limit (default is 0.25 MCPS)
  Sensor7.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18); // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  Sensor7.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  Sensor7.startContinuous();

  delay(100);

  digitalWrite(7, HIGH);
  Sensor6.init(true);
  Sensor6.setAddress(0x26);
  Sensor6.setSignalRateLimit(0.1); // lower the return signal rate limit (default is 0.25 MCPS)
  Sensor6.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18); // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  Sensor6.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  Sensor6.startContinuous();

  delay(100);
  
  digitalWrite(6, HIGH);
  Sensor5.init(true);
  Sensor5.setAddress(0x25);
  Sensor5.setSignalRateLimit(0.1); // lower the return signal rate limit (default is 0.25 MCPS)
  Sensor5.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18); // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  Sensor5.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  Sensor5.startContinuous();

  delay(100);

  digitalWrite(5, HIGH);
  Sensor4.init(true);
  Sensor4.setAddress(0x24);
  Sensor4.setSignalRateLimit(0.1); // lower the return signal rate limit (default is 0.25 MCPS)
  Sensor4.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18); // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  Sensor4.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  Sensor4.startContinuous();

  delay(100);

  digitalWrite(4, HIGH);
  Sensor3.init(true);
  Sensor3.setAddress(0x23);
  Sensor3.setSignalRateLimit(0.1); // lower the return signal rate limit (default is 0.25 MCPS)
  Sensor3.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18); // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  Sensor3.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  Sensor3.startContinuous();

  delay(100);

  digitalWrite(3, HIGH);
  Sensor2.init(true);
  Sensor2.setAddress(0x22);
  Sensor2.setSignalRateLimit(0.1); // lower the return signal rate limit (default is 0.25 MCPS)
  Sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18); // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  Sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  Sensor2.startContinuous();

  delay(100);

  digitalWrite(2, HIGH);
  Sensor1.init(true);
  Sensor1.setAddress(0x21);
  Sensor1.setSignalRateLimit(0.1); // lower the return signal rate limit (default is 0.25 MCPS)
  Sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18); // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  Sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  Sensor1.startContinuous();
  
  data_to_send = &sensor_data[0];

   ///////////////Iniciamos SPI/////////////////////
  //Modo esclavo SPI
  SPCR |= bit(SPE);
  //Salida lo ponemos a salida
  pinMode(MISO,OUTPUT);
  // Iniciamos el bus
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  //Habilitamos la interrupción
  SPI.attachInterrupt();
  
  /////////////////////////////////////////////////
  
}

void loop() {

  uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
  uint32_t Now = 0;
  uint32_t count_mag = 0, count_laser = 0; 
  
  while(1)
  {

      if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {

       
        
        count_mag += 1;
        count_laser +=1;
        //Get data from accel, gyro and magnetometer
        readAccelData(ai);
        readGyroData(gi);  // Read the x/y/z adc values
     
        if(count_mag>=3)
        {
          
          count_mag = 0;
          
          if(readByte(HMC5883L_ADDRESS, HMC5883L_STATUS) & 0x01) { // If data ready bit set, then read magnetometer data

          Now = micros();
          lastUpdate = Now - lastUpdate;
          //Serial.print(lastUpdate);
          //Serial.print("\t usec between data ready\n");
          lastUpdate = Now; 
  
            // Read the x/y/z adc values
            readMagData(mi);  
            
           // Apply mag offset compensation (base values in uTesla)
            float x = mi[0] - mag_offsets[0];
            float y = mi[1] - mag_offsets[1];
            float z = mi[2] - mag_offsets[2];
        
            // Apply mag soft iron error compensation
            m[0] = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
            m[1] = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
            m[2] = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
            cli();
            sensor_data[28] = (uint8_t)(mi[0] >> 8);
            sensor_data[29] = (uint8_t)(mi[0] & 0xFF);
            sensor_data[30] = (uint8_t)(mi[1] >> 8);
            sensor_data[31] = (uint8_t)(mi[1] & 0xFF);
            sensor_data[32] = (uint8_t)(mi[2] >> 8);
            sensor_data[33] = (uint8_t)(mi[2] & 0xFF);
            sei();
          }
          cli();
          sensor_data[16] = (uint8_t)(ai[0] >> 8);
          sensor_data[17] = (uint8_t)(ai[0] & 0xFF);
          sensor_data[18] = (uint8_t)(ai[1] >> 8);
          sensor_data[19] = (uint8_t)(ai[1] & 0xFF);
          sensor_data[20] = (uint8_t)(ai[2] >> 8);
          sensor_data[21] = (uint8_t)(ai[2] & 0xFF);
          sensor_data[22] = (uint8_t)(gi[0] >> 8);
          sensor_data[23] = (uint8_t)(gi[0] & 0xFF);
          sensor_data[24] = (uint8_t)(gi[1] >> 8);
          sensor_data[25] = (uint8_t)(gi[1] & 0xFF);
          sensor_data[26] = (uint8_t)(gi[2] >> 8);
          sensor_data[27] = (uint8_t)(gi[2] & 0xFF);
          sei();
        }
        if(count_laser>=7)
          {
            
            count_laser = 0;
            
            // Get Laser data  
            measure[0] = Sensor1.readReg16Bit(Sensor1.RESULT_RANGE_STATUS + 10);    
            measure[1] = Sensor2.readReg16Bit(Sensor2.RESULT_RANGE_STATUS + 10);  
            measure[2] = Sensor3.readReg16Bit(Sensor3.RESULT_RANGE_STATUS + 10);  
            measure[3] = Sensor4.readReg16Bit(Sensor4.RESULT_RANGE_STATUS + 10);  
            measure[4] = Sensor5.readReg16Bit(Sensor5.RESULT_RANGE_STATUS + 10);  
            measure[5] = Sensor6.readReg16Bit(Sensor6.RESULT_RANGE_STATUS + 10);  
            measure[6] = Sensor7.readReg16Bit(Sensor7.RESULT_RANGE_STATUS + 10);  
            measure[7] = Sensor8.readReg16Bit(Sensor8.RESULT_RANGE_STATUS + 10);  

            
            /*
            if(l<1000)
            {
                suma += measure[7];
                l++;            
            }
            if(l==1000)
            {
                media = suma/1000;
                suma = 0;
                Serial.print(media);
                Serial.println();
            }
            if(l >= 1000)
            {
                suma += (measure[7]-media)*(measure[7]-media);
                l++;
            }
            if(l== 2000)
            {
              cov = suma/1000;
              Serial.print(cov);
              Serial.println();
              suma = 0;
              l = 0;
            }*/
            
            sensor_data[0] = (uint16_t)measure[0] >> 8;
            sensor_data[1] = (uint16_t)measure[0] & 0xFF;
            sensor_data[2] = (uint16_t)measure[1] >> 8;
            sensor_data[3] = (uint16_t)measure[1] & 0xFF;
            sensor_data[4] = (uint16_t)measure[2] >> 8;
            sensor_data[5] = (uint16_t)measure[2] & 0xFF;
            sensor_data[6] = (uint16_t)measure[3] >> 8;
            sensor_data[7] = (uint16_t)measure[3] & 0xFF;
            sensor_data[8] = (uint16_t)measure[4] >> 8;
            sensor_data[9] = (uint16_t)measure[4] & 0xFF;
            sensor_data[10] = (uint16_t)measure[5] >> 8;
            sensor_data[11] = (uint16_t)measure[5] & 0xFF;
            sensor_data[12] = (uint16_t)measure[6] >> 8;
            sensor_data[13] = (uint16_t)measure[6] & 0xFF;
            sensor_data[14] = (uint16_t)measure[7]>> 8;
            sensor_data[15] = (uint16_t)measure[7] & 0xFF;
            
          }
      }
             
  }
}

ISR(SPI_STC_vect)
{
  c = SPDR; //Obtenemos el byte del SPI Data Register
  SPDR = *data_to_send;
  data_to_send++;
  
  //Serial.print(c);
  

  if(c==0)
  {
    data_to_send = &sensor_data[0];  
  }
}


void initMPU6050()
{  
// wake up device-don't need this here if using calibration function below
//  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
//  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // get stable time source
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

 // Configure Gyro and Accelerometer
 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
 // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
 // Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
  writeByte(MPU6050_ADDRESS, CONFIG, 0x03);  
 
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
   
 // Set accelerometer configuration
  c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);    
   writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

void initHMC5883L()
{  
 // Set magnetomer ODR; default is 15 Hz 
  writeByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A, Mrate << 2);   
  writeByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_B, 0x00);  // set gain (bits[7:5]) to maximum resolution of 0.73 mG/LSB
  writeByte(HMC5883L_ADDRESS, HMC5883L_MODE, 0x80 );     // enable continuous data mode
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ; 
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ; 
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(HMC5883L_ADDRESS, HMC5883L_OUT_X_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[4] << 8) | rawData[5];  
  destination[2] = ((int16_t)rawData[2] << 8) | rawData[3]; 
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}


uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
        Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}


