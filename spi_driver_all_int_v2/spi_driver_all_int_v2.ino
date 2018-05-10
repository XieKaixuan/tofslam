#include <SPI.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <I2Cdev.h>
#include "HMC5883L.h"
#include "MPU6050.h"
#include "MadgwickAHRS.h"
#include "MahonyAHRS.h"

typedef struct {
  double q; //process noise covariance
  double r; //measurement noise covariance
  double x; //value
  double p; //estimation error covariance
  double k; //kalman gain
} kalman_state;

kalman_state state[7];

int send_data = 0, ind=0, sending=0,word_part=0;
volatile uint8_t c=0, data=0;
uint8_t sensor_data[16];
int measure[7];
uint8_t *data_to_send;

VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor3;
VL53L0X Sensor4;
VL53L0X Sensor5;
VL53L0X Sensor6;
VL53L0X Sensor7;

MPU6050 accelgyro;
HMC5883L mag;
//Mahony filter;
Madgwick filter;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

float yawAngle_f;
int yawAngle;

float mag_offsets[3]    ={9.89F, -30.5F, 2.23F };

float mag_softiron_matrix[3][3] = { {0.975, 0.013, -0.001 },
                                    {0.013, 0.953, -0.002},
                                    {-0.001, -0.002, +1.076} };

float mag_field_strength  = 55.34F;

void setup() {

  //Setting up serial
  Serial.begin(115200);

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
  
  ///////////////   Bus I2C   /////////////////////
  Wire.begin();
  accelgyro.setI2CMasterModeEnabled(false);
  accelgyro.setI2CBypassEnabled(true) ;
  accelgyro.setSleepEnabled(false);
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  mag.initialize();
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  accelgyro.setXAccelOffset(-4230);
  accelgyro.setYAccelOffset(-740);
  accelgyro.setZAccelOffset(1250);
  
  accelgyro.setXGyroOffset(65);
  accelgyro.setYGyroOffset(20);
  accelgyro.setZGyroOffset(-25); 
  
  /////////////////////////////////////////////////

  
  
  Serial.print("Seting up ToF lasers...\n");
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

  
  digitalWrite(6, HIGH);
  Sensor1.init(true);
  Sensor1.setAddress(0x11);
  Sensor1.startContinuous(10);

  delay(100);

  digitalWrite(5, HIGH);
  Sensor2.init(true);
  Sensor2.setAddress(0x12);
  Sensor2.startContinuous(10);

  delay(100);

  digitalWrite(4, HIGH);
  Sensor3.init(true);
  Sensor3.setAddress(0x13);
  Sensor3.startContinuous(10);

  delay(100);

  digitalWrite(3, HIGH);
  Sensor4.init(true);
  Sensor4.setAddress(0x14);
  Sensor4.startContinuous(10);

  delay(100);

  digitalWrite(2, HIGH);
  Sensor5.init(true);
  Sensor5.setAddress(0x15);
  Sensor5.startContinuous(10);
  
  data_to_send = &sensor_data[0];
}

void loop() {

  //Init kalman filters 
  state[0] = kalman_init(0.125,1,1023,100);
  state[1] = kalman_init(0.125,1,1023,100);
  state[2] = kalman_init(0.125,1,1023,100);
  state[3] = kalman_init(0.125,1,1023,100);
  state[4] = kalman_init(0.125,1,1023,100);
  //state[5] = kalman_init(0.125,1,1023,100);
  //state[6] = kalman_init(0.125,1,1023,100);

  int aix, aiy, aiz;
  int gix, giy, giz;
  int mix, miy, miz;

  while(1)
  {
    measure[0] = Sensor1.readReg16Bit(Sensor1.RESULT_RANGE_STATUS + 10);
    measure[1] = Sensor2.readReg16Bit(Sensor2.RESULT_RANGE_STATUS + 10);
    measure[2] = Sensor3.readReg16Bit(Sensor3.RESULT_RANGE_STATUS + 10);
    measure[3] = Sensor4.readReg16Bit(Sensor4.RESULT_RANGE_STATUS + 10);
    measure[4] = Sensor5.readReg16Bit(Sensor5.RESULT_RANGE_STATUS + 10);

    //Update kalman filters  
    state[0] =kalman_update(state[0],measure[0]);
    state[1] =kalman_update(state[1],measure[1]);
    state[2] =kalman_update(state[2],measure[2]);
    state[3] =kalman_update(state[3],measure[3]);
    state[4] =kalman_update(state[4],measure[4]);
    state[5] =kalman_update(state[5],measure[4]);
    state[6] =kalman_update(state[6],measure[4]);

    //Get data from accel, gyro and magnetometer
    accelgyro.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);
    mag.getHeading(&mix, &miy, &miz);

    // Apply mag offset compensation (base values in uTesla)
    float x = mix - mag_offsets[0];
    float y = miy - mag_offsets[1];
    float z = miz - mag_offsets[2];

    // Apply mag soft iron error compensation
    float mix = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
    float miy = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
    float miz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
    
  
    //Convert accel and gyro data to correct units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);
    mx = convertRawMag(mix);
    my = convertRawMag(miy);
    mz = convertRawMag(miz);

    filter.update(gx,gy,gz,ax,ay,az,mx,my,mz);

    //rollAngle = filter.getRoll();
    //pitchAngle = filter.getPitch();
    yawAngle = filter.getYaw();

    // To calculate heading in degrees. 0 degree indicates North
    yawAngle_f = atan2(my, mx);
    if(yawAngle_f < 0)
      yawAngle_f += 2 * M_PI;
    yawAngle = (int)(yawAngle_f*180/M_PI);
    
    sensor_data[0] = (uint16_t)state[0].x >> 8;
    sensor_data[1] = (uint16_t)state[0].x & 0xFF;
    sensor_data[2] = (uint16_t)state[1].x >> 8;
    sensor_data[3] = (uint16_t)state[1].x & 0xFF;
    sensor_data[4] = (uint16_t)state[2].x >> 8;
    sensor_data[5] = (uint16_t)state[2].x & 0xFF;
    sensor_data[6] = (uint16_t)state[3].x >> 8;
    sensor_data[7] = (uint16_t)state[3].x & 0xFF;
    sensor_data[8] = (uint16_t)state[4].x >> 8;
    sensor_data[9] = (uint16_t)state[4].x & 0xFF;
    sensor_data[10] = (uint16_t)state[5].x >> 8;
    sensor_data[11] = (uint16_t)state[5].x & 0xFF;
    sensor_data[12] = (uint16_t)state[6].x >> 8;
    sensor_data[13] = (uint16_t)state[6].x & 0xFF;
    sensor_data[14] = (uint16_t)yawAngle >> 8;
    sensor_data[15] = (uint16_t)yawAngle & 0xFF;
    
    /*Serial.print("Measure 1:");
    Serial.print(state[0].x);
    Serial.println();
    Serial.print("Measure 2:");
    Serial.print(state[1].x);
    Serial.println();
    Serial.print("Measure 3:");
    Serial.print(state[2].x);
    Serial.println();
    Serial.print("Measure 4:");
    Serial.print(state[3].x);
    Serial.println();
    Serial.print("Measure 5:");
    Serial.print(state[4].x);
    Serial.println();
    Serial.print("Yaw:");
    Serial.print(yawAngle);
    Serial.println();*/
  }
}

ISR(SPI_STC_vect)
{
  c = SPDR; //Obtenemos el byte del SPI Data Register
  Serial.print(c);
  if(c==80)
  {
    SPDR = *data_to_send;
    data_to_send++;
  }

  if(c==0)
  {
    data_to_send = &sensor_data[0];  
  }
}

kalman_state kalman_init(double q, double r, double p, double initial_value)
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

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = ((float)aRaw * 2.0) / 32768.0f;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = ((float)gRaw * 250.0) / 32768.0;
  return g;
}

float convertRawMag(int mRaw) {
   
  float m = ((float)mRaw * 8) / 2048.0;
  return m;
}
