// Wire Slave Transmitter and receiver
//Uno, Ethernet A4 (SDA), A5 (SCL)
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

Adafruit_MPU6050 mpu;

// Include the required Wire library for I2C<br>#include <Wire.h>
#define SLAVE_ADDR 0x51
#define SEND_LENGTH 0X51
#define SEND_DATA 0X52
uint8_t active_command = 0x17;
uint8_t i = 0;
int32_t dummylong=32629527;
int32_t dummylat=39972918;
float flat, flon;
unsigned long age;



typedef struct readData{
  int16_t a_x;
  int16_t a_y;
  int16_t a_z;

  int16_t g_x;
  int16_t g_y;
  int16_t g_z;

  int16_t temp;
  int16_t padding=0;

  int32_t longtitude=dummylong;
  int32_t latitude=dummylat;
};
readData Datas;

uint8_t len=sizeof(Datas);

TinyGPS gps;
SoftwareSerial ss(3, 1);


//write
void receiveEvent(int bytes) {
  while(Wire1.available())
  {
    int data=Wire1.read();
    if(data==0x17)
    {
      active_command=data;
      Wire1.slaveWrite(&len,1);
      Serial.print(data);
      Serial.println(" Active Command: 0x17 Will tell STM32 about the SIZE");
    }else if(data==0x18)
    {
      active_command=data;
      Wire1.slaveWrite((uint8_t *)&Datas,len);
      Serial.print(data);
      Serial.println(" Active Command: 0x18 Will send STM32 the DATA");
      //printData();
    }
  }
}

//read
void requestEvent() {

    //Serial.printf("Request BEGUN \n");
    Wire1.slaveWrite((uint8_t *)&Datas,len);
    //Serial.printf("Request phase is over \n");

}


void setup() {
  Serial.begin(115200);
    while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.setDebugOutput(true);
    Serial.println("Adafruit MPU6050 test!");
    Serial.println(sizeof(readData));
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  ss.begin(115200);
  }
  Serial.println("MPU6050 Found!");
  Serial.println(len);
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
  Wire1.begin(SLAVE_ADDR,26,27,100000);
  // Attach a function to trigger when something is received.
  Wire1.onReceive(receiveEvent);
  Wire1.onRequest(requestEvent);
}

void printData()
{
  Serial.print("Acceleration: ");
  Serial.print(Datas.a_x);
  Serial.print(" ,");
  Serial.print(Datas.a_y);
  Serial.print(" ,");
  Serial.print(Datas.a_z);
  Serial.print("\t\t");

  Serial.print("GYRO: ");
  Serial.print(Datas.g_x);
  Serial.print(" ,");
  Serial.print(Datas.g_y);
  Serial.print(" ,");
  Serial.print(Datas.g_z);
  Serial.print("\t\t");

  Serial.print("TEMP: ");
  Serial.print(Datas.temp);
  Serial.print("\t\t");
  Serial.print("COORDINATES LAT: ");
  Serial.print(Datas.latitude);
  Serial.print("\t");
  Serial.print("COORDINATES LONG: ");
  Serial.print(Datas.longtitude);
  Serial.println();
}

void loop() {
    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gps.f_get_position(&flat, &flon);
  Datas.latitude=flat*pow(10,6);
  Datas.longtitude=flat*pow(10,6);


  Datas.a_x=a.acceleration.x*1000;
  Datas.a_y=a.acceleration.y*1000;
  Datas.a_z=a.acceleration.z*1000;
  Datas.g_x=g.gyro.x*1000;
  Datas.g_y=g.gyro.y*1000;
  Datas.g_z=g.gyro.z*1000;
  Datas.temp=temp.temperature*1000;

}