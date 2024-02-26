bool handshake=0;
char handshake_Success_msg[]="Arduino_SUCCESS";
char handshake_Fail_msg[]="Arduino_FAILEDD";
char expected_handshake[]="handshake";
int len=sizeof(expected_handshake)-1;
char handshake_ReadBuff[sizeof(expected_handshake)];
String handshake_ReadString;
char readbuff[128];
typedef struct readData{
  int16_t a_x;
  int16_t a_y;
  int16_t a_z;

  int16_t g_x;
  int16_t g_y;
  int16_t g_z;

  int16_t temp;
  int16_t padding=0;

  int32_t latitude=0;
  int32_t longtitude=0;
};
readData ESP1_DATA;
readData ESP2_DATA;

typedef struct readData_FLOAT{
  float a_x;
  float a_y;
  float a_z;

  float g_x;
  float g_y;
  float g_z;

  float temp;
  float padding=0;

  double latitude=0;
  double longtitude=0;
};
readData_FLOAT ESP1_FLOAT_DATA;
readData_FLOAT ESP2_FLOAT_DATA;


void Handshake_begin()
{
  handshake=0;
  Serial.setTimeout(10);
      while(handshake!=1)
      {
          while(!Serial.available());
          delay(10);
          if(Serial.available()>=9)
          {
            handshake_ReadString=Serial.readString();
            
            if(handshake_ReadString==expected_handshake)
            {
              Serial.print(handshake_ReadString);
              handshake=1;
            }else{
              Serial.print(handshake_ReadString);
            }
          }

        
      }

}


void Reset_handshake_ReadBuff()
{
    for(int j=0;j<=len;j++)
  {
    handshake_ReadBuff[j]=0;
  }
}

void printData_ESP1()
{
  Serial.println("ESP1 DATAS ARE BELOW");
  Serial.print("Acceleration: ");
  Serial.print(ESP1_DATA.a_x);
  Serial.print(" ,");
  Serial.print(ESP1_DATA.a_y);
  Serial.print(" ,");
  Serial.print(ESP1_DATA.a_z);
  Serial.print("\t\t");

  Serial.print("GYRO: ");
  Serial.print(ESP1_DATA.g_x);
  Serial.print(" ,");
  Serial.print(ESP1_DATA.g_y);
  Serial.print(" ,");
  Serial.print(ESP1_DATA.g_z);
  Serial.print("\t\t");

  Serial.print("TEMP: ");
  Serial.print(ESP1_DATA.temp);
  Serial.print("\t");
  Serial.print(ESP1_DATA.padding);
  Serial.print("\t\t");
  Serial.print("COORDINATES LAT: ");
  Serial.print(ESP1_DATA.latitude);
  Serial.print("\t");
  Serial.print("COORDINATES LONG: ");
  Serial.print(ESP1_DATA.longtitude);
  Serial.println();
}

void printData_ESP2()
{
  Serial.println("ESP2 DATAS ARE BELOW");
  Serial.print("Acceleration: ");
  Serial.print(ESP2_DATA.a_x);
  Serial.print(" ,");
  Serial.print(ESP2_DATA.a_y);
  Serial.print(" ,");
  Serial.print(ESP2_DATA.a_z);
  Serial.print("\t\t");

  Serial.print("GYRO: ");
  Serial.print(ESP2_DATA.g_x);
  Serial.print(" ,");
  Serial.print(ESP2_DATA.g_y);
  Serial.print(" ,");
  Serial.print(ESP2_DATA.g_z);
  Serial.print("\t\t");

  Serial.print("TEMP: ");
  Serial.print(ESP2_DATA.temp);
  Serial.print("\t");
  Serial.print(ESP1_DATA.padding);
  Serial.print("\t\t");
  Serial.print("COORDINATES LAT: ");
  Serial.print(ESP2_DATA.latitude);
  Serial.print("\t");
  Serial.print("COORDINATES LONG: ");
  Serial.print(ESP2_DATA.longtitude);
  Serial.println();
}

void printFLOATData_ESP1()
{
    Serial.println("ESP1 DATAS ARE BELOW");
  Serial.print("Acceleration: ");
  Serial.print(ESP1_FLOAT_DATA.a_x,3);
  Serial.print(" ,");
  Serial.print(ESP1_FLOAT_DATA.a_y,3);
  Serial.print(" ,");
  Serial.print(ESP1_FLOAT_DATA.a_z,3);
  Serial.print("\t\t");

  Serial.print("GYRO: ");
  Serial.print(ESP1_FLOAT_DATA.g_x,3);
  Serial.print(" ,");
  Serial.print(ESP1_FLOAT_DATA.g_y,3);
  Serial.print(" ,");
  Serial.print(ESP1_FLOAT_DATA.g_z,3);
  Serial.print("\t\t");

  Serial.print("TEMP: ");
  Serial.print(ESP1_FLOAT_DATA.temp,3);
  Serial.print("\t\t");
  Serial.print("COORDINATES LAT: ");
  Serial.print(ESP1_FLOAT_DATA.latitude,6);
  Serial.print("\t");
  Serial.print("COORDINATES LONG: ");
  Serial.print(ESP1_FLOAT_DATA.longtitude,6);
  Serial.println();
}

void printFLOATData_ESP2()
{
    Serial.println("ESP2 DATAS ARE BELOW");
  Serial.print("Acceleration: ");
  Serial.print(ESP2_FLOAT_DATA.a_x,3);
  Serial.print(" ,");
  Serial.print(ESP2_FLOAT_DATA.a_y,3);
  Serial.print(" ,");
  Serial.print(ESP2_FLOAT_DATA.a_z,3);
  Serial.print("\t\t");

  Serial.print("GYRO: ");
  Serial.print(ESP2_FLOAT_DATA.g_x,3);
  Serial.print(" ,");
  Serial.print(ESP2_FLOAT_DATA.g_y,3);
  Serial.print(" ,");
  Serial.print(ESP2_FLOAT_DATA.g_z,3);
  Serial.print("\t\t");

  Serial.print("TEMP: ");
  Serial.print(ESP2_FLOAT_DATA.temp,3);
  Serial.print("\t\t");
  Serial.print("COORDINATES LAT: ");
  Serial.print(ESP2_FLOAT_DATA.latitude,6);
  Serial.print("\t");
  Serial.print("COORDINATES LONG: ");
  Serial.print(ESP2_FLOAT_DATA.longtitude,6);
  Serial.println();
}


void setup() {
  Serial.begin(115200);
  
  // Define the LED pin as Output
  pinMode (13, OUTPUT);
  Serial.setTimeout(4);
  Serial.println("Arduino UART Receiver");
  Serial.println("-----------------------------");
  Serial.println(sizeof(expected_handshake));
  Serial.println(sizeof(char));
}


void loop() {
  if(!handshake)
  {
    Handshake_begin();
    String read=Serial.readString();
    Serial.println();
  }else
  {
    while(Serial.available()<12);
    delay(5);

    Serial.readBytes((byte*)&ESP1_DATA.a_x, 2);
    Serial.readBytes((byte*)&ESP1_DATA.a_y, 2);
    Serial.readBytes((byte*)&ESP1_DATA.a_z, 2);
    Serial.readBytes((byte*)&ESP1_DATA.g_x, 2);
    Serial.readBytes((byte*)&ESP1_DATA.g_y, 2);
    Serial.readBytes((byte*)&ESP1_DATA.g_z, 2);
    Serial.readBytes((byte*)&ESP1_DATA.temp, 2);
    Serial.readBytes((byte*)&ESP1_DATA.padding, 2);

    Serial.readBytes((byte*)&ESP1_DATA.longtitude, 4);

    Serial.readBytes((byte*)&ESP1_DATA.latitude, 4);

    ESP1_FLOAT_DATA.a_x=ESP1_DATA.a_x/1000.;
    ESP1_FLOAT_DATA.a_y=ESP1_DATA.a_y/1000.;
    ESP1_FLOAT_DATA.a_z=ESP1_DATA.a_z/1000.;
    ESP1_FLOAT_DATA.g_x=ESP1_DATA.g_x/1000.;
    ESP1_FLOAT_DATA.g_y=ESP1_DATA.g_y/1000.;
    ESP1_FLOAT_DATA.g_z=ESP1_DATA.g_z/1000.;
    ESP1_FLOAT_DATA.temp=ESP1_DATA.temp/1000.;
    ESP1_FLOAT_DATA.longtitude=ESP1_DATA.longtitude/1000000.;
    ESP1_FLOAT_DATA.latitude=ESP1_DATA.latitude/1000000.;
    
    printFLOATData_ESP1();
    while(Serial.available()<12);
    delay(5);
    
    Serial.readBytes((byte*)&ESP2_DATA.a_x, 2);
    Serial.readBytes((byte*)&ESP2_DATA.a_y, 2);
    Serial.readBytes((byte*)&ESP2_DATA.a_z, 2);
    Serial.readBytes((byte*)&ESP2_DATA.g_x, 2);
    Serial.readBytes((byte*)&ESP2_DATA.g_y, 2);
    Serial.readBytes((byte*)&ESP2_DATA.g_z, 2);
    Serial.readBytes((byte*)&ESP2_DATA.temp, 2);
    Serial.readBytes((byte*)&ESP2_DATA.padding, 2);

 
    Serial.readBytes((byte*)&ESP2_DATA.longtitude, 4);

    Serial.readBytes((byte*)&ESP2_DATA.latitude, 4);

    ESP2_FLOAT_DATA.a_x=ESP2_DATA.a_x/1000.;
    ESP2_FLOAT_DATA.a_y=ESP2_DATA.a_y/1000.;
    ESP2_FLOAT_DATA.a_z=ESP2_DATA.a_z/1000.;
    ESP2_FLOAT_DATA.g_x=ESP2_DATA.g_x/1000.;
    ESP2_FLOAT_DATA.g_y=ESP2_DATA.g_y/1000.;
    ESP2_FLOAT_DATA.g_z=ESP2_DATA.g_z/1000.;
    ESP2_FLOAT_DATA.temp=ESP2_DATA.temp/1000.;
    ESP2_FLOAT_DATA.longtitude=ESP2_DATA.longtitude/1000000.;
    ESP2_FLOAT_DATA.latitude=ESP2_DATA.latitude/1000000.;
/*
    Serial.print(ESP1_FLOAT_DATA.a_x,2);
    Serial.println(ESP1_FLOAT_DATA.a_y);
    Serial.println(ESP1_FLOAT_DATA.a_z);
    Serial.println(ESP1_FLOAT_DATA.g_x);
    Serial.println(ESP1_FLOAT_DATA.g_y);
    Serial.println(ESP1_FLOAT_DATA.g_z);
    Serial.println(ESP1_FLOAT_DATA.longtitude);
    Serial.println(ESP1_FLOAT_DATA.latitude);

*/
    printFLOATData_ESP2();

  }


}