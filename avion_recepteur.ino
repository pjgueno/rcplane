////AVION

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <esp32-hal-ledc.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define LED_PIN 2

RF24 radio(4, 5); // CE, CSN 

const byte address1[6] = "00001"; // telecommande => avion

struct DataControl
{
  uint16_t manetteG_X;
  uint16_t manetteG_Y;
  uint16_t manetteD_X;
  uint16_t manetteD_Y;
  uint16_t motor;
};
DataControl dataRecved;


struct DataNavigation
{
  int16_t tangage;
  int16_t roulis;
  int16_t gravite;
  int16_t temperature;
  int16_t valeur;
};
DataNavigation dataToSend;

//int16_t dataToSend[5]={0,0,0,0,0};


int lacetChannel = 1;
int roulisChannelG = 2;
int roulisChannelD = 3;
int tangageChannelG = 4;
int tangageChannelD = 5;
int moteurChannel = 6;
int servofreq = 50;
int servoResolution = 16; 


class Lacet
{
    int pin_L;
    int MIN_PULSE_WIDTH;
    int MAX_PULSE_WIDTH;
    
  public:
Lacet(int pin){ 
  pin_L = pin;
 MIN_PULSE_WIDTH = 544;
 MAX_PULSE_WIDTH = 2400;
  ledcSetup(lacetChannel,servofreq,servoResolution);
  ledcAttachPin(pin_L,lacetChannel);
    }


   int pulseWidth(uint16_t angle)
    {
      int pulse_wide, analog_value;
      pulse_wide = map(angle, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
      analog_value = int(float(pulse_wide) / 1000000 * servofreq * 65536);
      return analog_value;
    }
  
  void Update(uint16_t valueLacet) {

    ledcWrite(lacetChannel, pulseWidth(valueLacet)); 

    }
};


class Roulis
{
    int pin_R_G;
    int pin_R_D;
    int MIN_PULSE_WIDTH;
    int MAX_PULSE_WIDTH;
    
  public:
Roulis(int pin1, int pin2 ){ 
  pin_R_G = pin1;
  pin_R_D = pin2;
  MIN_PULSE_WIDTH = 800; //544
  MAX_PULSE_WIDTH = 2200; //2400
  ledcSetup(roulisChannelG,servofreq,servoResolution);
  ledcSetup(roulisChannelD,servofreq,servoResolution);
  ledcAttachPin(pin_R_G,roulisChannelG);
  ledcAttachPin(pin_R_D,roulisChannelD);
    }


   int pulseWidth(uint16_t angle)
    {
      int pulse_wide, analog_value;
      pulse_wide = map(angle, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
      analog_value = int(float(pulse_wide) / 1000000 * servofreq * 65536);
      return analog_value;
    }
  
  void Update(uint16_t valueRoulis) {

    ledcWrite(roulisChannelG, pulseWidth(valueRoulis)); 
    ledcWrite(roulisChannelD, pulseWidth(valueRoulis)); 

    }
};


class Tangage
{
    int pin_T_G;
    int pin_T_D;
    int MIN_PULSE_WIDTH;
    int MAX_PULSE_WIDTH;
    
  public:
Tangage(int pin1, int pin2 ){ 
  pin_T_G = pin1;
  pin_T_D = pin2;
  MIN_PULSE_WIDTH = 900;
  MAX_PULSE_WIDTH = 2100;
  ledcSetup(tangageChannelG,servofreq,servoResolution);
  ledcSetup(tangageChannelD,servofreq,servoResolution);
  ledcAttachPin(pin_T_G,tangageChannelG);
  ledcAttachPin(pin_T_D,tangageChannelD);
    }


   int pulseWidthG(uint16_t angle)
    {
      int pulse_wide, analog_value;
      pulse_wide = map(angle, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
      analog_value = int(float(pulse_wide) / 1000000 * servofreq * 65536);
      return analog_value;
    }

       int pulseWidthD(uint16_t angle)
    {
      int pulse_wide, analog_value, revert;
      revert = 1023 - angle;
      pulse_wide = map(revert, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
      analog_value = int(float(pulse_wide) / 1000000 * servofreq * 65536);
      return analog_value;
    }
  
  void Update(uint16_t valueTangage) {

    ledcWrite(tangageChannelG, pulseWidthG(valueTangage)); 
    ledcWrite(tangageChannelD, pulseWidthD(valueTangage)); 

    }
};





//class Moteur
//{
//    int pin_M;
//    int MIN_PULSE_WIDTH;
//    int MAX_PULSE_WIDTH;
//    
//  public:
//Moteur(int pin){ 
//  pin_M = pin;
//  MIN_PULSE_WIDTH = 1000;
//  MAX_PULSE_WIDTH = 2000;
//  ledcSetup(moteurChannel,servofreq,servoResolution);
//  ledcAttachPin(pin_M,moteurChannel);
//    }
//
//
//   int pulseWidth(uint16_t angle)
//    {
//      int pulse_wide, analog_value;
//      pulse_wide = map(angle, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
//      analog_value = int(float(pulse_wide) / 1000000 * servofreq * 65536);
//      return analog_value;
//    }
//  
//  void Update(uint16_t valueMoteur) {
//    ledcWrite(moteurChannel, pulseWidth(valueMoteur)); 
//    }
//};



class Moteur
{
    int pin_M;
    
  public:
  Moteur(int pin){ 
  pin_M = pin;
  ledcSetup(moteurChannel,servofreq,servoResolution);
  ledcAttachPin(pin_M,moteurChannel);
    }

   int pulseWidthFactored(uint16_t angle,uint16_t factor)
    {
      int pulse_wide, analog_value, PULSE_MIDDLE;

      PULSE_MIDDLE = map(angle, 0, 1023, 1000, 2000);

      if (angle < 10){
        pulse_wide = 1000;
        }

       if (angle > 1015){
        pulse_wide = 2000;
       }

      if (angle >= 10 && angle <= 1015 && factor < 520){
        pulse_wide = map(factor, 0, 526, 1000, PULSE_MIDDLE);
        }
        
      if (angle >= 10 && angle <= 1015 && factor >= 520 && factor <= 532 ){
        pulse_wide = map(angle, 0, 1023, 1000, 2000);
        }
        
      if ( angle >= 10 && angle <= 1015 && factor > 532){
        pulse_wide = map(factor, 526, 1023, PULSE_MIDDLE, 2000);
        }   
      
      analog_value = int(float(pulse_wide) / 1000000 * servofreq * 65536);
      return analog_value;
    }
  
  void Update(uint16_t valueMoteur,uint16_t valueFacteur) {
      ledcWrite(moteurChannel, pulseWidthFactored(valueMoteur,valueFacteur));
    }
};


class Listener
{
  public:
  Listener(){ 
   
    }
  
  void Update() {
          
  if (radio.available()) {

      Serial.println(radio.getDynamicPayloadSize());
      radio.read(&dataRecved,sizeof(dataRecved));

      Serial.print(F("Received: "));
      Serial.println();
      Serial.print(dataRecved.manetteG_X);
      Serial.print(";");
      Serial.print(dataRecved.manetteG_Y);
      Serial.print(";");
      Serial.print(dataRecved.manetteD_X);
      Serial.print(";");
      Serial.print(dataRecved.manetteD_Y);
      Serial.print(";");
      Serial.print(dataRecved.motor);
      Serial.println(); 

          sensors_event_t a, g, temp;
          mpu.getEvent(&a, &g, &temp);

//          dataToSend[0] = int16_t((atan(-1 * a.acceleration.x / sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / PI) + 0);
//          dataToSend[1] = int16_t((atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * 180 / PI) - 0);
//          dataToSend[2] = int16_t(a.acceleration.z);
//          dataToSend[3] = int16_t(temp.temperature);
//          dataToSend[4] = 0;


            dataToSend.tangage = int16_t((atan(-1 * a.acceleration.x / sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / PI) + 0);
            dataToSend.roulis = int16_t((atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * 180 / PI) - 0);
            dataToSend.gravite = int16_t(a.acceleration.z);
            dataToSend.temperature = int16_t(temp.temperature);
            dataToSend.valeur = 0;
  
          
            Serial.print(F("Valeurs Ack payload:"));
            Serial.println();
            Serial.print(dataToSend.tangage);
            Serial.print(";");
            Serial.print(dataToSend.roulis);
            Serial.print(";");
            Serial.print(dataToSend.gravite);
            Serial.print(";");
            Serial.print(dataToSend.temperature);
            Serial.print(";");
            Serial.print(dataToSend.valeur);
            Serial.println();

          radio.writeAckPayload(1, &dataToSend, sizeof(dataToSend));     
    }
    }
};




Moteur moteur(13);
Roulis roulis(32,33);
Tangage tangage(26 ,27);
Lacet lacet(25);
Listener listener;

void setup()
{
  Serial.begin(115200);

  dataToSend.tangage = 0;
  dataToSend.roulis = 0;
  dataToSend.gravite = 0;
  dataToSend.temperature = 0;
  dataToSend.valeur = 0;
  
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.openReadingPipe(1, address1);
  radio.enableAckPayload();  
  radio.startListening();
  radio.writeAckPayload(1, &dataToSend, sizeof(dataToSend)); // pre-load data

Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
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

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
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

}

void loop()
{
  listener.Update();
  //moteur.Update(dataRecved.motor);
  moteur.Update(dataRecved.motor,dataRecved.manetteG_Y);
  roulis.Update(dataRecved.manetteD_X);
  tangage.Update(dataRecved.manetteD_Y);
  lacet.Update(dataRecved.manetteG_X);
}
