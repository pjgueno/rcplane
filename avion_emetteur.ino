//TELECOMMANDE
//TELECOMMANDE

#include <TFT_ST7735.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

TFT_ST7735 tft = TFT_ST7735();
RF24 radio(7, 8); // CE, CSN 

const byte address1[6] = "00001"; // telecommande => avion

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 50;

struct DataControl
{
  uint16_t manetteG_X;
  uint16_t manetteG_Y;
  uint16_t manetteD_X;
  uint16_t manetteD_Y;
  uint16_t motor;
};
DataControl dataToSend;


struct DataNavigation
{
  int16_t tangage;
  int16_t roulis;
  int16_t gravite;
  int16_t temperature;
  int16_t valeur;
};
DataNavigation dataRecved;

//int16_t dataAck[5]={0,0,0,0,0};


#define REDRAW_DELAY 16 // minimum delay in milliseconds between display updates
#define HOR 102    // Horizon circle outside radius (205 is corner to corner
#define BROWN      0x5140 //0x5960
#define SKY_BLUE   0x02B5 //0x0318 //0x039B //0x34BF
#define DARK_RED   0x8000
#define DARK_GREY  0x39C7
#define XC 64 // x coord of centre of horizon
#define YC 80 // y coord of centre of horizon
#define ANGLE_INC 1 // Angle increment for arc segments, 1 will give finer resolution, 2 or more gives faster rotation
#define DEG2RAD 0.0174532925

int roulis = 180; // These must be initialed to 180 so updateHorizon(0); in setup() draws
int last_roulis = 0; // the whole horizon graphic
int last_tangage = 0;
int roulis_delta = 90;  // This is used to set arc drawing direction, must be set to 90 here

// Variables for test only
int test_angle = 0;
int delta = ANGLE_INC;

unsigned long redrawTime = 0;



class ManetteG_X
{
  int pinG_X;
   
  public:
  ManetteG_X(int pin)
  {
  pinG_X = pin;
  }
 
  void Update()
  {

    dataToSend.manetteG_X= analogRead(pinG_X);

     
  }
};


class ManetteG_Y
{
  int pinG_Y;
   
  public:
  ManetteG_Y(int pin)
  {
  pinG_Y = pin;
  }
 
  void Update()
  {
    dataToSend.manetteG_Y= analogRead(pinG_Y);    
  }
};

class ManetteD_X
{
  int pinD_X;
 
  public:
  ManetteD_X(int pin)
  {
  pinD_X = pin;
  }
 
  void Update()
  {
    dataToSend.manetteD_X= analogRead(pinD_X);    
  }
};


class ManetteD_Y
{
  int pinD_Y;
 
  public:
  ManetteD_Y(int pin)
  {
  pinD_Y = pin;
  }
 
  void Update()
  {
    dataToSend.manetteD_Y= analogRead(pinD_Y);    
  }
};


class Motor
{
  int pinM;
 
  public:
  Motor(int pin){
    pinM = pin;
    }
  
  void Update() {
    dataToSend.motor= analogRead(pinM);
    }
};


class Message
{
  
  public:
  Message(){ 
    }
  
  void Update() {

      Serial.print(F("Sending: "));
      Serial.print(dataToSend.manetteG_X);
      Serial.print(";");
      Serial.print(dataToSend.manetteG_Y);
      Serial.print(";");
      Serial.print(dataToSend.manetteD_X);
      Serial.print(";");
      Serial.print(dataToSend.manetteD_Y);
      Serial.print(";");
      Serial.print(dataToSend.motor);
      Serial.println();
    
    if (radio.write(&dataToSend, sizeof(dataToSend))){
          Serial.println(F("Success!"));
          
          if ( radio.isAckPayloadAvailable()) {
            
               radio.read(&dataRecved, sizeof(dataRecved));
//
//                dataRecved.tangage = dataAck[0];
//                dataRecved.roulis = dataAck[1];
//                dataRecved.gravite = dataAck[2];
//                dataRecved.temperature = dataAck[3];
//                dataRecved.valeur = dataAck[4];
               
               Serial.print(F("Received: "));
                Serial.println();
                Serial.print(dataRecved.tangage);
                Serial.print(";");
                Serial.print(dataRecved.roulis);
                Serial.print(";");
                Serial.print(dataRecved.gravite);
                Serial.print(";");
                Serial.print(dataRecved.temperature);
                Serial.print(";");
                Serial.print(dataRecved.valeur);
                Serial.println();
           
              }
              else {
                  Serial.println("Acknowledge but no data");
              }
         }else{
           Serial.println(F("Failed!"));
           }
           
     }
};

class Screen
{

  public:
  Screen()
  {

  }

  void Update(int16_t roulis, int16_t tangage){
   if (millis() > redrawTime) {
    redrawTime = millis() + REDRAW_DELAY;
    updateHorizon(roulis,tangage);
  }
   }

void updateHorizon(int16_t roulis, int16_t tangage)
{
  bool draw = 1;
  int delta_tangage = 0;
  int tangage_error = 0;
  int delta_roulis  = 0;
  while ((last_tangage != tangage) || (last_roulis != roulis))
  {
    delta_tangage = 0;
    delta_roulis  = 0;

    if (last_tangage < tangage) {
      delta_tangage = 1;
      tangage_error = tangage - last_tangage;
    }
    
    if (last_tangage > tangage) {
      delta_tangage = -1;
      tangage_error = last_tangage - tangage;
    }
    
    if (last_roulis < roulis) delta_roulis  = 1;
    if (last_roulis > roulis) delta_roulis  = -1;
    
    if (delta_roulis == 0) {
      if (tangage_error > 1) delta_tangage *= 2;
    }
    
    drawHorizon(last_roulis + delta_roulis, last_tangage + delta_tangage);
    drawInfo();
  }
}

void drawHorizon(int roulis, int tangage)
{
  // Fudge factor adjustment for this sketch (so horizon is horizontal when start angle is 0)
  // This is needed as we draw extra pixels to avoid leaving plotting artifacts

  // Calculate coordinates for line start
  
  
  float sx = cos(-roulis * DEG2RAD);
  float sy = sin(-roulis * DEG2RAD);

  int16_t x0 = sx * HOR;
  int16_t y0 = sy * HOR;

  if ((roulis != last_roulis) && ((abs(roulis) > 35)  || (tangage != last_tangage)))
  {
    tft.drawLine(XC - x0, YC - y0 - 3 + tangage, XC + x0, YC + y0 - 3 + tangage, SKY_BLUE);
    tft.drawLine(XC - x0, YC - y0 + 3 + tangage, XC + x0, YC + y0 + 3 + tangage, BROWN);
    tft.drawLine(XC - x0, YC - y0 - 4 + tangage, XC + x0, YC + y0 - 4 + tangage, SKY_BLUE);
    tft.drawLine(XC - x0, YC - y0 + 4 + tangage, XC + x0, YC + y0 + 4 + tangage, BROWN);
  }

  tft.drawLine(XC - x0, YC - y0 - 2 + tangage, XC + x0, YC + y0 - 2 + tangage, SKY_BLUE);
  tft.drawLine(XC - x0, YC - y0 + 2 + tangage, XC + x0, YC + y0 + 2 + tangage, BROWN);

  tft.drawLine(XC - x0, YC - y0 - 1 + tangage, XC + x0, YC + y0 - 1 + tangage, SKY_BLUE);
  tft.drawLine(XC - x0, YC - y0 + 1 + tangage, XC + x0, YC + y0 + 1 + tangage, BROWN);

  tft.drawLine(XC - x0, YC - y0 + tangage,   XC + x0, YC + y0 + tangage,   TFT_WHITE);

  last_roulis = roulis;
  last_tangage = tangage;

}


void drawInfo(void)
{
  // Update things near middle of screen first (most likely to get obscured)

  // Level wings graphic
  tft.fillRect(64 - 1, 80 - 1, 3, 3, TFT_RED);
  tft.drawFastHLine(64 - 30,   80, 24, TFT_RED);
  tft.drawFastHLine(64 + 30 - 24, 80, 24, TFT_RED);
  tft.drawFastVLine(64 - 30 + 24, 80, 3, TFT_RED);
  tft.drawFastVLine(64 + 30 - 24, 80, 3, TFT_RED);

  tft.drawFastHLine(64 - 12,   80 - 40, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 - 30, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 - 20, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 - 10, 12, TFT_WHITE);

  tft.drawFastHLine(64 -  6,   80 + 10, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 + 20, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 + 30, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 + 40, 24, TFT_WHITE);


  tft.setTextColor(TFT_WHITE);
  tft.setCursor(64 - 12 - 13, 80 - 20 - 3);
  tft.print("10");
  tft.setCursor(64 + 12 + 1, 80 - 20 - 3);
  tft.print("10");
  tft.setCursor(64 - 12 - 13, 80 + 20 - 3);
  tft.print("10");
  tft.setCursor(64 + 12 + 1, 80 + 20 - 3);
  tft.print("10");

  tft.setCursor(64 - 12 - 13, 80 - 40 - 3);
  tft.print("20");
  tft.setCursor(64 + 12 + 1, 80 - 40 - 3);
  tft.print("20");
  tft.setCursor(64 - 12 - 13, 80 + 40 - 3);
  tft.print("20");
  tft.setCursor(64 + 12 + 1, 80 + 40 - 3);
  tft.print("20");

  // Display justified angle value near bottom of screen
  tft.setTextColor(TFT_YELLOW, BROWN); // Text with background
  tft.setTextDatum(MC_DATUM);            // Centre middle justified
  tft.setTextPadding(24);                // Padding width to wipe previous number
  tft.drawNumber(dataRecved.gravite, 64, 142, 1);
  tft.drawNumber(dataRecved.temperature, 64, 151, 1);

}
  
};


ManetteG_X manetteG_X(A1);
ManetteG_Y manetteG_Y(A0);
ManetteD_X manetteD_X(A3);
ManetteD_Y manetteD_Y(A2);
Motor motor(A4);
Message message;
Screen screen;

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Telecommande"));
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.enableAckPayload();
  radio.setRetries(5,5); // delay, count
  radio.openWritingPipe(address1);
  prevMillis = 0;

  tft.begin();
  tft.setRotation(0);
  tft.fillRect(0,  0, 128, 80, SKY_BLUE);
  tft.fillRect(0, 80, 128, 80, BROWN);
  drawHorizon(0, 0);
  drawInfo();


  // Test roulis and tangage
  //testRoulis();
  //testTangage();


}

void loop()
{
manetteG_X.Update();
manetteG_Y.Update();
manetteD_X.Update();
manetteD_Y.Update(); 
motor.Update();
currentMillis = millis();
if (currentMillis - prevMillis >= txIntervalMillis) {
    message.Update();
    prevMillis = millis();
}
screen.Update(dataRecved.roulis,dataRecved.tangage);
}


// #########################################################################
// Function to generate roulis angles for testing only
// #########################################################################

int angleGenerator(int maxAngle)
{

  // Synthesize a smooth +/- 50 degree roulis value for testing
  delta++; if (delta >= 360) test_angle = 0;
  test_angle = (maxAngle + 1) * sin((delta) * DEG2RAD);

  // Clip value so we hold angle near peak
  if (test_angle >  maxAngle) test_angle =  maxAngle;
  if (test_angle < -maxAngle) test_angle = -maxAngle;

  return test_angle;
}

void testRoulis(void)
{
  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("Roulis test", 64, 10, 1);

  for (int a = 0; a < 360; a++) {
    //delay(REDRAW_DELAY / 2);
    updateHorizon(angleGenerator(50), 0);
  }
  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("         ", 64, 10, 1);
}

void testTangage(void)
{

  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("Tangage test", 64, 10, 1);

  for (int p = 0; p > -50; p--) {
    delay(REDRAW_DELAY / 2);
    updateHorizon(0, p);
  }

  for (int p = -50; p < 50; p++) {
    delay(REDRAW_DELAY / 2);
    updateHorizon(0, p);
  }


  for (int p = 50; p > 0; p--) {
    delay(REDRAW_DELAY / 2);
    updateHorizon(0, p);
  }

  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("          ", 64, 10, 1);
}

void drawHorizon(int roulis, int tangage)
{
  // Fudge factor adjustment for this sketch (so horizon is horizontal when start angle is 0)
  // This is needed as we draw extra pixels to avoid leaving plotting artifacts

  // Calculate coordinates for line start
  float sx = cos(roulis * DEG2RAD);
  float sy = sin(roulis * DEG2RAD);

  int16_t x0 = sx * HOR;
  int16_t y0 = sy * HOR;

  if ((roulis != last_roulis) && ((abs(roulis) > 35)  || (tangage != last_tangage)))
  {
    tft.drawLine(XC - x0, YC - y0 - 3 + tangage, XC + x0, YC + y0 - 3 + tangage, SKY_BLUE);
    tft.drawLine(XC - x0, YC - y0 + 3 + tangage, XC + x0, YC + y0 + 3 + tangage, BROWN);
    tft.drawLine(XC - x0, YC - y0 - 4 + tangage, XC + x0, YC + y0 - 4 + tangage, SKY_BLUE);
    tft.drawLine(XC - x0, YC - y0 + 4 + tangage, XC + x0, YC + y0 + 4 + tangage, BROWN);
  }

  tft.drawLine(XC - x0, YC - y0 - 2 + tangage, XC + x0, YC + y0 - 2 + tangage, SKY_BLUE);
  tft.drawLine(XC - x0, YC - y0 + 2 + tangage, XC + x0, YC + y0 + 2 + tangage, BROWN);

  tft.drawLine(XC - x0, YC - y0 - 1 + tangage, XC + x0, YC + y0 - 1 + tangage, SKY_BLUE);
  tft.drawLine(XC - x0, YC - y0 + 1 + tangage, XC + x0, YC + y0 + 1 + tangage, BROWN);

  tft.drawLine(XC - x0, YC - y0 + tangage,   XC + x0, YC + y0 + tangage,   TFT_WHITE);

  last_roulis = roulis;
  last_tangage = tangage;

}

void drawInfo(void)
{
  // Update things near middle of screen first (most likely to get obscured)

  // Level wings graphic
  tft.fillRect(64 - 1, 80 - 1, 3, 3, TFT_RED);
  tft.drawFastHLine(64 - 30,   80, 24, TFT_RED);
  tft.drawFastHLine(64 + 30 - 24, 80, 24, TFT_RED);
  tft.drawFastVLine(64 - 30 + 24, 80, 3, TFT_RED);
  tft.drawFastVLine(64 + 30 - 24, 80, 3, TFT_RED);

  tft.drawFastHLine(64 - 12,   80 - 40, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 - 30, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 - 20, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 - 10, 12, TFT_WHITE);

  tft.drawFastHLine(64 -  6,   80 + 10, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 + 20, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 + 30, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 + 40, 24, TFT_WHITE);


  tft.setTextColor(TFT_WHITE);
  tft.setCursor(64 - 12 - 13, 80 - 20 - 3);
  tft.print("10");
  tft.setCursor(64 + 12 + 1, 80 - 20 - 3);
  tft.print("10");
  tft.setCursor(64 - 12 - 13, 80 + 20 - 3);
  tft.print("10");
  tft.setCursor(64 + 12 + 1, 80 + 20 - 3);
  tft.print("10");

  tft.setCursor(64 - 12 - 13, 80 - 40 - 3);
  tft.print("20");
  tft.setCursor(64 + 12 + 1, 80 - 40 - 3);
  tft.print("20");
  tft.setCursor(64 - 12 - 13, 80 + 40 - 3);
  tft.print("20");
  tft.setCursor(64 + 12 + 1, 80 + 40 - 3);
  tft.print("20");

  // Display justified angle value near bottom of screen
  tft.setTextColor(TFT_YELLOW, BROWN); // Text with background
  tft.setTextDatum(MC_DATUM);            // Centre middle justified
  tft.setTextPadding(24);                // Padding width to wipe previous number
  tft.drawNumber(last_roulis, 64, 142, 1);

}

void updateHorizon(int roulis, int tangage)
{
  bool draw = 1;
  int delta_tangage = 0;
  int tangage_error = 0;
  int delta_roulis  = 0;
  while ((last_tangage != tangage) || (last_roulis != roulis))
  {
    delta_tangage = 0;
    delta_roulis  = 0;

    if (last_tangage < tangage) {
      delta_tangage = 1;
      tangage_error = tangage - last_tangage;
    }
    
    if (last_tangage > tangage) {
      delta_tangage = -1;
      tangage_error = last_tangage - tangage;
    }
    
    if (last_roulis < roulis) delta_roulis  = 1;
    if (last_roulis > roulis) delta_roulis  = -1;
    
    if (delta_roulis == 0) {
      if (tangage_error > 1) delta_tangage *= 2;
    }
    
    drawHorizon(last_roulis + delta_roulis, last_tangage + delta_tangage);
    drawInfo();
  }
}

