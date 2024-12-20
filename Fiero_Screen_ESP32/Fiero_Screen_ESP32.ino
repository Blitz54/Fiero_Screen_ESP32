//  3.5 inch 9488 TFT for Pontiac Fiero to replace
//  the fuel gauge and the coolant gauge, as well as
//  add a washer fluid level, and a gear selection.
//  Node32S
//ILI9488_DRIVER needs to be set in User_Setup.h
//#include <User_Setups/Setup21_ILI9488.h> selected in User_Setup_Select.h
//#define SPI_FREQUENCY  40000000 needs to be set in User_Setup.h
//  Eventually I should set unused pins to ground, like 34-39 and others. Would need to design new pcb again.
// TEMP READER AAAAAAH. Put a 3.3v wire from the main pin to the ds18b20 VSS pin. Maybe board copper was too narrow?
//  Maybe switch Fuel pin. 32 has no pull down, which would be useful. Should be pulled down so if the pump wire
//  is disconnected, it shows an empty tank, not a full one. Reverse gear pin 13 has a pull down, but maybe just put a resistor on 32 instead?
#include <SPI.h>
#include <TFT_eSPI.h>           // Hardware-specific library
#include <DallasTemperature.h>  //
#include <OneWire.h>
#include <Wire.h>
#include "esp_system.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "fierologo.h"
#include <TJpg_Decoder.h>
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_cali.h"
#include <ezButton.h>
#include "MultiMap.h"
#include "Arial_Black30.h"
#include "CadmanEdited105.h"

#define FONT_GEARS CadmanEdited105
#define FONT_SMALL Arial_Black30

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

#define BLACK 0x0000
#define BGCOLOR 0x1000
#define RED 0xF800
#define WHITE 0xFFFF
#define ORANGE 0xFC00

#define ONE_WIRE_BUS 14  //should be 14 but its not fucking working
#define SENSOR_RESOLUTION 9
#define SENSOR_INDEX 0
//USE THESE TO SHRINK TASK SIZE IF NEEDED, UNLIKELY BUT NICE TO LEARN
//#define INCLUDE_uxTaskGetStackHighWaterMark 1
//Serial.println(uxTaskGetStackHighWaterMark(NULL));
//WHEN RAN ON OTHER MCU WITH BREAKOUT BOARD, THIS WORKS Found 1-Wire device with address: 0x28, 0x5E, 0xE9, 0x76, 0xE0, 0x01, 0x3C, 0xB5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorDeviceAddress;

// 'FieroDrawing', 60x100px
const unsigned char FieroDrawing[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xf0, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
  0x00, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x08, 0x00, 0xff, 0xf0, 0x01, 0x00, 0x00,
  0x00, 0x08, 0xff, 0x00, 0x0f, 0xf1, 0x00, 0x00, 0x00, 0x13, 0x04, 0x00, 0x02, 0x0c, 0x80, 0x00,
  0x00, 0x14, 0xf4, 0x00, 0x02, 0xf2, 0x80, 0x00, 0x00, 0x15, 0x14, 0x00, 0x02, 0x8a, 0x80, 0x00,
  0x00, 0x15, 0x14, 0x00, 0x02, 0x8a, 0x80, 0x00, 0x00, 0x15, 0x14, 0x00, 0x02, 0x8a, 0x80, 0x00,
  0x00, 0x15, 0x14, 0x00, 0x02, 0x8a, 0x80, 0x00, 0x00, 0x15, 0xf4, 0x00, 0x02, 0xfa, 0x80, 0x00,
  0x00, 0x24, 0x04, 0x00, 0x02, 0x02, 0x40, 0x00, 0x00, 0x24, 0x04, 0x00, 0x02, 0x02, 0x40, 0x00,
  0x00, 0x24, 0x04, 0x00, 0x02, 0x02, 0x40, 0x00, 0x00, 0x24, 0x0c, 0x00, 0x03, 0x02, 0x40, 0x00,
  0x00, 0x24, 0x08, 0x00, 0x01, 0x02, 0x40, 0x00, 0x00, 0x24, 0x08, 0x00, 0x01, 0x02, 0x40, 0x00,
  0x00, 0x44, 0x08, 0x00, 0x01, 0x02, 0x20, 0x00, 0x00, 0x44, 0x08, 0x00, 0x01, 0x02, 0x20, 0x00,
  0x00, 0x44, 0x08, 0x00, 0x01, 0x02, 0x20, 0x00, 0x00, 0x48, 0x08, 0x00, 0x01, 0x01, 0x20, 0x00,
  0x00, 0x48, 0x08, 0x00, 0x01, 0x01, 0x20, 0x00, 0x00, 0x48, 0x08, 0x00, 0x01, 0x01, 0x20, 0x00,
  0x00, 0x48, 0x08, 0x00, 0x01, 0x01, 0x20, 0x00, 0x00, 0x48, 0x08, 0x00, 0x01, 0x01, 0x20, 0x00,
  0x00, 0x48, 0x08, 0x00, 0x01, 0x01, 0x20, 0x00, 0x00, 0x48, 0x08, 0x00, 0x01, 0x01, 0x20, 0x00,
  0x00, 0x48, 0x0f, 0xff, 0xff, 0x01, 0x20, 0x00, 0x00, 0x48, 0xf8, 0x00, 0x01, 0xf1, 0x20, 0x00,
  0x00, 0xc8, 0x80, 0x00, 0x00, 0x11, 0x30, 0x00, 0x00, 0x8b, 0x00, 0x00, 0x00, 0x0d, 0x10, 0x00,
  0x00, 0x8c, 0x00, 0x00, 0x00, 0x03, 0x10, 0x00, 0x00, 0x98, 0x00, 0x00, 0x00, 0x01, 0x90, 0x00,
  0x00, 0xb8, 0x00, 0x00, 0x00, 0x01, 0xd0, 0x00, 0x00, 0xec, 0x00, 0x00, 0x00, 0x03, 0x70, 0x00,
  0x00, 0x8c, 0x00, 0x00, 0x00, 0x03, 0x10, 0x00, 0x01, 0x8c, 0x00, 0x00, 0x00, 0x03, 0x18, 0x00,
  0x01, 0x1a, 0x00, 0x00, 0x00, 0x05, 0x88, 0x00, 0x01, 0xfa, 0x00, 0x00, 0x00, 0x05, 0xf8, 0x00,
  0x00, 0xaa, 0x00, 0x00, 0x00, 0x05, 0x50, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0x0a, 0x50, 0x00,
  0x00, 0xa5, 0x00, 0x00, 0x00, 0x0a, 0x50, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0x0a, 0x50, 0x00,
  0x00, 0xa2, 0x80, 0x00, 0x00, 0x14, 0x50, 0x00, 0x00, 0xa2, 0x87, 0xff, 0xfe, 0x14, 0x50, 0x00,
  0x00, 0xa1, 0x78, 0x00, 0x01, 0xe8, 0x50, 0x00, 0x00, 0xa1, 0x00, 0x00, 0x00, 0x08, 0x50, 0x00,
  0x00, 0xa1, 0x00, 0x00, 0x00, 0x08, 0x50, 0x00, 0x00, 0xa1, 0x3f, 0xff, 0xff, 0xc8, 0x50, 0x00,
  0x00, 0xa1, 0x40, 0x00, 0x00, 0x28, 0x50, 0x00, 0x00, 0xa1, 0x40, 0x00, 0x00, 0x28, 0x50, 0x00,
  0x00, 0xa1, 0x40, 0x00, 0x00, 0x28, 0x50, 0x00, 0x00, 0xa1, 0x40, 0x00, 0x00, 0x28, 0x50, 0x00,
  0x00, 0xa1, 0x40, 0x00, 0x00, 0x28, 0x50, 0x00, 0x00, 0xa1, 0x40, 0x00, 0x00, 0x28, 0x50, 0x00,
  0x00, 0xa1, 0x40, 0x00, 0x00, 0x28, 0x50, 0x00, 0x00, 0xa1, 0x40, 0x00, 0x00, 0x28, 0x50, 0x00,
  0x00, 0xa1, 0x40, 0x00, 0x00, 0x28, 0x50, 0x00, 0x00, 0xbf, 0x40, 0x00, 0x00, 0x2f, 0xd0, 0x00,
  0x00, 0xa1, 0x40, 0x00, 0x00, 0x28, 0x50, 0x00, 0x00, 0xa1, 0x3f, 0xff, 0xff, 0xc8, 0x50, 0x00,
  0x00, 0xa1, 0x00, 0x00, 0x00, 0x08, 0x50, 0x00, 0x00, 0xa1, 0x00, 0x00, 0x00, 0x08, 0x50, 0x00,
  0x00, 0xa1, 0x00, 0x00, 0x00, 0x08, 0x50, 0x00, 0x00, 0xa1, 0x00, 0x00, 0x00, 0x08, 0x50, 0x00,
  0x00, 0xa1, 0x3f, 0xff, 0xff, 0xc8, 0x50, 0x00, 0x00, 0xa2, 0x4c, 0x00, 0x03, 0x24, 0x50, 0x00,
  0x00, 0xa2, 0x8c, 0x00, 0x03, 0x14, 0x50, 0x00, 0x00, 0xa2, 0x8c, 0x00, 0x03, 0x14, 0x50, 0x00,
  0x00, 0xa2, 0x8c, 0x00, 0x03, 0x14, 0x50, 0x00, 0x00, 0xa2, 0x8c, 0x00, 0x03, 0x14, 0x50, 0x00,
  0x00, 0xa5, 0x0c, 0x00, 0x03, 0x0a, 0x50, 0x00, 0x00, 0xa5, 0x0c, 0x00, 0x03, 0x0a, 0x50, 0x00,
  0x00, 0xa5, 0x0c, 0x00, 0x03, 0x0a, 0x50, 0x00, 0x00, 0x95, 0x0c, 0x00, 0x03, 0x0a, 0x90, 0x00,
  0x00, 0x95, 0x0c, 0x00, 0x03, 0x0a, 0x90, 0x00, 0x00, 0x95, 0x0c, 0x00, 0x03, 0x0a, 0x90, 0x00,
  0x00, 0x8b, 0xfc, 0x00, 0x03, 0xfd, 0x10, 0x00, 0x00, 0x82, 0x0c, 0x00, 0x03, 0x04, 0x10, 0x00,
  0x00, 0x82, 0x0c, 0x00, 0x03, 0x04, 0x10, 0x00, 0x00, 0x82, 0x0c, 0x00, 0x03, 0x04, 0x10, 0x00,
  0x00, 0x82, 0x0c, 0x00, 0x03, 0x04, 0x10, 0x00, 0x00, 0x86, 0x0c, 0x00, 0x03, 0x06, 0x10, 0x00,
  0x00, 0x84, 0x0c, 0x00, 0x03, 0x02, 0x10, 0x00, 0x00, 0x84, 0x0c, 0x00, 0x03, 0x02, 0x10, 0x00,
  0x00, 0x84, 0x0c, 0x00, 0x03, 0x02, 0x10, 0x00, 0x00, 0x9f, 0xfe, 0x00, 0x07, 0xff, 0x90, 0x00,
  0x00, 0x90, 0x01, 0xff, 0xf8, 0x00, 0x90, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x00,
  0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x00,
  0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x00, 0x58, 0x00, 0x00, 0x00, 0x01, 0xa0, 0x00,
  0x00, 0x46, 0x00, 0x00, 0x00, 0x06, 0x20, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x04, 0x40, 0x00,
  0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'wiper', 40x40px
const unsigned char wiper[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1c, 0x80, 0x78, 0x00, 0x00, 0x7c, 0xc7, 0x7c, 0x00, 0x00, 0x70, 0xef, 0x0c, 0x00, 0x00, 0x00,
  0x7c, 0x00, 0x00, 0x01, 0x80, 0x3c, 0x03, 0x80, 0x01, 0x80, 0x18, 0x03, 0x80, 0x01, 0x80, 0x00,
  0x01, 0x80, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x01, 0xff, 0x80, 0x00, 0x00, 0x1f, 0xff, 0xf0,
  0x00, 0x00, 0x7f, 0xff, 0xfe, 0x00, 0x03, 0xf0, 0x18, 0x1f, 0x80, 0x07, 0xc0, 0x00, 0x03, 0xe0,
  0x1f, 0x00, 0x00, 0x00, 0xf0, 0x3c, 0x00, 0x18, 0x00, 0x3c, 0x70, 0x20, 0x18, 0x00, 0x1e, 0xe0,
  0x70, 0x18, 0x00, 0x0f, 0xe0, 0x30, 0x18, 0x00, 0x07, 0x70, 0x38, 0x18, 0x00, 0x0e, 0x38, 0x1c,
  0x00, 0x00, 0x1c, 0x1c, 0x0c, 0x18, 0x00, 0x38, 0x0e, 0x0e, 0x18, 0x00, 0x70, 0x07, 0x07, 0x18,
  0x00, 0xe0, 0x03, 0x87, 0x00, 0x01, 0xc0, 0x01, 0xc3, 0x80, 0x03, 0x80, 0x00, 0xe1, 0xfe, 0x07,
  0x00, 0x00, 0x73, 0xff, 0x8e, 0x00, 0x00, 0x3f, 0xe7, 0xfc, 0x00, 0x00, 0x1f, 0x60, 0xf8, 0x00,
  0x00, 0x0c, 0x78, 0x70, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00,
  0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'gaspump', 30x30px
const unsigned char gaspump[] PROGMEM = {
  0x1f, 0xff, 0xe0, 0x00, 0x3f, 0xff, 0xe0, 0x00, 0x3c, 0x00, 0xf6, 0x00, 0x38, 0x00, 0x7f, 0x00,
  0x38, 0x00, 0x7f, 0x80, 0x38, 0x00, 0x77, 0xc0, 0x38, 0x00, 0x73, 0xe0, 0x38, 0x00, 0x71, 0xf0,
  0x38, 0x00, 0x70, 0xf8, 0x38, 0x00, 0x70, 0xfc, 0x38, 0x00, 0x70, 0xfc, 0x3f, 0xff, 0xf0, 0xfc,
  0x3f, 0xff, 0xf0, 0xfc, 0x3f, 0xff, 0xf0, 0x7c, 0x3f, 0xff, 0xf8, 0x3c, 0x3f, 0xff, 0xfe, 0x1c,
  0x3f, 0xff, 0xff, 0x0c, 0x3f, 0xff, 0xff, 0x0c, 0x3f, 0xff, 0xf3, 0x8c, 0x3f, 0xff, 0xf3, 0x8c,
  0x3f, 0xff, 0xf3, 0x9c, 0x3f, 0xff, 0xf3, 0x9c, 0x3f, 0xff, 0xf3, 0x9c, 0x3f, 0xff, 0xf3, 0x9c,
  0x3f, 0xff, 0xf3, 0x9c, 0x3f, 0xff, 0xf3, 0xf8, 0x3f, 0xff, 0xf1, 0xf8, 0x3f, 0xff, 0xf0, 0xf0,
  0xff, 0xff, 0xfc, 0x00, 0xff, 0xff, 0xfc, 0x00
};
// 'coolanticon', 35x30px
const unsigned char coolanticon[] PROGMEM = {
  0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00,
  0x01, 0xf0, 0x00, 0x00, 0x00, 0x01, 0xff, 0xc0, 0x00, 0x00, 0x01, 0xff, 0xe0, 0x00, 0x00, 0x01,
  0xff, 0xe0, 0x00, 0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x01, 0xff,
  0xc0, 0x00, 0x00, 0x01, 0xff, 0xe0, 0x00, 0x00, 0x01, 0xff, 0xe0, 0x00, 0x00, 0x01, 0xf8, 0x00,
  0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x01, 0xff, 0xe0, 0x00,
  0x00, 0x01, 0xff, 0xe0, 0x00, 0x00, 0x01, 0xff, 0xc0, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00,
  0x03, 0xf8, 0x00, 0x00, 0x0c, 0x07, 0xfc, 0x06, 0x00, 0x3f, 0x17, 0xfd, 0x1f, 0x80, 0xff, 0xf7,
  0xfd, 0xff, 0xe0, 0xf3, 0xf3, 0xf9, 0xf9, 0xe0, 0xe1, 0xe1, 0xf0, 0xf0, 0xe0, 0x00, 0x00, 0xe0,
  0x00, 0x00, 0x3f, 0x1f, 0x1f, 0x1f, 0x80, 0x1f, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xff, 0xff,
  0x80, 0x01, 0xf1, 0xf1, 0xf0, 0x00
};
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
  // Stop further decoding as image is running off bottom of screen
  if (y >= tft.height()) return 0;

  // This function will clip the image block rendering automatically at the TFT boundaries
  tft.pushImage(x, y, w, h, bitmap);

  // Return 1 to decode next block
  return 1;
}

//int datascreen = 0; //for potentially a button with different screens for data
bool MainButton;  //main button
const int LEDPin = 22;
const int LEDFreq = 5000;
const int LEDRes = 8;
int LEDDuty = 0;  //0 - 255. 0 Is grounded, hence LED brightest because PNP transistor is on fully.

ezButton GearBut1(16);
ezButton GearBut2(17);
ezButton GearBut3(25);
ezButton GearBut4(26);
ezButton GearButRev(13);

int GEAR = 0;
unsigned long geartimer = millis();

int LastPrintWARN;
bool ColdWeatherWarnedAlready = false;

const int WasherLVL = 27;  //pin 27
bool washerlow = false;
int WasherArray[16] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };  //starts full
int WasherIndex = 0;
int WasherCount = 0;

int CarVoltagePin = 33;

float average;

const int FuelR1 = 200;
float FuelR2 = 0;
float FuelTotal;
float FuelAverage;
float FuelCurrent;
int FuelArray[16];  //averaging readings
int FuelBarSIZE;
int FuelIndex = 0;
const int FuelPin = 32;  //Fuel and coolant are on ADC1, so that I can use bluetooth as ADC2 can't be used with bluetooth on.
char fuelstring[5];
float FuelPercentage;

int CoolantR1 = 1000;
float CoolantTotal;
float CoolantAverage;
float CoolantCurrent;
int CoolantArray[16];  //averaging readings
int CoolantBarSIZE;
int CoolantIndex = 0;
const int CoolantPin = 35;  // MIGHT BE FRIED, OLD MCU NOT WORKING AND RED LIGHT
char coolantstring[7];

float FuelValuesIN[] = { 640, 752, 809, 891, 960, 1040, 1107, 1171, 1231, 1288, 1342 };  //lowest reading SHOULD be 628, but increased it a little so that the fuel shows empty near 2 ohms.
float FuelValuesOUT[] = { 0, 21, 42, 63, 84, 105, 126, 147, 168, 189, 210 };

//array for coolant readings. First one is analogReadMilliVolts, second one is Fahrenheit. 1k R1 and 47ohm R2 inline with sensor.
float CoolantValuesIN[] = { 253, 279, 318, 344, 381, 441, 452, 509, 595, 676, 742, 857, 892, 953, 1049, 1212, 1336, 1463, 1754, 1966, 2172, 2449, 2790, 2863, 2943, 2998, 3039, 3098, 3129 };
float CoolantValuesOUT[29] = { 290, 280, 270, 260, 250, 240, 230, 220, 210, 200, 190, 180, 175, 170, 160, 150, 140, 130, 110, 100, 90, 70, 40, 30, 20, 10, 0, -20, -40 };  //17 readings, from 100 to 260

float temperatureInCelsius = sensors.getTempCByIndex(SENSOR_INDEX);
float temperatureInFahrenheit = sensors.getTempFByIndex(SENSOR_INDEX);
char currenttemp[9];
float oldtemp = 0;
char tempstring[12];


void FillSensorBarsSmoothlyToReading(int y, int f, int c) {
  if (y <= f) {
    tft.fillRect(280, 420 - y, 25, y, ORANGE);  //fill bar to new fuel level mapped
  }
  if (y <= c) {
    tft.fillRect(15, 420 - y, 25, y, ORANGE);  //fill bar to new Coolant level mapped
  }
  Serial.println(y);
}

void showmsgXY(int x, int y, int w, const uint8_t *f, const char *msg) {
  tft.setTextPadding(w);
  tft.setTextColor(RED, BGCOLOR);
  tft.loadFont(f);  //NEED TO MAKE AA FONTS FOR TOP TEMP, NUMBERS ABOVE BARS+PSI, AND GEARS.
  tft.drawString(msg, x, y);
}
SemaphoreHandle_t LCDMutex;  //mutex for printing to screen. If two tasks print at same time screen gets corrupted

TaskHandle_t PrintCRNTGear;
TaskHandle_t TempSensor;
TaskHandle_t WasherTask;
TaskHandle_t Fuel;
TaskHandle_t Coolant;
TaskHandle_t WarningTask;
TaskHandle_t TPMS;

void setup(void) {
  TJpgDec.setCallback(tft_output);
  Serial.begin(115200);  // For debug
  tft.setSwapBytes(true);
  tft.init();
  tft.fillScreen(BGCOLOR);
  tft.setRotation(2);  //2 is 180, 0 is default
  TJpgDec.drawJpg(0, 0, fierologo, sizeof(fierologo));
  //delay(200);//led delayed so white flash doesnt happen on boot. Not needed if using PWM.
  ledcAttach(LEDPin, LEDFreq, LEDRes);
  ledcWrite(LEDPin, LEDDuty);
  sensors.begin();
  sensors.getAddress(sensorDeviceAddress, 0);
  sensors.setResolution(sensorDeviceAddress, SENSOR_RESOLUTION);
  pinMode(27, INPUT_PULLUP);     //Arduino Internal Resistor 10K Washer Level
  GearBut1.setDebounceTime(50);  // all EzPins are pulled up with internal resistor
  GearBut2.setDebounceTime(50);
  GearBut3.setDebounceTime(50);
  GearBut4.setDebounceTime(50);
  GearButRev.setDebounceTime(50);  // set debounce time to 50 milliseconds for gear buttons
  for (int i = 0; i < 16; i++)     //read fuel level on start
  {
    FuelArray[i] = (analogReadMilliVolts(FuelPin) * 0.977);  //calibrate millivolt reading by multiplying
    FuelTotal += (analogReadMilliVolts(FuelPin) * 0.977);
  }
  FuelAverage = FuelTotal / 16;
  FuelCurrent = FuelAverage;
  for (int i = 0; i < 16; i++)
  {
    CoolantArray[i] = (analogReadMilliVolts(CoolantPin) * 0.977);  //calibrate millivolt reading by multiplying
    CoolantTotal += (analogReadMilliVolts(CoolantPin) * 0.977);
  }
  CoolantAverage = CoolantTotal / 16;
  CoolantCurrent = CoolantAverage;
  vTaskDelay(2400);
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(RED, BGCOLOR, true);
  ledcDetach(LEDPin);  //disable LCD backlight temporarily to erase Fiero logo and print new stuff
  pinMode(LEDPin, INPUT);
  tft.fillScreen(BGCOLOR);
  tft.fillRect(270, 200, 45, 230, RED);                  //fuel ba
  tft.fillRect(5, 200, 45, 230, RED);                    // temp bar
  tft.fillRect(275, 205, 35, 220, BGCOLOR);              //fuel bar bg
  tft.fillRect(10, 205, 35, 220, BGCOLOR);               //temp bar bg
  tft.fillRect(270, 255, 8, 6, RED);                     //3/4 fuel bar
  tft.fillRect(270, 312, 8, 6, RED);                     //1/2 fuel bar
  tft.fillRect(270, 369, 8, 6, RED);                     //1/4 fuel bar
  tft.fillRect(42, 255, 8, 6, RED);                      //top temp bar
  tft.fillRect(0, 40, 320, 5, RED);                      //Horizontal bar 1
  tft.fillRect(0, 140, 320, 5, RED);                     //Horizontal bar 2
  tft.fillRect(0, 178, 320, 5, RED);                     //Horizontal bar 3
  tft.fillRect(157, 145, 6, 33, RED);                    //split down middle
  tft.drawBitmap(130, 265, FieroDrawing, 60, 100, RED);  //image for wheel PSI
  tft.fillRect(90, 282, 40, 3, RED);                     //psi FL
  tft.fillRect(190, 282, 40, 3, RED);                    //psi FR
  tft.fillRect(90, 347, 40, 3, RED);                     //psi RL
  tft.fillRect(190, 347, 40, 3, RED);                    //psi RR
  showmsgXY(110, 280, 20, FONT_SMALL, "30");             //FL
  showmsgXY(210, 280, 20, FONT_SMALL, "30");             //FR
  showmsgXY(110, 345, 20, FONT_SMALL, "30");             //RL
  showmsgXY(210, 345, 20, FONT_SMALL, "30");             //RR
  showmsgXY(158, 400, 20, FONT_SMALL, "PSI");
  CoolantBarSIZE = multiMap<float>(CoolantCurrent, CoolantValuesIN, CoolantValuesOUT, 29);  //constrains value automatically
  dtostrf(CoolantBarSIZE, 3, 0, coolantstring);
  strcat(coolantstring, "°F");  //this and line above NEED to be before we rewrite CoolantBARSIZE again
  if (CoolantBarSIZE < 100)
  {
    strcpy(coolantstring, "---°F");
  }
  showmsgXY(78, 177, 140, FONT_SMALL, coolantstring);                           //CoolantNumberAboveBar
  CoolantBarSIZE = map(constrain(CoolantBarSIZE, 100, 260), 100, 260, 0, 210);  //constrain Fahrenheit to 0-210 pixels for bar.
  FuelBarSIZE = multiMap<float>(FuelCurrent, FuelValuesIN, FuelValuesOUT, 11);  //constrains value automatically
  FuelPercentage = map(FuelBarSIZE, 0, 210, 0, 100);
  dtostrf(FuelPercentage, 3, 0, fuelstring);
  strcat(fuelstring, "%");
  showmsgXY(244, 177, 20, FONT_SMALL, fuelstring);  //FuelNumberAboveBar
  tft.drawBitmap(280, 440, gaspump, 30, 30, RED);
  tft.fillTriangle(260, 455, 270, 445, 270, 465, RED);  //triangle for fuel cap
  tft.drawBitmap(10, 440, coolanticon, 35, 30, RED);
  showmsgXY(155, 133, 45, FONT_GEARS, "N");  //GEAR
  sensors.requestTemperatures();
  float temperatureInCelsius = sensors.getTempCByIndex(SENSOR_INDEX);
  dtostrf(temperatureInCelsius, 4, 1, currenttemp);
  strcat(currenttemp, "°C");
  showmsgXY(245, 37, 25, FONT_SMALL, currenttemp);
  oldtemp = temperatureInCelsius;
  LCDMutex = xSemaphoreCreateMutex();
  ledcAttach(LEDPin, LEDFreq, LEDRes);
  ledcWrite(LEDPin, LEDDuty);     //turn backlight back on
  for (int i = 0; i <= 210; i++)  // smoothly fills coolant and fuel bar
  {
    tft.fillRect(280, 420 - i, 25, i, ORANGE);  //fill bar to new fuel level mapped
    tft.fillRect(15, 420 - i, 25, i, ORANGE);   //fill bar to new Coolant level mapped
    delay(1);
  }
  for (int i = 0; i <= 210; i++)  // smoothly empties coolant and fuel bar
  {
    tft.fillRect(280, 210, 25, i, BGCOLOR);  //empty bar to new level
    tft.fillRect(15, 210, 25, i, BGCOLOR);   //empty bar to new level
    delay(1);
  }
  int bf = 1;
  while (bf <= CoolantBarSIZE || bf <= FuelBarSIZE)
  {
    FillSensorBarsSmoothlyToReading(bf, FuelBarSIZE, CoolantBarSIZE);
    bf++;
    delay(1);
  }
  xTaskCreatePinnedToCore(
    PrintCRNTGearCode,
    "PrintCRNTGear",
    10000,
    NULL,
    1,
    &PrintCRNTGear,
    1);
  delay(10);
  xTaskCreate(
    TempSensorCode, /* Task function. */
    "TempSensor",   /* name of task. */
    10000,           /* Stack size of task --has about 2000 spare--*/
    NULL,           /* parameter of the task */
    1,              /* priority of the task */
    &TempSensor);
  delay(10);
  xTaskCreate(
    WasherTaskCode, /* Task function. */
    "WasherTask",   /* name of task. */
    4000,           /* Stack size of task --has about 3300 spare--*/
    NULL,           /* parameter of the task */
    1,              /* priority of the task */
    &WasherTask);
  delay(10);
  xTaskCreate(
    FuelCode, /* Task function. */
    "Fuel",   /* name of task. */
    10000,    /* Stack size of task */
    NULL,     /* parameter of the task */
    1,        /* priority of the task */
    &Fuel);   /* Task handle to keep track of created task */
  delay(10);
  xTaskCreate(
    CoolantCode, /* Task function. */
    "Coolant",   /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Coolant);   /* Task handle to keep track of created task */
  delay(10);
  xTaskCreate(
    TPMSCode, /* Task function. */
    "TPMS",   /* name of task. */
    10000,    /* Stack size of task */
    NULL,     /* parameter of the task */
    1,        /* priority of the task */
    &TPMS);   /* Task handle to keep track of created task */
  delay(10);
  vTaskDelay(50);
}

void PrintCRNTGearCode(void *pvParameters) {
  for (;;) {
    GearBut1.loop();  //ezbutton loops
    GearBut2.loop();
    GearBut3.loop();
    GearBut4.loop();
    GearButRev.loop();
    if ((GearBut1.getState() == 0) && (GEAR != 1))
    {
      delay(50);
      xSemaphoreTake(LCDMutex, portMAX_DELAY);
      showmsgXY(150, 133, 45, FONT_GEARS, "1");
      GEAR = 1;
      geartimer = millis();
      xSemaphoreGive(LCDMutex);  // release mutex
      Serial.println("1");
    }
    else if ((GearBut2.getState() == 0) && (GEAR != 2))
    {
      xSemaphoreTake(LCDMutex, portMAX_DELAY);
      showmsgXY(155, 133, 45, FONT_GEARS, "2");
      GEAR = 2;
      geartimer = millis();
      xSemaphoreGive(LCDMutex);  // release mutex
      Serial.println("2");
    }
    else if ((GearBut3.getState() == 0) && (GEAR != 3))
    {
      xSemaphoreTake(LCDMutex, portMAX_DELAY);
      showmsgXY(155, 133, 45, FONT_GEARS, "3");
      GEAR = 3;
      geartimer = millis();
      xSemaphoreGive(LCDMutex);  // release mutex
      Serial.println("3");
    }
    else if ((GearBut4.getState() == 0) && (GEAR != 4))
    {
      xSemaphoreTake(LCDMutex, portMAX_DELAY);
      showmsgXY(155, 133, 45, FONT_GEARS, "4");
      GEAR = 4;
      geartimer = millis();
      xSemaphoreGive(LCDMutex);  // release mutex
      Serial.println("4");
    }

    else if ((GearButRev.getState() == 0) && (GEAR != 5)) {
      xSemaphoreTake(LCDMutex, portMAX_DELAY);
      showmsgXY(155, 133, 45, FONT_GEARS, "R");
      GEAR = 5;
      geartimer = millis();
      xSemaphoreGive(LCDMutex);  // release mutex
      Serial.println("R");
    }

    else if ((GearBut1.getState() == 1) && (GearBut2.getState() == 1) && (GearBut3.getState() == 1) && (GearBut4.getState() == 1) && (GearButRev.getState() == 1) && (GEAR != 0))
    {
      if ((millis()) >= (geartimer + 1400))
      {
        xSemaphoreTake(LCDMutex, portMAX_DELAY);
        showmsgXY(155, 133, 45, FONT_GEARS, "N");
        GEAR = 0;
        xSemaphoreGive(LCDMutex);  // release mutex
        Serial.println("N");
      }
    }
    else
    {
      geartimer = millis();
    }

    //vTaskDelay(100);
  }
}

void FuelCode(void *pvParameters) {
  for (;;) {
    FuelTotal = FuelTotal - FuelArray[FuelIndex];
    FuelArray[FuelIndex] = (analogReadMilliVolts(FuelPin) * 0.977);  //1125 eFuse, multiplied to adjust voltage reading.
    FuelTotal = FuelTotal + FuelArray[FuelIndex];
    FuelIndex += 1;
    if (FuelIndex >= 16)
    {
      FuelIndex = 0;
    }
    FuelAverage = FuelTotal / 16;
    if (FuelAverage >= FuelCurrent + 6 || FuelAverage <= FuelCurrent - 6)
    {
      FuelCurrent = FuelAverage;
      FuelBarSIZE = multiMap<float>(FuelCurrent, FuelValuesIN, FuelValuesOUT, 11);  //constrains value automatically
      FuelPercentage = map(FuelBarSIZE, 0, 210, 0, 100);
      dtostrf(FuelPercentage, 3, 0, fuelstring);
      strcat(fuelstring, "%");
      xSemaphoreTake(LCDMutex, portMAX_DELAY);
      showmsgXY(244, 177, 140, FONT_SMALL, fuelstring);               //FuelNumberAboveBar
      tft.fillRect(280, 210, 25, 210 - FuelBarSIZE, BGCOLOR);         //empty bar to new level
      tft.fillRect(280, 420 - FuelBarSIZE, 25, FuelBarSIZE, ORANGE);  //fill bar to new fuel level mapped
      xSemaphoreGive(LCDMutex);
    }
    vTaskDelay(1000);
  }
}


//coolant reading might need to be mapped differently, as its not linear. Update range might need to be different
//than 10 as well, 10 is fine for upper ranges, but at low temp it would update constantly. Might be an issue on highway
//driving and stopping lots might make level update too often, add some dampening maybe? so it grows the bar a little slower when warming up.
//ALSO. Using 210 ohm and 20 ohm divider, brings voltage low but a little too low. Overheating temps too accurate compared to proper running temps,
//should bring voltage up to about 2v, right now overheating 35ohm (300c) is at 1.1 ish. Need 2 watt resistors to be safe
//60 reading is about 69 celcius right now,

//OKAY. 1000 on R1 resister, 47 on second one (inline with thermistor). Could do other values, but this feels okay.
//Fiero Gauge shows 100F lowest, 220f middle, and 260f highest (end of red overheating).

//lots of the above might be wrong now, as im putting 3.16v (on this specific board) to the sensor rather than 12v.
void CoolantCode(void *pvParameters) {
  for (;;) {
    CoolantTotal = CoolantTotal - CoolantArray[CoolantIndex];
    CoolantArray[CoolantIndex] = (analogReadMilliVolts(CoolantPin) * 0.977);
    CoolantTotal = CoolantTotal + CoolantArray[CoolantIndex];
    CoolantIndex += 1;
    if (CoolantIndex >= 16)
    {
      CoolantIndex = 0;
    }
    CoolantAverage = CoolantTotal / 16;
    if (CoolantAverage >= CoolantCurrent + 6 || CoolantAverage <= CoolantCurrent - 6)
    {
      CoolantCurrent = CoolantAverage;                                                          ///need to get array temp values, and interpolate between the Fahrenheit. THEN map that value to the FuelBarSIZE.
      CoolantBarSIZE = multiMap<float>(CoolantCurrent, CoolantValuesIN, CoolantValuesOUT, 29);  //constrains value automatically
      dtostrf(CoolantBarSIZE, 3, 0, coolantstring);
      strcat(coolantstring, "°F");  //this and line above NEED to be before we rewrite CoolantBARSIZE again
      if (CoolantBarSIZE < 100)
      {
        strcpy(coolantstring, "---°F");
      }
      CoolantBarSIZE = map(constrain(CoolantBarSIZE, 100, 260), 100, 260, 0, 210);  //constrain Fahrenheit to 0-210 pixels for bar.
      xSemaphoreTake(LCDMutex, portMAX_DELAY);
      showmsgXY(78, 177, 140, FONT_SMALL, coolantstring);                  //CoolantNumberAboveBar
      tft.fillRect(15, 210, 25, 210 - CoolantBarSIZE, BGCOLOR);            //empty bar to new level
      tft.fillRect(15, 420 - CoolantBarSIZE, 25, CoolantBarSIZE, ORANGE);  //fill bar to new Coolant level mapped
      xSemaphoreGive(LCDMutex);
    }
    vTaskDelay(1000);
  }
}

void WasherTaskCode(void *pvParameters) {  //NEED TO REDO
  for (;;) {
    if (WasherIndex == 15) {
      WasherArray[WasherIndex] = (digitalRead(WasherLVL));
      WasherIndex = 0;
      WasherCount += (digitalRead(WasherLVL));
      if ((WasherCount <= 5) && (washerlow == false))
      {
        WasherCount = 0;
        xSemaphoreTake(LCDMutex, portMAX_DELAY);
        tft.drawBitmap(5, 0, wiper, 40, 40, RED);  //image for washer
        washerlow = true;
        xSemaphoreGive(LCDMutex);
      }
      else if ((WasherCount > 5) && (washerlow == true))
      {
        WasherCount = 0;
        tft.fillRect(5, 5, 50, 50, BGCOLOR);
        washerlow = false;
      }
      else
      {
        //nothing
      }
    }
    else
    {
      WasherArray[WasherIndex] = (digitalRead(WasherLVL));
      WasherIndex += 1;
      WasherCount += (digitalRead(WasherLVL));
    }
    vTaskDelay(1000);
  }
}

void TPMSCode(void *pvParameters) {
  for (;;) {
    Serial.println("TPMS STUFF TO DO");
    vTaskDelay(2000);
  }
}

void TempSensorCode(void *pvParameters) {
  for (;;) {
    sensors.requestTemperatures();
    float temperatureInCelsius = sensors.getTempCByIndex(SENSOR_INDEX);
    Serial.println(temperatureInCelsius);
    vTaskDelay(500);
    // float temperatureInFahrenheit = sensors.getTempFByIndex(SENSOR_INDEX);
    if (oldtemp != temperatureInCelsius)
    {
      xSemaphoreTake(LCDMutex, portMAX_DELAY);
      dtostrf(temperatureInCelsius, 9, 1, currenttemp);
      showmsgXY(245, 37, 25, FONT_SMALL, currenttemp);
      xSemaphoreGive(LCDMutex);
      oldtemp = temperatureInCelsius;
    }
    vTaskDelay(2000);  // 2 second delay on checking outside temp. Preeeetty sure other tasks on core 2 still run?
  }
}

void loop() {vTaskDelete(NULL);}  // disable main loop