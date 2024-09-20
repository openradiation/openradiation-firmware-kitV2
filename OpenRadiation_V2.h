#ifndef __OPEN_RADIATION_V2_H__
#define __OPEN_RADIATION_V2_H__

#include <Adafruit_NeoPixel.h>

#define BUZZER_PIN 13
#define LED_PIN 12
#define NEO_PIXEL 27
#define COUNT_INPUT 15
#define ON_OFF 33

#define TEMP_SENSOR_PIN 19

#define HIGH_VOLTAGE_CAN A6
#define BATTERY_CAN A13

#define DEBUG_MODE false

boolean sound;
float temp_of_board;
float battery_voltage;
float ht_voltage;

void initialization();

void communication_enable();
void communication_disable();

void enableFollowerLED();
void disableFollowerLED();

void enableFollowerBuzzer();
void disableFollowerBuzzer();

void led_on();
void led_off();

void radiation_detected(bool sound_on);

float get_temperature(unsigned char sensor_id, boolean b_celsius);

struct temp_sensor {
  byte adress[8];
  byte data[12];
  byte crc;
  byte present;
  byte type;
};

#endif
