#include <Adafruit_NeoPixel.h>
#include <OneWire.h>

#include "OpenRadiation_V2.h"
#include "Soft_PWM.h"

#define NUMPIXELS 2

#define LED_BLUETOOTH 0
#define LED_ETAT 1

#define TEMPO_1S 1000

#define NB_SENSOR_MAX 8
#define NB_DECIMAL 2

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEO_PIXEL, NEO_GRB + NEO_KHZ800);

OneWire  ds(TEMP_SENSOR_PIN);

boolean led_bluetooth_on = false;

boolean pulse_buzzer_enable = true;
boolean pulse_led_enable = true;

unsigned char nb_sensor = 0;
unsigned char old_sensor_id = NB_SENSOR_MAX;


temp_sensor sensor[NB_SENSOR_MAX];

int16_t analyze_raw_temperature(temp_sensor sensor_data){
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (sensor_data.data[1] << 8) | sensor_data.data[0];
  if (sensor_data.type) {
    raw = raw << 3; // 9 bit resolution default
    if (sensor_data.data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - sensor_data.data[6];
    }
  } else {
    byte cfg = (sensor_data.data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }

  return raw;
}


boolean check_adress_crc(byte addr[8]) {
  boolean crc_valid = false;
  
  if (OneWire::crc8(addr, 7) != addr[7]) {
      crc_valid = false;
  }
  else
  {
      crc_valid = true;
  }
  return crc_valid;
}

boolean check_data_crc(byte data[12]) {
  boolean crc_valid = false;
  
  if (OneWire::crc8(data, 8) != data[8]) {
      crc_valid = false;
  }
  else
  {
      crc_valid = true;
  }
  return crc_valid;
}

byte check_sensor_type(byte addr[8]) {
  byte type_s = 255;
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      type_s = 255;
  }
  return type_s;
}



unsigned int get_number_of_sensor() {
  byte addr[8];
  
  nb_sensor = 0;
  
  ds.reset_search();
  delay(250);
  while(ds.search(addr)) {
    nb_sensor++;
  }

  return(nb_sensor);
}

boolean get_address_of_sensor( unsigned char sensor_id, byte addr[8])
{
  boolean status;
  
  if (sensor_id < nb_sensor)
  {
    ds.reset_search();
    delay(250);
    for(unsigned char i = 0; i < sensor_id+1; i++)
    {
      //Serial.print("Get adress of sensor : ");
      //Serial.println(i, DEC);
      ds.search(addr);
      delay(250);
    }
    status = check_adress_crc(addr);
  }
  else
  {
    status = false;
  }

  return status;
}

void print_rom(byte addr[8]) {
  //Serial.print("ROM =");
  for(unsigned int i = 0; i < 8; i++) {
    Serial.write(' ');
    if (addr[i] < 16) Serial.write('0');
    Serial.print(addr[i], HEX);
  }
  Serial.write('\n');
}



float get_temperature_of_current_sensor(boolean b_celsius){
  int16_t raw;
  float temperature=1000.0;

  sensor[old_sensor_id].present = ds.reset();
  ds.select(sensor[old_sensor_id].adress);    
  ds.write(0xBE);         // Read Scratchpad

  for (unsigned char i = 0; i < 9; i++) {           // we need 9 bytes
    sensor[old_sensor_id].data[i] = ds.read();
  }

  if (check_data_crc(sensor[old_sensor_id].data)){
    raw = analyze_raw_temperature(sensor[old_sensor_id]);
  
    temperature = (float)raw / 16.0;
    if (!b_celsius) temperature = temperature * 1.8 + 32.0;
  }
  return temperature;
}

void switch_sensor(unsigned char sensor_id) {
  if (sensor_id < nb_sensor) { 
    ds.reset();
    ds.select(sensor[sensor_id].adress);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  }  
}

float get_temperature(unsigned char sensor_id, boolean b_celsius){
  
  if (nb_sensor == 0) {
    init_temp_sensor();
    return (25.0);
  }
  else {
    if (sensor_id != old_sensor_id)
    {
      switch_sensor(sensor_id);
      old_sensor_id = sensor_id;
    }
     
    return (get_temperature_of_current_sensor(b_celsius));
  } 
    
}

void init_temp_sensor(){
  unsigned char nb_temp_sensor_detected = 0;

  nb_temp_sensor_detected = get_number_of_sensor();
  Serial.print("Nb sensor detected : ");
  Serial.println(nb_temp_sensor_detected, DEC);

  for(unsigned char i = 0; i < nb_temp_sensor_detected; i++) {
    sensor[i].crc = get_address_of_sensor(i, sensor[i].adress);
    
    if (sensor[i].crc)
    {
      sensor[i].type = check_sensor_type(sensor[i].adress);
      
      Serial.print("Adress of sensor : ");
      Serial.print(i, DEC);
      Serial.write('\t');
      print_rom(sensor[i].adress);
    }
  }
}

void initialization()
{
  sound = true;

  //Confiuration of PIN
  pinMode(LED_PIN,OUTPUT);
  pinMode(NEO_PIXEL,OUTPUT);
  pinMode(ON_OFF,OUTPUT);
  pinMode(COUNT_INPUT, INPUT);

  //PWM configuration
  set_pwm_frequency(4000, BUZZER_PIN);
  
  //NeoPixel construction
  pixels.begin();

  //Delay to activate on NEO_PIXEL
  delay(100);

  led_off();
  digitalWrite(ON_OFF,LOW);
 
  pixels.setPixelColor(LED_BLUETOOTH, pixels.Color(255,0,0));
  pixels.setPixelColor(LED_ETAT, pixels.Color(0,255,0));
  pixels.show();
  delay(TEMPO_1S);
  init_temp_sensor();
  pixels.setPixelColor(LED_BLUETOOTH,pixels.Color(0,255,0));
  pixels.setPixelColor(LED_ETAT,pixels.Color(0,0,255));
  pixels.show();
  temp_of_board = get_temperature(0, true);
  delay(TEMPO_1S);
  pixels.setPixelColor(LED_BLUETOOTH,pixels.Color(0,0,255));
  pixels.setPixelColor(LED_ETAT,pixels.Color(255,0,0));
  pixels.show();
  delay(TEMPO_1S);
  
  battery_voltage = 0.0;
  ht_voltage = 0.0;

  pixels.setPixelColor(LED_BLUETOOTH,pixels.Color(0,0,0));
  pixels.setPixelColor(LED_ETAT,pixels.Color(255,0,0));
  pixels.show();
}

void communication_enable()
{
  if(led_bluetooth_on == false)
  { 
    pixels.setPixelColor(LED_BLUETOOTH,pixels.Color(0,0,255));
    pixels.show();
    led_bluetooth_on = true;
    digitalWrite(ON_OFF,HIGH);
  }
}

void communication_disable()
{
  if(led_bluetooth_on)
  {
    digitalWrite(ON_OFF,LOW);
    pixels.setPixelColor(LED_BLUETOOTH,pixels.Color(0,0,0));
    pixels.show();
    led_bluetooth_on = false;
  }
}

void enableFollowerLED(){
  pulse_led_enable = true;
}
  
void disableFollowerLED(){
  pulse_led_enable = false;
}

void enableFollowerBuzzer() {
  pulse_buzzer_enable = true;
}

void disableFollowerBuzzer() {
  pulse_buzzer_enable = false;
}

void led_on() {
  if (pulse_led_enable) digitalWrite(LED_PIN,HIGH);
}

void led_off() {
  digitalWrite(LED_PIN,LOW);
}

void buzzer_on() {
  if (pulse_buzzer_enable) set_pwm_half();
}

void buzzer_off(){
  set_pwm_off();
}
