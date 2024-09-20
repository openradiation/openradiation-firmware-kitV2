/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/
#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "OpenRadiation_V2.h"
#include "Utils.h"
#include "OpenRadiation_Oled.h"

#define BLUETOOTH_NAME "OpengKIT72"

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define RX_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TX_CHARACTERISTIC_UUID "beb5483f-36e1-4688-b7f5-ea07361b26a8"

#define MANUFACTURER_DATA {0x00,0x00,0x30,0x30,0x30,0x30,0x31}

/** In packets types **/

#define IN_PACKET_STEALTH                   0x01
#define IN_PACKET_SILENT                    0x02
#define IN_PACKET_SET_TENSION               0x11
#define IN_PACKET_SEND_INFO                 0x12

#define VERSION_BUFFER_SIZE                    5
#define SENSOR_BUFFER_SIZE                    20
#define TUBE_BUFFER_SIZE                       8
#define VOLTAGE_BUFFER_SIZE                    5
#define DATA_BUFFER_SIZE                      19

uint8_t p_version_buffer[VERSION_BUFFER_SIZE] = {0x02, 0x03, '1', '.', '0'};
uint8_t p_sensor_buffer[SENSOR_BUFFER_SIZE] = {0x03, 0x12, 'G', 'e', 'i', 'g', 'e', 'r', '-', 'M', 'u', 'l', 'l', 'e', 'r', ' ','t' ,'u' ,'b' ,'e'};
uint8_t p_tube_buffer[TUBE_BUFFER_SIZE] = {0x10,0x06, 'J', '3', '0', '5', 0x62, 0x64};
uint8_t p_actual_tension[VOLTAGE_BUFFER_SIZE] = {0x12, 0x43, 0xBE, 0x00, 0x00};

//uint8_t p_tube_buffer_p_actual_tension[TUBE_BUFFER_SIZE+VOLTAGE_BUFFER_SIZE] = {0x10,0x06, 'J', '3', '0', '5', 0x62, 0x64, 0x12, 0x00, 0x00, 0xBE, 0x43};
uint8_t p_tube_buffer_p_actual_tension[TUBE_BUFFER_SIZE+VOLTAGE_BUFFER_SIZE] = {0x10,0x06, 'S', 'B', 'M', '-', '2', '0', 0x12, 0x00, 0x00, 0xBE, 0x43};


uint8_t p_data_buffer[DATA_BUFFER_SIZE] = {0x05, 0x00, 0x06, 0x00, 0x00, 0xD8, 0x41, 0xD1, 0x1D, 0x12, 0x00, 0x00, 0xBE, 0x43, 0x13, 0x00, 0x00, 0x00, 0x00};

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
BLECharacteristic * pRxCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

unsigned long pulse_millis = 0;
unsigned int nb_pulse = 0;

unsigned long previousMillis = 0;
const long interval = 1000;  
const long led_interval = 50;

unsigned int nb_pulse_to_send = 0;
int current_state = 0;
int old_state = 0;
unsigned int nb_measure = 0;

boolean b_new_pulse = false;
boolean b_data_to_send = false;
boolean b_debug_data_to_send = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

boolean stealthMode = false;
boolean silentMode = false;

class MyTxCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pTxCharacteristic) {
      std::string value = pTxCharacteristic->getValue();

      if (value.length() > 0) {
        byte datatype = value[0];

        switch (datatype) {
          
          case IN_PACKET_STEALTH:
            stealthMode = value[1];
            if (stealthMode) {
              disableFollowerLED();
            } else {
              enableFollowerLED();
            }
          break;
      
          case IN_PACKET_SILENT:
            silentMode = value[1];
            if (silentMode) {
              disableFollowerBuzzer();
            } else {
              enableFollowerBuzzer();
            }
            break;
          
          case IN_PACKET_SEND_INFO:
              pRxCharacteristic->setValue(p_version_buffer, VERSION_BUFFER_SIZE);
              pRxCharacteristic->notify();
              delay(100);
              pRxCharacteristic->setValue(p_sensor_buffer,SENSOR_BUFFER_SIZE);
              pRxCharacteristic->notify();
              delay(100);
              pRxCharacteristic->setValue(p_tube_buffer_p_actual_tension,TUBE_BUFFER_SIZE+VOLTAGE_BUFFER_SIZE);
              pRxCharacteristic->notify();
              delay(100);
          break;
          default: break;
        }
      }
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  delay(100);
  initialization();

  // Create the BLE Device
  BLEDevice::init(BLUETOOTH_NAME);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pRxCharacteristic = pService->createCharacteristic(
                                         RX_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                      );
  
  

  pTxCharacteristic = pService->createCharacteristic(
                                         TX_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  
  pTxCharacteristic->setCallbacks(new MyTxCallbacks());

  // Create a BLE Descriptor
  pRxCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
 
  oAdvertisementData.setManufacturerData(MANUFACTURER_DATA);

  //BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pRxAdvertising = BLEDevice::getAdvertising();
  
  pRxAdvertising->addServiceUUID(SERVICE_UUID);
  pRxAdvertising->setScanResponse(false);
  pRxAdvertising->setMinPreferred(0x00);  // functions that help with iPhone connections issue
  pRxAdvertising->setMinPreferred(0x12);
  pRxAdvertising->setScanResponseData(oAdvertisementData);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");

  pRxCharacteristic->setValue(p_version_buffer, VERSION_BUFFER_SIZE);
  pRxCharacteristic->notify();
  delay(10);
  pRxCharacteristic->setValue(p_sensor_buffer,SENSOR_BUFFER_SIZE);
  pRxCharacteristic->notify();
  delay(10);
  pRxCharacteristic->setValue(p_tube_buffer,TUBE_BUFFER_SIZE);
  pRxCharacteristic->notify();
  delay(10);

  //OledInitialization();
 }


void loop() {
  unsigned long currentMillis = millis();
 
  current_state = digitalRead(COUNT_INPUT);
  if ((old_state == 0) && (current_state == 1))
  {
    nb_pulse++;
    b_new_pulse = true;
  }
  old_state = current_state;

  //Check timer
  if (currentMillis - previousMillis >= interval)
  {
      previousMillis = currentMillis;
      
      nb_pulse_to_send = nb_pulse;
      nb_pulse = 0;

      temp_of_board = get_temperature(0, true);
      switch_sensor(0);

      battery_voltage = analogRead(BATTERY_CAN) / 512;

      ht_voltage = analogRead(HIGH_VOLTAGE_CAN) / 1024;

      b_data_to_send = true;
      b_debug_data_to_send = true;
            
  }


  if ((DEBUG_MODE) && (b_debug_data_to_send))
      {
          String data_to_send = "Nb pulse = ";
          data_to_send += String(nb_pulse_to_send, DEC);
          Serial.println(data_to_send);
          data_to_send = String("Temp = ");
          data_to_send += String(temp_of_board, DEC);
          Serial.println(data_to_send);
          data_to_send = String("Batterie Voltage = ");
          data_to_send += String(battery_voltage, DEC);
          Serial.println(data_to_send);
          data_to_send = String("Hight Voltage = ");
          data_to_send += String(ht_voltage, DEC);
          Serial.println(data_to_send);
          b_debug_data_to_send = false;
      }
  
  if (deviceConnected) {
    if (b_data_to_send)
      {
          b_data_to_send = false;
          p_data_buffer[1] = nb_pulse_to_send;
          ConvertFloatToBuffer(temp_of_board, p_data_buffer, 3); 
          pRxCharacteristic->setValue(p_data_buffer, DATA_BUFFER_SIZE);
          pRxCharacteristic->notify();
          delay(10);
      }
    }

      // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
        communication_disable();
    }
  
  // connecting
    if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
        Serial.println("Connected");
        oldDeviceConnected = deviceConnected;
        communication_enable();
    }

  //Flash led
  if (b_new_pulse)
  {
    buzzer_on();
    led_on();
    pulse_millis = currentMillis;
    b_new_pulse = false;
  }

  if ((currentMillis - pulse_millis) >= led_interval)
  {
    //digitalWrite(LED_PIN, HIGH);
    buzzer_off();
    led_off();
  }
 }
