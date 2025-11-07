#include <Wire.h>
#include "Adafruit_DRV2605.h"
#include <ArduinoBLE.h>

Adafruit_DRV2605 drv;


BLEService hapticService("19B10010-E8F2-537E-4F6C-D104768A1214");  // create service

// Distance to Hazard Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic distanceCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

// User speed characteristic
BLEByteCharacteristic speedCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);


uint8_t effect = 0;
uint8_t STARTUP_FLAG = 1;


void setup() {
  Serial.begin(9600);


  // Bluetooth Setup

  // Start Bluetooth Low Energy module
  BLE.begin();

  // Set the name and create the advertised BLE service
  BLE.setLocalName("Haptic");
  BLE.setAdvertisedService(hapticService);

  // Add the two characteristics to the service
  hapticService.addCharacteristic(distanceCharacteristic);
  hapticService.addCharacteristic(speedCharacteristic);
  // Add the service
  BLE.addService(hapticService);

  // Set the inital vlaues of the characterisitcs to 0
  distanceCharacteristic.setValue(0);
  speedCharacteristic.setValue(0);

  // Start advertising
  BLE.advertise();


  // Haptic Driver Setup
  drv.begin();
  // I2C trigger by sending 'go' command
  drv.setMode(DRV2605_MODE_INTTRIG);  // default, internal trigger when sending GO command

  drv.selectLibrary(1);

  // Physical OUTPUTS(established through setmode funciton):
  // - SDA ON DRV TO A4(SDA) PIN;
  // - SCL ON DRV TO A5(SCL) PIN;
}

void loop() {
  BLEDevice central = BLE.central();

  // 64 -> 37

  if (central.connected()) {  // need to check return on central.connected(), might need while loop

    // indicated successful connection
    if (STARTUP_FLAG){
      effect = 0;
      drv.setWaveform(0, 64);
      drv.setWaveform(1, 37);
      drv.setWaveform(2, 37);
      drv.setWaveform(3, 0);  // end of waveforms
      drv.go();
      STARTUP_FLAG = 0;

    }

    // if the remote device wrote to the characteristic, use the value to control the haptic motor:
    
    if (distanceCharacteristic.written()){
      uint8_t distance = distanceCharacteristic.value();
      if (distance == 0) {
        Serial.println("Haptic Off");
        effect = 0;
      } else if (distance == 1) {
        Serial.println("Buzz 40%");
        effect = 50;  // buzz 40%
      } else if (distance == 2) {
        Serial.println("Buzz 80%");
        effect = 48;  // buzz 80%
      } else if (distance == 3) {
        Serial.println("Buzz 100%");
        effect = 47;  // buzz 100%
      }
    } else {
      effect = 0;
    }
    



    // Run haptic motor
    if (effect) {
      drv.setWaveform(0, effect);
      drv.setWaveform(1, 0);  // end of waveforms
      drv.go();
    }

    // modulate frequency of stimulation based on user speed
    //  - Faster Frequency: faster approach speed to hazard
    //  - Slower Frequency: slower approach speed to hazard

    uint8_t speed = speedCharacteristic.value();
    if (speed == 2) {
      delay(200);
    } else {
      delay(500);
    }
  } else {
    STARTUP_FLAG = 1;
  }
}
