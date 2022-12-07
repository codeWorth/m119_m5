#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

#define BLE_UUID_PADDLE_SERVICE "5102"
#define BLE_UUID_AZ "2101"

BLEService paddleService(BLE_UUID_PADDLE_SERVICE);
BLEFloatCharacteristic az(BLE_UUID_AZ, BLERead | BLENotify);

unsigned long last_micros = 0;
float movement;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  Serial.print(F("Accelerometer sample rate = "));
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(F("Hz"));

  // initialize BLE
  if (!BLE.begin()) {
    Serial.println(F("starting Bluetooth® Low Energy failed!"));

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setDeviceName("Andrew Arduino");
  BLE.setLocalName("Andrew Arduino");
  BLE.setAdvertisedService(paddleService);

  // add the characteristic to the service
  paddleService.addCharacteristic(az);

  // add service
  BLE.addService(paddleService);

  // set the initial value for the characteristic:
  az.writeValue(1);

  // start advertising
  BLE.advertise();

  Serial.println(F("BLE LED Peripheral"));
}

void loop() {
  float ax, ay, az;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);

    BLEDevice central = BLE.central();
    if (central) {
      az.writeValue(az);
    }
  }
}