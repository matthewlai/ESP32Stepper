#ifndef BLE_H
#define BLE_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

const char* kServiceUuid = "30218d2c-b180-4651-aa2c-6ab3f320044e";
const char* kStartStopCharUuid = "bf95e3b2-4ffe-485a-acd9-e54afc8a5e0c";
const char* kBLEDeviceName = "Woodpecker";

class WoodpeckerBLEServer : public BLEServerCallbacks {
 public:
  WoodpeckerBLEServer();

  bool Started();

 private:
  void onConnect(BLEServer* pServer) {
    Serial.println("On connect");
  };

  void onDisconnect(BLEServer* pServer) {
    Serial.println("On disconnect");
    BLEDevice::startAdvertising();
  }
 
  BLEServer* server_;
  BLEService* service_;
  BLEAdvertising* advertising_;
  BLECharacteristic* start_stop_char_;
};

WoodpeckerBLEServer::WoodpeckerBLEServer() {
  // Despite what these names would suggest, they all return data owned by the BLEDevice singleton class, so we don't have to worry about
  // freeing them.
  BLEDevice::init(kBLEDeviceName);
  server_ = BLEDevice::createServer();
  server_->setCallbacks(this);
  service_ = server_->createService(kServiceUuid);
  start_stop_char_ = service_->createCharacteristic(kStartStopCharUuid, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  uint8_t val = 0x00;
  start_stop_char_->setValue(&val, 1);
  service_->start();
  advertising_ = BLEDevice::getAdvertising();
  advertising_->addServiceUUID(kServiceUuid);
  advertising_->setScanResponse(true);

  // A website says this helps with iPhone connections issue - https://randomnerdtutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/
  advertising_->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
}

bool WoodpeckerBLEServer::Started() {
  auto val = start_stop_char_->getValue();
  return val[0] != 0x00;
}

#endif  // BLE_H
