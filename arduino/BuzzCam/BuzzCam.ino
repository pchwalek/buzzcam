/*
    Author: Patrick Chwalek (09/22/2023)
    Description: Example code used to control Insta360 X3 and RS 1-inch cameras
*/

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>

#include "pb.h"
#include "message.pb.h"
#include "pb_encode.h"

#define SERVICE_UUID "ce70"
#define CHARACTERISTIC_SYS_INFO "ce71"
#define CHARACTERISTIC_SYS_CONFIG "ce72"
#define CHARACTERISTIC_RX "ce73"

packet_t message;

packet_t message_system_info;
packet_t message_config;
packet_t message_rx;

uint8_t buffer[400];

uint32_t data_32;
uint16_t data_16;
uint8_t data_8[30];

BLEServer *pServer = NULL;
BLE2902 *pDescriptor2902;

bool deviceConnected = false;
bool oldDeviceConnected = false;

BLECharacteristic *pCharacteristicRx;
BLECharacteristic *pCharacteristicSysInfo;
BLECharacteristic *pCharacteristicSysConfig;

unsigned long myTime;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        deviceConnected = true;
        myTime = millis();
        BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer *pServer) { deviceConnected = false; }
};

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE work!");

    BLEDevice::init("BuzzCam");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    // BLECharacteristic *pCharacteristic1 = pService->createCharacteristic(
    //     CHARACTERISTIC_UUID1, BLECharacteristic::PROPERTY_WRITE || BLECharacteristic::PROPERTY_READ);

    pCharacteristicSysInfo = pService->createCharacteristic(
        CHARACTERISTIC_SYS_INFO,  BLECharacteristic::PROPERTY_NOTIFY);
    // pDescriptor2902 = new BLE2902();
    // pDescriptor2902->setNotifications(true);
    // pDescriptor2902->setIndications(true);
    // pCharacteristicTx->addDescriptor(pDescriptor2902);
    pCharacteristicSysInfo->setReadProperty(true);
    pCharacteristicSysInfo->setNotifyProperty(true);
    pCharacteristicSysInfo->setIndicateProperty(true);

    pCharacteristicSysConfig = pService->createCharacteristic(
    CHARACTERISTIC_SYS_CONFIG,  BLECharacteristic::PROPERTY_NOTIFY);
    // pDescriptor2902 = new BLE2902();
    // pDescriptor2902->setNotifications(true);
    // pDescriptor2902->setIndications(true);
    // pCharacteristicTx->addDescriptor(pDescriptor2902);
    pCharacteristicSysConfig->setReadProperty(true);
    pCharacteristicSysConfig->setNotifyProperty(true);
    pCharacteristicSysConfig->setIndicateProperty(true);

    // system state read value
    pCharacteristicRx = pService->createCharacteristic(
        CHARACTERISTIC_RX, BLECharacteristic::PROPERTY_WRITE);
    pCharacteristicRx->setReadProperty(true);

  

    pService->start();
    // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still
    // is working for backward compatibility
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    // pAdvertising->addServiceUUID(SECONDARY_SERVICE_UUID);

    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(
        0x06); // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);

    BLEDevice::startAdvertising();
    Serial.println(
        "Characteristic defined! Now you can read it in your phone!");


    /* PROTOBUF SPECIFIC */
    message_system_info.has_header = true;
    message_system_info.header.epoch = 1213232; // unix timestamp
    message_system_info.header.ms_from_start = millis();
    message_system_info.header.system_uid = 0x1234; // UID
    message_system_info.which_payload = Packet_system_info_packet_tag; // packet type is system_info
    message_system_info.payload.system_info_packet.device_recording = false;
    message_system_info.payload.system_info_packet.number_discovered_devices = 0;
    message_system_info.payload.system_info_packet.has_mark_state = true;
    message_system_info.payload.system_info_packet.mark_state.mark_number = 0;
    message_system_info.payload.system_info_packet.mark_state.beep_enabled = false;
    message_system_info.payload.system_info_packet.mark_state.timestamp_unix = 1232422;
    message_system_info.payload.system_info_packet.has_sdcard_state = true;
    message_system_info.payload.system_info_packet.sdcard_state.detected = 1;
    message_system_info.payload.system_info_packet.sdcard_state.space_remaining = 123412342;
    message_system_info.payload.system_info_packet.sdcard_state.estimated_remaining_recording_time = 214332;
    message_system_info.payload.system_info_packet.has_simple_sensor_reading = true;
    message_system_info.payload.system_info_packet.simple_sensor_reading.timestamp_unix = 12312421;
    message_system_info.payload.system_info_packet.simple_sensor_reading.co2 = 300;
    message_system_info.payload.system_info_packet.simple_sensor_reading.humidity = 40;
    message_system_info.payload.system_info_packet.simple_sensor_reading.temperature = 70;
    message_system_info.payload.system_info_packet.simple_sensor_reading.index = 3;
    // define stream and encode
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    pb_encode(&stream, Packet_fields, &message_system_info);
    pCharacteristicSysInfo->setValue(buffer, stream.bytes_written);

    Serial.print("Setting Info characteristic. Byte size: ");
    Serial.println( stream.bytes_written);
    Serial.write(buffer, stream.bytes_written);
    Serial.println();

        /* PROTOBUF SPECIFIC */
    message_config.has_header = true;
    message_config.header.epoch = 1213232; // unix timestamp
    message_config.header.ms_from_start = millis();
    message_config.header.system_uid = 0x1234; // UID
    message_config.which_payload = Packet_config_packet_tag; // packet type is system_info
    // define stream and encode
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    pb_encode(&stream, Packet_fields, &message_config);
    pCharacteristicSysConfig->setValue(buffer, stream.bytes_written);

    Serial.print("Setting Config characteristic. Byte size: ");
    Serial.println( stream.bytes_written);
    Serial.write(buffer, stream.bytes_written);
    Serial.println();
}

void loop() {
    // notify changed value
    if (deviceConnected) {
        delay(10); // bluetooth stack will go into congestion, if too many packets
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        Serial.println("connecting");
        oldDeviceConnected = deviceConnected;
    }

    /* this loop just goes through all the features */
    // if (((millis() - myTime) > CAPTURE_DELAY)) {
    //     Serial.println("capture");
    //     myTime = millis();
        // shutterButton(pCharacteristicRx);
        // delay(30000);

        // Serial.println("screen off");
        // screenToggle(pCharacteristicRx);
        // delay(5000);

        // Serial.println("screen on");
        // screenToggle(pCharacteristicRx);
        // delay(5000);
        
        // Serial.println("power off");
        // powerOff(pCharacteristicRx);
        // delay(30000);

    //     Serial.println("power on");
    //     BLEDevice::stopAdvertising();
    //     powerOnPrevConnectedCameras();
    // }
}





// void screenToggle(BLECharacteristic *characteristic) {
//     data_8[0] = 0xfc;
//     data_8[1] = 0xef;
//     data_8[2] = 0xfe;
//     data_8[3] = 0x86;
//     data_8[4] = 0x00;
//     data_8[5] = 0x03;
//     data_8[6] = 0x01;
//     data_8[7] = 0x00;
//     data_8[8] = 0x00;
//     characteristic->setValue(data_8, 9);
//     characteristic->notify();
// }

// void powerOff(BLECharacteristic *characteristic) {
//     data_8[0] = 0xfc;
//     data_8[1] = 0xef;
//     data_8[2] = 0xfe;
//     data_8[3] = 0x86;
//     data_8[4] = 0x00;
//     data_8[5] = 0x03;
//     data_8[6] = 0x01;
//     data_8[7] = 0x00;
//     data_8[8] = 0x03;
//     characteristic->setValue(data_8, 9);
//     characteristic->notify();
// }

// void powerOnPrevConnectedCameras() {
//     /* the below might be camera specific and you may need to sniff them yourself for your camera */

//     /* used for Insta360 X3 */
//     // manuf_data[14] = 0x37;
//     // manuf_data[15] = 0x4b;
//     // manuf_data[16] = 0x43;
//     // manuf_data[17] = 0x4d;
//     // manuf_data[18] = 0x54;
//     // manuf_data[19] = 0x4b;

//     /* used for Insta360 RS 1-inch */
//     manuf_data[14] = 0x38;
//     manuf_data[15] = 0x51;
//     manuf_data[16] = 0x53;
//     manuf_data[17] = 0x4a;
//     manuf_data[18] = 0x38;
//     manuf_data[19] = 0x52;

//     // BLEAdvertisementData advertisementData;
//     // advertisementData.setName("Insta360 GPS Remote");
//     // advertisementData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_DMT_CONTROLLER_SPT | ESP_BLE_ADV_FLAG_DMT_HOST_SPT);
//     // advertisementData.setManufacturerData((char *) manuf_data);

//     // BLEAdvertising* advertisement = new BLEAdvertising();
//     // advertisement->setAdvertisementData(advertisementData);
// 	  // advertisement->start();

//     // custom function that adds manufacturing data and modifies flags of advertisement
//     BLEDevice::startAdvertisingWithManufData(manuf_data);
// }

// void modeButton(BLECharacteristic *characteristic) {
//     data_8[0] = 0xfc;
//     data_8[1] = 0xef;
//     data_8[2] = 0xfe;
//     data_8[3] = 0x86;
//     data_8[4] = 0x00;
//     data_8[5] = 0x03;
//     data_8[6] = 0x01;
//     data_8[7] = 0x01;
//     data_8[8] = 0x00;
//     characteristic->setValue(data_8, 9);
//     characteristic->notify();
// }

// void shutterButton(BLECharacteristic *characteristic) {
//     data_8[0] = 0xfc;
//     data_8[1] = 0xef;
//     data_8[2] = 0xfe;
//     data_8[3] = 0x86;
//     data_8[4] = 0x00;
//     data_8[5] = 0x03;
//     data_8[6] = 0x01;
//     data_8[7] = 0x02;
//     data_8[8] = 0x00;
//     characteristic->setValue(data_8, 9);
//     characteristic->notify();
// }
