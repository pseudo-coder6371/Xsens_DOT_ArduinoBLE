#include "xsens_dot_arduinoble.h"
#include <Arduino.h>
#include <ArduinoBLE.h>

BLEHandler::BLEHandler(){}

BLEHandler handler;

// UUID definitions
BLEUuid DOT_Configuration_ServiceUUID("15171000-4947-11E9-8646-D663BD873D93");
BLEUuid DOT_Configuration_Control_CharacteristicUUID("15171002-4947-11E9-8646-D663BD873D93");
BLEUuid DOT_Measure_ServiceUUID("15172000-4947-11E9-8646-D663BD873D93");
BLEUuid Heading_Reset_Control_CharacteristicUUID("15172006-4947-11E9-8646-D663BD873D93");
BLEUuid DOT_ShortPayload_CharacteristicUUID("15172004-4947-11E9-8646-D663BD873D93");
BLEUuid DOT_MediumPayload_CharacteristicUUID("15172003-4947-11E9-8646-D663BD873D93");
BLEUuid DOT_Battery_CharacteristicUUID("15173001-4947-11E9-8646-D663BD873D93");
BLEUuid DOT_LongPayload_CharacteristicUUID("15172002-4947-11E9-8646-D663BD873D93");
BLEUuid DOT_Battery_ServiceUUID("15173000-4947-11E9-8646-D663BD873D93");
BLEUuid DOT_Control_Measure_CharacteristicUUID("15172001-4947-11E9-8646-D663BD873D93");

BLEService pService_battery(DOT_Battery_ServiceUUID.str());
BLEService pConfig_Service(DOT_Configuration_ServiceUUID.str());
BLEService pMeasurement(DOT_Measure_ServiceUUID.str());
BLECharacteristic pBattery(DOT_Battery_CharacteristicUUID.str(), BLERead | BLENotify, 128);
BLECharacteristic pGetTag(DOT_Configuration_Control_CharacteristicUUID.str(), BLERead, 128);
BLECharacteristic pIdentifySensor(DOT_Configuration_Control_CharacteristicUUID.str(), BLEWrite | BLERead, 128);
BLECharacteristic pPowerOff(DOT_Configuration_Control_CharacteristicUUID.str(), BLEWrite | BLERead, 128);
BLECharacteristic pHeadingReset(Heading_Reset_Control_CharacteristicUUID.str(), BLEWrite, 128);
BLECharacteristic pShortPayload(DOT_ShortPayload_CharacteristicUUID.str(), BLENotify, 128);
BLECharacteristic pOutputRate(DOT_Configuration_Control_CharacteristicUUID.str(), BLEWrite | BLERead, 128);
BLECharacteristic pControl_measure(DOT_Control_Measure_CharacteristicUUID.str(), BLEWrite, 128);
BLECharacteristic pMediumPayload(DOT_MediumPayload_CharacteristicUUID.str(), BLENotify, 128);
BLECharacteristic pLongPayload(DOT_LongPayload_CharacteristicUUID.str(), BLENotify, 128);

// Control command arrays
const byte Select_Orientation_Euler[] = {1, 1, 4};
const byte Deselect_Orientation_Euler[] = {1, 0, 4};
const byte Select_Orientation_Quaternion[] = {1, 1, 5};
const byte Deselect_Orientation_Quaternion[] = {1, 0, 5};
const byte Heading_Reset_Buffer[] = {1, 0};
const byte Select_Custom_Mode[] = {1, 1, 22};
const byte Deselect_Custom_Mode[] = {1, 0, 22};
const byte Select_Custom_Mode5[] = {1, 1 , 26};
const byte Deselect_Custom_Mode5[] = {1, 0 , 26};

bool isInit = false;
extern unsigned long startTime;
extern int time_duration;
extern bool measurementStarted_yes;
bool isConnected = false;

void BLEHandler::begin(){
    if(!BLE.begin()){
        Serial.println("Failed to initialise BLE");
        while(1);
    }
    Serial.println("BLE initialised");
}

void BLEHandler::end(){
    BLE.end();
    Serial.println("BLE un-initialised");
}

void BLEHandler::scan(){
    Serial.println("Starting scan");
    BLE.scan();
}

void BLEHandler::handleBLE(){
    BLEDevice discoveredDevice = BLE.available();
    if(discoveredDevice){
        onPeripheralDiscovered(discoveredDevice);
    }

    if(isConnected && !peripheral.connected()){
        Serial.println("Sensor disconnected");
        isConnected = false;
        onPeripheralDisconnected();
    }
}

void BLEHandler::onPeripheralDiscovered(BLEDevice& device){
    Serial.print("Discovered device: ");
    Serial.println(device.localName());
    if(peripheral.localName() == "Movella DOT"){
        Serial.println("Found Xsens DOT sensor");
        Serial.print("Address: ");
        Serial.println(peripheral.address());
        peripheral = device;
        BLE.stopScan();
        Serial.print("Device signal strength: ");
        Serial.print(peripheral.rssi());
        Serial.println(" dBm");
    }
}

void BLEHandler::onPeripheralConnected(BLEDevice& device){  
    Serial.println("Peripheral connected");
    handleCharacteristic(device);
}

void BLEHandler::onPeripheralDisconnected(){
    Serial.println("Restarting scan..");
    scan();
}

bool Scanner::connect_to_sensor(BLEDevice& device){
    if(device.connect() && !isConnected){
        isConnected = true;
        Serial.println("Connected to sensor");
        pService_battery = device.service(DOT_Battery_ServiceUUID.str());
        pConfig_Service = device.service(DOT_Configuration_ServiceUUID.str());
        pMeasurement = device.service(DOT_Measure_ServiceUUID.str());
        return true;
    }else{
        Serial.println("Failed to connect, retrying.. ");
        delay(1000);
        handler.scan();
        return false;
    }
}

void Scanner::disconnect_from_sensor(BLEDevice& device){
    if(device.connected()){
        device.disconnect();
        Serial.println("Sensor is not connected");
        return;
    }
    handler.end();
    isConnected = false;
}

void Scanner::getBatteryInfo(){
    byte batteryValue[2];
    pBattery = pService_battery.characteristic(DOT_Battery_CharacteristicUUID.str());
    batteryValue[2] = pBattery.readValue(batteryValue, sizeof(batteryValue));
    uint8_t batteryPer = batteryValue[0];
    uint8_t status = batteryValue[1];

    Serial.print("Battery level: ");
    Serial.print(batteryPer);
    Serial.print(" %, Charging status: ");
    Serial.println(status == 0 ? "Not charging" : "Charging");
}

String Scanner::getDeviceTag(){
    pGetTag = pConfig_Service.characteristic(DOT_Configuration_Control_CharacteristicUUID.str());
    String tag_device_bytes = "";
    String tag_test_raw = "";
    byte* Tag;
    Tag = (byte*)malloc(20*sizeof(byte));
    pGetTag.readValue(Tag, 20);

    for(int i = 8; i < 20; ++i){
        String hexByte = String(Tag[i], HEX);
        if(hexByte.length()==1 ) hexByte = "0" + hexByte;
        tag_device_bytes+=hexByte;
    }
    String name = hexToAscii(tag_device_bytes);
    Serial.print("Read device tag: ");
    Serial.println(name);
    return name;
    if(!pGetTag){
        Serial.println("Device tag not found");
        return "Not found";
    }
    free(Tag);
}

String Scanner::hexToAscii(String &name){
    String ascii = ""; 
    for (size_t i = 0; i < name.length(); i += 2) {
        String part = name.substring(i, i + 2);
        char ch = strtoul(part.c_str(), nullptr, 16);
        if (ch != '\0') {
            ascii += ch;
        }
    }
    return ascii;
}

void Scanner::identifySensor(){
    if(pIdentifySensor = pConfig_Service.characteristic(DOT_Configuration_Control_CharacteristicUUID.str())){
        String UUID = pIdentifySensor.uuid();
        // Serial.println(UUID);
        delay(1000);
        uint8_t bArr[32];
        bArr[0] = 1;
        bArr[1] = 1;
        pIdentifySensor.writeValue(bArr, sizeof(bArr));
        Serial.println("Identifying, check the LED on your sensor");
    }else{
        Serial.println("Cannot identify sensor, please check the code");
    }

}

void Scanner::powerOffSensor(){
    if(pPowerOff = pConfig_Service.characteristic(DOT_Configuration_Control_CharacteristicUUID.str())){
        delay(1000);
        uint8_t bArr[32];
        bArr[0] = 2;
        bArr[1] = 0;
        bArr[2] = 1;
        pPowerOff.writeValue(bArr, sizeof(bArr));
        Serial.println("Sensor powering off.. Goodbye.");
    }else{
        Serial.println("Cannot switch off the sensor");
    }
}

void Scanner::headingReset(){
    if(pHeadingReset = pMeasurement.characteristic(Heading_Reset_Control_CharacteristicUUID.str())){
        Serial.println("Heading reset complete.");
    }else{
        Serial.println("Heading reset failed");
    }
}

bool Scanner::setOutputRate(uint8_t outPutRate){
    const int validRates[] = {1,4,10,12,15,20,30,60,120};
    uint8_t bArr[32];
    bool isValid = false;

    for (size_t i = 0; i < sizeof(validRates); i++){
        if(outPutRate == validRates[i]){
            isValid = true;
            break;
        }
    }
    if(isValid == true){
        bArr[0] = 16;
        bArr[24] = outPutRate & 0xff;
        bArr[25] = (outPutRate >>8) & 0xff;
        Serial.println("Setting output rate");
        pOutputRate = pConfig_Service.characteristic(DOT_Configuration_Control_CharacteristicUUID.str());
        if(pOutputRate.writeValue(bArr, sizeof(bArr))){
            Serial.print("Successfully set output rate to: ");
            Serial.print(outPutRate);
            Serial.println(" Hz");
            delay(100);
            return true;
        }else{
            Serial.println("Error setting output rate");
        }
    }
    return false;
}

bool measurementStarted = false;


void Scanner::startMeasurement() {
    if (PayloadType == PayloadMode::OrientationEuler) {
        // Serial.println("Payload mode: Euler Angles");
        if (!measurementStarted) {
            // pShortPayload = pMeasurement.characteristic(DOT_ShortPayload_CharacteristicUUID.str());
            if (!pShortPayload) {
                Serial.println("Short Payload characteristic not found!");
                return;
            }
            enableSensor(Select_Orientation_Euler, sizeof(Select_Orientation_Euler));
            if (pShortPayload.subscribe()) {
                Serial.println("Successfully subscribed to notifications for Short payload");
                BLEDeviceEvent BLENotify;
                BLEDeviceEventHandler OrientationEuler_notification_handler;
                BLE.setEventHandler(BLENotify, OrientationEuler_notification_handler);
            } else {
                Serial.println("Failed to subscribe to notifications");
            }
            measurementStarted = true;
        } else {
            // Serial.println("Measurement already started, continuing data streaming...");
        }
    }else if (PayloadType == PayloadMode::OrientationQuaternion){
        if(!measurementStarted){
            // pShortPayload = pMeasurement.characteristic(DOT_ShortPayload_CharacteristicUUID.str());
            if(!pShortPayload){
                Serial.println("Short payload characteristic not found");
                return;
            }
            enableSensor(Select_Orientation_Quaternion, sizeof(Select_Orientation_Quaternion));
            if(pShortPayload.subscribe()){
                Serial.println("Successfully subscribed to short payload notifications");
                BLEDeviceEvent BLENotify;
                BLEDeviceEventHandler OrientationQuaternion_notification_handler;
                BLE.setEventHandler(BLENotify, OrientationQuaternion_notification_handler);
            }else{
                Serial.println("Failed to subscribe to notifications");
            }
            measurementStarted = true;
        }else{
            Serial.println("Measurement already started..");
        }
    }else if(PayloadType == PayloadMode::CustomMode1){
        if(!measurementStarted){
            // pMediumPayload = pMeasurement.characteristic(DOT_MediumPayload_CharacteristicUUID.str());
            if(!pMediumPayload){
                Serial.println("Characteristic not found");
                return;
            }
        }   enableSensor(Select_Custom_Mode, sizeof(Select_Custom_Mode));
            if(pMediumPayload.subscribe()){
                Serial.println("Successfully subscribed to medium payload notifications");
                BLEDeviceEvent BLENotify;
                BLEDeviceEventHandler CustomMode1_notification_handler;
                BLE.setEventHandler(BLENotify, CustomMode1_notification_handler);
            }else{
                Serial.println("Failed to subscribe to medium payload notifications");
            }
            measurementStarted = true;
    }else if(PayloadType == PayloadMode::CustomMode5){
        if(!measurementStarted){
            // pLongPayload = pMeasurement.characteristic(DOT_LongPayload_CharacteristicUUID.str());
            if(!pLongPayload){
                Serial.println("Characteristic not found");
                return;
            }
            enableSensor(Select_Custom_Mode5, sizeof(Select_Custom_Mode5));
            if(pLongPayload.subscribe()){
                Serial.println("Successfully subscribed to long payload notifications");
                BLEDeviceEvent BLENotify;
                BLEDeviceEventHandler CustomMode5_notification_handler;
                BLE.setEventHandler(BLENotify, CustomMode5_notification_handler);
            }else{
                Serial.println("Could not subscribe to long payload notifications");
            }
            measurementStarted = true;
        }
    }
}

void Scanner::stopMeasurement(){
    if (PayloadType == PayloadMode::OrientationEuler){
        disableSensor(Deselect_Orientation_Euler, sizeof(Deselect_Orientation_Euler));
        pShortPayload.unsubscribe();
        Serial.println();
        Serial.println("Unsubscribed from notifications.. Stopping measurement");

    }else if (PayloadType == PayloadMode::OrientationQuaternion){
        disableSensor(Deselect_Orientation_Quaternion, sizeof(Deselect_Orientation_Quaternion));
        pShortPayload.unsubscribe();
        Serial.println();
        Serial.println("Unsubscribed from notifications.. Stopping measurement");

    }else if(PayloadType == PayloadMode::CustomMode1){
        disableSensor(Deselect_Custom_Mode, sizeof(Deselect_Custom_Mode));
        pMediumPayload.unsubscribe();
        Serial.println();
        Serial.println("Unsubscribed from notifications.. Stopping measurement");
    }else if(PayloadType == PayloadMode::CustomMode5){
        disableSensor(Deselect_Custom_Mode5, sizeof(Deselect_Custom_Mode5));
        pLongPayload.unsubscribe();
        Serial.println();
        Serial.println("Unsubscribed from notifications.. Stopping measurement");
    }
}

void Scanner::enableSensor(const uint8_t* payload, size_t length){
    if(pControl_measure.writeValue(payload, length)){
        delay(100);
        Serial.println("Sensor is enabled");
    }else{
        Serial.println("Could not write values to enable sensor");
    }
}

void Scanner::disableSensor(const uint8_t* payload, size_t length){
    if(pControl_measure.writeValue(payload, length)){
        delay(100);
        Serial.println();
        Serial.println("Sensor is disabled");
    }else{
        Serial.println("Could not write values to disable sensor");
    }
}

void Scanner::OrientationEuler_notification_handler(const uint8_t* byteData, size_t length) {
    uint32_t timestamp = *reinterpret_cast<const uint32_t*>(byteData);
    float roll = *reinterpret_cast<const float*>(byteData + 4);
    float pitch = *reinterpret_cast<const float*>(byteData + 8);
    float yaw = *reinterpret_cast<const float*>(byteData + 12);

    String StringToPrint = "Time: " + String(timestamp) + ", Roll: " + String(roll,6) + ", Pitch: " + String(pitch,6) + ", Yaw: " + String(yaw,6);
    Serial.println(StringToPrint);

}


void Scanner::OrientationQuaternion_notification_handler(const uint8_t* byteData, size_t length){

    uint32_t timestamp = *reinterpret_cast<const uint32_t*>(byteData);
    float w = *reinterpret_cast<const float*>(byteData + 4);
    float x = *reinterpret_cast<const float*>(byteData + 8);
    float y = *reinterpret_cast<const float*>(byteData + 12);
    float z = *reinterpret_cast<const float*>(byteData + 16);

    String StringToPrint = "Time: " + String(timestamp) + " , q0: "  + String(w,6) + " , q1: " + String(x,6) + ", q2: " + String(y,6) + ", q3: " + String(z,6);
    Serial.println(StringToPrint);

}


void Scanner::CustomMode1_notification_handler(const uint8_t* byteData, size_t length){

    uint32_t timestamp = *reinterpret_cast<const uint32_t*>(byteData);
    float roll = *reinterpret_cast<const float*>(byteData + 4);
    float pitch = *reinterpret_cast<const float*>(byteData + 8);
    float yaw = *reinterpret_cast<const float*>(byteData + 12);
    float acc_x = *reinterpret_cast<const float*>(byteData + 16);
    float acc_y = *reinterpret_cast<const float*>(byteData + 20);
    float acc_z = *reinterpret_cast<const float*>(byteData + 24);
    float gyro_x = *reinterpret_cast<const float*>(byteData + 28);
    float gyro_y = *reinterpret_cast<const float*>(byteData + 32);
    float gyro_z = *reinterpret_cast<const float*>(byteData + 36);

    String StringToPrint = "Time: " + String(timestamp) + ", roll: " + String(roll, 6) + ", pitch: " + String(pitch, 6) + ", yaw: " + String(yaw,6) + ", Acc_X: " + String(acc_x,6) 
    + ", Acc_Y: " + String(acc_y,6) + ", Acc_Z: " + String(acc_z,6) + ", Gyro_X: " + String(gyro_x,6) + ", Gyro_Y: " + String(gyro_y,6) + ", Gyro_Z: " + String(gyro_z,6);
    Serial.println(StringToPrint);
}

void Scanner::CustomMode5_notification_handler(const uint8_t* byteData, size_t length){

    uint32_t timestamp = *reinterpret_cast<const uint32_t*>(byteData);
    float w = *reinterpret_cast<const float*>(byteData + 4);
    float x = *reinterpret_cast<const float*>(byteData + 8);
    float y = *reinterpret_cast<const float*>(byteData + 12);
    float z = *reinterpret_cast<const float*>(byteData + 16);
    float acc_x = *reinterpret_cast<const float*>(byteData + 20);
    float acc_y = *reinterpret_cast<const float*>(byteData + 24);
    float acc_z = *reinterpret_cast<const float*>(byteData + 28);
    float gyro_x = *reinterpret_cast<const float*>(byteData + 32);
    float gyro_y = *reinterpret_cast<const float*>(byteData + 36);
    float gyro_z = *reinterpret_cast<const float*>(byteData + 40);

    String StringToPrint = "Time: " + String(timestamp) + " , q0: " + String(w,6) + " , q1: " + String(x,6) + " , q2: " + String(y,6) + " , q3: " + String(z,6) 
    + " , Acc_X: " + String(acc_x,6) + " , Acc_Y: " + String(acc_y,6) + " , Acc_Z: " + String(acc_z,6) + " , Gyro_X: " + String(gyro_x,6) + " , Gyro_Y: " + String(gyro_y,6) + 
    " , Gyro_Z: " + String(gyro_z,6);

    Serial.println(StringToPrint);

}

