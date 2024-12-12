#ifndef XSENS_DOT_ARDUINOBLE_H
#define XSENS_DOT_ARDUINOBLE_H

#include <ArduinoBLE.h>
#include <vector>

class BLEHandler{
public:
    BLEHandler();
    void begin();
    void end();
    void scan();
    void handleBLE();
    void onPeripheralDiscovered(BLEDevice& device);
    void onPeripheralConnected(BLEDevice& device);
    void onPeripheralDisconnected();
    void handleCharacteristic(BLEDevice& device);
    BLEDevice peripheral;
    bool isConnected = false;
};

enum class PayloadMode{
    OrientationEuler = 0,
    OrientationQuaternion = 1,
    CustomMode1 = 2,
    CustomMode5 = 3
};

class DotData{
public:    
    String name;
    String address;
    String timestamp;
    std::vector<std::vector<float>> Euler;
    std::vector<std::vector<float>> Quaternion;

};

class Scanner{
public:
    PayloadMode PayloadType;
    DotData dotdata;
    BLEHandler handler;
    bool init();
    bool connect_to_sensor(BLEDevice& device);
    void disconnect_from_sensor(BLEDevice& device);
    void getBatteryInfo();
    String getDeviceTag();
    String hexToAscii(String &name);
    void identifySensor();
    void powerOffSensor();
    void headingReset();
    void startMeasurement();
    void stopMeasurement();
    bool setOutputRate(uint8_t outPutRate);
    void enableSensor(const uint8_t* payload, size_t length);
    void disableSensor(const uint8_t* payload, size_t length);
    void OrientationEuler_notification_handler(const uint8_t* byteData, size_t length);
    float hexToFloat(const char* hexString);
    void OrientationQuaternion_notification_handler(const uint8_t* byteData, size_t length);
    void CustomMode1_notification_handler(const uint8_t* byteData, size_t length);
    void CustomMode5_notification_handler(const uint8_t* byteData, size_t length);
    void CustomMode5_notification_handler_calibration(const uint8_t* byteData, size_t length);
    void timestampConvert(const uint8_t* byteData, size_t length);
};



#endif
