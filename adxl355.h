// Adding libaries and header 


#ifndef STUDIENARBEIT_ADXL_H
#define STUDIENARBEIT_ADXL_H
#pragma once 


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"  
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <iostream>
#include "WiFi.h"
#include "string.h"
#include "PubSubClient.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include <SPI.h>
#include "driver/spi_master.h"
#include "string.h"
#include "driver/uart.h"


#define WIFI_SSID ""       // Replace with your WiFi SSID
#define WIFI_PASS ""   // Replace with your WiFi Password
#define MAX_RETRY 10                // Number of retries before failing

using namespace std;
// End of libraries and header

 

// Definition ports as I2C_Master
#define I2C_MASTER_SDA GPIO_NUM_21
#define I2C_MASTER_SCL GPIO_NUM_22
#define I2C_MASTER_PORT I2C_NUM_0
// End of defintion 


//Scaling-Factors 
#define SCALING_FACTOR_2G 3.9e-6  // scaling factor for 2g
#define SCALING_FACTOR_4G 7.8e-6  //4g 
#define SCALING_FACTOR_8G 15.6e-6 // 8g 




// Register Adresses
#define ADXL355      0x1D // R/W
#define DEVID_AD     0x00  // R
#define DEVID_MST    0x01  // R
#define PARTID       0x02  // R
#define REVID        0x03  // R
#define STATUS__     0x04  // R
#define FIFO_ENTRIES 0x05  // R
#define TEMP2        0x06  // R
#define TEMP1        0x07  // R
#define XDATA3       0x08  // R
#define XDATA2       0x09  // R
#define XDATA1       0x0A  // R
#define YDATA3       0x0B  // R
#define YDATA2       0x0C  // R
#define YDATA1       0x0D  // R
#define ZDATA3       0x0E  // R
#define ZDATA2       0x0F  // R
#define ZDATA1       0x10  // R
#define FIFO_DATA    0x11  // R
#define OFFSET_X_H   0x1E  // R/W
#define OFFSET_X_L   0x1F  // R/W
#define OFFSET_Y_H   0x20  // R/W
#define OFFSET_Y_L   0x21  // R/W
#define OFFSET_Z_H   0x22  // R/W
#define OFFSET_Z_L   0x23  // R/W
#define ACT_EN       0x24  // R/W
#define ACT_TRESH_H  0x25  // R/W
#define ACT_TRESH_L  0x26  // R/W
#define ACT_COUNT    0x27  // R/W
#define Filter       0x28  // R/W
#define FIFO_SAMPLES 0x29  // R/W
#define INT_MAP      0x2A  // R/W
#define SYNC         0x2B  // R/W
#define Range        0x2C  // R/W
#define POWER_CTL    0x2D  // R/W
#define SELF_TEST    0x2E  // R/W
#define Reset        0x2F  // W
#define FIFO_MAX 96 
#define Bytes_per_Sample 9 
#define READ_BYTE    0x01
#define WRITE_BYTE   0x00



// Register Definition End



class I2C{
    public:
void init_i2c();
void writeRegister(uint8_t device_address,uint8_t register_adress,uint8_t data);
void scanDevices();
uint8_t readRegister(uint8_t device_adress, uint8_t register_adress);
};

class ADXL : public I2C{
    private: 
    public:
    ADXL();   // Constructor
    ~ADXL();  // Destructor
    #define SAMPLE_COUNT 6000
    uint64_t data_index=0;  
    uint8_t data_x[3];
    uint8_t data_y[3];
    uint8_t data_z[3];
    struct adxl355_data {
        float x;
        float y;
        float z;
        };
    struct adxl355_data data_buffer[SAMPLE_COUNT];
    volatile bool start_collection = false;    
void getRawAccelerations(uint8_t device_address,uint8_t register_address,uint8_t *buffer,uint8_t length);
float combine(uint8_t *data);
void setRange(int RangeInput);
uint8_t getRange();
float GetandSetScalingFactor();
float getAccelerations(int32_t raw_value);
void setActivitythreshold(float g);
void setOffset(int axis,float g);
void measurementMode();
void standbyMode();
//void collect_adxl355_data();
void stream_adxl355_data();
void collect_Data();
};

class Wifi_setup {
        private:
        const char* ssid = "";
        const char* password = "";
        public: 
        void setSSID(const char* your_ssid, const char* your_password);
        void connect();
        bool isConnected();
        void Current_status();
        void scanWiFiNetworks();

};

class MQTT_setup {
    private: 
    const char* mqtt_server = "";  // Replace with your laptop’s IP
    #define MQTT_BROKER ""  // Find your Windows IP using ipconfig
    #define MQTT_PORT
    #define MQTT_TOPIC ""
    public:
    void connect_to_mqtt();
    void publish_to_broker(float x,float y,float z);
    void check_connection();
};



#endif 
