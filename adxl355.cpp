
#include "ADXL355.h"




// I2C Class function implementation
void I2C::init_i2c(){
    i2c_config_t i2c_configuration = {
        i2c_configuration.mode = I2C_MODE_MASTER,
        i2c_configuration.sda_io_num = GPIO_NUM_21,
        i2c_configuration.scl_io_num = GPIO_NUM_22,
        i2c_configuration.sda_pullup_en = GPIO_PULLUP_ENABLE,
        i2c_configuration.scl_pullup_en = GPIO_PULLUP_ENABLE,
        i2c_configuration.master.clk_speed=1000000,
        i2c_configuration.clk_flags=I2C_SCLK_SRC_FLAG_FOR_NOMAL,
    };
       
  

    esp_err_t config_ret=i2c_param_config(I2C_NUM_0,&i2c_configuration); 
        if(config_ret!= ESP_OK){
        printf("Failed Configuration\n");
    };
    esp_err_t install_ret=i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0);
        if (install_ret!= ESP_OK){
        printf("Failed installation\n"); 
    };
}
void I2C::writeRegister(uint8_t device_address,uint8_t register_adress,uint8_t data) {
  
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(device_address<<1) | I2C_MASTER_WRITE ,true);
    i2c_master_write_byte(cmd,register_adress,true);
    i2c_master_write_byte(cmd,data,true);
    i2c_master_stop(cmd);


    i2c_master_cmd_begin(I2C_NUM_0,cmd,100);
    i2c_cmd_link_delete(cmd);
}
void I2C::scanDevices() {
    for (uint8_t address = 1; address < 127; address++) {

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd,pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            printf("I2C device found at address 0x%02X\n", address);
        }
    }
}
uint8_t I2C::readRegister(uint8_t device_adress, uint8_t register_adress) {
    
    uint8_t data[1]={0};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(device_adress<<1)| I2C_MASTER_WRITE ,true);
    i2c_master_write_byte(cmd,register_adress,true);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_NUM_0,cmd,1000);
    i2c_cmd_link_delete(cmd);

    cmd=i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(device_adress<<1)|I2C_MASTER_READ,true);
    i2c_master_read_byte(cmd,data,I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_NUM_0,cmd,1000);
    i2c_cmd_link_delete(cmd);
    printf("Status : Register 0%x\n",data[0]);
    return data[0];

}
// I2C Class function implementation


// ADXL Class function implementation
ADXL::ADXL() {
    data_index = 0;
    start_collection = false;
}
ADXL::~ADXL() {
    // No dynamic memory to release, but add logging or cleanup if needed
}
void ADXL::getRawAccelerations(uint8_t device_address,uint8_t register_address,uint8_t *buffer,uint8_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Start I2C transaction (Write: Select register)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, register_address, true);
    i2c_master_stop(cmd);  // Ensure stop condition

    // Execute Write command
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 100);
    i2c_cmd_link_delete(cmd);

    // Start I2C transaction (Read data)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd,buffer,length,I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    // Execute Read command
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 100);
    i2c_cmd_link_delete(cmd);
  
}
float ADXL::combine(uint8_t *data) { //input is a 20 bit raw data extracted from the registers of the adxl355
    int32_t value = (int32_t)((data[0]) << 16) | ((int32_t)data[1] << 8) | (data[2] & 0xF0); 
      value = (value>>4);
    if (value & 0x80000) {  // If the 20th bit is 1 (negative number)
        value |= 0xFFF00000;  // Extend sign bit to 32-bit integer
    }
    float end_value = value * SCALING_FACTOR_2G;
    return end_value;
}
void ADXL::setRange(int RangeInput) {
    switch (RangeInput) {
        case 2 :
        printf("Range set to 2g !");
        writeRegister(0x1D,Range,0x2C|0x01);
        break;
        case 4 : 
        printf("Range set to 4g !");
        writeRegister(0x1D,Range,0x2C|0x02);
        break;
        case 8 : 
        printf("Range set to 8g !");
        writeRegister(0x1D,Range,0x2C|0x03);
        break;
        default:
       writeRegister(0x1D,Range,0x2C|0x01);
    }   
}
float ADXL::GetandSetScalingFactor(){
    float Scaling_Factor=0;
    uint8_t  g = readRegister(ADXL355,Range);
    if (g&0x01) {Scaling_Factor = SCALING_FACTOR_2G;}          // Sets the scaling factor to the set range, since different scaling factors 
    else if (g&0x02){ Scaling_Factor=SCALING_FACTOR_4G;}       // for different range 
    else if (g&0x03) {Scaling_Factor=SCALING_FACTOR_8G;}
    return Scaling_Factor;
}
float ADXL::getAccelerations(int32_t raw_value) {
    return raw_value *SCALING_FACTOR_2G;                          // gives the valie in g
}
void ADXL::setActivitythreshold(float g) {
    float scaling_factor=GetandSetScalingFactor();
    int32_t lsb = (g/scaling_factor); 
    lsb = (lsb>>3)&(0xFFFF);
    uint8_t high_byte = (lsb>>8);
    uint8_t low_byte = (lsb&0xFF);
    writeRegister(ADXL355,ACT_EN,0x07); // Activity Enable for all Three Axis
    writeRegister(ADXL355,ACT_TRESH_H,high_byte); // Set the Threshold to 0.1G --> necessary to store in two 8 bit register this is the higher 8 bits
    writeRegister(ADXL355,ACT_TRESH_L,low_byte); //Activity Threshold low register 
    }
void ADXL::setOffset(int axis,float g) { // 1 == x , 2== y , 3==z
    float scaling_factor=SCALING_FACTOR_2G;
    int32_t lsb = (g/scaling_factor);
    lsb = (lsb>>4)&(0xFFFF);
    int8_t high_byte = ((lsb>>8) & 0xFF);
    int8_t low_byte = (lsb&0xFF);
    // 1 equals axis x , 2 equals y axis , 3 equals z axis
    if (axis == (1)) {
        writeRegister(ADXL355,OFFSET_X_H,high_byte); // subtract 1g to zero the sensor out  
        writeRegister(ADXL355,OFFSET_X_L,low_byte);  
    }
    else if (axis == (2)) {
        writeRegister(ADXL355,OFFSET_Y_H,high_byte); // subtract 1g to zero the sensor out  
        writeRegister(ADXL355,OFFSET_Y_L,low_byte);  
    }
    else if (axis == (3)) {
        writeRegister(ADXL355,OFFSET_Z_H,high_byte); // subtract 1g to zero the sensor out  
        writeRegister(ADXL355,OFFSET_Z_L,low_byte);  
    }
}
void ADXL::measurementMode() {
    writeRegister(ADXL355,POWER_CTL, 0x00);  // Put in measurement mode
    vTaskDelay(pdMS_TO_TICKS(100));  // Allow time to stabilize
}
uint8_t ADXL::getRange() {
    int range  = 0;
    uint8_t  g =  readRegister(ADXL355,Range);
    if (g&0x01) {range=2;}          // Sets the scaling factor to the set range, since different scaling factors 
    else if (g&0x02){range=4;}       // for different range 
    else if (g&0x03) {range=8;}
    return range;
}
void ADXL::standbyMode() {
writeRegister(ADXL355,POWER_CTL, 0x01);  // Put in measurement mode
vTaskDelay(pdMS_TO_TICKS(100));  // Allow time to stabilize
};
void ADXL::collect_Data() {
 printf("Collecting ADXL355 data for 10 seconds...\n");
  
 uint64_t start_time = esp_timer_get_time();
 data_index = 0; // Reset index for storing data
  
    while ((esp_timer_get_time() - start_time) < 5 * 1000000 && data_index < SAMPLE_COUNT) {
        // Collect X-axis data
        getRawAccelerations(ADXL355, XDATA3, data_x, 3);
        data_buffer[data_index].x=combine(data_x);
        // Collect y-axis data 
        getRawAccelerations(ADXL355, YDATA3, data_y, 3);
        data_buffer[data_index].y=combine(data_y);
        // Collect Z-axis data
        getRawAccelerations(ADXL355, ZDATA3, data_z, 3);
        data_buffer[data_index].z=combine(data_z);
        data_index++; 
};
  // Number of samples stored in Buffer
  printf("Data collection finished. %d samples stored.\n", data_index);

  // for loop to serial print out the saved data, done after collecting data to save more data points 
  start_collection = false;
    for (int i=0;i<data_index;i++){
      cout << i << ", " << data_buffer[i].x << ", " <<  data_buffer[i].y << ", "<< data_buffer[i].z  << endl;
    }
};
void ADXL::stream_adxl355_data(){    
    getRawAccelerations(ADXL355, XDATA3, data_x, 3);
    getRawAccelerations(ADXL355, YDATA3, data_y, 3);
    getRawAccelerations(ADXL355, ZDATA3, data_z, 3);


    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);

    Serial.write(data_x[0]);
    Serial.write(data_x[1]);
    Serial.write(data_x[2]);

    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);

    Serial.write(data_y[0]);
    Serial.write(data_y[1]);
    Serial.write(data_y[2]);

    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);
    Serial.write(0xFF);

    Serial.write(data_z[0]);
    Serial.write(data_z[1]);
    Serial.write(data_z[2]);
    

}
      //ADXL Class function implemenetation



// Wifi Class function implementation
void Wifi_setup::setSSID(const char* your_ssid, const char* your_password) {
    Wifi_setup wifi;
    wifi.ssid = your_ssid;
    wifi.password= your_password;
}
void Wifi_setup::connect(){
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid,password);
    cout << "Connecting to Wifi.." << endl;
    int attempts = 0;

    while ((WiFi.status() != WL_CONNECTED) && (attempts <= 20)) {
        vTaskDelay(pdMS_TO_TICKS(500));
        attempts++;
    }
    if(WiFi.status() == WL_CONNECTED) {
        cout << " Wifi is connected to : " << WiFi.localIP().toString().c_str() << endl;
    } else {
        cout << "Error! Wifi is not connected" << endl;
    }

}
void Wifi_setup::scanWiFiNetworks() {
    Serial.println("Scanning for available networks...");
    
    // Start scanning
    int numNetworks = WiFi.scanNetworks();
    
    if (numNetworks == 0) {
        cout <<"No networks found." << endl;
    } else {
        cout << "Found"  <<  numNetworks<<  "networks:\n" << endl; 
        
        for (int i = 0; i < numNetworks; i++) {
            printf("%d: %s, Signal Strength (RSSI): %d dBm, Encryption: %s\n", 
                i + 1, 
                WiFi.SSID(i).c_str(), 
                WiFi.RSSI(i),
                (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Secured"
            );
        }
    }

    // Cleanup scan results to free memory
    WiFi.scanDelete();
}
// Wifi Class function implementation

//MQTT Class function implementation 
void MQTT_setup::connect_to_mqtt() {
    WiFiClient espClient;
    PubSubClient mqttClient(espClient);
    while (!mqttClient.connected()) {
        cout << "MQTT Connecting.." << endl;
        if (mqttClient.connect("ESP32Client")) {  // Unique client ID
            cout << "Connected to MQTT!" << endl;
        } else {
            cout << "MQTT Failed, retrying in ...";
        }
    }
}
void MQTT_setup::publish_to_broker(float x,float y,float z){
    WiFiClient espClient;
    PubSubClient mqttClient(espClient);
    char msg[100];
    snprintf(msg, sizeof(msg), "{\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}", x, y, z);
    mqttClient.publish(MQTT_TOPIC, msg);
    cout << "Data Sent: " + String(msg);
}
void MQTT_setup::check_connection(){
    WiFiClient espClient;
    PubSubClient mqttClient(espClient);
    
}
