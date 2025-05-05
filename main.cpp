#include "ADXL355.h"

// Class implementation 
ADXL adxl;
uint64_t data_counter=0;
const uint8_t HEADER = 0xAA;
const uint8_t FOOTER = 0x55;
const uint8_t ESCAPE = 0xBB;
uint8_t data_index=0;
void sendBitstuffedPacket(uint8_t data[3]){
  if(data[2]=0xF0){data[2]=0xE0;data[2]|=(1<<2);}
  if(data[1]==0xFF){data[1]=0xFE;data[2]|=2;}
  if(data[0]==0xFF){data[0]=0xFE;data[2]|=(1);}
  Serial.write(data[0]);
  Serial.write(data[1]);
  Serial.write(data[2]);
}
void sendBinaryData(){
  uint64_t start_time=esp_timer_get_time();
  while((esp_timer_get_time()-start_time) < 5 *1000000){
  adxl.getRawAccelerations(ADXL355, XDATA3, adxl.data_x, 3);
  adxl.getRawAccelerations(ADXL355, YDATA3, adxl.data_y, 3);
  adxl.getRawAccelerations(ADXL355, ZDATA3, adxl.data_z, 3);

  Serial.write(0xFF);
  sendBitstuffedPacket(adxl.data_x);
  sendBitstuffedPacket(adxl.data_y);
  sendBitstuffedPacket(adxl.data_z);
}
}
void stream(){
  
  adxl.getRawAccelerations(ADXL355, XDATA3, adxl.data_x, 3);
  adxl.getRawAccelerations(ADXL355, YDATA3, adxl.data_y, 3);
  adxl.getRawAccelerations(ADXL355, ZDATA3, adxl.data_z, 3);

  Serial.write(0xFF); 
  sendBitstuffedPacket(adxl.data_x); 

}
  
void sample_and_print_adxl355() {
    uint64_t start_time = esp_timer_get_time();
    uint64_t data_index = 0;
    uint64_t samples=0;

    // Data collection phase
    while (((esp_timer_get_time() - start_time) < 5 * 1000000) ) {  // 5 seconds in microseconds
        // Read sensor data (replace with your actual read functions)
      adxl.getRawAccelerations(ADXL355, XDATA3, adxl.data_x, 3);
      float x=adxl.combine(adxl.data_x);
       // Collect y-axis data 
       adxl.getRawAccelerations(ADXL355, YDATA3, adxl.data_y, 3);
       float y=adxl.combine(adxl.data_y);
       // Collect Z-axis data
       adxl.getRawAccelerations(ADXL355, ZDATA3, adxl.data_z, 3);
       float z=adxl.combine(adxl.data_z);
       samples++;
       cout << samples <<";" << x <<";"<< y<< ";"<< z<<"\n"; 
      
    }

}
void collect_adxl355_data(){
uint64_t start_time = esp_timer_get_time();
uint64_t data_index=0;

      while ((esp_timer_get_time() - start_time) < 2 * 1000000){ //&& data_index < 5000) {
      
             //Collect X-axis data
             adxl.getRawAccelerations(ADXL355, XDATA3, adxl.data_x, 3);
             adxl.data_buffer[data_index].x=adxl.combine(adxl.data_x);
            // Collect y-axis data 
            adxl.getRawAccelerations(ADXL355, YDATA3, adxl.data_y, 3);
            adxl.data_buffer[data_index].y=adxl.combine(adxl.data_y);
            // Collect Z-axis data
            adxl.getRawAccelerations(ADXL355, ZDATA3, adxl.data_z, 3);
            adxl.data_buffer[data_index].z=adxl.combine(adxl.data_z);
            data_index++;     
       }
       //vTaskDelay(pdMS_TO_TICKS(5000));
      
       //for loop to serial print out the saved data, done after collecting data to save more data points 
      for (int i=0;i<data_index;i++){
            
            cout << i <<";" << adxl.data_buffer[i].x<<";"<<adxl.data_buffer[i].y<< ";"<<adxl.data_buffer[i].z<<"\n";
          }
          data_index=0;
      }
      
  
void setup() {
  vTaskDelay(pdMS_TO_TICKS(200));
  Serial.begin(256000);
  
  adxl.init_i2c(); // initialize i2c driver 
  adxl.writeRegister(ADXL355,0x2D,0x01);
  adxl.writeRegister(ADXL355,0x2C,0x81);
  adxl.readRegister(ADXL355,0x2C);
  adxl.readRegister(ADXL355,0x28);
  adxl.setOffset(1,0.0159705);
  adxl.setOffset(2,0.0638937);
  adxl.setOffset(3,0.-0.03894);
  adxl.measurementMode();
 
}


void loop() {  


sendBinaryData();
Serial.flush();
}
