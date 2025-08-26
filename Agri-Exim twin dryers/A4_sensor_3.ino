/* 6 APRIL, 2025
  * WIRELESS SOIL SENSING CODE - AI
  * Scan temp every 3 seconds.... 
  
  * Share reading once every 10 seconds
  * Display only current temp on SCREEN in large font// backlit or in idle
  * If Reading is 0, flash XX on screen
  * 
  * Update EEPROM uptime timer once every 15mins
  */

#include <ArduinoJson.h>

JsonDocument JSON_data;



//    const char SensorID[6] = "T_1"; 
  const char SensorID[6] = "T_2"; 
 // const char SensorID[6] = "T_3"; 
  //const char SensorID[6] = "T_4"; 
 // const char SensorID[6] = "T_5";
 // const char SensorID[6] = "T_6";
///F4:65:0B:E8:C6:30

//   uint8_t broadcastAddress[] = {0xCC, 0xDB, 0xA7, 0x3E, 0x82, 0x80}; //  NIC for MUARIK DRYER
//     uint8_t broadcastAddress[] = {0xF4, 0x65, 0x0B, 0xE8, 0xC6, 0x30}; //  NIC for MUARIK irrikit
       uint8_t broadcastAddress[] = {0x1C, 0x69, 0x20, 0xA3, 0xF6, 0x60}; //  NIC for TWIN DRYER
  
  char broadcastAddress_str[50] = "MUARIK DATA ENGINE";

#include "DHT.h"

#define DHTPIN 13 //27     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
  #define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)


DHT sensor(DHTPIN, DHTTYPE);
 uint8_t reset_button = 39;

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBold24pt7b.h> // FreeSerifBold24pt7b
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 LCD(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


char temp_char[5] = "xx";
char humi_char[5] = "xx";

int backlight = 21;  bool backlit = false;
uint8_t currentScreen = 0;

// Include Libraries
  #include <esp_now.h>
  #include <WiFi.h>




typedef struct tempData{
  char who[6] = "";
  float temperature;
  float humidity; // in seconds
  float sampling_duration;
  unsigned long long uptime;
  //constructor

  tempData(){ //constructor to initializing the variables!
      strcpy(who, SensorID);
      temperature = 0.00;
      humidity = 0.00;
      sampling_duration = 0.00;
      uptime = 0;
   }
  
} tempData; 

tempData senderObj2;




typedef struct sensorData{ // all stringified
      char sendable_data_bundle[200] = "";
   }  sensorData; 

sensorData senderObj;



// Define the command data structure
typedef struct struct_cmd {
    bool ON = false;
 
} struct_cmd;

// Create a structured object
struct_cmd CMD;



uint8_t on_board = 4;
   




// Peer info 
esp_now_peer_info_t peerInfo;

const uint8_t on_board_temp = 36;
/*
    // Enable temperature sensor
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
    // Get converted sensor data
    float tsens_out;
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));
    printf("Temperature in %f °C\n", tsens_out);
    // Disable the temperature sensor if it is not needed and save the power
    ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));
*/

uint8_t green = 4;
uint8_t red   = 2;
bool flip = false;


uint8_t buzz = 14;


unsigned long long time_spent_ON = 0; 


unsigned long started = 0;
int64_t now_now = 0, prev = 0, start_sensing = 0;

uint8_t hr_int = 0, min_int = 0, sec_int = 0;
uint8_t day = 0, month = 0, year = 0;

void setup() { delay(500);
   Serial.begin(115200);
  
  pinMode(on_board, OUTPUT); 
  pinMode(buzz, OUTPUT);
  //pinMode(backlight, OUTPUT); digitalWrite(backlight, HIGH); backlit = true;


  sensor.begin();

  
  delay(50);


if(!LCD.begin(SSD1306_SWITCHCAPVCC, 0x3C)){ // Address 0x3D for 128x64
     // Serial.println(F("SSD1306 allocation failed"));
      //for(;;);
  }

  
  

  LCD.clearDisplay();

  LCD.setFont();
  LCD.setTextSize(1);
  LCD.setTextColor(WHITE); 
        LCD.setCursor(25, 5);  LCD.print("IntelliSys");
        LCD.setCursor(20, 20); LCD.print("Temperature &");
        LCD.setCursor(30, 35); LCD.print("Humidity");


  LCD.setTextSize(1);
  LCD.setTextColor(WHITE);  LCD.setCursor(40, 50);  LCD.println("Sensor");

  //LCD.setFont(&FreeSansBold24pt7b); // FreeSerifBold24pt7b
 // LCD.setTextSize(1);  LCD.setTextColor(WHITE);  LCD.setCursor(35, 55);  LCD.println(temp);
  LCD.display(); 

delay(2000);


pinMode(red, OUTPUT); 
    for(int i=0; i<3; i++){
      digitalWrite(red, HIGH); digitalWrite(buzz, HIGH);
        delay(50);
      digitalWrite(red, LOW); digitalWrite(buzz, LOW);
        delay(50);
    }
delay(500);

pinMode(green, OUTPUT); 
    for(int i = 0; i<5; i++){
        digitalWrite(green, HIGH); digitalWrite(on_board, HIGH);
        delay(50);
        digitalWrite(green, LOW); digitalWrite(on_board, LOW);
        delay(50);
      }
   
 pinMode(reset_button, INPUT); 
// BOOT SCREEN
// intellisys logo
//config...'
//superfast loading bar if necessary
// 3 sec boot

    int loader = 0; int pos = 0;

      // Set ESP32 as a Wi-Fi Station
      //WiFi.begin();
      WiFi.mode(WIFI_STA);

      // Initilize ESP-NOW
      if (esp_now_init() != ESP_OK) {
          // Serial.println("Error initializing ESP-NOW");
            return;
      }

      // Register SEND callback
      esp_now_register_send_cb(OnDataSent);
      
      // Register peer
      memcpy(peerInfo.peer_addr, broadcastAddress, 6);
      peerInfo.channel = 0;  
      peerInfo.encrypt = false;
      
      // Add peer        
      if (esp_now_add_peer(&peerInfo) != ESP_OK){
      // Serial.println("Failed to add peer");
        return;
      }


 //setCpuFrequencyMhz(240); // makes the device to draw 96mA @12V [1.1W]
 //  setCpuFrequencyMhz(80); //makes the device consume 80mA @12V [0.9W]
 delay(500);

    scan_temp();
  //Serial.println("Done Booting!");   Serial.println();
    delay(2000);
    currentScreen = 2;
 //CLOSE SETUP
}


    float temp_now = 0.00;
    float humi_now = 0.00;


unsigned long read_now = 0; // 0 to 4E9 (4Billion)
bool can_send = true;
bool swing = true;/// default back to normal 10 secs after button press
void loop() { // --- SENSOR HAS ONLY 3 TASKS: Scanning Soil Moisture, Updating display and wireless connectivity

now_now = esp_timer_get_time()/1000ULL;
         
    if((now_now - prev) >= 100){ // since every loop's cycle speed drops; use >= instead of == 1000

         buttonScan();
         read_now++;

    // pass through this only once every 2 seconds
        //if(read_now != 0 &&){
      if(read_now%100 == 0){ //since the sensor is too slow: 2s latency, request read once every 10 seconds
          scan_temp();
          update_display();
          //swing = !swing; 
         
        }

      if(read_now%200 == 0){  //transmit once every 10 seconds
          if(can_send) send_as_JSON();

               if(currentScreen == 1) currentScreen = 2;
          else if(currentScreen == 2) currentScreen = 1;
          update_display();
           Serial.print("Current Time: "); Serial.println(now_now/1000);
        }

/*
        if(read_now%6000 == 0){ // flip screen once every 300seconds // 5mins
               if(currentScreen == 1) currentScreen = 2;
          else if(currentScreen == 2) currentScreen = 1;
        }
 */     
        
        prev = now_now;
    // digitalWrite(on_board, LOW);
    }



}//loop






bool pressed = false; bool firstPass = false;

uint8_t resetted = 0;
  bool lock_press = false, passed_yet = false;
  unsigned long start_counting = 0;

uint16_t buttoncount = 0;
bool btn_locked = false; bool long_locked = false; 

void buttonScan(){
    resetted = digitalRead(reset_button);    // Serial.print("Button: "); Serial.println(resetted);   

            if(!resetted) { if(!firstPass) {pressed = true; start_counting = now_now; firstPass = true; }} //start_counting = now_now;
            else { pressed = false;  firstPass = false; start_counting = now_now; buttoncount = 0; btn_locked = false; long_locked = false; }


            if(pressed){  
               
                    if((now_now - start_counting) >= 50 && (now_now - start_counting) < 300) { // short press
                             if(!btn_locked){ btn_locked = true; 
                                 // Serial.print("Short Press registered"); // Serial.println(buttoncount); 
                                 // short_press();
                                 digitalWrite(buzz, HIGH);   delay(50); digitalWrite(buzz, LOW);
                                 if(currentScreen < 2) currentScreen++;
                                 else  currentScreen = 1;
                                 update_display();

                             }
                            
                    }

                    else if((now_now - start_counting) >= 1500) { //long Press
                           if(!long_locked){ long_locked = true;
                                // Serial.print("\tLong Press "); //Serial.println(buttoncount); 
                               //  long_press();
                              if(can_send) send_as_JSON(); delay(2000); // to prevent cross sendings
                             // last_backlight_ = time_now;
                            //  digitalWrite(buzz, HIGH);   delay(50); digitalWrite(buzz, LOW);
                           } 
                            
                    }
            }

}




void update_display(){ //Serial.println("update disp summoned!");
  
 
  if(currentScreen == 1){ // tempScreen();
     
        LCD.clearDisplay();
        LCD.setFont();
        LCD.setTextSize(2);
        LCD.setTextColor(WHITE); 
       // LCD.setCursor(28, 4); LCD.print("Temperature"); 
         LCD.setCursor(0, 0); LCD.print("Temperatur"); 
        
        LCD.setTextSize(2);
        LCD.setTextColor(WHITE);  LCD.setCursor(90, 22);  LCD.println("o");
      
        LCD.setFont(&FreeSansBold24pt7b); // FreeSerifBold24pt7b
        LCD.setTextSize(1);  LCD.setTextColor(WHITE);  LCD.setCursor(35, 60);  LCD.println(temp_char);
        LCD.display(); 
  }

  else if(currentScreen == 2) { // humiScreen();

      
      LCD.clearDisplay();
      LCD.setFont();
      LCD.setTextSize(2);
      LCD.setTextColor(WHITE); 
      //LCD.setCursor(38, 4); LCD.print("Humidity");
        LCD.setCursor(18, 0); LCD.print("Humidity");
      
      
      LCD.setTextSize(2);
      LCD.setTextColor(WHITE);  LCD.setCursor(90, 40);  LCD.println("%");
    
      LCD.setFont(&FreeSansBold24pt7b); // FreeSerifBold24pt7b
      LCD.setTextSize(1);  LCD.setTextColor(WHITE);  LCD.setCursor(35, 60);  LCD.println(humi_char);
      LCD.display(); 
  }
}




    float reading_duration = 0.00;
    int samples = 0;



//TASK 1
void scan_temp(){

     digitalWrite(green, HIGH); 
     unsigned long start_reading = 0;
     unsigned long stop_reading = 0;
      
      start_reading = micros();

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  temp_now = sensor.readTemperature();
  // Read temperature as Celsius (the default)
  humi_now = sensor.readHumidity();

  
     stop_reading = micros();
     reading_duration = float(stop_reading - start_reading)/1000.0; // by a million
     

/*
  Serial.println();
  Serial.print("Temperature: ");  Serial.print(temp_now);  Serial.println("°C ");

  Serial.print("Humidity: "); Serial.print(humi_now); Serial.println("%");
  
  Serial.print("Read Duration (in ms): "); Serial.println(reading_duration, 4); Serial.println();

*/

delay(50);
digitalWrite(green, LOW);
can_send = true;
    //save_in_24_hour_array[]
    //5 saves over 24 hours is 120 floats
    //get highest, lowest and average temp of the day from the float of 120 and save it to EEPROM
 
 
    // Check if any reads failed and exit early (to try again).
  if (isnan(temp_now) || isnan(humi_now)) {
   //   Serial.println("Failed to read from DHT sensor!");
      can_send = false;
      strcpy(temp_char, "!");
      strcpy(humi_char, "!");
      
      return;
  }

   dtostrf(temp_now, 0, 0, temp_char); 
   dtostrf(humi_now, 0, 0, humi_char);

 }

//TASK 2
//display time if we must



char hr_str[5]; char min_str[5]; char sec_str[5]; char day_str[10];  char zero_holder[3] = "0"; char bucket[4];

unsigned long last_backup_time = 0;


uint32_t sends = 0; // 0 TO 4BILLION
char transmissions[12] = ""; //10 digit value: 1Billion sends | many years

char appender[3] = "1";

void send_as_JSON(){ // bundle of JOY
   //strcat(SensorID, appender);

     JSON_data["SensorID2"] = SensorID;
     JSON_data["Moisture2"] = humi_char; //Humidity1
     JSON_data["Temperature2"] =  temp_char;
     JSON_data["Delivereds2"] =  transmissions;
   //  JSON_data["Voltage1"] = batt_volts;


        // char output[200];
    serializeJson(JSON_data, senderObj.sendable_data_bundle);  //Serial.println(output);

        
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*) &senderObj, sizeof(senderObj));

    if(result == ESP_OK)  { Serial.print("\t"); Serial.print(senderObj.sendable_data_bundle); Serial.println(" Sent!"); }
    else Serial.println("Send Failed!");

      digitalWrite(buzz, HIGH); delay(50); digitalWrite(buzz, LOW);


}



//TASK 3
void send_by_ESPNW(){ //send every 10 seconds
//Sending Round
  
  digitalWrite(red, HIGH); delay(100); digitalWrite(buzz,  HIGH);
     
//PREPARE SENDABLE PACKETS
  senderObj2.temperature = temp_now;
  senderObj2.humidity = humi_now;
  senderObj2.sampling_duration = reading_duration;
  senderObj2.uptime = now_now; //intra-bed deviation expected

      
  // Send message via ESP-NOW

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*) &senderObj2, sizeof(senderObj2));
    
    if(result == ESP_OK)  {/*Serial.print(senderObj.ID); Serial.println(" has Sent packet successfully!");*/}
    
    //else  Serial.println("Error Sending!");
     delay(50);
    digitalWrite(red, LOW); digitalWrite(buzz,  LOW);

}

// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  //  Serial.print("\r\nLast Packet Send Status:\t");
   // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Successfully Delivered!" : "Failed 2 Delivery!");
    
      if(status == ESP_NOW_SEND_SUCCESS) {  Serial.print("\tSuccessfully Delivered to: "); Serial.println(broadcastAddress_str);   sends++; }
      if(status != ESP_NOW_SEND_SUCCESS) { Serial.print("\t\tFailed 2 Deliver!");; }

   ltoa(sends, transmissions, 10); 

  
}
