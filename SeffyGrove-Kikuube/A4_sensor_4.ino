/* XODDOCODE 20TH AUGUST, 2025
  * WIRELESS TEMP AND HUMIDITY SENSING CODE 
  
  * Share reading once every 10 seconds
  * Display only current temp on SCREEN in large font// backlit or in idle
  * If Reading is 0, flash XX on screen
  * if is nan... flash !!
  */
#include <Arduino.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBold24pt7b.h> // FreeSerifBold24pt7b
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

StaticJsonDocument<200> JSON_data;
//JsonDocument JSON_data;


uint8_t green = 12;
uint8_t red   = 13;
uint8_t on_board = 4;


uint8_t buzz = 14;


 //   const char SensorID[6] = "T_1"; char ID[2] = "1";
//  const char SensorID[6] = "T_2"; char ID[2] = "2";
//  const char SensorID[6] = "T_3"; char ID[2] = "3";
  const char SensorID[6] = "T_4"; char ID[2] = "4";
  // const char SensorID[6] = "T_5";char ID[2] = "5";
 // const char SensorID[6] = "T_6"; char ID[2] = "6";
///1C:69:20:A3:6B:CC

//   uint8_t broadcastAddress[] = {0xCC, 0xDB, 0xA7, 0x3E, 0x82, 0x80}; //  NIC for MUARIK DRYER
//   uint8_t broadcastAddress[] = {0xF4, 0x65, 0x0B, 0xE8, 0xC6, 0x30}; //  NIC for MUARIK irrikit
     uint8_t broadcastAddress[] = {0x1C, 0x69, 0x20, 0xA3, 0x6B, 0xCC}; //  NIC for KIKUUBE DRYER AUTO
  
char broadcastAddress_str[50] = "SEFFY-GROVE KIKUUBE";

char temp_char[5] = "xx";
char humi_char[5] = "xx";

uint32_t sends = 0; // 0 TO 4BILLION
char transmissions[12] = ""; //10 digit value: 1Billion sends | many years

char appender[3] = "1";

bool successfully_delivered = false;



typedef struct sensorData{ // all stringified
      char sendable_data_bundle[200] = "";
   }  sensorData; 

sensorData senderObj;


#define DHTPIN 13 //27     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

  #define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321


DHT sensor(DHTPIN, DHTTYPE);
 uint8_t reset_button = 39;


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 LCD(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


//int backlight = 21;  bool backlit = false;
uint8_t currentScreen = 0;



   




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

/*
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  //  Serial.print("\r\nLast Packet Send Status:\t");
   // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Successfully Delivered!" : "Failed 2 Delivery!");
    
      if(status == ESP_NOW_SEND_SUCCESS) { successfully_delivered = true;    sends++; }
      if(status != ESP_NOW_SEND_SUCCESS) { successfully_delivered = false; ; }

}
*/
/*
In older ESP32 Arduino cores, the ESP-NOW send callback signature was:
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

But in ESP-IDF v5 (which your error log shows you’re on), it has changed to:
void OnDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status);
*/
// Old (before ESP-IDF v5)
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

// New (ESP-IDF v5+)
void OnDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status) {
  /*
    // You can still access the peer MAC address like this:
    char macStr[18];
    snprintf(macStr, sizeof(macStr),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr->broadcastAddress[0], mac_addr->broadcastAddress[1], mac_addr->broadcastAddress[2],
             mac_addr->broadcastAddress[3], mac_addr->broadcastAddress[4], mac_addr->broadcastAddress[5]);

    Serial.print("Last Packet Sent to: ");
    Serial.println(macStr);
*/
    if (status == ESP_NOW_SEND_SUCCESS) {
        successfully_delivered = true;    sends++;
    } else {
        successfully_delivered = false;
    }
}



unsigned long long time_spent_ON = 0; 


unsigned long started = 0;
int64_t now_now = 0, prev = 0, start_sensing = 0;

uint8_t hr_int = 0, min_int = 0, sec_int = 0;
uint8_t day = 0, month = 0, year = 0;

    float temp_now = 0.00;
    float humi_now = 0.00;



// { "SensorIDX":"T_X", "MoistureX":XX.XX, "TemperatureX":YY.YY, "DeliveredsX":"X"}
void send_as_JSON(){ // bundle of JOY
   //strcat(SensorID, appender);

     JSON_data["SensorID4"] = SensorID;
     JSON_data["Moisture4"] = temp_now; //humi_char; //Humidity1
     JSON_data["Temperature4"] = humi_now;  //temp_char;
     JSON_data["Delivereds4"] =  transmissions;
   //  JSON_data["Voltage3\1"] = batt_volts;


        // char output[200];
    serializeJson(JSON_data, senderObj.sendable_data_bundle);  //Serial.println(output);

        
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*) &senderObj, sizeof(senderObj));

    if(result == ESP_OK)  { Serial.print("\t"); Serial.print(senderObj.sendable_data_bundle); Serial.println(" Sent!"); }
    else Serial.println("Send Failed!");

      digitalWrite(buzz, HIGH); delay(50); digitalWrite(buzz, LOW);

    if(successfully_delivered) {
      Serial.print("\tSuccessfully Delivered to: "); Serial.println(broadcastAddress_str);
      ltoa(sends, transmissions, 10);
} else Serial.print("\t\tFailed 2 Deliver!");
    


}


void setup() { delay(500);
  // Serial.begin(115200);
  
  pinMode(on_board, OUTPUT); 
  pinMode(buzz, OUTPUT); digitalWrite(buzz, HIGH); delay(50); digitalWrite(buzz, LOW);
  //pinMode(backlight, OUTPUT); digitalWrite(backlight, HIGH); backlit = true;


  sensor.begin();

  
  delay(50);


if(!LCD.begin(SSD1306_SWITCHCAPVCC, 0x3C)){ // Address 0x3D for 128x64
      Serial.println(F("SSD1306 allocation failed"));
      //for(;;);
  }

  else{
  

  LCD.clearDisplay();

  LCD.setFont();
  LCD.setTextSize(1);
  LCD.setTextColor(WHITE); 
        LCD.setCursor(30, 5);  LCD.print("IntelliSys"); // or show our logo
        LCD.setCursor(24, 20); LCD.print("Temperature &");
        LCD.setCursor(34, 35); LCD.print("Humidity");


  LCD.setTextSize(1);
  LCD.setTextColor(WHITE);  LCD.setCursor(40, 50);  LCD.println("Sensor");

  //LCD.setFont(&FreeSansBold24pt7b); // FreeSerifBold24pt7b
 // LCD.setTextSize(1);  LCD.setTextColor(WHITE);  LCD.setCursor(35, 55);  LCD.println(temp);
  LCD.display(); 

  delay(2000);

  LCD.clearDisplay();
  LCD.setCursor(10, 20); LCD.print(broadcastAddress_str);
  LCD.display(); 
  }

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
      WiFi.disconnect(); // to ensure the ESP32 is not connected to any router. ESP-NOW works directly between peers (MAC-to-MAC) and doesn’t need an active WiFi connection.
       //If your ESP32 tries to stay connected to a router (from previous boot), it can sometimes interfere with ESP-NOW transmissions, so WiFi.disconnect(); was used as a cleanup step.
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
    delay(500);
    currentScreen = 2;
 //CLOSE SETUP
}



unsigned long read_now = 0; // 0 to 4E9 (4Billion)
bool can_send = true;
bool swing = true;/// default back to normal 10 secs after button press
void loop() { // --- SENSOR HAS ONLY 3 TASKS: Scanning Soil Moisture, Updating display and wireless connectivity

  now_now = esp_timer_get_time()/1000000ULL; //  each second
         // buttonHandler();
    if((now_now - prev) >= 10){ // once every 10 seconds

          scan_temp(); //look for temperature and humidity
          update_display();
          if(can_send) send_as_JSON();

          if(currentScreen == 1) currentScreen = 2;
          else if(currentScreen == 2) currentScreen = 1;

          //swing = !swing; 
         prev = now_now;
        }


}//loop







bool is_home = false;

void update_display(){ //Serial.println("update disp summoned!");

 // Only redraw when values change (compare with old values). 
 //the statics
 if(!is_home){

 }

//the dynamics
  if(currentScreen == 1){ // tempScreen();
     
        LCD.clearDisplay();
        LCD.setTextSize(1);
        LCD.setFont(&FreeSans9pt7b);
       // 
        LCD.setTextColor(WHITE); 
       // LCD.setCursor(28, 4); LCD.print("Temperature"); 
         LCD.setCursor(12, 13); LCD.print("Temperature"); 
        
       //  LCD.setFont(); LCD.setTextSize(2);
        LCD.setTextColor(WHITE);  LCD.setCursor(84, 32);  LCD.println("o");
      
        LCD.setFont(&FreeSansBold24pt7b); // FreeSerifBold24pt7b
        LCD.setTextSize(1);  LCD.setTextColor(WHITE);  LCD.setCursor(32, 60);  LCD.println(temp_char);
        if(now_now <= 1*60*60) { // if has been ON for an HR
            LCD.setFont(); LCD.setTextSize(1); LCD.setCursor(110,50); LCD.print(ID);
        }
        
        LCD.display(); 
  }

  else if(currentScreen == 2) { // humiScreen();      
      LCD.clearDisplay();
      LCD.setTextSize(1);
        LCD.setFont(&FreeSans9pt7b);
       // 
        LCD.setTextColor(WHITE); 
         LCD.setCursor(30, 13); LCD.print("Humidity"); 
      
      
      //LCD.setTextSize(2);
      LCD.setTextColor(WHITE);  LCD.setCursor(84, 40);  LCD.println("%");
    
      LCD.setFont(&FreeSansBold24pt7b); // FreeSerifBold24pt7b
      LCD.setTextSize(1);  LCD.setTextColor(WHITE);  LCD.setCursor(32, 60);  LCD.println(humi_char);
      if(now_now <= 1*60*60) { // if has been ON for an HR
        LCD.setFont(); LCD.setCursor(110,50); LCD.print(ID);
      }
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
        
    delay(50);
    digitalWrite(green, LOW);

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

  else {
      can_send = true;
      dtostrf(temp_now, 0, 0, temp_char); 
      dtostrf(humi_now, 0, 0, humi_char);
  
  }


      /*
        Serial.println();
        Serial.print("Temperature: ");  Serial.print(temp_now);  Serial.println("°C ");

        Serial.print("Humidity: "); Serial.print(humi_now); Serial.println("%");
        
        Serial.print("Read Duration (in ms): "); Serial.println(reading_duration, 4); Serial.println();

      */


 }

//TASK 2
//SEND the packet


char hr_str[5]; char min_str[5]; char sec_str[5]; char day_str[10];  char zero_holder[3] = "0"; char bucket[4];

unsigned long last_backup_time = 0;



/*
//TASK 3
void send_as_floats(){ //send every 10 seconds
//Sending Round
  
  digitalWrite(red, HIGH); delay(100); digitalWrite(buzz,  HIGH);
     
//PREPARE SENDABLE PACKETS
  senderObj2.temperature = temp_now;
  senderObj2.humidity = humi_now;
  senderObj2.sampling_duration = reading_duration;
  senderObj2.uptime = now_now; //intra-bed deviation expected

      
  // Send message via ESP-NOW

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*) &senderObj2, sizeof(senderObj2));
    
    if(result == ESP_OK)  {/*Serial.print(senderObj.ID); Serial.println(" has Sent packet successfully!");*}
    
    //else  Serial.println("Error Sending!");
   //  delay(50);
  //  digitalWrite(red, LOW); digitalWrite(buzz,  LOW);

//}

*/

bool pressed = false; bool firstPass = false;

uint8_t resetted = 0;
bool passed_yet = false;
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

// ===== BUTTONS & BUZZER =====
#define BUTTON_PIN 36

bool buttonState = false;
unsigned long pressStart = 0;
bool longPressHandled = false;
const unsigned long longPressTime = 2000; // 2s

void buttonHandler() {
  bool pressed = (digitalRead(BUTTON_PIN) == LOW);

  if (pressed && !buttonState) {
    // button just pressed
    pressStart = now_now;
    longPressHandled = false;
    buttonState = true;
  } 
  else if (!pressed && buttonState) {
    // button just released
    unsigned long pressDuration = now_now - pressStart;
    if (pressDuration < longPressTime) {
      shortPressAction();
    }
    buttonState = false;
  } 
  else if (pressed && buttonState && !longPressHandled) {
    // still holding
    if (now_now - pressStart >= longPressTime) {
      longPressAction();
      longPressHandled = true;
    }
  }
}

// ===== ACTIONS =====
void shortPressAction() {
  Serial.println("Short press → toggle screen");
  beep(1);
  // e.g. cycle display mode
}

void longPressAction() {
  Serial.println("Long press → send JSON");
  beep(2);
  send_as_JSON();
}

// ===== BUZZER =====
#define BUZZ 25

int beepCount = 0;          // total beeps requested
int beepDone = 0;           // beeps completed
unsigned long beepTimer = 0;
bool buzzerOn = false;
const unsigned long beepDuration = 100;   // ms ON
const unsigned long beepGap = 100;        // ms OFF

void beep(int times) {
  beepCount = times;
  beepDone = 0;
  buzzerOn = false;
  beepTimer = now_now;
}

void handleBeep() {
  if (beepDone >= beepCount) return; // nothing to do

  unsigned long now = now_now;
  if (!buzzerOn && now - beepTimer >= beepGap) {
    // turn ON buzzer
    digitalWrite(buzz, HIGH);
    buzzerOn = true;
    beepTimer = now;
  } 
  else if (buzzerOn && now - beepTimer >= beepDuration) {
    // turn OFF buzzer
    digitalWrite(buzz, LOW);
    buzzerOn = false;
    beepTimer = now;
    beepDone++;
  }
}

