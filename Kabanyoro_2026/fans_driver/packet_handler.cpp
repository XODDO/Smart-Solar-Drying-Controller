#include "packet_handler.h"
#include <esp_timer.h>

// Initialize static instance pointer
PacketHandler* PacketHandler::_instance = nullptr;

// External RTC time string
extern TimerKeeper sawa;

extern Buzzer buzzer;  // Adjust type as needed

PacketHandler::PacketHandler() {
    _packet_loss_counter = 0;
    _total_packets_received = 0;
    _jsonPacketQueue = nullptr;
    _instance = this;
}

bool PacketHandler::begin() {
    // Set WiFi mode
    WiFi.mode(WIFI_STA);
    
    // Initialize ESP-NOW
    if(esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return false;
    }
    
    Serial.println("ESPNOW successfully initialized");
    
    // Register callback function
    esp_err_t result = esp_now_register_recv_cb(onDataReceived);
    if(result == ESP_OK) {
        Serial.println("Callback successfully set!");
    } else {
        Serial.println("Failed to set callback!");
        return false;
    }
    
    delay(1000);
    
    // Create queue for JSON packets
    _jsonPacketQueue = xQueueCreate(QUEUE_SIZE, sizeof(QueuedJsonPacket));
    if (_jsonPacketQueue == NULL) {
        Serial.println("[ERROR] Failed to create queue!");
        return false;
    }
    
    // Check free memory before task creation
    Serial.printf("Heap Before Task: Free=%u bytes, Min=%u bytes\n",
                 esp_get_free_heap_size(),
                 esp_get_minimum_free_heap_size());
    
    // Create task on Core 0
    BaseType_t result_task = xTaskCreatePinnedToCore(
        packetHandlerTask,
        "PacketHandler",
        PACKET_STACK_SIZE,
        this,  // Pass instance pointer as parameter
        PACKET_TASK_PRIORITY,
        NULL,
        PACKET_TASK_CORE
    );
    
    if (result_task != pdPASS) {
        Serial.println("[ERROR] Failed to create packet handler task!");
        Serial.printf("Reason: %d\n", result_task);
        return false;
    } else {
        Serial.println("[OK] Packet handler task created successfully");
    }
    
    Serial.printf("Heap After Task: Free=%u bytes\n", esp_get_free_heap_size());
    
    return true;
}


// Static ISR callback
void PacketHandler::onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    if (_instance) {
        QueuedJsonPacket jsonPacket;
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        
        if (len <= sizeof(jsonPacket.data)) {
            memcpy(jsonPacket.data, data, len);
            jsonPacket.length = len;
            
            if (xQueueSendFromISR(_instance->_jsonPacketQueue, &jsonPacket, &higherPriorityTaskWoken) != pdTRUE) {
                _instance->_packet_loss_counter++;
            }
            
            if (higherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
            }
        } else {
            _instance->_packet_loss_counter++;
        }
    }
}

// Static task function
void PacketHandler::packetHandlerTask(void *pvParams) {
    PacketHandler* handler = (PacketHandler*)pvParams;
    QueuedJsonPacket jsonPacket;
    ReceivedPacket packet;
    
    while (true) {
        if (xQueueReceive(handler->_jsonPacketQueue, &jsonPacket, portMAX_DELAY) == pdTRUE) {
            handler->_total_packets_received++;
            
            // Parse JSON and fill packet struct
            handler->parseJsonPacket(jsonPacket.data, jsonPacket.length, &packet);
            
            // Extract readings to global variables
            handler->extractReadingsFromPacket(&packet);
            
            // Optional: Beep for testing
             buzzer.beep(1, 50, 0);
        }
    }
}

void PacketHandler::parseJsonPacket(const uint8_t *data, int len, ReceivedPacket *packet) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, data, len);
    
    if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
    }
    
    // Extract sensor number from S_ID string (e.g., "T_1" -> 1)
    const char* sensor_id_str = doc["S_ID"] | "T_0";
    packet->sensor_id = extractSensorNumber(sensor_id_str);
    
    // Extract all values
    packet->temperature = doc["Temp"] | NAN;
    packet->humidity = doc["Humi"] | NAN;
    packet->pressure = doc["Pre"] | NAN;
    packet->elevation = doc["Elv"] | NAN;
    packet->heat_index = doc["Heat_ind"] | NAN;
    packet->time_on_in_ms = doc["UpTime"] | 0;
    
    // Copy string fields
    strlcpy(packet->CPU_Freq, doc["CPU"] | "0", sizeof(packet->CPU_Freq));
    strlcpy(packet->transmissions, doc["Sends"] | "0", sizeof(packet->transmissions));
    
    // Set timestamp using RTC time
    packet->last_seen_ms = esp_timer_get_time()/1000ULL;  // esp_timer_get_time millisecondseconds since boot
    strlcpy(packet->last_seen, sawa.SystemTime, sizeof(packet->last_seen));
}

int PacketHandler::extractSensorNumber(const char *identity) {
    if (identity == NULL) return 0;
    
    int num = 0;
    const char* p = identity;
    
    // Skip any non-digit characters
    while (*p && !isdigit(*p)) p++;
    
    // Extract the number
    if (*p) {
        num = atoi(p);
    }
    
    // Validate range
    if (num < 1 || num > 12) {
        Serial.printf("Invalid sensor number: %d from '%s'\n", num, identity);
        return 0;
    }
    
    return num;
}

void PacketHandler::extractReadingsFromPacket(ReceivedPacket *packet) {
    if (packet == NULL) return;
    
    int sensor_id = packet->sensor_id;
    
    switch(sensor_id) {
        case 1:
            sensor_1_temp = packet->temperature;
            sensor_1_humidity = packet->humidity;
            sensor_1_pressure = packet->pressure;
            sensor_1_elevation = packet->elevation;
            sensor_1_h_index = packet->heat_index;
            sensor_1_running_time_ms = packet->time_on_in_ms;
            sensor_1_last_seen_ms = esp_timer_get_time()/1000ULL;  // Current time since boot
            strncpy(sensor_1_CPU_freq, packet->CPU_Freq, sizeof(sensor_1_CPU_freq) - 1);
            sensor_1_CPU_freq[sizeof(sensor_1_CPU_freq) - 1] = '\0';
            strncpy(sensor_1_last_seen, sawa.SystemTime, sizeof(sensor_1_last_seen) - 1);
            sensor_1_last_seen[sizeof(sensor_1_last_seen) - 1] = '\0';
            break;
            
        case 2:
            sensor_2_temp = packet->temperature;
            sensor_2_humidity = packet->humidity;
            sensor_2_pressure = packet->pressure;
            sensor_2_elevation = packet->elevation;
            sensor_2_h_index = packet->heat_index;
            sensor_2_running_time_ms = packet->time_on_in_ms;
            sensor_2_last_seen_ms = esp_timer_get_time()/1000ULL;
            strncpy(sensor_2_CPU_freq, packet->CPU_Freq, sizeof(sensor_2_CPU_freq) - 1);
            sensor_2_CPU_freq[sizeof(sensor_2_CPU_freq) - 1] = '\0';
            strncpy(sensor_2_last_seen, sawa.SystemTime, sizeof(sensor_2_last_seen) - 1);
            sensor_2_last_seen[sizeof(sensor_2_last_seen) - 1] = '\0';
            break;
            
        case 3:
            sensor_3_temp = packet->temperature;
            sensor_3_humidity = packet->humidity;
            sensor_3_pressure = packet->pressure;
            sensor_3_elevation = packet->elevation;
            sensor_3_h_index = packet->heat_index;
            sensor_3_running_time_ms = packet->time_on_in_ms;
            sensor_3_last_seen_ms = esp_timer_get_time()/1000ULL;
            strncpy(sensor_3_CPU_freq, packet->CPU_Freq, sizeof(sensor_3_CPU_freq) - 1);
            sensor_3_CPU_freq[sizeof(sensor_3_CPU_freq) - 1] = '\0';
            strncpy(sensor_3_last_seen, sawa.SystemTime, sizeof(sensor_3_last_seen) - 1);
            sensor_3_last_seen[sizeof(sensor_3_last_seen) - 1] = '\0';
            break;
            
        case 4:
            sensor_4_temp = packet->temperature;
            sensor_4_humidity = packet->humidity;
            sensor_4_pressure = packet->pressure;
            sensor_4_elevation = packet->elevation;
            sensor_4_h_index = packet->heat_index;
            sensor_4_running_time_ms = packet->time_on_in_ms;
            sensor_4_last_seen_ms = esp_timer_get_time()/1000ULL;
            strncpy(sensor_4_CPU_freq, packet->CPU_Freq, sizeof(sensor_4_CPU_freq) - 1);
            sensor_4_CPU_freq[sizeof(sensor_4_CPU_freq) - 1] = '\0';
            strncpy(sensor_4_last_seen, sawa.SystemTime, sizeof(sensor_4_last_seen) - 1);
            sensor_4_last_seen[sizeof(sensor_4_last_seen) - 1] = '\0';
            break;
            
        case 5:
            sensor_5_temp = packet->temperature;
            sensor_5_humidity = packet->humidity;
            sensor_5_pressure = packet->pressure;
            sensor_5_elevation = packet->elevation;
            sensor_5_h_index = packet->heat_index;
            sensor_5_running_time_ms = packet->time_on_in_ms;
            sensor_5_last_seen_ms = esp_timer_get_time()/1000ULL;
            strncpy(sensor_5_CPU_freq, packet->CPU_Freq, sizeof(sensor_5_CPU_freq) - 1);
            sensor_5_CPU_freq[sizeof(sensor_5_CPU_freq) - 1] = '\0';
            strncpy(sensor_5_last_seen, sawa.SystemTime, sizeof(sensor_5_last_seen) - 1);
            sensor_5_last_seen[sizeof(sensor_5_last_seen) - 1] = '\0';
            break;
            
        case 6:
            sensor_6_temp = packet->temperature;
            sensor_6_humidity = packet->humidity;
            sensor_6_pressure = packet->pressure;
            sensor_6_elevation = packet->elevation;
            sensor_6_h_index = packet->heat_index;
            sensor_6_running_time_ms = packet->time_on_in_ms;
            sensor_6_last_seen_ms = esp_timer_get_time()/1000ULL;
            strncpy(sensor_6_CPU_freq, packet->CPU_Freq, sizeof(sensor_6_CPU_freq) - 1);
            sensor_6_CPU_freq[sizeof(sensor_6_CPU_freq) - 1] = '\0';
            strncpy(sensor_6_last_seen, sawa.SystemTime, sizeof(sensor_6_last_seen) - 1);
            sensor_6_last_seen[sizeof(sensor_6_last_seen) - 1] = '\0';
            break;
            
        case 7:
            sensor_7_temp = packet->temperature;
            sensor_7_humidity = packet->humidity;
            sensor_7_pressure = packet->pressure;
            sensor_7_elevation = packet->elevation;
            sensor_7_h_index = packet->heat_index;
            sensor_7_running_time_ms = packet->time_on_in_ms;
            sensor_7_last_seen_ms = esp_timer_get_time()/1000ULL;
            strncpy(sensor_7_CPU_freq, packet->CPU_Freq, sizeof(sensor_7_CPU_freq) - 1);
            sensor_7_CPU_freq[sizeof(sensor_7_CPU_freq) - 1] = '\0';
            strncpy(sensor_7_last_seen, sawa.SystemTime, sizeof(sensor_7_last_seen) - 1);
            sensor_7_last_seen[sizeof(sensor_7_last_seen) - 1] = '\0';
            break;
            
        case 8:
            sensor_8_temp = packet->temperature;
            sensor_8_humidity = packet->humidity;
            sensor_8_pressure = packet->pressure;
            sensor_8_elevation = packet->elevation;
            sensor_8_h_index = packet->heat_index;
            sensor_8_running_time_ms = packet->time_on_in_ms;
            sensor_8_last_seen_ms = esp_timer_get_time()/1000ULL;
            strncpy(sensor_8_CPU_freq, packet->CPU_Freq, sizeof(sensor_8_CPU_freq) - 1);
            sensor_8_CPU_freq[sizeof(sensor_8_CPU_freq) - 1] = '\0';
            strncpy(sensor_8_last_seen, sawa.SystemTime, sizeof(sensor_8_last_seen) - 1);
            sensor_8_last_seen[sizeof(sensor_8_last_seen) - 1] = '\0';
            break;
            
        case 9:
            sensor_9_temp = packet->temperature;
            sensor_9_humidity = packet->humidity;
            sensor_9_pressure = packet->pressure;
            sensor_9_elevation = packet->elevation;
            sensor_9_h_index = packet->heat_index;
            sensor_9_running_time_ms = packet->time_on_in_ms;
            sensor_9_last_seen_ms = esp_timer_get_time()/1000ULL;
            strncpy(sensor_9_CPU_freq, packet->CPU_Freq, sizeof(sensor_9_CPU_freq) - 1);
            sensor_9_CPU_freq[sizeof(sensor_9_CPU_freq) - 1] = '\0';
            strncpy(sensor_9_last_seen, sawa.SystemTime, sizeof(sensor_9_last_seen) - 1);
            sensor_9_last_seen[sizeof(sensor_9_last_seen) - 1] = '\0';
            break;
            
        case 10:
            sensor_10_temp = packet->temperature;
            sensor_10_humidity = packet->humidity;
            sensor_10_pressure = packet->pressure;
            sensor_10_elevation = packet->elevation;
            sensor_10_h_index = packet->heat_index;
            sensor_10_running_time_ms = packet->time_on_in_ms;
            sensor_10_last_seen_ms = esp_timer_get_time()/1000ULL;
            strncpy(sensor_10_CPU_freq, packet->CPU_Freq, sizeof(sensor_10_CPU_freq) - 1);
            sensor_10_CPU_freq[sizeof(sensor_10_CPU_freq) - 1] = '\0';
            strncpy(sensor_10_last_seen, sawa.SystemTime, sizeof(sensor_10_last_seen) - 1);
            sensor_10_last_seen[sizeof(sensor_10_last_seen) - 1] = '\0';
            break;
            
        case 11:
            sensor_11_temp = packet->temperature;
            sensor_11_humidity = packet->humidity;
            sensor_11_pressure = packet->pressure;
            sensor_11_elevation = packet->elevation;
            sensor_11_h_index = packet->heat_index;
            sensor_11_running_time_ms = packet->time_on_in_ms;
            sensor_11_last_seen_ms = esp_timer_get_time()/1000ULL;
            strncpy(sensor_11_CPU_freq, packet->CPU_Freq, sizeof(sensor_11_CPU_freq) - 1);
            sensor_11_CPU_freq[sizeof(sensor_11_CPU_freq) - 1] = '\0';
            strncpy(sensor_11_last_seen, sawa.SystemTime, sizeof(sensor_11_last_seen) - 1);
            sensor_11_last_seen[sizeof(sensor_11_last_seen) - 1] = '\0';
            break;
            
        case 12:
            sensor_12_temp = packet->temperature;
            sensor_12_humidity = packet->humidity;
            sensor_12_pressure = packet->pressure;
            sensor_12_elevation = packet->elevation;
            sensor_12_h_index = packet->heat_index;
            sensor_12_running_time_ms = packet->time_on_in_ms;
            sensor_12_last_seen_ms = esp_timer_get_time()/1000ULL;
            strncpy(sensor_12_CPU_freq, packet->CPU_Freq, sizeof(sensor_12_CPU_freq) - 1);
            sensor_12_CPU_freq[sizeof(sensor_12_CPU_freq) - 1] = '\0';
            strncpy(sensor_12_last_seen, sawa.SystemTime, sizeof(sensor_12_last_seen) - 1);
            sensor_12_last_seen[sizeof(sensor_12_last_seen) - 1] = '\0';
            break;
            
        default:
            Serial.printf("Unknown sensor ID: %d\n", packet->sensor_id);
            break;
    }
}

void PacketHandler::printStatistics() {
    if (_jsonPacketQueue != NULL) {
        UBaseType_t queueSpace = uxQueueSpacesAvailable(_jsonPacketQueue);
        Serial.printf("Packet Queue: %u free spaces, %llu total received, %llu lost\n",
                      queueSpace, _total_packets_received, _packet_loss_counter);
    }
}