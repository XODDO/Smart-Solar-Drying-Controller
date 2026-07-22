#include "WiFi.h"
#include <esp_wifi.h>   // For ESP-NOW internal Wi-Fi APIs
#include <esp_now.h>
#include <esp_wifi_types.h>  // For wifi_interface_t
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

/*
bool PacketHandler::begin() {
    // Set WiFi mode
    WiFi.mode(WIFI_STA);

    // Add this - ESP-NOW requires WiFi to be initialized
    if (WiFi.status() == WL_NO_SHIELD) {
        Serial.println("WiFi not initialized properly");
        return false;
    }
    
    // Give WiFi time to initialize
    delay(100);
    
    // Initialize ESP-NOW
    if(esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return false;
    }
    
    Serial.println("ESPNOW successfully initialized");

     // Add broadcast peer for receiving from all devices
    esp_now_peer_info_t broadcastPeer = {};
    memset(&broadcastPeer, 0, sizeof(broadcastPeer));
    broadcastPeer.channel = 0;  // Use current channel
    //broadcastPeer.ifidx = ESP_IF_WIFI_STA;
    broadcastPeer.encrypt = false;
    
    // Set broadcast MAC address
    for (int i = 0; i < 6; i++) {
        broadcastPeer.peer_addr[i] = 0xFF;
    }
    
    esp_err_t addPeerResult = esp_now_add_peer(&broadcastPeer);
    if (addPeerResult != ESP_OK) {
        Serial.printf("Failed to add broadcast peer: %d\n", addPeerResult);
        // This might still work without adding peer, but try it
    }

    
    
    // Register callback function
    esp_err_t result = esp_now_register_recv_cb(onDataReceived);
    if(result == ESP_OK) {
        Serial.println("[Sensor] Callback successfully set!");
    } else {
        Serial.println("[Sensor] Failed to set callback!");
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
    Serial.printf("[Sensor] Heap Before Sensor Task: Free=%u bytes, Min=%u bytes\n",
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
        Serial.println("[Sensor ERROR] Failed to create packet handler task!");
        Serial.printf("Reason: %d\n", result_task);
        return false;
    } else {
        Serial.println("[Sensor OK] Packet handler task created successfully");
    }
    
    Serial.printf("[Sensor] Heap After Task: Free=%u bytes\n", esp_get_free_heap_size());
    
    return true;
}



bool PacketHandler::begin() {
     // Ensure clean state first
   // shutdown();  // ← Critical! Clean up before initializing
    
    // Make sure to store the task handle:

    Serial.println("=== Starting memory-safe PacketHandler initialization ===");
    
    // Step 1: Check WiFi hardware
    Serial.println("Step 1: Checking WiFi hardware...");
    WiFi.mode(WIFI_OFF);
    // After WiFi.mode(WIFI_STA), add:
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    Serial.println("Receiver channel set to 1");

    delay(100);
    
    // Try to initialize WiFi with more control
    Serial.println("Step 2: Setting WiFi mode to STA...");
    bool modeSet = WiFi.mode(WIFI_STA);
    Serial.printf("WiFi.mode(WIFI_STA) returned: %s\n", modeSet ? "true" : "false");
    
    // Give time for WiFi to initialize
    delay(500);
    
    // Check WiFi status with more details
    Serial.println("Step 3: Checking WiFi status...");
    wl_status_t status = WiFi.status();
    Serial.printf("WiFi.status() = %d\n", status);
    
    switch(status) {
        case WL_NO_SHIELD:
            Serial.println("ERROR: WL_NO_SHIELD - WiFi hardware not detected!");
            Serial.println("Possible causes:");
            Serial.println("  - WiFi antenna not connected");
            Serial.println("  - ESP32 hardware issue");
            Serial.println("  - Power supply insufficient");
            Serial.println("  - Previous WiFi operations not properly deinitialized");
            break;
        case WL_IDLE_STATUS:
            Serial.println("Status: WL_IDLE_STATUS - WiFi idle, waiting for connection");
            break;
        case WL_NO_SSID_AVAIL:
            Serial.println("Status: WL_NO_SSID_AVAIL - No networks found");
            break;
        case WL_SCAN_COMPLETED:
            Serial.println("Status: WL_SCAN_COMPLETED - Scan completed");
            break;
        case WL_CONNECTED:
            Serial.println("Status: WL_CONNECTED - Connected to WiFi");
            break;
        case WL_CONNECT_FAILED:
            Serial.println("Status: WL_CONNECT_FAILED - Connection failed");
            break;
        case WL_CONNECTION_LOST:
            Serial.println("Status: WL_CONNECTION_LOST - Connection lost");
            break;
        case WL_DISCONNECTED:
            Serial.println("Status: WL_DISCONNECTED - Disconnected");
            break;
        default:
            Serial.printf("Unknown status: %d\n", status);
            break;
    }
    
    // Step 4: Get MAC address to verify WiFi is working
    if (status != WL_NO_SHIELD) {
        uint8_t mac[6];
        WiFi.macAddress(mac);
        Serial.printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    
    // Step 5: If WiFi hardware not detected, try to reinitialize
    if (status == WL_NO_SHIELD) {
        Serial.println("Attempting to recover WiFi hardware...");
        
        // Try to deinit and reinit WiFi
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        delay(1000);
        
        // Try ESP32 WiFi specific initialization
        esp_wifi_stop();
        delay(100);
        esp_wifi_deinit();
        delay(100);
        
        // Reinitialize WiFi
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_wifi_init(&cfg);
        Serial.printf("esp_wifi_init returned: %s\n", esp_err_to_name(err));
        
        if (err == ESP_OK) {
            err = esp_wifi_set_mode(WIFI_MODE_STA);
            Serial.printf("esp_wifi_set_mode returned: %s\n", esp_err_to_name(err));
            
            if (err == ESP_OK) {
                err = esp_wifi_start();
                Serial.printf("esp_wifi_start returned: %s\n", esp_err_to_name(err));
                delay(100);
            }
        }
        
        // Check status again
        status = WiFi.status();
        Serial.printf("After recovery, WiFi.status() = %d\n", status);
        
        if (status == WL_NO_SHIELD) {
            Serial.println("FATAL: Cannot recover WiFi hardware!");
            return false;
        }
    }
    
    // Step 6: Initialize ESP-NOW
    Serial.println("Step 4: Initializing ESP-NOW...");
    esp_err_t esp_now_err = esp_now_init();
    
    if(esp_now_err != ESP_OK) {
        Serial.printf("Error initializing ESP-NOW: %s (%d)\n", 
                     esp_err_to_name(esp_now_err), esp_now_err);
        
        // Try to get more details
        switch(esp_now_err) {
            case ESP_ERR_ESPNOW_NOT_INIT:
                Serial.println("  - ESP-NOW not initialized");
                break;
            case ESP_ERR_ESPNOW_ARG:
                Serial.println("  - Invalid argument");
                break;
            case ESP_ERR_ESPNOW_INTERNAL:
                Serial.println("  - Internal error");
                break;
            case ESP_ERR_ESPNOW_NO_MEM:
                Serial.println("  - Out of memory");
                break;
            case ESP_ERR_ESPNOW_NOT_FOUND:
                Serial.println("  - Peer not found");
                break;
            default:
                Serial.println("  - Unknown error");
                break;
        }
        
        return false;
    }
    
    Serial.println("ESP-NOW successfully initialized");
    
    // Step 7: Add broadcast peer
    Serial.println("Step 5: Adding broadcast peer...");
    esp_now_peer_info_t broadcastPeer = {};
    memset(&broadcastPeer, 0, sizeof(broadcastPeer));
    broadcastPeer.channel = 0;
   // broadcastPeer.ifidx = ESP_IF_WIFI_STA;
    broadcastPeer.encrypt = false;
    
    // Set broadcast MAC address
    for (int i = 0; i < 6; i++) {
        broadcastPeer.peer_addr[i] = 0xFF;
    }
    
    esp_err_t addPeerResult = esp_now_add_peer(&broadcastPeer);
    if (addPeerResult != ESP_OK) {
        Serial.printf("Failed to add broadcast peer: %s (%d)\n", 
                     esp_err_to_name(addPeerResult), addPeerResult);
        
        // Try alternative - add peer with specific MAC if you know it
        // or continue without adding peer (some ESP-NOW versions work without)
        if (addPeerResult == ESP_ERR_ESPNOW_EXIST) {
            Serial.println("  - Peer already exists, continuing...");
        } else {
            Serial.println("  - Continuing anyway, might still work...");
        }
    } else {
        Serial.println("  - Broadcast peer added successfully");
    }
    
    // Step 8: Register callback
    Serial.println("Step 6: Registering receive callback...");
    esp_err_t result = esp_now_register_recv_cb(onDataReceived);
    if(result == ESP_OK) {
        Serial.println("  - Callback successfully registered!");
    } else {
        Serial.printf("  - Failed to set callback: %s (%d)\n", 
                     esp_err_to_name(result), result);
        return false;
    }
    
    delay(1000);
    
    // Step 9: Create queue
    Serial.printf("Step 7: Creating queue of size %d...\n", QUEUE_SIZE);
    _jsonPacketQueue = xQueueCreate(QUEUE_SIZE, sizeof(QueuedJsonPacket));
    if (_jsonPacketQueue == NULL) {
        Serial.println("ERROR: Failed to create queue!");
        Serial.printf("  - Free heap: %u bytes\n", esp_get_free_heap_size());
        return false;
    }
    Serial.println("  - Queue created successfully");
    
    // Step 10: Create task
    Serial.println("Step 8: Creating packet handler task...");
    Serial.printf("  - Free heap before task: %u bytes\n", esp_get_free_heap_size());
    Serial.printf("  - Minimum free heap: %u bytes\n", esp_get_minimum_free_heap_size());
    
    BaseType_t result_task = xTaskCreatePinnedToCore(
        packetHandlerTask,
        "PacketHandler",
        PACKET_STACK_SIZE,
        this,
        PACKET_TASK_PRIORITY,
        &_packetTaskHandle,
        PACKET_TASK_CORE
    );
    
    if (result_task != pdPASS) {
        Serial.printf("ERROR: Failed to create task! Reason: %d\n", result_task);
        
        switch(result_task) {
            case errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY:
                Serial.println("  - Could not allocate required memory");
                Serial.printf("  - Try increasing PACKET_STACK_SIZE from %d\n", PACKET_STACK_SIZE);
                break;
            default:
                Serial.printf("  - Unknown error: %d\n", result_task);
                break;
        }
        
        return false;
    }
    
    Serial.println("  - Task created successfully");
    Serial.printf("  - Free heap after task: %u bytes\n", esp_get_free_heap_size());
    
    // Step 11: Print final status
    Serial.println("=== PacketHandler initialization complete ===");
    Serial.printf("WiFi Status: %d\n", WiFi.status());
    
    // Optional: Perform a WiFi scan to verify radio works
    Serial.println("Testing WiFi scan (this confirms radio is working)...");
    int networksFound = WiFi.scanNetworks();
    Serial.printf("Scan complete: %d networks found\n", networksFound);
    if (networksFound > 0) {
        for(int i = 0; i < min(3, networksFound); i++) {
            Serial.printf("  - %s (RSSI: %d)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
        }
    }
    WiFi.scanDelete();  // Clean up
    
    return true;
}

*/

bool PacketHandler::begin() {

    Serial.println("=== PacketHandler::begin() ===");
    Serial.printf("Heap on entry: free=%u, largest=%u\n",
                  esp_get_free_heap_size(),
                  heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    // ── Step 1: Bring WiFi up cleanly ────────────────────────────────────────
    // WiFi MUST already be OFF before this is called (setup() tears it down
    // after NTP). We just set STA mode and start the driver.
    Serial.println("Step 1: Starting WiFi driver in STA mode...");
    WiFi.mode(WIFI_STA);
    delay(200);

    wl_status_t status = WiFi.status();
    Serial.printf("  WiFi.status() = %d\n", (int)status);

    if (status == WL_NO_SHIELD) {
        // Driver failed to start — one recovery attempt via low-level API
        Serial.println("  WL_NO_SHIELD detected, attempting low-level recovery...");

        esp_wifi_stop();   delay(100);
        esp_wifi_deinit(); delay(100);

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_wifi_init(&cfg);
        Serial.printf("  esp_wifi_init: %s\n", esp_err_to_name(err));
        if (err != ESP_OK) { Serial.println("  FATAL: WiFi hardware unrecoverable"); return false; }

        err = esp_wifi_set_mode(WIFI_MODE_STA);
        Serial.printf("  esp_wifi_set_mode: %s\n", esp_err_to_name(err));
        if (err != ESP_OK) return false;

        err = esp_wifi_start();
        Serial.printf("  esp_wifi_start: %s\n", esp_err_to_name(err));
        if (err != ESP_OK) return false;

        delay(200);
        status = WiFi.status();
        Serial.printf("  Status after recovery: %d\n", (int)status);

        if (status == WL_NO_SHIELD) {
            Serial.println("  FATAL: Cannot recover WiFi hardware");
            return false;
        }
    }

    // Print status string
    const char* statusStr = "UNKNOWN";
    switch (status) {
        case WL_IDLE_STATUS:    statusStr = "IDLE";        break;
        case WL_DISCONNECTED:   statusStr = "DISCONNECTED"; break;
        case WL_CONNECTED:      statusStr = "CONNECTED";   break;
        case WL_NO_SSID_AVAIL:  statusStr = "NO_SSID";     break;
        default: break;
    }
    Serial.printf("  WiFi status: %s\n", statusStr);

    // Print MAC
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("  MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // ── Step 2: Lock channel to 1 (must match transmitters) ──────────────────
    Serial.println("Step 2: Setting channel 1...");
    esp_err_t ch_err = esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    Serial.printf("  esp_wifi_set_channel: %s\n", esp_err_to_name(ch_err));
    // Non-fatal — ESP-NOW will still work on whatever channel is active

    // ── Step 3: Initialise ESP-NOW ────────────────────────────────────────────
    Serial.println("Step 3: Initialising ESP-NOW...");
    esp_err_t now_err = esp_now_init();
    if (now_err != ESP_OK) {
        Serial.printf("  esp_now_init FAILED: %s (%d)\n",
                      esp_err_to_name(now_err), (int)now_err);
        return false;
    }
    Serial.println("  ESP-NOW initialised OK");

    // ── Step 4: Add broadcast peer ────────────────────────────────────────────
    Serial.println("Step 4: Adding broadcast peer...");
    esp_now_peer_info_t peer = {};
    memset(&peer, 0, sizeof(peer));
    memset(peer.peer_addr, 0xFF, 6);   // FF:FF:FF:FF:FF:FF
    peer.channel = 0;                  // 0 = use current channel
    peer.encrypt = false;

    esp_err_t peer_err = esp_now_add_peer(&peer);
    if (peer_err == ESP_OK) {
        Serial.println("  Broadcast peer added");
    } else if (peer_err == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("  Broadcast peer already exists — OK");
    } else {
        Serial.printf("  add_peer warning: %s — continuing\n",
                      esp_err_to_name(peer_err));
    }

    // ── Step 5: Register receive callback ────────────────────────────────────
    Serial.println("Step 5: Registering receive callback...");
    esp_err_t cb_err = esp_now_register_recv_cb(onDataReceived);
    if (cb_err != ESP_OK) {
        Serial.printf("  register_recv_cb FAILED: %s\n", esp_err_to_name(cb_err));
        esp_now_deinit();
        return false;
    }
    Serial.println("  Callback registered");

    // ── Step 6: Create FreeRTOS queue ─────────────────────────────────────────
    Serial.printf("Step 6: Creating queue (size=%d, item=%u bytes)...\n",
                  QUEUE_SIZE, (unsigned)sizeof(QueuedJsonPacket));
    Serial.printf("  Heap before queue: free=%u, largest=%u\n",
                  esp_get_free_heap_size(),
                  heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    _jsonPacketQueue = xQueueCreate(QUEUE_SIZE, sizeof(QueuedJsonPacket));
    if (_jsonPacketQueue == NULL) {
        Serial.println("  FATAL: xQueueCreate returned NULL — out of heap");
        Serial.printf("  free=%u  largest=%u\n",
                      esp_get_free_heap_size(),
                      heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        esp_now_deinit();
        return false;
    }
    Serial.println("  Queue created OK");

    // ── Step 7: Create packet-handler task ───────────────────────────────────
    Serial.printf("Step 7: Creating task (stack=%d, core=%d, pri=%d)...\n",
                  PACKET_STACK_SIZE, PACKET_TASK_CORE, PACKET_TASK_PRIORITY);
    Serial.printf("  Heap before task: free=%u, largest=%u\n",
                  esp_get_free_heap_size(),
                  heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    BaseType_t task_ok = xTaskCreatePinnedToCore(
        packetHandlerTask,
        "PacketHandler",
        PACKET_STACK_SIZE,
        this,
        PACKET_TASK_PRIORITY,
        &_packetTaskHandle,
        PACKET_TASK_CORE
    );

    if (task_ok != pdPASS) {
        Serial.printf("  FATAL: xTaskCreatePinnedToCore failed (%d)\n", (int)task_ok);
        if (task_ok == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) {
            Serial.printf("  Not enough heap for %d-byte stack\n", PACKET_STACK_SIZE);
        }
        vQueueDelete(_jsonPacketQueue);
        _jsonPacketQueue = NULL;
        esp_now_deinit();
        return false;
    }

    Serial.println("  Task created OK");
    Serial.printf("  Heap after task:  free=%u, largest=%u\n",
                  esp_get_free_heap_size(),
                  heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    // ── Step 8: Optional WiFi scan (radio sanity check) ───────────────────────
    Serial.println("Step 8: WiFi scan (radio sanity check)...");
    int found = WiFi.scanNetworks(false, true);   // blocking, include hidden
    if (found <= 0) {
        Serial.println("  No networks found (normal if in a shielded space)");
    } else {
        Serial.printf("  %d network(s) found:\n", found);
        int show = (found < 3) ? found : 3;
        for (int i = 0; i < show; i++) {
            Serial.printf("    [%d] %s  RSSI=%d  ch=%d\n",
                          i, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
        }
    }
    WiFi.scanDelete();

    // ── Done ──────────────────────────────────────────────────────────────────
    Serial.println("=== PacketHandler ready ===");
    Serial.printf("Final heap: free=%u, largest=%u, minEver=%u\n",
                  esp_get_free_heap_size(),
                  heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                  esp_get_minimum_free_heap_size());

    return true;
}

// Static ISR callback
void PacketHandler::onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
  //  Serial.printf("ISR: Data received from MAC: %02X:%02X:%02X:%02X:%02X:%02X, Len: %d\n",
 //                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], len); // delete me after testing

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
    //Serial.println("Handler summoned!");
    PacketHandler* handler = (PacketHandler*)pvParams;
    QueuedJsonPacket jsonPacket;
    ReceivedPacket packet;
    
    // Get initial stack high water mark
    UBaseType_t initial_watermark = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("[Stack] Initial high water: %u bytes\n", initial_watermark);
    
    uint32_t packets_processed = 0;
    uint32_t last_print = 0;

    
    while (true) {
        if (xQueueReceive(handler->_jsonPacketQueue, &jsonPacket, portMAX_DELAY) == pdTRUE) {
            handler->_total_packets_received++;
            packets_processed++; 
            
            // // Parse JSON - this uses the most stack
            handler->parseJsonPacket(jsonPacket.data, jsonPacket.length, &packet);
            
            // Extract readings to global variables
            handler->extractReadingsFromPacket(&packet);
            
            // Optional: Beep for testing
             buzzer.beep(1, 20, 0);
             digitalWrite(flashing_led, HIGH); delay(20); digitalWrite(flashing_led, LOW);

          
          //   Serial.println("Readings Extracted and sent over to JSON Parser");

            // Check stack usage every 10 packets
             // Check stack usage every 10 packets
            if (packets_processed % 10 == 0) {
                UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
                uint32_t used = PACKET_STACK_SIZE - watermark;
                uint32_t percent = (used * 100) / PACKET_STACK_SIZE;
                
                Serial.printf("[Stack] After %lu packets: Used %u/%u bytes (%u%%)\n",
                             packets_processed, used, PACKET_STACK_SIZE, percent);
                last_print = packets_processed;
            }
            
        }
    }
}

void PacketHandler::parseJsonPacket(const uint8_t *data, int len, ReceivedPacket *packet) {
   // StaticJsonDocument<256> doc;
   // DeserializationError error = deserializeJson(doc, data, len);
    JsonDocument doc;  // Changed from StaticJsonDocument<256>
    DeserializationError error = deserializeJson(doc, data, len);
    

    if (error) {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return;
    }
    
          //  Serial.println("Pasrsing JSON...");
           

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

    packet->speed_of_wind = doc["Wind_Speed"] | NAN;
    packet->solar_radiation = doc["Solar_Rad"] | NAN;
    
    // Copy string fields
    strlcpy(packet->CPU_Freq, doc["CPU"] | "0", sizeof(packet->CPU_Freq));
    strlcpy(packet->transmissions, doc["Sends"] | "0", sizeof(packet->transmissions));
    
    // Set timestamp using RTC time
    packet->last_seen_ms = esp_timer_get_time()/1000ULL;  // esp_timer_get_time millisecondseconds since boot
    strlcpy(packet->last_seen, sawa.SystemTime, sizeof(packet->last_seen));

        /*
    
     Serial.print("Parsed ID: "); Serial.println(sensor_id_str); 
     Serial.print("\tParsed Temp: "); Serial.println(packet->temperature);
     Serial.print("\tParsed Humi: "); Serial.println(packet->humidity);
     Serial.print("\tParsed Pre: "); Serial.println(packet->pressure);
     Serial.print("\tParsed Elv: "); Serial.println(packet->elevation);
     Serial.print("\tParsed CPU Freq: "); Serial.println(packet->CPU_Freq);
     Serial.print("\tLast Seen: "); Serial.println(packet->last_seen);
     Serial.print("\tLast Seen (ms): "); Serial.println(packet->last_seen_ms);
     */
     
     
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
    
    // Validate range only 1-15 not 1-12
    if (num < 1 || num > 15) {
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
           // sensor_1_h_index = packet->heat_index;
            sensor_1_running_time_ms = packet->time_on_in_ms;
            sensor_1_last_seen_ms = esp_timer_get_time()/1000ULL;  // Current time since boot
           // strncpy(sensor_1_CPU_freq, packet->CPU_Freq, sizeof(sensor_1_CPU_freq) - 1);
           // sensor_1_CPU_freq[sizeof(sensor_1_CPU_freq) - 1] = '\0';
            strncpy(sensor_1_last_seen, sawa.SystemTime, sizeof(sensor_1_last_seen) - 1);
            sensor_1_last_seen[sizeof(sensor_1_last_seen) - 1] = '\0';
            break;
            
        case 2:
            sensor_2_temp = packet->temperature;
            sensor_2_humidity = packet->humidity;
            sensor_2_pressure = packet->pressure;
            sensor_2_elevation = packet->elevation;
            sensor_2_running_time_ms = packet->time_on_in_ms;
            sensor_2_last_seen_ms = esp_timer_get_time()/1000ULL;
          //  strncpy(sensor_2_CPU_freq, packet->CPU_Freq, sizeof(sensor_2_CPU_freq) - 1);
          //  sensor_2_CPU_freq[sizeof(sensor_2_CPU_freq) - 1] = '\0';
            strncpy(sensor_2_last_seen, sawa.SystemTime, sizeof(sensor_2_last_seen) - 1);
            sensor_2_last_seen[sizeof(sensor_2_last_seen) - 1] = '\0';
            break;
            
        case 3:
            sensor_3_temp = packet->temperature;
            sensor_3_humidity = packet->humidity;
            sensor_3_pressure = packet->pressure;
            sensor_3_elevation = packet->elevation;
            sensor_3_running_time_ms = packet->time_on_in_ms;
            sensor_3_last_seen_ms = esp_timer_get_time()/1000ULL;
          //  strncpy(sensor_3_CPU_freq, packet->CPU_Freq, sizeof(sensor_3_CPU_freq) - 1);
        //    sensor_3_CPU_freq[sizeof(sensor_3_CPU_freq) - 1] = '\0';
            strncpy(sensor_3_last_seen, sawa.SystemTime, sizeof(sensor_3_last_seen) - 1);
            sensor_3_last_seen[sizeof(sensor_3_last_seen) - 1] = '\0';
            break;
            
        case 4:
            sensor_4_temp = packet->temperature;
            sensor_4_humidity = packet->humidity;
            sensor_4_pressure = packet->pressure;
            sensor_4_elevation = packet->elevation;
            sensor_4_running_time_ms = packet->time_on_in_ms;
            sensor_4_last_seen_ms = esp_timer_get_time()/1000ULL;
          //  strncpy(sensor_4_CPU_freq, packet->CPU_Freq, sizeof(sensor_4_CPU_freq) - 1);
        //    sensor_4_CPU_freq[sizeof(sensor_4_CPU_freq) - 1] = '\0';
            strncpy(sensor_4_last_seen, sawa.SystemTime, sizeof(sensor_4_last_seen) - 1);
            sensor_4_last_seen[sizeof(sensor_4_last_seen) - 1] = '\0';
            break;
            
        case 5:
            sensor_5_temp = packet->temperature;
            sensor_5_humidity = packet->humidity;
            sensor_5_pressure = packet->pressure;
            sensor_5_elevation = packet->elevation;
            sensor_5_running_time_ms = packet->time_on_in_ms;
            sensor_5_last_seen_ms = esp_timer_get_time()/1000ULL;
         //   strncpy(sensor_5_CPU_freq, packet->CPU_Freq, sizeof(sensor_5_CPU_freq) - 1);
         //   sensor_5_CPU_freq[sizeof(sensor_5_CPU_freq) - 1] = '\0';
            strncpy(sensor_5_last_seen, sawa.SystemTime, sizeof(sensor_5_last_seen) - 1);
            sensor_5_last_seen[sizeof(sensor_5_last_seen) - 1] = '\0';
            break;
            
        case 6:
            sensor_6_temp = packet->temperature;
            sensor_6_humidity = packet->humidity;
            sensor_6_pressure = packet->pressure;
            sensor_6_elevation = packet->elevation;
            sensor_6_running_time_ms = packet->time_on_in_ms;
            sensor_6_last_seen_ms = esp_timer_get_time()/1000ULL;
        //    strncpy(sensor_6_CPU_freq, packet->CPU_Freq, sizeof(sensor_6_CPU_freq) - 1);
       //     sensor_6_CPU_freq[sizeof(sensor_6_CPU_freq) - 1] = '\0';
            strncpy(sensor_6_last_seen, sawa.SystemTime, sizeof(sensor_6_last_seen) - 1);
            sensor_6_last_seen[sizeof(sensor_6_last_seen) - 1] = '\0';
            break;
            
            
        default:
            Serial.printf("Unknown sensor ID: %d\n", packet->sensor_id);
            break;
    }
}


void PacketHandler::shutdown() {
    Serial.println("[PacketHandler] Shutting down...");
    
    // 1. Stop receiving packets
    esp_now_register_recv_cb(NULL);
    
    // 2. Delete queue
    if (_jsonPacketQueue != NULL) {
        vQueueDelete(_jsonPacketQueue);
        _jsonPacketQueue = NULL;
        Serial.println("  - Queue deleted");
    }
    
    // 3. Delete task
    if (_packetTaskHandle != NULL) {
        vTaskDelete(_packetTaskHandle);
        _packetTaskHandle = NULL;
        Serial.println("  - Task deleted");
    }
    
    // 4. Deinit ESP-NOW
    if (esp_now_deinit() == ESP_OK) {
        Serial.println("  - ESP-NOW deinitialized");
    } else {
        Serial.println("  - ESP-NOW deinit warning");
    }
    
    // 5. Clean up WiFi
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    
    Serial.println("[PacketHandler] Shutdown complete");
}



void PacketHandler::printStatistics() {
    if (_jsonPacketQueue != NULL) {
        UBaseType_t queueSpace = uxQueueSpacesAvailable(_jsonPacketQueue);
        Serial.printf("Packet Queue: %u free spaces, %llu total received, %llu lost\n",
                      queueSpace, _total_packets_received, _packet_loss_counter);
    }
}