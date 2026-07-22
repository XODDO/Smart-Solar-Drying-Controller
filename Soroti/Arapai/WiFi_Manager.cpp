#include "WiFi_Manager.h"

WiFi_Manager::WiFi_Manager(uint8_t wifi_led_pin)
    : devicename(nullptr), currentNetworkIndex(-1),
      wifi_connect_attempts(0), wifi_led(wifi_led_pin), lastReconnectAttempt(0),
      backoff_delay_ms(1000), consecutive_failures(0), ntp_synced(false) {}

// In WiFi_Manager.cpp
bool WiFi_Manager::check_hardware() {
    Serial.println("[WiFi] ========== HARDWARE CHECK ==========");
    Serial.println("[WiFi] Step 1: Checking WiFi hardware...");
    
    // Power cycle the WiFi module
    WiFi.mode(WIFI_OFF);
    delay(500);
    
    Serial.println("[WiFi] Receiver channel set to 1");
    delay(100);
    
    Serial.println("[WiFi] Step 2: Setting WiFi mode to STA...");
    WiFi.mode(WIFI_STA);
    delay(1000);  // CRITICAL: Wait for mode to stabilize
    
    wl_status_t status = WiFi.status();
    Serial.printf("[WiFi] WiFi.status() = %d (", status);
    switch(status) {
        case WL_NO_SHIELD: Serial.println("NO_SHIELD)"); break;
        case WL_IDLE_STATUS: Serial.println("IDLE_STATUS)"); break;
        case WL_NO_SSID_AVAIL: Serial.println("NO_SSID_AVAIL)"); break;
        case WL_SCAN_COMPLETED: Serial.println("SCAN_COMPLETED)"); break;
        case WL_CONNECTED: Serial.println("CONNECTED)"); break;
        case WL_CONNECT_FAILED: Serial.println("CONNECT_FAILED)"); break;
        case WL_CONNECTION_LOST: Serial.println("CONNECTION_LOST)"); break;
        case WL_DISCONNECTED: Serial.println("DISCONNECTED)"); break;
        default: Serial.println("UNKNOWN)"); break;
    }
    
    // Get MAC address
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("[WiFi] MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    Serial.println("[WiFi] =========================================\n");
    return (status != WL_NO_SHIELD);
}

bool WiFi_Manager::recover_hardware() {
    Serial.println("[WiFi] ========== HARDWARE RECOVERY ==========");
    Serial.println("[WiFi] Attempting to recover WiFi hardware...");
    
    WiFi.disconnect(true);
    delay(500);
    
    WiFi.mode(WIFI_OFF);
    delay(500);
    
    Serial.println("[WiFi] Stopping ESP WiFi...");
    esp_wifi_stop();
    delay(200);
    
    Serial.println("[WiFi] Deinitializing ESP WiFi...");
    esp_wifi_deinit();
    delay(200);
    
    Serial.println("[WiFi] Re-initializing ESP WiFi...");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[WiFi] ❌ esp_wifi_init failed: %d\n", err);
        return false;
    }
    Serial.println("[WiFi] ✅ esp_wifi_init successful");
    
    Serial.println("[WiFi] Setting WiFi mode to STA...");
    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        Serial.printf("[WiFi] ❌ esp_wifi_set_mode failed: %d\n", err);
        return false;
    }
    Serial.println("[WiFi] ✅ esp_wifi_set_mode successful");
    
    Serial.println("[WiFi] Starting ESP WiFi...");
    err = esp_wifi_start();
    delay(200);
    if (err != ESP_OK) {
        Serial.printf("[WiFi] ❌ esp_wifi_start failed: %d\n", err);
        return false;
    }
    Serial.println("[WiFi] ✅ esp_wifi_start successful");
    
    Serial.println("[WiFi] =========================================\n");
    return (err == ESP_OK);
}

// Simplified main connection function
WiFiMgrStatus WiFi_Manager::initialize_ESP_WiFi(const char* device_name) {
    devicename = device_name;
    
    Serial.println("\n[WiFi] ===== INITIALIZING WIFI MANAGER =====");
    
    // 1. Check hardware
    Serial.println("[WiFi] Performing initial hardware check...");
    if (!check_hardware()) {
        Serial.println("[WiFi] ⚠️  Hardware check failed, attempting recovery...");
        if (!recover_hardware()) {
            Serial.println("[WiFi] ❌ Hardware recovery failed!");
            return WIFI_MGR_CONN_FAIL;
        }
    }
    
    // 2. Show known networks
    Serial.printf("[WiFi] Known networks count: %d\n", knownCount);
    for (int i = 0; i < knownCount; i++) {
        Serial.printf("[WiFi]   Known network %d: %s\n", i, knownNetworks[i].SECRET_SSID);
    }
    
    // 3. Scan and connect (existing code)
    pinMode(wifi_led, OUTPUT);
    set_led(false);
    
    Serial.println("\n[WiFi] ========== INITIAL SCAN ==========");
    log_message("Scanning for available WiFi networks...");
    
    Serial.println("[WiFi] Starting WiFi scan (passive mode)...");
    int n = WiFi.scanNetworks(false, true);  // async=false, show_hidden=true
    
    Serial.printf("[WiFi] Scan returned: %d\n", n);
    if (n < 0) {
        log_message("❌ WiFi scan error.");
        WiFi.scanDelete();
        return WIFI_MGR_SCAN_ERR;
    }
    if (n == 0) {
        log_message("❌ No WiFi networks found.");
        WiFi.scanDelete();
        return WIFI_MGR_NO_NETWORKS;
    }
    
    // Print found networks
    Serial.printf("[WiFi] Found %d network(s):\n", n);
    for (int i = 0; i < min(n, 10); i++) {
        Serial.printf("[WiFi]   %d. %s (RSSI: %d dBm, CH: %d, SEC: %d)\n", 
                     i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i), WiFi.encryptionType(i));
    }
    if (n > 10) {
        Serial.printf("[WiFi]   ... and %d more networks\n", n - 10);
    }
    
    // Find strongest known network
    int bestIndex = -1, bestRSSI = -1000;
    String bestSSID = "";
    Serial.println("[WiFi] Matching against known networks...");
    
    for (int i = 0; i < n; i++) {
        String foundSSID = WiFi.SSID(i);
        for (int k = 0; k < knownCount; k++) {
            if (foundSSID == knownNetworks[k].SECRET_SSID) {
                Serial.printf("[WiFi]   ✓ Found known network: %s (RSSI: %d dBm)\n", foundSSID.c_str(), WiFi.RSSI(i));
                if (WiFi.RSSI(i) > bestRSSI) {
                    bestIndex = k;
                    bestRSSI = WiFi.RSSI(i);
                    bestSSID = foundSSID;
                }
            }
        }
    }
    WiFi.scanDelete();
    
    if (bestIndex < 0) {
        Serial.println("[WiFi] ❌ No known networks matched. Known networks list:");
        for (int k = 0; k < knownCount; k++) {
            Serial.printf("[WiFi]   - %s\n", knownNetworks[k].SECRET_SSID);
        }
        log_message("❌ No known networks available.");
        return WIFI_MGR_NO_KNOWN;
    }
    
    Serial.printf("[WiFi] ✓ Best known network: %s (RSSI: %d dBm)\n", bestSSID.c_str(), bestRSSI);
    log_message("Found known network. Connecting...");
    
    Serial.println("[WiFi] =========================================\n");
    
    if (try_connect(knownNetworks[bestIndex].SECRET_SSID, knownNetworks[bestIndex].SECRET_PASS)) {
        currentNetworkIndex = bestIndex;
        set_led(true);
        log_message("✅ WiFi connected successfully.");
        
        print_connection_stats();
        
        consecutive_failures = 0;
        backoff_delay_ms = 1000;
        return WIFI_MGR_SUCCESS;
    } else {
        set_led(false);
        log_message("❌ Connection attempt failed.");
        consecutive_failures++;
        return WIFI_MGR_CONN_FAIL;
    }
}

bool WiFi_Manager::ensure_wifi() {
    // If already connected, DO NOT scan, do NOT reconnect
    if (WiFi.status() == WL_CONNECTED) {
        set_led(true);
        if (consecutive_failures > 0) {
            consecutive_failures = 0;
            backoff_delay_ms = 1000;
            Serial.println("[WiFi] ✓ Connection restored, resetting backoff counter.");
        }
        return true;
    }

    uint64_t now_us = esp_timer_get_time();
    uint64_t now_ms = now_us / 1000;
    
    // Check if we're in backoff period
    if (consecutive_failures >= 5 && (now_ms - lastReconnectAttempt) < backoff_delay_ms) {
        static uint32_t last_backoff_print = 0;
        if ((now_ms - last_backoff_print) > 10000) { // Print every 10 seconds
            Serial.printf("[WiFi] ⏳ Exponential backoff: Waiting %lu ms before next attempt (failures: %d)\n", 
                         backoff_delay_ms, consecutive_failures);
            last_backoff_print = now_ms;
        }
        return false; // Still in backoff period
    }
    
    if (now_us - lastReconnectAttempt < RECONNECT_INTERVAL_US) {
        return false; // throttle attempts
    }
    lastReconnectAttempt = now_us;

    Serial.println("\n[WiFi] ========== RECONNECTION ATTEMPT ==========");
    log_message("WiFi lost. Attempting reconnection...");
    Serial.printf("[WiFi] Current status code: %d\n", WiFi.status());
    set_led(false);

    // Step 1: Try previous network if still visible and decent RSSI
    if (currentNetworkIndex >= 0) {
        Serial.printf("[WiFi] Step 1: Checking previous network: %s\n", 
                     knownNetworks[currentNetworkIndex].SECRET_SSID);
        
        Serial.println("[WiFi] Starting scan for previous network...");
        int n = WiFi.scanNetworks(false, true);
        bool foundPrev = false;
        int prevRSSI = -1000;
        
        Serial.printf("[WiFi] Scan returned: %d networks\n", n);
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                String foundSSID = WiFi.SSID(i);
                if (foundSSID == knownNetworks[currentNetworkIndex].SECRET_SSID) {
                    foundPrev = true;
                    prevRSSI = WiFi.RSSI(i);
                    Serial.printf("[WiFi] ✓ Previous network found! RSSI: %d dBm\n", prevRSSI);
                    break;
                }
            }
        }
        WiFi.scanDelete();
        
        if (foundPrev) {
            Serial.printf("[WiFi] Previous network RSSI: %d dBm, Threshold: %d dBm\n", prevRSSI, RSSI_THRESHOLD);
        } else {
            Serial.println("[WiFi] Previous network not found in scan.");
        }
        
        if (foundPrev && prevRSSI > RSSI_THRESHOLD) {
            log_message("Previous network still visible. Reconnecting...");
            if (try_connect(knownNetworks[currentNetworkIndex].SECRET_SSID,
                            knownNetworks[currentNetworkIndex].SECRET_PASS)) {
                set_led(true);
                log_message("✓ Reconnected to previous network.");
                print_connection_stats();
                consecutive_failures = 0;
                backoff_delay_ms = 1000;
                return true;
            }
        } else {
            log_message("Searching alternatives...");
        }
    } else {
        Serial.println("[WiFi] No previous network index set.");
    }

    // Step 2: Try strongest of all known
    Serial.println("[WiFi] Step 2: Scanning for alternative networks...");
    int n = WiFi.scanNetworks(false, true);
    int bestIndex = -1, bestRSSI = -1000;
    String bestSSID = "";
    
    Serial.printf("[WiFi] Scan returned: %d networks\n", n);
    if (n > 0) {
        Serial.printf("[WiFi] Found %d network(s) in scan:\n", n);
        for (int i = 0; i < n && i < 10; i++) { // Limit to first 10 for readability
            Serial.printf("[WiFi]   %d. %s (RSSI: %d dBm)\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
        }
        if (n > 10) {
            Serial.printf("[WiFi]   ... and %d more networks\n", n - 10);
        }
        
        Serial.println("[WiFi] Matching against known networks...");
        for (int i = 0; i < n; i++) {
            String foundSSID = WiFi.SSID(i);
            for (int k = 0; k < knownCount; k++) {
                if (foundSSID == knownNetworks[k].SECRET_SSID && WiFi.RSSI(i) > bestRSSI) {
                    bestIndex = k;
                    bestRSSI = WiFi.RSSI(i);
                    bestSSID = foundSSID;
                    Serial.printf("[WiFi]   ✓ Candidate: %s (RSSI: %d dBm)\n", bestSSID.c_str(), bestRSSI);
                }
            }
        }
    }
    WiFi.scanDelete();

    if (bestIndex >= 0) {
        Serial.printf("[WiFi] ✓ Best alternative: %s (RSSI: %d dBm)\n", bestSSID.c_str(), bestRSSI);
        log_message("Connecting to strongest known network...");
        if (try_connect(knownNetworks[bestIndex].SECRET_SSID,
                        knownNetworks[bestIndex].SECRET_PASS)) {
            currentNetworkIndex = bestIndex;
            set_led(true);
            log_message("✓ Connected to new network successfully.");
            print_connection_stats();
            consecutive_failures = 0;
            backoff_delay_ms = 1000;
            return true;
        }
    } else {
        Serial.println("[WiFi] No alternative known networks found.");
    }

    // Connection failed - update backoff
    consecutive_failures++;
    
    // Exponential backoff: 1s, 2s, 4s, 8s, 16s, 32s, 64s... capped at 5 minutes
    if (consecutive_failures >= 5) {
        unsigned long new_backoff = backoff_delay_ms * 2;
        backoff_delay_ms = (new_backoff < 300000) ? new_backoff : 300000; // Cap at 5 minutes
        Serial.printf("[WiFi] ⏳ Exponential backoff increased to %lu ms (failures: %d)\n", 
                     backoff_delay_ms, consecutive_failures);
    }
    
    // Print WDT warning if needed
    if (consecutive_failures >= 10) {
        Serial.println("[WiFi] ⚠️  WARNING: Many consecutive failures! Watchdog may trigger soon.");
        Serial.printf("[WiFi] Last successful connection: %d attempts ago\n", consecutive_failures);
    }
    
    log_message("WiFi reconnect failed. Will retry later.");
    set_led(false);
    Serial.println("[WiFi] =========================================\n");
    return false;
}

bool WiFi_Manager::is_connected() const {
    bool connected = (WiFi.status() == WL_CONNECTED);
    if (!connected && wifi_connect_attempts > 0) {
        // Don't spam, just occasional debug
        static uint32_t last_status_print = 0;
        uint64_t now_ms = esp_timer_get_time() / 1000;
        if ((now_ms - last_status_print) > 30000) { // Every 30 seconds
            Serial.printf("[WiFi] Status: Disconnected (code: %d)\n", WiFi.status());
            last_status_print = now_ms;
        }
    }
    return connected;
}

String WiFi_Manager::get_ip() const {
    return (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "0.0.0.0";
}

bool WiFi_Manager::try_connect(const char* ssid, const char* pass, int timeout_ms) {
    Serial.println("\n[WiFi] ========== CONNECTION ATTEMPT ==========");
    Serial.printf("[WiFi] SSID: '%s'\n", ssid);
    Serial.printf("[WiFi] Password length: %d chars\n", strlen(pass));
    Serial.printf("[WiFi] Timeout: %d ms\n", timeout_ms);
    
    // Clean disconnect first
    Serial.println("[WiFi] Performing clean disconnect...");
    WiFi.disconnect(false);  // Don't turn off radio
    delay(500);
    
    // Configure WiFi before begin (CRITICAL for ESP32-S3)
    Serial.println("[WiFi] Configuring WiFi static settings...");
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    delay(200);
    
    // Verify mode
    Serial.printf("[WiFi] Current WiFi mode: %d\n", WiFi.getMode());
    
    // Begin connection
    Serial.println("[WiFi] Calling WiFi.begin()...");
    WiFi.begin(ssid, pass);
    delay(500);  // Critical delay after begin
    
    unsigned long start = millis();
    int last_status = -1;
    int status_change_count = 0;
    
    Serial.println("[WiFi] Waiting for connection...");
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
        delay(250);
        int status = WiFi.status();
        
        // Log on any status change
        if (status != last_status) {
            status_change_count++;
            const char* status_str;
            switch(status) {
                case WL_NO_SSID_AVAIL: status_str = "NO_SSID_AVAIL (network not found)"; break;
                case WL_CONNECT_FAILED: status_str = "CONNECT_FAILED (wrong password or blocked)"; break;
                case WL_IDLE_STATUS: status_str = "IDLE"; break;
                case WL_DISCONNECTED: status_str = "DISCONNECTED"; break;
                case WL_CONNECTION_LOST: status_str = "CONNECTION_LOST"; break;
                case WL_SCAN_COMPLETED: status_str = "SCAN_COMPLETED"; break;
                case WL_CONNECTED: status_str = "CONNECTED"; break;
                default: status_str = "UNKNOWN";
            }
            Serial.printf("[WiFi] Status change #%d: %s (%d) @ %lu ms\n", 
                         status_change_count, status_str, status, millis() - start);
            last_status = status;
            
            // Early exit on authentication failure
            if (status == WL_CONNECT_FAILED) {
                Serial.println("[WiFi] ❌ Authentication failed! Wrong password or MAC filtered.");
                Serial.println("[WiFi] =========================================\n");
                return false;
            }
        }
        
        // Print progress every 5 seconds
        if ((millis() - start) % 5000 == 0 && (millis() - start) > 0) {
            Serial.printf("[WiFi] Still waiting... (%lu ms / %d ms)\n", millis() - start, timeout_ms);
        }
    }
    
    // Check final result
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] ✅ Successfully connected to '%s' in %lu ms\n", ssid, millis() - start);
        print_connection_stats();
        Serial.println("[WiFi] =========================================\n");
        return true;
    }
    
    Serial.printf("[WiFi] ❌ Failed to connect to '%s' after %d ms\n", ssid, timeout_ms);
    Serial.printf("[WiFi] Final status: %d\n", WiFi.status());
    Serial.printf("[WiFi] Status changes observed: %d\n", status_change_count);
    Serial.println("[WiFi] =========================================\n");
    return false;
}

void WiFi_Manager::log_message(const char* msg) const {
    Serial.print("[WiFi] ");
    Serial.println(msg);
}

void WiFi_Manager::set_led(bool connected) const {
    digitalWrite(wifi_led, connected ? LOW : HIGH);
}

bool WiFi_Manager::sync_ntp(long gmtOffset_sec, int daylightOffset_sec, uint8_t timeoutSeconds) {
    if (WiFi.status() != WL_CONNECTED) {
        log_message("❌ Cannot sync NTP - WiFi not connected.");
        ntp_synced = false;
        return false;
    }

    log_message("Starting NTP sync...");
    Serial.println("[WiFi] NTP Servers: pool.ntp.org, time.nist.gov");
    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");

    uint8_t attempts = 0;
    uint8_t maxAttempts = timeoutSeconds/2;  // check every 500ms
    struct tm test_time;

    while (!getLocalTime(&test_time) && attempts < maxAttempts) {
        delay(500);
        attempts++;
        if (attempts % 4 == 0) { // Print every 2 seconds
            Serial.printf("[WiFi] ⏳ Waiting for NTP... (%d/%d attempts)\n", attempts, maxAttempts);
        }
    }

    if (!getLocalTime(&ntp_timeinfo)) {
        log_message("❌ NTP sync timed out.");
        Serial.printf("[WiFi] NTP failed after %d attempts\n", maxAttempts);
        ntp_synced = false;
        return false;
    }

    char buf[128];
    snprintf(buf, sizeof(buf), "✅ NTP synced: %04d-%02d-%02d %02d:%02d:%02d (UTC%+ld)",
             ntp_timeinfo.tm_year + 1900,
             ntp_timeinfo.tm_mon  + 1,
             ntp_timeinfo.tm_mday,
             ntp_timeinfo.tm_hour,
             ntp_timeinfo.tm_min,
             ntp_timeinfo.tm_sec,
             gmtOffset_sec / 3600);
    log_message(buf);
    
    Serial.printf("[WiFi] NTP Server: %s\n", "Synced");
    Serial.printf("[WiFi] Time source: %s\n", "Network");

    ntp_synced = true;
    return true;
}

// Helper method to get connection statistics
void WiFi_Manager::print_connection_stats() const {
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[WiFi] ========== CONNECTION STATS ==========");
        Serial.printf("[WiFi] SSID: %s\n", WiFi.SSID().c_str());
        Serial.printf("[WiFi] IP Address: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("[WiFi] Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
        Serial.printf("[WiFi] Subnet Mask: %s\n", WiFi.subnetMask().toString().c_str());
        Serial.printf("[WiFi] DNS: %s\n", WiFi.dnsIP().toString().c_str());
        Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
        Serial.printf("[WiFi] Channel: %d\n", WiFi.channel());
        Serial.printf("[WiFi] BSSID: %s\n", WiFi.BSSIDstr().c_str());
        Serial.printf("[WiFi] Consecutive Failures: %d\n", consecutive_failures);
        if (consecutive_failures >= 5) {
            Serial.printf("[WiFi] Current Backoff delay: %lu ms\n", backoff_delay_ms);
        }
        Serial.println("[WiFi] =========================================\n");
    } else {
        Serial.println("[WiFi] ❌ Not connected - cannot print stats\n");
    }
}

// ====== Wi-Fi Connect Cycle ======
bool connectKnownWiFi() {
    Serial.println("\n[WiFi] ===== LEGACY CONNECT FUNCTION =====");
    bool connected_state = false;
    
    WiFi.mode(WIFI_STA);
    delay(500);
    
    WiFi.disconnect(true);
    delay(500);

    Serial.println("[WiFi] Starting WiFi scan...");
    int n = WiFi.scanNetworks();
    Serial.printf("[WiFi] Scan returned: %d networks\n", n);
    
    if (n <= 0) {
        Serial.println("[WiFi] ❌ No networks found");
        return false;
    }

    int bestIndex = -1, bestRSSI = -999;
    Serial.println("[WiFi] Searching for known networks...");
    
    for (int i = 0; i < n; i++) {
        String foundSSID = WiFi.SSID(i);
        int32_t rssi = WiFi.RSSI(i);
        for (int j = 0; j < knownCount; j++) {
            if (foundSSID == knownNetworks[j].SECRET_SSID) {
                Serial.printf("[WiFi] Found: %s (RSSI: %d dBm)\n", foundSSID.c_str(), rssi);
                if (rssi > bestRSSI) { 
                    bestRSSI = rssi; 
                    bestIndex = j; 
                }
            }
        }
    }

    if (bestIndex == -1) {
        Serial.println("[WiFi] ❌ No known networks found");
        return false;
    }

    Serial.printf("[WiFi] Connecting to: %s (RSSI: %d dBm)\n", 
                 knownNetworks[bestIndex].SECRET_SSID, bestRSSI);
    
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
    delay(200);
    
    WiFi.begin(knownNetworks[bestIndex].SECRET_SSID, knownNetworks[bestIndex].SECRET_PASS);
    delay(500);
    
    unsigned long startAttempt = millis();
    int last_status = -1;
    
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
        delay(250);
        yield();
        
        int status = WiFi.status();
        if (status != last_status) {
            const char* status_str;
            switch(status) {
                case WL_NO_SSID_AVAIL: status_str = "NO_SSID_AVAIL"; break;
                case WL_CONNECT_FAILED: status_str = "CONNECT_FAILED"; break;
                case WL_IDLE_STATUS: status_str = "IDLE"; break;
                case WL_DISCONNECTED: status_str = "DISCONNECTED"; break;
                default: status_str = "UNKNOWN";
            }
            Serial.printf("[WiFi] Status: %s (%d) @ %lu ms\n", status_str, status, millis() - startAttempt);
            last_status = status;
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("[WiFi] ✅ Connected successfully!");
        connected_state = true;
    } else {
        Serial.println("[WiFi] ❌ Connection failed");
        connected_state = false;
    }

    Serial.println("[WiFi] =========================================\n");
    return connected_state;
}
