#ifndef CREDENTIALS_H
#define CREDENTIALS_H

// Structure for WiFi credentials
struct WiFiCred {
  const char* SECRET_SSID;
  const char* SECRET_PASS;
};

// List of known WiFi networks
const WiFiCred knownNetworks[] = {
  {"MUARIK_DRYER_ONLINE", "MUARIK@2026"},
  {"IntelliSys Air", "intel_cool@2025!"},
  {"IntelliSys Air_5G", "intel_cool@2025!"},
  {"irrikit_Cloud", "IntelliSys@2025!"},
  {"IntelliSys Online", "qwerty0976_"},
  {"IntelliSys Pro 2025", "intel_cool@2025"},
  {"IntelliSys Cloud", "IntelliSys@2025!"},
  {"iPaul", "wrongpassword"}
};
const int knownCount = sizeof(knownNetworks) / sizeof(knownNetworks[0]);


#define SECRET_CH_ID 3016343  //  2556889  if for Write to Rwebitaba\s Weather Data channel
#define SECRET_WRITE_APIKEY "M685BTJ8CJDC5ILP" //"6FX2CZJ97H3K3MMB"   // Write API key for Things

class OTA_Credentials {
public: 
    const char* secure_ota_password = "1A2B";

};

#endif


