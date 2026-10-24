#ifndef WIFI_HELPER_HPP
#define WIFI_HELPER_HPP

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

// WiFi credentials, server address and robot id (not in the repository,
// copy config.example.hpp to config.hpp)
#include "config.hpp"


inline void initWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
}

inline String httpGet(const String& path) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(String(SERVER_URL) + path);
        
        int httpResponseCode = http.GET();
        String payload = "";

        if (httpResponseCode > 0) {
            payload = http.getString();
            payload.trim();
        } else {
            Serial.printf("HTTP error: %d\n", httpResponseCode);
        }
        
        http.end();
        return payload;
    }
    return "DISCONNECTED";
}

// Server reply: "<id> <COMMAND>", id = 0 when there is nothing to do
inline String fetchCommand() {
    return httpGet("/command?robot=" ROBOT_ID);
}

// If the confirmation gets lost, the server sends the same command again;
// the robot recognises the duplicate by its id and only confirms it again.
inline void sendDone(long id) {
    httpGet(String("/done?robot=" ROBOT_ID "&id=") + String(id));
}

#endif