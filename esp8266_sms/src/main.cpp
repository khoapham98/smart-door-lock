#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "twilio.hpp"
#include "esp_uart.h"

const char* ssid     = "YOUR WIFI SSID";
const char* password = "YOUR WIFI PASSWORD";

const char* accountSID  = "YOUR TWILIO ACCOUNT SID";
const char* authToken   = "YOUR TWILIO AUTH TOKEN";
const char* fingerprint = "XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX";

const String fromNumber = "+<YOUR TWILIO PHONE NUMBER>";  // Twilio number
const String toNumber   = "+<COUNTRY CODE><YOUR PHONE NUMBER>";  // Destination number

Twilio twilio(accountSID, authToken, fingerprint);

void setup() 
{
  ESP_UART_Init();
  WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("WiFi connected!");
}

void loop() 
{
  String msg = recv_msg();
  if (msg.length() > 0) 
  {
    Serial.println("Received: " + msg);
    // Send SMS with the received message as body
    String response;
    bool success = twilio.send_message(toNumber, fromNumber, msg, response, "");
    if (success) 
    {
      Serial.println("SMS sent: " + msg);
    } 
    else 
    {
      Serial.println("SMS failed: " + response);
    }
  }
  delay(100);
}