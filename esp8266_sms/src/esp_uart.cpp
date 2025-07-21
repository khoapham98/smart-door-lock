#include <Arduino.h>
#include "esp_uart.h"

String recv_msg()
{
    if (!Serial.available()) return ""; 
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    return msg;
}

void ESP_UART_Init()
{
    Serial.begin(9600, SERIAL_8E1);
}