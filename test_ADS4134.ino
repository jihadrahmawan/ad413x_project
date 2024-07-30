#include <Arduino.h>
#include "AD4134.h"

const int csPin = 10; // Define the Chip Select pin
AD4134 ad4134(csPin);

void setup() {
    Serial.begin(9600);
    while (!Serial); // Wait for Serial to initialize
    Serial.println("Starting setup...");
    delay(4000);

    ad4134.begin();
    ad4134.configure();

    Serial.println("Setup complete.");
}

void loop() {
    uint32_t adcValue = ad4134.readADC(0); // Read from channel 0
    Serial.print("ADC Channel 0 Value: ");
    Serial.println(adcValue);
    delay(1000); // Read every second
}
