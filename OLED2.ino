// Include essential libraries
#include <Wire.h>
#include <Arduino.h>
#include <QMC5883LCompass.h>
#include <U8g2lib.h> 

#define RADIOSTATION 6     // Button pin
#define RADIOADDRESS 0x60  // TEA5767 I2C address
#define AUDIOPIN A0        // Pin for ADC 

const uint16_t samples = 16; // 16 samples for the audio waveform
const double samplingFrequency = 20000; // Frequency for the sampling
double vReal[samples];  // Array storing analog audio samples 

const float stationFrequency[] = {99.9, 97.3, 94.1, 93.5, 98.7}; // Store radio stations
const int stationCount = sizeof(stationFrequency) / sizeof(stationFrequency[0]); // Track current station
int currentStation = 0;  // Radio begins at first radio station (99.9) 

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // OLED I2C Initialization

void setup() {
  Serial.begin(9600);                             // Set baudrate to 9600
  u8g2.begin();                                  // Begin initalization of OLED
  analogReference(DEFAULT);                      // Set the reference voltage (5 V) for wave
  delay(1000);                                  // Allow Serial to initialize properly
  Wire.begin();                                 // Begin I2C process
  pinMode(RADIOSTATION, INPUT);                 // Set button as input
  Serial.println("Radio");                      // Print "radio"
  tuneRadio(stationFrequency[currentStation]); // Start at default station
}

void tuneRadio(double freq) {
  unsigned char buffer[5] = {0};            // Buffer of 5 elements set to 0 
  unsigned int internalFrequency = 4 * (freq * 1e6 - 225e3) / 32768; // Calculate bits for radio

  buffer[0] = (internalFrequency >> 8);     // Set the high byte 
  buffer[1] = lowByte(internalFrequency);   // Set the low byte
  buffer[2] = 0b01101000;                   // Enable different features of radio
  buffer[3] = 0b00011110;                   // Enable different features of radio 

  Wire.beginTransmission(RADIOADDRESS);     // Begin I2C communication
  Wire.write(buffer, 5);                    // Send 5 bytes
  Wire.endTransmission();                   // End I2C communication

  Serial.print("Tuned to: ");               // Print "tuned to:"
  Serial.println(freq);                     // Print the radio station    
}

void generateWave() {
  for (int i = 0; i < samples; i++) {       // Capture ADC readings
    vReal[i] = analogRead(AUDIOPIN) - 512; // Center around 0
    delayMicroseconds(1000000 / samplingFrequency); // Define delay 
  }

  u8g2.clearBuffer();                     // Clear the OLED screen
  u8g2.setFont(u8g2_font_ncenB08_tr);     // Set the font
  u8g2.drawStr(10, 10, "Audio Waveform"); // Print "audio waveform"

  int centerY = 32; //Center waveform at y = 32 
  int xStart = 50; // Shift right by 10 pixels

  for (int i = 0; i < samples; i++) {                   // Map the ADC values on OLED
    int barHeight = map(vReal[i], -512, 512, -20, 20); // Set dimensions
    int xPos = xStart + (i * 2);                       // Shift bars to the center
    u8g2.drawLine(xPos, centerY, xPos, centerY - barHeight); // Draw bars
  }

  u8g2.sendBuffer(); // Send display data to OLED
}


void loop() {
  static bool previousState = HIGH;        // Define previous state as HIGH
  static unsigned long lastDebounce = 0;   // Last debounce is 0 
  const unsigned long debounceDelay = 200; // Create debounce delay of 200

  bool buttonState = digitalRead(RADIOSTATION);  // Read the button
  generateWave();                                // Generate audio wave

  if (buttonState == LOW && previousState == HIGH) { // Detect button press
    if (millis() - lastDebounce > debounceDelay) { // Set button debounce
      currentStation = (currentStation + 1) % stationCount; // Go to next station
      Serial.println("Changing Station"); // Change radio station
      tuneRadio(stationFrequency[currentStation]); // Tune radio to new station
      lastDebounce = millis();        // Track the ms passed
    }
  }
  previousState = buttonState;        // Resetting button press 
}
