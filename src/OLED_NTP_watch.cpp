/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-date-time-ntp-client-server-arduino/

  This code is modified to:
  - Display an analog or digital clock on an SSD1306 OLED.
  - Toggle between faces with a button on GPIO 0.
  - Use a built-in GFX font for the digital clock.
  - Drive a NeoPixel on GPIO 48 as a smooth green-to-red 60-second color timer.
  - Set NeoPixel brightness to 10/255.
  - Draw analog tick marks correctly on the rectangular border, which fills the entire screen.
*/

#include <WiFi.h>
#include <esp_wifi.h>
#include "time.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <math.h> // For sin, cos, and PI

// Include the built-in monospace font from the GFX library
#include <Fonts/FreeMonoBold9pt7b.h>

// OLED parameters
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C

// I2C and Button pins
#define OLED_SDA 8
#define OLED_SCL 9
#define BUTTON_PIN 0

// NeoPixel parameters
#define NEOPIXEL_PIN 48
#define NUM_PIXELS   1

// Object Instantiation
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Multi-network credentials
const int NUM_NETWORKS = 3;
const char* WIFI_SSIDS[NUM_NETWORKS] = {
    "Pixel_OF13",
    "WiFi-Rguez-Moya",
    "MOYA-LAPTOP 3439"
};
const char* WIFI_PASSWORDS[NUM_NETWORKS] = {
    "mynameisjeff",
    "Trece131313!",
    "00v44Q2["
};

// Network and Time configuration
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

// Global state and timing variables
volatile bool showAnalogClock = true;
struct tm timeinfo;
unsigned long lastSecondTick = 0;
bool timeSynced = false; // Flag to check if we successfully synced time

// Function Prototypes
void updateOledDisplay();
void updateNeoPixelColor();
void drawAnalogClock(int hour, int minute, int second);
void drawDigitalClock(int hour, int minute, int second);
void getRectPerimeterPoint(float distance, float x, float y, float w, float h, float& px, float& py);

// Interrupt Service Routine for button press
void IRAM_ATTR button_ISR() {
  showAnalogClock = !showAnalogClock;
}

void setup(){
  Serial.begin(115200);

  // Initialize I2C for OLED
  Wire.begin(OLED_SDA, OLED_SCL);

  // Initialize the OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.display();

  // Initialize NeoPixel
  pixels.begin();
  pixels.setBrightness(10); // Set brightness to 10 (out of 255)
  pixels.clear();
  pixels.show();

  // Setup button with interrupt
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_ISR, FALLING);

  // Connect to Wi-Fi
  // --- Multi-Network Connection Logic ---
  bool wifi_connected = false;
  Serial.println("Attempting to connect to available Wi-Fi networks...");
  WiFi.mode(WIFI_STA); // Set to station mode

  esp_wifi_set_max_tx_power(40);

  for (int i = 0; i < NUM_NETWORKS; i++) {
    const char* current_ssid = WIFI_SSIDS[i];
    const char* current_password = WIFI_PASSWORDS[i];

    Serial.print("Trying SSID: ");
    Serial.println(current_ssid);

    WiFi.begin(current_ssid, current_password);

    int attempts = 0;
    // Wait up to 10 seconds (20 attempts * 500ms) for connection
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("\nSuccessfully connected to: ");
      Serial.println(current_ssid);
      wifi_connected = true;
      break; // Exit the for loop upon success
    } else {
      Serial.println("\nFailed to connect to this network.");
      // Stop trying this network and prepare for the next
      WiFi.disconnect();
      delay(100); // Small pause before next attempt
    }
  }

  if (!wifi_connected) {
    Serial.println("FATAL: Could not connect to any specified Wi-Fi network. Clock time will be inaccurate.");
  }

  // Init and get time (only proceed if connected)
  if (wifi_connected) {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain initial time");
    } else {
      Serial.println("Initial time obtained.");
      Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
      timeSynced = true;
    }

    // Disconnect WiFi to save power (as per original logic)
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  // Initialize timing variable
  lastSecondTick = millis();
  updateOledDisplay(); // Initial display draw
}

void loop(){
  unsigned long currentMillis = millis();

  // --- Task 1: Update Time and OLED Display (once per second) ---
  if (currentMillis - lastSecondTick >= 1000) {
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
    } else {
      updateOledDisplay();
    }
    lastSecondTick += 1000;
  }

  // --- Task 2: Update NeoPixel Color (as fast as possible for smoothness) ---
  updateNeoPixelColor();
}

void updateNeoPixelColor() {
  unsigned long millisIntoMinute = (timeinfo.tm_sec * 1000) + (millis() - lastSecondTick);
  millisIntoMinute = constrain(millisIntoMinute, 0, 59999);

  uint8_t red   = map(millisIntoMinute, 0, 59999, 0, 255);
  uint8_t green = map(millisIntoMinute, 0, 59999, 255, 0);

  pixels.setPixelColor(0, pixels.Color(red, green, 0));
  pixels.show();
}

void updateOledDisplay() {
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  if (showAnalogClock) {
    drawAnalogClock(timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    drawDigitalClock(timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }
}

void drawDigitalClock(int hour, int minute, int second) {
  display.clearDisplay();
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  char timeString[9];
  sprintf(timeString, "%02d:%02d:%02d", hour, minute, second);
  
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(timeString, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2 + h);

  display.print(timeString);
  display.display();
}

// Helper function to find a point on the perimeter of a rectangle
void getRectPerimeterPoint(float distance, float x, float y, float w, float h, float& px, float& py) {
    // The drawable area is w-1 and h-1
    float drawn_w = w - 1;
    float drawn_h = h - 1;
    float perimeter = 2 * (drawn_w + drawn_h);
    distance = fmod(distance, perimeter);

    if (distance < drawn_w) { // Top edge
        px = x + distance;
        py = y;
    } else if (distance < drawn_w + drawn_h) { // Right edge
        px = x + drawn_w;
        py = y + (distance - drawn_w);
    } else if (distance < 2 * drawn_w + drawn_h) { // Bottom edge
        px = x + drawn_w - (distance - (drawn_w + drawn_h));
        py = y + drawn_h;
    } else { // Left edge
        px = x;
        py = y + drawn_h - (distance - (2 * drawn_w + drawn_h));
    }
}

// Function to draw the rectangular analog clock
void drawAnalogClock(int hour, int minute, int second) {
  display.clearDisplay();
  display.setFont();

  // Make the clock face fill the entire screen
  int clockX = 0;
  int clockY = 0;
  int clockWidth = SCREEN_WIDTH;
  int clockHeight = SCREEN_HEIGHT;
  display.drawRect(clockX, clockY, clockWidth, clockHeight, SSD1306_WHITE);

  int centerX = clockX + clockWidth / 2;
  int centerY = clockY + clockHeight / 2;
  
  // Draw 12 evenly-spaced hour ticks on the rectangular contour
  const int tickLength = 4;
  float perimeter = 2.0 * ((clockWidth - 1) + (clockHeight - 1));
  float hour_spacing = perimeter / 12.0;
  float start_offset = (float)(clockWidth - 1) / 2.0; // Start at 12 o'clock (top center)

  for (int i = 0; i < 12; i++) {
      float distance = fmod(start_offset + i * hour_spacing, perimeter);
      float outerX, outerY;
      
      getRectPerimeterPoint(distance, (float)clockX, (float)clockY, (float)clockWidth, (float)clockHeight, outerX, outerY);

      // Draw line inward toward the center
      float dirX = centerX - outerX;
      float dirY = centerY - outerY;
      float len = sqrt(dirX * dirX + dirY * dirY);
      if (len > 0) {
          dirX /= len;
          dirY /= len;
      }
      float innerX = outerX + dirX * tickLength;
      float innerY = outerY + dirY * tickLength;
      
      display.drawLine((int16_t)outerX, (int16_t)outerY, (int16_t)innerX, (int16_t)innerY, SSD1306_WHITE);
  }

  // Calculate angles for hands
  float secAngle = (second * 6.0) * PI / 180.0;
  float minAngle = ((minute * 6.0) + (second * 0.1)) * PI / 180.0;
  float hourAngle = (((hour % 12) * 30.0) + (minute * 0.5)) * PI / 180.0;

  // Draw hands (their tips still form an ellipse, which is natural)
  int secX = centerX + (int)((clockWidth / 2 - 4) * sin(secAngle));
  int secY = centerY - (int)((clockHeight / 2 - 4) * cos(secAngle));
  display.drawLine(centerX, centerY, secX, secY, SSD1306_WHITE);

  int minX = centerX + (int)((clockWidth / 2 - 8) * sin(minAngle));
  int minY = centerY - (int)((clockHeight / 2 - 8) * cos(minAngle));
  display.drawLine(centerX, centerY, minX, minY, SSD1306_WHITE);

  int hourX = centerX + (int)((clockWidth / 2 - 14) * sin(hourAngle));
  int hourY = centerY - (int)((clockHeight / 2 - 14) * cos(hourAngle));
  display.drawLine(centerX, centerY, hourX, hourY, SSD1306_WHITE);

  display.fillCircle(centerX, centerY, 2, SSD1306_WHITE);
  display.display();
}
