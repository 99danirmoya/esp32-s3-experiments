/*
 * Rui Santos
 * Complete project details at https://RandomNerdTutorials.com/esp32-date-time-ntp-client-server-arduino/
 *
 * This code is modified to:
 * - Display an analog or digital clock on an SSD1306 OLED.
 * - Toggle between faces with a button on GPIO 0.
 * - USE CUSTOM DRAWING PRIMITIVES for the digital clock to ensure a perfect 7-segment look.
 * - Drive a NeoPixel on GPIO 48 as a smooth green-to-red 60-second color timer.
 * - Set NeoPixel brightness to 10/255.
 * - Draw analog tick marks correctly on the rectangular border, which fills the entire screen.
 *
 * --- MODIFIED TO ADD ALARM FUNCTIONALITY ---
 * - Uses std::vector for a dynamic list of alarms.
 * - Alarms trigger on the o'clock hour.
 * - Plays a non-blocking, multi-frequency beep pattern on BUZZER_PIN 7 using LEDC.
 * - Button on PIN 0 disables the *current* alarm when active, otherwise toggles clock.
 */

#include <WiFi.h>
#include <esp_wifi.h>
#include "time.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <math.h> // For sin, cos, and PI
#include <vector> // --- NEW: For a dynamic array of alarms ---

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

// --- NEW: Alarm and Buzzer Definitions ---
#define BUZZER_PIN 7
const int LEDC_CHANNEL = 0;      // ESP32 LEDC channel 0 for buzzer
const int LEDC_RESOLUTION = 8;     // 8-bit resolution
const int LEDC_BASE_FREQ = 5000;   // Base frequency for LEDC setup
const int MIN_ALARM_FREQ = 200;    // Starting (low) beep pitch
const int MAX_ALARM_FREQ = 1500;   // Max (moderate-high) beep pitch
const int TOTAL_ALARM_DURATION_MS = 60000; // 60 seconds in milliseconds
const int ALARM_CYCLE_MS = 2500;           // 0.5+0.5+0.5+1.0 seconds = 2500 ms
const int TOTAL_FREQ_STEPS = TOTAL_ALARM_DURATION_MS / ALARM_CYCLE_MS; // 60000 / 2500 = 24
const int FREQ_INCREMENT = 55;             // (1500 - 200) / 24 steps = 54.17. We use 55 Hz.

// Object Instantiation
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Multi-network credentials (Replace with your own if needed)
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

// Network and Time configuration (For CET/CEST time zone)
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;      // 1 hour (Central European Time)
const int   daylightOffset_sec = 3600; // 1 hour for Daylight Saving Time

// Global state and timing variables
volatile bool showAnalogClock = true;
struct tm timeinfo;
unsigned long lastSecondTick = 0;
bool timeSynced = false; // Flag to check if we successfully synced time

// --- MODIFIED: Alarm System Globals ---
struct Alarm {
  int hour;   // 0-23
  int minute; // *** NEW: 0-59 ***
  bool enabled;
};
std::vector<Alarm> alarms; // Dynamic array (vector) for alarms

volatile bool isAlarmActive = false;      // True if alarm is currently sounding
volatile bool alarmDisableRequest = false; // Flag set by ISR to disable an alarm
int activeAlarmHour = -1;                 // Hour of the ringing alarm
int activeAlarmMinute = -1;               // *** NEW: Minute of the ringing alarm ***
unsigned long alarmStartTime = 0;         // millis() when the alarm started (for 1-min timeout)
unsigned long lastAlarmBeepTime = 0;      // millis() for the alarm beep state machine
int alarmState = 0;                       // State machine for beep pattern (0-4)
int currentFrequency = MIN_ALARM_FREQ;    // Current beep pitch


// --- Custom 7-Segment Drawing Variables and Functions ---
// ... (All your existing 7-segment functions: digit_array, render_digit, render_colon, render_time_string) ...
// [No changes to this section]

static const int digit_array[10][7] = {
    {1, 1, 1, 1, 1, 1, 0},  // 0
    {0, 1, 1, 0, 0, 0, 0},  // 1
    {1, 1, 0, 1, 1, 0, 1},  // 2
    {1, 1, 1, 1, 0, 0, 1},  // 3
    {0, 1, 1, 0, 0, 1, 1},  // 4
    {1, 0, 1, 1, 0, 1, 1},  // 5
    {1, 0, 1, 1, 1, 1, 1},  // 6
    {1, 1, 1, 0, 0, 0, 0},  // 7
    {1, 1, 1, 1, 1, 1, 1},  // 8
    {1, 1, 1, 0, 0, 1, 1}   // 9
};
const int DIGIT_H = 20;
const int SEG_THICKNESS = 2;
const int SEG_LEN_H = 6;
const int SEG_LEN_V = 7;
const int DIGIT_W_RENDER = SEG_LEN_H + 2 * SEG_THICKNESS;
const int COLON_WIDTH = 4;
const int CHAR_SPACING = 0;
const int SEG_ROUNDNESS = 1;
const int DIGIT_ADVANCE = DIGIT_W_RENDER + 2;
const int COLON_ADVANCE = 4;

void render_digit(int16_t pos_x, int16_t pos_y, uint8_t digit, uint8_t color) {
    if (digit > 9) return;
    for (uint8_t i = 0; i < 7; i++) {
        bool seg_on = digit_array[digit][i];
        if (seg_on) {
            switch (i) {
                case 0: display.fillRoundRect(pos_x + SEG_THICKNESS, pos_y, SEG_LEN_H, SEG_THICKNESS, SEG_ROUNDNESS, color); break;
                case 1: display.fillRoundRect(pos_x + SEG_THICKNESS + SEG_LEN_H, pos_y + SEG_THICKNESS, SEG_THICKNESS, SEG_LEN_V, SEG_ROUNDNESS, color); break;
                case 2: display.fillRoundRect(pos_x + SEG_THICKNESS + SEG_LEN_H, pos_y + 2*SEG_THICKNESS + SEG_LEN_V, SEG_THICKNESS, SEG_LEN_V, SEG_ROUNDNESS, color); break;
                case 3: display.fillRoundRect(pos_x + SEG_THICKNESS, pos_y + DIGIT_H - SEG_THICKNESS, SEG_LEN_H, SEG_THICKNESS, SEG_ROUNDNESS, color); break;
                case 4: display.fillRoundRect(pos_x, pos_y + 2*SEG_THICKNESS + SEG_LEN_V, SEG_THICKNESS, SEG_LEN_V, SEG_ROUNDNESS, color); break;
                case 5: display.fillRoundRect(pos_x, pos_y + SEG_THICKNESS, SEG_THICKNESS, SEG_LEN_V, SEG_ROUNDNESS, color); break;
                case 6: display.fillRoundRect(pos_x + SEG_THICKNESS, pos_y + SEG_THICKNESS + SEG_LEN_V, SEG_LEN_H, SEG_THICKNESS, SEG_ROUNDNESS, color); break;
            }
        }
    }
}

void render_colon(int16_t pos_x, int16_t pos_y, uint8_t color) {
    int dot_size = SEG_THICKNESS;
    const int VIRTUAL_COLON_WIDTH = 4; 
    int dot_spacing = DIGIT_H / 4;
    int top_dot_y = pos_y + dot_spacing;
    int bottom_dot_y = pos_y + DIGIT_H - dot_spacing - dot_size;
    int dot_x = pos_x + (VIRTUAL_COLON_WIDTH - dot_size) / 2;
    display.fillRect(dot_x, top_dot_y, dot_size, dot_size, color);
    display.fillRect(dot_x, bottom_dot_y, dot_size, dot_size, color);
}

void render_time_string(int hour, int minute, int second, uint8_t color) {
    const int total_width = (5 * DIGIT_ADVANCE) + (2 * COLON_ADVANCE) + DIGIT_W_RENDER;
    int16_t x_cursor = (SCREEN_WIDTH - total_width) / 2;
    int16_t y_pos = (SCREEN_HEIGHT - DIGIT_H) / 2;

    render_digit(x_cursor, y_pos, hour / 10, color);
    x_cursor += DIGIT_ADVANCE;
    render_digit(x_cursor, y_pos, hour % 10, color);
    x_cursor += DIGIT_ADVANCE;

    if (second % 2 == 0) {
        render_colon(x_cursor, y_pos, color);
    }
    x_cursor += COLON_ADVANCE;

    render_digit(x_cursor, y_pos, minute / 10, color);
    x_cursor += DIGIT_ADVANCE;
    render_digit(x_cursor, y_pos, minute % 10, color);
    x_cursor += DIGIT_ADVANCE;

    render_colon(x_cursor, y_pos, color);
    x_cursor += COLON_ADVANCE;

    render_digit(x_cursor, y_pos, second / 10, color);
    x_cursor += DIGIT_ADVANCE;
    render_digit(x_cursor, y_pos, second % 10, color);
}

// --- End of Custom 7-Segment Drawing Functions ---


// Function Prototypes
void updateOledDisplay();
void updateNeoPixelColor();
void drawAnalogClock(int hour, int minute, int second);
void drawDigitalClock(int hour, int minute, int second);
void getRectPerimeterPoint(float distance, float x, float y, float w, float h, float& px, float& py);

// --- NEW: Alarm Function Prototypes ---
void setupAlarms();
void checkAlarmTrigger();
void handleAlarmSound();
void checkAlarmDisableRequest();


// --- MODIFIED: Interrupt Service Routine for button press ---
/**
 * @brief ISR for the button on GPIO 0.
 * If an alarm is active, it sets a flag to disable that alarm.
 * If no alarm is active, it toggles the clock face.
 */
void IRAM_ATTR button_ISR() {
  if (isAlarmActive) {
    // Don't do heavy work in ISR! Just set a flag.
    alarmDisableRequest = true;
  } else {
    // Original functionality
    showAnalogClock = !showAnalogClock;
  }
}

// --- NEW: Function to populate the alarms vector ---
/**
 * @brief Adds alarms to the global vector.
 * Alarms are defined by hour (0-23) and enabled status.
 */
void setupAlarms() {
// --- NEW Format: { hour, minute, enabled } ---
  alarms.push_back({23, 20, true});   // Add an alarm for 8:30 AM
  alarms.push_back({23, 22, true});   // Add an alarm for 2:05 PM
  alarms.push_back({6, 0, true});   // Add an alarm for 10:00 PM (on the hour)

  Serial.print("Initialized ");
  Serial.print(alarms.size());
  Serial.println(" alarms.");
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

  // --- NEW: Setup Buzzer Pin and LEDC Channel ---
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(LEDC_CHANNEL, LEDC_BASE_FREQ, LEDC_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, LEDC_CHANNEL);
  ledcWriteTone(LEDC_CHANNEL, 0); // Ensure buzzer is off
  
  // --- NEW: Load the alarms into memory ---
  setupAlarms();

  // Connect to Wi-Fi
  // ... (Existing multi-network connection logic) ...
  bool wifi_connected = false;
  Serial.println("Attempting to connect to available Wi-Fi networks...");
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(40);
  for (int i = 0; i < NUM_NETWORKS; i++) {
    const char* current_ssid = WIFI_SSIDS[i];
    const char* current_password = WIFI_PASSWORDS[i];
    Serial.print("Trying SSID: ");
    Serial.println(current_ssid);
    WiFi.begin(current_ssid, current_password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("\nSuccessfully connected to: ");
      Serial.println(current_ssid);
      wifi_connected = true;
      break;
    } else {
      Serial.println("\nFailed to connect to this network.");
      WiFi.disconnect();
      delay(100);
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
    // Disconnect WiFi to save power
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }

  // Initialize timing variable
  lastSecondTick = millis();
  updateOledDisplay(); // Initial display draw
}

/**
 * @brief Main application loop.
 */
void loop(){
  unsigned long currentMillis = millis();

  // --- Task 1: Check for user alarm disable request (from ISR) ---
  checkAlarmDisableRequest();

  // --- Task 2: Handle active alarm sound generation (non-blocking) ---
  handleAlarmSound();

  // --- Task 3: Update Time, OLED, and Check for Alarm Trigger (once per second) ---
  if (currentMillis - lastSecondTick >= 1000) {
    bool timeUpdated = false;
    
    if(!getLocalTime(&timeinfo)){
      // Handle time manually if NTP fails after initial sync
      if(timeSynced) {
        timeinfo.tm_sec++;
        if (timeinfo.tm_sec >= 60) { timeinfo.tm_sec = 0; timeinfo.tm_min++; }
        if (timeinfo.tm_min >= 60) { timeinfo.tm_min = 0; timeinfo.tm_hour++; }
        if (timeinfo.tm_hour >= 24) { timeinfo.tm_hour = 0; }
        
        updateOledDisplay();
        timeUpdated = true;
      } else {
         Serial.println("Failed to obtain time");
      }
    } else {
      updateOledDisplay();
      timeUpdated = true;
    }

    // --- NEW ALARM TRIGGER LOGIC ---
    // Only check for a new alarm if time was successfully updated
    // and an alarm is not already active.
    if (timeUpdated && !isAlarmActive) {
      checkAlarmTrigger();
    }
    // --- END NEW LOGIC ---

    lastSecondTick += 1000;
  }

  // --- Task 4: Update NeoPixel Color (as fast as possible for smoothness) ---
  updateNeoPixelColor();
}

// --- NEW: Function to check and handle alarm disable request ---
/**
 * @brief Checks if the ISR has requested an alarm disable.
 * If so, it stops the sound and permanently disables the
 * alarm that was ringing.
 */
void checkAlarmDisableRequest() {
  if (alarmDisableRequest) {
    alarmDisableRequest = false; // Clear the flag
    
    if (isAlarmActive) {
      Serial.print("Disabling alarm for: ");
      Serial.printf("%02d:%02d\n", activeAlarmHour, activeAlarmMinute);
      
      isAlarmActive = false; // Stop the sound
      ledcWriteTone(LEDC_CHANNEL, 0); // Explicitly stop sound

      // --- MODIFIED: Find and disable the exact H:M match ---
      for (auto& alarm : alarms) {
        if (alarm.hour == activeAlarmHour && alarm.minute == activeAlarmMinute) {
          alarm.enabled = false;
          Serial.println("Alarm disabled.");
          break;
        }
      }
      activeAlarmHour = -1;
      activeAlarmMinute = -1; // Reset minute tracker
    }
  }
}

// --- NEW: Function to check if an alarm should start ---
/**
 * @brief Checks if the current time (at the top of the hour)
 * matches any enabled alarm.
 */
void checkAlarmTrigger() {
  // Check only at the top of the minute (00 seconds) to prevent re-triggering
  if (timeinfo.tm_sec == 0) {
    /*
    // Debug: Uncomment this to see the check every minute
    Serial.print("Checking for alarm at: ");
    Serial.printf("%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min);
    */

    for (const auto& alarm : alarms) {
      // --- MODIFIED: Check for hour, minute, and enabled status ---
      if (alarm.hour == timeinfo.tm_hour && 
          alarm.minute == timeinfo.tm_min && 
          alarm.enabled) {
            
        Serial.printf("!!! ALARM TRIGGERED for %02d:%02d !!!\n", alarm.hour, alarm.minute);
        isAlarmActive = true;
        activeAlarmHour = alarm.hour;   // Store which alarm is ringing
        activeAlarmMinute = alarm.minute; // Store which alarm is ringing
        alarmStartTime = millis();        // For 1-min timeout
        alarmState = 0;                   // Reset state machine
        currentFrequency = MIN_ALARM_FREQ; // Reset frequency
        lastAlarmBeepTime = millis();     // Start immediately
        break; // Only trigger one alarm
      }
    }
  }
}

// --- NEW: Function to generate the alarm sound pattern ---

/**
 * @brief Non-blocking state machine to play the alarm sound.
 * Must be called continuously from the main loop.
 */
void handleAlarmSound() {
  // If no alarm is active, do nothing.
  if (!isAlarmActive) {
    return;
  }

  unsigned long now = millis();

// Check for alarm timeout (1 minute)
  if (now - alarmStartTime > 60000) {
    Serial.println("Alarm finished (1 min timeout).");
    isAlarmActive = false;
    activeAlarmHour = -1;
    activeAlarmMinute = -1; // --- ADD THIS LINE ---
    ledcWriteTone(LEDC_CHANNEL, 0); // Stop sound
    return;
  }

  // State Machine for Beep Pattern:
  // 0: Start Beep 1
  // 1: In Beep 1 (wait 0.5s)
  // 2: In Silence 1 (wait 0.5s)
  // 3: In Beep 2 (wait 0.5s)
  // 4: In Silence 2 (wait 1.0s) -> loop to 0
  
  switch (alarmState) {
    case 0: // Start Beep 1
      ledcWriteTone(LEDC_CHANNEL, currentFrequency);
      lastAlarmBeepTime = now;
      alarmState = 1;
      break;
    
    case 1: // In Beep 1, wait for 0.5s
      if (now - lastAlarmBeepTime >= 500) {
        ledcWriteTone(LEDC_CHANNEL, 0); // Start Silence 1
        lastAlarmBeepTime = now;
        alarmState = 2;
      }
      break;

    case 2: // In Silence 1, wait for 0.5s
      if (now - lastAlarmBeepTime >= 500) {
        ledcWriteTone(LEDC_CHANNEL, currentFrequency); // Start Beep 2
        lastAlarmBeepTime = now;
        alarmState = 3;
      }
      break;

    case 3: // In Beep 2, wait for 0.5s
      if (now - lastAlarmBeepTime >= 500) {
        ledcWriteTone(LEDC_CHANNEL, 0); // Start Silence 2 (long)
        lastAlarmBeepTime = now;
        alarmState = 4;
        
        // Increase frequency for the *next* pair
        currentFrequency += FREQ_INCREMENT;
        
        // --- MODIFIED: Ensure we don't exceed the max pitch, but DO NOT RESET. ---
        if (currentFrequency > MAX_ALARM_FREQ) {
            currentFrequency = MAX_ALARM_FREQ; // Cap it at the maximum pitch
        }
      }
      break;

    case 4: // In Silence 2, wait for 1.0s
      if (now - lastAlarmBeepTime >= 1000) {
        alarmState = 0; // Loop back to Start Beep 1
      }
      break;
  }
}


void updateNeoPixelColor() {
  // ... (Existing NeoPixel logic - no changes) ...
  if (!timeSynced) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
    return;
  }
  unsigned long millisIntoMinute = (timeinfo.tm_sec * 1000) + (millis() - lastSecondTick);
  millisIntoMinute = constrain(millisIntoMinute, 0, 59999);
  uint8_t red   = map(millisIntoMinute, 0, 59999, 0, 255);
  uint8_t green = map(millisIntoMinute, 0, 59999, 255, 0);
  pixels.setPixelColor(0, pixels.Color(red, green, 0));
  pixels.show();
}

void updateOledDisplay() {
  // ... (Existing OLED update logic - no changes) ...
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  if (showAnalogClock) {
    drawAnalogClock(timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    drawDigitalClock(timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }
}

void drawDigitalClock(int hour, int minute, int second) {
  // ... (Existing digital clock logic - no changes) ...
  display.clearDisplay();
  render_time_string(hour, minute, second, SSD1306_WHITE);
  display.display();
}

void getRectPerimeterPoint(float distance, float x, float y, float w, float h, float& px, float& py) {
  // ... (Existing analog clock helper - no changes) ...
  float drawn_w = w - 1;
  float drawn_h = h - 1;
  float perimeter = 2 * (drawn_w + drawn_h);
  distance = fmod(distance, perimeter);
  if (distance < drawn_w) {
    px = x + distance;
    py = y;
  } else if (distance < drawn_w + drawn_h) {
    px = x + drawn_w;
    py = y + (distance - drawn_w);
  } else if (distance < 2 * drawn_w + drawn_h) {
    px = x + drawn_w - (distance - (drawn_w + drawn_h));
    py = y + drawn_h;
  } else {
    px = x;
    py = y + drawn_h - (distance - (2 * drawn_w + drawn_h));
  }
}

void drawAnalogClock(int hour, int minute, int second) {
  // ... (Existing analog clock drawing logic - no changes) ...
  display.clearDisplay();
  display.setFont(NULL);
  int clockX = 0;
  int clockY = 0;
  int clockWidth = SCREEN_WIDTH;
  int clockHeight = SCREEN_HEIGHT;
  display.drawRect(clockX, clockY, clockWidth, clockHeight, SSD1306_WHITE);
  int centerX = clockX + clockWidth / 2;
  int centerY = clockY + clockHeight / 2;
  const int tickLength = 4;
  const int TICK_OFFSET = 5;
  float perimeter = 2.0 * ((clockWidth - 1) + (clockHeight - 1));
  float hour_spacing = perimeter / 12.0;
  float start_offset = (float)(clockWidth - 1) / 2.0;
  for (int i = 0; i < 12; i++) {
    float distance = fmod(start_offset + i * hour_spacing, perimeter);
    float edgeX, edgeY;
    getRectPerimeterPoint(distance, (float)clockX, (float)clockY, (float)clockWidth, (float)clockHeight, edgeX, edgeY);
    float dirX = centerX - edgeX;
    float dirY = centerY - edgeY;
    float len = sqrt(dirX * dirX + dirY * dirY);
    if (len > 0) {
      dirX /= len;
      dirY /= len;
    }
    float startX = edgeX + dirX * TICK_OFFSET;
    float startY = edgeY + dirY * TICK_OFFSET;
    float endX = startX + dirX * tickLength;
    float endY = startY + dirY * tickLength;
    display.drawLine((int16_t)startX, (int16_t)startY, (int16_t)endX, (int16_t)endY, SSD1306_WHITE);
  }
  float secAngle = (second * 6.0) * PI / 180.0;
  float minAngle = ((minute * 6.0) + (second * 0.1)) * PI / 180.0;
  float hourAngle = (((hour % 12) * 30.0) + (minute * 0.5)) * PI / 180.0;
  const int maxRadius = clockWidth / 2;
  int secX = centerX + (int)((maxRadius - 2) * sin(secAngle));
  int secY = centerY - (int)((maxRadius - 2) * cos(secAngle));
  display.drawLine(centerX, centerY, secX, secY, SSD1306_WHITE);
  int minX = centerX + (int)((maxRadius * 0.75) * sin(minAngle));
  int minY = centerY - (int)((maxRadius * 0.75) * cos(minAngle));
  display.drawLine(centerX, centerY, minX, minY, SSD1306_WHITE);
  int hourX = centerX + (int)((maxRadius * 0.5) * sin(hourAngle));
  int hourY = centerY - (int)((maxRadius * 0.5) * cos(hourAngle));
  display.drawLine(centerX, centerY, hourX, hourY, SSD1306_WHITE);
  display.fillCircle(centerX, centerY, 2, SSD1306_WHITE);
  display.display();
}
