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
 */

#include <WiFi.h>
#include <esp_wifi.h>
#include "time.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <math.h> // For sin, cos, and PI

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

// --- Custom 7-Segment Drawing Variables and Functions ---

// Define segment truth table for each digit (a-g)
// Segments are indexed 0 (a) to 6 (g)
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

// --- Segment Dimensions (ADJUSTED FOR 128x64 SCREEN SIZE) ---
const int DIGIT_H = 20;        // Total height of one digit bounding box (Reduced from 30)
const int SEG_THICKNESS = 2;   // Thickness of the segment line (Kept at 2)
const int SEG_LEN_H = 6;       // Length of horizontal segments (A, D, G) (Reduced from 18)
const int SEG_LEN_V = 7;       // Length of vertical segments (B, C, E, F) (Reduced from 28)
const int DIGIT_W_RENDER = SEG_LEN_H + 2 * SEG_THICKNESS; // Total rendered width of digit (10)
const int COLON_WIDTH = 4;     // Width of the colon separator (Kept at 2)
const int CHAR_SPACING = 0;    // Space between elements (digits/colon) (Kept at 2)
const int SEG_ROUNDNESS = 1;   // Roundness for fillRoundRect (Kept at 1)

// Calculated Advances
const int DIGIT_ADVANCE = DIGIT_W_RENDER + 2; // 10 + 2 = 12
const int COLON_ADVANCE = 4;    // 2 + 2 = 4

/**
 * Draws a single 7-segment digit (0-9) at a given position.
 */
void render_digit(int16_t pos_x, int16_t pos_y, uint8_t digit, uint8_t color) {
    if (digit > 9) return;

    for (uint8_t i = 0; i < 7; i++) {
        bool seg_on = digit_array[digit][i];
        if (seg_on) {
            // NOTE: The coordinates are complex because they rely on multiple constants.
            switch (i) {
                // Segment A (Top horizontal) - Starts at pos_x + thickness
                case 0: display.fillRoundRect(pos_x + SEG_THICKNESS, pos_y, SEG_LEN_H, SEG_THICKNESS, SEG_ROUNDNESS, color); break;
                
                // Segment B (Top right vertical) - Starts after the horizontal segment ends
                case 1: display.fillRoundRect(pos_x + SEG_THICKNESS + SEG_LEN_H, pos_y + SEG_THICKNESS, SEG_THICKNESS, SEG_LEN_V, SEG_ROUNDNESS, color); break;
                
                // Segment C (Bottom right vertical) - Starts after G segment
                case 2: display.fillRoundRect(pos_x + SEG_THICKNESS + SEG_LEN_H, pos_y + 2*SEG_THICKNESS + SEG_LEN_V, SEG_THICKNESS, SEG_LEN_V, SEG_ROUNDNESS, color); break;
                
                // Segment D (Bottom horizontal) - Starts at pos_x + thickness
                case 3: display.fillRoundRect(pos_x + SEG_THICKNESS, pos_y + DIGIT_H - SEG_THICKNESS, SEG_LEN_H, SEG_THICKNESS, SEG_ROUNDNESS, color); break;
                
                // Segment E (Bottom left vertical) - Starts at pos_x
                case 4: display.fillRoundRect(pos_x, pos_y + 2*SEG_THICKNESS + SEG_LEN_V, SEG_THICKNESS, SEG_LEN_V, SEG_ROUNDNESS, color); break;
                
                // Segment F (Top left vertical) - Starts at pos_x
                case 5: display.fillRoundRect(pos_x, pos_y + SEG_THICKNESS, SEG_THICKNESS, SEG_LEN_V, SEG_ROUNDNESS, color); break;
                
                // Segment G (Middle horizontal) - Separates top and bottom halves
                case 6: display.fillRoundRect(pos_x + SEG_THICKNESS, pos_y + SEG_THICKNESS + SEG_LEN_V, SEG_LEN_H, SEG_THICKNESS, SEG_ROUNDNESS, color); break;
            }
        }
    }
}

/**
 * Draws a colon separator (two dots).
 * [FIXED: Using fillRect for 2x2 dots]
 */
void render_colon(int16_t pos_x, int16_t pos_y, uint8_t color) {
    int dot_size = SEG_THICKNESS; // 2 pixels
    
    // Use the full advance distance (4 pixels) as the virtual width for centering.
    const int VIRTUAL_COLON_WIDTH = 4; 

    // Vertical Positioning based on DIGIT_H = 20
    int dot_spacing = DIGIT_H / 4; // 5 pixels
    int top_dot_y = pos_y + dot_spacing; // Starts at Y+5
    int bottom_dot_y = pos_y + DIGIT_H - dot_spacing - dot_size; // Starts at Y+13
    
    // Horizontal Positioning: Center the 2px dot within the 4px VIRTUAL_COLON_WIDTH.
    // (4 - 2) / 2 = 1. This gives a 1-pixel buffer on either side of the dot.
    int dot_x = pos_x + (VIRTUAL_COLON_WIDTH - dot_size) / 2;

    // Draw the two dots (now centered horizontally)
    // Using fillRect as it's more reliable for 2x2 squares than fillRoundRect
    display.fillRect(dot_x, top_dot_y, dot_size, dot_size, color);
    display.fillRect(dot_x, bottom_dot_y, dot_size, dot_size, color);
}

/**
 * Renders the full HH:MM:SS time string, centered on the display.
 * [FIXED: Corrected total_width calculation for perfect centering]
 */
void render_time_string(int hour, int minute, int second, uint8_t color) {
    
    // --- Correct total_width calculation ---
    // The total width is the sum of advances for the first 7 elements, 
    // PLUS the width of the final (8th) element.
    // (5 * DIGIT_ADVANCE) + (2 * COLON_ADVANCE) + (1 * DIGIT_W_RENDER)
    // (5 * 12) + (2 * 4) + (1 * 10) = 60 + 8 + 10 = 78 pixels.
    const int total_width = (5 * DIGIT_ADVANCE) + (2 * COLON_ADVANCE) + DIGIT_W_RENDER;
    
    // Calculate the starting X position for horizontal centering (128 - 78) / 2 = 25
    int16_t x_cursor = (SCREEN_WIDTH - total_width) / 2;
    // Calculate the starting Y position for vertical centering (64 - 20) / 2 = 22
    int16_t y_pos = (SCREEN_HEIGHT - DIGIT_H) / 2;

    // --- RENDER HOURS (HH) ---
    // 1. Hour (Tens)
    render_digit(x_cursor, y_pos, hour / 10, color);
    x_cursor += DIGIT_ADVANCE; // Advance by 12

    // 2. Hour (Units)
    render_digit(x_cursor, y_pos, hour % 10, color);
    x_cursor += DIGIT_ADVANCE; // Advance by 12

    // 3. Colon 1 (HH:MM Separator - Blinking)
    if (second % 2 == 0) {
        render_colon(x_cursor, y_pos, color);
    }
    x_cursor += COLON_ADVANCE; // Advance by 4

    // --- RENDER MINUTES (MM) ---
    // 4. Minute (Tens)
    render_digit(x_cursor, y_pos, minute / 10, color);
    x_cursor += DIGIT_ADVANCE; // Advance by 12

    // 5. Minute (Units)
    render_digit(x_cursor, y_pos, minute % 10, color);
    x_cursor += DIGIT_ADVANCE; // Advance by 12

    // 6. Colon 2 (MM:SS Separator - Static)
    render_colon(x_cursor, y_pos, color);
    x_cursor += COLON_ADVANCE; // Advance by 4

    // --- RENDER SECONDS (SS) ---
    // 7. Second (Tens)
    render_digit(x_cursor, y_pos, second / 10, color);
    x_cursor += DIGIT_ADVANCE; // Advance by 12

    // 8. Second (Units)
    render_digit(x_cursor, y_pos, second % 10, color);
    // No need to advance cursor after the last digit
}

// --- End of Custom 7-Segment Drawing Functions ---


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
      // Handle time manually if NTP fails after initial sync
      if(timeSynced) {
        timeinfo.tm_sec++;
        if (timeinfo.tm_sec >= 60) { timeinfo.tm_sec = 0; timeinfo.tm_min++; }
        if (timeinfo.tm_min >= 60) { timeinfo.tm_min = 0; timeinfo.tm_hour++; }
        if (timeinfo.tm_hour >= 24) { timeinfo.tm_hour = 0; /* simplified date increment */ }
        
        updateOledDisplay();
      } else {
         Serial.println("Failed to obtain time");
      }
    } else {
      updateOledDisplay();
    }
    lastSecondTick += 1000;
  }

  // --- Task 2: Update NeoPixel Color (as fast as possible for smoothness) ---
  updateNeoPixelColor();
}

void updateNeoPixelColor() {
  // Keep NeoPixel off if time isn't synced yet
  if (!timeSynced) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
    return;
  }
  
  unsigned long millisIntoMinute = (timeinfo.tm_sec * 1000) + (millis() - lastSecondTick);
  millisIntoMinute = constrain(millisIntoMinute, 0, 59999);

  // Smooth transition from Green (0ms) to Red (60000ms)
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

/**
 * Draws the digital clock face using custom 7-segment drawing primitives.
 */
void drawDigitalClock(int hour, int minute, int second) {
  display.clearDisplay();
  
  render_time_string(hour, minute, second, SSD1306_WHITE);

  display.display();
}

// Helper function to find a point on the perimeter of a rectangle
void getRectPerimeterPoint(float distance, float x, float y, float w, float h, float& px, float& py) {
    // The drawable area is w-1 and h-1
    float drawn_w = w - 1;
    float drawn_h = h - 1;
    float perimeter = 2 * (drawn_w + drawn_h);
    distance = fmod(distance, perimeter);

    if (distance < drawn_w) { // Top edge (x to x+w-1)
        px = x + distance;
        py = y;
    } else if (distance < drawn_w + drawn_h) { // Right edge (y to y+h-1)
        px = x + drawn_w;
        py = y + (distance - drawn_w);
    } else if (distance < 2 * drawn_w + drawn_h) { // Bottom edge (x+w-1 to x)
        px = x + drawn_w - (distance - (drawn_w + drawn_h));
        py = y + drawn_h;
    } else { // Left edge (y+h-1 to y)
        px = x;
        py = y + drawn_h - (distance - (2 * drawn_w + drawn_h));
    }
}

// Function to draw the rectangular analog clock
void drawAnalogClock(int hour, int minute, int second) {
  display.clearDisplay();
  display.setFont(NULL);

  // Make the clock face fill the entire screen
  int clockX = 0;
  int clockY = 0;
  int clockWidth = SCREEN_WIDTH;
  int clockHeight = SCREEN_HEIGHT;
  // Draw the delimiting rectangle
  display.drawRect(clockX, clockY, clockWidth, clockHeight, SSD1306_WHITE);
  

  int centerX = clockX + clockWidth / 2;
  int centerY = clockY + clockHeight / 2;
  
  // --- Constants for Tick Spacing ---
  const int tickLength = 4;      
  const int TICK_OFFSET = 5;     

  // Draw 12 evenly-spaced hour ticks on the rectangular contour
  float perimeter = 2.0 * ((clockWidth - 1) + (clockHeight - 1));
  float hour_spacing = perimeter / 12.0;
  float start_offset = (float)(clockWidth - 1) / 2.0; 

  for (int i = 0; i < 12; i++) {
      float distance = fmod(start_offset + i * hour_spacing, perimeter);
      float edgeX, edgeY; 
      
      // Step 1: Find the point on the very edge
      getRectPerimeterPoint(distance, (float)clockX, (float)clockY, (float)clockWidth, (float)clockHeight, edgeX, edgeY);

      // Step 2: Calculate the normalized direction vector (Edge -> Center)
      float dirX = centerX - edgeX;
      float dirY = centerY - edgeY;
      float len = sqrt(dirX * dirX + dirY * dirY);
      if (len > 0) {
          dirX /= len; 
          dirY /= len;
      }
      
      // Step 3: Determine the START point (Offset inward from the edge)
      float startX = edgeX + dirX * TICK_OFFSET;
      float startY = edgeY + dirY * TICK_OFFSET;

      // Step 4: Determine the END point (Start point + visible length)
      float endX = startX + dirX * tickLength;
      float endY = startY + dirY * tickLength;
      
      // Step 5: Draw the line segment
      display.drawLine((int16_t)startX, (int16_t)startY, (int16_t)endX, (int16_t)endY, SSD1306_WHITE);
  }
  // --- End of Tick Drawing Loop ---

  // Calculate angles for hands
  float secAngle = (second * 6.0) * PI / 180.0;
  float minAngle = ((minute * 6.0) + (second * 0.1)) * PI / 180.0;
  float hourAngle = (((hour % 12) * 30.0) + (minute * 0.5)) * PI / 180.0;

  // Draw hands (adjusting hand lengths based on clock size)
  const int maxRadius = clockWidth / 2;
  
  // Second Hand (use full radius)
  int secX = centerX + (int)((maxRadius - 2) * sin(secAngle));
  int secY = centerY - (int)((maxRadius - 2) * cos(secAngle));
  display.drawLine(centerX, centerY, secX, secY, SSD1306_WHITE);

  // Minute Hand (shorter)
  int minX = centerX + (int)((maxRadius * 0.75) * sin(minAngle));
  int minY = centerY - (int)((maxRadius * 0.75) * cos(minAngle));
  display.drawLine(centerX, centerY, minX, minY, SSD1306_WHITE);

  // Hour Hand (shortest)
  int hourX = centerX + (int)((maxRadius * 0.5) * sin(hourAngle));
  int hourY = centerY - (int)((maxRadius * 0.5) * cos(hourAngle));
  display.drawLine(centerX, centerY, hourX, hourY, SSD1306_WHITE);

  // Center point
  display.fillCircle(centerX, centerY, 2, SSD1306_WHITE);
  display.display();
}
