// ALCSS - Automatic Light Car Stopping Signal
// Reads from an analog weight sensor.  When weight is detected, flashes red lights

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

#define TRIGGER_PIN 4 // pin connected to the trigger pin of the ultrasonic sensor
#define ECHO_PIN    1 // pin connected to the echo pin of the ultrasonic sensor
#define PIXEL_PIN   0 // Digital IO pin connected to the NeoPixels.
#define BUTTON_PIN  2 // pin connected to the button that will store the desired distance in inches to trigger the lights

#define PIXEL_COUNT 12

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool oldButtonState = HIGH;                 // tracks previously read push button state
bool wasInRange = false;                    // tracks whether or not weight was detected in the prior run
bool warningRange = false;                  // tracks whether or not the 
uint32_t red = strip.Color(255, 0, 0);
uint32_t yellow = strip.Color(255, 255, 0);
uint32_t green = strip.Color(0, 255, 0);
uint32_t off = strip.Color(0, 0, 0);
int stopDistance;
int warnDistance;
int currDistance;

void setup() {
  stopDistance = EEPROM.read(0); // get last light show preference reading before the device was turned off/restarted

  // nothing previously set, default to 2 feet (24 inches)
  if (stopDistance == NULL) {
    stopDistance = 24;
  }

  warnDistance = stopDistance + 20; // warn with a yellow light when 20 inches away from stop distance
  
  pinMode(TRIGGER_PIN, OUTPUT);       // set the trigger pin to output
  pinMode(ECHO_PIN, INPUT);           // set the echo pin to input
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // set button pin to input, engage pullup resistor (button press brings reading to LOW)
  pinMode(PIXEL_PIN, OUTPUT);         // set pixel pin to output
  
  strip.begin();
  strip.setBrightness(191); // full brightness
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  checkButtonPress();
  
  currDistance = readSensor();

  if (currDistance <= warnDistance && currDistance > stopDistance) {
    if (!warningRange) {
      lightsOn(yellow);
      warningRange = true;
      wasInRange = false;
    }
  }
  else if (currDistance <= stopDistance) {
    if (!wasInRange) {
      blink(red, 15000);
      wasInRange = true;
      warningRange = false;
    }
  }
  else {
    if (wasInRange || warningRange) {
      lightsOff();
      wasInRange = false;
      warningRange = false;
    }
  }

  delay(100);
}

void checkButtonPress() {
  bool currState = digitalRead(BUTTON_PIN); // Get current button state.

  // Check if state changed from high to low (button press).
  if (currState == LOW && oldButtonState == HIGH) {
    stopDistance = currDistance;
    warnDistance = stopDistance + 20;

    EEPROM.write(0, stopDistance);

    blink(green, 5000);
  }

  oldButtonState = currState;
}

// reads the ultrasonic sensor 3x and returns the average distance in inches
int readSensor() {
  int totInches = 0;

  for (int x = 0; x < 3; x++) {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH);

    totInches += duration*0.0133/2;
  }
  
  return totInches / 3;
}

void lightsOn(uint32_t color) {
  for (uint8_t x = 0; x < strip.numPixels(); x++) {
    strip.setPixelColor(x, color);
  } 

  strip.show();
}

void lightsOff() {
  for (uint8_t x = 0; x < strip.numPixels(); x++) {
    strip.setPixelColor(x, off);
  } 

  strip.show();
}

void blink(uint32_t color, long duration) {
  unsigned long start = millis();

  while (millis() - start < duration) {  // blink for 30 seconds
    lightsOn(color);
    delay(100);
    lightsOff();
    delay(100);
  }
}
