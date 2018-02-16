// This is an adaptation of the buttoncycle example from the adafruit neopixel library.
// It is used for Christmas lights
// This will detect a press of a momentary push button.  When you
// press the button it will change to a new pixel animation.

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

#define BUTTON_PIN   1    // Digital IO pin connected to the button.  This will be
// driven with a pull-up resistor so the switch should
// pull the pin to ground momentarily.  On a high -> low
// transition the button press logic will execute.

#define PIXEL_PIN    0    // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 2

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool oldState = HIGH;    // tracks previously read push button state
uint8_t showType = 0;    // the light show type we want to watch (see startShow() for explanation of each number)
boolean doCycle = false; // whether or not to keep the current light show cycling.  Used for animations so they keep running

void setup() {
  showType = EEPROM.read(0); // get last light show preference reading before the device was turned off/restarted

  pinMode(BUTTON_PIN, INPUT_PULLUP); // set the button pin as input and engage the pullup resistor
  strip.begin();
  strip.setBrightness(255); // set to full power
  strip.show(); // Initialize all pixels to 'off'

  startShow(showType); // start the light show the user last selected before power off/restart
}

void loop() {
  CheckButtonPress();

  // Ensure that animations keep animating
  if (doCycle) {
    startShow(showType);
  }
}

bool CheckButtonPress() {
  bool newState = digitalRead(BUTTON_PIN); // Get current button state.

  // Check if state changed from high to low (button press).
  if (newState == LOW && oldState == HIGH) {
    // Short delay to debounce button.
    delay(20);
    // Check if button is still low after debounce.
    newState = digitalRead(BUTTON_PIN);
    if (newState == LOW) {
      showType++;

      if (showType > 7) {
        showType = 0;
      }

      EEPROM.write(0, showType); // save the user's preference so we load it back to the same state on startup
      startShow(showType);
    }
  }

  // Set the last button state to the old state.
  oldState = newState;

  return newState;
}

void startShow(uint8_t i) {
  switch (i) {
    case 0: colorWipe(strip.Color(255, 255, 255), 100);  // White
      doCycle = false;
      break;
    case 1: colorWipe(strip.Color(255, 140, 0), 100);  // Orange
      doCycle = false;
      break;
    case 2: colorWipe(strip.Color(255, 0, 0), 100);  // Red
      doCycle = false;
      break;
    case 3: colorWipe(strip.Color(0, 255, 0), 100);  // Green
      doCycle = false;
      break;
    case 4: colorWipe(strip.Color(128, 0, 128), 100);  // Purple
      doCycle = false;
      break;
    case 5: 
      // continuous color wipe cycle
      colorWipe(strip.Color(255, 255, 255), 100);  // White
      colorWipe(strip.Color(255, 140, 0), 100);  // Orange 
      colorWipe(strip.Color(255, 0, 0), 100);  // Red
      colorWipe(strip.Color(0, 255, 0), 100);  // Green
      colorWipe(strip.Color(128, 0, 128), 100);  // Purple
      doCycle = true;
      break;
    case 6: rainbow(20);
      doCycle = true;
      break;
    case 7: rainbowCycle(20);
      doCycle = true;
      break;
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);

    // button pressed in the middle of the animation, stop execution of this animation
    if (CheckButtonPress() == LOW) {
      return;
    }
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    delay(wait);

    // button pressed in the middle of the animation, stop execution of this animation
    if (CheckButtonPress() == LOW) {
      return;
    }
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 5; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);

    // button pressed in the middle of the animation, stop execution of this animation
    if (CheckButtonPress() == LOW) {
      return;
    }
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, c);  //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }

    // button pressed in the middle of the animation, stop execution of this animation
    if (CheckButtonPress() == LOW) {
      return;
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j = 0; j < 256; j++) {   // cycle all 256 colors in the wheel
    for (int q = 0; q < 3; q++) {
      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, Wheel( (i + j) % 255)); //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }

    // button pressed in the middle of the animation, stop execution of this animation
    if (CheckButtonPress() == LOW) {
      return;
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
