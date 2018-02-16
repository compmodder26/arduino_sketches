// This is an adaptation of the buttoncycle example from the adafruit neopixel library.
// It is used for Christmas lights
// This will detect a press of a momentary push button.  When you
// press the button it will change to a new pixel animation.

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

#define BUTTON_PIN   2    // Digital IO pin connected to the button.  This will be
// driven with a pull-up resistor so the switch should
// pull the pin to ground momentarily.  On a high -> low
// transition the button press logic will execute.

#define PIXEL_PIN    0    // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 50

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
boolean switchShow = false; // whether or not to switch the light show.  Set to true when button press occurs.
uint16_t globalColor = 0; // used to keep track of color array position for functions that require global tracking

// Define an array of popular christmas colors
//                             Red                     Green                   Blue                    Yellow                    Purple
uint32_t christmasColors[] = { strip.Color(0, 255, 0), strip.Color(255, 0, 0), strip.Color(0, 0, 255), strip.Color(255, 255, 0), strip.Color(0, 40, 60) };
uint32_t red[] = { strip.Color(0, 255, 0) };
uint32_t green[] = { strip.Color(255, 0, 0) };
uint32_t blue[] = { strip.Color(0, 0, 255) };
uint32_t yellow[] = { strip.Color(255, 255, 0) };
uint32_t purple[] = { strip.Color(0, 40, 60) };;
uint32_t white[] = { strip.Color(255, 255, 255) };

void setup() {
  showType = EEPROM.read(0); // get last light show preference reading before the device was turned off/restarted

  pinMode(BUTTON_PIN, INPUT_PULLUP); // set the button pin as input and engage the pullup resistor
  strip.begin();
  strip.setBrightness(191); // set to 75% power (full brightness hurts the eyes)
  strip.show(); // Initialize all pixels to 'off'

  startShow(showType); // start the light show the user last selected before power off/restart
}

void loop() {
  CheckButtonPress();

  if (switchShow) {
    showType++;

    if (showType > 15) {
      showType = 0;
    }

    EEPROM.write(0, showType); // save the user's preference so we load it back to the same state on startup
  }

  // Ensure that animations keep animating or switch the show because a button press was detected
  if (doCycle || switchShow) {
    switchShow = false;
    startShow(showType);
  }
}

void CheckButtonPress() {
  bool currState = digitalRead(BUTTON_PIN); // Get current button state.

  // Check if state changed from high to low (button press).
  if (currState == LOW && oldState == HIGH) {
    switchShow = true;
  }

  oldState = currState;
}

void startShow(uint8_t i) {
  switch (i) {
    case 0: colorWipe(christmasColors, sizeof christmasColors / sizeof *christmasColors, 100);
      doCycle = false;
      break;
    case 1:colorWipe(white, 1, 100);  // White
      doCycle = false;
      break;
    case 2: colorWipe(red, 1, 100);  // Red
      doCycle = false;
      break;
    case 3: colorWipe(green, 1, 100);  // Green
      doCycle = false;
      break;
    case 4: colorWipe(blue, 1, 100);  // Blue
      doCycle = false;
      break;
    case 5: colorWipe(purple, 1, 100);  // Purple
      doCycle = false;
      break;
    case 6: 
      // continuous color wipe cycle
      colorWipe(red, 1, 100);  // Red 
      colorWipe(green, 1, 100);  // Green 
      colorWipe(blue, 1, 100);  // Blue
      colorWipe(purple, 1, 100);  // Purple
      doCycle = true;
      break;
    case 7: theaterChase(white, 1, 150); // White
      doCycle = true;
      break;
    case 8: theaterChase(red, 1, 150); // Red
      doCycle = true;
      break;
    case 9: theaterChase(green, 1, 150); // Green
      doCycle = true;
      break;
    case 10: theaterChase(blue, 1, 150); // Blue
      doCycle = true;
      break;
    case 11: theaterChase(purple, 1, 150); // Purple
      doCycle = true;
      break;
    case 12: theaterChase(christmasColors, sizeof christmasColors / sizeof *christmasColors, 150); // Christmas Colors
      doCycle = true;
      break;
    case 13: rainbow(20);
      doCycle = true;
      break;
    case 14: globalColor = 0;
      colorChase(christmasColors, sizeof christmasColors / sizeof *christmasColors, 200);
      doCycle = true;
      break;
    case 15: christmasFade(50); // Christmas colors in a color wipe with multiple colors wiping at the same time
      doCycle = true;
      break;
  }
}

// Fill the dots one after the other with an alternating traditional christmas color (red, green, blue, or yellow)
void colorWipe(uint32_t colors[], size_t arrLength, uint8_t wait) {
  int colorNum = 0;

  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, colors[colorNum]);
    strip.show();

    colorNum++;

    if (colorNum >= arrLength) {
      colorNum = 0;
    }

    delay(wait);

    // button pressed in the middle of the animation
    if (!switchShow) {
      CheckButtonPress();
    }

    //if (switchShow) {
    //  return;
    //}
  }
}

void christmasFade (uint16_t wait) {
  uint16_t currColor = 0;

  for (int16_t i = 0; i < strip.numPixels(); i++) {
    currColor++;

    if (currColor == 5) {
      currColor = 0;
    }

    for (uint8_t z = 0; z < 41; z++) {
      float multiplier = 0.05 * z;

      if (z > 20) {
        multiplier = (0.05 * (40 - z));
      }
      
      uint32_t colors[] = { strip.Color(0, 255 * multiplier, 0), strip.Color(255 * multiplier, 0, 0), strip.Color(0, 0, 255 * multiplier), strip.Color(255 * multiplier, 255 * multiplier, 0), strip.Color(0, 40 * multiplier, 60 * multiplier) };
      
      for (int16_t x = 0; x < strip.numPixels(); x++) {
        strip.setPixelColor(x, colors[currColor]);

        currColor++;

        if (currColor == 5) {
          currColor = 0;
        }
      }

      strip.show();

      if (z == 20) {
       delay(750); 
      }
      else {
        delay(20);
      }

      delete []colors;

      if (!switchShow) {
        CheckButtonPress();
      }
    }
    
    //delay(wait);

    if (switchShow) {
      return;
    }
  }
}

void colorChase (uint32_t colors[], size_t arrLength, uint8_t wait) {
  uint8_t concurrentPixels = strip.numPixels() / arrLength;
  
  for (int16_t i = 0; i < strip.numPixels(); i++) {
    for (int16_t x = 0; x < strip.numPixels(); x++) {
      int16_t modifier = x + i;

      if (modifier != 0 && modifier % concurrentPixels == 0) {
        globalColor++;

        if (globalColor >= arrLength) {
          globalColor = 0;
        }
      }

      strip.setPixelColor(x, colors[globalColor]);
    }

    strip.show();
    delay(wait);

    // button pressed in the middle of the animation
    if (!switchShow) {
      CheckButtonPress();
    }

    if (switchShow) {
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

    // button pressed in the middle of the animation
    if (!switchShow) {
      CheckButtonPress();
    }

    //if (switchShow) {
    //  return;
    //}
  }
}

//Theatre-style crawling lights with multiple colors.
void theaterChase(uint32_t colors[], size_t arrLength, uint8_t wait) {
  int colorNum = 0;
  
  for (int q = 0; q < 3; q++) {
    for (int i = 0; i < strip.numPixels(); i = i + 3) {
      strip.setPixelColor(i + q, colors[colorNum]);  //turn every third pixel on

      colorNum++;

      if (colorNum >= arrLength) {
        colorNum = 0;
      }
    }
    strip.show();

    delay(wait);

    for (int i = 0; i < strip.numPixels(); i = i + 3) {
      strip.setPixelColor(i + q, 0);      //turn every third pixel off
    }

    // button pressed in the middle of the animation
    if (!switchShow) {
      CheckButtonPress();
    }

    //if (switchShow) {
    //  return;
    //}
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

      // button pressed in the middle of the animation
      if (!switchShow) {
        CheckButtonPress();
      }

      if (switchShow) {
        return;
      }
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
