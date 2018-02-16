#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define LEDPIN 0
#define TRIGGER 2

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, LEDPIN, NEO_GRB + NEO_KHZ800);

int buttonState = 0;


// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void setup() {
  // initialize the pushbutton pin as an input:
  pinMode(TRIGGER, INPUT);
  digitalWrite(TRIGGER, LOW);
  
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code


  strip.begin();
  strip.show();
}

void loop() {
  buttonState = digitalRead(TRIGGER);
  
  if (buttonState == HIGH) {
    lightningFlashLeftRight();
    delay(6000);
    lightningFlashFull();
    lightningFlashLeftRight();
    delay(7900);
    lightningFlashLeftRight();
    delay(6000);
    lightningFlashFull();
  }
}

void light (int n) {
  strip.setPixelColor(n, strip.Color(255, 255, 255));
}

void off (int n) {
  strip.setPixelColor(n, strip.Color(0, 0, 0));
}

void lightningFlashFull() {
  // first flash
  light(0);
  light(1);
  light(2);
  light(3);
  strip.show();
  delay(400);
  off(0);
  off(1);
  off(2);
  off(3);
  strip.show();
  delay(150);

  // second flash
  light(0);
  light(1);
  light(2);
  light(3);
  strip.show();
  delay(150);
  off(0);
  off(1);
  off(2);
  off(3);
  strip.show();
}

void lightningFlashLeftRight() {
  // first flash
  light(0);
  light(1);
  off(2);
  off(3);
  strip.show();
  delay(400);
  off(0);
  off(1);
  light(2);
  light(3);
  strip.show();
  delay(150);

  // second flash
  light(0);
  light(1);
  off(2);
  off(3);
  strip.show();
  delay(350);
  off(0);
  off(1);
  light(2);
  light(3);
  strip.show();
  delay(300);

  // third flash
  light(0);
  light(1);
  off(2);
  off(3);
  strip.show();
  delay(150);
  off(0);
  off(1);
  strip.show();
}

