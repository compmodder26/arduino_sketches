// code for radio transmitter for custom drone
// reads analog inputs from 2 joysticks to send data
// over 915mhz radio for throttle, yaw, pitch, and roll

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

// Speed limit toggle switch
#define SPEED_LIMIT_PIN 5
#define SPEED_LIMIT_MAX 192
#define FULL_SPEED_MAX  255
int maxPotValue = FULL_SPEED_MAX;

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// The sizeof this struct should not exceed 32 bytes
struct FlightData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
};

FlightData data;

// RF communication, Dont put this on the stack:
byte buf[sizeof(data)] = {0};

void resetData() 
{
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
}

void setup() 
{
  Serial.begin(115200);

  pinMode(SPEED_LIMIT_PIN, INPUT_PULLUP);
  
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the receiver
  uint8_t key[] = "SC7PW2IYHqHbjNnEmAnS";
  rf69.setEncryptionKey(key);
  
  resetData();

  Serial.print("RFM69 radio @");  
  Serial.print((int)RF69_FREQ);  
  Serial.println(" MHz");
}

void loop() {
  // check speed limit switch to see if it's toggled on
  bool speedLimited = digitalRead(SPEED_LIMIT_PIN);

  if (speedLimited) {
    maxPotValue = SPEED_LIMIT_MAX;
  }
  else {
    maxPotValue = FULL_SPEED_MAX;
  }
  
  // The calibration numbers used here should be measured 
  // for your joysticks using the TestJoysticks sketch.
  data.throttle = mapJoystickValues( analogRead(A0), 0, 512, 1024, true );
  data.yaw      = mapJoystickValues( analogRead(A1), 0, 512, 1024, true );
  data.pitch    = mapJoystickValues( analogRead(A2), 0, 512, 1024, true );
  data.roll     = mapJoystickValues( analogRead(A3), 0, 512, 1024, true );
  
  byte zize=sizeof(data);
  memcpy (buf, &data, zize);
  
  // Send a message!
  rf69.send(buf, zize);
  rf69.waitPacketSent();
}

// Returns a corrected value for a joystick position that takes into account
// the values of the outer extents and the middle of the joystick range.
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse) {
  val = constrain(val, lower, upper);
  
  if ( val < middle ) {
    val = map(val, lower, middle, 0, 128);
  }
  else {
    val = map(val, middle, upper, 128, FULL_SPEED_MAX);
  }

  if (reverse) {
    val = FULL_SPEED_MAX - val;
  }

  // if speed limiting switch is toggled, then we don't allow the upper bound reading of the joystick to go higher than the defined maximum
  if (val > maxPotValue) {
    val = maxPotValue;
  }
  
  return val;
}
