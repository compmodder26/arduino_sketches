// code for radio receiver for custom drone
// receives struct from transmitter and provides the 
// throttle, yaw, pitch, and roll values to the flight controller via combined ppm signal
// ppm output is controlled by a timer (TC4)

#include <SPI.h>
#include <RH_RF69.h>

#define DEBUG 1       // whether or not to display serial console output for general debug messages
//#define DEBUG_IRQ 1 // whether or not to display serial console output for PPM interrups

/************ Radio Setup ***************/
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

/************ Metrics ***************/
int32_t totRssi = 0;
int32_t iterations = 0;
uint32_t packetsReceived = 0;
uint32_t packetsFailed = 0;
long lastOutput;
long lastReceive;

/************ PPM Config ***************/
#define channel_number 4  // set the number of channels
#define sigPin 5          // set PPM signal output pin on the arduino
#define PPM_FrLen 22500   // set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  // set the pulse length
#define clockMultiplier 3 // used to lengthen pulses/frames to match 16mhz clock

int ppm[channel_number];

// data structure to hold flight data
struct FlightData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
};

FlightData data;

// TIMER setup1024
void startTimer(int frequency);
void setTimerFrequency(int frequency);
void TC4_Handler();


void setup() 
{ 
  Serial.begin(115200);

  #ifdef DEBUG
    delay(1000);
  #endif
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
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

  // The encryption key has to be the same as the one on the transmitter
  uint8_t key[] = "SC7PW2IYHqHbjNnEmAnS";
  rf69.setEncryptionKey(key);

  // initialize values for our flight data, set throttle to 0
  resetData(0);
  setPPMValuesFromData();

  lastOutput = millis();

  #ifdef DEBUG
    Serial.print("RFM69 radio @");  
    Serial.print((int)RF69_FREQ);  
    Serial.println(" MHz");
  #endif
  
  startTimer(100);
}


void loop() {
  receiveData();

  // no radio contact in the last second, perform failsafe measure
  if (millis() - lastReceive >= 1000) {
    failSafe();
  }

  #ifdef DEBUG
    if (millis() - lastOutput >= 10000) {
      Serial.print("Average RSSI: ");
      Serial.println(totRssi / iterations);
  
      Serial.print("Total iterations: ");
      Serial.println(iterations);
        
      Serial.print("Total Successful Packets: ");
      Serial.println(packetsReceived);
        
      Serial.print("Total Failed Packets: ");
      Serial.println(packetsFailed);
  
      lastOutput = millis();
      totRssi = 0;
      iterations = 0;
      packetsReceived = 0;
      packetsFailed = 0;
    }
  #endif

  setPPMValuesFromData();
}

void receiveData() {
  if (rf69.available()) {
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf69.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;

      totRssi += rf69.lastRssi();

      // save new flight data received from transmitter
      memcpy(&data, buf, sizeof(data));

      byte currThrottleVal = data.throttle;

      // ensure that
      if (currThrottleVal >= 0 && currThrottleVal <= 255) {
        packetsReceived++;
        lastReceive = millis();
      }
      // packet must be malformed, let's ensure that the drone has sane values
      else {
        resetData(127);  // sets throttle to middle, to make the drone hover
        packetsFailed++;
      }
      
      iterations++;
    } else {
      packetsFailed++;
      Serial.println("Flight Data Reception failed");
    }
  }
}

void resetData(byte throttleVal) 
{
  data.throttle = throttleVal;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
}

// called when we lose radio contact
void failSafe() {
  if (data.throttle > 0) {
    #ifdef DEBUG
      Serial.println("Lost radio contact, performing failsafe measures");
    #endif

    // immediately set throttle to hover if we are ascending
    if (data.throttle > 127) {
      resetData(127);

      setPPMValuesFromData();
    }

    // until the throttle is completely off, step it down by 5
    while (data.throttle > 0) {
      int newThrottle = data.throttle - 5;

      if (newThrottle < 0) {
        newThrottle = 0;
      }
      
      #ifdef DEBUG
        Serial.println(newThrottle);
      #endif
      
      resetData((byte) newThrottle);
      setPPMValuesFromData();

      #ifdef DEBUG
        Serial.print("New throttle value is ");
        Serial.println(data.throttle);
      #endif

      // don't want to slam the drone into the ground
      delay(1000);
    }
  }
}

void setPPMValuesFromData()
{
  ppm[0] = map(data.throttle, 0, 255, 1000, 2000);
  ppm[1] = map(data.yaw,      0, 255, 1000, 2000);
  ppm[2] = map(data.pitch,    0, 255, 1000, 2000);
  ppm[3] = map(data.roll,     0, 255, 1000, 2000);  
}

void setTimerFrequency(int frequency) {
  //long start = micros();
  int compareValue = frequency;
  TcCount16* TC = (TcCount16*) TC4;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  //long diff = micros() - start;

  //Serial.print("Took ");
  //Serial.print(diff);
  //Serial.println(" microseconds to change frequency");
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC4;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1 (full 48MHZ)
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC4_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

// interrupt handler, controls output of PPM
void TC4_Handler() {
  static boolean state = true;
  
  TcCount16* TC = (TcCount16*) TC4;
  // If this interrupt is due to the compare register matching the timer count
  // we perform a step in the PPM signal output.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    
    // Custom callback code
      if ( state ) {
        //end pulse
        digitalWrite(sigPin,0);
        setTimerFrequency(PPM_PulseLen * clockMultiplier); // set the interrupt compare register to trigger for the length of a PPM end pulse
        state = false;

        #ifdef DEBUG_IRQ
          Serial.println("End Pulse");
        #endif
      }
      else {
        //start pulse
        static byte cur_chan_numb;
        static unsigned int calc_rest;
    
        digitalWrite(sigPin,1);
        state = true;

        // our final channel has been sent, now we need to wait until we reach our frame length
        if(cur_chan_numb >= channel_number) {
          cur_chan_numb = 0;
          calc_rest += PPM_PulseLen;
          setTimerFrequency((PPM_FrLen - calc_rest) * clockMultiplier);
          calc_rest = 0;

          #ifdef DEBUG_IRQ
            Serial.println("End PPM");
          #endif
        }
        // send the next channel's value
        else {
          setTimerFrequency((ppm[cur_chan_numb] - PPM_PulseLen) * clockMultiplier);
          
          #ifdef DEBUG_IRQ
            Serial.print("Sending data for channel ");
            Serial.println(cur_chan_numb);
          #endif
          
          calc_rest += ppm[cur_chan_numb];
          cur_chan_numb++;
        }     
      }
    // End custom callback code
  }
}
