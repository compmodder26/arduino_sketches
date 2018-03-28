// code for radio receiver for feather bot
// receives struct from transmitter and provides the 
// forward and turn values to be converted to pwm to drive the bot

#include <SPI.h>
#include <RH_RF69.h>

//#define DEBUG 1       // whether or not to display serial console output for general debug messages

/************ Radio Setup ***************/
// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

// Drive Pins
#define RIGHT_FORWARD    5
#define RIGHT_REVERSE    6
#define RIGHT_PWM       12
#define LEFT_FORWARD    10
#define LEFT_REVERSE     9
#define LEFT_PWM        11

// Sensor Pins 
#define RIGHT_TRIG      16
#define RIGHT_ECHO      17
#define MIDDLE_TRIG     18
#define MIDDLE_ECHO     19
#define LEFT_TRIG        0
#define LEFT_ECHO        1

// Threshold Values
#define MAX_FOWARD_PWM        180
#define MAX_REVERSE_PWM       150
#define MAX_AUTONOMOUS_PWM    100
#define MIN_TURN_PWM          135
#define CAUTION_PWM            75
#define CAUTION_THRESHOLD      16
#define STOP_THRESHOLD          8   // how far in inches we can be before we need to stop to avoid collision
#define FORWARD_THRESHOLD     140   // joystick reading that will tell us if we should be moving forward
#define REVERSE_THRESHOLD     110   // joystick reading that will tell us if we should be moving in reverse
#define RIGHT_THRESHOLD       140   // joystick reading that will tell us if we should be turning right
#define LEFT_THRESHOLD        110   // joystick reading that will tell us if we should be turning left
#define NEUTRAL               128   // value we deem as neutral for PWM (no movement either direction)

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

/************ Metrics ***************/
int32_t totRssi = 0;
int32_t iterations = 0;
uint32_t packetsReceived = 0;
uint32_t packetsFailed = 0;
long lastOutput;
long lastReceive;

// data structure to hold drive data
struct DriveData {
  byte forward;
  byte turn;
  byte autonomous;
};

DriveData data;

bool autonomous = false;

// initialize distance measurements, we set it high to avoid immediately stopping the bot
unsigned int rightDistance = 1000;
unsigned int middleDistance = 1000;
unsigned int leftDistance = 1000;

// TIMER setup
uint16_t sampleRate = 56250;  // using prescaler of 256, and wanting interrupt to happen every .3 seconds


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

  // initialize values for our drive data, set drive to NEUTRAL (stop)
  resetData(NEUTRAL);

  lastOutput = millis();

  #ifdef DEBUG
    Serial.print("RFM69 radio @");  
    Serial.print((int)RF69_FREQ);  
    Serial.println(" MHz");
  #endif

  // set up drive pins
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_REVERSE, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_REVERSE, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);

  // initialize the pins
  stop();

  // set up sensor pins
  pinMode(RIGHT_TRIG, OUTPUT);    // set the right trigger pin to output
  pinMode(RIGHT_ECHO, INPUT);     // set the right echo pin to input
  pinMode(MIDDLE_TRIG, OUTPUT);   // set the middle trigger pin to output
  pinMode(MIDDLE_ECHO, INPUT);    // set the middle echo pin to input
  pinMode(LEFT_TRIG, OUTPUT);     // set the left trigger pin to output
  pinMode(LEFT_ECHO, INPUT);      // set the left echo pin to input

  digitalWrite(RIGHT_TRIG, LOW);
  digitalWrite(MIDDLE_TRIG, LOW);
  digitalWrite(LEFT_TRIG, LOW);

  delay(2000);

  tcConfigure(sampleRate); //configure the timer to run at <sampleRate>Hertz
  tcStartCounter(); //starts the timer
}


void loop() {
  receiveData();

  //Serial.print("Y: ");
  //Serial.println(data.forward);
  //Serial.print("X: ");
  //Serial.println(data.turn);

  // no radio contact in the last half second, perform failsafe measure
  if (millis() - lastReceive >= 500) {
    resetData(NEUTRAL);
    stop();
    digitalWrite(LED, HIGH);  // give visual indication that we've lost signal to transimitter
  }
  else {
    digitalWrite(LED, LOW);
  }

  if (data.autonomous == 1) {
    autonomous = true;
  }
  else {
    autonomous = false;
  }

  if (autonomous) {
    #ifdef DEBUG
      Serial.println("Autonomous Engaged");
    #endif
      
    if (detectCollision()) {
      stop();
      
      if (rightDistance + middleDistance > leftDistance + middleDistance) {
        // To make a quick right hand turn, rotate the left wheel forward, and the right wheel backward
        digitalWrite(RIGHT_FORWARD, LOW);  
        digitalWrite(LEFT_FORWARD, HIGH);
        digitalWrite(RIGHT_REVERSE, HIGH);
        digitalWrite(LEFT_REVERSE, LOW);
      }
      else {
        // To make a quick left hand turn, rotate the left wheel in reverse, and the right wheel forward
        digitalWrite(RIGHT_FORWARD, HIGH);  
        digitalWrite(LEFT_FORWARD, LOW);
        digitalWrite(RIGHT_REVERSE, LOW);
        digitalWrite(LEFT_REVERSE, HIGH);
      }
      
      analogWrite(RIGHT_PWM, MAX_AUTONOMOUS_PWM);
      analogWrite(LEFT_PWM, MAX_AUTONOMOUS_PWM);

      delay(200);

      stop();

      delay(400);
    }
    else {
      analogWrite(RIGHT_PWM, MAX_AUTONOMOUS_PWM);
      analogWrite(LEFT_PWM, MAX_AUTONOMOUS_PWM - 5);
  
      digitalWrite(RIGHT_FORWARD, HIGH);  // set both side forward pins high
      digitalWrite(LEFT_FORWARD, HIGH);
      digitalWrite(RIGHT_REVERSE, LOW);  // set both side reverse pins high
      digitalWrite(LEFT_REVERSE, LOW);
    }
  }
  else {
    // Joystick is pushed up, user intends to move the bot forward
    if (data.forward >= FORWARD_THRESHOLD) {
      // close to an object, stop the bot
      if (detectCollision()) {
        stop();
      }
      else {
        uint8_t LIMIT_PWM = MAX_FOWARD_PWM;

        //if (objectNear()) {
        //  LIMIT_PWM = CAUTION_PWM;
        //}
        
        digitalWrite(RIGHT_FORWARD, HIGH);  
        digitalWrite(LEFT_FORWARD, HIGH);
        digitalWrite(RIGHT_REVERSE, LOW);
        digitalWrite(LEFT_REVERSE, LOW);
  
        uint8_t forwardVal = map(data.forward, FORWARD_THRESHOLD, 255, 0, LIMIT_PWM);
  
        // turn left (data.turn is between 128 and 255)
        if (data.turn >= LEFT_THRESHOLD) {
          analogWrite(LEFT_PWM, map(forwardVal - (data.turn - NEUTRAL), NEUTRAL, forwardVal, MIN_TURN_PWM, forwardVal));
          analogWrite(RIGHT_PWM, forwardVal);
        }
        // turn right (data.turn is between 0 and 128)
        else if (data.turn <= RIGHT_THRESHOLD) {
          analogWrite(LEFT_PWM, forwardVal);
          analogWrite(RIGHT_PWM, map(forwardVal  - (NEUTRAL - data.turn), forwardVal , NEUTRAL, forwardVal, MIN_TURN_PWM));
        }
        // go straight
        else {
          analogWrite(RIGHT_PWM, forwardVal);
          analogWrite(LEFT_PWM, forwardVal);
        }
      }
    }
    // Joystick is pushed back, user intends to move the bot in reverse
    else if (data.forward <= REVERSE_THRESHOLD) {
      
      digitalWrite(RIGHT_FORWARD, LOW);  
      digitalWrite(LEFT_FORWARD, LOW);
      digitalWrite(RIGHT_REVERSE, HIGH);
      digitalWrite(LEFT_REVERSE, HIGH);

      uint8_t reverseVal = map(data.forward, 0, REVERSE_THRESHOLD, MAX_REVERSE_PWM, 0);
      //uint8_t reverseVal = MAX_REVERSE_PWM;

      // turn left
      if (data.turn >= LEFT_THRESHOLD) {
        analogWrite(LEFT_PWM, reverseVal);
        analogWrite(RIGHT_PWM, map(reverseVal - (data.turn - NEUTRAL), NEUTRAL, reverseVal, MIN_TURN_PWM, reverseVal));
      }
      // turn right
      else if (data.turn <= RIGHT_THRESHOLD) {
        analogWrite(LEFT_PWM, map(reverseVal  - (NEUTRAL - data.turn), reverseVal, NEUTRAL, reverseVal, MIN_TURN_PWM));
        analogWrite(RIGHT_PWM, reverseVal);
      }
      // go straight
      else {
        analogWrite(RIGHT_PWM, reverseVal);
        analogWrite(LEFT_PWM, reverseVal);
      }
    }
    // Joystick in in neutral, user intends forward/reverse movement to be none
    else {
      // pivot left
      if (data.turn >= LEFT_THRESHOLD) {
        
        digitalWrite(RIGHT_FORWARD, HIGH);  
        digitalWrite(LEFT_FORWARD, LOW);
        digitalWrite(RIGHT_REVERSE, LOW);
        digitalWrite(LEFT_REVERSE, HIGH);

        uint8_t turnPWM = map(data.turn, LEFT_THRESHOLD, 255, 0, MIN_TURN_PWM);
        
        analogWrite(LEFT_PWM, 0);
        analogWrite(RIGHT_PWM, turnPWM);
      }
      // pivot left
      else if (data.turn <= RIGHT_THRESHOLD) {
        
        digitalWrite(RIGHT_FORWARD, LOW);  
        digitalWrite(LEFT_FORWARD, HIGH);
        digitalWrite(RIGHT_REVERSE, HIGH);
        digitalWrite(LEFT_REVERSE, LOW);
        
        uint8_t turnPWM = map(data.turn, 0, RIGHT_THRESHOLD, MIN_TURN_PWM, 0);
        
        analogWrite(RIGHT_PWM, turnPWM);
        analogWrite(LEFT_PWM, 0);
      }
      // stop
      else {
        stop();
      }
    }
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
}

bool detectCollision() {
  if ((rightDistance <= STOP_THRESHOLD && rightDistance != 0) || (middleDistance <= STOP_THRESHOLD && middleDistance != 0) || (leftDistance <= STOP_THRESHOLD && leftDistance != 0)) {
    return true;
  }
  else {
    return false;
  }
}

bool objectNear() {
  if ((rightDistance <= CAUTION_THRESHOLD && rightDistance != 0) || (middleDistance <= CAUTION_THRESHOLD && middleDistance != 0) || (leftDistance <= CAUTION_THRESHOLD && leftDistance != 0)) {
    return true;
  }
  else {
    return false;
  }
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

      byte currForwardVal = data.forward;

      // ensure that
      if (currForwardVal >= 0 && currForwardVal <= 255) {
        packetsReceived++;
        lastReceive = millis();
        digitalWrite(LED, LOW);
      }
      // packet must be malformed, let's ensure that the bot has sane values
      else {
        resetData(NEUTRAL);  // sets drive to middle, to make bot stop
        packetsFailed++;
        digitalWrite(LED, HIGH);
      }
      
      iterations++;
    } else {
      packetsFailed++;
      Serial.println("Drive Data Reception failed");
      digitalWrite(LED, HIGH);
    }
  }
}

void stop() {
  digitalWrite(RIGHT_FORWARD, LOW);
  digitalWrite(RIGHT_REVERSE, LOW);
  digitalWrite(LEFT_FORWARD, LOW);
  digitalWrite(LEFT_REVERSE, LOW);
  analogWrite(RIGHT_PWM, 0);
  analogWrite(LEFT_PWM, 0);
}

void resetData(byte forwardVal) 
{
  data.forward = forwardVal;
  data.turn = NEUTRAL;
  data.autonomous = 0;
}

// reads the ultrasonic sensor and returns the distance in inches
int readSensor(uint8_t trigger_pin, uint8_t echo_pin) {
  digitalWrite(trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);
  
  long duration = pulseIn(echo_pin, HIGH);

  return duration*0.0133/2;
}

//this function gets called by the interrupt at <sampleRate>Hertz
void TC4_Handler (void) {
  //YOUR CODE HERE 
  rightDistance = readSensor(RIGHT_TRIG, RIGHT_ECHO);
  middleDistance = readSensor(MIDDLE_TRIG, MIDDLE_ECHO);
  leftDistance = readSensor(LEFT_TRIG, LEFT_ECHO);

  #if DEBUG
    Serial.print("Right Ultrasonic reading: ");
    Serial.println(rightDistance);
    Serial.print("Middle Ultrasonic reading: ");
    Serial.println(middleDistance);
    Serial.print("Left Ultrasonic reading: ");
    Serial.println(leftDistance);
  #endif
  // END OF YOUR CODE
  TC4->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}

/* 
 *  TIMER SPECIFIC FUNCTIONS FOLLOW
 *  you shouldn't change these unless you know what you're doing
 */

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
 void tcConfigure(int sampleRate) {
 // Enable GCLK for TCC4 and TC5 (timer counter input clock)
 GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
 while (GCLK->STATUS.bit.SYNCBUSY);

 tcReset(); //reset TC4

 // Set Timer counter Mode to 16 bits
 TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set TC4 mode as match frequency
 TC4->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
 //set prescaler and enable TC4
 TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_ENABLE;
 //set TC4 timer counter based off of the system clock and the user defined sample rate or waveform
 TC4->COUNT16.CC[0].reg = sampleRate;
 while (tcIsSyncing());
 
 // Configure interrupt request
 NVIC_DisableIRQ(TC4_IRQn);
 NVIC_ClearPendingIRQ(TC4_IRQn);
 NVIC_SetPriority(TC4_IRQn, 0);
 NVIC_EnableIRQ(TC4_IRQn);

 // Enable the TC4 interrupt request
 TC4->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing()); //wait until TC4 is done syncing 
} 

//Function that is used to check if TC4 is done syncing
//returns true when it is done syncing
bool tcIsSyncing() {
  return TC4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC4 and waits for it to be ready
void tcStartCounter() {
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC4 
void tcReset() {
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC4->COUNT16.CTRLA.bit.SWRST);
}

//disable TC4
void tcDisable() {
  TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}
