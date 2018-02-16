// ALCSS - Automatic Light Car Stopping Signal Sensor Test
// Reads from an ultrasonic sensor and prints the value to console

#define TRIGGER_PIN 4 // pin connected to the trigger pin of the ultrasonic sensor
#define ECHO_PIN  3   // pin connected to the echo pin of the ultrasonic sensor

bool wasInRange = false;    // tracks whether or not prior run already detected the stopping distance required

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT); // set the trigger pin to output
  pinMode(ECHO_PIN, INPUT);     // set the echo pin to input

  Serial.begin(9600);
}

void loop() {
  int sensorValue = readSensor();

  Serial.println(sensorValue);

  if (sensorValue <= 24) {
    if (!wasInRange) {
      Serial.println("Would have triggered light");
      wasInRange = true;
    }
  }
  else {
    wasInRange = false;
  }

  delay(100);
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

