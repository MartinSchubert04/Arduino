#include <ESP32Servo.h>

#define PIN_TRIG 0
#define PIN_ECHO 1
#define SERVO_PIN 4

Servo servo;

int pos = 0;
int angle = 90;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  servo.attach(SERVO_PIN);
}

void loop() {

  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  int duration = pulseIn(PIN_ECHO, HIGH);
  int distance = duration / 58; // in cm
  Serial.print("Distance in CM: ");
  Serial.println(duration / 58);

  if (distance < 10) {
    for (pos = 0; pos < angle; pos++) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      servo.write(pos); // tell servo to go to position in variable 'pos'
      delay(15);        // waits 15ms for the servo to reach the position
    }

    int newDistance = 999; // set this high so that enters the loop
    while (newDistance < 10) {
      digitalWrite(PIN_TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(PIN_TRIG, LOW);

      int newDistance = pulseIn(PIN_ECHO, HIGH) / 58;
    }

    for (pos = angle; pos > 0; pos--) { // goes from 90 degrees to 0 degrees

      servo.write(pos);
      delay(15); // waits 15ms for the servo to reach the position
    }
  }
}
