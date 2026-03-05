const int BUTTON_PIN = 18;
const int LED_PIN = 4;
int oldValue = HIGH; // default/idle value for pin 4 is high.
int active = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Press the button.");

  // Initialize the pin for reading the button.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Read the value of pin 4.
  int newValue = digitalRead(BUTTON_PIN);

  // Check if the value was changed,
  // by comparing it with the previous value.
  if (oldValue == HIGH && newValue == LOW) {
    active = !active;

    if (active) {
      Serial.println("System is active");
      digitalWrite(LED_PIN, HIGH);
    } else {
      Serial.println("System is not active");
      digitalWrite(LED_PIN, LOW);
    }
    // Remember the value for the next time.
  }

  oldValue = newValue;

  delay(20);
}
