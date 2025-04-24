const int encoder0PinA = 2;
const int encoder0PinB = 3;
volatile int encoder0Pos = 0;
volatile int encoder0PinALast = LOW;
volatile int n = LOW;
volatile int m = LOW;
int valNew = 0;
int valOld = 0;

const int pwmPin = 11;      // PWM output pin
const int dirPin = 13;      // Direction control pin
const int pwmValue = 195;  // Fixed PWM value
int targetPosition = 0;    // Target encoder position
bool pwmRunning = false;   // Flag for movement control
const int buffer = 100;      // Deadband buffer range

void setup() {
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  Serial.begin(9600);

  encoder0Pos = 0;
  encoder0PinALast = digitalRead(encoder0PinA);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), CountA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), StateB, CHANGE);

  Serial.println("Arduino Ready. Waiting for encoder commands...");
}

void loop() {
  valNew = encoder0Pos;
  if (valNew != valOld) {
    Serial.print("Encoder Position: ");
    Serial.println(encoder0Pos);
    valOld = valNew;
  }

  // Check for Serial input from Python
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("SET: ")) {
      targetPosition = command.substring(5).toInt();
      Serial.print("New target position received: ");
      Serial.println(targetPosition);
      pwmRunning = true;
    }
  }

  // Move motor to target position with buffer
  if (pwmRunning) {
    int error = targetPosition - encoder0Pos;
    if (abs(error) > buffer) {
      if (error > 0) {
        digitalWrite(dirPin, LOW);  // Move forward
      } else {
        digitalWrite(dirPin, HIGH); // Move backward
      }
      analogWrite(pwmPin, pwmValue);
    } else {
      analogWrite(pwmPin, 0);  // Stop motor
      pwmRunning = false;
      Serial.println("Target reached within buffer! Resuming lane control...");
    }
  }
}

void CountA() {
  n = digitalRead(encoder0PinA);
  if ((encoder0PinALast == LOW) && (n == HIGH)) {
    encoder0Pos += (m == LOW) ? -1 : 1;
  }
  encoder0PinALast = n;
}

void StateB() {
  m = digitalRead(encoder0PinB);
}
