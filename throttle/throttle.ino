// Pin Definitions
#define ESP_PIN 5        // Pin connected to ESP state
#define BRAKE_DIR_PIN 13 // Direction pin for brake
#define BRAKE_PWM_PIN 11 // PWM pin for brake
#define ACCN_1_PIN 7     // Motor driver input 1 for acceleration
#define ACCN_2_PIN 8     // Motor driver input 2 for acceleration
#define ACCN_PWM_PIN 9   // PWM pin for acceleration
#define SAFETY_PIN 2  // safety handles

// Variables
int esp_state;       // Stores the state of the ESP pin
int safety_state = 0;  
int brake_active = 0;    // Tracks if brakes are active
int currentspeed = 0;    // Current speed of the vehicle
int maxspeed = 255; 
int speed = 0; 

String command;     


void setup() {
  // Initialize Serial communication
  Serial.begin(9600);

  // Set pin modes
  pinMode(ESP_PIN, INPUT);
  pinMode(SAFETY_PIN, INPUT_PULLUP);
  pinMode(BRAKE_DIR_PIN, OUTPUT);
  pinMode(BRAKE_PWM_PIN, OUTPUT);
  pinMode(ACCN_1_PIN, OUTPUT);
  pinMode(ACCN_2_PIN, OUTPUT);
  pinMode(ACCN_PWM_PIN, OUTPUT);

  // Initialize all outputs to LOW
  digitalWrite(BRAKE_DIR_PIN, LOW);
  digitalWrite(BRAKE_PWM_PIN, LOW);
  digitalWrite(ACCN_1_PIN, HIGH);         // Set motor direction
  digitalWrite(ACCN_2_PIN, LOW);
  analogWrite(ACCN_PWM_PIN, 0);
  delay(5000);
}

void loop() {
  // Check for incoming serial commands
  esp_state = digitalRead(ESP_PIN);
  safety_state= digitalRead(SAFETY_PIN);
  Serial.println("esp_state");
  Serial.println(esp_state);
  Serial.println("handle state");
  Serial.println(safety_state);

  if ((safety_state==1)&&brake_active==0){
      brake_active = 1;
      analogWrite(ACCN_PWM_PIN, 0); // Set PWM to 0
      digitalWrite(ACCN_1_PIN, LOW);
      digitalWrite(ACCN_2_PIN, LOW);
      currentspeed = 0;
      Serial.println("Acceleration stopped");

      // Engage brake (e.g., forward direction)
      digitalWrite(BRAKE_DIR_PIN, HIGH); // Direction HIGH
      analogWrite(BRAKE_PWM_PIN, 255);   // Full speed (PWM)
      Serial.println("Brakes activated");
      delay(3000);
      analogWrite(BRAKE_PWM_PIN, 0);     // Stop PWM

      // Reverse direction (to disengage brake, or simulate release if brake is motorized)
      digitalWrite(BRAKE_DIR_PIN, LOW);  // Change direction
      analogWrite(BRAKE_PWM_PIN, 255);   // Again full speed
      Serial.println("Brakes released");
      delay(2000); //lesser than activation time so that the brake wire doesnt wind in the other dirn
      brake_active=0;

      // Stop motor
      analogWrite(BRAKE_PWM_PIN, 0);     // Stop PWM

  }

  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n'); // Read the incoming command
    command.trim();}   // Remove any extra whitespace
  

  if (command == "BRAKE_ON") {
      // Activate brakes
      brake_active = 1;
      digitalWrite(BRAKE_DIR_PIN, HIGH); // Direction HIGH
      analogWrite(BRAKE_PWM_PIN, 255);   // Full speed (PWM)
      Serial.println("Brakes activated");
      delay(3000);


    } else if (command == "BRAKE_OFF") {
      // Release brakes
      if (brake_active == 1){
        analogWrite(BRAKE_PWM_PIN, 0);     // Stop PWM
        digitalWrite(BRAKE_DIR_PIN, LOW);  // Change direction
        analogWrite(BRAKE_PWM_PIN, 255);   // Again full speed
        Serial.println("Brakes released");
        delay(2000);
        brake_active=0;

        // Stop motor
        analogWrite(BRAKE_PWM_PIN, 0);     // Stop PWM 
        delay(1500);
        Serial.println("Brakes released");
      }

    } else if (command.length() > 0 && command.toInt() >= 0) {
      // Handle numeric values directly (without "ACCEL_")
      int newValue = command.toInt(); // Convert command to integer
      if (newValue >= 0 && newValue <= 255) {
        currentspeed = newValue;
        analogWrite(ACCN_PWM_PIN, currentspeed); // Set PWM for acceleration
        digitalWrite(ACCN_1_PIN, HIGH);         // Set motor direction
        digitalWrite(ACCN_2_PIN, LOW);
        Serial.print("Acceleration set to: ");
        Serial.println(currentspeed);
      } else {
        Serial.println("Invalid speed value");
      }
      } else if (command == "ACCEL_OFF") {
      // Stop acceleration
      analogWrite(ACCN_PWM_PIN, 0); // Set PWM to 0
      currentspeed = 0;
      Serial.println("Acceleration stopped");
    } else {
      Serial.println("No commands");
    }
  
  delay(50);
}