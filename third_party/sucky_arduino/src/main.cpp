
#include <Arduino.h>
#include <Servo.h>

//function prototypes
void processCommand();
void setCycloneState(bool state);
void setShakerState(bool state);
void setDoorState(bool open);
void updateDoorMovement();
void printStatus();
void printHelp();
void serialEvent();

constexpr uint8_t PWM_PIN = 10;        // ESC PWM pin (OC1B on UNO)
constexpr uint8_t DOOR_LEFT_PIN = 5;   // Left door servo pin
constexpr uint8_t DOOR_RIGHT_PIN = 3;  // Right door servo pin
constexpr uint8_t SHAKER_RELAY_PIN = 12; // Shaker motor relay pin

// ESC pulse configuration
constexpr uint16_t PULSE_MIN = 1000;   // µs (idle)
constexpr uint16_t PULSE_MAX = 2980;   // µs (full power)

// Door servo positions
constexpr uint8_t DOOR_LEFT_CLOSED = 180;    // Left door closed position
constexpr uint8_t DOOR_LEFT_OPEN = 0;    // Left door open position
constexpr uint8_t DOOR_RIGHT_CLOSED = 0; // Right door closed position  
constexpr uint8_t DOOR_RIGHT_OPEN = 180;     // Right door open position

// Global objects and state
Servo doorLeft;
Servo doorRight;
Servo esc;

bool cycloneOn = false;
bool doorsOpen = false;
bool shakerOn = false;
String inputString = "";
bool stringComplete = false;

// Safety timeout (10 minutes max cyclone runtime)
constexpr unsigned long CYCLONE_TIMEOUT_MS = 600000;
unsigned long cycloneStartTime = 0;

// Door movement variables
bool doorMoving = false;
unsigned long lastDoorUpdate = 0;
constexpr unsigned long DOOR_STEP_DELAY = 15; // milliseconds between steps
constexpr uint8_t DOOR_STEP_SIZE = 2; // degrees per step
uint8_t currentLeftPos = DOOR_LEFT_CLOSED;
uint8_t currentRightPos = DOOR_RIGHT_CLOSED;
uint8_t targetLeftPos = DOOR_LEFT_CLOSED;
uint8_t targetRightPos = DOOR_RIGHT_CLOSED;

void setup() {
  Serial.begin(115200); // Set serial baud rate

  // Initialize shaker relay
  pinMode(SHAKER_RELAY_PIN, OUTPUT);
  digitalWrite(SHAKER_RELAY_PIN, LOW); // Start with shaker off
  shakerOn = false;

  doorLeft.attach(5);  // Attach left door servo to pin 5
  doorRight.attach(3); // Attach right door servo to pin 3
  
  doorLeft.write(180);  // Set left door to neutral position 180-closed
  doorRight.write(0); // Set right door to neutral position 0-closed
  doorsOpen = false;  // Initialize doors as closed
  
  // Initialize door positions
  currentLeftPos = DOOR_LEFT_CLOSED;
  currentRightPos = DOOR_RIGHT_CLOSED;
  targetLeftPos = DOOR_LEFT_CLOSED;
  targetRightPos = DOOR_RIGHT_CLOSED;

  esc.attach(PWM_PIN);            // 50 Hz by default (Timer-1)

  /* --------- NORMAL ARM --------- */
  esc.writeMicroseconds(PULSE_MIN);   // idle pulse
  delay(3000);                        // wait for ESC “ready” beep
  /* --------------------------------*/

  // Wait for serial connection to stabilize
  while (!Serial) {
    delay(10);
  }
  delay(500); // Additional stabilization time
  
  // Send ready signal
  Serial.println("ARDUINO_READY");

}

void loop() {
  // Safety check: Auto-shutdown cyclone after timeout
  if (cycloneOn && (millis() - cycloneStartTime > CYCLONE_TIMEOUT_MS)) {
    cycloneOn = false;
    esc.writeMicroseconds(PULSE_MIN);
    Serial.println("SAFETY: Cyclone auto-shutdown after timeout");
  }
  
  // Update door movement
  updateDoorMovement();
  
  // Handle serial input
  if (stringComplete) {
    processCommand();
  }
  
  // Small delay to prevent overwhelming the system
  delay(10);
}

void processCommand() {
  inputString.trim();
  inputString.toUpperCase();
  
  if (inputString == "CYCLONE_ON") {
    setCycloneState(true);
  }
  else if (inputString == "CYCLONE_OFF") {
    setCycloneState(false);
  }
  else if (inputString == "SHAKER_ON") {
    setShakerState(true);
  }
  else if (inputString == "SHAKER_OFF") {
    setShakerState(false);
  }
  else if (inputString == "DOOR_OPEN") {
    setDoorState(true);
  }
  else if (inputString == "DOOR_CLOSE") {
    setDoorState(false);
  }
  else if (inputString == "STATUS") {
    printStatus();
  }
  else if (inputString == "HELP") {
    printHelp();
  }
  else {
    Serial.print("ERROR: Unknown command '");
    Serial.print(inputString);
    Serial.println("'. Send HELP for available commands.");
  }
  
  // Clear the string for next command
  inputString = "";
  stringComplete = false;
}

void setCycloneState(bool state) {
  if (state && !cycloneOn) {
    cycloneOn = true;
    cycloneStartTime = millis();
    
    // Send immediate acknowledgment
    Serial.println("CYCLONE_STARTING");
    Serial.flush(); // Ensure immediate transmission
    
    // Smooth exponential acceleration curve
    const int steps = 100;
    const float startPulse = 1600.0;
    const float endPulse = PULSE_MAX;
    
    for (int i = 0; i <= steps; i++) {
      // Exponential curve: starts slow, accelerates faster toward the end
      float progress = (float)i / steps;
      float exponentialProgress = (exp(progress * 2.0) - 1.0) / (exp(2.0) - 1.0);
      
      int pulseWidth = startPulse + (endPulse - startPulse) * exponentialProgress;
      esc.writeMicroseconds(pulseWidth);
      
      // Process serial during delays to maintain responsiveness
      delay(10);
      if (Serial.available()) {
        serialEvent();
      }
      delay(5);
    }
    Serial.println("Cyclone ON");
  }
  else if (!state && cycloneOn) {
    cycloneOn = false;
    
    // Send immediate acknowledgment
    Serial.println("CYCLONE_STOPPING");
    Serial.flush(); // Ensure immediate transmission
    
    // Smooth logarithmic deceleration curve
    const int steps = 80;
    const float startPulse = PULSE_MAX;
    const float endPulse = 1600.0;
    
    for (int i = 0; i <= steps; i++) {
      // Logarithmic curve: starts fast, slows down toward the end
      float progress = (float)i / steps;
      float logarithmicProgress = log(1.0 + progress * (exp(1.0) - 1.0));
      
      int pulseWidth = startPulse - (startPulse - endPulse) * logarithmicProgress;
      esc.writeMicroseconds(pulseWidth);
      
      // Process serial during delays to maintain responsiveness
      delay(15);
      if (Serial.available()) {
        serialEvent();
      }
      delay(5);
    }
    esc.writeMicroseconds(PULSE_MIN);
    Serial.println("Cyclone OFF");
  }
  else {
    Serial.print("Cyclone already ");
    Serial.println(cycloneOn ? "ON" : "OFF");
  }
}

void setShakerState(bool state) {
  if (state && !shakerOn) {
    shakerOn = true;
    digitalWrite(SHAKER_RELAY_PIN, HIGH);
    Serial.println("Shaker ON");
  }
  else if (!state && shakerOn) {
    shakerOn = false;
    digitalWrite(SHAKER_RELAY_PIN, LOW);
    Serial.println("Shaker OFF");
  }
  else {
    Serial.print("Shaker already ");
    Serial.println(shakerOn ? "ON" : "OFF");
  }
}

void updateDoorMovement() {
  if (!doorMoving) return;
  
  if (millis() - lastDoorUpdate >= DOOR_STEP_DELAY) {
    bool leftDone = false;
    bool rightDone = false;
    
    // Update left door position
    if (currentLeftPos != targetLeftPos) {
      if (currentLeftPos < targetLeftPos) {
        currentLeftPos = min(currentLeftPos + DOOR_STEP_SIZE, targetLeftPos);
      } else {
        currentLeftPos = max(currentLeftPos - DOOR_STEP_SIZE, targetLeftPos);
      }
      doorLeft.write(currentLeftPos);
    } else {
      leftDone = true;
    }
    
    // Update right door position
    if (currentRightPos != targetRightPos) {
      if (currentRightPos < targetRightPos) {
        currentRightPos = min(currentRightPos + DOOR_STEP_SIZE, targetRightPos);
      } else {
        currentRightPos = max(currentRightPos - DOOR_STEP_SIZE, targetRightPos);
      }
      doorRight.write(currentRightPos);
    } else {
      rightDone = true;
    }
    
    // Check if movement is complete
    if (leftDone && rightDone) {
      doorMoving = false;
      Serial.print("Doors ");
      Serial.println(doorsOpen ? "OPEN" : "CLOSED");
    }
    
    lastDoorUpdate = millis();
  }
}

void setDoorState(bool open) {
  if (open && !doorsOpen) {
    doorsOpen = true;
    targetLeftPos = DOOR_LEFT_OPEN;
    targetRightPos = DOOR_RIGHT_OPEN;
    doorMoving = true;
    Serial.println("Doors opening...");
  }
  else if (!open && doorsOpen) {
    doorsOpen = false;
    targetLeftPos = DOOR_LEFT_CLOSED;
    targetRightPos = DOOR_RIGHT_CLOSED;
    doorMoving = true;
    Serial.println("Doors closing...");
  }
  else {
    Serial.print("Doors already ");
    Serial.println(doorsOpen ? "OPEN" : "CLOSED");
  }
}

void printStatus() {
  Serial.println("=== SYSTEM STATUS ===");
  Serial.print("Cyclone: ");
  Serial.println(cycloneOn ? "ON" : "OFF");
  Serial.print("Shaker: ");
  Serial.println(shakerOn ? "ON" : "OFF");
  Serial.print("Doors: ");
  Serial.print(doorsOpen ? "OPEN" : "CLOSED");
  if (doorMoving) {
    Serial.print(" (MOVING)");
  }
  Serial.println();
  Serial.print("Door Positions - Left: ");
  Serial.print(currentLeftPos);
  Serial.print("°, Right: ");
  Serial.print(currentRightPos);
  Serial.println("°");
  Serial.print("ESC Pulse: ");
  Serial.print(cycloneOn ? PULSE_MAX : PULSE_MIN);
  Serial.println(" µs");
  if (cycloneOn) {
    unsigned long runtime = (millis() - cycloneStartTime) / 1000;
    Serial.print("Cyclone Runtime: ");
    Serial.print(runtime);
    Serial.println(" seconds");
  }
  Serial.println("====================");
}

void printHelp() {
  Serial.println("=== AVAILABLE COMMANDS ===");
  Serial.println("CYCLONE_ON  - Turn cyclone ON");
  Serial.println("CYCLONE_OFF - Turn cyclone OFF");
  Serial.println("SHAKER_ON   - Turn shaker motor ON");
  Serial.println("SHAKER_OFF  - Turn shaker motor OFF");
  Serial.println("DOOR_OPEN   - Open doors");
  Serial.println("DOOR_CLOSE  - Close doors");
  Serial.println("STATUS      - Show system status");
  Serial.println("HELP        - Show this help");
  Serial.println("===========================");
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX.
  This routine is run between each time loop() runs, so using delay inside
  loop can delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // Get the new byte
    char inChar = (char)Serial.read();
    
    // If the incoming character is a newline, set a flag so the main loop can
    // do something about it
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      // Add it to the inputString
      inputString += inChar;
    }
  }
}