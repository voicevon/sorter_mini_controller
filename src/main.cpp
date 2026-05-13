#include <Arduino.h>
#include <AccelStepper.h>
#include "pins.h"

// Instantiate steppers. Driver interface: AccelStepper::DRIVER (1)
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

// ============================================================================
// Configuration & Consts
// ============================================================================

// --- Microstep Configuration (must match MS1/MS2/MS3 jumpers on the board) ---
// A4988 / DRV8825: MS pins are set via hardware jumpers, NOT software.
// Verify: all 3 jumpers installed under each driver = 1/16 microstep.
#define MICROSTEP_RESOLUTION  16    // 1, 2, 4, 8, or 16

// --- Stepper Geometry ---
#define MOTOR_FULL_STEPS      200   // steps per revolution (1.8 deg/step motor)
#define GEAR_RATIO            4     // 1:4 gear ratio (motor turns 4x per wheel turn)
// Steps to rotate wheel 90 degrees = (full_steps * microstep * gear_ratio) / 4
#define STEPS_PER_90DEG       (MOTOR_FULL_STEPS * MICROSTEP_RESOLUTION * GEAR_RATIO / 4)  // = 3200

// --- Motion Parameters ---
#define STEPPER_MAX_SPEED     3200.0f  // steps/s  (~1 rev/s of the wheel)
#define STEPPER_ACCELERATION  1600.0f  // steps/s^2
#define HOMING_CONSTANT_SPEED (-800.0f) // steps/s, negative = toward endstop

const long POS_NEUTRAL      =  0;                // Home/Resting position
const long ROTATE_LEFT_POS  = -STEPS_PER_90DEG; // -3200 steps
const long ROTATE_RIGHT_POS =  STEPS_PER_90DEG; // +3200 steps
const unsigned long SLIDE_WAIT_MS = 2000;
const float HOMING_SPEED = HOMING_CONSTANT_SPEED;  // alias used by homing routine

// ============================================================================
// Global State Tracking
// ============================================================================
enum SystemState {
  STATE_IDLE,
  STATE_NEW_BEAT_PREP,
  STATE_MOTORS_MOVING_TO_ROUTE,
  STATE_SLIDING_WAIT,
  STATE_COMPLETED_BEAT,
  STATE_HOMING
};

SystemState currentState = STATE_IDLE;
unsigned long timerMark = 0;

// PIPELINE: Represents objects flowing down the 3 stages
// stageItems[0] -> The item currently at Motor X (Entering stage)
// stageItems[1] -> The item currently at Motor Y
// stageItems[2] -> The item currently at Motor Z
// Value 0 = Empty slot. Values 1,2,3,4 = Target destination IDs.
int stageItems[3] = {0, 0, 0}; 

// ============================================================================
// Simple Input FIFO Queue for incoming Button presses
// ============================================================================
#define QUEUE_MAX 10
int inputQueue[QUEUE_MAX];
int queueHead = 0;
int queueTail = 0;

void pushQueue(int target) {
  int next = (queueHead + 1) % QUEUE_MAX;
  if (next != queueTail) { // Only if queue not full
    inputQueue[queueHead] = target;
    queueHead = next;
    Serial.print("Queued Task -> Target "); Serial.println(target);
  }
}

int popQueue() {
  if (queueHead == queueTail) return 0; // Empty
  int target = inputQueue[queueTail];
  queueTail = (queueTail + 1) % QUEUE_MAX;
  return target;
}

bool isQueueEmpty() {
  return queueHead == queueTail;
}

// ============================================================================
// Helper Functions
// ============================================================================

// Core logic converting current stage and item target into motor target position
long getTargetPosForStage(int stage, int targetID) {
  if (targetID == 0) return POS_NEUTRAL; // Stage is empty

  switch (stage) {
    case 0: // Motor X (First Layer)
      if (targetID == 1) return ROTATE_LEFT_POS; // Target 1 goes Left
      else return ROTATE_RIGHT_POS;              // Target 2, 3, 4 goes Right
      
    case 1: // Motor Y (Second Layer)
      if (targetID == 1) return POS_NEUTRAL; // Has already exited path. Neutral.
      if (targetID == 2) return ROTATE_RIGHT_POS; 
      return ROTATE_LEFT_POS; // Targets 3, 4 go Left/Inner

    case 2: // Motor Z (Third Layer)
      if (targetID == 1 || targetID == 2) return POS_NEUTRAL; // Already exited.
      if (targetID == 3) return ROTATE_LEFT_POS;
      return ROTATE_RIGHT_POS; // Target 4 goes Right
      
    default:
      return POS_NEUTRAL;
  }
}

void runSteppersSimultaneously() {
  stepperX.run();
  stepperY.run();
  stepperZ.run();
}

bool areSteppersMoving() {
  return stepperX.isRunning() || stepperY.isRunning() || stepperZ.isRunning();
}

// Dedicated homing routine running concurrently
void performHomingStep() {
  static bool xHomeDone = false;
  static bool yHomeDone = false;
  static bool zHomeDone = false;

  // Setup variables on initial entry
  static bool homingInit = false;
  if (!homingInit) {
    Serial.println("Initiating Simultaneous Homing...");
    xHomeDone = false;
    yHomeDone = false;
    zHomeDone = false;
    
    // Directly driving speed bypassing standard acceleration
    stepperX.setSpeed(HOMING_SPEED);
    stepperY.setSpeed(HOMING_SPEED);
    stepperZ.setSpeed(HOMING_SPEED);
    
    homingInit = true;
  }

  // Process X
  if (!xHomeDone) {
    // Read endstop (Trigger assumes LOW activation due to INPUT_PULLUP, adjust if needed)
    if (digitalRead(X_HOME_PIN) == LOW) {
      stepperX.stop();
      stepperX.setCurrentPosition(0);
      xHomeDone = true;
      Serial.println("X Homing Completed.");
    } else {
      stepperX.runSpeed();
    }
  }

  // Process Y
  if (!yHomeDone) {
    if (digitalRead(Y_HOME_PIN) == LOW) {
      stepperY.stop();
      stepperY.setCurrentPosition(0);
      yHomeDone = true;
      Serial.println("Y Homing Completed.");
    } else {
      stepperY.runSpeed();
    }
  }

  // Process Z
  if (!zHomeDone) {
    if (digitalRead(Z_HOME_PIN) == LOW) {
      stepperZ.stop();
      stepperZ.setCurrentPosition(0);
      zHomeDone = true;
      Serial.println("Z Homing Completed.");
    } else {
      stepperZ.runSpeed();
    }
  }

  // Check full completion
  if (xHomeDone && yHomeDone && zHomeDone) {
    Serial.println("--- ALL MOTORS HOMED & REZEROED ---");
    // Release drivers / stop explicitly
    stepperX.moveTo(POS_NEUTRAL);
    stepperY.moveTo(POS_NEUTRAL);
    stepperZ.moveTo(POS_NEUTRAL);
    homingInit = false; // Reset for next call
    currentState = STATE_IDLE; 
  }
}

// Debounced quick button scanner with verbose state logging
void scanButtons() {
  static unsigned long lastScan = 0;
  if (millis() - lastScan < 50) return; // 50ms debounce
  lastScan = millis();

  bool b1Now = !digitalRead(BTN_TARGET_1_PIN); // Invert since LOW=pressed
  bool b2Now = !digitalRead(BTN_TARGET_2_PIN);
  bool b3Now = !digitalRead(BTN_TARGET_3_PIN);
  bool b4Now = !digitalRead(BTN_TARGET_4_PIN);

  static bool b1State = false, b2State = false, b3State = false, b4State = false;

  // Debug transitions & Action Queueing
  if (b1Now != b1State) {
    Serial.print("[BTN DEBUG] Pin "); Serial.print(BTN_TARGET_1_PIN);
    Serial.println(b1Now ? " (BTN 1) -> LOW (Pressed)" : " (BTN 1) -> HIGH (Released)");
    if (b1Now) pushQueue(1);
    b1State = b1Now;
  }
  
  if (b2Now != b2State) {
    Serial.print("[BTN DEBUG] Pin "); Serial.print(BTN_TARGET_2_PIN);
    Serial.println(b2Now ? " (BTN 2) -> LOW (Pressed)" : " (BTN 2) -> HIGH (Released)");
    if (b2Now) pushQueue(2);
    b2State = b2Now;
  }

  if (b3Now != b3State) {
    Serial.print("[BTN DEBUG] Pin "); Serial.print(BTN_TARGET_3_PIN);
    Serial.println(b3Now ? " (BTN 3) -> LOW (Pressed)" : " (BTN 3) -> HIGH (Released)");
    if (b3Now) pushQueue(3);
    b3State = b3Now;
  }

  if (b4Now != b4State) {
    Serial.print("[BTN DEBUG] Pin "); Serial.print(BTN_TARGET_4_PIN);
    Serial.println(b4Now ? " (BTN 4) -> LOW (Pressed)" : " (BTN 4) -> HIGH (Released)");
    if (b4Now) pushQueue(4);
    b4State = b4Now;
  }

  int pressCount = 0;
  if (b1State) pressCount++;
  if (b2State) pressCount++;
  if (b3State) pressCount++;
  if (b4State) pressCount++;

  // CRITICAL EVENT: 2 or more buttons pressed triggers HOMING
  if (pressCount >= 2 && currentState != STATE_HOMING) {
    Serial.println("Triggered Multi-Button Event -> Entering Homing Mode.");
    
    // Flush existing pipeline and queues to prioritize calibration
    queueHead = 0; queueTail = 0;
    stageItems[0] = stageItems[1] = stageItems[2] = 0;
    
    currentState = STATE_HOMING;
  }
}

void setup() {
  // Initialize standard onboard LED for visual heartbeat
  pinMode(LED_PIN, OUTPUT);

  // Initialize serial for debugging "Hello World"
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  Serial.println("--- Sorter Mini Controller ---");
  Serial.println("Hardware: MKS BASE V1.6 (ATmega2560)");
  Serial.println("Firmware Status: Loading Motors & Buttons...");

  // --- Initialize Stepper Enable Pins ---
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);
  // Set to LOW to enable drivers, or HIGH to disable
  digitalWrite(X_ENABLE_PIN, LOW); 
  digitalWrite(Y_ENABLE_PIN, LOW);
  digitalWrite(Z_ENABLE_PIN, LOW);

  // --- Configure AccelStepper Defaults ---
  stepperX.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperX.setAcceleration(STEPPER_ACCELERATION);

  stepperY.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperY.setAcceleration(STEPPER_ACCELERATION);

  stepperZ.setMaxSpeed(STEPPER_MAX_SPEED);
  stepperZ.setAcceleration(STEPPER_ACCELERATION);

  // --- Initialize Endstop Inputs (Use pull-ups) ---
  pinMode(X_HOME_PIN, INPUT_PULLUP);
  pinMode(Y_HOME_PIN, INPUT_PULLUP);
  pinMode(Z_HOME_PIN, INPUT_PULLUP);

  // --- Initialize Goal Selection Buttons (Use pull-ups) ---
  pinMode(BTN_TARGET_1_PIN, INPUT_PULLUP);
  pinMode(BTN_TARGET_2_PIN, INPUT_PULLUP);
  pinMode(BTN_TARGET_3_PIN, INPUT_PULLUP);
  pinMode(BTN_TARGET_4_PIN, INPUT_PULLUP);

  Serial.println("Hardware and Steppers initialization completed.");
}

void loop() {
  // 1. Handle Background Monitoring & Button Inputs
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 1000) {
    lastHeartbeat = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  scanButtons();

  // 2. Process State Machine for Beat Synchronization
  switch (currentState) {
    case STATE_IDLE:
      // Start next cycle only if queue isn't empty OR pipeline still has processing objects
      if (!isQueueEmpty() || stageItems[0] != 0 || stageItems[1] != 0 || stageItems[2] != 0) {
        currentState = STATE_NEW_BEAT_PREP;
      }
      break;

    case STATE_NEW_BEAT_PREP:
      Serial.println("\n--- NEW BEAT STARTED ---");
      // 1. Advancing the Pipeline (The Shifting mechanism)
      stageItems[2] = stageItems[1]; // Stage 1 object goes to Stage 2
      stageItems[1] = stageItems[0]; // Stage 0 object goes to Stage 1
      stageItems[0] = popQueue();    // Pop fresh item from buffer into Stage 0
      
      // Print Status 
      Serial.print("Pipeline State: [M1: Target "); Serial.print(stageItems[0]);
      Serial.print("] -> [M2: Target "); Serial.print(stageItems[1]);
      Serial.print("] -> [M3: Target "); Serial.print(stageItems[2]);
      Serial.println("]");

      // 2. Configure destination positions for current step
      stepperX.moveTo(getTargetPosForStage(0, stageItems[0]));
      stepperY.moveTo(getTargetPosForStage(1, stageItems[1]));
      stepperZ.moveTo(getTargetPosForStage(2, stageItems[2]));
      
      currentState = STATE_MOTORS_MOVING_TO_ROUTE;
      break;

    case STATE_MOTORS_MOVING_TO_ROUTE:
      runSteppersSimultaneously();
      if (!areSteppersMoving()) {
        Serial.println("Reached Routing Positions. Waiting for slide...");
        timerMark = millis();
        currentState = STATE_SLIDING_WAIT;
      }
      break;

    case STATE_SLIDING_WAIT:
      // Objects slide through the tree naturally. Keep pumping steppers (though idle).
      runSteppersSimultaneously();
      if (millis() - timerMark >= SLIDE_WAIT_MS) {
        // Slide complete. Motors stay at current routing position until the next
        // beat issues a fresh moveTo() — no unnecessary return-to-neutral trip.
        Serial.println("Slide delay ended. Beat Finished.");
        currentState = STATE_COMPLETED_BEAT;
      }
      break;

    case STATE_COMPLETED_BEAT:
      // End of beat cooldown / prep
      currentState = STATE_IDLE; 
      break;

    case STATE_HOMING:
      performHomingStep();
      break;
  }

  // Make sure run() is constantly triggered just in case
  runSteppersSimultaneously();
}
