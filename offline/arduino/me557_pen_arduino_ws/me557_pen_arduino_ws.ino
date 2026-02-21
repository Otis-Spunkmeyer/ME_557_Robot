#include <Dynamixel2Arduino.h>
#include "ace_trajectory_data.h"

// Serial port connected to Dynamixel bus and host PC
#define DXL_SERIAL Serial1
#define PC_SERIAL Serial

using namespace ControlTableItem;

// Dynamixel driver instance
Dynamixel2Arduino d2a(DXL_SERIAL);

// --- GLOBAL CONFIGURATION ---
// Default speeds and smoothing parameters used when commanding servos.
const int GLOBAL_SPEED_RAW = 40;
const int SWEEP_SPEED_RAW = 60;
const int TRAJECTORY_SPEED_RAW = 40;
const int ACCEL_VAL_SMOOTH = 15;

// State for safely tucking/untucking motor 5 when motor 4 moves
float previousM5Angle = 180.0;
bool m5IsTucked = false;

// Per-motor calibration, mapping physical to logical angle ranges and offsets
struct RobotMotor {
  uint8_t id;            // Dynamixel ID
  float physicalOffset;  // Physical zero offset used to align to logical 180deg
  float lowerLimit;      // Logical lower limit (deg)
  float upperLimit;      // Logical upper limit (deg)
};

RobotMotor motors[] = {
  {1, 117.0, 120.0, 220.0},
  {2, 187.0, 120.0, 240.0},
  {3, 189.0, 120.0, 240.0},
  {4, 150.0,  40.0, 240.0},
  {5, 150.0, 100.0, 260.0},
  {6, 150.0, 140.0, 300.0}
};

const uint8_t MOVEIT_TO_SERVO_ID[5] = {1, 2, 4, 5, 6};
const char* MOVEIT_JOINT_NAMES[5] = {
  "Motor1_joint", "Motor2_L", "Motor4_elb", "Motor5_wr", "Joint_EE"
};

// --- MoveIt (radians) <-> logical (degrees) calibration ---
// These values map MoveIt joint space to the sketch's logical degree space.
// Tune once on hardware and keep consistent with the MoveIt export tool.
const float MOVEIT_HOME_RAD[5] = {
  0.0f,       // Motor1_joint -> ID1
   0.0f,    // Motor2_L     -> ID2 (ID3 mirrors via setAngle)
  0.0f,  // Motor4_elb   -> ID4
   0.0f,    // Motor5_wr    -> ID5
  0.0f        // Joint_EE     -> ID6
};
// Direction signs to correct coordinate system differences
const float MOVEIT_DIR_SIGN[5] = {-1.0f, 1.0f, 1.0f, -1.0f, 1.0f};

// --- Utility ---
// Return pointer to `RobotMotor` for the given dynamixel `id`, or nullptr.
RobotMotor* findMotor(uint8_t id) {
  for (int i = 0; i < 6; i++) {
    if (motors[i].id == id) return &motors[i];
  }
  return nullptr;
}

// Convert raw Dynamixel position units to physical degrees depending on model
float rawToPhysicalDegrees(uint16_t model, int32_t raw) {
  if (model == 12) {
    // AX-12 style 10-bit resolution over 300 deg
    return ((float)raw) * (300.0f / 1023.0f);
  }
  // e.g., XM/XH 12-bit over 360 deg
  return ((float)raw) * (360.0f / 4095.0f);
}

// Convert from this sketch's logical degrees to MoveIt joint radians
float logicalToMoveitRad(uint8_t idx, float logicalDeg) {
  const float DEG2RAD = 0.01745329252f;
  return MOVEIT_HOME_RAD[idx] +
         ((logicalDeg - 180.0f) * DEG2RAD) / MOVEIT_DIR_SIGN[idx];
}

// --- MOTION SMOOTHING ---
// Apply motion smoothing / compliance parameters appropriate for model
void applySmoothing(uint8_t id, uint16_t model) {
  if (model == 12) {
    // Older AX-style servo: use compliance settings
    d2a.writeControlTableItem(CW_COMPLIANCE_SLOPE, id, 32);
    d2a.writeControlTableItem(CCW_COMPLIANCE_SLOPE, id, 32);
    d2a.writeControlTableItem(CW_COMPLIANCE_MARGIN, id, 2);
    d2a.writeControlTableItem(CCW_COMPLIANCE_MARGIN, id, 2);
  } else {
    // Newer servos: set profile acceleration for smoother moves
    d2a.writeControlTableItem(PROFILE_ACCELERATION, id, ACCEL_VAL_SMOOTH);
  }
}

// Low-level: command a servo by physical degrees (0..360 or 0..300 depending on model)
// Ensures smoothing is applied once per servo and writes speed + goal position.
void executeMove(uint8_t id, float physicalDegrees, int speed = GLOBAL_SPEED_RAW) {
  uint16_t model = d2a.getModelNumber(id);
  static bool smoothed[11] = {false};
  if (id < 11 && !smoothed[id]) {
    applySmoothing(id, model);
    smoothed[id] = true;
  }
  d2a.writeControlTableItem(MOVING_SPEED, id, speed);
  uint16_t bitValue = (model == 12)
    ? (uint16_t)(constrain(physicalDegrees, 0, 300) * (1023.0 / 300.0))
    : (uint16_t)(constrain(physicalDegrees, 0, 360) * (4095.0 / 360.0));
  d2a.torqueOn(id);
  d2a.setGoalPosition(id, bitValue);
}

// Convert logical angle (user-facing degrees centered at 180) to physical servo degrees
// Applies per-motor limits and physical offset calibration.
float logicalToPhysical(uint8_t id, float logicalAngle) {
  RobotMotor* m = findMotor(id);
  if (!m) return logicalAngle;
  float safeLogical = constrain(logicalAngle, m->lowerLimit, m->upperLimit);
  return safeLogical + (m->physicalOffset - 180.0);
}

// Convert physical servo degrees back into logical degrees used by this sketch
float physicalToLogical(uint8_t id, float physicalAngle) {
  RobotMotor* m = findMotor(id);
  if (!m) return physicalAngle;
  return physicalAngle - (m->physicalOffset - 180.0);
}

// Read current servo position and return both logical and physical angles.
// Returns false if the servo does not respond to ping.
bool readPresentLogical(uint8_t id, float &logicalOut, float &physicalOut) {
  if (!d2a.ping(id)) return false;
  uint16_t model = d2a.getModelNumber(id);
  int32_t raw = d2a.readControlTableItem(PRESENT_POSITION, id);
  physicalOut = rawToPhysicalDegrees(model, raw);
  logicalOut = physicalToLogical(id, physicalOut);
  return true;
}

// High-level: set a motor to a `logicalAngle` (deg) while enforcing safety behaviors:
// - Tucks motor 5 when motor 4 moves into dangerous range
// - Mirrors IDs 2/3 so one follows the other for a mirrored joint pair
void setAngle(uint8_t id, float logicalAngle, int speed = GLOBAL_SPEED_RAW) {
  if (id == 5 && !m5IsTucked) previousM5Angle = logicalAngle;

  if (id == 4) {
    // If motor 4 is commanded to tuck, move M5 out of the way first
    if (logicalAngle < 70.0 && !m5IsTucked) {
      PC_SERIAL.println("!!! M4 DANGER: Tucking M5.");
      m5IsTucked = true;
      executeMove(5, logicalToPhysical(5, 100.0), speed);
      delay(800);
    }
    else if (logicalAngle >= 70.0 && m5IsTucked) {
      m5IsTucked = false;
      executeMove(5, logicalToPhysical(5, previousM5Angle), speed);
    }
  }

  executeMove(id, logicalToPhysical(id, logicalAngle), speed);

  // Mirror joint for IDs 2<->3 (physical arrangement requires mirrored values)
  if (id == 3 || id == 2) {
    uint8_t partnerID = (id == 3) ? 2 : 3;
    executeMove(partnerID, logicalToPhysical(partnerID, 360.0 - logicalAngle), speed);
  }
}

// Convert MoveIt radians to this sketch's logical degrees (inverse of above)
float moveitRadToLogical(uint8_t idx, float rad) {
  const float RAD2DEG = 57.2957795f;
  return 180.0f + MOVEIT_DIR_SIGN[idx] * (rad - MOVEIT_HOME_RAD[idx]) * RAD2DEG;
}


// Helper: check whether a servo reached a logical target within `tolerance` degrees.
// Returns false if the servo cannot be read (useful to block until read succeeds).
bool isAtGoal(uint8_t id, float targetLogical, float tolerance = 4.0) {
  float currentLogical, currentPhysical;
  if (!readPresentLogical(id, currentLogical, currentPhysical)) return false; 
  return abs(currentLogical - targetLogical) < tolerance;
}
/*
old move it trajectory
 void playMoveitTrajectory() {
  PC_SERIAL.println("\n--- PLAYING EXPORTED MOVEIT TRAJECTORY (555) ---");
  if (kAceMoveitTrajectoryCount == 0) {
    PC_SERIAL.println("No trajectory points loaded in ace_trajectory_data.h");
    return;
  }
    for (size_t i = 0; i < kAceMoveitTrajectoryCount; ++i) {
    const MoveitJointSampleRad &p = kAceMoveitTrajectory[i];

    float a1 = moveitRadToLogical(0, p.j1_rad);
    float a2 = moveitRadToLogical(1, p.j2l_rad);
    float a4 = moveitRadToLogical(2, p.j4_rad);
    float a5 = moveitRadToLogical(3, p.j5_rad);
    float a6 = moveitRadToLogical(4, p.j6_rad);

    setAngle(1, a1, TRAJECTORY_SPEED_RAW);
    setAngle(2, a2, TRAJECTORY_SPEED_RAW);  // ID3 follows as mirror in setAngle()
    setAngle(4, a4, TRAJECTORY_SPEED_RAW);
    setAngle(5, a5, TRAJECTORY_SPEED_RAW);
    setAngle(6, a6, TRAJECTORY_SPEED_RAW);

    delay((unsigned long)p.dt_ms);
  }

  PC_SERIAL.println("--- TRAJECTORY PLAYBACK COMPLETE ---");
}
*/

// Play a MoveIt-exported trajectory using smart, keyframe-based waiting.
// For each keyframe we send commands and then poll positions until joints
// either reach targets or stall for a timeout (skip to next keyframe).
void playMoveitTrajectory() {
  PC_SERIAL.println("\n--- PLAYING KEYFRAME WAYPOINTS (SMART WAIT) ---");
  if (kAceMoveitTrajectoryCount == 0) {
    PC_SERIAL.println("No trajectory points loaded.");
    return;
  }

  for (size_t i = 0; i < kAceMoveitTrajectoryCount; ++i) {
    const MoveitJointSampleRad &p = kAceMoveitTrajectory[i];
    
    // 1. Calculate Targets
    float targets[5];
    targets[0] = moveitRadToLogical(0, p.j1_rad);
    targets[1] = moveitRadToLogical(1, p.j2l_rad);
    targets[2] = moveitRadToLogical(2, p.j4_rad);
    targets[3] = moveitRadToLogical(3, p.j5_rad);
    targets[4] = moveitRadToLogical(4, p.j6_rad);
    
    // Map indices to IDs for easier looping
    uint8_t ids[5] = {1, 2, 4, 5, 6};

    PC_SERIAL.print("Moving to Keyframe "); PC_SERIAL.print(i); PC_SERIAL.println("...");

    // 2. Send Commands 
    for(int k=0; k<5; k++) {
       setAngle(ids[k], targets[k], 20);
    }

    // 3. SMART WAIT LOOP
    unsigned long lastProgressTime = millis();
    float lastPositions[5] = {0,0,0,0,0}; 

    while (true) {
      bool allArrived = true;
      bool isMoving = false;
      
      for(int k=0; k<5; k++) {
        float currentLog, currentPhys;
        readPresentLogical(ids[k], currentLog, currentPhys);
        
        if (abs(currentLog - targets[k]) > 4.0) { 
          allArrived = false; 
        }

        if (abs(currentLog - lastPositions[k]) > 0.5) {
          isMoving = true;
        }
        lastPositions[k] = currentLog;
      }

      if (allArrived) break;
      if (isMoving) lastProgressTime = millis();
      if (millis() - lastProgressTime > 5000) {
        PC_SERIAL.println(" -> STALLED: Skipping to next point.");
        break;
      }
      delay(100);
    }
  }

  PC_SERIAL.println("--- DONE ---");
}

// Print a concise report of each servo's logical and physical positions,
// and provide a MoveIt-space estimate for the mapped joints.
void printPresentStateReport() {
  PC_SERIAL.println("\n--- PRESENT JOINT REPORT (901) ---");
  for (int i = 0; i < 6; i++) {
    float logical = 0.0f, physical = 0.0f;
    uint8_t id = motors[i].id;
    if (!readPresentLogical(id, logical, physical)) {
      PC_SERIAL.print("ID "); PC_SERIAL.print(id); PC_SERIAL.println(": not responding");
      continue;
    }
    PC_SERIAL.print("ID "); PC_SERIAL.print(id);
    PC_SERIAL.print(" logical="); PC_SERIAL.print(logical, 3);
    PC_SERIAL.print(" deg, physical="); PC_SERIAL.print(physical, 3);
    PC_SERIAL.println(" deg");
  }

  PC_SERIAL.println("MoveIt joint estimate from current logical angles:");
  for (uint8_t j = 0; j < 5; ++j) {
    uint8_t servoID = MOVEIT_TO_SERVO_ID[j];
    float logical = 0.0f, physical = 0.0f;
    if (!readPresentLogical(servoID, logical, physical)) continue;
    float estRad = logicalToMoveitRad(j, logical);
    PC_SERIAL.print("  "); PC_SERIAL.print(MOVEIT_JOINT_NAMES[j]);
    PC_SERIAL.print(" -> est_rad="); PC_SERIAL.print(estRad, 6);
    PC_SERIAL.print(" (from ID "); PC_SERIAL.print(servoID); PC_SERIAL.println(")");
  }
  PC_SERIAL.println("---------------------------------");
}

// Interactive helper to determine correct `MOVEIT_DIR_SIGN` and `MOVEIT_HOME_RAD`
// for each MoveIt joint by prompting the user to observe direction of motion.
void guidedMoveitSignTest() {
  PC_SERIAL.println("\n--- SIGN TEST (902) ---");
  PC_SERIAL.println("Select MoveIt joint index:");
  PC_SERIAL.println(" 1: Motor1_joint (ID1)");
  PC_SERIAL.println(" 2: Motor2_L (ID2, ID3 mirrors)");
  PC_SERIAL.println(" 3: Motor4_elb (ID4)");
  PC_SERIAL.println(" 4: Motor5_wr (ID5)");
  PC_SERIAL.println(" 5: Joint_EE (ID6)");
  PC_SERIAL.println(" 999: exit");

  while (PC_SERIAL.available() == 0);
  int idx1 = PC_SERIAL.parseInt();
  while (PC_SERIAL.available() > 0) PC_SERIAL.read();
  if (idx1 == 999) return;
  if (idx1 < 1 || idx1 > 5) {
    PC_SERIAL.println("Invalid index.");
    return;
  }

  uint8_t idx = (uint8_t)(idx1 - 1);
  uint8_t servoID = MOVEIT_TO_SERVO_ID[idx];

  PC_SERIAL.print("Delta logical degrees (suggest 8): ");
  while (PC_SERIAL.available() == 0);
  float delta = PC_SERIAL.parseFloat();
  while (PC_SERIAL.available() > 0) PC_SERIAL.read();
  if (delta <= 0.0f) delta = 8.0f;

  float startLogical = 180.0f, startPhysical = 0.0f;
  if (!readPresentLogical(servoID, startLogical, startPhysical)) {
    PC_SERIAL.println("Target servo not responding.");
    return;
  }

  PC_SERIAL.print("Testing "); PC_SERIAL.print(MOVEIT_JOINT_NAMES[idx]);
  PC_SERIAL.print(" on ID "); PC_SERIAL.println(servoID);
  PC_SERIAL.print("Start logical = "); PC_SERIAL.println(startLogical, 3);

  float target = constrain(startLogical + delta, findMotor(servoID)->lowerLimit, findMotor(servoID)->upperLimit);
  setAngle(servoID, target, SWEEP_SPEED_RAW);
  delay(1800);

  float endLogical = target;
  float endPhysical = 0.0f;
  readPresentLogical(servoID, endLogical, endPhysical);
  PC_SERIAL.print("End logical = "); PC_SERIAL.println(endLogical, 3);
  PC_SERIAL.println("Did this move in + MoveIt direction?");
  PC_SERIAL.println(" 1 = yes");
  PC_SERIAL.println(" 0 = no, flip");

  while (PC_SERIAL.available() == 0);
  int yes = PC_SERIAL.parseInt();
  while (PC_SERIAL.available() > 0) PC_SERIAL.read();

  float suggestedSign = (yes == 1) ? MOVEIT_DIR_SIGN[idx] : -MOVEIT_DIR_SIGN[idx];
  float suggestedHome = MOVEIT_HOME_RAD[idx] +
      ((endLogical - 180.0f) * 0.01745329252f) / suggestedSign;

  PC_SERIAL.println("Use these values:");
  PC_SERIAL.print("  MOVEIT_DIR_SIGN["); PC_SERIAL.print(idx); PC_SERIAL.print("] = ");
  PC_SERIAL.println(suggestedSign, 1);
  PC_SERIAL.print("  MOVEIT_HOME_RAD["); PC_SERIAL.print(idx); PC_SERIAL.print("] = ");
  PC_SERIAL.println(suggestedHome, 6);

  setAngle(servoID, startLogical, SWEEP_SPEED_RAW);
  delay(1600);
  PC_SERIAL.println("Returned to start.");
}

// --- TRIGGERED ROUTINES ---

// Run a simple range-of-motion sweep for individual axes (manual demo)
void individualSweep() {
  PC_SERIAL.println("\n--- RUNNING INDIVIDUAL ROM SWEEP (777) ---");
  uint8_t sweepOrder[] = {6, 5, 4, 3, 1};
  for (uint8_t id : sweepOrder) {
    float low = 180.0, high = 180.0;
    for (int j = 0; j < 6; j++) {
      if (motors[j].id == id) {
        low = motors[j].lowerLimit;
        high = motors[j].upperLimit;
      }
    }

    PC_SERIAL.print("Sweeping ID "); PC_SERIAL.println(id);
    if (id == 4) { setAngle(4, 69.0, SWEEP_SPEED_RAW); delay(2000); }

    setAngle(id, low, SWEEP_SPEED_RAW); delay(3000);
    setAngle(id, high, SWEEP_SPEED_RAW); delay(4500);
    setAngle(id, 180.0, SWEEP_SPEED_RAW); delay(3000);
  }
  PC_SERIAL.println("--- INDIVIDUAL SWEEP COMPLETE ---");
}

// Show full range-of-motion across most axes in parallel as a demo routine
void showcaseFullROM() {
  PC_SERIAL.println("\n--- SHOWCASING FULL 6-DOF ROM (888) ---");
  PC_SERIAL.println("Step 1: Tucking for clearance...");
  setAngle(5, 100, SWEEP_SPEED_RAW);
  delay(1500);

  PC_SERIAL.println("Step 2: Moving to Lower Limits...");
  for (int i = 0; i < 6; i++) {
    if (motors[i].id != 2) setAngle(motors[i].id, motors[i].lowerLimit, SWEEP_SPEED_RAW);
  }
  delay(5000);

  PC_SERIAL.println("Step 3: Moving to Upper Limits...");
  for (int i = 0; i < 6; i++) {
    if (motors[i].id != 2) setAngle(motors[i].id, motors[i].upperLimit, SWEEP_SPEED_RAW);
  }
  delay(5000);

  PC_SERIAL.println("Step 4: Returning Home...");
  for (int i = 0; i < 6; i++) {
    if (motors[i].id != 2) setAngle(motors[i].id, 180.0, SWEEP_SPEED_RAW);
  }
  delay(4000);
  PC_SERIAL.println("--- SHOWCASE COMPLETE ---");
}

void setup() {
  PC_SERIAL.begin(1000000);
  while (!PC_SERIAL);
  d2a.begin(1000000);
  d2a.setPortProtocolVersion(1.0);

  PC_SERIAL.println("--- SYSTEM INITIALIZED ---");
  PC_SERIAL.println("Homing all motors to 180...");
  for (int i = 0; i < 6; i++) setAngle(motors[i].id, 180.0, 40);

  PC_SERIAL.println("\nREADY. CODES:");
  PC_SERIAL.println("555: Play exported MoveIt ACE trajectory");
  PC_SERIAL.println("777: Individual ROM Sweep");
  PC_SERIAL.println("888: Parallel ROM Showcase");
  PC_SERIAL.println("901: Print current logical/physical + MoveIt rad estimate");
  PC_SERIAL.println("902: Sign/home calibration test");
  PC_SERIAL.println("[ID]: Enter manual control mode");
}

void loop() {
  if (PC_SERIAL.available() > 0) {
    int inputCode = PC_SERIAL.parseInt();
    while (PC_SERIAL.available() > 0) PC_SERIAL.read();

    if (inputCode == 555) {
      playMoveitTrajectory();
    } else if (inputCode == 777) {
      individualSweep();
    } else if (inputCode == 888) {
      showcaseFullROM();
    } else if (inputCode == 901) {
      printPresentStateReport();
    } else if (inputCode == 902) {
      guidedMoveitSignTest();
    } else if (d2a.ping(inputCode)) {
      PC_SERIAL.print("Locked: ID "); PC_SERIAL.println(inputCode);
      while (true) {
        PC_SERIAL.print("ID "); PC_SERIAL.print(inputCode); PC_SERIAL.println(" Angle (999 to exit):");
        while (PC_SERIAL.available() == 0);
        float angle = PC_SERIAL.parseFloat();
        while (PC_SERIAL.available() > 0) PC_SERIAL.read();
        if (angle == 999) break;
        setAngle(inputCode, angle);
      }
    } else {
      PC_SERIAL.println("INVALID INPUT or Motor not found.");
    }
  }
}
