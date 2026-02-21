#include <Dynamixel2Arduino.h>

// --- REMOVED: #include "ace_trajectory_data.h" --- 
// We no longer read from a file. We listen to USB.

// Serial port connected to Dynamixel bus and host PC
#define DXL_SERIAL Serial1
#define PC_SERIAL Serial

using namespace ControlTableItem;

// Dynamixel driver instance
Dynamixel2Arduino d2a(DXL_SERIAL);

// --- GLOBAL CONFIGURATION (KEPT EXACTLY THE SAME) ---
const int GLOBAL_SPEED_RAW = 40;
const int SWEEP_SPEED_RAW = 60;
const int TRAJECTORY_SPEED_RAW = 40;
const int ACCEL_VAL_SMOOTH = 15;

// State for safely tucking/untucking motor 5 when motor 4 moves
float previousM5Angle = 180.0;
bool m5IsTucked = false;

// Per-motor calibration
struct RobotMotor {
  uint8_t id;            
  float physicalOffset;  
  float lowerLimit;      
  float upperLimit;      
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

// --- CALIBRATION (KEPT EXACTLY THE SAME) ---
const float MOVEIT_HOME_RAD[5] = {
  0.0f,       // Motor1_joint -> ID1
  0.0f,       // Motor2_L     -> ID2
  0.0f,       // Motor4_elb   -> ID4
  0.0f,       // Motor5_wr    -> ID5
  0.0f        // Joint_EE     -> ID6
};
const float MOVEIT_DIR_SIGN[5] = {-1.0f, 1.0f, 1.0f, -1.0f, 1.0f};


// --- UTILITY FUNCTIONS (KEPT EXACTLY THE SAME) ---
RobotMotor* findMotor(uint8_t id) {
  for (int i = 0; i < 6; i++) {
    if (motors[i].id == id) return &motors[i];
  }
  return nullptr;
}

float rawToPhysicalDegrees(uint16_t model, int32_t raw) {
  if (model == 12) return ((float)raw) * (300.0f / 1023.0f);
  return ((float)raw) * (360.0f / 4095.0f);
}

void applySmoothing(uint8_t id, uint16_t model) {
  if (model == 12) {
    d2a.writeControlTableItem(CW_COMPLIANCE_SLOPE, id, 32);
    d2a.writeControlTableItem(CCW_COMPLIANCE_SLOPE, id, 32);
    d2a.writeControlTableItem(CW_COMPLIANCE_MARGIN, id, 2);
    d2a.writeControlTableItem(CCW_COMPLIANCE_MARGIN, id, 2);
  } else {
    d2a.writeControlTableItem(PROFILE_ACCELERATION, id, ACCEL_VAL_SMOOTH);
  }
}

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

float logicalToPhysical(uint8_t id, float logicalAngle) {
  RobotMotor* m = findMotor(id);
  if (!m) return logicalAngle;
  float safeLogical = constrain(logicalAngle, m->lowerLimit, m->upperLimit);
  return safeLogical + (m->physicalOffset - 180.0);
}

float physicalToLogical(uint8_t id, float physicalAngle) {
  RobotMotor* m = findMotor(id);
  if (!m) return physicalAngle;
  return physicalAngle - (m->physicalOffset - 180.0);
}

bool readPresentLogical(uint8_t id, float &logicalOut, float &physicalOut) {
  // Optimization: Ping takes time. If we trust the connection, we can skip it for speed.
  // if (!d2a.ping(id)) return false; 
  int32_t raw = d2a.readControlTableItem(PRESENT_POSITION, id);
  // Check for error value (0 usually valid, but check datasheet if needed)
  uint16_t model = d2a.getModelNumber(id);
  physicalOut = rawToPhysicalDegrees(model, raw);
  logicalOut = physicalToLogical(id, physicalOut);
  return true;
}

// --- CORE LOGIC (KEPT EXACTLY THE SAME) ---
// Handles Tucking, Limits, and Mirroring
void setAngle(uint8_t id, float logicalAngle, int speed = GLOBAL_SPEED_RAW) {
  if (id == 5 && !m5IsTucked) previousM5Angle = logicalAngle;

  if (id == 4) {
    if (logicalAngle < 70.0 && !m5IsTucked) {
      // PC_SERIAL.println("!!! M4 DANGER: Tucking M5."); 
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

  if (id == 3 || id == 2) {
    uint8_t partnerID = (id == 3) ? 2 : 3;
    executeMove(partnerID, logicalToPhysical(partnerID, 360.0 - logicalAngle), speed);
  }
}

float moveitRadToLogical(uint8_t idx, float rad) {
  const float RAD2DEG = 57.2957795f;
  return 180.0f + MOVEIT_DIR_SIGN[idx] * (rad - MOVEIT_HOME_RAD[idx]) * RAD2DEG;
}

// ==========================================
// NEW: LIVE COMMAND PROCESSING
// ==========================================

// Parse "1.57,0.2,-0.5..." string -> MoveIt Rads -> Logical Degs -> setAngle
void processLiveCommand(String input) {
  float targets_logical[5];
  float targets_rads[5];
  int lastIndex = 0;
  bool valid = true;

  // 1. Parse Input String (CSV of Radians)
  for (int i = 0; i < 5; i++) {
    int nextComma = input.indexOf(',', lastIndex);
    if (nextComma == -1 && i < 4) { valid = false; break; }
    
    String val = (i == 4) ? input.substring(lastIndex) : input.substring(lastIndex, nextComma);
    targets_rads[i] = val.toFloat();
    lastIndex = nextComma + 1;
  }

  if (!valid) return; // Ignore bad data

  // 2. Convert Rads -> Logical Degrees (Using YOUR calibration)
  targets_logical[0] = moveitRadToLogical(0, targets_rads[0]); // ID1
  targets_logical[1] = moveitRadToLogical(1, targets_rads[1]); // ID2 (ID3 mirrors)
  targets_logical[2] = moveitRadToLogical(2, targets_rads[2]); // ID4
  targets_logical[3] = moveitRadToLogical(3, targets_rads[3]); // ID5
  targets_logical[4] = moveitRadToLogical(4, targets_rads[4]); // ID6

  // 3. Move Servos (Using YOUR setAngle logic for safety/mirroring)
  // Mapping: Index 0->ID1, 1->ID2, 2->ID4, 3->ID5, 4->ID6
  setAngle(1, targets_logical[0], TRAJECTORY_SPEED_RAW);
  setAngle(2, targets_logical[1], TRAJECTORY_SPEED_RAW);
  setAngle(4, targets_logical[2], TRAJECTORY_SPEED_RAW);
  setAngle(5, targets_logical[3], TRAJECTORY_SPEED_RAW);
  setAngle(6, targets_logical[4], TRAJECTORY_SPEED_RAW);

  // 4. Smart Wait (Block until arrived)
  smartWait(targets_logical);
  
  // 5. Signal Completion
  PC_SERIAL.println("DONE");
}

void smartWait(float* targets_logical) {
    unsigned long lastProgressTime = millis();
    float lastPositions[5] = {0,0,0,0,0}; 
    uint8_t ids[5] = {1, 2, 4, 5, 6};

    while (true) {
      bool allArrived = true;
      bool isMoving = false;
      
      for(int k=0; k<5; k++) {
        float currentLog, currentPhys;
        readPresentLogical(ids[k], currentLog, currentPhys);
        
        // Check if close to target (Tolerance 4.0 deg)
        if (abs(currentLog - targets_logical[k]) > 4.0) { 
          allArrived = false; 
        }

        // Check if moving (Tolerance 0.5 deg)
        if (abs(currentLog - lastPositions[k]) > 0.5) {
          isMoving = true;
        }
        lastPositions[k] = currentLog;
      }

      if (allArrived) break;
      if (isMoving) lastProgressTime = millis();
      
      // Timeout: If not moved for 2 seconds, give up and say DONE
      if (millis() - lastProgressTime > 2000) {
        // PC_SERIAL.println("STALLED"); // Optional debug
        break;
      }
      delay(50); // Check every 50ms
    }
}

// ==========================================
// SETUP & LOOP
// ==========================================

void setup() {
  PC_SERIAL.begin(115200); // Standard Baud for USB
  // while (!PC_SERIAL); // Removed blocking wait so it runs on power-up
  
  d2a.begin(1000000);
  d2a.setPortProtocolVersion(1.0);

  // Initial Home
  for (int i = 0; i < 6; i++) setAngle(motors[i].id, 180.0, 40);
  
  PC_SERIAL.println("READY_FOR_LIVE_COMMANDS");
}

void loop() {
  // Listen for: "1.57,0.5,-1.2,0.0,0.0\n"
  if (PC_SERIAL.available() > 0) {
    String input = PC_SERIAL.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      processLiveCommand(input);
    }
  }
}