#include <Dynamixel2Arduino.h>
#include <Servo.h>
#include "trajectory_data.h"

#define DXL_SERIAL Serial1
#define PC_SERIAL  Serial

using namespace ControlTableItem;

// Forward declaration — required so the Arduino IDE auto-generated prototype
// for findMotor() compiles before struct RobotMotor is fully defined below.
struct RobotMotor;

Dynamixel2Arduino d2a(DXL_SERIAL);

// PWM hobby servo on pin 3 — controlled as ID 7
#define PEN_SERVO_PIN      3
#define PEN_SERVO_HOME_DEG 70   // Open position
#define PEN_SERVO_GRAB_DEG 90   // Closed / writing position
Servo penServo;
int g_penServoDeg = PEN_SERVO_HOME_DEG;  // tracks current pen servo position

// =============================================================================
// CONFIGURATION — all tunable values in one place
// =============================================================================

// --- Speeds (Dynamixel MOVING_SPEED units, roughly 0–300) -------------------
// AX-12A:  1 unit ≈ 0.111 RPM    MX-64A:  1 unit ≈ 0.114 RPM
const int HOME_SPEED              = 15;  // Homing and code-0 moves
const int MANUAL_SPEED            = 15;  // Joint control menu (code 100)
const int TRAJECTORY_SPEED        = 10;  // Streaming keyframes (555 mode)
                                         //   Lower = slower = smoother blending
const int TRAJECTORY_START_SPEED  = HOME_SPEED + 5;       // Move to first keyframe — gentle enough to avoid lurch
const int SERVO4_TRAJECTORY_SPEED = 15;  // Joint 4 override — needs extra torque
                                          // against gravity when arm is extended
const int SERVO23_TRAJECTORY_SPEED = 20; // Joints 2/3 override — MX-64 shoulder pair,
                                          // heaviest loaded joints; needs higher speed
                                          // to avoid stalling under arm weight
// --- Trajectory timing (ms) -------------------------------------------------
const unsigned long MIN_STREAM_INTERVAL_MS = 20; // 555 mode: minimum inter-waypoint delay (ms).
                                                  //   We preserve MoveIt's planned dt_ms whenever
                                                  //   possible and only clamp extremely small gaps
                                                  //   to keep serial update cadence reliable.
const unsigned long SETTLE_MS            = 60;   // Extra wait after MOVING clears
const unsigned long MOTION_TIMEOUT_MS    = 1200; // Hard timeout for waitForMotionComplete
const unsigned long CORNER_TIMEOUT_MS    = 3000; // Longer timeout for sharp corner settles.
const unsigned long CONTACT_PAUSE_MS     = 300;  // Dwell at each pen-lift / pen-plant boundary
                                                  //   (pause==1 waypoints: retract and touch-down).
const unsigned long PAUSE_POINT_MS       = 500; // Dwell (ms) when the arm reaches a user-defined
                                                  //   pause=3 waypoint in the trajectory header.
                                                  //   Set to any duration you need; exported via
                                                  //   the PAUSE POINT tool in letterCoordinateMaker.
const bool ENABLE_CORNER_FULL_STOPS      = true;  // pause=2 handling. true = full-stop at corners.
const bool ENABLE_USER_PAUSE_POINTS       = true; // pause=3 handling. false = ignore pause=3 tags.
const bool ENABLE_BIG_DELTA_CORNER_STOPS = true;  // Also stop for large direction changes without pause tags.
const float BIG_DELTA_CORNER_RAD         = 0.10f; // ~5.7 deg threshold (joint-space max delta).
const int MX_TRAJECTORY_GOAL_ACCEL       = 20;   // MX-64 Goal Acceleration (addr 73, Proto 1.0).
                                                  //   Limits jerk at direction changes during
                                                  //   trajectory playback.

// --- AX-12A compliance per servo -------------------------------------------
// Slope: torque ramp near target. Higher = softer spring = less overshoot.
//   Valid values: 2, 4, 8, 16, 32, 64, 128
// Margin: dead-zone (raw encoder units) where zero torque is applied.
//   0 = fight every micro-error (buzzing)   1-3 = typical   5+ = ignore noise
// MX-64 (IDs 2, 3) are PID-controlled — slope/margin do not apply to them.
struct ServoCompliance { uint8_t id; int slope; int margin; };
const ServoCompliance SERVO_COMPLIANCE[] = {
  // id  slope  margin
  {  1,    32,     2  },  // AX-12A — base rotation
  {  4,    32,     2  },  // AX-12A — elbow, wider margin damps gravity hunt
  {  5,     32,     3  },  // AX-12A — wrist pitch
  {  6,     32,     3  },  // AX-12A — wrist roll
};

// --- Ready position (logical degrees, before each trajectory run) -----------
// Recorded via limp-mode read.  The arm moves here before executing 555
// so it starts from a known, safe intermediate pose.
const float READY_J1 = 180.0f;
const float READY_J2 = 215.0f;  // ID 3 is mirrored automatically to 360 - READY_J2
const float READY_J4 =  60.0f;
const float READY_J5 = 180.0f;
const float READY_J6 = 225.0f;

// --- Per-motor calibration --------------------------------------------------
// physicalOffset: value added to shift logical 180° to the servo's physical zero.
// lowerLimit / upperLimit: allowed logical angle range (degrees).
// modelNumber: filled automatically on first use (0 = not yet read).
struct RobotMotor {
  uint8_t  id;
  float    physicalOffset;
  float    lowerLimit;
  float    upperLimit;
  uint16_t modelNumber;
};

RobotMotor motors[] = {
  {1, 150.0, 120.0, 220.0, 0},
  {2, 170.0, 120.0, 240.0, 0},  // MX-64A — Protocol 1.0, model 310
  {3, 187.0, 120.0, 240.0, 0},  // MX-64A — Protocol 1.0, model 310
  {4, 150.0,  40.0, 240.0, 0},
  {5, 150.0, 100.0, 260.0, 0},
  {6, 150.0, 140.0, 300.0, 0}
};

// --- MoveIt <-> servo calibration -------------------------------------------
// MOVEIT_HOME_RAD: MoveIt joint angle (rad) that maps to logical 180° on hardware.
// MOVEIT_DIR_SIGN: +1 if MoveIt and servo angles increase in the same direction.
// Order matches MOVEIT_TO_SERVO_ID: {J1, J2/3, J4, J5, J6}
const uint8_t MOVEIT_TO_SERVO_ID[5] = {1, 2, 4, 5, 6};
const float   MOVEIT_HOME_RAD[5]    = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
const float   MOVEIT_DIR_SIGN[5]    = {-1.0f, 1.0f, 1.0f, -1.0f, 1.0f};

// =============================================================================

// Move the pen servo from its current position to targetDeg over durationMs.
// The servo is kept attached at all times so it holds position between calls.
void penServoSweep(int targetDeg, unsigned long durationMs) {
  penServo.attach(PEN_SERVO_PIN);
  int startDeg = g_penServoDeg;
  int delta = targetDeg - startDeg;
  if (delta != 0) {
    int steps = abs(delta);
    unsigned long stepMs = durationMs / steps;
    if (stepMs < 1) stepMs = 1;
    int dir = (delta > 0) ? 1 : -1;
    for (int i = 1; i <= steps; i++) {
      penServo.write(startDeg + dir * i);
      delay(stepMs);
    }
  }
  // Always write and hold the final position — servo stays attached so it
  // does not go limp between calls.
  penServo.write(targetDeg);
  g_penServoDeg = targetDeg;
  delay(200);  // settle: give the servo time to physically reach the target
}

// Close the pen gripper (grab/writing position).
void servoClose() {
  penServoSweep(PEN_SERVO_GRAB_DEG, 500);
  PC_SERIAL.println("Pen servo -> CLOSED");
}

// Open the pen gripper (home/release position).
void servoOpen() {
  penServoSweep(PEN_SERVO_HOME_DEG, 500);
  PC_SERIAL.println("Pen servo -> OPEN");
}

// =============================================================================
// UTILITIES
// =============================================================================

RobotMotor* findMotor(uint8_t id) {
  for (int i = 0; i < 6; i++) {
    if (motors[i].id == id) return &motors[i];
  }
  return nullptr;
}

float rawToPhysicalDegrees(uint16_t model, int32_t raw) {
  if (model == 12)  return ((float)raw) * (300.0f / 1023.0f);  // AX-12A:  10-bit / 300 deg
  return ((float)raw) * (360.0f / 4095.0f);                    // MX-64 (model 310): 12-bit / 360 deg
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
  if (!d2a.ping(id)) return false;
  uint16_t model = d2a.getModelNumber(id);
  int32_t raw = d2a.readControlTableItem(PRESENT_POSITION, id);
  physicalOut = rawToPhysicalDegrees(model, raw);
  logicalOut  = physicalToLogical(id, physicalOut);
  return true;
}

float moveitRadToLogical(uint8_t idx, float rad) {
  const float RAD2DEG = 57.2957795f;
  return 180.0f + MOVEIT_DIR_SIGN[idx] * (rad - MOVEIT_HOME_RAD[idx]) * RAD2DEG;
}

// =============================================================================
// DIAGNOSTICS
// =============================================================================

// Read and print key registers for every servo so we can see exactly what
// state the hardware is in BEFORE we try to move anything.
void scanServos() {
  PC_SERIAL.println("");
  PC_SERIAL.println("=== SERVO SCAN ===");
  uint8_t ids[] = {1, 2, 3, 4, 5, 6};
  for (uint8_t id : ids) {
    PC_SERIAL.print("  ID "); PC_SERIAL.print(id); PC_SERIAL.print(": ");
    if (!d2a.ping(id)) {
      PC_SERIAL.println("NO RESPONSE (not found on bus)");
      continue;
    }
    uint16_t model = d2a.getModelNumber(id);
    PC_SERIAL.print("model="); PC_SERIAL.print(model);
    if (model == 12) {
      int32_t movSpeed = d2a.readControlTableItem(MOVING_SPEED, id);
      int32_t cwSlope  = d2a.readControlTableItem(CW_COMPLIANCE_SLOPE,  id);
      int32_t ccwSlope = d2a.readControlTableItem(CCW_COMPLIANCE_SLOPE, id);
      int32_t cwMarg   = d2a.readControlTableItem(CW_COMPLIANCE_MARGIN,  id);
      int32_t ccwMarg  = d2a.readControlTableItem(CCW_COMPLIANCE_MARGIN, id);
      PC_SERIAL.print(" (AX-12)");
      PC_SERIAL.print(" | MOVING_SPEED=");        PC_SERIAL.print(movSpeed);
      PC_SERIAL.print(" | CW_SLOPE=");            PC_SERIAL.print(cwSlope);
      PC_SERIAL.print(" | CCW_SLOPE=");           PC_SERIAL.print(ccwSlope);
      PC_SERIAL.print(" | CW_MARGIN=");           PC_SERIAL.print(cwMarg);
      PC_SERIAL.print(" | CCW_MARGIN=");          PC_SERIAL.println(ccwMarg);
    } else if (model == 310) {
      int32_t movSpeed = d2a.readControlTableItem(MOVING_SPEED,    id);
      int32_t pGain    = d2a.readControlTableItem(POSITION_P_GAIN, id);
      int32_t iGain    = d2a.readControlTableItem(POSITION_I_GAIN, id);
      int32_t dGain    = d2a.readControlTableItem(POSITION_D_GAIN, id);
      PC_SERIAL.print(" (MX-64A Proto1)");
      PC_SERIAL.print(" | MOVING_SPEED="); PC_SERIAL.print(movSpeed);
      PC_SERIAL.print(" | P_GAIN=");       PC_SERIAL.print(pGain);
      PC_SERIAL.print(" | I_GAIN=");       PC_SERIAL.print(iGain);
      PC_SERIAL.print(" | D_GAIN=");       PC_SERIAL.println(dGain);
    } else {
      PC_SERIAL.print(" (unknown model — expected AX-12A or MX-64)");
      PC_SERIAL.println();
    }
  }
  PC_SERIAL.println("=== END SCAN ===");
  PC_SERIAL.println("");
}

// =============================================================================
// MOTION
// =============================================================================

// Apply the global SERVO_COMPLIANCE table to one servo.
// AX-12A (model 12): writes CW/CCW slope and margin registers.
// MX-64  (model 310): PID-controlled — compliance registers don't exist, skip.
void applySmoothing(uint8_t id, uint16_t model) {
  if (model != 12) return;  // MX-64: nothing to do
  // Look up this servo in the table; use last entry as fallback.
  int slope  = SERVO_COMPLIANCE[0].slope;
  int margin = SERVO_COMPLIANCE[0].margin;
  for (const ServoCompliance &c : SERVO_COMPLIANCE) {
    if (c.id == id) { slope = c.slope; margin = c.margin; break; }
  }
  d2a.writeControlTableItem(CW_COMPLIANCE_SLOPE,   id, slope);
  d2a.writeControlTableItem(CCW_COMPLIANCE_SLOPE,  id, slope);
  d2a.writeControlTableItem(CW_COMPLIANCE_MARGIN,  id, margin);
  d2a.writeControlTableItem(CCW_COMPLIANCE_MARGIN, id, margin);
  PC_SERIAL.print("  [compliance] ID="); PC_SERIAL.print(id);
  PC_SERIAL.print(" slope="); PC_SERIAL.print(slope);
  PC_SERIAL.print(" margin="); PC_SERIAL.println(margin);
}

// Convert physical degrees to the servo's raw bit value.
uint16_t physicalToRaw(uint16_t model, float physicalDegrees) {
  return (model == 12)
    ? (uint16_t)(constrain(physicalDegrees, 0, 300) * (1023.0 / 300.0))  // AX-12A
    : (uint16_t)(constrain(physicalDegrees, 0, 360) * (4095.0 / 360.0)); // MX-64
}

// Low-level: write speed + goal position to a servo in physical degrees.
// Used for homing and manual mode where per-call overhead is fine.
void executeMove(uint8_t id, float physicalDegrees, int speed) {
  RobotMotor* m = findMotor(id);
  if (m && m->modelNumber == 0) m->modelNumber = d2a.getModelNumber(id);
  uint16_t model = m ? m->modelNumber : d2a.getModelNumber(id);

  static bool smoothed[11] = {false};
  if (id < 11 && !smoothed[id]) {
    PC_SERIAL.print("  [executeMove] First call for ID="); PC_SERIAL.print(id);
    PC_SERIAL.println(" -> applySmoothing");
    applySmoothing(id, model);
    smoothed[id] = true;
  }

  // AX-12A and MX-64 (Protocol 1.0) both use MOVING_SPEED.
  d2a.writeControlTableItem(MOVING_SPEED, id, speed);
  int32_t readBack = d2a.readControlTableItem(MOVING_SPEED, id);
  PC_SERIAL.print("  [executeMove] ID="); PC_SERIAL.print(id);
  PC_SERIAL.print(model == 12 ? " AX-12A" : " MX-64");
  PC_SERIAL.print(" | set MOVING_SPEED="); PC_SERIAL.print(speed);
  PC_SERIAL.print(" readback="); PC_SERIAL.print(readBack);
  PC_SERIAL.print(" | goal_phys="); PC_SERIAL.println(physicalDegrees, 1);
  d2a.torqueOn(id);
  d2a.setGoalPosition(id, physicalToRaw(model, physicalDegrees));
}

// Prepare all servos for trajectory playback: apply smoothing, set speed, and
// enable torque ONCE. During playback only bare goal-position writes are sent,
// so all joints get their commands as close together as possible.
void prepareServosForTrajectory() {
  uint8_t allIds[] = {1, 2, 3, 4, 5, 6};
  for (uint8_t id : allIds) {
    RobotMotor* m = findMotor(id);
    if (m && m->modelNumber == 0) m->modelNumber = d2a.getModelNumber(id);
    uint16_t model = m ? m->modelNumber : d2a.getModelNumber(id);
    applySmoothing(id, model);
    if (model == 310) {
      d2a.writeControlTableItem(GOAL_ACCELERATION, id, MX_TRAJECTORY_GOAL_ACCEL);
    }
    int spd = (id == 4) ? SERVO4_TRAJECTORY_SPEED
            : (id == 2 || id == 3) ? SERVO23_TRAJECTORY_SPEED
            : TRAJECTORY_SPEED;
    d2a.writeControlTableItem(MOVING_SPEED, id, spd);  // AX-12A & MX-64 both use MOVING_SPEED
    d2a.torqueOn(id);
  }
}

// =============================================================================
// MOTION COMPLETION HELPERS
// =============================================================================

// Returns true if ANY of the 6 servos still has its MOVING flag set.
bool isAnyServoMoving() {
  uint8_t ids[] = {1, 2, 3, 4, 5, 6};
  for (uint8_t id : ids) {
    if (d2a.readControlTableItem(MOVING, id)) return true;
  }
  return false;
}

// Block until all servos have stopped (MOVING flag clears), then wait SETTLE_MS.
// Exits early after timeoutMs if the flag never clears.
void waitForMotionComplete(unsigned long timeoutMs = MOTION_TIMEOUT_MS) {
  // Short startup window so MOVING has time to assert after a goal write.
  delay(40);

  unsigned long hardLimit = millis() + timeoutMs;
  while (millis() < hardLimit) {
    if (!isAnyServoMoving()) break;
    delay(10);
  }
  delay(SETTLE_MS);
}

// =============================================================================

// Lightweight goal-position-only write. No speed or torque overhead.
// Requires prepareServosForTrajectory() to have been called first.
void sendGoalPosition(uint8_t id, float physicalDegrees) {
  RobotMotor* m = findMotor(id);
  uint16_t model = m ? m->modelNumber : 12;
  d2a.setGoalPosition(id, physicalToRaw(model, physicalDegrees));
}

// Send goal positions to all 6 servos so they start moving simultaneously.
//
// Protocol 1.0 synchronous motion — REG_WRITE + ACTION:
//   1. regWrite(id, ...) queues the goal position on each servo without moving.
//   2. action(0xFE)  broadcasts a single packet that fires all queued moves
//      at the same instant.  No motor starts until every motor has its target.
//
// This replaces sequential setGoalPosition() calls which caused visible
// stagger (motor 1 starts, then 2, then 3...) because each write instantly
// executed on that servo.
//
// Goal Position address for both AX-12A and MX-64 (Protocol 1.0): 30, 2 bytes.
// physicals[] must be ordered: {ID1, ID2, ID3, ID4, ID5, ID6}.
void syncAllGoalPositions(float physicals[6]) {
  static const uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
  static const uint16_t GOAL_POS_ADDR = 30;  // same address on AX-12A and MX-64 (Proto 1)

  // Queue each servo's goal without triggering motion.
  for (int i = 0; i < 6; i++) {
    RobotMotor* m = findMotor(ids[i]);
    uint16_t model = m ? m->modelNumber : 12;
    uint16_t raw = physicalToRaw(model, physicals[i]);
    uint8_t data[2] = { (uint8_t)(raw & 0xFF), (uint8_t)(raw >> 8) };
    d2a.regWrite(ids[i], GOAL_POS_ADDR, data, 2);
  }

  // Fire all queued positions simultaneously.
  d2a.action(0xFE);
}

// High-level: move a motor to a logical angle (deg).
// Handles ID 2/3 mirroring automatically.
void setAngle(uint8_t id, float logicalAngle, int speed = MANUAL_SPEED) {
  executeMove(id, logicalToPhysical(id, logicalAngle), speed);

  // Mirror IDs 2 <-> 3 (physical mirrored joint pair)
  if (id == 2 || id == 3) {
    uint8_t partnerID = (id == 3) ? 2 : 3;
    executeMove(partnerID, logicalToPhysical(partnerID, 360.0 - logicalAngle), speed);
  }
}

// =============================================================================
// SEQUENCES
// =============================================================================

// 100: Interactive joint control menu — select a joint and move it continuously.
void printMainMenu() {
  PC_SERIAL.println("");
  PC_SERIAL.println("--- MAIN MENU ---");
  PC_SERIAL.println("  0   : Home all motors");
  PC_SERIAL.println("  100 : Joint control menu (1,2,4,5,6,7)");
  PC_SERIAL.println("  200 : Limp/read mode (torque off, ENTER reads positions, 999 exits)");
  PC_SERIAL.println("  555 : Play trajectory (fixed speed)");
  PC_SERIAL.println("Enter code:");
}

// 200: All motors go limp (torque off). Press ENTER to read all servo positions.
// Type 999 and press ENTER to re-enable torque and return to the main menu.
void limpReadMode() {
  PC_SERIAL.println("\n--- LIMP / READ MODE ---");
  PC_SERIAL.println("All motors are now LIMP (torque off). Move the arm freely.");
  PC_SERIAL.println("Press ENTER to read current positions. Type 999 + ENTER to exit.");

  // Release torque on all Dynamixel joints.
  uint8_t allIds[] = {1, 2, 3, 4, 5, 6};
  for (uint8_t id : allIds) d2a.torqueOff(id);

  while (true) {
    // Block until the user sends something (even a bare newline).
    while (PC_SERIAL.available() == 0);

    // Peek at the input: if it is "999" exit, otherwise just read positions.
    String raw = PC_SERIAL.readStringUntil('\n');
    raw.trim();

    if (raw == "999") {
      PC_SERIAL.println("Exiting limp mode — re-enabling torque...");
      for (uint8_t id : allIds) d2a.torqueOn(id);
      PC_SERIAL.println("Torque ON. Returning to main menu.");
      return;
    }

    // Read and print logical (degrees) and physical (degrees) for every joint.
    PC_SERIAL.println("--- CURRENT POSITIONS ---");
    for (uint8_t id : allIds) {
      float logDeg = 0.0f, physDeg = 0.0f;
      bool ok = readPresentLogical(id, logDeg, physDeg);
      PC_SERIAL.print("  ID "); PC_SERIAL.print(id);
      if (ok) {
        PC_SERIAL.print("  logical=");     PC_SERIAL.print(logDeg,  2);
        PC_SERIAL.print(" deg  physical="); PC_SERIAL.print(physDeg, 2);
        PC_SERIAL.println(" deg");
      } else {
        PC_SERIAL.println("  (no response)");
      }
    }
    PC_SERIAL.println("--- Press ENTER for another read, or 999+ENTER to exit ---");
  }
}

// 999 from inside a joint returns to joint selection; 999 from joint selection
// returns to the main menu.
void jointControlMenu() {
  PC_SERIAL.println("\n--- JOINT CONTROL MENU ---");

  while (true) {
    PC_SERIAL.println("Select joint [ 1 | 2 | 4 | 5 | 6 | 7 ] or 999 to return:");
    while (PC_SERIAL.available() == 0);
    int jointId = PC_SERIAL.parseInt();
    while (PC_SERIAL.available() > 0) PC_SERIAL.read();
    PC_SERIAL.println();

    if (jointId == 999) {
      PC_SERIAL.println("Returning to home menu.");
      return;
    }

    bool validJoint = (jointId == 1 || jointId == 2 || jointId == 4 ||
                       jointId == 5 || jointId == 6 || jointId == 7);
    if (!validJoint) {
      PC_SERIAL.println("Invalid joint ID. Choose from: 1, 2, 4, 5, 6, 7");
      continue;
    }

    PC_SERIAL.print("  Locked: Joint "); PC_SERIAL.println(jointId);

    if (jointId == 7) {
      // PWM hobby servo on pin 3 (pen servo)
      PC_SERIAL.println("  Enter angle for pen servo (0-180 deg, 999 to back):");
      while (PC_SERIAL.available() == 0);
      float penAngle = PC_SERIAL.parseFloat();
      while (PC_SERIAL.available() > 0) PC_SERIAL.read();
      if ((int)penAngle != 999) {
        penAngle = constrain(penAngle, 0, 180);
        penServoSweep((int)penAngle, 500);
        PC_SERIAL.print("  Pen servo moved to "); PC_SERIAL.println(penAngle, 0);
      }
    } else {
      // Dynamixel servo
      float logAngle = 180.0f, physAngle = 0.0f;
      readPresentLogical(jointId, logAngle, physAngle);
      float lastCmdAngle = logAngle;

      RobotMotor* m = findMotor(jointId);

      while (true) {
        float curLog = 0.0f, curPhys = 0.0f;
        readPresentLogical(jointId, curLog, curPhys);

        PC_SERIAL.println("");
        PC_SERIAL.print("  Joint "); PC_SERIAL.print(jointId);
        PC_SERIAL.print(" | Last cmd: "); PC_SERIAL.print(lastCmdAngle, 1);
        PC_SERIAL.print(" deg | Current: "); PC_SERIAL.print(curLog, 1); PC_SERIAL.println(" deg");
        if (m) {
          PC_SERIAL.print("  Limits: [");
          PC_SERIAL.print(m->lowerLimit, 0); PC_SERIAL.print(", ");
          PC_SERIAL.print(m->upperLimit, 0); PC_SERIAL.println("]");
        }
        PC_SERIAL.println("  Enter angle (999 to back):");

        while (PC_SERIAL.available() == 0);
        float angle = PC_SERIAL.parseFloat();
        while (PC_SERIAL.available() > 0) PC_SERIAL.read();
        PC_SERIAL.println();
        if ((int)angle == 999) break;

        // Clamp to limits before commanding
        if (m) angle = constrain(angle, m->lowerLimit, m->upperLimit);
        setAngle(jointId, angle, MANUAL_SPEED);
        lastCmdAngle = angle;
      }
    }

    PC_SERIAL.println("  Returning to joint selection...");
  }
}

// Pre-run ready position — recorded from limp-mode read.
// The arm moves here before homing or before any trajectory run so it
// starts from a known, safe intermediate pose rather than wherever it
// last stopped.
void goToReadyPosition() {
  PC_SERIAL.println("Moving to ready position...");
  setAngle(1, READY_J1, HOME_SPEED);
  setAngle(2, READY_J2, HOME_SPEED);  // also mirrors ID 3 to 360-READY_J2
  setAngle(4, READY_J4, HOME_SPEED);
  setAngle(5, READY_J5, HOME_SPEED);
  setAngle(6, READY_J6, HOME_SPEED);
  waitForMotionComplete();
  PC_SERIAL.println("Ready position reached.");
}

// 101: Home all motors to logical 180 deg.
// If every joint is already within HOME_ALREADY_DEG of 180, skip the move entirely.
// Otherwise, go to the READY position first so the arm never swings through an
// NOTE: ID 3 is NOT commanded directly — setAngle(2,...) mirrors it automatically.
const float HOME_ALREADY_DEG = 5.0f;  // tolerance to consider "already homed"

void homeAllMotors() {
  uint8_t homeIds[] = {1, 2, 4, 5, 6};  // ID 3 is mirrored by setAngle(2)

  // Check whether all joints are already near home.
  bool alreadyHomed = true;
  for (uint8_t id : homeIds) {
    float logDeg = 0.0f, physDeg = 0.0f;
    if (!readPresentLogical(id, logDeg, physDeg)) {
      alreadyHomed = false;  // can't confirm — treat as not homed
      break;
    }
    if (abs(logDeg - 180.0f) > HOME_ALREADY_DEG) {
      alreadyHomed = false;
      break;
    }
  }

  if (alreadyHomed) {
    PC_SERIAL.println("Already at home position — skipping home move.");
    return;
  }

  PC_SERIAL.println("Homing all motors to 180 (speed=" + String(HOME_SPEED) + ")...");
  for (uint8_t id : homeIds) {
    PC_SERIAL.print("  Homing motor ID="); PC_SERIAL.println(id);
    setAngle(id, 180.0, HOME_SPEED);
  }
  waitForMotionComplete();
  PC_SERIAL.println("Homed.");
}

// 555: Play the exported MoveIt trajectory from trajectory_data.h

// Move all joints to the first keyframe of a trajectory at HOME_SPEED and
// wait for full arrival before streaming begins.  Without this, the arm makes
// a slow, uncoordinated crawl from wherever it stopped because the streaming
// loop fires keyframes at TRAJECTORY_SPEED while the arm is still far from
// the start position.
void goToTrajectoryStart(float physicals[6]) {
  PC_SERIAL.println("Moving to trajectory start position...");
  uint8_t allIds[] = {1, 2, 3, 4, 5, 6};
  for (uint8_t id : allIds) {
    RobotMotor* m = findMotor(id);
    if (m && m->modelNumber == 0) m->modelNumber = d2a.getModelNumber(id);
    d2a.writeControlTableItem(MOVING_SPEED, id, TRAJECTORY_START_SPEED);
    d2a.torqueOn(id);
  }
  syncAllGoalPositions(physicals);
  waitForMotionComplete();
  PC_SERIAL.println("At trajectory start.");
}

void playMoveitTrajectory() {
  PC_SERIAL.println("\n--- PLAYING TRAJECTORY (555) ---");
  if (kTrajectoryCount == 0) { PC_SERIAL.println("No trajectory points loaded."); return; }

  prepareServosForTrajectory();

  // Move to first keyframe at fast speed, wait for full arrival.
  {
    const JointSample &p0 = kTrajectory[0];
    float firstFrame[6] = {
      logicalToPhysical(1, moveitRadToLogical(0, p0.j1_rad)),
      logicalToPhysical(2, moveitRadToLogical(1, p0.j2l_rad)),
      logicalToPhysical(3, 360.0 - moveitRadToLogical(1, p0.j2l_rad)),
      logicalToPhysical(4, moveitRadToLogical(2, p0.j4_rad)),
      logicalToPhysical(5, moveitRadToLogical(3, p0.j5_rad)),
      logicalToPhysical(6, moveitRadToLogical(4, p0.j6_rad))
    };
    goToTrajectoryStart(firstFrame);
  }
  prepareServosForTrajectory();  // restore drawing speed

  // Marker has just touched the board — begin drawing immediately.
  PC_SERIAL.println("--- STARTING ---");

  bool previousWasPauseTag = false;  // collapse consecutive pause-tagged waypoints

  for (size_t i = 0; i < kTrajectoryCount; ++i) {
    const JointSample &p = kTrajectory[i];
    float physicals[6] = {
      logicalToPhysical(1, moveitRadToLogical(0, p.j1_rad)),
      logicalToPhysical(2, moveitRadToLogical(1, p.j2l_rad)),
      logicalToPhysical(3, 360.0 - moveitRadToLogical(1, p.j2l_rad)),
      logicalToPhysical(4, moveitRadToLogical(2, p.j4_rad)),
      logicalToPhysical(5, moveitRadToLogical(3, p.j5_rad)),
      logicalToPhysical(6, moveitRadToLogical(4, p.j6_rad))
    };

    const bool isPauseTag = (p.pause > 0);
    const bool executePauseHere = isPauseTag && !previousWasPauseTag;

    bool virtualCornerStop = false;
    if (ENABLE_BIG_DELTA_CORNER_STOPS && i > 0 && p.pause == 0) {
      const JointSample &prev = kTrajectory[i - 1];
      float maxDelta = fabsf(p.j1_rad  - prev.j1_rad);
      float d = fabsf(p.j2l_rad - prev.j2l_rad); if (d > maxDelta) maxDelta = d;
      d = fabsf(p.j4_rad  - prev.j4_rad);        if (d > maxDelta) maxDelta = d;
      d = fabsf(p.j5_rad  - prev.j5_rad);        if (d > maxDelta) maxDelta = d;
      d = fabsf(p.j6_rad  - prev.j6_rad);        if (d > maxDelta) maxDelta = d;
      virtualCornerStop = (maxDelta >= BIG_DELTA_CORNER_RAD);
    }

    if (p.pause == 1 && executePauseHere) {
      // Pen-lift / touch-down boundary: reach target, settle, then dwell.
      syncAllGoalPositions(physicals);
      waitForMotionComplete();
      PC_SERIAL.print("Pen boundary at keyframe "); PC_SERIAL.println(i);
      delay(CONTACT_PAUSE_MS);
    } else if ((p.pause == 2 && executePauseHere) || virtualCornerStop) {
      // Draw-to-draw corner. Full-stop mode is optional because it can be slow
      // if MOVING flags remain asserted under load.
      syncAllGoalPositions(physicals);
      if (ENABLE_CORNER_FULL_STOPS) {
        waitForMotionComplete(CORNER_TIMEOUT_MS);
        PC_SERIAL.print("Corner settle at keyframe "); PC_SERIAL.println(i);
      } else {
        unsigned long interval = (p.dt_ms > MIN_STREAM_INTERVAL_MS) ? p.dt_ms : MIN_STREAM_INTERVAL_MS;
        delay(interval);
      }
    } else if (p.pause == 3 && executePauseHere) {
      // User-defined pause point can be disabled globally for speed/debugging.
      syncAllGoalPositions(physicals);
      if (ENABLE_USER_PAUSE_POINTS) {
        waitForMotionComplete();
        PC_SERIAL.print("User pause at keyframe "); PC_SERIAL.println(i);
        delay(PAUSE_POINT_MS);
      } else {
        unsigned long interval = (p.dt_ms > MIN_STREAM_INTERVAL_MS) ? p.dt_ms : MIN_STREAM_INTERVAL_MS;
        delay(interval);
      }
    } else {
      // Normal waypoint OR collapsed back-to-back pause tag:
      // preserve MoveIt-planned dt_ms; only clamp tiny gaps.
      unsigned long interval = (p.dt_ms > MIN_STREAM_INTERVAL_MS) ? p.dt_ms : MIN_STREAM_INTERVAL_MS;
      unsigned long t0 = millis();
      syncAllGoalPositions(physicals);
      unsigned long elapsed = millis() - t0;
      if (interval > elapsed) delay(interval - elapsed);
    }

    previousWasPauseTag = isPauseTag;
  }

  // Wait for the last move to fully complete before the arm homes.
  waitForMotionComplete();
  PC_SERIAL.println("End of stroke — trajectory complete.");

  PC_SERIAL.println("--- DONE ---");
}

void setup() {
  PC_SERIAL.begin(1000000);
  // 3-second timeout — runs headless (no serial monitor) without blocking
  { unsigned long _t = millis(); while (!PC_SERIAL && (millis() - _t) < 3000); }
  d2a.begin(1000000);
  d2a.setPortProtocolVersion(1.0);

  PC_SERIAL.println("--- SYSTEM INITIALIZED ---");
  scanServos();
  homeAllMotors();
  scanServos();  // second scan: confirm registers after homing write

  // Assume servo is at GRAB (worst case) so the open sweep always runs.
  g_penServoDeg = PEN_SERVO_GRAB_DEG;
  servoOpen();  // sweeps from GRAB -> HOME, holds position
  printMainMenu();
  homeAllMotors();
}

void loop() {
  if (PC_SERIAL.available() > 0) {
    int inputCode = PC_SERIAL.parseInt();
    while (PC_SERIAL.available() > 0) PC_SERIAL.read();

    if (inputCode == 0) {
      homeAllMotors();
      servoOpen();
    } else if (inputCode == 100) {
      jointControlMenu();
    } else if (inputCode == 200) {
      limpReadMode();
    } else if (inputCode == 555) {
      servoClose();
      playMoveitTrajectory();
      homeAllMotors();
      servoOpen();
    } else {
      PC_SERIAL.println("Invalid input.");
    }
    printMainMenu();
  }
}
