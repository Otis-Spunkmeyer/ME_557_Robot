#include <Dynamixel2Arduino.h>
// #include <Servo.h>  // DISABLED — Servo.h timer conflicts with Dynamixel UART
#include "trajectory_data.h"
#include "moveItData.h"

#define DXL_SERIAL Serial1
#define PC_SERIAL  Serial

using namespace ControlTableItem;

// Forward declaration — required so the Arduino IDE auto-generated prototype
// for findMotor() compiles before struct RobotMotor is fully defined below.
struct RobotMotor;

Dynamixel2Arduino d2a(DXL_SERIAL);

// PWM hobby servo on pin 3 — controlled as ID 7
// #define PEN_SERVO_PIN      3
// #define PEN_SERVO_HOME_DEG 70
// #define PEN_SERVO_GRAB_DEG 90
// Servo penServo;
// int g_penServoDeg = PEN_SERVO_HOME_DEG;  // tracks current pen servo position

// Move the pen servo from its current position to targetDeg over durationMs.
// DISABLED — Servo.h timer conflicts with Dynamixel UART
// void penServoSweep(int targetDeg, unsigned long durationMs) {
//   penServo.attach(PEN_SERVO_PIN);
//   int startDeg = g_penServoDeg;
//   int delta = targetDeg - startDeg;
//   if (delta != 0) {
//     int steps = abs(delta);
//     unsigned long stepMs = durationMs / steps;
//     if (stepMs < 1) stepMs = 1;
//     int dir = (delta > 0) ? 1 : -1;
//     for (int i = 1; i <= steps; i++) {
//       penServo.write(startDeg + dir * i);
//       delay(stepMs);
//     }
//     g_penServoDeg = targetDeg;
//   }
//   penServo.detach();
// }

// =============================================================================
// CONFIGURATION — edit these values to tune robot behaviour
// =============================================================================
const int   HOME_SPEED         = 15;    // Speed when homing (seq 100 / startup)
const int   MANUAL_SPEED       = 15;    // Speed in manual ID control mode
const int   TRAJECTORY_SPEED   = 10;     // Speed during trajectory streaming — slow so moves take LONGER than the interval

// Trajectory playback
// During a drawing segment keyframes are STREAMED at this interval.  The servo
// receives the next goal position while still moving toward the previous one;
// its internal velocity profile blends smoothly through each waypoint.
// Tune so the interval roughly matches how long each inter-keyframe move takes
// at TRAJECTORY_SPEED — too short and servos lag; too long and they over-stop.
const unsigned long  KEYFRAME_INTERVAL_MS = 50;  // ms between streamed keyframes
// Extra settle time (ms) after all servos report stopped at a pause boundary.
const unsigned long  SETTLE_MS            = 50;
// Dwell time (ms) applied at draw<->travel boundary keyframes (pause flag).
const unsigned long  RETRACT_PAUSE_MS     = 500;
// Timeout (ms) for waitForMotionComplete — safety valve so a stalled servo
// does not hang the sketch forever.
const unsigned long  MOTION_TIMEOUT_MS    = 8000;

// Servo 1 (base) is AX-12A — compliance slope controls stiffness, not PID.
// Softer slope = gentler approach to target = less bounce/overshoot.
// Valid values: 2, 4, 8, 16, 32, 64, 128. Higher = softer.
const int SERVO1_AX12_SLOPE      = 96;    // softer than default (32) — reduces oscillation

// Per-servo compliance overrides for servo 4 (AX-12 with worst moment arm).
// High load at a disadvantaged moment arm causes the servo to hunt:
// it overshoots slightly, brakes, overshoots the other way, repeat.
//
// Slope controls the torque ramp around the target (higher = softer spring).
//   Valid values: 2, 4, 8, 16, 32, 64, 128
//   Higher slope = gentler approach = less snap-back jitter, but slightly
//   less positional stiffness.  Start at 128 and tighten down if needed.
//
// Margin is the dead-zone (raw units) where zero torque is applied.
//   0 = fight every tiny error (causes buzzing under load)
//   2-4 = ignore sub-degree noise (stops the hunting loop)
//   Don't go above ~5 or positional accuracy visibly degrades.
const int SERVO4_AX12_SLOPE          = 64;  // middle ground: fast enough, soft enough
const int SERVO4_AX12_MARGIN         = 3;   // wider dead-zone kills resonance-exciting micro-hunt
// Servo 4 gets its own trajectory speed, higher than the global setting.
// At speed 8 it lacks torque authority against gravity when the arm is
// extended — the joint lags, then snaps/bounces to catch up.
// A higher speed keeps torque output high enough to move decisively.
const int SERVO4_TRAJECTORY_SPEED    = 15;
// =============================================================================

// Per-motor calibration, mapping physical to logical angle ranges and offsets
struct RobotMotor {
  uint8_t  id;            // Dynamixel ID
  float    physicalOffset; // Physical zero offset used to align to logical 180deg
  float    lowerLimit;    // Logical lower limit (deg)
  float    upperLimit;    // Logical upper limit (deg)
  uint16_t modelNumber;   // Cached on first use — avoids a bus read every move
};

RobotMotor motors[] = {
  {1, 117.0, 120.0, 220.0, 0},
  {2, 187.0, 120.0, 240.0, 0},  // MX-64A — Protocol 1.0, model 310
  {3, 189.0, 120.0, 240.0, 0},  // MX-64A — Protocol 1.0, model 310
  {4, 150.0,  40.0, 240.0, 0},
  {5, 150.0, 100.0, 260.0, 0},
  {6, 150.0, 140.0, 300.0, 0}
};

const uint8_t MOVEIT_TO_SERVO_ID[5] = {1, 2, 4, 5, 6};

// MoveIt (radians) <-> logical (degrees) calibration.
// Tune once on hardware; keep consistent with the MoveIt export tool.
const float MOVEIT_HOME_RAD[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
const float MOVEIT_DIR_SIGN[5] = {-1.0f, 1.0f, 1.0f, -1.0f, 1.0f};

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
// ALARM RECOVERY
// =============================================================================

// Clear ALARM_SHUTDOWN on every servo by cycling torque.
// An AX-12A/MX-64 that tripped overload/thermal shuts its output but keeps
// responding to pings — it silently ignores every goal-position write until
// torque is cycled.  Call this on startup and before any trajectory run.
void clearAlarms() {
  uint8_t allIds[] = {1, 2, 3, 4, 5, 6};
  PC_SERIAL.println("Clearing alarms (torque cycle)...");
  for (uint8_t id : allIds) {
    if (!d2a.ping(id)) continue;
    d2a.torqueOff(id);
    delay(20);
    d2a.torqueOn(id);
  }
  PC_SERIAL.println("Alarms cleared.");
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

void applySmoothing(uint8_t id, uint16_t model) {
  PC_SERIAL.print("  [applySmoothing] ID="); PC_SERIAL.print(id);
  PC_SERIAL.print(" model="); PC_SERIAL.print(model);
  if (model == 12) {
    // AX-12: compliance slopes + margin
    int slope  = 32;
    int margin = 3;  // wider default dead-zone — reduces hunting on IDs 5 & 6 under end-effector load
    if (id == 1) { slope = SERVO1_AX12_SLOPE; }
    if (id == 4) { slope = SERVO4_AX12_SLOPE; margin = SERVO4_AX12_MARGIN; }
    d2a.writeControlTableItem(CW_COMPLIANCE_SLOPE,   id, slope);
    d2a.writeControlTableItem(CCW_COMPLIANCE_SLOPE,  id, slope);
    d2a.writeControlTableItem(CW_COMPLIANCE_MARGIN,  id, margin);
    d2a.writeControlTableItem(CCW_COMPLIANCE_MARGIN, id, margin);
    PC_SERIAL.print(" | AX-12 slope="); PC_SERIAL.print(slope);
    PC_SERIAL.print(" margin="); PC_SERIAL.println(margin);
  } else if (model == 310) {
    // MX-64 Protocol 1.0: MOVING_SPEED controls velocity.
    // PID gains are at different register addresses in Protocol 1.0 vs 2.0 —
    // leave them at hardware defaults to avoid mis-writing the wrong address.
    PC_SERIAL.println(" | MX-64 Proto1 OK (default PID)");
  } else {
    PC_SERIAL.println(" | (unknown model — no smoothing applied)");
  }
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
    int spd = (id == 4) ? SERVO4_TRAJECTORY_SPEED : TRAJECTORY_SPEED;
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

// Block until all servos have stopped (or timeout expires), then wait an
// additional SETTLE_MS to let mechanical oscillation die out.
void waitForMotionComplete() {
  // Give servos a minimum startup window so the MOVING flag has time to assert.
  delay(KEYFRAME_INTERVAL_MS);
  unsigned long start = millis();
  while ((millis() - start) < MOTION_TIMEOUT_MS) {
    if (!isAnyServoMoving()) break;
    delay(15);  // poll interval — low enough to be responsive, not so tight
                // that it floods the bus
  }
  // Extra settle time: absorbs resonance / micro-oscillations after stop.
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

// Send goal positions to all 6 servos in a SINGLE Protocol 1.0 Sync Write
// packet (instruction 0x83).  All AX-12 and MX-64A Protocol 1.0 servos share
// the same GOAL_POSITION address (30) and data width (2 bytes), so they can
// all be included in one packet — the bus carries one transaction instead of 6,
// eliminating the ~1 ms stagger between joints that sequential writes produce.
// physicals[] must be ordered: {ID1, ID2, ID3, ID4, ID5, ID6}.
void syncAllGoalPositions(float physicals[6]) {
  static const uint8_t  ids[6]    = {1, 2, 3, 4, 5, 6};
  static const uint16_t GOAL_POS_ADDR   = 30;  // same for AX-12 & MX-64A Proto1
  static const uint16_t GOAL_POS_LEN    = 2;

  uint16_t goals[6];
  for (int i = 0; i < 6; i++) {
    RobotMotor* m = findMotor(ids[i]);
    uint16_t model = m ? m->modelNumber : 12;
    goals[i] = physicalToRaw(model, physicals[i]);
  }

  DYNAMIXEL::InfoSyncWriteInst_t  sw_info;
  DYNAMIXEL::XELInfoSyncWrite_t   xel_infos[6];

  sw_info.addr             = GOAL_POS_ADDR;
  sw_info.addr_length      = GOAL_POS_LEN;
  sw_info.p_xels           = xel_infos;
  sw_info.xel_count        = 6;
  sw_info.is_info_changed  = true;

  for (int i = 0; i < 6; i++) {
    xel_infos[i].id     = ids[i];
    xel_infos[i].p_data = (uint8_t*)&goals[i];
  }

  d2a.syncWrite(&sw_info);
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
  PC_SERIAL.println("  777 : Play MoveIt dynamic trajectory (velocity-aware)");
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
      // PWM hobby servo on pin 3 — DISABLED (Servo.h timer conflict)
      PC_SERIAL.println("  Joint 7 (pen servo) disabled — Servo.h commented out.");
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

// 101: Home all motors to logical 180 deg
void homeAllMotors() {
  PC_SERIAL.println("Homing all motors to 180 (speed="  + String(HOME_SPEED) + ")...");
  for (int i = 0; i < 6; i++) {
    PC_SERIAL.print("  Homing motor ID="); PC_SERIAL.println(motors[i].id);
    setAngle(motors[i].id, 180.0, HOME_SPEED);
  }
  // Wait for all servos to settle before declaring homed.
  waitForMotionComplete();
  PC_SERIAL.println("Homed.");
}

// 555: Play the exported MoveIt trajectory from trajectory_data.h
void playMoveitTrajectory() {
  PC_SERIAL.println("\n--- PLAYING TRAJECTORY ---");
  if (kTrajectoryCount == 0) { PC_SERIAL.println("No trajectory points loaded."); return; }

  // penServoSweep(PEN_SERVO_GRAB_DEG, 1000);  // DISABLED
  // PC_SERIAL.println("Pen servo -> GRAB");

  clearAlarms();
  // Set speed, smoothing, and torque once for all servos before the loop.
  prepareServosForTrajectory();

  for (size_t i = 0; i < kTrajectoryCount; ++i) {
    const JointSample &p = kTrajectory[i];

    // Single sync-write packet: all 6 joints commanded simultaneously.
    {
      float physicals[6] = {
        logicalToPhysical(1, moveitRadToLogical(0, p.j1_rad)),
        logicalToPhysical(2, moveitRadToLogical(1, p.j2l_rad)),
        logicalToPhysical(3, 360.0 - moveitRadToLogical(1, p.j2l_rad)),
        logicalToPhysical(4, moveitRadToLogical(2, p.j4_rad)),
        logicalToPhysical(5, moveitRadToLogical(3, p.j5_rad)),
        logicalToPhysical(6, moveitRadToLogical(4, p.j6_rad))
      };
      syncAllGoalPositions(physicals);
    }

    if (p.pause == 1) {
      // Boundary keyframe (pen lift / touch-down): wait for the arm to
      // fully stop and settle before moving on, then dwell.
      waitForMotionComplete();
      PC_SERIAL.print("Boundary pause at keyframe "); PC_SERIAL.println(i);
      delay(RETRACT_PAUSE_MS);
    } else if (p.pause == 2) {
      // Corner settle (draw-to-draw direction change): wait for full stop
      // so the arm actually reaches the corner, then continue immediately.
      waitForMotionComplete();
      PC_SERIAL.print("Corner settle at keyframe "); PC_SERIAL.println(i);
    } else {
      // Streaming: fire the next keyframe after a fixed interval while the
      // servos are still moving.  The internal velocity profile blends
      // smoothly through each waypoint — no stop-start stepping.
      delay(KEYFRAME_INTERVAL_MS);
    }
  }
  PC_SERIAL.println("--- DONE ---");
}

// =============================================================================
// 777: Play velocity-aware MoveIt dynamic trajectory from moveItData.h
// =============================================================================

// Convert a planned joint velocity (rad/s from MoveIt) to a Dynamixel
// MOVING_SPEED register value, accounting for model-specific unit scales:
//   Servos 1,4,5,6 — AX-12A  (model  12,  Protocol 1.0): 1 unit = 0.111 RPM ≈ 86.0 units/(rad/s)
//   Servos 2,3     — MX-64   (model 310,  Protocol 1.0): 1 unit = 0.114 RPM ≈ 83.7 units/(rad/s)
static uint16_t radsToSpeedUnits(float v_rads, uint16_t floor_speed, uint16_t model) {
  float rtu = (model == 310) ? 83.7f : 86.0f;  // MX-64 vs AX-12A
  uint16_t computed = (uint16_t)(fabsf(v_rads) * rtu);
  uint16_t result = (computed > floor_speed) ? computed : floor_speed;
  return (result < 300) ? result : 300;  // hard cap at 300 for safety
}

// Write the speed register for one servo.
// AX-12A and MX-64 (Protocol 1.0) both use MOVING_SPEED.
static void setServoSpeed(uint8_t id, uint16_t speed) {
  d2a.writeControlTableItem(MOVING_SPEED, id, speed);
}

void playMoveitDynamicTrajectory() {
  PC_SERIAL.println("\n--- PLAYING DYNAMIC TRAJECTORY (777) ---");
  // penServoSweep(PEN_SERVO_GRAB_DEG, 1000);  // DISABLED
  // PC_SERIAL.println("Pen servo -> GRAB");
  clearAlarms();
  if (kDynTrajectoryCount == 0) {
    PC_SERIAL.println("No dynamic trajectory loaded. Run export tool with --output-moveit.");
    return;
  }

  // --- Diagnostic dump ---
  // Count pause boundaries and total planned duration so you can verify the
  // loaded trajectory looks correct before the arm starts moving.
  uint16_t pauseCount = 0;
  uint32_t totalMs = 0;
  for (size_t i = 0; i < kDynTrajectoryCount; ++i) {
    if (kDynTrajectory[i].pause) pauseCount++;
    totalMs += kDynTrajectory[i].dt_ms;
  }
  PC_SERIAL.print("  Waypoints loaded : "); PC_SERIAL.println(kDynTrajectoryCount);
  PC_SERIAL.print("  Pause boundaries : "); PC_SERIAL.println(pauseCount);
  PC_SERIAL.print("  Planned duration : "); PC_SERIAL.print(totalMs / 1000); PC_SERIAL.println(" s");
  PC_SERIAL.print("  First dt_ms      : "); PC_SERIAL.println(kDynTrajectory[0].dt_ms);
  PC_SERIAL.print("  Last  dt_ms      : "); PC_SERIAL.println(kDynTrajectory[kDynTrajectoryCount-1].dt_ms);
  PC_SERIAL.println("--- STARTING ---");
  uint8_t allIds[] = {1, 2, 3, 4, 5, 6};
  for (uint8_t id : allIds) {
    RobotMotor* m = findMotor(id);
    if (m && m->modelNumber == 0) m->modelNumber = d2a.getModelNumber(id);
    uint16_t model = m ? m->modelNumber : d2a.getModelNumber(id);
    applySmoothing(id, model);
    d2a.torqueOn(id);
  }

  for (size_t i = 0; i < kDynTrajectoryCount; ++i) {
    const MoveitDynamicSample &p = kDynTrajectory[i];

    // --- Per-keyframe speed (from MoveIt planned velocities) ---
    // Servo 4 gets a higher floor because it fights gravity at the worst
    // moment arm — too low and it stalls before reaching the target.
    // Model numbers are cached in motors[].modelNumber so the right
    // unit scale is used for each servo protocol (see radsToSpeedUnits).
    // Model numbers are cached during the setup loop above — these fallbacks
    // match the known hardware in case a servo didn't respond to getModelNumber.
    uint16_t m1  = findMotor(1) ? findMotor(1)->modelNumber : 12;   // AX-12A
    uint16_t m2  = findMotor(2) ? findMotor(2)->modelNumber : 310;  // MX-64
    uint16_t m4  = findMotor(4) ? findMotor(4)->modelNumber : 12;   // AX-12A
    uint16_t m5  = findMotor(5) ? findMotor(5)->modelNumber : 12;   // AX-12A
    uint16_t m6  = findMotor(6) ? findMotor(6)->modelNumber : 12;   // AX-12A
    uint16_t spd1 = radsToSpeedUnits(p.v1_rads,  5,                    m1);
    uint16_t spd2 = radsToSpeedUnits(p.v2l_rads, 5,                    m2);
    uint16_t spd4 = radsToSpeedUnits(p.v4_rads,  SERVO4_TRAJECTORY_SPEED, m4);
    uint16_t spd5 = radsToSpeedUnits(p.v5_rads,  5,                    m5);
    uint16_t spd6 = radsToSpeedUnits(p.v6_rads,  5,                    m6);

    // Timestamp before bus writes so we can subtract the overhead from
    // the inter-keyframe delay (each waypoint fires ~11 serial transactions
    // at 1 Mbaud ≈ 1-3 ms, making delays consistently late without this).
    unsigned long t_write_start = millis();

    setServoSpeed(1, spd1);
    setServoSpeed(2, spd2);
    setServoSpeed(3, spd2);  // mirrored joint shares velocity magnitude
    setServoSpeed(4, spd4);
    setServoSpeed(5, spd5);
    setServoSpeed(6, spd6);

    // Single sync-write packet: all 6 joints commanded simultaneously.
    {
      float physicals[6] = {
        logicalToPhysical(1, moveitRadToLogical(0, p.j1_rad)),
        logicalToPhysical(2, moveitRadToLogical(1, p.j2l_rad)),
        logicalToPhysical(3, 360.0 - moveitRadToLogical(1, p.j2l_rad)),
        logicalToPhysical(4, moveitRadToLogical(2, p.j4_rad)),
        logicalToPhysical(5, moveitRadToLogical(3, p.j5_rad)),
        logicalToPhysical(6, moveitRadToLogical(4, p.j6_rad))
      };
      syncAllGoalPositions(physicals);
    }

    // --- Timing ---
    // At pause boundaries (pen lift/plant) wait for full stop + settle.
    // At draw-to-draw corners (pause==2) wait for full stop but no dwell.
    // Between normal waypoints, delay exactly MoveIt's planned dt minus the
    // time already spent on bus writes, so the total cycle matches the plan.
    if (p.pause == 1) {
      waitForMotionComplete();
      PC_SERIAL.print("Boundary pause at keyframe "); PC_SERIAL.println(i);
      delay(RETRACT_PAUSE_MS);
    } else if (p.pause == 2) {
      // Corner settle: arm must fully reach the corner before changing direction.
      waitForMotionComplete();
      PC_SERIAL.print("Corner settle at keyframe "); PC_SERIAL.println(i);
    } else {
      unsigned long elapsed = millis() - t_write_start;
      if (p.dt_ms > (uint16_t)(elapsed + 1)) {
        delay(p.dt_ms - (uint16_t)elapsed);
      }
      // If bus writes took >= dt_ms, skip the delay — don't fall further behind.
    }
  }
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
  clearAlarms();
  homeAllMotors();
  scanServos();  // second scan: confirm registers after homing write

  // Set initial pen servo position — DISABLED (Servo.h timer conflict)
  // penServoSweep(PEN_SERVO_HOME_DEG, 0);
  // PC_SERIAL.println("Pen servo (ID 7, pin 3) set to " + String(PEN_SERVO_HOME_DEG) + " deg.");

  printMainMenu();
}

void loop() {
  if (PC_SERIAL.available() > 0) {
    int inputCode = PC_SERIAL.parseInt();
    while (PC_SERIAL.available() > 0) PC_SERIAL.read();

    if (inputCode == 0) {
      homeAllMotors();
    } else if (inputCode == 100) {
      jointControlMenu();
    } else if (inputCode == 200) {
      limpReadMode();
    } else if (inputCode == 555) {
      playMoveitTrajectory();
      homeAllMotors();
      // penServoSweep(PEN_SERVO_HOME_DEG, 1000);  // DISABLED
    } else if (inputCode == 777) {
      playMoveitDynamicTrajectory();
      homeAllMotors();
      // penServoSweep(PEN_SERVO_HOME_DEG, 1000);  // DISABLED
    } else {
      PC_SERIAL.println("Invalid input.");
    }
    printMainMenu();
  }
}
