#include <Dynamixel2Arduino.h>
#include <Servo.h>
#include "trajectory_data.h"
#include "moveItData.h"

#define DXL_SERIAL Serial1
#define PC_SERIAL  Serial

using namespace ControlTableItem;

Dynamixel2Arduino d2a(DXL_SERIAL);

// PWM hobby servo on pin 3 — controlled as ID 7
#define PEN_SERVO_PIN      3
#define PEN_SERVO_HOME_DEG 90
Servo penServo;

// =============================================================================
// CONFIGURATION — edit these values to tune robot behaviour
// =============================================================================
const int   HOME_SPEED         = 15;    // Speed when homing (seq 100 / startup)
const int   MANUAL_SPEED       = 15;    // Speed in manual ID control mode
const int   TRAJECTORY_SPEED   = 8;     // Speed during trajectory streaming — slow so moves take LONGER than the interval
const int   ACCEL_SMOOTH       = 15;    // Profile acceleration — moderate ramp, allows smooth blending

// Trajectory playback
// During a drawing segment keyframes are STREAMED at this interval.  The servo
// receives the next goal position while still moving toward the previous one;
// its internal velocity profile blends smoothly through each waypoint.
// Tune so the interval roughly matches how long each inter-keyframe move takes
// at TRAJECTORY_SPEED — too short and servos lag; too long and they over-stop.
const unsigned long  KEYFRAME_INTERVAL_MS = 50;  // ms between streamed keyframes
// Extra settle time (ms) after all servos report stopped at a pause boundary.
const unsigned long  SETTLE_MS            = 40;
// Dwell time (ms) applied at draw<->travel boundary keyframes (pause flag).
const unsigned long  RETRACT_PAUSE_MS     = 500;
// Timeout (ms) for waitForMotionComplete — safety valve so a stalled servo
// does not hang the sketch forever.
const unsigned long  MOTION_TIMEOUT_MS    = 8000;

// Per-servo PID overrides for servo 1 (base rotation).
// This servo has an extra bearing load causing overshoot oscillation.
// Lowering P reduces the aggressiveness of correction (less overshoot).
// Adding D adds velocity-based damping that actively kills oscillation.
// These only apply to XM/XH servos; AX-12 uses compliance slope instead.
//   XM/XH defaults: P=800, I=0, D=0
//   AX-12: compliance slope used instead (see applySmoothing)
const int SERVO1_P_GAIN          = 400;   // < 800 default: reduce overshoot aggressiveness
const int SERVO1_D_GAIN          = 400;   // > 0 default: add damping to kill oscillation
const int SERVO1_I_GAIN          = 0;     // keep at 0 — I gain adds wind-up, worsens oscillation
const int SERVO1_AX12_SLOPE      = 96;    // AX-12 only: softer compliance = less oscillation

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
  {2, 187.0, 120.0, 240.0, 0},
  {3, 189.0, 120.0, 240.0, 0},
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
  if (model == 12) return ((float)raw) * (300.0f / 1023.0f);   // AX-12: 10-bit / 300 deg
  return ((float)raw) * (360.0f / 4095.0f);                    // XM/XH:  12-bit / 360 deg
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
// MOTION
// =============================================================================

void applySmoothing(uint8_t id, uint16_t model) {
  if (model == 12) {
    // AX-12: per-servo compliance tuning.
    // Slope controls torque ramp near target (higher = softer, less snap-back).
    // Margin is the dead-zone where no torque is applied (kills hunting under load).
    int slope  = 32;  // default for all AX-12s
    int margin = 1;   // default
    if (id == 1) { slope = SERVO1_AX12_SLOPE; }
    if (id == 4) { slope = SERVO4_AX12_SLOPE; margin = SERVO4_AX12_MARGIN; }
    d2a.writeControlTableItem(CW_COMPLIANCE_SLOPE,   id, slope);
    d2a.writeControlTableItem(CCW_COMPLIANCE_SLOPE,  id, slope);
    d2a.writeControlTableItem(CW_COMPLIANCE_MARGIN,  id, margin);
    d2a.writeControlTableItem(CCW_COMPLIANCE_MARGIN, id, margin);
  } else {
    // XM/XH: set both profile velocity AND acceleration so the firmware
    // generates a proper trapezoidal (or S-curve) motion profile internally.
    d2a.writeControlTableItem(PROFILE_ACCELERATION, id, ACCEL_SMOOTH);
    // Drive mode default is velocity-based profile; PROFILE_VELOCITY is set
    // in prepareServosForTrajectory / executeMove.

    // Per-servo PID override: servo 1 has an extra bearing load that causes
    // overshoot oscillation.  Lower P reduces correction aggression; D gain
    // actively damps velocity, killing the bounce-back loop.
    if (id == 1) {
      d2a.writeControlTableItem(POSITION_P_GAIN, id, SERVO1_P_GAIN);
      d2a.writeControlTableItem(POSITION_D_GAIN, id, SERVO1_D_GAIN);
      d2a.writeControlTableItem(POSITION_I_GAIN, id, SERVO1_I_GAIN);
    }
  }
}

// Convert physical degrees to the servo's raw bit value.
uint16_t physicalToRaw(uint16_t model, float physicalDegrees) {
  return (model == 12)
    ? (uint16_t)(constrain(physicalDegrees, 0, 300) * (1023.0 / 300.0))
    : (uint16_t)(constrain(physicalDegrees, 0, 360) * (4095.0 / 360.0));
}

// Low-level: write speed + goal position to a servo in physical degrees.
// Used for homing and manual mode where per-call overhead is fine.
void executeMove(uint8_t id, float physicalDegrees, int speed) {
  RobotMotor* m = findMotor(id);
  if (m && m->modelNumber == 0) m->modelNumber = d2a.getModelNumber(id);
  uint16_t model = m ? m->modelNumber : d2a.getModelNumber(id);

  static bool smoothed[11] = {false};
  if (id < 11 && !smoothed[id]) { applySmoothing(id, model); smoothed[id] = true; }

  if (model == 12) {
    d2a.writeControlTableItem(MOVING_SPEED,     id, speed);
  } else {
    d2a.writeControlTableItem(PROFILE_VELOCITY, id, speed);
  }
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
    // Servo 4 gets its own speed: global TRAJECTORY_SPEED is too low for it
    // to maintain torque authority against gravity at full extension.
    int spd = (id == 4) ? SERVO4_TRAJECTORY_SPEED : TRAJECTORY_SPEED;
    if (model == 12) d2a.writeControlTableItem(MOVING_SPEED,     id, spd);
    else             d2a.writeControlTableItem(PROFILE_VELOCITY, id, spd);
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

// 100: Home all motors to logical 180 deg
void homeAllMotors() {
  PC_SERIAL.println("Homing all motors to 180...");
  for (int i = 0; i < 6; i++) setAngle(motors[i].id, 180.0, HOME_SPEED);
  // Wait for all servos to settle before declaring homed.
  waitForMotionComplete();
  PC_SERIAL.println("Homed.");
}

// 555: Play the exported MoveIt trajectory from trajectory_data.h
void playMoveitTrajectory() {
  PC_SERIAL.println("\n--- PLAYING TRAJECTORY ---");
  if (kTrajectoryCount == 0) { PC_SERIAL.println("No trajectory points loaded."); return; }

  // Set speed, smoothing, and torque once for all servos before the loop.
  // This cuts per-frame bus traffic from 15 writes to 5 (goal positions only),
  // so all joints receive their commands nearly simultaneously — critical for
  // smooth diagonal strokes.
  prepareServosForTrajectory();

  for (size_t i = 0; i < kTrajectoryCount; ++i) {
    const JointSample &p = kTrajectory[i];

    // Compute physical targets for all joints, then fire all goal-position
    // writes back-to-back with no other bus traffic in between.
    float phys1  = logicalToPhysical(1, moveitRadToLogical(0, p.j1_rad));
    float phys2  = logicalToPhysical(2, moveitRadToLogical(1, p.j2l_rad));
    float phys3  = logicalToPhysical(3, 360.0 - moveitRadToLogical(1, p.j2l_rad)); // mirror
    float phys4  = logicalToPhysical(4, moveitRadToLogical(2, p.j4_rad));
    float phys5  = logicalToPhysical(5, moveitRadToLogical(3, p.j5_rad));
    float phys6  = logicalToPhysical(6, moveitRadToLogical(4, p.j6_rad));

    sendGoalPosition(1, phys1);
    sendGoalPosition(2, phys2);
    sendGoalPosition(3, phys3);
    sendGoalPosition(4, phys4);
    sendGoalPosition(5, phys5);
    sendGoalPosition(6, phys6);

    if (p.pause) {
      // Boundary keyframe (pen lift / touch-down): wait for the arm to
      // fully stop and settle before moving on.  This is the ONLY place
      // where we block for completion — everywhere else we stream.
      waitForMotionComplete();
      PC_SERIAL.print("Boundary pause at keyframe "); PC_SERIAL.println(i);
      delay(RETRACT_PAUSE_MS);
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
// Protocol 1.0 MOVING_SPEED value.  Protocol 1.0 speed units ≈ 0.111 RPM for
// AX-12. A floor_speed ensures the servo always has enough torque authority
// even when the planned velocity is near zero (e.g. at approach/departure).
static uint16_t radsToSpeedUnits(float v_rads, uint16_t floor_speed) {
  const float RADS_TO_UNITS = 86.0f;  // 1 rad/s ≈ 86 speed units (Protocol 1.0)
  uint16_t computed = (uint16_t)(fabsf(v_rads) * RADS_TO_UNITS);
  uint16_t result = (computed > floor_speed) ? computed : floor_speed;
  return (result < 300) ? result : 300;  // hard cap at 300 for safety
}

// Write the speed register for one servo using the correct control table item
// (MOVING_SPEED for AX-12, PROFILE_VELOCITY for MX/XM/XH series).
static void setServoSpeed(uint8_t id, uint16_t speed) {
  RobotMotor* m = findMotor(id);
  uint16_t model = m ? m->modelNumber : 0;
  if (model == 12)
    d2a.writeControlTableItem(MOVING_SPEED,     id, speed);
  else
    d2a.writeControlTableItem(PROFILE_VELOCITY, id, speed);
}

void playMoveitDynamicTrajectory() {
  PC_SERIAL.println("\n--- PLAYING DYNAMIC TRAJECTORY (777) ---");
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
    uint16_t spd1 = radsToSpeedUnits(p.v1_rads,  5);
    uint16_t spd2 = radsToSpeedUnits(p.v2l_rads, 5);
    uint16_t spd4 = radsToSpeedUnits(p.v4_rads,  SERVO4_TRAJECTORY_SPEED);
    uint16_t spd5 = radsToSpeedUnits(p.v5_rads,  5);
    uint16_t spd6 = radsToSpeedUnits(p.v6_rads,  5);

    setServoSpeed(1, spd1);
    setServoSpeed(2, spd2);
    setServoSpeed(3, spd2);  // mirrored joint shares velocity magnitude
    setServoSpeed(4, spd4);
    setServoSpeed(5, spd5);
    setServoSpeed(6, spd6);

    // --- Fire all goal positions simultaneously ---
    float phys1 = logicalToPhysical(1, moveitRadToLogical(0, p.j1_rad));
    float phys2 = logicalToPhysical(2, moveitRadToLogical(1, p.j2l_rad));
    float phys3 = logicalToPhysical(3, 360.0 - moveitRadToLogical(1, p.j2l_rad));
    float phys4 = logicalToPhysical(4, moveitRadToLogical(2, p.j4_rad));
    float phys5 = logicalToPhysical(5, moveitRadToLogical(3, p.j5_rad));
    float phys6 = logicalToPhysical(6, moveitRadToLogical(4, p.j6_rad));

    sendGoalPosition(1, phys1);
    sendGoalPosition(2, phys2);
    sendGoalPosition(3, phys3);
    sendGoalPosition(4, phys4);
    sendGoalPosition(5, phys5);
    sendGoalPosition(6, phys6);

    // --- Timing ---
    // At pause boundaries (pen lift/plant) wait for full stop + settle.
    // Between normal waypoints, delay exactly MoveIt's planned dt so the
    // servo receives the next command while still on its velocity ramp —
    // the arm traces the planned path instead of stopping at each waypoint.
    if (p.pause) {
      waitForMotionComplete();
      PC_SERIAL.print("Boundary pause at keyframe "); PC_SERIAL.println(i);
      delay(RETRACT_PAUSE_MS);
    } else {
      delay(p.dt_ms);
    }
  }
  PC_SERIAL.println("--- DONE ---");
}

void setup() {
  PC_SERIAL.begin(1000000);
  while (!PC_SERIAL);
  d2a.begin(1000000);
  d2a.setPortProtocolVersion(1.0);

  PC_SERIAL.println("--- SYSTEM INITIALIZED ---");
  homeAllMotors();

  // Attach and home the PWM servo on pin 3 (ID 7)
  penServo.attach(PEN_SERVO_PIN);
  penServo.write(PEN_SERVO_HOME_DEG);
  PC_SERIAL.println("Pen servo (ID 7, pin 3) homed to 90 deg.");

  PC_SERIAL.println("\nREADY. CODES:");
  PC_SERIAL.println("  100 : Home all motors");
  PC_SERIAL.println("  555 : Play trajectory (fixed speed)");
  PC_SERIAL.println("  777 : Play MoveIt dynamic trajectory (velocity-aware)");
  PC_SERIAL.println("  7   : Manual control — pen servo (pin 3, 0-180 deg)");
  PC_SERIAL.println("  [ID]: Manual control mode");
}

void loop() {
  if (PC_SERIAL.available() > 0) {
    int inputCode = PC_SERIAL.parseInt();
    while (PC_SERIAL.available() > 0) PC_SERIAL.read();

    if (inputCode == 100) {
      homeAllMotors();
    } else if (inputCode == 555) {
      playMoveitTrajectory();
      homeAllMotors();
    } else if (inputCode == 777) {
      playMoveitDynamicTrajectory();
      homeAllMotors();
    } else if (inputCode == 7) {
      // PWM hobby servo on pin 3 — not a Dynamixel, handled separately
      PC_SERIAL.println("Locked: ID 7 (pen servo, pin 3)");
      while (true) {
        PC_SERIAL.println("ID 7 Angle 0-180 (999 to exit):");
        while (PC_SERIAL.available() == 0);
        float angle = PC_SERIAL.parseFloat();
        while (PC_SERIAL.available() > 0) PC_SERIAL.read();
        if (angle == 999) break;
        int deg = (int)constrain(angle, 0, 180);
        penServo.write(deg);
        PC_SERIAL.print("Pen servo -> "); PC_SERIAL.println(deg);
      }
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
      PC_SERIAL.println("Invalid input or motor not found.");
    }
  }
}
