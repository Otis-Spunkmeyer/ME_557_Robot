#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1 
#define PC_SERIAL Serial   

using namespace ControlTableItem; 

Dynamixel2Arduino d2a(DXL_SERIAL);

// --- GLOBAL CONFIGURATION ---
const int GLOBAL_SPEED_RAW = 100; 
const int SWEEP_SPEED_RAW = 60; 
const int ACCEL_VAL_SMOOTH = 15; 

float previousM5Angle = 180.0; 
bool m5IsTucked = false;

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
  {4, 150.0, 40.0,  240.0}, 
  {5, 150.0, 100.0, 260.0}, 
  {6, 150.0, 140.0, 300.0}  
};

// --- MOTION SMOOTHING ---
void applySmoothing(uint8_t id, uint16_t model) {
  if (model == 12) { 
    d2a.writeControlTableItem(CW_COMPLIANCE_SLOPE, id, 128);
    d2a.writeControlTableItem(CCW_COMPLIANCE_SLOPE, id, 128);
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
  uint16_t bitValue = (model == 12) ? (uint16_t)(constrain(physicalDegrees, 0, 300) * (1023.0 / 300.0)) : (uint16_t)(constrain(physicalDegrees, 0, 360) * (4095.0 / 360.0));
  d2a.torqueOn(id);
  d2a.setGoalPosition(id, bitValue);
}

float logicalToPhysical(uint8_t id, float logicalAngle) {
  for (int i = 0; i < 6; i++) {
    if (motors[i].id == id) {
      float safeLogical = constrain(logicalAngle, motors[i].lowerLimit, motors[i].upperLimit);
      return safeLogical + (motors[i].physicalOffset - 180.0);
    }
  }
  return logicalAngle;
}

void setAngle(uint8_t id, float logicalAngle, int speed = GLOBAL_SPEED_RAW) {
  if (id == 5 && !m5IsTucked) previousM5Angle = logicalAngle;

  if (id == 4) {
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

  if (id == 3 || id == 2) {
    uint8_t partnerID = (id == 3) ? 2 : 3;
    executeMove(partnerID, logicalToPhysical(partnerID, 360.0 - logicalAngle), speed);
  }
}

// --- TRIGGERED ROUTINES ---

void individualSweep() {
  PC_SERIAL.println("\n--- RUNNING INDIVIDUAL ROM SWEEP (777) ---");
  uint8_t sweepOrder[] = {6, 5, 4, 3, 1}; 
  for (uint8_t id : sweepOrder) {
    float low, high;
    for(int j=0; j<6; j++) if(motors[j].id == id) { low = motors[j].lowerLimit; high = motors[j].upperLimit; }
    
    PC_SERIAL.print("Sweeping ID "); PC_SERIAL.println(id);
    if (id == 4) { setAngle(4, 69.0, SWEEP_SPEED_RAW); delay(2000); }

    setAngle(id, low, SWEEP_SPEED_RAW); delay(3000);
    setAngle(id, high, SWEEP_SPEED_RAW); delay(4500);
    setAngle(id, 180.0, SWEEP_SPEED_RAW); delay(3000);
  }
  PC_SERIAL.println("--- INDIVIDUAL SWEEP COMPLETE ---");
}

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
  while(!PC_SERIAL); 
  d2a.begin(1000000);
  d2a.setPortProtocolVersion(1.0);

  PC_SERIAL.println("--- SYSTEM INITIALIZED ---");
  PC_SERIAL.println("Homing all motors to 180...");
  for (int i = 0; i < 6; i++) setAngle(motors[i].id, 180.0, 40);
  
  PC_SERIAL.println("\nREADY. COMMAND CODES:");
  PC_SERIAL.println("777: Individual ROM Sweep");
  PC_SERIAL.println("888: Parallel ROM Showcase");
  PC_SERIAL.println("[ID]: Enter manual control mode");
}

void loop() {
  if (PC_SERIAL.available() > 0) {
    int inputCode = PC_SERIAL.parseInt();
    while(PC_SERIAL.available() > 0) PC_SERIAL.read();

    if (inputCode == 777) {
      individualSweep();
    } else if (inputCode == 888) {
      showcaseFullROM();
    } else if (d2a.ping(inputCode)) {
      PC_SERIAL.print("Locked: ID "); PC_SERIAL.println(inputCode);
      while (true) {
        PC_SERIAL.print("ID "); PC_SERIAL.print(inputCode); PC_SERIAL.println(" Angle (999 to exit):");
        while (PC_SERIAL.available() == 0);
        float angle = PC_SERIAL.parseFloat();
        while(PC_SERIAL.available() > 0) PC_SERIAL.read();
        if (angle == 999) break;
        setAngle(inputCode, angle);
      }
    } else {
       PC_SERIAL.println("INVALID INPUT or Motor not found.");
    }
  }
}
