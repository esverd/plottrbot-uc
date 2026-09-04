#include <Arduino.h>
#include <Servo.h>
#include <TMCStepper.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include <plottrbot/kinematics.h>
#include <plottrbot/protocol.h>
#include <plottrbot/controller_core.h>

using namespace plottrbot;

const int enablePinLR = 2;
const int stepPinL = 3;
const int dirPinL = 4;
const int csPinL = 7;
const int stepPinR = 5;
const int dirPinR = 6;
const int csPinR = 8;
const int servoPin = 10;

Servo penServo;
float r_sense = 0.11f;
TMC2130Stepper leftStepperDriver(csPinL, r_sense);
TMC2130Stepper rightStepperDriver(csPinR, r_sense);

const float CALIBRATED_PULLEY_DIAMETER_MM = 12.0f;
const float CALIBRATED_BELT_SCALE = (17.9f / 18.0f) * (19.1f / 18.0f);
const float DEFAULT_MM_PER_MICROSTEP = (CALIBRATED_PULLEY_DIAMETER_MM * CALIBRATED_BELT_SCALE * PI) / 3200.0f;

// Keep scales independent even while their compiled defaults match. Future
// calibration must update one field, never collapse them into a shared scale.
MachineConfig machineConfig = {
  1162.0f, 581.0f, 240.0f, 1162.0f, 1000.0f,
  DEFAULT_MM_PER_MICROSTEP, DEFAULT_MM_PER_MICROSTEP
};

// This state is authoritative. It remains unknown until the operator confirms
// the physical homing key with G92 H.
ControllerState controllerState = {false, {0, 0}};

const int servoPosDraw = 100;
const int servoPosNoDraw = 140;
int servoPosCurrent = servoPosNoDraw;
const int servoDrawDelay = 12;
const int servoNoDrawDelay = 4;

const int DEFAULT_SPEED_DELAY = 90;
const int SLOWEST_SPEED_DELAY = 240;
const int STEPS_TO_ACCEL_DECCEL = 180;
float currentSpeedDelay = DEFAULT_SPEED_DELAY;
float totalLineSteps = 0.0f;
float traveledSteps = 0.0f;
int accelMode = 0;
const float MAX_CARTESIAN_CHORD_MM = 2.0f;

const size_t CMD_BUFFER_SIZE = 160;
char cmdBuffer[CMD_BUFFER_SIZE] = {0};
size_t cmdLength = 0;
bool commandOverflow = false;

void resetCommandBuffer();
void handleGCODE();
void handleAccel();
void pulseMotor(bool leftMotor, bool beltLengthening);
void servoPenDraw(bool draw);
void printError(const char *code);
bool executeStepTarget(const StepPosition &target);
bool executeCartesianLine(const CartesianMovePlan &motion);
void reportM503();

void setup() {
  Serial.begin(9600);
  pinMode(stepPinL, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  pinMode(stepPinR, OUTPUT);
  pinMode(dirPinR, OUTPUT);
  pinMode(enablePinLR, OUTPUT);
  digitalWrite(enablePinLR, HIGH);  // Boot fail-safe: drivers are disabled.
  penServo.attach(servoPin);        // Do not write at boot: do not move pen.

  SPI.begin();
  leftStepperDriver.begin();
  leftStepperDriver.rms_current(700);
  leftStepperDriver.microsteps(16);
  leftStepperDriver.en_pwm_mode(true);
  leftStepperDriver.pwm_autoscale(true);
  rightStepperDriver.begin();
  rightStepperDriver.rms_current(700);
  rightStepperDriver.microsteps(16);
  rightStepperDriver.en_pwm_mode(true);
  rightStepperDriver.pwm_autoscale(true);

  if(!isValidConfig(machineConfig))
    printError("config:invalid");
  Serial.println("Starting");
}

void loop() {
  while(Serial.available() > 0) {
    const char inChar = (char)Serial.read();
    if(inChar == '\r')
      continue;
    if(inChar == '\n') {
      if(commandOverflow)
        printError("command:too_long");
      else if(cmdLength > 0)
        handleGCODE();
      if(cmdLength > 0 || commandOverflow)
        Serial.println("GO");
      resetCommandBuffer();
    } else if(cmdLength + 1 < CMD_BUFFER_SIZE) {
      cmdBuffer[cmdLength++] = inChar;
      cmdBuffer[cmdLength] = '\0';
    } else {
      commandOverflow = true;
    }
  }
}

void handleAccel() {
  const float incDecToDelay = (SLOWEST_SPEED_DELAY - DEFAULT_SPEED_DELAY) / (float)STEPS_TO_ACCEL_DECCEL;
  if(traveledSteps < STEPS_TO_ACCEL_DECCEL) {
    accelMode = 1;
    currentSpeedDelay = SLOWEST_SPEED_DELAY;
  }
  if(traveledSteps + STEPS_TO_ACCEL_DECCEL > totalLineSteps)
    accelMode = -1;
  if(accelMode == 1) {
    currentSpeedDelay -= incDecToDelay;
    if(currentSpeedDelay < DEFAULT_SPEED_DELAY) {
      currentSpeedDelay = DEFAULT_SPEED_DELAY;
      accelMode = 0;
    }
  } else if(accelMode == -1) {
    currentSpeedDelay += incDecToDelay;
    if(currentSpeedDelay > SLOWEST_SPEED_DELAY) {
      currentSpeedDelay = SLOWEST_SPEED_DELAY;
      accelMode = 0;
    }
  }
  ++traveledSteps;
}

void pulseMotor(bool leftMotor, bool beltLengthening) {
  const int stepPin = leftMotor ? stepPinL : stepPinR;
  const int dirPin = leftMotor ? dirPinL : dirPinR;
  // The left motor's physical direction is opposite the right motor's.
  const bool driverDirection = leftMotor ? !beltLengthening : beltLengthening;
  digitalWrite(dirPin, driverDirection);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds((unsigned int)currentSpeedDelay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds((unsigned int)currentSpeedDelay);
}

bool executeStepTarget(const StepPosition &target) {
  TwoAxisDda dda(controllerState.steps, target);
  DdaTick tick;
  const int32_t leftDirection = dda.leftDelta() < 0 ? -1 : 1;
  const int32_t rightDirection = dda.rightDelta() < 0 ? -1 : 1;
  while(dda.next(&tick)) {
    handleAccel();
    if(tick.pulse_left) {
      pulseMotor(true, leftDirection > 0);
      controllerState.steps.left += leftDirection;
    }
    if(tick.pulse_right) {
      pulseMotor(false, rightDirection > 0);
      controllerState.steps.right += rightDirection;
    }
  }
  return controllerState.steps.left == target.left && controllerState.steps.right == target.right;
}

bool executeCartesianLine(const CartesianMovePlan &motion) {
  if(motion.chord_count == 0)
    return true;
  totalLineSteps = motion.total_pulses;
  traveledSteps = 0.0f;
  accelMode = 0;
  digitalWrite(enablePinLR, LOW);
  const float deltaX = motion.target.x_mm - motion.start.x_mm;
  const float deltaY = motion.target.y_mm - motion.start.y_mm;
  for(uint16_t chord = 1; chord <= motion.chord_count; ++chord) {
    const float t = chord / (float)motion.chord_count;
    const CartesianPoint chordTarget = {motion.start.x_mm + deltaX * t, motion.start.y_mm + deltaY * t};
    StepPosition targetSteps;
    if(!cartesianToSteps(machineConfig, chordTarget, &targetSteps) || !executeStepTarget(targetSteps)) {
      printError("motion:planner");
      return false;
    }
  }
  return true;
}

void servoPenDraw(bool draw) {
  const int target = draw ? servoPosDraw : servoPosNoDraw;
  const int delayMs = draw ? servoDrawDelay : servoNoDrawDelay;
  while(servoPosCurrent != target) {
    servoPosCurrent += target > servoPosCurrent ? 1 : -1;
    penServo.write(servoPosCurrent);
    delay(delayMs);
  }
  delay(50);
}

void printError(const char *code) {
  Serial.print("ERR ");
  Serial.println(code);
}

void reportM503() {
  Serial.print("M503 ANCHOR_SPAN:"); Serial.print(machineConfig.anchor_span_mm, 3);
  Serial.print(" HOME_X:"); Serial.print(machineConfig.home_x_mm, 3);
  Serial.print(" HOME_Y:"); Serial.print(machineConfig.home_y_mm, 3);
  Serial.print(" WIDTH:"); Serial.print(machineConfig.canvas_width_mm, 3);
  Serial.print(" HEIGHT:"); Serial.print(machineConfig.canvas_height_mm, 3);
  Serial.print(" LEFT_MM_PER_MICROSTEP:"); Serial.print(machineConfig.left_mm_per_microstep, 7);
  Serial.print(" RIGHT_MM_PER_MICROSTEP:"); Serial.println(machineConfig.right_mm_per_microstep, 7);
}

void handleGCODE() {
  CommandPlan plan;
  const char *error = nullptr;
  if(!planCommand(cmdBuffer, machineConfig, controllerState, MAX_CARTESIAN_CHORD_MM, &plan, &error)) {
    printError(error);
    return;
  }
  switch(plan.kind) {
    case COMMAND_M115: {
      char response[230]; formatM115(response, sizeof(response)); Serial.println(response); break;
    }
    case COMMAND_M114: {
      char response[100]; formatM114(response, sizeof(response), machineConfig, controllerState.steps, controllerState.position_known); Serial.println(response); break;
    }
    case COMMAND_M503:
      reportM503(); break;
    case COMMAND_SAFE_DIAGNOSTIC: {
      char response[150]; formatSafeDiagnostic(response, sizeof(response), machineConfig, MAX_CARTESIAN_CHORD_MM); Serial.println(response); break;
    }
    case COMMAND_G92_HOME:
      controllerState.steps = plan.home_steps;
      controllerState.position_known = true;
      Serial.println("OK G92 H");
      break;
    case COMMAND_G1:
      if(plan.change_pen) servoPenDraw(plan.pen_draw);
      if(plan.motion.chord_count > 0) executeCartesianLine(plan.motion);
      break;
    case COMMAND_G28:
      if(plan.motion.chord_count > 0) executeCartesianLine(plan.motion);
      break;
    case COMMAND_M17:
      digitalWrite(enablePinLR, LOW); Serial.println("OK M17"); break;
    case COMMAND_M18:
      digitalWrite(enablePinLR, HIGH); Serial.println("OK M18"); break;
    default:
      printError("internal:command_plan"); break;
  }
}

void resetCommandBuffer() {
  cmdLength = 0;
  cmdBuffer[0] = '\0';
  commandOverflow = false;
}
