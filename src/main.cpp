#include <Arduino.h>
#include <Servo.h>
#include <TMCStepper.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include <plottrbot/kinematics.h>
#include <plottrbot/protocol.h>

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

// This state is authoritative. It is updated only after a physical pulse.
StepPosition currentSteps = {0, 0};

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
bool executeCartesianLine(const CartesianPoint &target);
bool getCurrentCartesian(CartesianPoint *point);
bool setLogicalPosition(const CartesianPoint &point);
bool hasCommand(const char *code);
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

  if(!setLogicalPosition(CartesianPoint{machineConfig.home_x_mm, machineConfig.home_y_mm}))
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
  TwoAxisDda dda(currentSteps, target);
  DdaTick tick;
  const int32_t leftDirection = dda.leftDelta() < 0 ? -1 : 1;
  const int32_t rightDirection = dda.rightDelta() < 0 ? -1 : 1;
  while(dda.next(&tick)) {
    handleAccel();
    if(tick.pulse_left) {
      pulseMotor(true, leftDirection > 0);
      currentSteps.left += leftDirection;
    }
    if(tick.pulse_right) {
      pulseMotor(false, rightDirection > 0);
      currentSteps.right += rightDirection;
    }
  }
  return currentSteps.left == target.left && currentSteps.right == target.right;
}

bool getCurrentCartesian(CartesianPoint *point) {
  return stepsToCartesian(machineConfig, currentSteps, point);
}

bool setLogicalPosition(const CartesianPoint &point) {
  StepPosition target;
  if(!cartesianToSteps(machineConfig, point, &target))
    return false;
  currentSteps = target;
  return true;
}

bool executeCartesianLine(const CartesianPoint &target) {
  if(!isInBounds(machineConfig, target)) {
    printError("bounds:xy");
    return false;
  }
  CartesianPoint start;
  if(!getCurrentCartesian(&start)) {
    printError("state:unresolvable");
    return false;
  }
  const float deltaX = target.x_mm - start.x_mm;
  const float deltaY = target.y_mm - start.y_mm;
  const float length = sqrtf(deltaX * deltaX + deltaY * deltaY);
  const uint16_t chordCount = length <= 0.0f ? 0 : (uint16_t)ceilf(length / MAX_CARTESIAN_CHORD_MM);

  // Preflight every absolute target before enabling hardware or emitting a pulse.
  StepPosition predicted = currentSteps;
  float pulseCount = 0.0f;
  for(uint16_t chord = 1; chord <= chordCount; ++chord) {
    const float t = chord / (float)chordCount;
    const CartesianPoint chordTarget = {start.x_mm + deltaX * t, start.y_mm + deltaY * t};
    StepPosition targetSteps;
    if(!cartesianToSteps(machineConfig, chordTarget, &targetSteps)) {
      printError("bounds:chord");
      return false;
    }
    TwoAxisDda planner(predicted, targetSteps);
    pulseCount += planner.totalTicks();
    predicted = targetSteps;
  }

  if(chordCount == 0)
    return true;
  totalLineSteps = pulseCount;
  traveledSteps = 0.0f;
  accelMode = 0;
  digitalWrite(enablePinLR, LOW);
  for(uint16_t chord = 1; chord <= chordCount; ++chord) {
    const float t = chord / (float)chordCount;
    const CartesianPoint chordTarget = {start.x_mm + deltaX * t, start.y_mm + deltaY * t};
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

bool hasCommand(const char *code) {
  const size_t length = strlen(code);
  return strncmp(cmdBuffer, code, length) == 0 &&
      (cmdBuffer[length] == '\0' || cmdBuffer[length] == ' ' || cmdBuffer[length] == '\t');
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
  if(hasCommand("M115")) {
    char response[190];
    formatM115(response, sizeof(response));
    Serial.println(response);
    return;
  }
  if(hasCommand("M114")) {
    char response[100];
    formatM114(response, sizeof(response), machineConfig, currentSteps);
    Serial.println(response);
    return;
  }
  if(hasCommand("M503")) {
    reportM503();
    return;
  }
  if(hasCommand("G5")) {
    printError("unsupported:G5_flatten_on_host");
    return;
  }
  if(hasCommand("G1") || hasCommand("G01")) {
    if(strchr(cmdBuffer, 'L') != nullptr || strchr(cmdBuffer, 'R') != nullptr) {
      printError("unsupported:G1_LR");
      return;
    }
    float x = 0.0f, y = 0.0f, z = 0.0f;
    const AxisValueResult xResult = extractAxisFloat(cmdBuffer, 'X', &x);
    const AxisValueResult yResult = extractAxisFloat(cmdBuffer, 'Y', &y);
    const AxisValueResult zResult = extractAxisFloat(cmdBuffer, 'Z', &z);
    if(xResult == AXIS_MALFORMED || yResult == AXIS_MALFORMED || zResult == AXIS_MALFORMED) {
      printError("malformed:G1");
      return;
    }
    if(xResult == AXIS_ABSENT && yResult == AXIS_ABSENT && zResult == AXIS_ABSENT) {
      printError("malformed:G1_missing_axis");
      return;
    }
    if(zResult == AXIS_VALID && z != 0.0f && z != 1.0f) {
      printError("bounds:Z");
      return;
    }
    CartesianPoint target;
    if(xResult == AXIS_VALID || yResult == AXIS_VALID) {
      if(!getCurrentCartesian(&target)) {
        printError("state:unresolvable");
        return;
      }
      if(xResult == AXIS_VALID) target.x_mm = x;
      if(yResult == AXIS_VALID) target.y_mm = y;
      if(!isInBounds(machineConfig, target)) {
        printError("bounds:xy");
        return;
      }
    }
    if(zResult == AXIS_VALID)
      servoPenDraw(z == 0.0f);
    if(xResult == AXIS_VALID || yResult == AXIS_VALID)
      executeCartesianLine(target);
    return;
  }
  if(hasCommand("G28")) {
    executeCartesianLine(CartesianPoint{machineConfig.home_x_mm, machineConfig.home_y_mm});
    return;
  }
  if(hasCommand("M17")) {
    digitalWrite(enablePinLR, LOW);
    Serial.println("OK M17");
    return;
  }
  if(hasCommand("M18")) {
    digitalWrite(enablePinLR, HIGH);
    Serial.println("OK M18");
    return;
  }
  if(hasCommand("G92")) {
    if(strstr(cmdBuffer, "G92 H") != nullptr) {
      if(setLogicalPosition(CartesianPoint{machineConfig.home_x_mm, machineConfig.home_y_mm}))
        Serial.println("OK G92 H");
      else
        printError("config:invalid");
      return;
    }
    float x = 0.0f, y = 0.0f;
    const AxisValueResult xResult = extractAxisFloat(cmdBuffer, 'X', &x);
    const AxisValueResult yResult = extractAxisFloat(cmdBuffer, 'Y', &y);
    if(xResult == AXIS_MALFORMED || yResult == AXIS_MALFORMED || (xResult == AXIS_ABSENT && yResult == AXIS_ABSENT)) {
      printError("malformed:G92");
      return;
    }
    CartesianPoint target;
    if(!getCurrentCartesian(&target)) {
      printError("state:unresolvable");
      return;
    }
    if(xResult == AXIS_VALID) target.x_mm = x;
    if(yResult == AXIS_VALID) target.y_mm = y;
    if(!setLogicalPosition(target))
      printError("bounds:G92");
    else
      Serial.println("OK G92");
    return;
  }
  printError("unsupported:command");
}

void resetCommandBuffer() {
  cmdLength = 0;
  cmdBuffer[0] = '\0';
  commandOverflow = false;
}
