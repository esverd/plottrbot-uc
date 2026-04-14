

#include <Arduino.h>
#include <Servo.h>
#include <TMCStepper.h>   //needs to be my fork which has some important changes for stepper motor driving
#include <stdlib.h>
#include <string.h>


//-------PIN I/O-------
const int enablePinLR = 2;
const int stepPinL = 3; 
const int dirPinL = 4; 
const int csPinL = 7; 
const int stepPinR = 5; 
const int dirPinR = 6; 
const int csPinR = 8; 
const int servoPin = 10;

//-------MOTOR CONFIG-------
Servo penServo;  // create servo object to control a servo
float r_sense = 0.11;
TMC2130Stepper leftStepperDriver(csPinL, r_sense);                           // Hardware SPI
TMC2130Stepper rightStepperDriver(csPinR, r_sense);                           // Hardware SPI  

//-------CALIBRATION-------
unsigned int canvasWidth = 1162;    //width between center of the two motor axis. unit is mm
unsigned int canvasHeight = 1000;   //TODO bruke denne variabelen for å ikke gå utenfor maks høyde. brukes til å oppgi maks høyde med vekt på belte
float homeX = (canvasWidth / 2.0);
float homeY = (32 + 208); //200.0;            //homing key neck (168mm) + center motor axle to center rail (32mm) = 200mm

float scaleTotalDistance = (17.9/18)*(19.1/18); //(73.0/70)*(54.3/55)*(55.0/57)*(55/55.5);
float diameterPulley = 12.0; //12.2; //12.723; //11.98;    //in mm  //var rundt 12.723 med gamle stepper drivers.- 11.98 med tmc2130
float Ts = (diameterPulley*scaleTotalDistance*PI)/(3200.0);    //3200 the number of steps to complete full rotation of motor. micro stepping = 16

const int servoPosDraw = 100;     //servo position when the pen touches the canvas
const int servoPosNoDraw = 140;   //servo position when the pen doesn't touch the canvas
int servoPosCurrent = servoPosDraw;   //sets the current position to drawing to make sure the robot later boots by moving to noDrawPosition
int servoDrawDelay = 12;    //delay in ms for the servo to move to the drawing position
int servoNoDrawDelay = 4;  //delay in ms for the servo to move to the no drawing position

float currentX = homeX;
float currentY = homeY;

//-------SPEED SETTINGS-------
const int DEFAULT_SPEED_DELAY = 90;   //100;    
const int SLOWEST_SPEED_DELAY = 240;  //280;    
float currentSpeedDelay = DEFAULT_SPEED_DELAY;
float totalLineSteps = 0;      //used to store the toal motor pulses to move a line. necessary for accel and deccel
float traveledSteps = 0.0;
const int STEPS_TO_ACCEL_DECCEL = 180;  //160;   
// const int MM_TO_ACCEL_DECCEL = 20;
// float totalMMtoTravel = 0.0;
int accelMode = 0;      //0 = plain. 1 = accelerate. -1 = deccelerate

//-------SERIAL COMMUNICATION-------
int incomingByte = 0; // for incoming serial data
// Use a fixed command buffer so long plots do not depend on AVR heap/String behavior.
const size_t CMD_BUFFER_SIZE = 160;
char cmdBuffer[CMD_BUFFER_SIZE] = {0};
size_t cmdLength = 0;

//-------FUNCTION PROTOTYPES-------
void runMotor(bool, int, bool);
void pulseMotor(bool, bool);
void moveToPosition(float, float, float, float);
void printXYfromHypo(float, float);
float getXfromHypo(float, float);
float getYfromHypo(float, float);
void interpolateToPosition(float, float, float, float);
void interpolateToPosition(float, float, float, float, bool);
void servoPenDraw(bool);
void readSerial();
void handleGCODE();
void G1xyz();
void G1lr();
void handleAccel();
bool tryExtractFloat(char, float *);
void resetCommandBuffer();

void setup() 
{
  Serial.begin(9600);
  penServo.attach(servoPin);
  Serial.println("Starting");
  servoPenDraw(false);      //starts with the pen not touching the canvas

  //-------stepper motor setup-------
  pinMode(stepPinL, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  pinMode(stepPinR, OUTPUT);
  pinMode(dirPinR, OUTPUT);
  pinMode(enablePinLR, OUTPUT);
  digitalWrite(enablePinLR, LOW);      // Enable driver in hardware
  
  SPI.begin();                    // SPI drivers

  leftStepperDriver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
  // leftStepperDriver.toff(5);                 // Enables driver in software
  leftStepperDriver.rms_current(700);        // Set motor RMS current
  leftStepperDriver.microsteps(16);          // Set microsteps to 1/16th
  leftStepperDriver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  leftStepperDriver.pwm_autoscale(true);     // Needed for stealthChop

  rightStepperDriver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
  // rightStepperDriver.toff(5);                 // Enables driver in software
  rightStepperDriver.rms_current(700);        // Set motor RMS current
  rightStepperDriver.microsteps(16);          // Set microsteps to 1/16th
  rightStepperDriver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160   skal egentlig være true for stealthchop
  rightStepperDriver.pwm_autoscale(true);     // Needed for stealthChop

}

void loop() 
{
  readSerial();
}


void handleAccel()
{
  float incDecToDelay = (SLOWEST_SPEED_DELAY - DEFAULT_SPEED_DELAY) / (float)STEPS_TO_ACCEL_DECCEL;   //the amount to decrease or increase step delay

  if(traveledSteps - STEPS_TO_ACCEL_DECCEL < 0)      //if its the first steps in a new line
  {
    accelMode = 1;    //accelerate the movement
    currentSpeedDelay = SLOWEST_SPEED_DELAY;
  }
  if(traveledSteps + STEPS_TO_ACCEL_DECCEL > totalLineSteps)    //if its the last steps in the current line line
    accelMode = -1;   //deccelerate the movement

  if(accelMode == 1)
  {
    currentSpeedDelay -= incDecToDelay;     //accelerates frequency of motor pulses
    if(currentSpeedDelay < DEFAULT_SPEED_DELAY)   //if the speed delay gets set to outside limits
    {
      currentSpeedDelay = DEFAULT_SPEED_DELAY;
      accelMode = 0;      //stop acceleration process
    }
  }
  if(accelMode == -1)
  {
    currentSpeedDelay += incDecToDelay;   //if the speed delay gets set to outside limits
    if(currentSpeedDelay > SLOWEST_SPEED_DELAY)
    {
      currentSpeedDelay = SLOWEST_SPEED_DELAY;
      accelMode = 0;    //stop decceleration process
    }
  }

  traveledSteps++;    //increment number of steps moved in the current line
}

void readSerial()
{
  //if serial available
  //send start signal to receive new commands
  //store commands in buffer
  //\n is end of command
  //process buffer by handling GCODE
  //loop

  while(Serial.available() > 0)    //if serial communication is avalable
  {
    char inChar = Serial.read();    //reads next character

    if(inChar == '\r')
      continue;

    if(inChar == '\n')    //if string ended with new line process command
    {
      if(cmdLength > 0)
      {
        handleGCODE();    //processes the received commans
        Serial.println("GO");   //prints GO to signal the arduino is ready for the next command
      }
    }
    else if(cmdLength + 1 < CMD_BUFFER_SIZE)
    {
      cmdBuffer[cmdLength++] = inChar;    //stores the incoming character string
      cmdBuffer[cmdLength] = '\0';
    }
  }

}

void handleGCODE()
{
  //eks: G1 X85.469 Y85.935
  //eks: G1 Z1

  if(strncmp(cmdBuffer, "G01 ", 4) == 0 || strcmp(cmdBuffer, "G01") == 0 || strncmp(cmdBuffer, "G1 ", 3) == 0 || strcmp(cmdBuffer, "G1") == 0)
  {
    if(strchr(cmdBuffer, 'X') != nullptr || strchr(cmdBuffer, 'Y') != nullptr || strchr(cmdBuffer, 'Z') != nullptr)
      G1xyz();   //moves the robot to the given coordinates
    else if(strchr(cmdBuffer, 'L') != nullptr || strchr(cmdBuffer, 'R') != nullptr)
      G1lr();
  }
  else if(strncmp(cmdBuffer, "G28", 3) == 0)    
  {
    // currentX = homeX;  //sets the current position to the home position
    // currentY = homeY;
    interpolateToPosition(currentX, currentY, homeX, homeY);    //moves to home position
  }
  else if(strncmp(cmdBuffer, "M17", 3) == 0)   
    digitalWrite(enablePinLR, LOW);       //enables power to stepper motors
  else if(strncmp(cmdBuffer, "M18", 3) == 0)   
    digitalWrite(enablePinLR, HIGH);    //disable power to stepper motors
  else if(strncmp(cmdBuffer, "G92", 3) == 0)   //sets the current coordinates without moving motors
  {
    if(strstr(cmdBuffer, "G92 H") != nullptr)
    {
      currentX = homeX;
      currentY = homeY;
    }
    else
    {
      float xVal;
      float yVal;
      if(tryExtractFloat('X', &xVal))    //if proper values were sent
        currentX = xVal;  //set current coordinates to sent coordinates
      if(tryExtractFloat('Y', &yVal))
        currentY = yVal;
    }
  }

  resetCommandBuffer();   //readies the buffer to receive a new command
}


//move to coordinates in a straight line
//void moveToPosition(int x0, int y0, int x1, int y1)
//calculate how much to move each motor, and slope of straight curve
//move motors in correct relationship
void interpolateToPosition(float x0, float y0, float x1, float y1, bool draw)   //overloaded draw function which also sets the pen position
{
  servoPenDraw(draw);
  interpolateToPosition(x0, y0, x1, y1);
}

void interpolateToPosition(float x0, float y0, float x1, float y1)
{
  if(x1 >= 0 && y1 >= 0 && x1 <= canvasWidth && y1 <= canvasHeight)   //if the new position is within the robot bounds
  {
    digitalWrite(enablePinLR, LOW);      //enables power to stepper motors
    float targetX = x1;
    float targetY = y1;

    float deltaX = targetX - x0;
    float deltaY = targetY - y0;
    float Tl = sqrt( pow(deltaX, 2) + pow(deltaY, 2) );   //total length to move

    //this block is needed for calculating number of steps which is needed for accel/deccel
    float hL0 = sqrt( pow(x0, 2) + pow(y0, 2) );    //calculate beginning left hypotenuse
    float hR0 = sqrt( pow(canvasWidth - x0, 2) + pow(y0, 2) );    //calculate beginning right hypotenuse
    float hL1 = sqrt( pow(targetX, 2) + pow(targetY, 2) );        //calculate end left hypotenuse
    float hR1 = sqrt( pow(canvasWidth - targetX, 2) + pow(targetY, 2) );      //calculate end right hypotenuse
    float deltahL = hL1 - hL0;    //calculate the total new distance for the left motor to move
    float deltahR = hR1 - hR0;    //calculate the total new distance for the right motor to move
    float deltaAbsMax = max(abs(deltahL), abs(deltahR));    //the longest distance one motor needs to move to reach the final point
    totalLineSteps = deltaAbsMax / Ts;    //number of steps to pulse = total length to move / distance moved with one pulse
    traveledSteps = 0;
      
    float distanceMoved = 0.0;
    float stepSize = 10.0;    //max distance in mm the robot sends to the function moveToPosition
      //by breaking the total length in smaller steps the line is kept straight, instead of getting a curve
      //which happens when whole distance is sent to moveToPosition
    
    while(distanceMoved < Tl)   //while the robot has not moved the total length
    {
      if(distanceMoved > Tl - stepSize)   //exit case: if less than 10mm is left to move
        stepSize = Tl - distanceMoved;    //set the stepSize equal to whatever length less than 10mm is left to move
      
      float nextX = x0 + deltaX*(stepSize/Tl);     //sets an intermediate point on the road to move to the total distance
      float nextY = y0 + deltaY*(stepSize/Tl);     //sets an intermediate point on the road to move to the total distance
      moveToPosition(x0, y0, nextX, nextY);     //moves 10mm on the road to the total distance
      x0 = nextX;      //updates the start point for the next 10mm line
      y0 = nextY;      //updates the start point for the next 10mm line
      distanceMoved += stepSize;      //updates the length moved so far
    }

    currentX = targetX;
    currentY = targetY;
  }

}


void moveToPosition(float x0, float y0, float x1, float y1)
{ 
  //h = hypotenuse
  float hL0 = sqrt( pow(x0, 2) + pow(y0, 2) );    //calculate beginning left hypotenuse
  float hR0 = sqrt( pow(canvasWidth - x0, 2) + pow(y0, 2) );    //calculate beginning right hypotenuse
  float hL1 = sqrt( pow(x1, 2) + pow(y1, 2) );        //calculate end left hypotenuse
  float hR1 = sqrt( pow(canvasWidth - x1, 2) + pow(y1, 2) );      //calculate end right hypotenuse
  float deltahL = hL1 - hL0;    //calculate the total new distance for the left motor to move
  float deltahR = hR1 - hR0;    //calculate the total new distance for the right motor to move
 
  //this block determines the direction for the motors to spin
  //depending on if the new hypotenuse is larger or smaller than the original
  bool motorDirLong, motorDirShort, motorDirLeft, motorDirRight;
  if(deltahL >= 0 && deltahR >= 0)        //0 0
  {
    motorDirLeft = true;
    motorDirRight = true;
  }
  else if(deltahL >= 0 && deltahR < 0)    //0 1
  {
    motorDirLeft = true;
    motorDirRight = false;
  }
  else if(deltahL < 0 && deltahR >= 0)    //1 0
  {
    motorDirLeft = false;
    motorDirRight = true;
  }
  else    //(deltahL < 0 && deltahR < 0)  //1 1
  {
    motorDirLeft = false;
    motorDirRight = false;
  }

  bool leftIsLongest;   //used to flag which motor moves the longest distance
  if(abs(deltahL) >= abs(deltahR))
  {
    leftIsLongest = true;
    motorDirLong = motorDirLeft;
    motorDirShort = motorDirRight;
  }
  else 
  {
    leftIsLongest = false;
    motorDirLong = motorDirRight;
    motorDirShort = motorDirLeft;
  }

  float deltaAbsMin = min(abs(deltahL), abs(deltahR));    //the shortes distance one motor needs to move to reach the final point
  float deltaAbsMax = max(abs(deltahL), abs(deltahR));    //the longest distance one motor needs to move to reach the final point

  float nSteps = deltaAbsMax / Ts;    //number of steps to pulse = total length to move / distance moved with one pulse

  float otherMotorThreshold = 0;  //used to trigger when the motor with shortest distance needs to move
  float movementRatio = deltaAbsMin / deltaAbsMax;

  for (int i = 0; i < nSteps; i++)          //disse to måtene å kjøre for-løkke på gir nøyaktig samme resultat
  {
    handleAccel();

    pulseMotor(leftIsLongest, motorDirLong);    //moves the motor with the longest distance one step
    otherMotorThreshold += movementRatio;   //increments the threshold determining when the shortest distance motor needs to move

    if(otherMotorThreshold >= 1)
    {
      pulseMotor(!leftIsLongest, motorDirShort);    //moves the shortest distance motor one step
      otherMotorThreshold -= 1;
    }

  }
}

void servoPenDraw(bool draw)   //moves the servo in a controlled and delayed fashion to avoid overshoots
{
  int servoNewPos;    //the position the servo should move to
  int delayMS;  // = 12;   //16
  if(draw)
  {
    servoNewPos = servoPosDraw;
    // delayMS = 18;   //longer delay when the robot is about to draw to prevent swinging motion in the drawing
    delayMS = servoDrawDelay;
  }
  else
  {
    servoNewPos = servoPosNoDraw;
    delayMS = servoNoDrawDelay;
  }

  //increments or decrements the servo position until it's at the target position
  //this is done in a loop with a delay to keep the servo movement slow and controlled which prevents the robot head from swinging
  while(servoPosCurrent != servoNewPos)   //prevents the servo getting told to move to the position it's already in
  {
    if(servoNewPos > servoPosCurrent)
      servoPosCurrent++;
    else
      servoPosCurrent--;
    penServo.write(servoPosCurrent);
    delay(delayMS);
  }
  // delay(4*delayMS);
  delay(50);
}

void pulseMotor(bool leftMotor, bool moveDown)    //pulses one motor by one step 
{
  int stepPin, dirPin;
  //selects the proper motor pin and direction pin based on boolean input in function
  if(leftMotor)
  {
    stepPin = stepPinL;
    dirPin = dirPinL;
    moveDown = !moveDown;
  }
  else
  {
    stepPin = stepPinR;
    dirPin = dirPinR;
  }

  //sends pulse to selected motor
  digitalWrite(dirPin, moveDown);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(currentSpeedDelay);            
  digitalWrite(stepPin, LOW);
  delayMicroseconds(currentSpeedDelay);
  
}

bool tryExtractFloat(char coordinateAxis, float *valueOut)
{
  char *axisPosition = strchr(cmdBuffer, coordinateAxis);
  if(axisPosition == nullptr)
    return false;

  axisPosition++;
  while(*axisPosition == ' ')
    axisPosition++;

  char firstChar = *axisPosition;
  if(firstChar == '\0')
    return false;
  if((firstChar < '0' || firstChar > '9') && firstChar != '-' && firstChar != '+' && firstChar != '.')
    return false;

  *valueOut = atof(axisPosition);
  return true;
}

void resetCommandBuffer()
{
  cmdLength = 0;
  cmdBuffer[0] = '\0';
}

void G1xyz()
{
  float xVal = 0;
  float yVal = 0;
  float zVal = 0;
  bool hasX = tryExtractFloat('X', &xVal);
  bool hasY = tryExtractFloat('Y', &yVal);
  bool hasZ = tryExtractFloat('Z', &zVal);

  if(hasZ && (zVal == 1 || zVal == 0))    //if a z value was sent
  {
    if(hasX && hasY)    //if xy values also were sent
      interpolateToPosition(currentX, currentY, xVal, yVal, zVal == 0);    //act on xy coordinates and z value
    else
      servoPenDraw(zVal == 0);           //act on z value alone
  }
  else    //if no z value was sent
  {
    if(hasX && hasY)    //and proper xy values were sent
      interpolateToPosition(currentX, currentY, xVal, yVal);    //act on xy coordinates
    else if(hasX)
      interpolateToPosition(currentX, currentY, xVal, currentY);    //act on only x coordinate
    else if(hasY)
      interpolateToPosition(currentX, currentY, currentX, yVal);    //act on only y coordinate
  }

}

void G1lr()
{
  float lVal = 0;
  float rVal = 0;
  bool hasL = tryExtractFloat('L', &lVal);
  bool hasR = tryExtractFloat('R', &rVal);
  float travelDistance = 0;
  bool leftMotor;
  if(hasL)
  {
    travelDistance = abs(lVal);
    leftMotor = true;
  }
  else if(hasR)
  {
    travelDistance = abs(rVal);
    leftMotor = false;
  }
  else
    return;

  bool moveDown = true;
  if(strstr(cmdBuffer, "L-") != nullptr || strstr(cmdBuffer, "R-") != nullptr)
      moveDown = false;
  
  float nSteps = travelDistance / Ts;    //number of steps to pulse = total length to move / distance moved with one pulse
  for (int i = 0; i < nSteps; i++)  
  {
    handleAccel();
    pulseMotor(leftMotor, moveDown);    //moves the motor with the longest distance one step
  }
    

}

void printXYfromHypo(float hL, float hR)    //used for debugging
{
  float x = (pow(hR, 2) - pow(hL, 2) - pow(canvasWidth, 2)) / (-2.0 * canvasWidth);
  float y = sqrt(pow(hL, 2) - pow(x, 2));

  Serial.print(x);
  Serial.print(",");
  Serial.println(y);
}

float getXfromHypo(float hL, float hR)    //used for debugging
{
  return (pow(hR, 2) - pow(hL, 2) - pow(canvasWidth, 2)) / (-2.0 * canvasWidth);
}

float getYfromHypo(float hL, float hR)    //used for debugging
{
  float x = (pow(hR, 2) - pow(hL, 2) - pow(canvasWidth, 2)) / (-2.0 * canvasWidth);
  return sqrt(pow(hL, 2) - pow(x, 2));
}

