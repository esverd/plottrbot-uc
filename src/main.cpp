

#include <Arduino.h>
#include <Servo.h>
#include <TMCStepper.h>   //needs to be my fork which has some important changes for stepper motor driving


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
unsigned int canvasWidth = 1460;    //width between center of the two motor axis. unit is mm
unsigned int canvasHeight = 1000;   //TODO bruke denne variabelen for å ikke gå utenfor maks høyde. brukes til å oppgi maks høyde med vekt på belte
float homeX = (canvasWidth / 2.0);
float homeY = 200.0;            //homing key neck (168mm) + center motor axle to center rail (32mm) = 200mm

float scaleTotalDistance = (73.0/70)*(54.3/55)*(55.0/57)*(55/55.5);
float diameterPulley = 12.2; //12.723; //11.98;    //in mm  //var rundt 12.723 med gamle stepper drivers.- 11.98 med tmc2130
float Ts = (diameterPulley*PI)/(3200.0*scaleTotalDistance);    //3200 the number of steps to complete full rotation of motor. micro stepping = 16

const int servoPosDraw = 100;     //servo position when the pen touches the canvas
const int servoPosNoDraw = 150;   //servo position when the pen doesn't touch the canvas
int servoPosCurrent = servoPosDraw;   //sets the current position to drawing to make sure the robot later boots by moving to noDrawPosition

float currentX = homeX;
float currentY = homeY;

//-------SPEED SETTINGS-------
const int DEFAULT_SPEED_DELAY = 90;   //100;    
const int SLOWEST_SPEED_DELAY = 240;  //280;    
float currentSpeedDelay = DEFAULT_SPEED_DELAY;
float totalLineSteps = 0;      //used to store the toal motor pulses to move a line. necessary for accel and deccel
float traveledSteps = 0.0;
const int STEPS_TO_ACCEL_DECCEL = 140;  //160;   
// const int MM_TO_ACCEL_DECCEL = 20;
// float totalMMtoTravel = 0.0;
int accelMode = 0;      //0 = plain. 1 = accelerate. -1 = deccelerate

//-------SERIAL COMMUNICATION-------
int incomingByte = 0; // for incoming serial data
String cmdBuffer = ""; 

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
String exctractCoordFromString(String, char);

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

  if(Serial.available() > 0)    //if serial communication is avalable
  {
    char inChar = Serial.read();    //reads next character

    if(inChar == '\n')    //if string ended with new line process command
    {
      handleGCODE();    //processes the received commans
      Serial.println("GO");   //prints GO to signal the arduino is ready for the next command
    }
    else
      cmdBuffer += inChar;    //stores the incomming character string
  }

}

void handleGCODE()
{
  //eks: G1 X85.469 Y85.935
  //eks: G1 Z1

  if(cmdBuffer.indexOf("G1") != -1 || cmdBuffer.indexOf("G01") != -1)
  {
    if(cmdBuffer.indexOf("X") != -1 || cmdBuffer.indexOf("Y") != -1 || cmdBuffer.indexOf("Z") != -1)
      G1xyz();   //moves the robot to the given coordinates
    else if(cmdBuffer.indexOf("L") != -1 || cmdBuffer.indexOf("R") != -1)
      G1lr();
  }
  else if(cmdBuffer.indexOf("G28") != -1)    
  {
    // currentX = homeX;  //sets the current position to the home position
    // currentY = homeY;
    interpolateToPosition(currentX, currentY, homeX, homeY);    //moves to home position
  }
  else if(cmdBuffer.indexOf("M17") != -1)   
  digitalWrite(enablePinLR, LOW);       //enables power to stepper motors
  else if(cmdBuffer.indexOf("M18") != -1)   
    digitalWrite(enablePinLR, HIGH);    //disable power to stepper motors
  else if(cmdBuffer.indexOf("G92") != -1)   //sets the current coordinates without moving motors
  {
    if(cmdBuffer.indexOf("G92 H") != -1)
    {
      currentX = homeX;
      currentY = homeY;
    }
    else
    {
      String xVal = exctractCoordFromString(cmdBuffer, 'X');
      String yVal = exctractCoordFromString(cmdBuffer, 'Y');
      if(xVal.toFloat() != -1)    //if proper values were sent
        currentX = xVal.toFloat();  //set current coordinates to sent coordinates
      if(yVal.toFloat() != -1)
        currentY = yVal.toFloat();
    }
  }

  cmdBuffer = "";   //readies the buffer to receive a new command
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
  if(x1 <= canvasWidth || y1 <= canvasHeight)   //if the new position is withing the robot bounds
  {
    digitalWrite(enablePinLR, LOW);      //enables power to stepper motors
    currentX = x1;    //saves the new position. needs to happen before scaling
    currentY = y1;    //saves the new position. needs to happen before scaling

    float deltaX = x1 - x0;
    float deltaY = y1 - y0;
    float Tl = sqrt( pow(deltaX, 2) + pow(deltaY, 2) );   //total length to move

    //this block is needed for calculating number of steps which is needed for accel/deccel
    float hL0 = sqrt( pow(x0, 2) + pow(y0, 2) );    //calculate beginning left hypotenuse
    float hR0 = sqrt( pow(canvasWidth - x0, 2) + pow(y0, 2) );    //calculate beginning right hypotenuse
    float hL1 = sqrt( pow(x1, 2) + pow(y1, 2) );        //calculate end left hypotenuse
    float hR1 = sqrt( pow(canvasWidth - x1, 2) + pow(y1, 2) );      //calculate end right hypotenuse
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
      
      x1 = x0 + deltaX*(stepSize/Tl);     //sets a x1 and y1 point on the road to move to the total distance
      y1 = y0 + deltaY*(stepSize/Tl);     //sets a x1 and y1 point on the road to move to the total distance
      moveToPosition(x0, y0, x1, y1);     //moves 10mm on the road to the total distance
      x0 = x1;      //updates the start point for the next 10mm line
      y0 = y1;      //updates the start point for the next 10mm line
      distanceMoved += stepSize;      //updates the length moved so far
    }
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
    delayMS = 18;   //longer delay when the robot is about to draw to prevent swinging motion in the drawing
  }
  else
  {
    servoNewPos = servoPosNoDraw;
    delayMS = 10;
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

String exctractCoordFromString(String coordinateString, char coordinateAxis)
{
  String foundCoord = "-1";
  char findChar = coordinateAxis;
  if(coordinateString.indexOf(findChar) != -1)   //extracts the X value from the incomming command
  {
    //substring(from, to)
    foundCoord = coordinateString.substring(coordinateString.indexOf(findChar)+1, (coordinateString.substring(coordinateString.indexOf(findChar)+1, coordinateString.indexOf(findChar)+2)).indexOf(' ') - coordinateString.indexOf(findChar)+1);
    foundCoord = foundCoord.substring(0, foundCoord.indexOf(' '));
  }
  return foundCoord;
}

void G1xyz()
{
  String xVal = exctractCoordFromString(cmdBuffer, 'X');
  String yVal = exctractCoordFromString(cmdBuffer, 'Y');
  String zVal = exctractCoordFromString(cmdBuffer, 'Z');

  if(zVal.toInt() == 1 || zVal.toInt() == 0)    //if a z value was sent
  {
    if(xVal.toFloat() != -1 && yVal.toFloat() != -1)    //if xy values also were sent
      interpolateToPosition(currentX, currentY, xVal.toFloat(), yVal.toFloat(), !zVal.toInt());    //act on xy coordinates and z value
    else
      servoPenDraw(!zVal.toInt());           //act on z value alone
  }
  else    //if no z value was sent
  {
    if(xVal.toFloat() != -1 && yVal.toFloat() != -1)    //and proper xy values were sent
      interpolateToPosition(currentX, currentY, xVal.toFloat(), yVal.toFloat());    //act on xy coordinates
    else if(xVal.toFloat() != -1)
      interpolateToPosition(currentX, currentY, xVal.toFloat(), currentY);    //act on only x coordinate
    else if(yVal.toFloat() != -1)
      interpolateToPosition(currentX, currentY, currentX, yVal.toFloat());    //act on only y coordinate
  }

}

void G1lr()
{
  String lVal = exctractCoordFromString(cmdBuffer, 'L');
  String rVal = exctractCoordFromString(cmdBuffer, 'R');

  float travelDistance = 0;
  bool leftMotor;
  if(lVal.toFloat() != -1)
  {
    travelDistance = abs(lVal.toFloat());
    leftMotor = true;
  }
  else if(rVal.toFloat() != -1)
  {
    travelDistance = abs(rVal.toFloat());
    leftMotor = false;
  }

  bool moveDown = true;
  if(cmdBuffer.indexOf("L-") != -1 || cmdBuffer.indexOf("R-") != -1)
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

