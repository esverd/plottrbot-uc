

#include <Arduino.h>
#include <Servo.h>
// #include <TMCStepper.h>
// #include <TMC2130.h>


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
// TMC2130Stepper leftStepperMotor(csPinL, r_sense);                           // Hardware SPI
// TMC2130Stepper rightStepperMotor(csPinR, r_sense);                           // Hardware SPI  

// AccelStepper leftStep = AccelStepper(leftStep.DRIVER, stepPinL, dirPinL);
// AccelStepper rightStep = AccelStepper(rightStep.DRIVER, stepPinR, dirPinR);


//-------CALIBRATION-------
unsigned int canvasWidth = 1460;    //width between center of the two motor axis. unit is mm
unsigned int canvasHeight = 1000;   //TODO bruke denne variabelen for å ikke gå utenfor maks høyde. brukes til å oppgi maks høyde med vekt på belte
float homeX = (canvasWidth / 2.0);
float homeY = 200.0;            //homing key neck (168mm) + center motor axle to center rail (32mm) = 200mm
//speed or delay in microSeconds

float diameterPulley = 12.723; //11.98;    //in mm  //var rundt 12.723 med gamle stepper drivers.- 11.98 med tmc2130
float Ts = diameterPulley*PI/3200.0;    //3200 the number of steps to complete full rotation of motor. micro stepping = 16

// float scaleX = 70/73.0;
// float scaleY = 1.0;

const int servoPosDraw = 150;     //servo position when the pen touches the canvas
const int servoPosNoDraw = 100;   //servo position when the pen doesn't touch the canvas
int servoPosCurrent = servoPosDraw;   //sets the current position to drawing to make sure the robot later boots by moving to noDrawPosition

const int SPEED_MICRO_S = 120;    //changes speed

float currentX = homeX;
float currentY = homeY;

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
void commandG1();


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
  

  // SPI.begin();                    // SPI drivers

  // leftStepperMotor.begin();                 //  SPI: Init CS pins and possible SW SPI pins
  // // leftStepperMotor.toff(5);                 // Enables driver in software
  // leftStepperMotor.rms_current(700);        // Set motor RMS current
  // leftStepperMotor.microsteps(16);          // Set microsteps to 1/16th
  // leftStepperMotor.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  // leftStepperMotor.pwm_autoscale(true);     // Needed for stealthChop


  // rightStepperMotor.begin();                 //  SPI: Init CS pins and possible SW SPI pins
  // // rightStepperMotor.toff(5);                 // Enables driver in software
  // rightStepperMotor.rms_current(700);        // Set motor RMS current
  // rightStepperMotor.microsteps(16);          // Set microsteps to 1/16th
  // rightStepperMotor.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160   skal egentlig være true for stealthchop
  // rightStepperMotor.pwm_autoscale(true);     // Needed for stealthChop


  // leftStepperMotor.blank_time(16);
  // rightStepperMotor.blank_time(16);
  // leftStepperMotor.ihold(0...31);    //konfigurere denne senere
  // rightStepperMotor.ihold(0...31);    //konfigurere denne senere
  // leftStep.setMaxSpeed(50*(1/Ts)); // 100mm/s @ 80 steps/mm
  // leftStep.setAcceleration(1000*(1/Ts)); // 2000mm/s^2
  // leftStep.setEnablePin(enablePinLR);
  // leftStep.setPinsInverted(false, false, true);
  // leftStep.enableOutputs();
  // rightStep.setMaxSpeed(50*(1/Ts)); // 100mm/s @ 80 steps/mm
  // rightStep.setAcceleration(1000*(1/Ts)); // 2000mm/s^2
  // rightStep.setEnablePin(enablePinLR);
  // rightStep.setPinsInverted(false, false, true);
  // rightStep.enableOutputs();


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
  //TODO change canvasSize. recalculate homeX. oppdatere størrelse i eeprom

  //eks: G1 X85.469 Y85.935
  //eks: G1 Z1

   if(cmdBuffer.indexOf("G1") != -1 || cmdBuffer.indexOf("G01") != -1)
   {
     commandG1();   //moves the robot to the given coordinates
   }
   else if(cmdBuffer.indexOf("G28") != -1)    
   {
      // currentX = homeX;  //sets the current position to the home position
      // currentY = homeY;
      interpolateToPosition(currentX, currentY, homeX, homeY);    //moves to home position
   }

  cmdBuffer = "";   //readies the buffer to receive a new command
}



void loop() 
{
  readSerial();
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
  if(x1 <= canvasWidth || y1 <= canvasHeight)   //if the new position is withing the robot coordinate system
  {

    currentX = x1;    //saves the new position. needs to happen before scaling
    currentY = y1;    //saves the new position. needs to happen before scaling

// G1 X1100 Y200
    // x0 *= sqrt(pow(x0 - homeX, 2))*scaleX;
    // x1 *= sqrt(pow(x1 - homeX, 2))*scaleX;
    // x0 *= scaleX;
    // x1 *= scaleX;
    // y0 *= scaleY;
    // y1 *= scaleY;

    float deltaX = x1 - x0;
    float deltaY = y1 - y0;
    // deltaX *= scaleX;
    // deltaY *= scaleY;

    float Tl = sqrt( pow(deltaX, 2) + pow(deltaY, 2) );   //total length to move
    // float theta = atan(deltaY / deltaX);    //angle between the two points
      
    float distanceMoved = 0.0;
    float stepSize = 10.0;    //max distance in mm the robot sends to the function moveToPosition
      //by breaking the total length in smaller steps the line is kept straight, instead of getting a curve
      //which happens when whole distance is sent to moveToPosition
    
    while(distanceMoved < Tl)   //while the robot has not moved the total length
    {
      if(distanceMoved > Tl - stepSize)   //exit case: if less than 10mm is left to move
        stepSize = Tl - distanceMoved;    //set the stepSize equal to whatever length less than 10mm is left to move

      // x1 = x0 + stepSize*cos(theta)*signX;
      // y1 = y0 + stepSize*sin(theta);    //trenger ikke forttegn på y
      
      x1 = x0 + deltaX*(stepSize/Tl);     //sets a x1 and y1 point on the road to move to the total distance
      y1 = y0 + deltaY*(stepSize/Tl);     //sets a x1 and y1 point on the road to move to the total distance
      moveToPosition(x0, y0, x1, y1);     //moves 10mm on the road to the total distance
      x0 = x1;      //updates the start point for the next 10mm line
      y0 = y1;      //updates the start point for the next 10mm line
      distanceMoved += stepSize;      //updates the length moved so far

      //distanceMoved = sqrt( pow(x1, 2) + pow(y1, 2) );
      // Serial.print("distanceMoved = ");
      // Serial.print(distanceMoved);
      // Serial.print("    Tl = ");
      // Serial.println(Tl);
    }
  }

}


void moveToPosition(float x0, float y0, float x1, float y1)
{
  //x0 *= scaleX;
  //y0 *= scaleY;
  //x1 *= scaleX;
  //y1 *= scaleY;
  
  //h = hypotenuse
  float hL0 = sqrt( pow(x0, 2) + pow(y0, 2) );    //calculate beginning left hypotenuse
  float hR0 = sqrt( pow(canvasWidth - x0, 2) + pow(y0, 2) );    //calculate beginning right hypotenuse
  float hL1 = sqrt( pow(x1, 2) + pow(y1, 2) );        //calculate end left hypotenuse
  float hR1 = sqrt( pow(canvasWidth - x1, 2) + pow(y1, 2) );      //calculate end right hypotenuse
  float deltahL = hL1 - hL0;    //calculate the total new distance for the left motor to move
  float deltahR = hR1 - hR0;    //calculate the total new distance for the right motor to move

  // float Tl = sqrt( pow(x1 - x0, 2) + pow(y1 - y0, 2) );   //total length to move
  // float nSteps = Tl / Ts;     //number of steps to pulse = total length to move / distance moved with one pulse
 
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
  // float deltaMin = min(deltahL, deltahR);
  // float deltaMax = max(deltahL, deltahR);

  float nSteps = deltaAbsMax / Ts;    //number of steps to pulse = total length to move / distance moved with one pulse

  float otherMotorThreshold = 0;  //used to trigger when the motor with shortest distance needs to move
  float movementRatio = deltaAbsMin / deltaAbsMax;

  for (int i = 0; i < nSteps; i++)          //disse to måtene å kjøre for-løkke på gir nøyaktig samme resultat
  {
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
  int delayMS;
  if(draw)
  {
    servoNewPos = servoPosDraw;
    delayMS = 14;   //longer delay when the robot is about to draw to prevent swinging motion in the drawing
  }
  else
  {
    servoNewPos = servoPosNoDraw;
    delayMS = 8;
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

}


// void pulseMotor(bool leftMotor, bool moveDown)    //pulses one motor by one step    TMC2130
// {
//   int stepPin;
//   //selects the proper motor pin and direction pin based on boolean input in function
//   if(leftMotor)
//   {
//     stepPin = stepPinL;
//     leftStepperMotor.shaft(!moveDown);
//   }
//   else
//   {
//     stepPin = stepPinR;
//     rightStepperMotor.shaft(moveDown);
//   }
//   //sends pulse to selected motor
//   digitalWrite(stepPin, HIGH);
//   delayMicroseconds(SPEED_MICRO_S);            
//   digitalWrite(stepPin, LOW);
//   delayMicroseconds(SPEED_MICRO_S);

//   // Run 5000 steps and switch direction in software
//   // for (uint16_t i = 5000; i>0; i--) {
//   //   digitalWrite(STEP_PIN, HIGH);
//   //   delayMicroseconds(160);
//   //   digitalWrite(STEP_PIN, LOW);
//   //   delayMicroseconds(160);
//   // }
//   // shaft = !shaft;
//   // driver.shaft(shaft);
// }

//lage pulseMotor som kun pulser ett steg
void pulseMotor(bool leftMotor, bool moveDown)    //pulses one motor by one step    //A4988
{
  int stepPin, dirPin;
  //selects the proper motor pin and direction pin based on boolean input in function
  if(leftMotor)
  {
    stepPin = stepPinL;
    dirPin = dirPinL;
    
  }
  else
  {
    stepPin = stepPinR;
    dirPin = dirPinR;
    moveDown = !moveDown;
  }

  //sends pulse to selected motor
  digitalWrite(dirPin, moveDown);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(SPEED_MICRO_S);            
  digitalWrite(stepPin, LOW);
  delayMicroseconds(SPEED_MICRO_S);
  
}

void commandG1()
{
  String xVal = "-1";
  String yVal = "-1";
  String zVal = "-1";

  char findChar = 'X';
  if(cmdBuffer.indexOf(findChar) != -1)   //extracts the X value from the incomming command
  {
    //substring(from, to)
    xVal = cmdBuffer.substring(cmdBuffer.indexOf(findChar)+1, (cmdBuffer.substring(cmdBuffer.indexOf(findChar)+1, cmdBuffer.indexOf(findChar)+2)).indexOf(' ') - cmdBuffer.indexOf(findChar)+1);
    xVal = xVal.substring(0, xVal.indexOf(' '));
  }
  
  findChar = 'Y';
  if(cmdBuffer.indexOf(findChar) != -1)   //extracts the Y value from the incomming command
  {
    yVal = cmdBuffer.substring(cmdBuffer.indexOf(findChar)+1, (cmdBuffer.substring(cmdBuffer.indexOf(findChar)+1, cmdBuffer.indexOf(findChar)+2)).indexOf(' '));
    yVal = yVal.substring(0, yVal.indexOf(' '));
  }
  
  findChar = 'Z';
  if(cmdBuffer.indexOf(findChar) != -1)   //extracts the Z value from the incomming command
  {
    zVal = cmdBuffer.substring(cmdBuffer.indexOf(findChar)+1, (cmdBuffer.substring(cmdBuffer.indexOf(findChar)+1, cmdBuffer.indexOf(findChar)+2)).indexOf(' '));
    zVal = zVal.substring(0, zVal.indexOf(' '));
  }


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

  // Serial.print("X value = ");
  // Serial.println(xVal.toFloat());
  // Serial.print("Y value = ");
  // Serial.println(yVal.toFloat());
  // Serial.print("Z value = ");
  // Serial.println(zVal.toFloat());
  // delay(1000);
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

