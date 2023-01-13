


#include <Servo.h>
#include <AFMotor.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#define LINE_BUFFER_LENGTH 512

/*
#############
# CONSTANTS #
#############
*/

// Stepper settings
const char STEP = MICROSTEP;   // Type of stepping
const int STEPS_PER_REV = 48;  // # of steps for one revolution

// Motor steps to move 1 mm
const int STEPS_PER_MM_X = 100;
const int STEPS_PER_MM_Y = 100;

const float CM_PER_SEG = 0.1f;

// Servo settings
const int PEN_UP_DEG = 90;     // Servo UP (deg)
const int PEN_DOWN_DEG = 77;   // Servo DOWN (deg)
const int PEN_SERVO_PIN = 10;  // Servo pin (PWM)

// Delays
const int LINE_DELAY = 50;
const int PEN_DELAY = 50;

// Drawing limits in mm
const int X_MIN = 0;
const int X_MAX = 40;
const int Y_MIN = 0;
const int Y_MAX = 40;
const int Z_MIN = 0;
const int Z_MAX = 1;

/*
###########
# GLOBALS #
###########
*/

// Servo object for the Z-axis
Servo penServo;

// Stepper objects for Adafruit Motor Shield V1
AF_Stepper stepperX(STEPS_PER_REV, 2);  // M3 M4
AF_Stepper stepperY(STEPS_PER_REV, 1);  // M1 M2

// Function wrappers for AccelStepper objects
void forwardStepX() {
  stepperX.onestep(FORWARD, STEP);
}

void backwardStepX() {
  stepperX.onestep(BACKWARD, STEP);
}

void forwardStepY() {
  stepperY.onestep(FORWARD, STEP);
}

void backwardStepY() {
  stepperY.onestep(BACKWARD, STEP);
}

// Wrap the AF_Stepper in AccelStepper
AccelStepper myStepperX(forwardStepX, backwardStepX);
AccelStepper myStepperY(forwardStepY, backwardStepY);

// MultiStepper object for controlling the steppers at the same time
MultiStepper steppers;

/*
############
# SETTINGS #
############
*/

// Direction for drawing circle arcs
bool clockwise_dir = true;

// Positioning mode: absolute or incremental
bool absolute_pos = true;
bool arc_absolute_pos = true;

// Motor speed
int max_speed_x = 2000;
int acceleration_x = 2000;
int max_speed_y = 2000;
int acceleration_y = 2000;

bool verbose = false;  // Set to true to get debug messages.

/*
####################
# STATUS VARIABLES #
####################
*/

// Current position of plothead
struct point {
  float x;
  float y;
  int z;
};

struct point plothead_pos;


void setup() {

  Serial.begin(9600);              // Set the BAUD rate
  penServo.attach(PEN_SERVO_PIN);  // Set up the servo motor
  penServo.write(PEN_UP_DEG);      // Lift the plothead
  plothead_pos.z = 1;

  delay(100);
  // Set the motor parameters
  myStepperX.setMaxSpeed(max_speed_x);
  myStepperX.setAcceleration(acceleration_x);
  myStepperX.setCurrentPosition(0);
  plothead_pos.x = 0;

  myStepperY.setMaxSpeed(max_speed_y);
  myStepperY.setAcceleration(acceleration_y);
  myStepperY.setCurrentPosition(0);
  plothead_pos.y = 0;
  // Add the stepper motors to the MultiStepper object
  steppers.addStepper(myStepperX);
  steppers.addStepper(myStepperY);

  printSettings();
}


void loop() {

  delay(100);
  char line[LINE_BUFFER_LENGTH];
  char c;
  int line_index = 0;
  bool comment = false, semicolon = false;

  while (1) {

    // Serial reception - Mostly from Grbl, added semicolon support
    while (Serial.available() > 0) {
      c = Serial.read();
      if ((c == '\n') || (c == '\r')) {  // End of line reached
        if (line_index > 0) {            // Line is complete. Then execute!
          line[line_index] = '\0';       // Terminate string
          if (verbose) {
            Serial.print("Received: ");
            Serial.println(line);
          }
          processIncomingLine(line, line_index);
          line_index = 0;
        } else {
          // Empty or comment line. Skip block.
        }
        comment = false;
        semicolon = false;
        Serial.println("ok");
      } else {
        if ((comment) || (semicolon)) {   // Throw away all comment characters
          if (c == ')') comment = false;  // End of comment. Resume line.
        } else {
          if (c <= ' ') {         // Throw away whitepace and control characters
          } else if (c == '/') {  // Block delete not supported. Ignore character.
          } else if (c == '(') {  // Enable comments flag and ignore all characters until ')' or EOL.
            comment = true;
          } else if (c == ';') {
            semicolon = true;
          } else if (line_index >= LINE_BUFFER_LENGTH - 1) {
            Serial.println("ERROR - lineBuffer overflow");
            comment = false;
            semicolon = false;
          } else if (c >= 'a' && c <= 'z') {  // Uppercase lowercase
            line[line_index++] = c - 'a' + 'A';
          } else {
            line[line_index++] = c;
          }
        }
      }
    }
  }
}

// Prints the current settings of the CNC
void printSettings() {

  Serial.println("CNC Plotter started!");
  Serial.println("RANGE");
  Serial.print("X [");
  Serial.print(X_MIN);
  Serial.print("mm, ");
  Serial.print(X_MAX);
  Serial.println("mm]");
  Serial.print("Y [");
  Serial.print(Y_MIN);
  Serial.print("mm, ");
  Serial.print(Y_MAX);
  Serial.println("mm]");

  // Serial.println("Positioning mode: " + pos_mode);
  // Serial.println("Arc positioning mode: " + arc_pos_mode);
  // Serial.println("Arc drawin direction: " + draw_dir);

  Serial.println("X AXIS:");
  Serial.print("Steps / mm: ");
  Serial.println(STEPS_PER_MM_X);
  Serial.print("Max speed: ");
  Serial.println(max_speed_x);
  Serial.print("Acceleration: ");
  Serial.println(acceleration_x);

  Serial.println("Y AXIS:");
  Serial.print("Steps / mm: ");
  Serial.println(STEPS_PER_MM_Y);
  Serial.print("Max speed: ");
  Serial.println(max_speed_y);
  Serial.print("Acceleration: ");
  Serial.println(acceleration_y);
}


void processIncomingLine(char* line, int charNB) {
  int currentIndex = 0;
  char buffer[64];  // Hope that 64 is enough for 1 parameter
  struct point newPos;
  newPos.x = 0.0;
  newPos.y = 0.0;

  long x, y;
  int z;
  // G01 X Y Z - Linear movement
  // G02 X Y I J - Circular movement with the center in (x,y) in CLOCKWISE direction
  // G03 - Same as G02 but COUNTER CLOCKWISE
  // G04 P150 - Wait 150ms
  // G28 - Return to home position
  // U - pen down (Z1)
  // D - pen up (Z-1)
  //  Discard anything with a (
  //  Discard any other command!

  while (currentIndex < charNB) {
    switch (line[currentIndex++]) {  // Select command, if any
      case 'U':
        pen_up();
        break;
      case 'D':
        pen_down();
        break;
      case 'G':
        buffer[0] = line[currentIndex++];
        buffer[1] = line[currentIndex++];
        buffer[2] = '\0';
        switch (atoi(buffer)) {  // Select G command
          case 0:
          case 1:
            {
              // /!\ Dirty - Suppose that X is before Y
              char* indexX = strchr(line + currentIndex, 'X');  // Get X/Y/Z position in the string (if any)
              char* indexY = strchr(line + currentIndex, 'Y');
              char* indexZ = strchr(line + currentIndex, 'Z');
              if (indexX > 0) {
                newPos.x = atof(indexX + 1);
                x = atol(indexX + 1);
              } else {
                x = plothead_pos.x;
                newPos.x = plothead_pos.x;
              }
              if (indexY > 0) {
                newPos.y = atof(indexY + 1);
                y = atol(indexY + 1);
              } else {
                y = plothead_pos.y;
                newPos.y = plothead_pos.y;
              }
              if (indexZ > 0) {
                newPos.z = atoi(indexZ + 1);
              } else {
                newPos.z = plothead_pos.z;
              }
              // If the Z coordinate is -1 then lower the pen
              if (newPos.z < 0) {
                pen_down();
              } else {
                pen_up();
              }
              G1(x, y);
              // Update the pen head position
              plothead_pos.x = newPos.x;
              plothead_pos.y = newPos.y;
              plothead_pos.z = newPos.z;
              break;
            }
          case 2:
            {
              float cx, cy;                                     // Coordinates of the circle's center
              char* indexX = strchr(line + currentIndex, 'X');  // Get X/Y/Z position in the string (if any)
              char* indexY = strchr(line + currentIndex, 'Y');
              char* indexZ = strchr(line + currentIndex, 'Z');
              char* indexI = strchr(line + currentIndex, 'I');
              char* indexJ = strchr(line + currentIndex, 'J');
              if (indexX > 0) {
                newPos.x = atof(indexX + 1);
              } else {
                newPos.x = plothead_pos.x;
              }
              if (indexY > 0) {
                newPos.y = atof(indexY + 1);
              } else {
                newPos.y = plothead_pos.y;
              }
              if (indexZ > 0) {
                newPos.z = atoi(indexZ + 1);
              } else {
                newPos.z = plothead_pos.z;
              }
              if (indexI > 0) {
                cx = atof(indexI + 1);
              } else {
                cx = 0;
              }
              if (indexJ > 0) {
                cy = atof(indexJ + 1);
              } else {
                cy = 0;
              }
              // If the Z coordinate is -1 then lower the pen
              if (newPos.z == -1) {
                pen_down();
              } else {
                pen_up();
              }
              clockwise_dir = true;
              drawArc(cx, cy, newPos.x, newPos.y);
              // Update the pen head position
              plothead_pos.x = newPos.x;
              plothead_pos.y = newPos.y;
              plothead_pos.z = newPos.z;
              break;
            }
          case 3:
            {
              float cx, cy;                                     // Coordinates of the circle's center
              char* indexX = strchr(line + currentIndex, 'X');  // Get X/Y/Z position in the string (if any)
              char* indexY = strchr(line + currentIndex, 'Y');
              char* indexZ = strchr(line + currentIndex, 'Z');
              char* indexI = strchr(line + currentIndex, 'I');
              char* indexJ = strchr(line + currentIndex, 'J');
              if (indexX > 0) {
                newPos.x = atof(indexX + 1);
              } else {
                newPos.x = plothead_pos.x;
              }
              if (indexY > 0) {
                newPos.y = atof(indexY + 1);
              } else {
                newPos.y = plothead_pos.y;
              }
              if (indexZ > 0) {
                newPos.z = atoi(indexZ + 1);
              } else {
                newPos.z = plothead_pos.z;
              }
              if (indexI > 0) {
                cx = atof(indexI + 1);
              } else {
                cx = 0;
              }
              if (indexJ > 0) {
                cy = atof(indexJ + 1);
              } else {
                cy = 0;
              }
              // If the Z coordinate is -1 then lower the pen
              if (newPos.z == -1) {
                pen_down();
              } else {
                pen_up();
              }
              clockwise_dir = false;
              drawArc(cx, cy, newPos.x, newPos.y);
              // Update the pen head position
              plothead_pos.x = newPos.x;
              plothead_pos.y = newPos.y;
              plothead_pos.z = newPos.z;
              break;
            }
        }
        break;
        // case 'M':
        //   break;
    }
  }
}

/*********************************************
 * Draw a line from current position to (x,y)
 *********************************************/

void G1(long x, long y) {

  long positions[2];
  // Bring instructions within limits
  if (x >= X_MAX) {
    x = X_MAX;
  }
  if (x <= X_MIN) {
    x = X_MIN;
  }
  if (y >= Y_MAX) {
    y = Y_MAX;
  }
  if (y <= Y_MIN) {
    y = Y_MIN;
  }
  // Translate the coordinates into steps
  positions[0] = x * STEPS_PER_MM_X;
  positions[1] = y * STEPS_PER_MM_Y;
  // Set the position accodring to the positioning mode
  if (absolute_pos == false) {
    // Set the origin to the current position and then move
    positions[0] += myStepperX.currentPosition();
    positions[1] += myStepperY.currentPosition();
  }
  // Move the motors
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  // Release the motors
  stepperX.release();
  stepperY.release();
  // Wait for next instruction
  delay(LINE_DELAY);
}

// This method assumes the limits have already been checked.
// This method assumes the start and end radius match (both points are same distance from center)
// This method assumes arcs are not >180 degrees (PI radians)
// This method assumes all movement is at a constant Z height
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
// (posx,posy) is the starting position
// line() is our existing command to draw a straight line using Bresenham's algorithm.

// Returns angle of dy/dx as a value from 0...2PI
float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if (a < 0) a = (PI * 2.0) + a;
  return a;
}

void drawArc(float cx, float cy, float x, float y) {

  // Bring instructions within limits
  if (x >= X_MAX) {
    x = X_MAX;
  }
  if (x <= X_MIN) {
    x = X_MIN;
  }
  if (y >= Y_MAX) {
    y = Y_MAX;
  }
  if (y <= Y_MIN) {
    y = Y_MIN;
  }

  // get radius
  long dx = plothead_pos.x - cx;
  long dy = plothead_pos.y - cy;
  float radius = sqrt(dx * dx + dy * dy);

  // find the sweep of the arc
  float angle1 = atan3(dy, dx);
  float angle2 = atan3(y - cy, x - cx);
  float sweep = angle2 - angle1;

  if (clockwise_dir && sweep < 0) {
    angle2 += 2 * PI;
  } else if (!clockwise_dir) {
    angle1 += 2 * PI;
  }

  sweep = angle2 - angle1;

  // get length of arc
  // float circumference = PI * 2.0 * radius;
  // float len = sweep * circumference / (PI * 2.0);
  // simplifies to
  float len = abs(sweep) * radius;

  int i = 0;
  int num_segments = floor(len / CM_PER_SEG);

  // declare variables outside of loops because compilers can be really dumb and inefficient some times.
  float nx, ny, nz, angle3, fraction;

  for (i = 0; i < num_segments; ++i) {
    // interpolate around the arc
    fraction = ((float)i) / ((float)num_segments);
    angle3 = (sweep * fraction) + angle1;

    // find the intermediate position
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    // make a line to that intermediate position
    G1(nx, ny);
  }

  // one last line hit the end
  G1(x, y);

  //  Delay before any next lines are submitted
  delay(LINE_DELAY);
  //  Update the current position
  plothead_pos.x = x;
  plothead_pos.y = y;
}

//  Raises pen
void pen_up() {
  penServo.write(PEN_UP_DEG);
  delay(PEN_DELAY);
  plothead_pos.z = Z_MAX;
  digitalWrite(15, LOW);
  digitalWrite(16, HIGH);
  if (verbose) {
    Serial.println("Pen up!");
  }
}
//  Lowers pen
void pen_down() {
  penServo.write(PEN_DOWN_DEG);
  delay(PEN_DELAY);
  plothead_pos.z = Z_MIN;
  digitalWrite(15, HIGH);
  digitalWrite(16, LOW);
  if (verbose) {
    Serial.println("Pen down.");
  }
}
