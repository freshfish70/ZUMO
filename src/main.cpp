#include <Arduino.h>

/*

                _______           _______  _______                                       
                / ___   )|\     /|(       )(  ___  )                                      
                \/   )  || )   ( || () () || (   ) |                                      
                    /   )| |   | || || || || |   | |                                      
                  /   / | |   | || |(_)| || |   | |                                      
                  /   /  | |   | || |   | || |   | |                                      
                /   (_/\| (___) || )   ( || (___) |                                      
                (_______/(_______)|/     \|(_______)                                      
                                                                          
 _______  _______           _______  _______  _______    _______  _______ 
(  ____ \(  ____ )|\     /|(  ____ )(  ____ )(  ____ \  / ___   )/ ___   )
| (    \/| (    )|| )   ( || (    )|| (    )|| (    \/  \/   )  |\/   )  |
| |      | (____)|| |   | || (____)|| (____)|| (__          /   )    /   )
| | ____ |     __)| |   | ||  _____)|  _____)|  __)       _/   /   _/   / 
| | \_  )| (\ (   | |   | || (      | (      | (         /   _/   /   _/  
| (___) || ) \ \__| (___) || )      | )      | (____/\  (   (__/\(   (__/\
(_______)|/   \__/(_______)|/       |/       (_______/  \_______/\_______/
                                                                          

                            CREATED BY
                  ********************************
                  *                              *
                  *   Per-Christian Henningsen   *
                  *   Linda Helen Sperre         *
                  *   Christoffer A Træen        *
                  *   Trym Vaaland               *
                  *                              *
                  ********************************

 
*/

/*
  Arduino Wire library
  Lets us use the Arduino I2C bus
*/
#include <Wire.h>
/*
  Zumo shield libraries
  includes everything we need to control the Zumo
*/
#include <ZumoShield.h>

const int DEBUG_REMOTE = true;
// Lightreflection read threshhold; higher is darker.
const int QTR_THRESHOLD = 1900; // time in microseconds

// SPEEDS 0(min) - 400(max)
const int REVERSE_SPEED = 400; // Wheel speed when reversing
const int TURN_SPEED = 350;    // Wheel speed when turning
const int FORWARD_SPEED = 300; // Wheel speed when driving forward
const int SEARCH_SPEED = 280;  // Wheel speed when searching
const int MAX_SPEED = 400;     // Max speed

// DURATIONS
const int REVERSE_DURATION = 400; // How long it should drive backwards, time in milliseconds DEF: 400
const int TURN_DURATION = 300;    // How many ms it shoukd spend to turn, time in milliseconds DEF : 300

// DISTANCES
const int MAX_CHARGE_DISTANCE = 45; // The maximum distance in CM to the enemy that we should allow for a charge attack

// DIGITAL OUTPUT
const int LED_PIN = 13; // SIGNAL LED

// ANALOG
const int FRONT_IR_SENSOR = A0;

// TIMERS
unsigned long frontSensorTimeout;
unsigned long searchSequenceTimeout;
unsigned long noDetectionTimeout;

// REFLECTION SENSOR ARRAY
const unsigned char NUMBER_OF_SENSORS = 2;                        // How many sensors we are using
const unsigned int SENSOR_READ_TIMEOUT = 2000;                    // How long the sensor should read before timing out
const int LEFT_SENSOR_PIN = 4;                                    // Left reflection sensor
const int RIGHT_SENSOR_PIN = 5;                                   // Right reflextion sensor
unsigned char sensorPins[] = {LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN}; // Sensor pins we are reading on
unsigned int sensor_values[NUMBER_OF_SENSORS];                    // Used to hold sensor readout values

// creats a ZumoReflectanceSensor array instance
ZumoReflectanceSensorArray sensors(sensorPins, NUMBER_OF_SENSORS, SENSOR_READ_TIMEOUT, QTR_NO_EMITTER_PIN);
ZumoBuzzer buzzer;              // creates a ZumoBuzzer object
ZumoMotors motors;              // creates a ZumoMotor object
Pushbutton button(ZUMO_BUTTON); // default Zumo Button - pin 12

const int LATCH_PIN = 6;
const int CLOCK_PIN = 2;
const int DATA_PIN = 11;

enum TURN_DIRECTION
{
  RANDOM_DIRECTION,
  LEFT_DIRECTION,
  RIGHT_DIRECTION
} turnDirection;

// Sensor trigger states
enum TRIGGERED_SENSOR
{
  NONE,
  LEFT_SENSOR,
  RIGHT_SENSOR
} triggeredSensor;

// STATES
enum STATES
{
  /*
    Start state when zumo is powered on
  */
  S_IDLE,
  /*
    Searching for another zumo
  */
  S_SEARCHING,
  /*
    When car is charging towards another zumo
  */
  S_CHARGING,
  /*
    When zumo has detected collison
  */
  S_COLLIDING,
  /*
    When Zumo is backing up
  */
  S_BACKING
} currentOperationState;
STATES lastOperationState; // Holds last operation state

// DECLARATIONS END HERE  DECLARATIONS END HERE  DECLARATIONS END HERE  DECLARATIONS END HERE  DECLARATIONS END HERE

/*
  Drive the Zumo forward at provided speed

  @Param forwardSpeed how fast the zumo should drive
*/
void driveForward(int forwardSpeed)
{
  motors.setSpeeds(forwardSpeed, forwardSpeed);
}

/*
  Drive the Zumo backwards at provided speed

  @Param reverseSpeed how fast the zumo should drive backwards
*/
void driveBackwards(int reverseSpeed)
{
  motors.setSpeeds(-reverseSpeed, -reverseSpeed);
}

/*
  Drive the Zumo backwards at provided speed

  @Param reverseSpeed how fast the zumo should drive backwards
*/
void driveLeftBackwards(int reverseSpeed)
{
  motors.setSpeeds(-reverseSpeed, -(reverseSpeed / 2));
}

/*
  Turn the Zumo left with a provided speed

  @Param turnSpeed how fast the zumo should turn 
*/
void turnLeft(int turnSpeed)
{
  turnDirection = LEFT_DIRECTION;
  motors.setSpeeds(-turnSpeed, turnSpeed);
}

/*
  Turn the Zumo right with a provided speed

  @Param turnSpeed how fast the zumo should turn 
*/
void turnRight(int turnSpeed)
{
  turnDirection = RIGHT_DIRECTION;
  motors.setSpeeds(turnSpeed, -turnSpeed);
}

/*
  Stop the motors by setting their speed to 0
*/
void stopMotors()
{
  motors.setSpeeds(0, 0);
}

/*
  Converts Sharp GP2Y0A41SK0F IR Sensor read data
  to distance in CM

  @Param sensorPin the Sharp Ir sensor connected pin
  @Return int distance in CM
*/
int getIRDistance(int sensorPin)
{
  float volts = analogRead(sensorPin) * 0.0048828125;
  int distance = 13 * pow(volts, -1);

  // Null is not a valid distance because it can not detect closer then 5cm
  // But readouts is not consistant and 0 sometimes occurs
  return distance != 0 ? distance : 1000;
}

/*
  Reads the reflections sensors and determine
  if the Zumo is on the edge
  if it is, set global triggeredSensor to the sensor that
  detected the edge

  @Return bool true if on edge else false
*/
bool isOnEdge()
{
  sensors.read(sensor_values);

  int leftSensorValue = sensor_values[0];
  int rightSensorValue = sensor_values[1];
  bool sensorIsTriggered = false;

  if (leftSensorValue < QTR_THRESHOLD)
  {
    triggeredSensor = LEFT_SENSOR;
    sensorIsTriggered = true;
  }
  else if (rightSensorValue < QTR_THRESHOLD)
  {
    triggeredSensor = RIGHT_SENSOR;
    sensorIsTriggered = true;
  }

  return sensorIsTriggered;
}

/*
  Checks which sensor triggered the edge check
  and runs a movement sequence based on which sensor triggered sensor
  and reset the triggered sensor
*/
void runEdgeEscapeSequence()
{
  driveBackwards(REVERSE_SPEED);
  delay(REVERSE_DURATION);
  switch (triggeredSensor)
  {
  case LEFT_SENSOR:
    turnRight(TURN_SPEED);
    break;
  case RIGHT_SENSOR:
    turnLeft(TURN_SPEED);
    break;
  }
  delay(TURN_DURATION);
  driveForward(FORWARD_SPEED);
  triggeredSensor = NONE;
}

/*
  Turns the Zumo rendomly right or left at speed set by SEARCH_SPEED
*/
void runSearchSequence(int speed, TURN_DIRECTION direction)
{
  int rnd = random(0, 2);
  Serial.println("SEARCH");
  switch (direction)
  {
  case LEFT_DIRECTION:
    turnLeft(speed);
    break;
  case RIGHT_DIRECTION:
    turnRight(speed);
    break;
  case RANDOM_DIRECTION:
    (rnd > 1) ? turnLeft(speed) : turnRight(speed);
    break;
  }
}

/*
  Check if we are on the edge
  and run the escape sequence if we are
*/
void edgeDetect()
{
  if (isOnEdge())
  {
    runEdgeEscapeSequence();
  }
}

/*
  Set the last operation state

  @Param lastState the state to set to lastOperationState
*/
void setLastOperationState(STATES lastState)
{
  lastOperationState = lastState;
}

/*
  Chane the opearation state to a new state

  @Param newState the new state to change to
*/
void changeOperationState(STATES newState)
{
  setLastOperationState(currentOperationState);
  currentOperationState = newState;
}

/*
  Check if last operation state is equal to current state

  @Return bool true if changed
*/
bool isOperationStateChanged()
{
  if (lastOperationState == currentOperationState)
  {
    return false;
  }
  return true;
}

/*
  Set a timers timeout time in milliseconds

  @Param *timer timer reference the timer to add time out on
  @Param time how much time to add
*/
void setTimeout(unsigned long *timer, float time)
{
  *timer = millis() + time;
}

/*
  Check if passed timer has timed out

  @Return bool true if expired
*/
bool timerTimedOut(unsigned long timer)
{
  bool isExpired = false;
  if (millis() > timer)
  {
    isExpired = true;
  }

  return isExpired;
}

/*
  Check if we have enemy in sight
  by the distance on the IR sensor

  @Return bool true if enemy is in sight
*/
bool enemyInSight()
{
  bool charge = false;

  int irDistance = getIRDistance(FRONT_IR_SENSOR);
  Serial.println(irDistance);
  if (irDistance <= MAX_CHARGE_DISTANCE)
  {
    charge = true;
  }

  return charge;
}

void lockTarget(int speed, TURN_DIRECTION wasTurningDirection)
{
  switch (wasTurningDirection)
  {
  case LEFT_DIRECTION:
    turnRight(speed);
    break;
  case RIGHT_DIRECTION:
    turnLeft(speed);
    break;
  }
}

/*
  Shift bits into the shift registers

  @Param dataPin the pin that writes the bit data
  @Param clockPin the pin that triggers the clock for pushing the bit into the shift register
  @Param birOrder the order of which the bits are written into the shift register
  @Param value the value you want to shift into the register. number of 2^x
*/
void shiftOut(int dataPin, int clockPin, int bitOrder, uint16_t value)
{
  uint8_t i;

  for (i = 0; i < 16; i++)
  {
    if (bitOrder == LSBFIRST)
      digitalWrite(dataPin, !!(value & (1 << i)));
    else
      digitalWrite(dataPin, !!(value & (1 << (15 - i))));

    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
  }
}

/*
  
*/
void shift(double power, bool single = false)
{
  int minus;
  (single == true) ? minus = 0 : minus = 1;

  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, pow(2, power) - minus);
  digitalWrite(LATCH_PIN, HIGH);
}

/*

*/
void waitForButtonAndCountDown()
{
  digitalWrite(LED_PIN, HIGH);
  button.waitForButton();
  digitalWrite(LED_PIN, LOW);
  for (int i = 0; i < 5; i++)
  {
    // shift(2 * (5 - i));
    delay(1000);
    buzzer.playNote(NOTE_C(5), 200, 20);
  }
  // shift(10);
  buzzer.playNote(NOTE_C(5), 1000, 20);
  changeOperationState(S_SEARCHING);
}

void readSerial()
{
  int incomingByte = 0;

  if (Serial.available() > 0)
  {

    incomingByte = Serial.read();
    switch (incomingByte)
    {
    case 114: // R
      changeOperationState(S_SEARCHING);
      break;
    case 113: // Q
      stopMotors();
      changeOperationState(S_IDLE);
      break;
    case 119: // W
      changeOperationState(S_IDLE);
      driveForward(150);
      break;
    case 97: // A
      changeOperationState(S_IDLE);
      turnLeft(150);
      break;
    case 115: // S
      changeOperationState(S_IDLE);
      driveBackwards(150);
      break;
    case 100: // D
      changeOperationState(S_IDLE);
      turnRight(150);
      break;
    }

    Serial.print("Button: ");
    Serial.println(incomingByte, DEC);
  }
}

////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);

  triggeredSensor = NONE;         // Set initial triggered sensor
  currentOperationState = S_IDLE; // Sets initial state
  pinMode(LED_PIN, HIGH);         // Turn signal LED on

  randomSeed(analogRead(0)); // Make sure our random seed is different each time we run

  pinMode(LATCH_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);

  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();
}

void loop()
{
  // loop_start_time = millis();
  // lsm303.readAcceleration(loop_start_time);

  if (button.isPressed())
  {
    changeOperationState(S_IDLE);
  }

  // HUSK Å TA DENNE AV NÅR VI SKAL TIL KAMP KONKURANSE!!!!!
  if (DEBUG_REMOTE)
  {
    readSerial();
  }

  /*
    STATE MACHINE
  */
  switch (currentOperationState)
  {
  case S_IDLE:
    if (button.isPressed())
    {
      stopMotors();
      button.waitForRelease();
      waitForButtonAndCountDown();
      changeOperationState(S_SEARCHING);
    }
    break;
  case S_SEARCHING:
    edgeDetect(); // CHECK IF ZUMO IS ON EDGE
    if (isOperationStateChanged())
    {
      setTimeout(&noDetectionTimeout, 2500);
      setTimeout(&searchSequenceTimeout, 0);
      setLastOperationState(S_SEARCHING);
    }

    if (timerTimedOut(frontSensorTimeout))
    {
      if (enemyInSight())
      {
        lockTarget(300, turnDirection);
        delay(60);
        driveForward(MAX_SPEED);
        changeOperationState(S_CHARGING);
      }
      setTimeout(&frontSensorTimeout, 40);
    }

    if (timerTimedOut(searchSequenceTimeout))
    {
      runSearchSequence(SEARCH_SPEED, RANDOM_DIRECTION);
      setTimeout(&searchSequenceTimeout, 1000);
    }

    if (timerTimedOut(noDetectionTimeout))
    {
      driveForward(300);
      delay(300);
      setTimeout(&noDetectionTimeout, 2500);
    }

    break;
  case S_CHARGING:
    edgeDetect(); // CHECK IF ZUMO IS ON EDGE
    // Serial.println("CHARGE");
    if (isOperationStateChanged())
    {
      setTimeout(&frontSensorTimeout, 300);
      driveForward(MAX_SPEED);
      setLastOperationState(S_CHARGING);
    }

    if (timerTimedOut(frontSensorTimeout))
    {
      if (!enemyInSight())
      {
        runSearchSequence(340, turnDirection);
        setTimeout(&searchSequenceTimeout, 1000);
        changeOperationState(S_SEARCHING);
      }
      setTimeout(&frontSensorTimeout, 50);
    }
  }
}