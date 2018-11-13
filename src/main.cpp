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

const int DEBUGGING = true;

// Lightreflection read threshhold; higher is darker.

// SPEEDS 0(min) - 400(max)
const int REVERSE_SPEED = 400; // Wheel speed when reversing : Default 400
const int TURN_SPEED = 350;    // Wheel speed when turning : Default 350
const int FORWARD_SPEED = 200; // Wheel speed when driving forward : Default 300
const int SEARCH_SPEED = 300;  // Wheel speed when searching : Defaulr 280
const int MAX_SPEED = 400;     // Max speed : Default 400

// DURATIONS
const int REVERSE_DURATION = 230; // How long it should drive backwards, time in milliseconds DEF: 400
const int TURN_DURATION = 280;    // How many ms it shoukd spend to turn, time in milliseconds DEF : 300
const int COUNTDOWN_TIME = 1;     // SECONDS

// DISTANCES

// 45 CM
const int MAX_CHARGE_DISTANCE = 45; // The maximum distance in CM to the enemy that we should allow for a charge attack

// DIGITAL OUTPUT
const int LED_PIN = 13; // SIGNAL LED

// ANALOG
const int FRONT_IR_SENSOR = A0;

// TIMERS
unsigned long frontSensorTimeout;
unsigned long searchSequenceTimeout;
unsigned long noDetectionTimeout;
unsigned long movingTimeout;

// REFLECTION SENSOR ARRAY
const int QTR_THRESHOLD = 1700;                                   // time in microseconds
const unsigned char NUMBER_OF_SENSORS = 2;                        // How many sensors we are using
const unsigned int SENSOR_READ_TIMEOUT = 2000;                    // How long the sensor should read before timing out
const int LEFT_SENSOR_PIN = 4;                                    // Left reflection sensor
const int RIGHT_SENSOR_PIN = 5;                                   // Right reflextion sensor
unsigned char sensorPins[] = {LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN}; // Sensor pins we are reading on
unsigned int sensor_values[NUMBER_OF_SENSORS];                    // Used to hold sensor readout values

const int ULTRASONIC_ECHO_PIN = 6;    // connects to the echo pin on the distance sensor
const int ULTRASONIC_TRIGGER_PIN = 2; // connects to the trigger pin on the distance sensor

// creats a ZumoReflectanceSensor array instance
ZumoReflectanceSensorArray sensors(sensorPins, NUMBER_OF_SENSORS, SENSOR_READ_TIMEOUT, QTR_NO_EMITTER_PIN);
ZumoBuzzer buzzer;              // creates a ZumoBuzzer object
ZumoMotors motors;              // creates a ZumoMotor object
Pushbutton button(ZUMO_BUTTON); // default Zumo Button - pin 12

/*
  Border colors
*/
enum BORDER_COLOR
{
  WHITE_BORDER,
  BLACK_BORDER
};
const int BORDER_DETECT_COLOR = WHITE_BORDER;

/*
  Directions which the Sumo can turn
  used to control turning
  and keep track of which direction it is currently tuning or 
  was turning.

  creates turningDirection
*/
enum TURN_DIRECTION
{
  RANDOM_DIRECTION,
  LEFT_DIRECTION,
  RIGHT_DIRECTION
} turningDirection;

/*
  Triggered sensors
  
  creates triggeredReflectSensor
*/
enum TRIGGERED_SENSOR
{
  NONE,
  LEFT_SENSOR,
  RIGHT_SENSOR
} triggeredReflectSensor;

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
    When zumo moves forward
  */
  S_DRIVING,
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
  Turn the Zumo left with a provided speed

  @Param turnSpeed how fast the zumo should turn 
*/
void turnLeft(int turnSpeed)
{
  turningDirection = LEFT_DIRECTION;
  motors.setSpeeds(-turnSpeed, turnSpeed);
}

/*
  Turn the Zumo right with a provided speed

  @Param turnSpeed how fast the zumo should turn 
*/
void turnRight(int turnSpeed)
{
  turningDirection = RIGHT_DIRECTION;
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
  Set the triggered reflectance sensor 

  @Param sensor that is triggered
*/
void setTriggeredReflectanceSensor(TRIGGERED_SENSOR sensor)
{
  triggeredReflectSensor = sensor;
}

/*
  Messures distance to object from HC-SR04 DISTANCE SENSOR in CM

  @Param sensorTriggerPin the trigger pin on the sensor
  @Param sensorEchoPin the echo pin on the sensor
  @Retrun float messured distance in cm
*/
float getUltrasonicMessuredDistance(int sensorTriggerPin, int sensorEchoPin)
{
  float echoTime;           //variable to store the time it takes for a ping to bounce off an object
  float calcualtedDistance; //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(sensorTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorTriggerPin, LOW);

  echoTime = pulseIn(sensorEchoPin, HIGH); //use the pulsein command to see how long it takes for the
                                           //pulse to bounce back to the sensor which is a time in microseconds

  calcualtedDistance = echoTime * 0.01715; //calculate the distance of the object that reflected the pulse (half the speed of sound converted to CM/microseconds) to get
  return calcualtedDistance;               //send back the distance that was calculated
}

/*
  Reads the reflections sensors and determine
  if the Zumo is on the edge
  if it is, set global triggeredReflectSensor to the sensor that
  detected the edge

  @Return bool true if on edge else false
*/
bool isOnEdge()
{
  sensors.read(sensor_values);

  int leftSensorValue = sensor_values[0];
  int rightSensorValue = sensor_values[1];
  bool sensorIsTriggered = false;
  TRIGGERED_SENSOR sensor = NONE;

  switch (BORDER_DETECT_COLOR)
  {
  case WHITE_BORDER:
    if (leftSensorValue < QTR_THRESHOLD)
    {
      sensor = LEFT_SENSOR;
    }
    else if (rightSensorValue < QTR_THRESHOLD)
    {
      sensor = RIGHT_SENSOR;
    }
    break;
  case BLACK_BORDER:
    if (leftSensorValue > QTR_THRESHOLD)
    {
      sensor = LEFT_SENSOR;
    }
    else if (rightSensorValue > QTR_THRESHOLD)
    {
      sensor = RIGHT_SENSOR;
    }
    break;
  }
  if (sensor != NONE)
  {
    sensorIsTriggered = true;
    setTriggeredReflectanceSensor(sensor);
  }

  return sensorIsTriggered;
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
void setOperationState(STATES newState)
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
  Turns the Zumo in given direction at givin speed

  @Param speed the speed to turn in
  @Param direction the direction to turn in
*/
void turn(int speed, TURN_DIRECTION direction)
{

  switch (direction)
  {
  case LEFT_DIRECTION:
    turnLeft(speed);
    break;
  case RIGHT_DIRECTION:
    turnRight(speed);
    break;
  case RANDOM_DIRECTION:
  {
    int rnd = random(0, 10);
    (rnd > 5) ? turnLeft(speed) : turnRight(speed);
    break;
  }
  }
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
  switch (triggeredReflectSensor)
  {
  case LEFT_SENSOR:
    turn(TURN_SPEED, RIGHT_DIRECTION);
    break;
  case RIGHT_SENSOR:
    turn(TURN_SPEED, LEFT_DIRECTION);
    break;
  }
  delay(TURN_DURATION);
  driveForward(MAX_SPEED);
  setOperationState(S_DRIVING);
  setTimeout(&movingTimeout, 350);
  triggeredReflectSensor = NONE;
}

/*
  Check if we are on the edge
  and run the escape sequence if we are
*/
void borderDetect()
{
  if (isOnEdge())
  {
    runEdgeEscapeSequence();
  }
}

/*
  Check if we have enemy in sight
  by getting the distance on the IR sensor
  and compare it to a limit

  @Return bool true if enemy is in sight
*/
bool IsEnemyInSight()
{
  bool charge = false;

  int distance = getUltrasonicMessuredDistance(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);

  Serial.println(distance);
  if (distance <= MAX_CHARGE_DISTANCE)
  {
    charge = true;
  }

  return charge;
}

/*
  Tries too lock the target by reversing the search spin direction for a very
  short period, to compansate for "sliding" when zumo stops turning at high speed before it charges.

  @Param speed the speed on turn
  @Param wasTurningDirection the direction the zumo was turning in when detected enemy
  @Param turnDuration how long it should execute the reversed turn
*/
void lockTarget(int speed, TURN_DIRECTION wasTurningDirection, int turnDuration = 60)
{
  switch (wasTurningDirection)
  {
  case LEFT_DIRECTION:
    turn(speed, RIGHT_DIRECTION);
    break;
  case RIGHT_DIRECTION:
    turn(speed, LEFT_DIRECTION);
    // turnLeft(speed);
    break;
  case RANDOM_DIRECTION:
    turnDuration = 0;
    break;
  }
  delay(turnDuration);
}

/*
  Waits for start button press and
  then runs a count down sequence

  @Param seconds how many seconds we should count down for
*/
void waitForStartButtonAndCountDown(int seconds)
{
  digitalWrite(LED_PIN, HIGH);
  button.waitForButton();
  digitalWrite(LED_PIN, LOW);
  for (int i = 0; i < seconds; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_C(5), 200, 20);
  }
  // buzzer.playNote(NOTE_C(5), 1000, 20);
}

/*
  Reads serial for incomming keyboard commands
  Used when debugging
*/
void readSerial()
{
  int incomingByte = 0;

  if (Serial.available() > 0)
  {

    incomingByte = Serial.read();
    switch (incomingByte)
    {
    case 114: // R
      setOperationState(S_SEARCHING);
      break;
    case 113: // Q
      stopMotors();
      setOperationState(S_IDLE);
      break;
    case 119: // W
      setOperationState(S_IDLE);
      driveForward(150);
      break;
    case 97: // A
      setOperationState(S_IDLE);
      turnLeft(150);
      break;
    case 115: // S
      setOperationState(S_IDLE);
      driveBackwards(150);
      break;
    case 100: // D
      setOperationState(S_IDLE);
      turnRight(150);
      break;
    }

    Serial.print("Button: ");
    Serial.println(incomingByte, DEC);
  }
}

///////////////////  LOOP & SETUP  /////////////////////////

void setup()
{
  Serial.begin(9600);

  triggeredReflectSensor = NONE;  // Set initial triggered sensor
  currentOperationState = S_IDLE; // Sets initial state

  pinMode(LED_PIN, OUTPUT);

  randomSeed(analogRead(0)); // Make sure our random seed is different each time we run

  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
}

void loop()
{

  bool operationStateChanged = isOperationStateChanged();

  /*

  */
  if (currentOperationState != S_IDLE)
  {
    borderDetect();
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
      waitForStartButtonAndCountDown(COUNTDOWN_TIME);
      setOperationState(S_SEARCHING);
    }
    break;
  case S_SEARCHING:

    /*
      Set start values for this state
    */
    if (operationStateChanged)
    {
      setTimeout(&noDetectionTimeout, random(2200, 2700));
      setTimeout(&searchSequenceTimeout, 0);
      setLastOperationState(S_SEARCHING);
    }

    if (timerTimedOut(frontSensorTimeout))
    {
      if (IsEnemyInSight())
      {
        // lockTarget(300, turningDirection, 40);
        driveForward(MAX_SPEED);
        setOperationState(S_CHARGING);
      }
      setTimeout(&frontSensorTimeout, 30);
    }

    if (timerTimedOut(searchSequenceTimeout))
    {
      turn(SEARCH_SPEED, RANDOM_DIRECTION);
      setTimeout(&searchSequenceTimeout, 1000);
    }

    if (timerTimedOut(noDetectionTimeout))
    {
      setOperationState(S_DRIVING);
      driveForward(300);
      setTimeout(&movingTimeout, 300);
    }

    break;
  case S_DRIVING:

    if (timerTimedOut(movingTimeout))
    {
      setOperationState(S_SEARCHING);
    }

    if (timerTimedOut(frontSensorTimeout))
    {
      if (IsEnemyInSight())
      {
        // lockTarget(300, turningDirection, 40);
        driveForward(MAX_SPEED);
        setOperationState(S_CHARGING);
      }
      setTimeout(&frontSensorTimeout, 40);
    }
    break;
  case S_CHARGING:

    /*
      Set start values for this state
    */
    if (operationStateChanged)
    {
      setTimeout(&frontSensorTimeout, 300);
      driveForward(MAX_SPEED);
      setLastOperationState(S_CHARGING);
    }

    if (timerTimedOut(frontSensorTimeout))
    {
      if (!IsEnemyInSight())
      {
        turn(340, turningDirection);
        setTimeout(&searchSequenceTimeout, 1000);
        setOperationState(S_SEARCHING);
      }
      setTimeout(&frontSensorTimeout, 50);
    }
    break;
  }

  if (button.isPressed())
  {
    setOperationState(S_IDLE);
  }

  // HUSK Å TA DENNE AV PÅ TOPPEN NÅR VI SKAL TIL KAMP / KONKURANSE!!!!!
  if (DEBUGGING)
  {
    readSerial();
  }
}