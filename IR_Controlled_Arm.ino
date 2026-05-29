#include <IRremote.h>
#include <Stepper.h>
#include <Servo.h>

// --- Configuration ---
const int stepsPerRevolution = 2048;
const int rolePerMinute = 15;
#define IR_RECEIVE_PIN 12
#define SERVO1_PIN 2   // Shoulder servo
#define SERVO2_PIN 3   // Elbow servo
#define SERVO3_PIN 4   // Grabber servo

// IR Remote Button Codes
#define IR_BUTTON_REV  68  // Stepper reverse
#define IR_BUTTON_FF   67  // Stepper forward
#define IR_BUTTON_VOLU 70  // Servo 1 up
#define IR_BUTTON_VOLD 21  // Servo 1 down
#define IR_BUTTON_UP   9   // Servo 2 up (elbow)
#define IR_BUTTON_DOWN 7   // Servo 2 down (elbow)
#define IR_BUTTON_PLAY 64  // Toggle grabber open/closed

// Precision Constants
#define STEPS_PER_TICK   5
#define SERVO_SPEED      2
#define SERVO_UPDATE_MS  15
#define HOLD_TIMEOUT_MS  110

// Grabber positions
#define GRABBER_OPEN   0
#define GRABBER_CLOSED 45

// --- Objects ---
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);
Servo servo1;  // Shoulder
Servo servo2;  // Elbow
Servo servo3;  // Grabber

// --- State ---
const int motorPins[] = {8, 10, 9, 11};

// Stepper
int motorDirection = 0;
bool motorRunning = false;

// Servo 1 (shoulder)
int servo1Direction = 0;
int servo1Pos = 90;
bool servo1Running = false;

// Servo 2 (elbow) — mirrors servo 1 pattern
int servo2Direction = 0;
int servo2Pos = 180;
bool servo2Running = false;

// Servo 3 (grabber) — toggle only, no hold state needed
bool grabberOpen = true;

// Shared timing
unsigned long lastSignalTime = 0;
unsigned long lastServoUpdate = 0;

// --- Helpers ---
void stopAll() 
{
  for (int i = 0; i < 4; i++) digitalWrite(motorPins[i], LOW);
  motorDirection = 0;   motorRunning = false;
  servo1Direction = 0;  servo1Running = false;
  servo2Direction = 0;  servo2Running = false;
  // Grabber state intentionally preserved — it's toggle-based, not hold-based
}

void setup() 
{
  Serial.begin(9600);
  myStepper.setSpeed(rolePerMinute);
  for (int i = 0; i < 4; i++) pinMode(motorPins[i], OUTPUT);

  servo1.attach(SERVO1_PIN);
  servo1.write(servo1Pos);

  servo2.attach(SERVO2_PIN);
  servo2.write(servo2Pos);

  servo3.attach(SERVO3_PIN);
  servo3.write(GRABBER_OPEN);  // Start open

  IrReceiver.begin(IR_RECEIVE_PIN);
}

void loop() 
{
  // 1. IR Detection
  if (IrReceiver.decode()) 
  {
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) 
    {
      // Held button — keep the timeout window alive
      // Grabber toggle deliberately excluded: repeats should not re-toggle it
      lastSignalTime = millis();
    } 
    else 
    {
      int command = IrReceiver.decodedIRData.command;
      if (command != 0) 
      {
        lastSignalTime = millis();
        switch (command) 
        {

          case IR_BUTTON_FF:
            motorDirection = 1;    motorRunning = true;
            servo1Running = false; servo1Direction = 0;
            servo2Running = false; servo2Direction = 0;
            break;

          case IR_BUTTON_REV:
            motorDirection = -1;   motorRunning = true;
            servo1Running = false; servo1Direction = 0;
            servo2Running = false; servo2Direction = 0;
            break;

          case IR_BUTTON_VOLD:
            servo1Direction = 1;   servo1Running = true;
            motorRunning = false;  motorDirection = 0;
            servo2Running = false; servo2Direction = 0;
            break;

          case IR_BUTTON_VOLU:
            servo1Direction = -1;  servo1Running = true;
            motorRunning = false;  motorDirection = 0;
            servo2Running = false; servo2Direction = 0;
            break;

          case IR_BUTTON_DOWN:
            servo2Direction = 1;   servo2Running = true;
            motorRunning = false;  motorDirection = 0;
            servo1Running = false; servo1Direction = 0;
            break;

          case IR_BUTTON_UP:
            servo2Direction = -1;  servo2Running = true;
            motorRunning = false;  motorDirection = 0;
            servo1Running = false; servo1Direction = 0;
            break;

          case IR_BUTTON_PLAY:
            // Toggle grabber on each fresh press; repeats are ignored above
            grabberOpen = !grabberOpen;
            servo3.write(grabberOpen ? GRABBER_OPEN : GRABBER_CLOSED);
            break;
        }
      }
    }
    IrReceiver.resume();
  }

  // 2. Release Detection — stop hold-based motors/servos if signals stop
  if ((motorRunning || servo1Running || servo2Running) &&
      (millis() - lastSignalTime > HOLD_TIMEOUT_MS)) {
    stopAll();
  }

  // 3. Execution
  if (motorRunning) 
  {
    myStepper.step(motorDirection * STEPS_PER_TICK);
  }

  if ((servo1Running || servo2Running) && (millis() - lastServoUpdate >= SERVO_UPDATE_MS)) {
    if (servo1Running) 
    {
      int nextPos = constrain(servo1Pos + (servo1Direction * SERVO_SPEED), 0, 180);
      if (nextPos != servo1Pos) { servo1Pos = nextPos; servo1.write(servo1Pos); }
    }
    if (servo2Running) 
    {
      int nextPos = constrain(servo2Pos + (servo2Direction * SERVO_SPEED), 0, 180);
      if (nextPos != servo2Pos) { servo2Pos = nextPos; servo2.write(servo2Pos); }
    }
    lastServoUpdate = millis();
  }
}