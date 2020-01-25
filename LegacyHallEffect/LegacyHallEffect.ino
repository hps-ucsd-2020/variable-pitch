/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LIBRARIES

//Servo
#include <Servo.h>                    // Servo Library
#include <PID_v1.h>                   // PID Library

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MACROS

//Hall Effect
#define hallPin 8
#define ledPin 13
#define UPDATE_TIME 1000 //Update rpm every second
#define UPDATE_SECS 1
#define NUM_MAGNETS 1

//Servo
#define START_ANGLE 30
#define END_ANGLE 330
#define DC_MIN 2.9        //from Parallax spec sheet
#define DC_MAX 97.1       //from Parallax spec sheet
//PID Gains
#define KP 0.15
#define KI 0.05
#define KD 0.01

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//GLOBAL VARIABLES

//Hall Effect
int current_state = 0, last_state = 0;
double timeold = 0, rpm = 0;
int magnet_counter = 0;


//Servo
//PID variables
double TARGET_ANGLE = 330.0;       // Target angle to achieve
double ANGLE = 0.0;               // Current angle read from the feedback pin
double SERVO_VAL = 93;            // Initial setpoint is the servo value that maintains zero motion
double OUTPUT_VAL = 0;            // Output value of the PID loop when it executes the Compute() function
int RESULT = 0;
//Parallax 360 Servo variables
int PIN_FEEDBACK = 8;         // Connect the feedback pin from the Parallax 360 servo to the #5 PWM pin
unsigned long T_HIGH = 0;
unsigned long T_LOW = 0;
unsigned long T_CYCLE = 0;
float DC = 0;
//Instantiate the PID and Servo instances
Servo CAM_CONTROL;
PID PID_LOOP(&ANGLE,&OUTPUT_VAL,&TARGET_ANGLE,KP,KI,KD,DIRECT);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  /****************************************
  * Hall Effect Setup
  ****************************************/
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);      
  
  // initialize the hall effect sensor pin as an input:
  pinMode(hallPin, INPUT); 
  Serial.begin(9600);
  //attachInterrupt( digitalPinToInterrupt(hallPin), rpm_fun, FALLING );
  //half_revolutions = 0;
  rpm = 0;
  timeold = 0;

  /****************************************
  * Servo Setup
  ****************************************/
  // Servo initialization
  CAM_CONTROL.attach(9);                    // Attach the signal pin of servo to pin 9 of arduino
  pinMode(PIN_FEEDBACK, INPUT);                        // Sets PWM pin 5 as the Feedback input pin
  
  // PID initialization
  PID_LOOP.SetMode(AUTOMATIC);              // Turns the PID loop on
  PID_LOOP.SetOutputLimits(-30,30);         // Sets the PID output to a range usable by the Parallax 360 Servo
  PID_LOOP.SetSampleTime(100);              // Set the PID to actually compute every 100 ms.
  
  timeold = millis();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  /****************************************
  * Calculate RPM
  ****************************************/
  int current_state = digitalRead(hallPin);
  if ((current_state != last_state) && current_state == LOW)
  {
    magnet_counter++;
  }

  if (millis() - timeold >= UPDATE_TIME) {
    if (magnet_counter > 0) {
      rpm = (magnet_counter * 60) / (NUM_MAGNETS * UPDATE_SECS);
    }
    else {
      rpm = 0;
    }
    Serial.println(rpm);
    timeold = millis();
    magnet_counter = 0;
  }
  last_state = current_state;

  /****************************************
  * Read Current Servo Angle
  ****************************************/
  while(1)
  {
    T_HIGH = pulseIn(PIN_FEEDBACK, HIGH);
    T_LOW = pulseIn(PIN_FEEDBACK, LOW);
    T_CYCLE = T_HIGH + T_LOW;
    if ( T_CYCLE > 1000 && T_CYCLE < 1200)
    {
      break;              //valid T_CYCLE;
    }
  }
  DC = (100 * T_HIGH) / T_CYCLE; 
  ANGLE = ((DC - DC_MIN) * 360) / (DC_MAX - DC_MIN + 1);

  /****************************************
  * Move Servo
  ****************************************/
  RESULT = PID_LOOP.Compute();
  if (RESULT == 1)
  {
    SERVO_VAL = 93 - OUTPUT_VAL;
    if (SERVO_VAL > 180) SERVO_VAL = 180;
    else if (SERVO_VAL < 93) SERVO_VAL = SERVO_VAL - 3;
    CAM_CONTROL.write(SERVO_VAL); //Move the servo
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FUNCTION DEFINITIONS
/*
void rpm_fun()
 {
   half_revolutions++;
   //Each rotation, this interrupt function is run twice
 }
 */
