/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LIBRARIES

//Servo
#include <Servo.h>                    // Servo Library
#include <PID_v1.h>                   // PID Library

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MACROS

//Hall Effect
#define hallPin 2
#define UPDATE_TIME 500 //Update rpm every 2 millisecond
#define UPDATE_SECS 1
#define NUM_MAGNETS 5
#define ledPin 13

//Ciruclar Buffer
#define BUFFER_SIZE 10

//RPM to output angle
#define MAX_RPM 240
#define MAX_OUTPUT_ANGLE 40
#define GEAR_RATIO 20

//Servo
#define PIN_PWM 5
#define PIN_FEEDBACK 4
#define END_ANGLE 330
#define DC_MIN 2.9        //from Parallax spec sheet
#define DC_MAX 97.1       //from Parallax spec sheet
//PID Gains
#define KP 0.075
#define KI 0.05
#define KD 0.01

//Setting zero
#define BUTTON_RIGHT 8
#define BUTTON_LEFT 9
#define BUTTON_STATE 10
#define CONTROL_PERIOD 20000    //Control signal requires a 20 ms period, 20ms = 20000us

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//GLOBAL VARIABLES

//Hall Effect
double timeold = 0, rpm = 0;
int magnet_counter = 0;
double average_rpm = 0;

//Servo
double START_ANGLE = 0;
double TARGET_ANGLE = START_ANGLE;       // Target angle to achieve
double ANGLE = 0.0;               // Current angle read from the feedback pin
double SERVO_VAL = 93;            // Initial setpoint is the servo value that maintains zero motion
double OUTPUT_VAL = 0;            // Output value of the PID loop when it executes the Compute() function
int RESULT = 0;
//Parallax 360 Servo variables
unsigned long T_HIGH = 0;
unsigned long T_LOW = 0;
unsigned long T_CYCLE = 0;
float DC = 0;
//Instantiate the PID and Servo instances
Servo CAM_CONTROL;
PID PID_LOOP(&ANGLE,&OUTPUT_VAL,&TARGET_ANGLE,KP,KI,KD,DIRECT);
int num_rotations = 0;
double prev_angle = 0;

//State Variables
bool curr_state = true;
int prev_state = 0;

//Setting zero
int t_control = 1350;
unsigned int t_prev = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CLASS DEFINITIONS
// Declare like Circularbuffer<type, size> var_name
template <typename Data, int Size>
class CircularBuffer{
public:
  CircularBuffer(): bufpos(0), buffer_full(false){}
  // adds the parameter to the buffer as the newest element
  // removes and returns the oldest element
  Data add(Data to_add) {
    Data removed = buf[bufpos];
    buf[bufpos] = to_add;
    bufpos++;
    // check for loop
    if(bufpos >= Size) {
      bufpos = 0;
      buffer_full = true;
    }
    return removed;
  }
  // sets all values in the buffer to 0, and sets the buffer to not full
  void clear_buffer() {
    for(int i = 0; i < Size; i++){
      buf[i] = 0;
    }
    bufpos = 0;
    buffer_full = false;
  }
  // returns the average of all the data in the buffer
  double average(){
    int data_points = buffer_full? Size : bufpos;
    double sum = 0;
    for(int i = 0; i < data_points; i++) {
      sum += buf[i];
    }
    return (data_points == 0)? 0 : sum / (double)data_points;
  }
private:
  Data buf[Size] = {}; //values initialized to NULL
  int bufpos; // position in the buffer
  bool buffer_full; // have we filled all the values in the buffer?
};


CircularBuffer<double,BUFFER_SIZE> averaging_filter;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  /****************************************
  * Hall Effect Setup
  ****************************************/
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(hallPin), isr, RISING);
  pinMode(hallPin, INPUT);
  pinMode(ledPin, OUTPUT);
  magnet_counter = 0;
  rpm = 0;

  /****************************************
  * Circular Buffer Setup
  ****************************************/
  average_rpm = 0;
  averaging_filter.clear_buffer();

  /****************************************
  * Servo Setup
  ****************************************/
  // Servo initialization
  CAM_CONTROL.attach(PIN_PWM);                    // Attach the signal pin of servo to pin 9 of arduino
  pinMode(PIN_FEEDBACK, INPUT);                        // Sets PWM pin 5 as the Feedback input pin
  
  // PID initialization
  PID_LOOP.SetMode(AUTOMATIC);              // Turns the PID loop on
  PID_LOOP.SetOutputLimits(-30,30);         // Sets the PID output to a range usable by the Parallax 360 Servo
  PID_LOOP.SetSampleTime(100);              // Set the PID to actually compute every 100 ms.

  /****************************************
  * State Machine Setup
  ****************************************/
  curr_state = true;
  prev_state = 0;
  pinMode(BUTTON_RIGHT, INPUT);
  pinMode(BUTTON_LEFT, INPUT);
  pinMode(BUTTON_STATE, INPUT);

  t_prev = micros();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  
  int button_state = digitalRead(BUTTON_STATE);
  Serial.println(button_state);
  /*
  Serial.print(button_state);
  Serial.print("\t");
  Serial.print(digitalRead(BUTTON_RIGHT));
  Serial.print("\t");
  Serial.println(digitalRead(BUTTON_LEFT));
  */
  
  if (button_state == HIGH && button_state != prev_state){
    delay(100);
    /*if (digitalRead(BUTTON_STATE == LOW)){
      return;
    }*/
    Serial.println("~SWITCH~");
    curr_state = !curr_state;

    while(digitalRead(BUTTON_STATE));

    /*
    if (curr_state == false){
      START_ANGLE = get_angle();
      num_rotations = 0;
      prev_angle = START_ANGLE;
    }
    */
  }
  
  if (curr_state){
    variable_pitch();
  }
  else{
    set_zero();
  }
  t_prev = micros();
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FUNCTION DEFINITIONS
void isr(){
  magnet_counter++;
}

double get_angle(){
  while(1)
  {
    T_HIGH = pulseIn(PIN_FEEDBACK, HIGH);
    T_LOW = pulseIn(PIN_FEEDBACK, LOW);
    T_CYCLE = T_HIGH + T_LOW;
    //Serial.print("T_HIGH: ");
    //Serial.print(T_HIGH);
    //Serial.print("\tT_LOW: ");
    //Serial.println(T_LOW);
    if ( T_CYCLE > 1000 && T_CYCLE < 1200)
    {
      break;              //valid T_CYCLE;
    }
  }
  //Convert duty cycle of signal to RPM, equation from spec sheet
  DC = (100 * T_HIGH) / T_CYCLE; 
  double curr_angle = ((DC - DC_MIN) * 360) / (DC_MAX - DC_MIN + 1);

  return curr_angle;
}

void variable_pitch(){
  /****************************************
  * Calculate RPM
  ****************************************/
  digitalWrite(ledPin, !digitalRead(hallPin));
  
  if (millis() - timeold >= UPDATE_TIME) {
    if (magnet_counter > 0) {
      rpm = (magnet_counter * 60) / (NUM_MAGNETS * UPDATE_SECS);
    }
    else {
      rpm = 0;
    }
    timeold = millis();
    magnet_counter = 0;

    averaging_filter.add(rpm);
    average_rpm = averaging_filter.average();
  }
  Serial.print("Averaged RPM: ");
  Serial.print(average_rpm);
  Serial.print("\t");

  /****************************************
  * Get target angle based on RPM
  ****************************************/
  if (average_rpm > MAX_RPM){
    TARGET_ANGLE = START_ANGLE - MAX_OUTPUT_ANGLE*GEAR_RATIO;
  }
  else if (average_rpm == 0){
    TARGET_ANGLE = START_ANGLE;
  }
  else{
    TARGET_ANGLE = map(MAX_RPM-average_rpm, 0, MAX_RPM, START_ANGLE-(MAX_OUTPUT_ANGLE*GEAR_RATIO), START_ANGLE);
  }
  Serial.print("Target Angle: ");
  Serial.print(TARGET_ANGLE);
  
  /****************************************
  * Read Current Servo Angle
  ****************************************/
  float curr_angle = get_angle();

  // Check if a full rotation was made
  if (curr_angle <= 60 && prev_angle >= 300){
    num_rotations++;
  }
  else if (curr_angle >= 300 && prev_angle <= 60){
     num_rotations--;
  }
  prev_angle = curr_angle;

  // Add offset
  ANGLE = curr_angle + (num_rotations*360);

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
  

  Serial.println();
}

void set_zero(){
  /****************************************
  * Read Current Servo Angle
  ****************************************/
  float curr_angle = get_angle(); 
  // Check if a full rotation was made
  if (curr_angle <= 60 && prev_angle >= 300){
    num_rotations++;
  }
  else if (curr_angle >= 300 && prev_angle <= 60){
     num_rotations--;
  }
  prev_angle = curr_angle;
  // Add offset
  ANGLE = curr_angle + (num_rotations*360);

  /****************************************
  * Change Zero Angle
  ****************************************/
  Serial.print("Set: ");
  if (digitalRead(BUTTON_RIGHT) == HIGH){
    //t_control = setServoSpeed(1);
    delay(100);
    START_ANGLE += 100;
    Serial.println("RIGHT");
    while(digitalRead(BUTTON_RIGHT));
  }
  else if (digitalRead(BUTTON_LEFT) == HIGH){
    //t_control = setServoSpeed(-1);
    START_ANGLE -= 100;
    Serial.println("LEFT");
    while(digitalRead(BUTTON_LEFT));
  }
  else{
    //t_control = setServoSpeed(0);
    Serial.println("ZERO");
  }
  TARGET_ANGLE = START_ANGLE;

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
  
  /*
  digitalWrite(PIN_PWM, HIGH);
  delayMicroseconds(t_control);
  
  digitalWrite(PIN_PWM, LOW);
  delayMicroseconds(CONTROL_PERIOD - t_control - (micros() - t_prev));   
  */
}

int setServoSpeed(float input){
  int high_time = 1490; //Period to stop the motor
  if (input < 0 && input >= -1){
    high_time = map(input, -1, 0, 1280, 1480); 
  }
  else if (input > 0 && input <= 1){
    high_time = map(input, 0, 1, 1520, 1720);
  return high_time;
  }
}

