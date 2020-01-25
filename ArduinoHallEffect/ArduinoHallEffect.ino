/*
 Hall Effect Switch
 
 Turns on and off a light emitting diode(LED) connected to digital  
 pin 13, when Hall Effect Sensor attached to pin 2 is triggered by a magnet
 http://www.hobbytronics.co.uk/arduino-tutorial8-hall-effect

 Most recent changes: Added circularbuffer support
*/
#include "CircularBuffer.hpp"

// conditional compiling debug stuff
// place code in D(/*code*/) and it will only compile if DEBUG is true (1)
#define DEBUG 1
#if DEBUG
#define D(x) x
#else
#define D(x)
#endif

#define SLEEP_ON 0
#if SLEEP_ON
#define S(x) x
#else
#define S(x)
#endif

#define HALL_PIN 8
#define LED_PIN 13
#define WAKE_PIN 8

#define READ_DELAY 1000
#define SLEEP_DELAY 100

#define FILTER_SIZE 10
 
int hallState = 0;          // variable for reading the hall sensor status
int half_revolutions;
unsigned int rpm;
unsigned long timeold;
unsigned int angle = 0;
CircularBuffer<unsigned int, FILTER_SIZE> rpm_buffer;

void setup() {
  // initialize the LED pin as an output:
  pinMode(LED_PIN, OUTPUT);      
  // initialize the hall effect sensor pin as an input:
  pinMode(HALL_PIN, INPUT); 

  
  Serial.begin(9600);
  Serial.print("Serial Online\n");
  attachInterrupt( digitalPinToInterrupt(HALL_PIN), rpm_fun, FALLING );
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;  

  S(sleep_setup()); // set up what we need for sleep things
}

int sleep_timer = 0; // keeps track of how long we're awake
void loop(){
  // read the state of the hall effect sensor:
  hallState = digitalRead( HALL_PIN );

  if (hallState == LOW) {     
    // turn LED on:    
    digitalWrite(LED_PIN, HIGH); 
  } 
  else {
    // turn LED off:
    digitalWrite( LED_PIN, LOW ); 
  }

  //servo.write(angle); TODO

  if (half_revolutions >= 1) { 
    sleep_timer = 0; // we read something, so stop falling asleep
    delay(READ_DELAY);
    detachInterrupt( digitalPinToInterrupt(HALL_PIN) );
     
     rpm = 30*1000/(millis() - timeold)*half_revolutions;
     rpm_buffer.add(rpm);
     unsigned int filtered_rpm = (unsigned int)rpm_buffer.average();
     timeold = millis();
     half_revolutions = 0;
     Serial.println(filtered_rpm,DEC);

     angle = map(filtered_rpm, 0, 120, 0, 270);

     attachInterrupt( digitalPinToInterrupt(HALL_PIN), rpm_fun, FALLING );
   } else {
      delay(SLEEP_DELAY); // runs each 0.1 second
      S(run_sleep_check());
      S(D(Serial.print("sleep timer: ")));
      S(D(Serial.println(sleep_timer)));
   }
  
}

void rpm_fun()
 {
   D(Serial.println("half revolution detected"));
   half_revolutions++;
   //Each rotation, this interrupt function is run twice
 }

/*
 * Sleep code lies below
 * Found procedure at https://playground.arduino.cc/Learning/arduinoSleepCode
 */
#include <avr/sleep.h>

// wait 15 seconds (150 tenths of a sec) before sleeping
#define SLEEP_TIME 150

/* TRIGGER OPTIONS:
 * 
 * LOW      low level trigger (supposedly the only one that works for SLEEP_MODE_PWR_DOWN)
 * CHANGE   change in level trigger
 * RISING   rising edge of level trigger (was set to this, but supposedly not supposed to work)
 * FALLING  falling edge of level trigger
 */
#define WAKE_TRIGGER_TYPE LOW

void sleep_setup(){
  pinMode(WAKE_PIN, INPUT); // set up a pin to wake us from sleep
  attachInterrupt(0, wake_up, WAKE_TRIGGER_TYPE); // attach the interrupt to 0 (pin 2), run wake_up when it gets LOW
}
  
void sleep() {
  D(Serial.println("going to sleep... "));
  delay(100); // make sure we don't interrupt processes too hard
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep strength: maximum
  sleep_enable(); // we are now able to sleep
  attachInterrupt(0, wake_up, WAKE_TRIGGER_TYPE); // reattach interrupt
  sleep_mode(); // go to sleep - we wait here until we wake up
  // sleeping... zzz
  sleep_disable(); // first things first, make sure we don't go back to sleep
  detachInterrupt(0); // stop listening to the interrupt pin
  D(Serial.println("Waking up... "));
}

// checks if we're over time; if we are, fall asleep
// if we're not, just increment the counter
void run_sleep_check() {
  if(sleep_timer > SLEEP_TIME) {
    sleep_timer = 0;
    sleep();
  } else {
    sleep_timer++;
  }
}

void wake_up() {
  // here is where things happen directly after waking up. occurs between sleep_mode() and sleep_disable()
  // timers don't work here for reasons, so don't try to print anything to the serial because it won't work
  rpm_buffer.clear_buffer();
}
