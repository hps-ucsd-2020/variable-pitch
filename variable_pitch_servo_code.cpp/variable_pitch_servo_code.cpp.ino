#include <stdlib.h>

#define SERVO_CTRL_PIN 5
#define BUFFER_SZ 255
#define ENDL '\n'
#define SERIAL_BAUD 9600

/*
   Sets the servo's rotation using PWM
   takes in a float from -1 to 1
   assigns a value from 0 to 255 to the PWM
*/
void set_rotation_speed(float val) {
  if (val > 1) val = 1;
  if (val < -1) val = -1;
  // now that val is def between -1 and 1, do our thing
  int pwm_val = (val + 1) * (255.0 / 2);
  analogWrite(SERVO_CTRL_PIN, pwm_val);
}

char input_buffer[BUFFER_SZ];
bool string_complete = false;
void read_input() {
  static byte input_dex = 0; // set to 0 when prog starts
  char in_char;

  while (Serial.available() > 0 && !string_complete) {
    in_char = Serial.read(); // read in one char
    if (in_char != ENDL) {
      input_buffer[input_dex] = in_char;
      input_dex++;
      if (input_dex >= BUFFER_SZ) in_char = BUFFER_SZ - 1;
    } else {
      // we got endl, pack up our string
      input_buffer[input_dex] = '/0'; //null terminate str
      input_dex = 0;
      string_complete = true; // set false after reading
    }
  }
}

void send_output() {
  if (!string_complete) return; // no input, nothing to do
  // process string
  float input = atof(input_buffer);
  // send rotation
  set_rotation_speed(input);
  Serial.print("setting speed to ");
  Serial.println(input);
  string_complete = false;
  Serial.print("ready for next input:\n");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.print("Ready to recieve input:\n");
}

/*
   takes in values from the terminal to move the servo
   at different speeds
*/
void loop() {
  // put your main code here, to run repeatedly:
  read_input();
  send_output();
}
