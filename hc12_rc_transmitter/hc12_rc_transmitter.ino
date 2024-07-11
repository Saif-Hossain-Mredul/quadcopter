#include <SoftwareSerial.h>
#include <Wire.h>

// here, 11 is the rx pin and 10 is the tx pin on arduino
SoftwareSerial HC12(11, 10);  //connect this to your respective tx, rx pins in hc12

// for joystick 1
int pin_throttle = A0;
int pin_yaw = A1;

// for joystick 2
int pin_pitch = A2;
int pin_roll = A3;

void setup() {
  HC12.begin(9600);  //Sets the hc 12 communication to 9600 baud
  Serial.begin(9600);

  pinMode(pin_throttle, INPUT);
  pinMode(pin_yaw, INPUT);
  pinMode(pin_pitch, INPUT);
  pinMode(pin_roll, INPUT);
}

void loop() {
  int throttle = analogRead(pin_throttle);
  int yaw = analogRead(pin_yaw);
  int pitch = analogRead(pin_pitch);
  int roll = analogRead(pin_roll);


  String output;

  throttle = mapValues(throttle, 0, 499, 1023);
  yaw = mapValues(yaw, 0, 515, 1023);
  pitch = mapValues(pitch, 0, 487, 1023);
  roll = mapValues(roll, 0, 509, 1023);



  output = get_num_string(throttle) + get_num_string(yaw) + get_num_string(pitch) + get_num_string(roll);

  Serial.println(output);

  HC12.println(output);
}

int mapValues(int val, int lower, int middle, int upper) {
  if (val < middle) {
    val = map(val, lower, middle, 0, 500);
  } else {
    val = map(val, middle, upper, 500, 999);
  }

  return val;
}

String get_num_string(int num) {
  if (num < 10) return "00" + String(num);
  if (num > 9 && num < 100) return "0" + String(num);
  else return String(num);
}
