#include <Wire.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
// #include <ESP32_Servo.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// for esp32
// #define RXp2 16
// #define TXp2 17


///// HardwareSerial HC12;
// for esp32
// #define HC12 Serial2

// for esp8266 d1 mini
SoftwareSerial HC12(D3, D4);


// esp32 GPIO 25, 26, 27, 33 are motor output

//------------------servo pins--------------//
// int motor_r_f = 25;
// int motor_r_b = 26;
// int motor_l_f = 27;
// int motor_l_b = 33;

// int motor_1_pin = 25;
// int motor_2_pin = 26;
// int motor_3_pin = 27;
// int motor_4_pin = 32;

int motor_1_pin = D5;
int motor_2_pin = D6;
int motor_3_pin = D7;
int motor_4_pin = D2;

Servo motor1, motor2, motor3, motor4;

char packet[7];
boolean recvState;

const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data

boolean newData = false;
int input_THROTTLE, input_YAW, input_PITCH, input_ROLL;
String input = "";
//-----------------------------------------------------------------------//
int ESCout_1, ESCout_2, ESCout_3, ESCout_4;
int state1, state2, state3, state4;
//-----------------------------------------------------------------------//
int16_t gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature, acc_total_vector;
float angle_pitch, angle_roll, angle_yaw;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
float elapsedTime;
long Time, timePrev, time2;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
//-----------------------------------------------------------------------//
float pitch_PID = 0, roll_PID = 0, yaw_PID = 0;
float roll_error = 0, roll_previous_error = 0, pitch_error = 0, pitch_previous_error = 0, yaw_error = 0;
float roll_pid_p = 0, roll_pid_d = 0, roll_pid_i = 0, pitch_pid_p = 0, pitch_pid_i = 0, pitch_pid_d = 0, yaw_pid_p = 0, yaw_pid_i = 0, yaw_pid_d = 0;
float roll_desired_angle, pitch_desired_angle, yaw_desired_angle;
double twoX_kp = 1.3;
double twoX_ki = 0.03;
double twoX_kd = 18;

double yaw_kp = 3;
double yaw_ki = 0.02;
double yaw_kd = 0.0;
//-----------------------------------------------------------------------//


float pid_p_gain_roll = 1.3;   //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;  //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;  //Gain setting for the roll D-controller
int pid_max_roll = 400;        //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;   //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;  //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;   //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;        //Maximum output of the PID-controller (+/-)

void setup() {
  /// for brushed drone uncomment these folowing lines
  // pinMode(motor_1_pin, OUTPUT);
  // pinMode(motor_1_pin, OUTPUT);
  // pinMode(motor_1_pin, OUTPUT);
  // pinMode(motor_1_pin, OUTPUT);

  motor1.attach(motor_1_pin);
  motor2.attach(motor_2_pin);
  motor3.attach(motor_3_pin);
  motor4.attach(motor_4_pin);

  // for esp32
  // Serial1.begin(9600, SERIAL_8N1, RXp2, TXp2);

  Serial.begin(115200);

  // for arduino avr boards and esp 8266
  HC12.begin(9600);

  // Setting up the gyro
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");


  // mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // Serial.print("Accelerometer range set to: +-8G");

  // mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Serial.print("Gyro range set to: +- 500 deg/s");


  // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // Serial.print("Filter bandwidth set to: 21 Hz");

  delay(1000);
  Serial.println("Place your controller in a flat surface still");
  delay(3000);

  for (int cal_int = 0; cal_int < 2000; cal_int++) {
    if (cal_int % 125 == 0) Serial.print(".");
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    acc_x = a.acceleration.x;
    acc_y = a.acceleration.y;
    acc_z = a.acceleration.z;

    gyro_x = g.gyro.x;  //Add the low and high byte to the gyro_x variable
    gyro_y = g.gyro.y;  //Add the low and high byte to the gyro_y variable
    gyro_z = g.gyro.z;

    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
  }

  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  //-----------------------------------------------------------------------//
  Time = micros();
}

///
///
///-------------------loop----------------
///
///
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  timePrev = Time;
  Time = micros();
  elapsedTime = (float)(Time - timePrev) / (float)1000000;

  roll_PID = 0;
  pitch_PID = 0;
  yaw_PID = 0;

  // read_incoming_data();
  long int start = micros();
  recvWithEndMarker();
  showNewData();
  long int finish = micros();

  // if (newData == true) {

  //   Serial.println(finish - start);
  //   newData = false;
  // }

  acc_x = a.acceleration.x;
  acc_y = a.acceleration.y;
  acc_z = a.acceleration.z;

  gyro_x = g.gyro.x;  //Add the low and high byte to the gyro_x variable
  gyro_y = g.gyro.y;  //Add the low and high byte to the gyro_y variable
  gyro_z = g.gyro.z;

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  angle_pitch += gyro_x * elapsedTime * 0.01526717557;  // 0.01526717557 = 1/65.5
  angle_roll += gyro_y * elapsedTime * 0.01526717557;
  angle_yaw += gyro_z * elapsedTime * 0.01526717557;

  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);

  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));


  if (abs(acc_y) < acc_total_vector) {                                 //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;  //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                 //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;  //Calculate the roll angle.
  }

  angle_pitch_acc += 0;
  angle_roll_acc += 0;

  if (set_gyro_angles) {
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  } else {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }

  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
  //-----------------------------------------------------------------------//

  ////////////////////my code///////////////////////////
  if (input_ROLL > 1508) roll_desired_angle = input_ROLL - 1508;
  else if (input_ROLL < 1492) roll_desired_angle = input_ROLL - 1492;
  roll_desired_angle = angle_roll * 15;
  roll_desired_angle /= 3.0;

  if (input_PITCH > 1508) pitch_desired_angle = input_PITCH - 1508;
  else if (input_PITCH < 1492) pitch_desired_angle = input_PITCH - 1492;
  pitch_desired_angle = angle_pitch * 15;
  pitch_desired_angle /= 3.0;

  if (input_YAW > 1508) yaw_desired_angle = input_YAW - 1508;
  else if (input_YAW < 1492) yaw_desired_angle = input_YAW - 1492;
  yaw_desired_angle = angle_yaw * 15;
  yaw_desired_angle /= 3.0;
  // /////////////////////////////////////////////////////////////////

  // // roll_desired_angle = 3 * ((float)input_ROLL / (float)10 - (float)5);
  // // pitch_desired_angle = 3 * ((float)input_PITCH / (float)10 - (float)5);
  // //yaw_desired_angle =0;

  // // roll_pid_i = roll_pid_i + (twoX_ki * roll_error);
  // // pitch_pid_i = pitch_pid_i + (twoX_ki * pitch_error);
  // // yaw_pid_i = yaw_pid_i + (yaw_ki * yaw_error);

  roll_error = angle_roll_output - roll_desired_angle;
  pitch_error = angle_pitch_output - pitch_desired_angle;
  yaw_error = angle_yaw - yaw_desired_angle;

  roll_pid_p = twoX_kp * roll_error;
  pitch_pid_p = twoX_kp * pitch_error;
  yaw_pid_p = yaw_kp * yaw_error;


  if (-3 < roll_error < 3) { roll_pid_i = roll_pid_i + (twoX_ki * roll_error); }
  if (-3 < pitch_error < 3) { pitch_pid_i = pitch_pid_i + (twoX_ki * pitch_error); }
  if (-3 < yaw_error < 3) { yaw_pid_i = yaw_pid_i + (yaw_ki * yaw_error); }

  roll_pid_d = twoX_kd * ((roll_error - roll_previous_error) / elapsedTime);
  pitch_pid_d = twoX_kd * ((pitch_error - pitch_previous_error) / elapsedTime);
  // yaw_pid_d = yaw_kd * ((yaw_error - yaw_previous_error) / elapsedTime);       //yaw_kd = 0 so the following term will be 0


  roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
  pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
  yaw_PID = yaw_pid_p + yaw_pid_i;

  if (roll_PID < -400) {
    roll_PID = -400;
  } else if (roll_PID > 400) {
    roll_PID = 400;
  }
  if (pitch_PID < -400) {
    pitch_PID = -400;
  } else if (pitch_PID > 400) {
    pitch_PID = 400;
  }
  if (yaw_PID < -400) {
    yaw_PID = -400;
  } else if (yaw_PID > 400) {
    yaw_PID = 400;
  }

  ESCout_1 = input_THROTTLE - roll_PID - pitch_PID - yaw_PID;
  ESCout_2 = input_THROTTLE + roll_PID - pitch_PID + yaw_PID;
  ESCout_3 = input_THROTTLE + roll_PID + pitch_PID - yaw_PID;
  ESCout_4 = input_THROTTLE - roll_PID + pitch_PID + yaw_PID;


  if (ESCout_1 > 2000) {
    ESCout_1 = 2000;
  } else if (ESCout_1 < 1100) ESCout_1 = 1100;
  if (ESCout_2 > 2000) {
    ESCout_2 = 2000;
  } else if (ESCout_2 < 1100) ESCout_2 = 1100;
  if (ESCout_3 > 2000) {
    ESCout_3 = 2000;
  } else if (ESCout_3 < 1100) ESCout_3 = 1100;
  if (ESCout_4 > 2000) {
    ESCout_4 = 2000;
  } else if (ESCout_4 < 1100) ESCout_4 = 1100;

  roll_previous_error = roll_error;
  pitch_previous_error = pitch_error;

  //-----------------------------------------------------------------------//
  ////////------------------50hz refresh rate------------------/////////
  motor1.writeMicroseconds(ESCout_1);
  motor2.writeMicroseconds(ESCout_2);
  motor3.writeMicroseconds(ESCout_3);
  motor4.writeMicroseconds(ESCout_4);

  // /// for brushed drone uncomment these folowing lines and comment the upper code segment
  // // analogWrite(motor_1_pin, map(ESCout_1, 1000, 2000, 0, 255));
  // // analogWrite(motor_2_pin, map(ESCout_2, 1000, 2000, 0, 255));
  // // analogWrite(motor_3_pin, map(ESCout_3, 1000, 2000, 0, 255));
  // // analogWrite(motor_4_pin, map(ESCout_4, 1000, 2000, 0, 255));

  while ((micros() - Time) < 2000)
    ;

  Serial.print(input_THROTTLE);
  Serial.print(" ");
  Serial.print(input_YAW);
  Serial.print(" ");
  Serial.print(input_PITCH);
  Serial.print(" ");
  Serial.print(input_ROLL);
  Serial.print(" ");

  Serial.print("         ");

  Serial.print(ESCout_1);
  Serial.print(" ");
  Serial.print(ESCout_2);
  Serial.print(" ");
  Serial.print(ESCout_3);
  Serial.print(" ");
  Serial.print(ESCout_4);

  Serial.print("        ");

  Serial.print(roll_PID);
  Serial.print("   ");
  Serial.print(pitch_PID);
  Serial.print("   ");
  Serial.print(yaw_PID);
  Serial.print("       ");

  Serial.print("  ");
  Serial.print(angle_roll_output);
  Serial.print("  ");
  Serial.print(angle_pitch_output);
  Serial.print(" | ");
  Serial.print(roll_desired_angle);
  Serial.print("  ");
  Serial.print(pitch_desired_angle);
  Serial.println();
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (HC12.available() > 0 && newData == false) {
    rc = HC12.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    // Serial.print("This just in ... ");
    String input = String(receivedChars);

    input.trim();

    int currentIndex = 0;
    int i = 0, length = input.length();

    int nextIndex = 0;

    while (nextIndex < length) {
      nextIndex = currentIndex + 3;
      String number = input.substring(currentIndex, nextIndex);

      int result = number.toInt();

      if (i == 0) input_THROTTLE = result + 1000;
      else if (i == 1) input_YAW = result + 1000;
      else if (i == 2) input_PITCH = result + 1000;
      else if (i == 3) input_ROLL = result + 1000;
      i++;

      currentIndex = nextIndex;
    }

    // Serial.print(input_THROTTLE);
    // Serial.print("   ");
    // Serial.print(input_YAW);
    // Serial.print("   ");
    // Serial.print(input_PITCH);
    // Serial.print("   ");
    // Serial.print(input_ROLL);
    // Serial.println("   ");
    // Serial.println(receivedChars);

    newData = false;
  }
}