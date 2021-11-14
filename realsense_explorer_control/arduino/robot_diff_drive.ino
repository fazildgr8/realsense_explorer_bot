#include "MPU6050_light.h" // https://www.arduino.cc/reference/en/libraries/mpu6050_light/
#include "Wire.h"
#include <Encoder.h> // https://www.arduino.cc/reference/en/libraries/encoder/
#include <PID_v1.h>  // https://github.com/br3ttb/Arduino-PID-Library

// Motor Left connections
Encoder enc_A(3, 12);
const int enA = 10;
const int in1 = 7;
const int in2 = 6;

// Motor Right connections
Encoder enc_B(2, 11);
const int enB = 9;
const int in3 = 4;
const int in4 = 5;

double l_pwm = 0;
double r_pwm = 0;

double l_ticks_rate = 0;
double r_ticks_rate = 0;

double l_desired_rate = 0;
double r_desired_rate = 0;

double l_desired_rate_abs = 0;
double r_desired_rate_abs = 0;

int left_ticks = 0;
int right_ticks = 0;

int l_direction = 1;
int r_direction = 1;

int previousReading_left = 0;
int previousReading_right = 0;

MPU6050 mpu(Wire);

double kp_l = 0.1, ki_l = 0, kd_l = 0;

//        Input          Output           SetPoint

PID l_PID(&l_ticks_rate, &l_pwm, &l_desired_rate_abs, kp_l, ki_l, kd_l, DIRECT);
PID r_PID(&r_ticks_rate, &r_pwm, &r_desired_rate_abs, kp_l, ki_l, kd_l, DIRECT);

const float interval = 25; //millis

void setup() {
  Serial.begin(115200);
  Wire.begin();
  byte status = mpu.begin();
  while (status != 0) { }
  delay(2000);
  mpu.calcOffsets(true, true);
  motor_stop();

  l_PID.SetMode(AUTOMATIC);
  l_PID.SetOutputLimits(0, 255);

  r_PID.SetMode(AUTOMATIC);
  r_PID.SetOutputLimits(0, 255);

}


void loop() {
  mpu.update();
  double tstart = millis();
  read_encoder();


  send_all_data();


  get_desired_wheel_velocities();
  calc_wheel_pwm();
  set_motor_pwm();

  delay(50);


  double tend = millis();
  l_ticks_rate = (double)((left_ticks - previousReading_left) / (tend - tstart)) * 1000;
  r_ticks_rate = (double)((right_ticks - previousReading_right) / (tend - tstart)) * 1000;

  previousReading_left = left_ticks;
  previousReading_right = right_ticks;

}

void read_encoder() {
  left_ticks = enc_A.read();
  right_ticks = enc_B.read();
}

void set_motor_pwm() {
  if (l_desired_rate != 0) {
    if (l_direction == 1) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, int(l_pwm));
    } else {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, int(abs(l_pwm)));
    }
  } else {
    analogWrite(enA, 0);
  }

  if (r_desired_rate != 0) {
    if (l_direction == 1) {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enB, int(r_pwm));
    } else {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enB, int(abs(r_pwm)));
    }
  } else {
    analogWrite(enB, 0);
  }

}

void send_all_data() {
  float imu_array[10];

  imu_array[0] = mpu.getAccX() * 9.81;
  imu_array[1] = -mpu.getAccY() * 9.81;
  imu_array[2] = mpu.getAccZ() * 9.81;

  imu_array[3] = mpu.getGyroX() * 0.0174533;
  imu_array[4] = mpu.getGyroY() * 0.0174533;
  imu_array[5] = mpu.getGyroZ() * 0.0174533;

  imu_array[6] = mpu.getAngleX() * 0.0174533;
  imu_array[7] = mpu.getAngleY() * 0.0174533;
  imu_array[8] = mpu.getAngleZ() * 0.0174533;
  imu_array[9] = 1;

  // Left_ticks, Right_ticks, AccX, AccY, AccZ, GyrX, GyrY, GyrZ, QuX, QuY, QuZ, QuW

  String main_str = "";
  main_str = main_str + int(left_ticks) + "/" + int(-right_ticks);

  for (int i = 0; i < 10; i++) {
    main_str = main_str + "/" + imu_array[i];
  }

  main_str = main_str + "/";
  Serial.println(main_str);
}


int* velocity_string_parse(String velocities) {
  velocities.remove(-2);
  int piviot = velocities.indexOf("/");

  String l_velocity = velocities;
  l_velocity.remove(piviot);

  String r_velocity = "";
  int i = piviot + 1;
  while (velocities[i]) {
    r_velocity = r_velocity + velocities[i];
    i = i + 1;
  }

  static int arr[2];
  arr[0] = l_velocity.toInt();
  arr[1] = r_velocity.toInt();

  return arr;
}

void get_desired_wheel_velocities() {
  if (Serial.available() > 0) {
    String velocities = Serial.readStringUntil('\n');
    int* arr;
    arr = velocity_string_parse(velocities);
    l_desired_rate = arr[0];
    r_desired_rate = arr[1];

    if (l_desired_rate < 0) {
      l_direction = -1;
    } else {
      l_direction = 1;
    }
    if (l_desired_rate < 0) {
      l_direction = -1;
    } else {
      l_direction = 1;
    }

    l_desired_rate_abs = abs(l_desired_rate);
    r_desired_rate_abs = abs(r_desired_rate);
  }
}


void calc_wheel_pwm() {
  //  l_pwm = map(l_desired_rate, -3250, 3250, -255, 255);
  //  r_pwm = map(r_desired_rate, -3250, 3250, -255, 255);
  l_PID.Compute();
  r_PID.Compute();
}


void motor_stop() {

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  analogWrite(enA, 0);
  analogWrite(enB, 0);
}