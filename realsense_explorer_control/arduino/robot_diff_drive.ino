#include "MPU9250.h" // https://www.arduino.cc/reference/en/libraries/mpu9250/
#include <Encoder.h> // https://www.arduino.cc/reference/en/libraries/encoder/

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

int left_ticks = 0;
int right_ticks = 0;

int previousReading_left = 0;
int previousReading_right = 0;

MPU9250 mpu;

const float interval = 25; //millis

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);
  motor_stop();

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

}


void loop() {

  double tstart = millis();
  read_encoder();

  if (mpu.update()) {
    send_all_data();
  }

  get_desired_wheel_velocities();
  calc_wheel_pwm();
  set_motor_pwm();
  
  delay(50);
  
  
  double tend = millis();
  l_ticks_rate = (double)((left_ticks - previousReading_left)/(tend-tstart))*1000;
  r_ticks_rate = (double)((right_ticks - previousReading_right)/(tend-tstart))*1000;

  previousReading_left = left_ticks;
  previousReading_right = right_ticks;

}

void read_encoder() {
  left_ticks = enc_A.read();
  right_ticks = enc_B.read();
}

void set_motor_pwm() {

  if (l_pwm > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, int(l_pwm));
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, int(abs(l_pwm)));
  }

  if (r_pwm > 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, int(r_pwm));
  } else {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, int(abs(r_pwm)));
  }
}

void send_all_data() {
  float imu_array[10];

  imu_array[0] = mpu.getAccX();
  imu_array[1] = mpu.getAccY();
  imu_array[2] = mpu.getAccZ();

  imu_array[3] = mpu.getGyroX();
  imu_array[4] = mpu.getGyroY();
  imu_array[5] = mpu.getGyroZ();

  imu_array[6] = mpu.getQuaternionX();
  imu_array[7] = mpu.getQuaternionY();
  imu_array[8] = mpu.getQuaternionZ();
  imu_array[9] = mpu.getQuaternionW();

  // Left_ticks, Right_ticks, AccX, AccY, AccZ, GyrX, GyrY, GyrZ, QuX, QuY, QuZ, QuW

  String main_str = "";
  main_str = main_str + left_ticks + "/" + right_ticks;

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
  }
}


void calc_wheel_pwm() {

  l_pwm = map(l_desired_rate, -3250, 3250, -255, 255);
  r_pwm = map(r_desired_rate, -3250, 3250, -255, 255);

  //  float l_err = l_desired_rate - l_ticks_rate;
  //
  //  if (abs(l_err) > 10) {
  //
  //    if (l_err > 0) {
  //      l_pwm = l_pwm + map(l_err, -3250, 3250, -255, 255);
  //    } else {
  //      l_pwm = l_pwm - map(l_err, -3250, 3250, -255, 255);
  //    }
  //  }
  //
  //  float r_err = r_desired_rate - r_ticks_rate;
  //  if (abs(r_err) > 10) {
  //    if (l_err > 0) {
  //      r_pwm = r_pwm + map(r_err, -3250, 3250, -255, 255);
  //    }
  //    else {
  //      r_pwm = r_pwm - map(r_err, -3250, 3250, -255, 255);
  //    }
  //  }
  //
  //  if (l_desired_rate == 0) {
  //    l_pwm = 0;
  //  }
  //  if (r_desired_rate == 0) {
  //    r_pwm = 0;
  //  }

}


void motor_stop() {

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  analogWrite(enA, 0);
  analogWrite(enB, 0);
}