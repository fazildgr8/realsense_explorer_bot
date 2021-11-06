#include "MPU9250.h" //https://www.arduino.cc/reference/en/libraries/mpu9250/
#include <Encoder.h> //https://www.arduino.cc/reference/en/libraries/encoder/

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

const int interval = 25;

MPU9250 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
}

void loop() {

  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + interval) {
      send_all_data();
      prev_ms = millis();
    }
  }
}


void send_all_data() {
  int left_ticks = enc_A.read();
  int right_ticks = enc_B.read();
  
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
  if(Serial.available() > 0) {
    String velocities = Serial.readStringUntil('\n');
    set_wheel_velocities(velocities);
  }
  main_str = main_str + "/";
  Serial.println(main_str);
}


void set_wheel_velocities(String velocity){}