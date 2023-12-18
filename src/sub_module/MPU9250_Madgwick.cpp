

#include "MPU9250.h"
#include "MadgwickAHRS.h"
MPU9250 IMU(Wire, 0x68);

#define GRAVITY 9.802 // The gravity acceleration in New York City

// Calibration outcomes
#define GYRO_X_OFFSET 0.0713998
#define GYRO_Y_OFFSET -0.0512795
#define GYRO_Z_OFFSET -0.0038634

#define ACCEL_X_OFFSET -0.1961766
#define ACCEL_Y_OFFSET 0.1277898
#define ACCEL_Z_OFFSET -10.2257710

const float magn_ellipsoid_center[3] = {-1.22362, -3.49591, -28.3068};
const float magn_ellipsoid_transform[3][3] = {{0.936683, -0.0120599, -0.00747369}, {-0.0120599, 0.997691, -5.88781e-05}, {-0.00747369, -5.88781e-05, 0.846255}};

// Sensor variables
float accel[3]; // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float magnetom[3];
float magnetom_tmp[3];
float gyro[3];

float time_now;
float time_former;
float deltat;

int roll;
int pitch;
int yaw;

// Task
TaskHandle_t Task2;

// Set structs for converting result from Quaternion to Euler angles
struct Quaternion
{
  float w, x, y, z;
};

struct EulerAngles
{
  float roll_e, pitch_e, yaw_e;
};

Quaternion qua;
EulerAngles eul;

// Read data from MPU9250
void read_sensors()
{
  IMU.readSensor();
  accel[0] = IMU.getAccelX_mss();
  accel[1] = IMU.getAccelY_mss();
  accel[2] = IMU.getAccelZ_mss();

  magnetom[0] = IMU.getMagX_uT();
  magnetom[1] = IMU.getMagY_uT();
  magnetom[2] = IMU.getMagZ_uT();

  gyro[0] = IMU.getGyroX_rads();
  gyro[1] = IMU.getGyroY_rads();
  gyro[2] = IMU.getGyroZ_rads();
}
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3]);

// Apply calibration to raw sensor readings
void compensate_sensor_errors()
{
  // Compensate accelerometer error
  accel[0] = accel[0] - ACCEL_X_OFFSET;
  accel[1] = accel[1] - ACCEL_Y_OFFSET;
  accel[2] = accel[2] - (ACCEL_Z_OFFSET + GRAVITY);

  // Compensate magnetometer error
  for (int i = 0; i < 3; i++)
    magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
  Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);

  // Compensate gyroscope error
  gyro[0] -= GYRO_X_OFFSET;
  gyro[1] -= GYRO_Y_OFFSET;
  gyro[2] -= GYRO_Z_OFFSET;
}

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
  for (int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}

void sendToPC(float *data1, float *data2, float *data3)
{
  byte *byteData1 = (byte *)(data1);
  byte *byteData2 = (byte *)(data2);
  byte *byteData3 = (byte *)(data3);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                  byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                  byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  // Serial.write(buf, 12);
}

EulerAngles ToEulerAngles(Quaternion q)
{
  EulerAngles angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll_e = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1)
    angles.pitch_e = -copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    angles.pitch_e = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw_e = atan2(siny_cosp, cosy_cosp);

  return angles;
}

void MPU9250Setup()
{
  // Serial.begin(115200);
  // while (!Serial)
  //   yield();

  IMU.begin();

  // setting the accelerometer full scale range to +/-8G
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(9);

  // IMU.enableWakeOnMotion
  time_former = micros();
}

unsigned int lastCalcTime = 0;

// -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz
void MPU9250Loop()
{
  // put your main code here, to run repeatedly:
  read_sensors();

  compensate_sensor_errors();

  time_now = micros();
  deltat = (float)(time_now - time_former) / 1000000.0f;
  time_former = time_now;
  MadgwickQuaternionUpdate(accel[0], accel[1], accel[2],
                           gyro[0], gyro[1], gyro[2],
                           magnetom[0], magnetom[1], magnetom[2], deltat);
  qua.w = q[0];
  qua.x = q[1];
  qua.y = q[2];
  qua.z = q[3];
  eul = ToEulerAngles(qua);
  eul.yaw_e += 0.8;
  if (eul.yaw_e > PI)
  {
    eul.yaw_e -= 2 * PI;
  }
  roll = eul.roll_e * 180.0 / PI;
  pitch = eul.pitch_e * 180.0 / PI;
  yaw = eul.yaw_e * 180.0 / PI;

  roll = roll > 0 ? 180 - roll : -(180 + roll);

  // sendToPC(&eul.roll_e, &eul.pitch_e, &eul.yaw_e);
  // vTaskDelay(1);
  // delayMicroseconds(500);
}

// #include <Arduino.h>

// #include "MPU9250.h"
// MPU9250 IMU(Wire, 0x68);

// float accel[3];
// float gyro[3];
// int sample_times = 1;

// float acc_cal[3] = {0.0, 0.0, 0.0};
// float gyro_cal[3] = {0.0, 0.0, 0.0};

// void read_sensors()
// {
//   IMU.readSensor();
//   accel[0] = IMU.getAccelX_mss();
//   accel[1] = IMU.getAccelY_mss();
//   accel[2] = IMU.getAccelZ_mss();

//   gyro[0] = IMU.getGyroX_rads();
//   gyro[1] = IMU.getGyroY_rads();
//   gyro[2] = IMU.getGyroZ_rads();
// }

// void setup()
// {
//   Serial.begin(115200);
//   while (!Serial)
//     yield();

//   // start communication with IMU
//   IMU.begin();
//   // setting the accelerometer full scale range to +/-8G
//   IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
//   // setting the gyroscope full scale range to +/-500 deg/s
//   IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
//   // setting DLPF bandwidth to 20 Hz
//   IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
//   // setting SRD to 19 for a 50 Hz update rate
//   IMU.setSrd(9);
// }

// void loop()
// {
//   read_sensors();
//   for (int i = 0; i < 3; i++)
//   {
//     acc_cal[i] += accel[i];
//     gyro_cal[i] += gyro[i];
//   }
//   Serial.print(acc_cal[0] / sample_times, 7);
//   Serial.print(" ");
//   Serial.print(acc_cal[1] / sample_times, 7);
//   Serial.print(" ");
//   Serial.print(acc_cal[2] / sample_times, 7);
//   Serial.print(" ");
//   Serial.print(gyro_cal[0] / sample_times, 7);
//   Serial.print(" ");
//   Serial.print(gyro_cal[1] / sample_times, 7);
//   Serial.print(" ");
//   Serial.println(gyro_cal[2] / sample_times, 7);
//   sample_times++;
// }
