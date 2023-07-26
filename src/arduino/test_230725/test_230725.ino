#include <Wire.h>

int ENA = 8;
int ENB = 9;
int ENC = 10;
int END = 11;
int INA1 = 30;
int INA2 = 31;
// A : 1) LOW, 2) HOGH >> forward
int INB1 = 32;
int INB2 = 33;
// B : 1) HIGH, 2) LOW >> forward
int INC1 = 34;
int INC2 = 35;
// C : 1) LOW, 2) HOGH >> forward
int IND1 = 36;
int IND2 = 37;
// D : 1) HIGH, 2) LOW >> forward

void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);

  pinMode(ENC, OUTPUT);
  pinMode(INC1, OUTPUT);
  pinMode(INC2, OUTPUT);

  pinMode(END, OUTPUT);
  pinMode(IND1, OUTPUT);
  pinMode(IND2, OUTPUT);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(ENC, HIGH);
  digitalWrite(END, HIGH);

  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  analogWrite(ENC, 100);
  analogWrite(END, 100);

  Wire.begin();
  init_MPU6050();
  Serial.begin(115200);
}

// ================================================================
// ===                          define                          ===
// ================================================================
#define MPU6050_AXOFFSET 138
#define MPU6050_AYOFFSET 9
#define MPU6050_AZOFFSET -134

long sampling_timer;
const int MPU_addr=0x68; // I2C address of the MPU-6050

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // Raw data of MPU6050
float GAcX, GAcY, GAcZ; // Convert accelerometer to gravity value
float acc_pitch, acc_roll, acc_yaw; // Pitch, Roll & Yaw from Accelerometer
float angle_pitch, angle_roll, angle_yaw; // Angle of Pitch, Roll, & Yaw

// ================================================================
// ===                       init_MPU6050                       ===
// ================================================================
void init_MPU6050(){
  //MPU6050 Initializing & Reset
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //MPU6050 Clock Type
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x03); // Selection Clock 'PLL with Z axis gyroscope reference'
  Wire.endTransmission(true);

  //MPU6050 Gyroscope Configuration Setting
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B); // Gyroscope Configuration register
  //Wire.write(0x00); // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
  //Wire.write(0x08); // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
  //Wire.write(0x10); // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
  Wire.write(0x18); // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]
  Wire.endTransmission(true);

  //MPU6050 Accelerometer Configuration Setting
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C); // Accelerometer Configuration register
  //Wire.write(0x00); // AFS_SEL=0, Full Scale Range = +/- 2 [g]
  //Wire.write(0x08); // AFS_SEL=1, Full Scale Range = +/- 4 [g]
  Wire.write(0x10); // AFS_SEL=2, Full Scale Range = +/- 8 [g]
  //Wire.write(0x18); // AFS_SEL=3, Full Scale Range = +/- 10 [g]
  Wire.endTransmission(true);

  //MPU6050 DLPF(Digital Low Pass Filter)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A); // DLPF_CFG register
  Wire.write(0x00); // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz
  //Wire.write(0x01); // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz
  //Wire.write(0x02); // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz
  //Wire.write(0x03); // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz
  //Wire.write(0x04); // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz
  //Wire.write(0x05); // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz
  //Wire.write(0x06); // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz
  Wire.endTransmission(true);
}

// ================================================================
// ===                     calculate_Ac,Gy                      ===
// ================================================================
void raw_data_IMU()
{
  // Read raw data of MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // Raw data of accelerometer corrected by offset value
  AcX -= MPU6050_AXOFFSET;
  AcY -= MPU6050_AYOFFSET;
  AcZ -= MPU6050_AZOFFSET;

  // Convert accelerometer to gravity value
  GAcX = (float) AcX / 4096.0;
  GAcY = (float) AcY / 4096.0;
  GAcZ = (float) AcZ / 4096.0;
}

// ================================================================
// ===                     calculate_YPR                        ===
// ================================================================
void calculate_YPR()
{
  // Calculate Pitch, Roll & Yaw from Accelerometer value
  // Reference are
  // https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data
  // https://www.dfrobot.com/wiki/index.php/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing
  acc_pitch = atan ((GAcY - (float)MPU6050_AYOFFSET/4096.0) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951; // 180 / PI = 57.29577951
  acc_roll = - atan ((GAcX - (float)MPU6050_AXOFFSET/4096.0) / sqrt(GAcY * GAcY + GAcZ * GAcZ)) * 57.29577951;
  //acc_yaw = atan ((GAcZ - (float)MPU6050_AZOFFSET/4096.0) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951;
  acc_yaw = atan (sqrt(GAcX * GAcX + GAcZ * GAcZ) / (GAcZ - (float)MPU6050_AZOFFSET/4096.0)) * 57.29577951;

  // Calculate Pitch, Roll & Yaw by Accelerometer
  angle_pitch = acc_pitch;
  angle_roll = acc_roll;
  angle_yaw = acc_yaw; // Infact accelerometer doesn't have yaw value
}

// ================================================================
// ===                       MOTOR CONTROL                      ===
// ================================================================
void forward(int Va, int Vb, int Vc, int Vd)
{
  analogWrite(ENA, Va);
  analogWrite(ENB, Vb);
  analogWrite(ENC, Vc);
  analogWrite(END, Vd);

  digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);
  digitalWrite(INB1, HIGH);
  digitalWrite(INB2, LOW);
  digitalWrite(INC1, LOW);
  digitalWrite(INC2, HIGH);
  digitalWrite(IND1, HIGH);
  digitalWrite(IND2, LOW);
}

void backward(int Va, int Vb, int Vc, int Vd)
{
  analogWrite(ENA, Va);
  analogWrite(ENB, Vb);
  analogWrite(ENC, Vc);
  analogWrite(END, Vd);

  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  digitalWrite(INB1, LOW);
  digitalWrite(INB2, HIGH);
  digitalWrite(INC1, HIGH);
  digitalWrite(INC2, LOW);
  digitalWrite(IND1, LOW);
  digitalWrite(IND2, HIGH);
}

void loop()
{
 // raw_data_IMU();
 // calculate_YPR();
 // // Print value of Pitch, Roll & Yaw reflected Complementary Filter
 // Serial.print(" | angle_pitch = "); Serial.print(angle_pitch);
 // Serial.print(" | angle_roll = "); Serial.println(angle_roll);
 // // Sampling Timer
 // while(micros() - sampling_timer < 4000); //
 // sampling_timer = micros(); //Reset the sampling timer
  Serial.println("forward");
  forward(255, 255, 255, 255);
}
