#include <Wire.h>

// IMU 변수 선언
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t offsets[] = {-2974, -1529, 1862, 91, -3, 34};

// MOTOR 변수 선언
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

// gate sensor 선언
const int gatePin = 7;

// case variable
int cnt=0;


void setup_imu(int bitlate)
{
  Wire.begin();
  init_MPU6050();
  Serial.begin(bitlate);
  Serial.print("AcX");
  Serial.print("\t");
  Serial.print("AcY");
  Serial.print("\t");
  Serial.print("AcZ");
  Serial.print("\t");
  Serial.print("GyX");
  Serial.print("\t");
  Serial.print("GyY");
  Serial.print("\t");
  Serial.println("GyZ");
}

void setup_motor()
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
}

void setup_gate()
{
  pinMode(gatePin, INPUT);
}

void init_MPU6050(){
  //MPU6050 Initializing & Reset
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //MPU6050 Clock Type
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x03);     // Selection Clock 'PLL with Z axis gyroscope reference'
  Wire.endTransmission(true);

  //MPU6050 Gyroscope Configuration Setting
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // Gyroscope Configuration register
  //Wire.write(0x00);     // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
  Wire.write(0x08);     // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
  //Wire.write(0x10);     // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
  //Wire.write(0x18);     // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]
  Wire.endTransmission(true);

  //MPU6050 Accelerometer Configuration Setting
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // Accelerometer Configuration register
  //Wire.write(0x00);     // AFS_SEL=0, Full Scale Range = +/- 2 [g]
  //Wire.write(0x08);     // AFS_SEL=1, Full Scale Range = +/- 4 [g]
  Wire.write(0x10);     // AFS_SEL=2, Full Scale Range = +/- 8 [g]
  //Wire.write(0x18);     // AFS_SEL=3, Full Scale Range = +/- 10 [g]
  Wire.endTransmission(true);

  //MPU6050 DLPF(Digital Low Pass Filter)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);  // DLPF_CFG register
  //Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz
  //Wire.write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz
  //Wire.write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz
  //Wire.write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz
  //Wire.write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz
  //Wire.write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz
  Wire.write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz
  Wire.endTransmission(true);
}

void read_imu()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void imu_serial()
{
  //Serial.print("AcX = ");
  Serial.print(AcX);
  Serial.print("\t");
  //Serial.print(" | AcY = ");
  Serial.print(AcY);
  Serial.print("\t");
  //Serial.print(" | AcZ = ");
  Serial.print(AcZ);
  Serial.print("\t");
  //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  //Serial.print(" | GyX = ");
  Serial.print(GyX);
  Serial.print("\t");
  //Serial.print(" | GyY = ");
  Serial.print(GyY);
  Serial.print("\t");
  //Serial.print(" | GyZ = ");
  Serial.println(GyZ);
  delay(10);
}

void mc(int fb, int di,int v)
{
  if(fb == 1){//앞바퀴
    if(di == 1){//정회전
      analogWrite(ENA, v);
      analogWrite(ENB, v);

      digitalWrite(INA1, LOW);
      digitalWrite(INA2, HIGH);
      digitalWrite(INB1, HIGH);
      digitalWrite(INB2, LOW);
    }else{
      analogWrite(ENA, v);
      analogWrite(ENB, v);

      digitalWrite(INA1, HIGH);
      digitalWrite(INA2, LOW);
      digitalWrite(INB1, LOW);
      digitalWrite(INB2, HIGH);
    }
  }else{//뒷바퀴
    if(di== 1){//정회전
      analogWrite(ENC, v);
      analogWrite(END, v);

      digitalWrite(INC1, LOW);
      digitalWrite(INC2, HIGH);
      digitalWrite(IND1, HIGH);
      digitalWrite(IND2, LOW);
    }else{//역회전
      analogWrite(ENC, v);
      analogWrite(END, v);

      digitalWrite(INC1, HIGH);
      digitalWrite(INC2, LOW);
      digitalWrite(IND1, LOW);
      digitalWrite(IND2, HIGH);
    }
  }
}

void setup()
{
  setup_motor();
  setup_imu(9600);
  setup_gate();
}

void loop()
{
  switch(cnt)
  {
    case 2:
    // 앞바퀴(1) : 정회전(1), 뒷바퀴(0) : 역회전(0)
      mc(1,1,255);//바퀴, 방향, 속도
      mc(0,0,255);
      read_imu();
      imu_serial();
    //  Serial.println("case 2");
      break;

    case 3:
    // 앞바퀴(1) : 역회전(0), 뒷바퀴(0) : 정회전(1)
      mc(1,0,255);//바퀴, 방향, 속도
      mc(0,1,255);
      read_imu();
      imu_serial();
    //  Serial.println("case 3");
      break;

    case 4:
    // 앞바퀴(1) : 역회전(0), 뒷바퀴(0) : 역회전(0)
      mc(1, 0, 255);
      mc(0, 0, 255);
      read_imu();
      imu_serial();
    //  Serial.println("case 4");
      break;

    default:
    // 앞바퀴(1) : 정회전(1), 뒷바퀴(0) : 정회전(1)
      mc(1,1,255);//바퀴, 방향, 속도
      mc(0,1,255);
      read_imu();
      imu_serial();
    //  Serial.println("case 1");
      break;
  }

  if(digitalRead(gatePin))
  {
    cnt = 1;
  }

}
