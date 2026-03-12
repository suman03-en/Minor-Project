#include <Wire.h>
#include <math.h>

// ================= UART =================
#define RXD2 16
#define TXD2 17

// ================= I2C ADDRESSES =================
#define MPU6050_ADDR 0x68
#define QMC5883_ADDR 0x0D

// ================= MPU6050 REGISTERS =================
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B

#define ACCEL_SCALE 16384.0
#define GYRO_SCALE 131.0

// ================= GLOBAL OFFSETS =================
double accelOffsetX=0, accelOffsetY=0, accelOffsetZ=0;
double gyroXOffset=0, gyroYOffset=0, gyroZOffset=0;
double magOffsetX=0, magOffsetY=0, magOffsetZ=0;

// ================= KALMAN VARIABLES =================
double Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
double anglePitch=0, biasPitch=0, Ppitch[2][2]={{0,0},{0,0}};
double angleRoll=0, biasRoll=0, Proll[2][2]={{0,0},{0,0}};

unsigned long lastTime;
double dt;

// =====================================================
void setup() {

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  Wire.begin();

  MPU6050_init();
  QMC5883_init();

  calibrate_MPU6050();
  calibrate_QMC5883();

  lastTime = millis();

  // Send CSV header
  Serial2.println("pitch,roll,yaw,ax,ay,az,gx,gy,gz");
}

// =====================================================
void loop() {

  double ax, ay, az, gx, gy, gz;
  double mx, my, mz;

  read_MPU6050(ax, ay, az, gx, gy, gz);
  read_QMC5883(mx, my, mz);

  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  double pitchAcc = atan2(-ax, sqrt(ay*ay + az*az)) * 180/PI;
  double rollAcc  = atan2(ay, az) * 180/PI;

  double pitch = Kalman_update(anglePitch, biasPitch, Ppitch, gx, pitchAcc);
  double roll  = Kalman_update(angleRoll, biasRoll, Proll, gy, rollAcc);

  double pitchRad = pitch * PI/180;
  double rollRad  = roll  * PI/180;

  double Xh = mx * cos(pitchRad) + mz * sin(pitchRad);
  double Yh = mx * sin(rollRad)*sin(pitchRad) + my*cos(rollRad) - mz*sin(rollRad)*cos(pitchRad);
  double yaw = atan2(Yh, Xh);

  yaw += 0.00727;

  if (yaw < 0) yaw += 2*PI;
  if (yaw > 2*PI) yaw -= 2*PI;

  double yawDeg = yaw * 180/PI;

  // ================= CREATE CSV =================
  String csv =
    String(pitch) + "," +
    String(roll)  + "," +
    String(yawDeg) + "," +
    String(ax) + "," +
    String(ay) + "," +
    String(az) + "," +
    String(gx) + "," +
    String(gy) + "," +
    String(gz);

  // Send to serial monitor
  Serial.println(csv);

  // Send to second ESP32
  Serial2.println(csv);

  delay(2000);
}

// =====================================================
// ================= MPU6050 FUNCTIONS =================
void MPU6050_init() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
}

void read_MPU6050(double &ax,double &ay,double &az,double &gx,double &gy,double &gz){

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR,14,true);

  int16_t rawAx = (Wire.read()<<8 | Wire.read());
  int16_t rawAy = (Wire.read()<<8 | Wire.read());
  int16_t rawAz = (Wire.read()<<8 | Wire.read());

  Wire.read(); Wire.read();

  int16_t rawGx = (Wire.read()<<8 | Wire.read());
  int16_t rawGy = (Wire.read()<<8 | Wire.read());
  int16_t rawGz = (Wire.read()<<8 | Wire.read());

  ax = rawAx/ACCEL_SCALE - accelOffsetX;
  ay = rawAy/ACCEL_SCALE - accelOffsetY;
  az = rawAz/ACCEL_SCALE - accelOffsetZ;

  gx = rawGx/GYRO_SCALE - gyroXOffset;
  gy = rawGy/GYRO_SCALE - gyroYOffset;
  gz = rawGz/GYRO_SCALE - gyroZOffset;
}

// =====================================================
// ================= MPU CALIBRATION =================
void calibrate_MPU6050(){

  Serial.println("Keep sensor flat...");
  delay(2000);

  double sumAx=0,sumAy=0,sumAz=0;
  double sumGx=0,sumGy=0,sumGz=0;

  for(int i=0;i<200;i++){

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR,14,true);

    int16_t rawAx = (Wire.read()<<8 | Wire.read());
    int16_t rawAy = (Wire.read()<<8 | Wire.read());
    int16_t rawAz = (Wire.read()<<8 | Wire.read());

    Wire.read(); Wire.read();

    int16_t rawGx = (Wire.read()<<8 | Wire.read());
    int16_t rawGy = (Wire.read()<<8 | Wire.read());
    int16_t rawGz = (Wire.read()<<8 | Wire.read());

    sumAx += rawAx/ACCEL_SCALE;
    sumAy += rawAy/ACCEL_SCALE;
    sumAz += rawAz/ACCEL_SCALE;

    sumGx += rawGx/GYRO_SCALE;
    sumGy += rawGy/GYRO_SCALE;
    sumGz += rawGz/GYRO_SCALE;

    delay(10);
  }

  accelOffsetX = sumAx/200;
  accelOffsetY = sumAy/200;
  accelOffsetZ = sumAz/200 - 1;

  gyroXOffset = sumGx/200;
  gyroYOffset = sumGy/200;
  gyroZOffset = sumGz/200;

  Serial.println("MPU Calibration complete");
}

// =====================================================
// ================= QMC5883 =================
void QMC5883_init(){

  Wire.beginTransmission(QMC5883_ADDR);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883_ADDR);
  Wire.write(0x09);
  Wire.write(0x1D);
  Wire.endTransmission();
}

void read_QMC5883(double &mx,double &my,double &mz){

  Wire.beginTransmission(QMC5883_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(QMC5883_ADDR,6);

  int16_t x = Wire.read() | (Wire.read()<<8);
  int16_t y = Wire.read() | (Wire.read()<<8);
  int16_t z = Wire.read() | (Wire.read()<<8);

  mx = x - magOffsetX;
  my = y - magOffsetY;
  mz = z - magOffsetZ;
}

void calibrate_QMC5883(){

  Serial.println("Rotate sensor in air...");
  delay(3000);

  int16_t minX=32767,maxX=-32768;
  int16_t minY=32767,maxY=-32768;
  int16_t minZ=32767,maxZ=-32768;

  unsigned long start=millis();

  while(millis()-start<10000){

    Wire.beginTransmission(QMC5883_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(QMC5883_ADDR,6);

    int16_t x = Wire.read() | (Wire.read()<<8);
    int16_t y = Wire.read() | (Wire.read()<<8);
    int16_t z = Wire.read() | (Wire.read()<<8);

    if(x<minX) minX=x;
    if(x>maxX) maxX=x;

    if(y<minY) minY=y;
    if(y>maxY) maxY=y;

    if(z<minZ) minZ=z;
    if(z>maxZ) maxZ=z;

    delay(100);
  }

  magOffsetX=(maxX+minX)/2.0;
  magOffsetY=(maxY+minY)/2.0;
  magOffsetZ=(maxZ+minZ)/2.0;

  Serial.println("Magnetometer calibration complete");
}

// =====================================================
// ================= KALMAN FILTER =================
double Kalman_update(double &angle,double &bias,double P[2][2],double gyroRate,double accAngle){

  double rate = gyroRate - bias;
  angle += dt * rate;

  P[0][0] += dt*(dt*P[1][1]-P[0][1]-P[1][0]+Q_angle);
  P[0][1] -= dt*P[1][1];
  P[1][0] -= dt*P[1][1];
  P[1][1] += Q_bias*dt;

  double S = P[0][0] + R_measure;

  double K0 = P[0][0]/S;
  double K1 = P[1][0]/S;

  double y = accAngle - angle;

  angle += K0*y;
  bias  += K1*y;

  double P00=P[0][0];
  double P01=P[0][1];

  P[0][0]-=K0*P00;
  P[0][1]-=K0*P01;
  P[1][0]-=K1*P00;
  P[1][1]-=K1*P01;

  return angle;
}