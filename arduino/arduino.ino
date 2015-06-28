#include <Wire.h>
#include <SoftwareSerial.h>
#include "def.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "Servo.h"

MPU6050 mpu6050;
HMC5883L hmc5883l;
BMP085 bmp085;
Measure meas;
Servo servo[4];
const int SERVO_PIN[4] = {12, 11, 10, 9};
int64_t lastMicros = 0;
double global_time_offset = 0;
/* SoftwareSerial piSerial(10, 11); */
#define piSerial Serial
short motor[4];

void setup() {
  /* Serial.begin(115200); */
  piSerial.begin(115200);
  Wire.begin();
  mpu6050.initialize();
  /* Serial.println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed"); */
  
  mpu6050.setDLPFMode(4); // 4 : 20Hz
  mpu6050.setI2CBypassEnabled(true);
  mpu6050.setI2CMasterModeEnabled(false);

  hmc5883l.initialize();
  /* Serial.println(hmc5883l.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed"); */

  bmp085.initialize();
  /* Serial.println(bmp085.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed"); */

  bmp085.setControl(BMP085_MODE_TEMPERATURE);

  for(int i=0; i<4; i++)
  {
    servo[i].attach(SERVO_PIN[i]);
    servo[i].writeMicroseconds(1000);
  }
}

double gettime()
{
  return global_time_offset + micros() * 1E-6;
}

void swf(float f) //serial write float
{
  Serial.write((char*)&f, 4);
}

void sws(short s) //serial write short
{
  Serial.write((char*)&s, 2);
}

void srs(short &s) //serial read short
{
  Serial.readBytes((char*)&s, 2);
}

Measure& read_data(bool prn=false)
{
  int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  meas.accel.x = ax;
  meas.accel.y = ay;
  meas.accel.z = az;
  meas.accel *= 9.8 * 2 / 32768;
  meas.gyro.x = gx;
  meas.gyro.y = gy;
  meas.gyro.z = gz;
  meas.gyro *= 3.14159265 / 180 * 250 / 32768;

  if(prn)
  {
    sws((short)ax); sws((short)ay); sws((short)az);
    sws((short)gx); sws((short)gy); sws((short)gz);
  }

  hmc5883l.getHeading(&mx, &my, &mz);

  meas.mag.x = mx;
  meas.mag.y = my;
  meas.mag.z = mz;
  meas.mag *= 1 / meas.mag.len();

  if(prn)
  {
    sws((short)mx); sws((short)my); sws((short)mz);
  }

  if(micros() - lastMicros > bmp085.getMeasureDelayMicroseconds())
  {
    if(bmp085.getControl() == 10)
    {
      meas.temperature = bmp085.getTemperatureC();
      bmp085.setControl(BMP085_MODE_PRESSURE_3);
    }
    else
    {
      meas.pressure = bmp085.getPressure();
      bmp085.setControl(BMP085_MODE_TEMPERATURE);
    }
    lastMicros = micros();
  }

  if(prn)
  {
    swf(meas.temperature); swf(meas.pressure);
  }

  float voltage = analogRead(1) * 5. / 1024.;
  float current = analogRead(2) * 5. / 1024.;
  
  if(prn)
  {
    swf(voltage); swf(current);
  }

  return meas;
}

void set_motor()
{
  for(int i=0; i<4; i++)
  {
    servo[i].writeMicroseconds(motor[i]);
  }
}

void loop()
{
  if(!Serial.available()) return;
  int c = Serial.read();

  if(c == 'R')
  {
    Measure &ms = read_data(true);
  }
  else if(c == 'M')
  {
    for(int i=0; i<4; i++)
    {
      srs(motor[i]);
      motor[i] = min(motor[i], 2000);
      motor[i] = max(motor[i], 1000);
    }
    set_motor();
    Serial.write('m');
  }
  else if(c == 'S')
  {
    Serial.write('s');
  }
}

