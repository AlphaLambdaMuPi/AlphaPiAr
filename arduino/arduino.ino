#include <Wire.h>
#include <SoftwareSerial.h>
#include "def.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "ArduinoJson.h"
#include "Servo.h"

MPU6050 mpu6050;
HMC5883L hmc5883l;
BMP085 bmp085;
Measure meas;
Servo servo[4];
const int SERVO_PIN[4] = {9, 10, 11, 12};
int64_t lastMicros = 0;
double global_time_offset = 0;
/* SoftwareSerial piSerial(10, 11); */
#define piSerial Serial
char indata[256];
int motor[4];
int pos;

void setup() {
  // put your setup code here, to run once:
  /* Serial.begin(115200); */
  piSerial.begin(115200);
  Wire.begin();
  mpu6050.initialize();
  /* Serial.println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed"); */
  
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
    servo[i].writeMicroseconds(0);
  }
}

double gettime()
{
  return global_time_offset + micros() * 1E-6;
}

Measure& read_data() {

  int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  hmc5883l.getHeading(&mx, &my, &mz);

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

  meas.accel.x = ax;
  meas.accel.y = ay;
  meas.accel.z = az;
  meas.accel *= 9.8 * 2 / 32768;

  meas.gyro.x = gx;
  meas.gyro.y = gy;
  meas.gyro.z = gz;
  meas.gyro *= 3.14159265 / 180 * 250 / 32768;

  meas.mag.x = mx;
  meas.mag.y = my;
  meas.mag.z = mz;
  meas.mag *= 1 / meas.mag.len();

  return meas;
}

void read_json()
{
  if(piSerial.available() == 0)
    return;
  int cnt = 0;
  while(piSerial.available() > 0)
  {
    cnt++;
    int c = piSerial.read();
    indata[pos] = c;
    pos += 1;
    if(c == '\n')
    {
      indata[pos-1] = '\0';
      pos = 0;
    }
  }

  if(pos > 0) return;

  /* piSerial.print("Total Cnt = "); */
  /* piSerial.println(cnt); */
  /* piSerial.print("Raw data = "); */
  /* piSerial.println(indata); */

  piSerial.println(indata);
  piSerial.flush();

  //Parse Json
  StaticJsonBuffer<256> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(indata);
  if(!root.success()) return;
  if(!root.containsKey("motor")) return;
  if(!root["motor"].is<JsonArray&>()) return;
  JsonArray& marr = root["motor"];
  for(int i=0; i<4; i++)
    motor[i] = marr[i];
  
  /* for(int i=0; i<4; i++) */
  /* { */
    /* piSerial.print(motor[i]); */
    /* piSerial.print(","); */
  /* } */
  /* piSerial.println(""); */
  /* piSerial.flush(); */
}

void set_motor()
{
  for(int i=0; i<4; i++)
  {
    servo[i].writeMicroseconds(motor[i]);
  }
}

void print_json(Measure &meas)
{
  StaticJsonBuffer<256> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["time"].set(gettime(), 6);
  root["timez"].set(global_time_offset, 6);

  JsonArray& arr_acc = root.createNestedArray("accel");
  arr_acc.add(meas.accel.x, 3);
  arr_acc.add(meas.accel.y, 3);
  arr_acc.add(meas.accel.z, 3);

  JsonArray& arr_gyro = root.createNestedArray("gyro");
  arr_gyro.add(meas.gyro.x, 3);
  arr_gyro.add(meas.gyro.y, 3);
  arr_gyro.add(meas.gyro.z, 3);

  JsonArray& arr_mag = root.createNestedArray("mag");
  arr_mag.add(meas.mag.x, 3);
  arr_mag.add(meas.mag.y, 3);
  arr_mag.add(meas.mag.z, 3);

  root["temp"].set(meas.temperature, 2);
  root["pressure"].set(meas.pressure, 0);

  root.printTo(piSerial);
  piSerial.println("");
  
  piSerial.flush();
}

void print_data(Measure &meas)
{
  Serial.print(F("Time : "));
  Serial.print(gettime(), 6);
  Serial.println(F(""));

  Serial.print(F("accel : "));
  Serial.print(meas.accel.x, 2);
  Serial.print(F(" , "));
  Serial.print(meas.accel.y, 2);
  Serial.print(F(" , "));
  Serial.print(meas.accel.z, 2);
  Serial.println(F(""));

  Serial.print(F("gyro : "));
  Serial.print(meas.gyro.x, 2);
  Serial.print(F(" , "));
  Serial.print(meas.gyro.y, 2);
  Serial.print(F(" , "));
  Serial.print(meas.gyro.z, 2);
  Serial.println(F(""));

  Serial.print(F("mag : "));
  Serial.print(meas.mag.x, 3);
  Serial.print(F(" , "));
  Serial.print(meas.mag.y, 3);
  Serial.print(F(" , "));
  Serial.print(meas.mag.z, 3);
  Serial.println(F(""));

  Serial.print(F("temp : "));
  Serial.print(meas.temperature, 2);
  Serial.println(F(""));

  Serial.print(F("pressure : "));
  Serial.print(meas.pressure, 0);
  Serial.println(F(""));
}

void loop()
{
  read_json();
  set_motor();
  Measure ms = read_data();
  print_json(ms);
  ms = Measure();
  /* delay(1000); */
}
