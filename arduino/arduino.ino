#include <Wire.h>
#include <SoftwareSerial.h>
#include "def.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "BMP085.h"
#include "ArduinoJson.h"

int N_SAMPLE = 1;

MPU6050 mpu6050;
HMC5883L hmc5883l;
BMP085 bmp085;
Measure meas;
int64_t lastMicros = 0;
SoftwareSerial piSerial(10, 11);
char indata[100];
float testval;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  piSerial.begin(9600);
  Wire.begin();
  mpu6050.initialize();
  Serial.println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  mpu6050.setI2CBypassEnabled(true);
  mpu6050.setI2CMasterModeEnabled(false);

  hmc5883l.initialize();
  Serial.println(hmc5883l.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  bmp085.initialize();
  Serial.println(bmp085.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");

  bmp085.setControl(BMP085_MODE_TEMPERATURE);

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
  int pos = 0;
  int cnt = 0;
  while(piSerial.available() > 0)
  {
    int c = piSerial.read();
    piSerial.print((unsigned int8_t)(c));
    piSerial.print(",");
    cnt += 1;
    indata[pos] = c;
    pos += 1;
    if(c == '\n' || piSerial.available() <= 0)
    {
      indata[pos-1] = '\0';
      pos = 0;
    }
  }

  piSerial.println("");
  piSerial.println(cnt);
  /* piSerial.println(indata); */

}

void print_json(Measure &meas)
{
  StaticJsonBuffer<256> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["time"] = micros();

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
  /* piSerial.print(micros()); */
  piSerial.println("");
}

void print_data(Measure &meas)
{

  Serial.print(F("Time : "));
  Serial.print(micros());
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

int scnt = 0;
Measure ms;
void loop()
{
  piSerial.println("\"A\"");
  read_json();
  ms += read_data();
  scnt++;
  if(scnt == N_SAMPLE)
  {
    scnt = 0;
    ms *= 1.0 / N_SAMPLE;
    /* print_json(ms); */
    ms = Measure();
    delay(1000);
  }
}
