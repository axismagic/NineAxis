#include "Wire.h"

#include "I2Cdev.h"
#include "ADXL345.h"
#include "BMP085.h"
#include "HMC5883L.h"
#include "L3G4200D.h"

//The standard atmosphere: 101325 Pa[1] or 1013.25 millibars or hectopascals. 29.92 inHg
#define SEA_LVL_PRESSURE 101325
#define LED_PIN 13

ADXL345 accel;
BMP085 barometer;
HMC5883L mag;
L3G4200D gyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float temperature;
float pressure;
float altitude;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);
    
    // initialize device
    Serial.println("Initializing I2C devices...");

    accel.initialize();
    boolean accOK = accel.testConnection();
    Serial.println(accOK ? "ADXL345 connection successful" : "ADXL345 connection failed");
    accel.setRange(ADXL345_RANGE_2G); // 256 = 1g
    
    barometer.initialize();
    boolean barOK = barometer.testConnection();
    Serial.println(barOK ? "BMP085 connection successful" : "BMP085 connection failed");

    gyro.initialize();
    boolean gyroOK = gyro.testConnection();
    Serial.println(gyroOK ? "L3G4200D connection successful" : "L3G4200D connection failed");
    gyro.setFullScale(2000);
    
    mag.initialize();
    boolean magOK = mag.testConnection();
    Serial.println(magOK ? "HMC5883L connection successful" : "HMC5883L connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, accOK && barOK && gyroOK && magOK);
}

void loop() {
    accel.getAcceleration(&ax, &ay, &az);
    gyro.getAngularVelocity(&gx, &gy, &gz);
    mag.getHeading(&mx, &my, &mz);
    barometer.setControl(BMP085_MODE_TEMPERATURE);
    delay(barometer.getMeasureDelayMicroseconds()/1000);
    temperature = barometer.getTemperatureC();
    barometer.setControl(BMP085_MODE_PRESSURE_3);
    delay(barometer.getMeasureDelayMicroseconds()/1000);
    pressure = barometer.getPressure();
    altitude = barometer.getAltitude(pressure, SEA_LVL_PRESSURE);

    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(",");
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.print(gz);
    Serial.print(",");
    Serial.print(mx);
    Serial.print(",");
    Serial.print(my);
    Serial.print(",");
    Serial.print(mz);
    Serial.print(",");
    Serial.print(temperature);
    Serial.print(",");
    Serial.print(pressure);
    Serial.print(",");
    Serial.print(altitude);
    Serial.println("");

    delay(20);
}

