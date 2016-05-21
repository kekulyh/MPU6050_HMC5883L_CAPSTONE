/*************************************************
File name: MPU6050_HMC5883L_CAPSTONE
Author: Yahong Liu
Version: 5.0
Date: 23/April/2016
Description: Read MPU6050 and HMC5883L accel & gyro & mag data
*************************************************/
#include <Wire.h>
#include <HMC5883L.h>
#include <MPU6050.h>

HMC5883L compass;
MPU6050 mpu;

// vector for reading data
Vector rawAccel;
Vector rawGyro;
Vector mag;

int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int minZ = 0;
int maxZ = 0;
int offX = 0;
int offY = 0;
int offZ = 0;

void setup()
{
  Serial.begin(38400);

//  Serial.println("Initialize MPU6050");

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Enable bypass mode
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true);
  mpu.setSleepEnabled(false);

  // Check MPU6050 offsets
  //origin offset: 1198  -6235 1710  0 0 0
//  checkMPUSettings();

  // Initialize Initialize HMC5883L
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

}

void checkMPUSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Gyroscope:         ");
  switch(mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
  } 

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());

  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());
  
  Serial.println();
}

void loop()
{
  char data;
  
  while ( Serial.available() ){
      
    data = Serial.read();
      
    if(data == 'R'){
      
        // Read acceleration
        rawAccel = mpu.readRawAccel();
        // Read gyro
        rawGyro = mpu.readRawGyro();
      
        // Read nomalized magnetic values (with offsets)
        // value unit is mGa (milli-gauss)
        mag = compass.readNormalize();
        
        // Determine Min / Max values
        if (mag.XAxis < minX) minX = mag.XAxis;
        if (mag.XAxis > maxX) maxX = mag.XAxis;
        if (mag.YAxis < minY) minY = mag.YAxis;
        if (mag.YAxis > maxY) maxY = mag.YAxis;
        if (mag.ZAxis < minZ) minZ = mag.ZAxis;
        if (mag.ZAxis > maxZ) maxZ = mag.ZAxis;
        
        // Calculate offsets
        offX = (maxX + minX)/2;
        offY = (maxY + minY)/2;
        offZ = (maxZ + minZ)/2;
        
        // Set offsets
        compass.setOffset(offX, offY, offZ);
      
        // output to serial
        Serial.print(rawAccel.XAxis/16384.00 - 0.02); Serial.print("\t");
        Serial.print(rawAccel.YAxis/16384.00 + 0.02); Serial.print("\t");
        Serial.print(rawAccel.ZAxis/16384.00); Serial.print("\t");
        Serial.print(rawGyro.XAxis/131.00 + 0.07); Serial.print("\t");
        Serial.print(rawGyro.YAxis/131.00 + 0.09); Serial.print("\t");
        Serial.print(rawGyro.ZAxis/131.00); Serial.print("\t");
        Serial.print(mag.XAxis/1000); Serial.print("\t");
        Serial.print(mag.YAxis/1000); Serial.print("\t");
        Serial.println(mag.ZAxis/1000);
        
        delay(100);
        
      }
      
    }
    
}

void readMpu(Vector rawAccel, Vector rawGyro){
  // Acceleration
  rawAccel = mpu.readRawAccel();
//  Vector normAccel = mpu.readNormalizeAccel();
  
  // Gyro
  rawGyro = mpu.readRawGyro();
//  Vector normGyro = mpu.readNormalizeGyro();

  Serial.print("AccelXraw = ");
  Serial.print(rawAccel.XAxis/16384.00 - 0.03);
  Serial.print(" AccelYraw = ");
  Serial.print(rawAccel.YAxis/16384.00 + 0.03);
  Serial.print(" AccelZraw = ");
  Serial.println(rawAccel.ZAxis/16384.00);
  
//  Serial.print(" AccelXnorm = ");
//  Serial.print(normAccel.XAxis);
//  Serial.print(" AccelYnorm = ");
//  Serial.print(normAccel.YAxis);
//  Serial.print(" AccelZnorm = ");
//  Serial.println(normAccel.ZAxis);

  Serial.print("GyroXraw = ");
  Serial.print(rawGyro.XAxis/131.00 + 0.06);
  Serial.print(" GyroYraw = ");
  Serial.print(rawGyro.YAxis/131.00 + 0.08);
  Serial.print(" GyroZraw = ");
  Serial.println(rawGyro.ZAxis/131.00 - 0.01);

//  Serial.print(" GyroXnorm = ");
//  Serial.print(normGyro.XAxis);
//  Serial.print(" GyroYnorm = ");
//  Serial.print(normGyro.YAxis);
//  Serial.print(" GyroZnorm = ");
//  Serial.println(normGyro.ZAxis);
}

void readMag(Vector mag){
  // Read nomalized value (with offsets)
  // value unit is mGa (milli-gauss)
  mag = compass.readNormalize();
  
  // Determine Min / Max values
  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;
  if (mag.ZAxis < minZ) minZ = mag.ZAxis;
  if (mag.ZAxis > maxZ) maxZ = mag.ZAxis;
  
  // Calculate offsets
  offX = (maxX + minX)/2;
  offY = (maxY + minY)/2;
  offZ = (maxZ + minZ)/2;
  
  // Set offsets
  compass.setOffset(offX, offY, offZ);
  
  // Calculate heading
  float heading = atan2(mag.YAxis, mag.XAxis);
  
  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Sydney:Magnetic declination: +12° 34'(POSITIVE (EAST))
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (12.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }
 
  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/PI;

  // we need gauss unit, so convert mGa to Ga.
  Serial.print("Heading:");Serial.print(heading);
  Serial.print(" mag.XAxis:");Serial.print(mag.XAxis/1000);
  Serial.print(" mag.YAxis:");Serial.print(mag.YAxis/1000);
  Serial.print(" mag.ZAxis:");Serial.println(mag.ZAxis/1000);
  
  Serial.print("minX:");Serial.print(minX);
  Serial.print(" maxX:");Serial.print(maxX);
  Serial.print(" minY:");Serial.print(minY);
  Serial.print(" maxY:");Serial.print(maxY);
  Serial.print(" minZ:");Serial.print(minZ);
  Serial.print(" maxZ:");Serial.print(maxZ);
  Serial.print(" offX:");Serial.print(offX);
  Serial.print(" offY:");Serial.print(offY);
  Serial.print(" offZ:");Serial.println(offZ);
}
