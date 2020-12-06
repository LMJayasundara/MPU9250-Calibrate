#include <SparkFunMPU9250-DMP.h>
MPU9250_DMP imu;

double ss_mag_x, ss_mag_y, ss_mag_z;
double ss_gyro_x, ss_gyro_y, ss_gyro_z;
double ss_accel_x, ss_accel_y, ss_accel_z;

float dest1[3] = {0, 0, 0}, dest2[3] = {0, 0, 0};

void setup() 
{
  Serial.begin(115200);

  // Call imu.begin() to verify communication with and
  // initialize the MPU-9250 to it's default values.
  // Most functions return an error code - INV_SUCCESS (0)
  // indicates the IMU was present and successfully set up
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }

  // Use setSensors to turn on or off MPU-9250 sensors.
  // Any of the following defines can be combined:
  // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
  // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
  // Enable all sensors:
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(5); // Set LPF corner frequency to 5Hz

  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(10); // Set sample rate to 10Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz

  Serial.println("Initialize variables");
  Initialize_variables();
  delay(1000);
  Serial.println("Calibrate Accel & Gyro");
  Calibrate();
  delay(3000);
  Serial.println("Calibrate Mag");
  CalibrateMag();
  delay(2000);
  Serial.println("Done");
  delay(1000);

  // Serial.println("acce x,acce y,acce z,gyro x,gyro y,gyro z,mag x,mag y,mag z");
  Serial.println("mag x,mag y,mag z");
}

void loop() 
{
  // dataReady() checks to see if new accel/gyro data
  // is available. It will return a boolean true or false
  // (New magnetometer data cannot be checked, as the library
  //  runs that sensor in single-conversion mode.)
    
  if ( imu.dataReady() )
  {
    
    // Call update() to update the imu objects sensor data.
    // You can specify which sensors to update by combining
    // UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
    // UPDATE_TEMPERATURE.
    // (The update function defaults to accel, gyro, compass,
    //  so you don't have to specify these values.)
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMUData();
  }
    
}

void printIMUData(void)
{  
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  float accelX = imu.calcAccel(imu.ax) - ss_accel_x;
  float accelY = imu.calcAccel(imu.ay) - ss_accel_y;
  float accelZ = imu.calcAccel(imu.az) - ss_accel_z;
  
  float gyroX = imu.calcGyro(imu.gx) - ss_gyro_x;
  float gyroY = imu.calcGyro(imu.gy) - ss_gyro_y;
  float gyroZ = imu.calcGyro(imu.gz) - ss_gyro_z;
  
  float magX = ((imu.calcMag(imu.mx) - ss_mag_x) - dest1[0]) * dest2[0];
  float magY = ((imu.calcMag(imu.my) - ss_mag_y) - dest1[1]) * dest2[1];
  float magZ = ((imu.calcMag(imu.mz) - ss_mag_z) - dest1[2]) * dest2[2];
  
  //  Serial.print(accelX); Serial.print(","); Serial.print(accelY); Serial.print(","); Serial.print (accelZ); Serial.print(","); // print all data
  //  Serial.print(gyroX); Serial.print(","); Serial.print(gyroY); Serial.print(","); Serial.print (gyroZ); Serial.print(",");
  Serial.print(magX); Serial.print(","); Serial.print(magY); Serial.print(","); Serial.print (magZ); Serial.print(",");
  Serial.println();
  
}

//initial values for global variables
void Initialize_variables(){

    ss_mag_x = 0;
    ss_mag_y = 0;
    ss_mag_z = 0;
  
    ss_gyro_x = 0;
    ss_gyro_y = 0;
    ss_gyro_z = 0;
    
    ss_accel_x = 0;
    ss_accel_y = 0;
    ss_accel_z = 0;
    
}

void Calibrate(void){
  unsigned int count1;
  count1 = 0;
    
  if ( imu.dataReady() ) {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

    do{ //accumulate samples
      
      ss_mag_x = ss_mag_x + imu.calcMag(imu.mx);
      ss_mag_y = ss_mag_y + imu.calcMag(imu.my);
      ss_mag_z = ss_mag_z + imu.calcMag(imu.mz);
      
      ss_gyro_x = ss_gyro_x + imu.calcGyro(imu.gx);
      ss_gyro_y = ss_gyro_y + imu.calcGyro(imu.gy);
      ss_gyro_z = ss_gyro_z + imu.calcGyro(imu.gz);
      
      ss_accel_x = ss_accel_x + imu.calcAccel(imu.ax);
      ss_accel_y = ss_accel_y + imu.calcAccel(imu.ay);
      ss_accel_z = ss_accel_z + imu.calcAccel(imu.az);
      count1++;
      
    }while(count1!=1024); //1024 times 

    //average the samples
    ss_mag_x = ss_mag_x/count1;
    ss_mag_y = ss_mag_y/count1;
    ss_mag_z = ss_mag_z/count1;
    
    ss_gyro_x = ss_gyro_x/count1;
    ss_gyro_y = ss_gyro_y/count1;
    ss_gyro_z = ss_gyro_z/count1;
    
    ss_accel_x = ss_accel_x/count1;                
    ss_accel_y = ss_accel_y/count1;
    ss_accel_z = ss_accel_z/count1;

    // Serial.println(String(ss_accel_x) + ", " + String(ss_accel_y) + ", " + String(ss_accel_z));
    
  }
}

void readMagData(int16_t * destination)
{
  if ( imu.dataReady() ) {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

    destination[0] = imu.calcMag(imu.mx) - ss_mag_x;
    destination[1] = imu.calcMag(imu.my) - ss_mag_y;
    destination[2] = imu.calcMag(imu.mz) - ss_mag_z;
  }
}

void CalibrateMag()
{
   uint16_t ii = 0, sample_count = 0;
   int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
   int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
 
   Serial.println("Mag Calibration: Wave device in a figure eight until done!");
   delay(4000);
  
   sample_count = 1500;
   for(ii = 0; ii < sample_count; ii++) {
   readMagData(mag_temp);  // Read the mag data   
   for (int jj = 0; jj < 3; jj++) {
     if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
     if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
   }
     delay(12);
   }

    // Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    // Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    // Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;

    dest1[0] = (float) mag_bias[0];
    dest1[1] = (float) mag_bias[1];
    dest1[2] = (float) mag_bias[2];
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
  
    Serial.println("Mag Calibration done!");
}
