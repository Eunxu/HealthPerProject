//#include <OzOLED.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double pastValueX, pastValueY;
double getminus;
double boxCal ;
double determin ;
int count ;
int16_t tempRaw;

int countP=0 ;
long startTime ;                    // 스탑워치의 시작 시간
long stopTime ;
long ppgValue ;
long yinterVal =200 ;
int flag = 0;
int Signal ;
long xinterVal = millis();
long compareValue = -0.5*xinterVal+yinterVal; // Signal함수와 비교를 위한 1차 함수 


double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  

  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100); // Wait for sensor to stabilize
  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}


void loop() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  
  /* Print Data */
  //Serial.print(roll); Serial.print("\t");
  //Serial.print(gyroXangle); Serial.print("\t");
  //Serial.print(compAngleX); Serial.print("\t");
  Calcu(kalAngleX);

  Signal = analogRead(A0)*20;
  compare(compareValue,Signal); 
  if(countP==1){
    startTime = millis();
  } // count가 시작된 시점의 시간을 저장함
  if(countP==10){  
    stopTime = millis();
   PPG();
  } // count가 10번 되었을 시점의 시간을 저장하고 맥박계산을 위한 함수를 넘어감  
  
  //Serial.print(count); Serial.print("\t");
  Serial.print(determin); Serial.print("\t");
  //Serial.print(determin); Serial.print("\t");
  //Serial.print(getminus); Serial.print("\t");
  Serial.print("\r\n");
  /*  
  OzOled.printString("count:", 0, 0); //Print the String 
  OzOled.printString("Determin", 0, 1); //Print the String 
  OzOled.printString("Getminus", 0, 2); //Print the String
  OzOled.printString("PPG", 0, 3); //Print the String  
  OzOled.printNumber((long)count, 10, 0); //Print the String
  OzOled.printNumber((long)determin, 10, 1); //Print the String
  OzOled.printNumber((long)getminus, 10, 2); //Print the String
  OzOled.printNumber((long)ppgValue, 10, 3); //Print the String 
  */     
  //Serial.print(pastValueX); Serial.print("\t");
  //Serial.print("\t");
  //Serial.print(pitch); Serial.print("\t");
  //Serial.print(gyroYangle); Serial.print("\t");
  //Serial.print(compAngleY); Serial.print("\t");
  //Serial.print(kalAngleY); Serial.print("\t");  
  //Serial.print("\r\n");
  delay(50);
  pastValueX = kalAngleX;
  pastValueY = kalAngleY;
}
void Calcu(double boxV){  
    getminus=pastValueX-boxV;
    miBun(getminus);    
}
void miBun(double getminuss){
    determin =  boxCal-getminuss/50;
    
   if(abs(getminuss)>4){ 
   if(determin>(-0.5)&&determin<0.5){
       count = count + 1;
   } 
  }  
  boxCal = getminuss ;
}
void PPG(){
    long interVal = stopTime - startTime;
    ppgValue = 60000/interVal ;
    //Serial.print(ppgValue);
    countP = 0; 
  } // count가 시작될때와 10번 끝낫을 시점의 시간을 이용하여 맥박수를 계산

void compare(long compareValue, int Signal){
 
if(compareValue<Signal&&flag==0){
    countP = countP +1 ;
    flag = 1;
  } // 1차 함수와 signal을 비교하여 signal이 더 큰 값일 경우 count를 올려줌.
if(compareValue<Signal&&flag==1){
    compareValue = Signal ;
  } // 1차 함수가 signal 보다 지속적으로 커질 경우 count를 하지 안으며 값을 재설정하기 위한 코딩
if(compareValue>Signal&&flag==1){
  flag = 0 ;
  yinterVal = compareValue ;
  } // signal 함수가 1차함수 보다 작을 경우 flag를 다시 반환 시키며

  //Serial.print(Signal); Serial.print("\t");
  //Serial.print(compareValue); Serial.print("\t");
  //Serial.print("\r\n") ;
       
}//#include <OzOLED.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double pastValueX, pastValueY;
double getminus;
double boxCal ;
double determin ;
int count ;
int16_t tempRaw;

int countP=0 ;
long startTime ;                    // 스탑워치의 시작 시간
long stopTime ;
long ppgValue ;
long yinterVal =200 ;
int flag = 0;
int Signal ;
long xinterVal = millis();
long compareValue = -0.5*xinterVal+yinterVal; // Signal함수와 비교를 위한 1차 함수 


double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  

  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100); // Wait for sensor to stabilize
  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}


void loop() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  
  /* Print Data */
  //Serial.print(roll); Serial.print("\t");
  //Serial.print(gyroXangle); Serial.print("\t");
  //Serial.print(compAngleX); Serial.print("\t");
  Calcu(kalAngleX);

  Signal = analogRead(A0)*20;
  compare(compareValue,Signal); 
  if(countP==1){
    startTime = millis();
  } // count가 시작된 시점의 시간을 저장함
  if(countP==10){  
    stopTime = millis();
   PPG();
  } // count가 10번 되었을 시점의 시간을 저장하고 맥박계산을 위한 함수를 넘어감  
  
  //Serial.print(count); Serial.print("\t");
  Serial.print(determin); Serial.print("\t");
  //Serial.print(determin); Serial.print("\t");
  //Serial.print(getminus); Serial.print("\t");
  Serial.print("\r\n");
  /*  
  OzOled.printString("count:", 0, 0); //Print the String 
  OzOled.printString("Determin", 0, 1); //Print the String 
  OzOled.printString("Getminus", 0, 2); //Print the String
  OzOled.printString("PPG", 0, 3); //Print the String  
  OzOled.printNumber((long)count, 10, 0); //Print the String
  OzOled.printNumber((long)determin, 10, 1); //Print the String
  OzOled.printNumber((long)getminus, 10, 2); //Print the String
  OzOled.printNumber((long)ppgValue, 10, 3); //Print the String 
  */     
  //Serial.print(pastValueX); Serial.print("\t");
  //Serial.print("\t");
  //Serial.print(pitch); Serial.print("\t");
  //Serial.print(gyroYangle); Serial.print("\t");
  //Serial.print(compAngleY); Serial.print("\t");
  //Serial.print(kalAngleY); Serial.print("\t");  
  //Serial.print("\r\n");
  delay(50);
  pastValueX = kalAngleX;
  pastValueY = kalAngleY;
}
void Calcu(double boxV){  
    getminus=pastValueX-boxV;
    miBun(getminus);    
}
void miBun(double getminuss){
    determin =  boxCal-getminuss/50;
    
   if(abs(getminuss)>4){ 
   if(determin>(-0.5)&&determin<0.5){
       count = count + 1;
   } 
  }  
  boxCal = getminuss ;
}
void PPG(){
    long interVal = stopTime - startTime;
    ppgValue = 60000/interVal ;
    //Serial.print(ppgValue);
    countP = 0; 
  } // count가 시작될때와 10번 끝낫을 시점의 시간을 이용하여 맥박수를 계산

void compare(long compareValue, int Signal){
 
if(compareValue<Signal&&flag==0){
    countP = countP +1 ;
    flag = 1;
  } // 1차 함수와 signal을 비교하여 signal이 더 큰 값일 경우 count를 올려줌.
if(compareValue<Signal&&flag==1){
    compareValue = Signal ;
  } // 1차 함수가 signal 보다 지속적으로 커질 경우 count를 하지 안으며 값을 재설정하기 위한 코딩
if(compareValue>Signal&&flag==1){
  flag = 0 ;
  yinterVal = compareValue ;
  } // signal 함수가 1차함수 보다 작을 경우 flag를 다시 반환 시키며

  //Serial.print(Signal); Serial.print("\t");
  //Serial.print(compareValue); Serial.print("\t");
  //Serial.print("\r\n") ;
       
}
