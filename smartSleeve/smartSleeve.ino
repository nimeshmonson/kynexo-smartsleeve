#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

#define LSM9DS1_SCK 13
#define LSM9DS1_MISO 2
#define LSM9DS1_MOSI 11
#define LSM9DS1_XGCS1 10
#define LSM9DS1_MCS 9
#define LSM9DS1_XGCS2 6
#define LSM9DS1_XGCS3 5
#define LSM9DS1_XGCS4 3

//struct to read sensor values
int backPin = 4;
int armPin1 = 8;
int armPin2 = 7;
int stretchPin = 12;

int incomingByte = 0;
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int minValue = 0;
int maxValue = 0;
int minFlag = 0;
int maxFlag = 0;

struct sensorVal{
  float ax;
  float ay;
  float az;
};

struct sensors{
  sensorVal value;
  float xWithReference;
  float yWithReference;
  float zWithReference;
};
int flag; 
int backz;

//Adafruit_LSM9DS1 forearm;
Adafruit_LSM9DS1 reference;
Adafruit_LSM9DS1 arm;
Adafruit_LSM9DS1 back;  

sensors referenceSensorData,backSensorData,armSensorData,forearmSensorData;
sensors_event_t a, m, g, temp;
//for setting up the sensors

void setupSensor()
{
}

void setup() 
{
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  Serial.println("LSM9DS1 data read demo");
  pinMode(backPin, OUTPUT);
  pinMode(armPin1, OUTPUT);
  pinMode(armPin2, OUTPUT);
  pinMode(stretchPin, OUTPUT);
  pinMode(analogInPin,INPUT);
  flag = 0;
  
}

void loop() 
{  
  if(flag == 0){
    
    //forearm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS1, LSM9DS1_MCS);
    reference = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS2, LSM9DS1_MCS);
    arm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS3, LSM9DS1_MCS);
    back = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS4, LSM9DS1_MCS);
  
    int setupSensor = 0;
    
    /*if (!forearm.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS1 on forearm. Check your wiring!");
      setupSensor = 1;
    }
    else{
      Serial.println("Found LSM9DS1 9DOF forearm");
    }*/
    if (!reference.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS1 on reference. Check your wiring!");
      setupSensor = 1;
    }
    else{
      Serial.println("Found LSM9DS1 9DOF reference");
    }
    if (!arm.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS1 on arm. Check your wiring!");
      setupSensor = 1;
    }
    else{
      Serial.println("Found LSM9DS1 9DOF arm");
    }
    
    if (!back.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS1 on back. Check your wiring!");
      setupSensor = 1;
    }
    else{
      Serial.println("Found LSM9DS1 9DOF back");
    }
    if(setupSensor == 1){
      Serial.println("Please check your wiring");
      while(1);
    }
    //forearm.setupAccel(forearm.LSM9DS1_ACCELRANGE_2G);
    arm.setupAccel(arm.LSM9DS1_ACCELRANGE_2G);
    back.setupAccel(back.LSM9DS1_ACCELRANGE_2G);
    reference.setupAccel(reference.LSM9DS1_ACCELRANGE_2G);
  
    //forearm.setupMag(forearm.LSM9DS1_MAGGAIN_4GAUSS);
    arm.setupMag(arm.LSM9DS1_MAGGAIN_4GAUSS);
    back.setupMag(back.LSM9DS1_MAGGAIN_4GAUSS);
    reference.setupMag(reference.LSM9DS1_MAGGAIN_4GAUSS);
  
    //forearm.setupGyro(forearm.LSM9DS1_GYROSCALE_245DPS);
    arm.setupGyro(arm.LSM9DS1_GYROSCALE_245DPS);
    back.setupGyro(back.LSM9DS1_GYROSCALE_245DPS);
    reference.setupGyro(reference.LSM9DS1_GYROSCALE_245DPS);
  
    flag = 1;
    //calibrateStretchSensor();
  }
  //forearm.read();  
  reference.read();
  arm.read();
  back.read();

  getValues(reference, arm, back);

 /* Serial.println("Forearm Sensor Readings with referece");
  Serial.print("Accel X: "); Serial.print(forearmSensorData.xWithReference); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(forearmSensorData.yWithReference);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(forearmSensorData.zWithReference);     Serial.println(" m/s^2 ");
*/
  /*Serial.println("Arm Sensor Readings with referece");
  Serial.print("Accel X: "); Serial.print(armSensorData.xWithReference); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(armSensorData.yWithReference);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(armSensorData.zWithReference);     Serial.println(" m/s^2 ");*/

  Serial.println("Back Sensor Readings with referece");
  Serial.print("Accel X: "); Serial.print(backSensorData.xWithReference); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(backSensorData.yWithReference);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(backSensorData.zWithReference);     Serial.println(" m/s^2 ");
  
  sensorValue = analogRead(analogInPin);
  outputValue = map(sensorValue, minValue, maxValue, 0, 10);
  //Serial.println(sensorValue);
  if(backSensorData.yWithReference < 18.6){
    digitalWrite(backPin,HIGH);
    if(armSensorData.zWithReference > 4.5|| armSensorData.zWithReference < -0.5){
      digitalWrite(armPin2,LOW);
    }
    if(armSensorData.yWithReference < 18.65){
      digitalWrite(armPin1,LOW);
    }
  }
  else{
    digitalWrite(backPin,LOW);
    if(armSensorData.zWithReference > 4.5|| armSensorData.zWithReference < -0.5){
      digitalWrite(armPin2,HIGH);
      if(armSensorData.yWithReference < 18.5){
        digitalWrite(armPin1,LOW);
      }
    }
    else{
     digitalWrite(armPin2,LOW);
      if(armSensorData.yWithReference < 18.5){
        digitalWrite(armPin1,HIGH);
      }
      else{
       digitalWrite(armPin1,LOW);
      }
    } 
  }
  
  if(sensorValue < 0 ){
    digitalWrite(stretchPin,HIGH);
  }
  else{
    digitalWrite(stretchPin,LOW);
  }
  delay(200);
  backz = backSensorData.zWithReference;
}

void getValues(Adafruit_LSM9DS1 reference, Adafruit_LSM9DS1 arm, Adafruit_LSM9DS1 back){
  reference.getEvent(&a, &m, &g, &temp);
  referenceSensorData.value.ax = a.acceleration.x;
  referenceSensorData.value.ay = a.acceleration.y;
  referenceSensorData.value.az = a.acceleration.z;

  back.getEvent(&a, &m, &g, &temp);
  backSensorData.value.ax = a.acceleration.x;
  backSensorData.value.ay = a.acceleration.y;
  backSensorData.value.az = a.acceleration.z;
  backSensorData.xWithReference = backSensorData.value.ax - referenceSensorData.value.ax;
  backSensorData.yWithReference = backSensorData.value.ay - referenceSensorData.value.ay;
  backSensorData.zWithReference = backSensorData.value.az - referenceSensorData.value.az;
  
  arm.getEvent(&a, &m, &g, &temp);
  armSensorData.value.ax = a.acceleration.x;
  armSensorData.value.ay = a.acceleration.y;
  armSensorData.value.az = a.acceleration.z;
  armSensorData.xWithReference = armSensorData.value.ax - referenceSensorData.value.ax;
  armSensorData.yWithReference = armSensorData.value.ay - referenceSensorData.value.ay;
  armSensorData.zWithReference = armSensorData.value.az - referenceSensorData.value.az;
  
  /*forearm.getEvent(&a, &m, &g, &temp);
  forearmSensorData.value.ax = a.acceleration.x;
  forearmSensorData.value.ay = a.acceleration.y;
  forearmSensorData.value.az = a.acceleration.z;
  forearmSensorData.xWithReference = forearmSensorData.value.ax - armSensorData.value.ax;
  forearmSensorData.yWithReference = forearmSensorData.value.ay - armSensorData.value.ay;
  forearmSensorData.zWithReference = forearmSensorData.value.az - armSensorData.value.az;*/
}

int calibrateStretchSensor(){

   Serial.println("\nTaking readings in");
    delay(1000);
    Serial.print("3 ");
    delay(1000);
    Serial.print("2 ");
    delay(1000);
    Serial.println("1 ");
    delay(1000);
    maxValue = analogRead(analogInPin);
}
