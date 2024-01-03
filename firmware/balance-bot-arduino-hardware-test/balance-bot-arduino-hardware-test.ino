#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

int i =0;
unsigned long last_time = 0;
float pitch;

const int bufferSize = 64;
char inputBuffer[bufferSize];
byte bufferIndex = 0;
int steeringRateRaw = 0;
float serialTargetPitch = 0.0f;

void setup() {
  // Init serial debugging
  Serial.begin(115200);

  // init esp comms
  Serial1.begin(9600);
  
  // Init IMU
  bno.begin();
  delay(100);
  bno.setExtCrystalUse(true);
}

void loop() {
  // Get latest pose from BNO055
  sensors_event_t orentationData, angVelocityData;
  bno.getEvent(&orentationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  pitch = orentationData.orientation.z;
  
  // Read bytes from ESP
  while (Serial1.available() > 0) {
    char inByte = Serial1.read();
    i++;
    if (inByte != '\n') {
      if (bufferIndex < bufferSize - 1) {
        inputBuffer[bufferIndex++] = inByte;
      }
    } else {
      inputBuffer[bufferIndex] = '\0';
      
      // parse controls as ints
      char *token = strtok(inputBuffer, ",");
      if (token != NULL) {
        serialTargetPitch = (atoi(inputBuffer) - 128) / 10.0;
        token = strtok(NULL, ",");
        if (token != NULL) {
          steeringRateRaw = atoi(token);
        }
      }
      
      bufferIndex = 0;
    }
  }
  
  if (millis() - last_time > 10) {
      last_time = millis();
  }
}
