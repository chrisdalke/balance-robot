

void setup() {
  // Init serial debugging
  Serial.begin(115200);

  // init esp comms
  Serial1.begin(4800);
  
  // 5v ref for logic level converter
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
}

double serialTargetPitch;

const int bufferSize = 64;
char inputBuffer[bufferSize];
byte bufferIndex = 0;

void loop() {
  // Read bytes from ESP
  while (Serial1.available() > 0) {
    char inByte = Serial1.read();
    if (inByte != '\n') {
      if (bufferIndex < bufferSize - 1) {
        inputBuffer[bufferIndex++] = inByte;
      }
    } else {
      inputBuffer[bufferIndex] = '\0';
      
      // parse as int
      serialTargetPitch = (atoi(inputBuffer) - 128) / 10.0;
      
      bufferIndex = 0;
    }
  }
  Serial.println(serialTargetPitch);
  delay(10);
}
