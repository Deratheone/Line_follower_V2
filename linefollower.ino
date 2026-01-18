const int AIN1 = 7;
const int AIN2 = 8;
const int PWMA = 5;
const int BIN1 = 9;
const int BIN2 = 11;
const int PWMB = 6;

const int sensorPins[8] = {A6, A0, A1, A2, A3, A4, A5, A7};
const int numSensors = 8;
const int buttonPin = 13;

const int BLUE_LED = 2;
const int RED_LED = 12;
const int GREEN_LED = 4;

int sensorMin[numSensors];
int sensorMax[numSensors];
int sensorValues[numSensors];

float Kp = 0.319;  
float Ki = 0.0;
float Kd = 2.9;

float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;

int baseSpeed = 120;
int maxSpeed = 150;
int minSpeed = -150;
int leftSpeed, rightSpeed;

int leftMotorTrim = 0;
int rightMotorTrim = 0;
const unsigned long CALIBRATION_TIME = 5000;
const int LINE_DETECTION_THRESHOLD = 500;
const int CHECKPOINT_THRESHOLD = 750;

float lastValidPosition = 3500;
unsigned long lastLineTime = 0;
const unsigned long LINE_LOST_TIMEOUT = 100;

unsigned long allBlackStartTime = 0;
bool allBlackDetected = false;
const unsigned long CHECKPOINT_DURATION = 80;
int checkpointCount = 0;
unsigned long lastCheckpointTime = 0;
const unsigned long CHECKPOINT_COOLDOWN = 2500;
int ignoreCheckpointCounter = 0;
enum RobotState {
  IDLE,
  CALIBRATING,
  READY,
  LINE_FOLLOWING,
  CHECKPOINT_ACTION
};

RobotState currentState = IDLE;
bool lastButtonState = LOW;
bool buttonPressed = false;

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(buttonPin, INPUT);
  
  Serial.begin(9600);
  
  for (int i = 0; i < numSensors; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
  
  stopMotors();
  allLedsOff();
}

void loop() {
  bool currentButtonState = digitalRead(buttonPin);
  if (currentButtonState == HIGH && lastButtonState == LOW) {
    buttonPressed = true;
    delay(50);
  }
  lastButtonState = currentButtonState;
  
  switch (currentState) {
    case IDLE:
      if (buttonPressed) {
        buttonPressed = false;
        blinkLed(RED_LED, 2);
        allLedsOff();
        currentState = CALIBRATING;
      }
      break;
      
    case CALIBRATING:
      calibrateSensors();
      blinkLed(GREEN_LED, 2);
      allLedsOff();
      currentState = READY;
      break;
      
    case READY:
      if (buttonPressed) {
        buttonPressed = false;
        currentState = LINE_FOLLOWING;
        checkpointCount = 0;
        lastCheckpointTime = 0;
        allBlackDetected = false;
        ignoreCheckpointCounter = 0;
      }
      break;
      
    case LINE_FOLLOWING:
      followLine();
      break;
      
    case CHECKPOINT_ACTION:
      handleCheckpoint();
      break;
  }
}

void calibrateSensors() {
  unsigned long startTime = millis();
  
  while (millis() - startTime < CALIBRATION_TIME) {
    readSensors();
    updateCalibration();
    delay(10);
  }
}

void readSensors() {
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

void updateCalibration() {
  for (int i = 0; i < numSensors; i++) {
    if (sensorValues[i] < sensorMin[i]) {
      sensorMin[i] = sensorValues[i];
    }
    if (sensorValues[i] > sensorMax[i]) {
      sensorMax[i] = sensorValues[i];
    }
  }
}

void runMotorsStraight(int speed) {
  int leftSpd = speed + leftMotorTrim;
  int rightSpd = speed + rightMotorTrim;
  runMotors(leftSpd, rightSpd);
}

void moveThroughCheckpoint() {
  unsigned long moveStart = millis();
  
  while (millis() - moveStart < 800) {
    readSensors();
    int blackCount = 0;
    
    for (int i = 0; i < numSensors; i++) {
      int normVal = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
      normVal = constrain(normVal, 0, 1000);
      if (normVal > CHECKPOINT_THRESHOLD) {
        blackCount++;
      }
    }
    
    if (blackCount < 6 && millis() - moveStart > 300) {
      break;
    }
    
    runMotorsStraight(baseSpeed);
    delay(10);
  }
  
  stopMotors();
  delay(200);
}

void handleCheckpoint() {
  checkpointCount++;
  Serial.print("Checkpoint: ");
  Serial.println(checkpointCount);
  
  switch (checkpointCount) {
    case 1:
      Serial.println("CP1: Start");
      moveThroughCheckpoint();
      break;
      
    case 2:
      Serial.println("CP2: Stop 5s");
      stopMotors();
      delay(5000);
      moveThroughCheckpoint();
      break;
      
    case 3:
      Serial.println("CP3: Green ON");
      digitalWrite(GREEN_LED, HIGH);
      moveThroughCheckpoint();
      break;
      
    case 4:
      Serial.println("CP4: Green OFF");
      digitalWrite(GREEN_LED, LOW);
      moveThroughCheckpoint();
      break;
      
    case 5:
      Serial.println("CP5: Junction - Left Turn");
      moveThroughCheckpoint();
      executeLeftTurn();
      break;
      
    case 6:
      Serial.println("CP6: 360 Rotate");
      moveThroughCheckpoint();
      delay(200);
      execute360Rotation();
      break;
      
    case 7:
      Serial.println("CP7: Reverse");
      executeReverseTraverse();
      break;
      
    case 8:
      Serial.println("CP8: Reverse + Ignore Next");
      executeReverseTraverse();
      ignoreCheckpointCounter = 1;
      break;
      
    case 9:
      Serial.println("CP9: STOP - Blink RED");
      stopMotors();
      blinkLed(RED_LED, 5);
      currentState = IDLE;
      return;
      
    default:
      Serial.println("Extra checkpoint detected");
      break;
  }
  
  lastCheckpointTime = millis();
  
  integral = 0;
  lastError = 0;
  error = 0;
  allBlackDetected = false;
  lastValidPosition = 3500;
  
  delay(100);
  
  currentState = LINE_FOLLOWING;
}

void executeLeftTurn() {
  unsigned long turnStart = millis();
  
  while (millis() - turnStart < 1000) {
    runMotors(-100, 100);
    delay(5);
    
    if ((millis() - turnStart) > 300) {
      readSensors();
      int activeCount = 0;
      for (int i = 0; i < numSensors; i++) {
        int normVal = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
        normVal = constrain(normVal, 0, 1000);
        if (normVal > LINE_DETECTION_THRESHOLD) {
          activeCount++;
        }
      }
      
      if (activeCount >= 2 && activeCount <= 5 && (millis() - turnStart) > 350) {
        break;
      }
    }
  }
  
  stopMotors();
  delay(100);
}

void execute360Rotation() {
  unsigned long rotationTime = 1600;
  unsigned long startTime = millis();
  
  while (millis() - startTime < rotationTime) {
    runMotors(120, -120);
    delay(5);
  }
  
  stopMotors();
  delay(100);
}

void executeReverseTraverse() {
  unsigned long turnStart = millis();
  
  while (millis() - turnStart < 2000) {
    runMotors(-150, 150);
    delay(5);
    
    if ((millis() - turnStart) > 500) {
      readSensors();
      int activeCount = 0;
      for (int i = 0; i < numSensors; i++) {
        int normVal = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
        normVal = constrain(normVal, 0, 1000);
        if (normVal > LINE_DETECTION_THRESHOLD) {
          activeCount++;
        }
      }
      
      if (activeCount >= 2 && activeCount <= 5) {
        break;
      }
    }
  }
  
  stopMotors();
  delay(100);
}

void followLine() {
  readSensors();
  
  int normalizedValues[numSensors];
  for (int i = 0; i < numSensors; i++) {
    normalizedValues[i] = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
    normalizedValues[i] = constrain(normalizedValues[i], 0, 1000);
  }
  
  int blackSensors = 0;
  for (int i = 0; i < numSensors; i++) {
    if (normalizedValues[i] > CHECKPOINT_THRESHOLD) {
      blackSensors++;
    }
  }
  
  bool allSensorsBlack = (blackSensors == 8);
  
  if (allSensorsBlack) {
    if (!allBlackDetected) {
      allBlackStartTime = millis();
      allBlackDetected = true;
    } else {
      unsigned long allBlackDuration = millis() - allBlackStartTime;
      unsigned long timeSinceLastCheckpoint = millis() - lastCheckpointTime;
      
      if (allBlackDuration >= CHECKPOINT_DURATION && 
          timeSinceLastCheckpoint > CHECKPOINT_COOLDOWN) {
        
        if (ignoreCheckpointCounter > 0) {
          ignoreCheckpointCounter--;
          Serial.println("Ignoring checkpoint - continuing");
          runMotorsStraight(baseSpeed);
          delay(500);
          allBlackDetected = false;
          lastCheckpointTime = millis();
          return;
        }
        
        currentState = CHECKPOINT_ACTION;
        return;
      }
    }
    runMotorsStraight(baseSpeed);
    return;
  } else {
    allBlackDetected = false;
  }
  
  int activeSensorCount = 0;
  float sensorSum = 0;
  bool onLine = false;
  
  for (int i = 0; i < numSensors; i++) {
    if (normalizedValues[i] > LINE_DETECTION_THRESHOLD) {
      activeSensorCount++;
      sensorSum += i;
      onLine = true;
    }
  }
  
  if (onLine) {
    lastLineTime = millis();
    
    long weightedSum = 0;
    long sum = 0;
    for (int i = 0; i < numSensors; i++) {
      long value = normalizedValues[i];
      long weight = (numSensors - 1 - i) * 1000;
      weightedSum += value * weight;
      sum += value;
    }
    
    float position = 3500;
    if (sum > 0) {
      position = (float)weightedSum / sum;
      lastValidPosition = position;
    }
    
    bool isTightTurn = (activeSensorCount >= 4) && (position < 1000 || position > 6000);
    
    if (isTightTurn) {
      float avgSensorPos = sensorSum / activeSensorCount;
      
      if (avgSensorPos < 3.5) {
        runMotors(-120, 120);
      } else {
        runMotors(120, -120);
      }
      
      integral = 0;
      lastError = 0;
      
    } else {
      error = position - 3500;
      integral += error;
      derivative = error - lastError;
      
      integral = constrain(integral, -5000, 5000);
      
      int adaptiveBaseSpeed = baseSpeed;
      
      if (position < 1500 || position > 5500) {
        adaptiveBaseSpeed = baseSpeed * 0.8;
      }
      
      correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
      
      leftSpeed = adaptiveBaseSpeed + correction;
      rightSpeed = adaptiveBaseSpeed - correction;
      
      leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
      rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);
      
      runMotors(leftSpeed, rightSpeed);
      lastError = error;
    }
    
  } else {
    unsigned long timeSinceLineLost = millis() - lastLineTime;
    
    if (timeSinceLineLost < LINE_LOST_TIMEOUT) {
      runMotors(leftSpeed, rightSpeed);
    } else {
      if (lastValidPosition < 3500) {
        runMotors(-150, 150);
      } else {
        runMotors(150, -150);
      }
      
      if (timeSinceLineLost > 1000) {
        if (lastValidPosition < 3500) {
          runMotors(-180, 180);
        } else {
          runMotors(180, -180);
        }
      }
    }
  }
  
  delay(5);
}

void runMotors(int speedA, int speedB) {
  setMotor(AIN1, AIN2, PWMA, speedA);
  setMotor(BIN1, BIN2, PWMB, speedB);
}

void stopMotors() {
  runMotors(0, 0);
}

void setMotor(int in1, int in2, int pwm, int spd) {
  spd = constrain(spd, -255, 255);
  
  if (spd > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, spd);
  } else if (spd < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -spd);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
  }
}

void blinkLed(int ledPin, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
}

void allLedsOff() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
}