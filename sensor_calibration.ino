#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define 
  if (digitalRead(CALIBRATION_BUTTON_PIN) == LOW) {
    calibrateThreshold();
  }
debugbegin(x) Serial.begin(x)
#define debugln(x)    Serial.println(x)
#define debug(x)      Serial.print(x)
#else
#define debugbegin(x)
#define debugln(x)
#define debug(x)
#endif

#define EN1 2
#define EN2 3
#define IN3 50
#define IN4 51
#define IN1 52
#define IN2 53

#define MRF(pwm) \
  digitalWrite(IN1, LOW); \
  digitalWrite(IN2, HIGH); \
  analogWrite(EN1, pwm);
#define MRB(pwm) \
  digitalWrite(IN1, HIGH); \
  digitalWrite(IN2, LOW); \
  analogWrite(EN1, pwm);
#define MRS() \
  digitalWrite(IN1, HIGH); \
  digitalWrite(IN2, HIGH); \
  analogWrite(EN1, 255);
#define MLF(pwm) \
  digitalWrite(IN3, HIGH); \
  digitalWrite(IN4, LOW); \
  analogWrite(EN2, pwm);
#define MLB(pwm) \
  digitalWrite(IN3, LOW); \
  digitalWrite(IN4, HIGH); \
  analogWrite(EN2, pwm);
#define MLS() \
  digitalWrite(IN3, HIGH); \
  digitalWrite(IN4, HIGH); \
  analogWrite(EN2, 255);

#define THRESHOLD 600
#define CALIBRATION_BUTTON_PIN 22
bool sensors[15];
int dynamicThreshold = THRESHOLD;

void setup() {
  debugbegin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);

  pinMode(CALIBRATION_BUTTON_PIN, INPUT_PULLUP);  
}


void loop() {
  // Read sensors with dynamic threshold
  sensors[0]  = analogRead(A0 ) > dynamicThreshold;
  sensors[1]  = analogRead(A1 ) > dynamicThreshold;
  sensors[2]  = analogRead(A2 ) > dynamicThreshold;
  sensors[3]  = analogRead(A3 ) > dynamicThreshold;
  sensors[4]  = analogRead(A4 ) > dynamicThreshold;
  sensors[5]  = analogRead(A5 ) > dynamicThreshold;
  sensors[6]  = analogRead(A6 ) > dynamicThreshold;
  sensors[7]  = analogRead(A7 ) > dynamicThreshold;
  sensors[8]  = analogRead(A8 ) > dynamicThreshold;
  sensors[9]  = analogRead(A9 ) > dynamicThreshold;
  sensors[10] = analogRead(A10) > dynamicThreshold;
  sensors[11] = analogRead(A11) > dynamicThreshold;
  sensors[12] = analogRead(A12) > dynamicThreshold;
  sensors[13] = analogRead(A13) > dynamicThreshold;
  sensors[14] = analogRead(A14) > dynamicThreshold;

  // Check calibration button
  if (digitalRead(CALIBRATION_BUTTON_PIN) == LOW) {
    calibrateThreshold();
  }

  // مثال: نمایش threshold روی سریال
  Serial.print("Threshold: ");
  Serial.println(dynamicThreshold);

  delay(100);
}


void calibrateThreshold() {
  debugln("Calibration started...");
  unsigned long startTime = millis();
  unsigned long calibrationDuration = 5000; 
  int maxValue = 0;

  while (millis() - startTime < calibrationDuration) {
    for (int pin = A0; pin <= A14; pin++) {
      int value = analogRead(pin);
      if (value > maxValue) {
        maxValue = value;
      }
    }
    delay(10);
  }

  dynamicThreshold = maxValue - 50; // threshold new
  debug("New threshold: ");
  debugln(dynamicThreshold);
}

