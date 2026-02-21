/*
* Integral wind-up
* Yön kontrol
*/

const uint8_t C1 = 2, C2 = 3, M1 = 4, M2 = 5;
double gearRatio = 840.0;
volatile uint8_t measurement = 0;
volatile uint8_t measurementOld = 0;

volatile long encoderCount = 0;

double Kp = 0.0;
double Ki = 0.0;
double Kd = 0.0;

double reference = 300.0;   // RPM
double integral = 0;
double lastError = 0;

int8_t encoderArray[16] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

void setup() {
  Serial.begin(9600);

  pinMode(C1, INPUT_PULLUP);
  pinMode(C2, INPUT_PULLUP);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(C1), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), isrEncoder, CHANGE);

  delay(3000);  // Time to let servo start

  measurementOld = 2*digitalRead(C1) + digitalRead(C2);
}

void isrEncoder() {
  measurement = 2*digitalRead(C1) + digitalRead(C2);  // 00 = 0, 01 = 1, 10 = 2, 11 = 3
  encoderCount += encoderArray[4*measurement+measurementOld];
  // encoderCount += 2*encoderArrayBool[4*measurement+measurementOld]-1;
  measurementOld=measurement;
}

void loop() {
  static long lastCount = 0;
  static unsigned long lastTime = 0;
  static unsigned long lastSerialTime = 0;
  unsigned long currentTime = millis();

  //
  long countCopy;
  noInterrupts();
  countCopy = encoderCount;
  interrupts();
  long deltaCounts = countCopy - lastCount;

  //
  unsigned long dt_ms = currentTime - lastTime;
  if (dt_ms == 0) return;   // Prevent divide-by-zero
  double deltaTime = dt_ms / 1000.0;

  double motorSpeedRPM = ((deltaCounts / gearRatio) / deltaTime) * 60;

  //
  double error = reference - motorSpeedRPM;
  integral += error * deltaTime;
  double derivative = (error - lastError) / deltaTime;
  double pidOutput = Kp * error + Ki * integral + Kd * derivative;

  pidOutput = constrain(pidOutput, 0, 255);   // Limit PWM to 0-255
  analogWrite(M2, pidOutput);

  lastError = error;
  lastCount = countCopy;
  lastTime = currentTime;

  //
  if (currentTime - lastSerialTime >= 10) {
    double motorTurns = countCopy / gearRatio;
    Serial.print("Total Turns: ");
    Serial.print(motorTurns);
    Serial.print(" | RPM: ");
    Serial.println(motorSpeedRPM);
    lastSerialTime = currentTime;
  }

  // WIP
  if (Serial.available() > 0) {
    double newReference = Serial.parseFloat();
    if (newReference >= 0) {
        reference = newReference;
    }
}
}
