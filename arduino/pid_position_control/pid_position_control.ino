// PID position control of a DC motor with a quadrature encoder
const int PWM_PIN = 11;   // PWM output to the motor driver
const int DIR1 = 4;       // direction pins
const int DIR2 = 5;
const int ENC_A = 2;      // encoder channels, both on interrupt pins
const int ENC_B = 3;

const double DEG_PER_COUNT = 0.9;     // set from the encoder: 360 / (4 * pulses per revolution)
const double SETPOINT = 100.0;        // target angle in degrees
const double Kp = 0.32, Ki = 0.1, Kd = 0.3;
const unsigned long SAMPLE_MS = 100;  // control period

volatile long count = 0;   // shared with the interrupt routine
volatile byte statep = 0;  // previous encoder state

double totalError = 0;
double lastError = 0;
unsigned long lastSample = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderChange, CHANGE);
}

void loop() {
  unsigned long now = millis();
  if (now - lastSample < SAMPLE_MS) return;   // run once per control period
  lastSample = now;

  noInterrupts();
  long c = count;          // read the shared counter without interruption
  interrupts();
  double angle = DEG_PER_COUNT * c;

  double error = SETPOINT - angle;
  double changeError = error - lastError;     // derivative term
  totalError += error;                        // integral term
  totalError = constrain(totalError, -255.0 / Ki, 255.0 / Ki);   // anti-windup

  double pidTerm = Kp * error + Ki * totalError + Kd * changeError;
  pidTerm = constrain(pidTerm, -255.0, 255.0);
  lastError = error;

  // the direction follows the sign of the PID output, not the sign of the error
  if (pidTerm > 0) {
    digitalWrite(DIR1, LOW);    // forward
    digitalWrite(DIR2, HIGH);
  } else {
    digitalWrite(DIR1, HIGH);   // reverse
    digitalWrite(DIR2, LOW);
  }
  analogWrite(PWM_PIN, (int)fabs(pidTerm));

  Serial.print("Angle: ");
  Serial.println(angle);
}

// one routine for both encoder channels, same decoding as the original
void encoderChange() {
  bool a = digitalRead(ENC_A);
  bool b = digitalRead(ENC_B);
  byte state;
  if (a && b) state = 1;
  else if (a && !b) state = 2;
  else if (!a && !b) state = 3;
  else state = 4;

  switch (state) {
    case 1:
      if (statep == 2) count++;
      if (statep == 4) count--;
      break;
    case 2:
      if (statep == 1) count--;
      if (statep == 3) count++;
      break;
    case 3:
      if (statep == 2) count--;
      if (statep == 4) count++;
      break;
    default:
      if (statep == 1) count++;
      if (statep == 3) count--;
  }
  statep = state;
}
