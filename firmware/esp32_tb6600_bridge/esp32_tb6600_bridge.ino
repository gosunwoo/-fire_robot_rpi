#include <AccelStepper.h>
#include <SPI.h>

/*
  ESP32 -> dual TB6600 bridge (1/8 microstep)

  The ESP32 is deliberately only a motor actuator. Map localization, path
  following, and waypoint completion belong to ROS where map->base_link can be
  observed. This prevents an ESP32 open-loop waypoint routine fighting ROS.

  Pi -> ESP32: M,<seq>,<left_sps>,<right_sps> | STOP,<seq> | PING,<seq> | ZERO,<seq>
  ESP32 -> Pi: ACK,<seq> | STAT,<ms>,<state>,<left_sps>,<right_sps>
                  | ENC,<ms>,<generated_left_steps>,<generated_right_steps>
                  | ENC_ABS,<ms>,<angle_deg>,<turns>,<distance_m>
                  | US,<ms>,<distance_m>,<valid>
*/

#define L_STEP 25
#define L_DIR 26
#define L_EN 27
#define R_STEP 14
#define R_DIR 12
#define R_EN 13

// AS5048A SPI bus (single encoder)
#define ENC_SCK 18
#define ENC_MOSI 23
#define ENC_MISO 19
#define ENC_LEFT_CS 17

// Front HC-SR04. ECHO is a 5 V signal: use a 1 kOhm series resistor and a
// 2 kOhm resistor from GPIO33 to GND. Never wire ECHO directly to the ESP32.
#define ULTRASONIC_TRIG 32
#define ULTRASONIC_ECHO 33

const bool ENABLE_ACTIVE_LOW = true;
const bool INVERT_LEFT_DIR = false;
const bool INVERT_RIGHT_DIR = false;
const float MAX_STEP_SPEED = 1600.0F;
// Limit command edges that made skid steering harsh. This ramps STEP frequency,
// while runSpeed() remains responsible for accurately timed pulses.
const float MAX_STEP_ACCEL = 900.0F;  // steps/s^2; tune after wheels-off-ground test
const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long TELEMETRY_PERIOD_MS = 200;
const unsigned long BAUDRATE = 115200;
const size_t MAX_INPUT_LINE = 96;

// AS5048A: 14-bit absolute angle, 16384 counts/revolution.
const uint16_t AS5048A_ANGLE_REGISTER = 0x3FFF;
const uint16_t AS5048A_CLEAR_ERROR_REGISTER = 0x0001;
const uint16_t AS5048A_READ_FLAG = 0x4000;
const int32_t AS5048A_COUNTS_PER_REV = 16384;
const int32_t AS5048A_HALF_COUNTS = AS5048A_COUNTS_PER_REV / 2;
const unsigned long ENCODER_SAMPLE_PERIOD_US = 10000;  // 100 Hz
const uint32_t ENCODER_SPI_HZ = 1000000;               // reliable starting speed
const unsigned long ULTRASONIC_TRIGGER_PERIOD_US = 60000;
const unsigned long ULTRASONIC_ECHO_TIMEOUT_US = 30000;
const float ULTRASONIC_MIN_RANGE_M = 0.02F;
const float ULTRASONIC_MAX_RANGE_M = 4.00F;

// User-requested wheel diameter: 10 mm.
// If the actual wheel is 10 cm, change this value to 100.0F.
const float WHEEL_DIAMETER_MM = 10.0F;
const float WHEEL_CIRCUMFERENCE_M = PI * (WHEEL_DIAMETER_MM / 1000.0F);

// Set to -1 only if the remaining encoder decreases while the robot moves forward.
const int ENCODER_SIGN = 1;

AccelStepper leftMotor(AccelStepper::DRIVER, L_STEP, L_DIR);
AccelStepper rightMotor(AccelStepper::DRIVER, R_STEP, R_DIR);
String inputLine;
float targetLeftSps = 0.0F;
float targetRightSps = 0.0F;
float currentLeftSps = 0.0F;
float currentRightSps = 0.0F;
unsigned long lastCommandMs = 0;
unsigned long lastRampUs = 0;
unsigned long lastTelemetryMs = 0;
String driveState = "BOOT";

SPISettings encoderSpiSettings(ENCODER_SPI_HZ, MSBFIRST, SPI_MODE1);
unsigned long lastEncoderSampleUs = 0;
bool encoderReady = false;
uint16_t rawAngle = 0;
uint16_t previousRaw = 0;
int64_t cumulativeCounts = 0;
uint32_t encoderErrors = 0;

portMUX_TYPE ultrasonicMux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long ultrasonicEchoRiseUs = 0;
volatile unsigned long ultrasonicEchoDurationUs = 0;
volatile bool ultrasonicEchoReady = false;
unsigned long lastUltrasonicTriggerUs = 0;
bool ultrasonicWaitingForEcho = false;
float ultrasonicDistanceM = NAN;
bool ultrasonicValid = false;

void IRAM_ATTR onUltrasonicEchoChange() {
  const unsigned long nowUs = micros();
  if (digitalRead(ULTRASONIC_ECHO) == HIGH) {
    ultrasonicEchoRiseUs = nowUs;
    return;
  }
  if (ultrasonicEchoRiseUs == 0) return;
  portENTER_CRITICAL_ISR(&ultrasonicMux);
  ultrasonicEchoDurationUs = nowUs - ultrasonicEchoRiseUs;
  ultrasonicEchoReady = true;
  portEXIT_CRITICAL_ISR(&ultrasonicMux);
  ultrasonicEchoRiseUs = 0;
}

void setEnable(bool enabled) {
  const int active = ENABLE_ACTIVE_LOW ? LOW : HIGH;
  const int inactive = ENABLE_ACTIVE_LOW ? HIGH : LOW;
  digitalWrite(L_EN, enabled ? active : inactive);
  digitalWrite(R_EN, enabled ? active : inactive);
}

float clampSps(float value) {
  return constrain(value, -MAX_STEP_SPEED, MAX_STEP_SPEED);
}

void setTargets(float left, float right) {
  targetLeftSps = clampSps(left);
  targetRightSps = clampSps(right);
  driveState = (targetLeftSps == 0.0F && targetRightSps == 0.0F) ? "STOP" : "RUN";
}

void emergencyStop() {
  targetLeftSps = targetRightSps = 0.0F;
  currentLeftSps = currentRightSps = 0.0F;
  leftMotor.setSpeed(0.0F);
  rightMotor.setSpeed(0.0F);
  driveState = "STOP";
}

float approach(float current, float target, float maximumDelta) {
  if (current < target) return min(current + maximumDelta, target);
  if (current > target) return max(current - maximumDelta, target);
  return current;
}

void updateSpeedRamp() {
  const unsigned long nowUs = micros();
  const unsigned long elapsedUs = nowUs - lastRampUs;
  if (elapsedUs < 1000) return;
  lastRampUs = nowUs;
  const float maxDelta = MAX_STEP_ACCEL * elapsedUs * 1.0e-6F;
  currentLeftSps = approach(currentLeftSps, targetLeftSps, maxDelta);
  currentRightSps = approach(currentRightSps, targetRightSps, maxDelta);
  leftMotor.setSpeed(INVERT_LEFT_DIR ? -currentLeftSps : currentLeftSps);
  rightMotor.setSpeed(INVERT_RIGHT_DIR ? -currentRightSps : currentRightSps);
}

int splitCsv(String line, String fields[], int capacity) {
  int count = 0;
  int start = 0;
  line.trim();
  while (count < capacity) {
    const int comma = line.indexOf(',', start);
    if (comma < 0) { fields[count++] = line.substring(start); break; }
    fields[count++] = line.substring(start, comma);
    start = comma + 1;
  }
  for (int i = 0; i < count; ++i) fields[i].trim();
  return count;
}

void ack(const String &seq) { Serial.println("ACK," + seq); }

bool hasEvenParity(uint16_t value) {
  bool parity = false;
  while (value != 0) {
    parity = !parity;
    value &= (value - 1);
  }
  return !parity;
}

uint16_t addEvenParity(uint16_t value) {
  value &= 0x7FFF;
  if (!hasEvenParity(value)) value |= 0x8000;
  return value;
}

uint16_t transferAs5048aFrame(uint8_t chipSelectPin, uint16_t transmitData) {
  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(1);

  const uint8_t highByte =
      SPI.transfer(static_cast<uint8_t>(transmitData >> 8));
  const uint8_t lowByte =
      SPI.transfer(static_cast<uint8_t>(transmitData & 0xFF));

  delayMicroseconds(1);
  digitalWrite(chipSelectPin, HIGH);
  delayMicroseconds(1);

  return (static_cast<uint16_t>(highByte) << 8) | lowByte;
}

void clearAs5048aError(uint8_t chipSelectPin) {
  const uint16_t clearCommand = addEvenParity(
      AS5048A_READ_FLAG | AS5048A_CLEAR_ERROR_REGISTER
  );

  SPI.beginTransaction(encoderSpiSettings);
  transferAs5048aFrame(chipSelectPin, clearCommand);
  transferAs5048aFrame(chipSelectPin, 0x0000U);
  SPI.endTransaction();
}

bool readAs5048aAngle(uint8_t chipSelectPin, uint16_t &rawAngle) {
  const uint16_t readCommand = addEvenParity(
      AS5048A_READ_FLAG | AS5048A_ANGLE_REGISTER
  );

  SPI.beginTransaction(encoderSpiSettings);
  transferAs5048aFrame(chipSelectPin, readCommand);
  const uint16_t response =
      transferAs5048aFrame(chipSelectPin, 0x0000U);
  SPI.endTransaction();

  const bool parityOk = hasEvenParity(response);
  const bool errorFlag = (response & 0x4000U) != 0U;

  if (!parityOk || errorFlag) {
    clearAs5048aError(chipSelectPin);
    return false;
  }

  rawAngle = response & 0x3FFFU;
  return true;
}

int32_t unwrapDelta(uint16_t currentRaw, uint16_t previousRaw) {
  int32_t delta = static_cast<int32_t>(currentRaw) - static_cast<int32_t>(previousRaw);
  if (delta > AS5048A_HALF_COUNTS) delta -= AS5048A_COUNTS_PER_REV;
  else if (delta < -AS5048A_HALF_COUNTS) delta += AS5048A_COUNTS_PER_REV;
  return delta;
}

void updateEncoders() {
  const unsigned long nowUs = micros();
  if (nowUs - lastEncoderSampleUs < ENCODER_SAMPLE_PERIOD_US) return;
  lastEncoderSampleUs = nowUs;

  uint16_t currentRaw = 0;
  if (readAs5048aAngle(ENC_LEFT_CS, currentRaw)) {
    rawAngle = currentRaw;

    if (!encoderReady) {
      previousRaw = currentRaw;
      encoderReady = true;
    } else {
      const int32_t delta = unwrapDelta(currentRaw, previousRaw);
      cumulativeCounts += static_cast<int64_t>(delta) * ENCODER_SIGN;
      previousRaw = currentRaw;
    }
  } else {
    ++encoderErrors;
  }
}

void updateUltrasonic() {
  unsigned long durationUs = 0;
  bool echoReady = false;
  portENTER_CRITICAL(&ultrasonicMux);
  if (ultrasonicEchoReady) {
    durationUs = ultrasonicEchoDurationUs;
    ultrasonicEchoReady = false;
    echoReady = true;
  }
  portEXIT_CRITICAL(&ultrasonicMux);

  if (echoReady) {
    // Round-trip flight time: distance = duration * speed_of_sound / 2.
    const float measuredM = durationUs * 0.0001715F;
    ultrasonicValid =
        measuredM >= ULTRASONIC_MIN_RANGE_M &&
        measuredM <= ULTRASONIC_MAX_RANGE_M;
    ultrasonicDistanceM = ultrasonicValid ? measuredM : NAN;
    ultrasonicWaitingForEcho = false;
  }

  const unsigned long nowUs = micros();
  if (
      ultrasonicWaitingForEcho &&
      nowUs - lastUltrasonicTriggerUs > ULTRASONIC_ECHO_TIMEOUT_US
  ) {
    ultrasonicWaitingForEcho = false;
    ultrasonicValid = false;
    ultrasonicDistanceM = NAN;
  }
  if (
      !ultrasonicWaitingForEcho &&
      nowUs - lastUltrasonicTriggerUs >= ULTRASONIC_TRIGGER_PERIOD_US
  ) {
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    lastUltrasonicTriggerUs = micros();
    ultrasonicWaitingForEcho = true;
  }
}

void zeroEncoderDistance() {
  // ZERO 명령은 누적 이동거리만 0으로 초기화한다.
  // 절대 각도(rawAngle)는 초기화하지 않는다.
  cumulativeCounts = 0;
  if (encoderReady) previousRaw = rawAngle;
}

void sendTelemetry() {
  Serial.printf(
    "STAT,%lu,%s,%.1f,%.1f\n",
    millis(),
    driveState.c_str(),
    currentLeftSps,
    currentRightSps
  );

  // 기존 모터 스텝 카운트 텔레메트리는 그대로 유지한다.
  const long left =
      INVERT_LEFT_DIR ? -leftMotor.currentPosition() : leftMotor.currentPosition();
  const long right =
      INVERT_RIGHT_DIR ? -rightMotor.currentPosition() : rightMotor.currentPosition();
  Serial.printf("ENC,%lu,%ld,%ld\n", millis(), left, right);

  if (encoderReady) {
    // AS5048A 절대 각도는 launch 시 초기화하지 않고 그대로 출력한다.
    const float angleDeg =
        rawAngle * (360.0F / AS5048A_COUNTS_PER_REV);

    const double turns =
        static_cast<double>(cumulativeCounts) / AS5048A_COUNTS_PER_REV;

    const double distanceM =
        turns * WHEEL_CIRCUMFERENCE_M;

    Serial.printf(
      "ENC_ABS,%lu,%.2f,%.6f,%.6f\n",
      millis(),
      angleDeg,
      turns,
      distanceM
    );
  } else {
    Serial.printf(
      "ERR,ENCODER_NOT_READY,%lu\n",
      static_cast<unsigned long>(encoderErrors)
    );
  }
  Serial.printf(
    "US,%lu,%.4f,%d\n",
    millis(),
    ultrasonicValid ? ultrasonicDistanceM : 0.0F,
    ultrasonicValid ? 1 : 0
  );
}

void handleLine(String line) {
  String fields[4];
  const int count = splitCsv(line, fields, 4);
  fields[0].toUpperCase();
  if (fields[0] == "M" && count == 4) {
    setTargets(fields[2].toFloat(), fields[3].toFloat());
    lastCommandMs = millis();
    ack(fields[1]);
  } else if (fields[0] == "STOP" && count >= 2) {
    emergencyStop();
    lastCommandMs = millis();
    ack(fields[1]);
  } else if (fields[0] == "PING" && count >= 2) {
    ack(fields[1]);
    sendTelemetry();
  } else if (fields[0] == "ZERO" && count >= 2) {
    leftMotor.setCurrentPosition(0);
    rightMotor.setCurrentPosition(0);
    zeroEncoderDistance();
    ack(fields[1]);
  } else {
    Serial.println("ERR,BAD_COMMAND");
  }
}

void readSerial() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (inputLine.length()) { handleLine(inputLine); inputLine = ""; }
    } else if (c >= 32 && c <= 126) {
      if (inputLine.length() < MAX_INPUT_LINE) inputLine += c;
      else { inputLine = ""; Serial.println("ERR,LINE_TOO_LONG"); }
    }
  }
}

void setup() {
  Serial.begin(BAUDRATE);
  inputLine.reserve(MAX_INPUT_LINE);
  pinMode(L_EN, OUTPUT); pinMode(R_EN, OUTPUT); setEnable(true);
  leftMotor.setMaxSpeed(MAX_STEP_SPEED); rightMotor.setMaxSpeed(MAX_STEP_SPEED);
  leftMotor.setMinPulseWidth(5); rightMotor.setMinPulseWidth(5);

  pinMode(ENC_LEFT_CS, OUTPUT);
  digitalWrite(ENC_LEFT_CS, HIGH);
  SPI.begin(ENC_SCK, ENC_MISO, ENC_MOSI, -1);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  pinMode(ULTRASONIC_ECHO, INPUT);
  attachInterrupt(
    digitalPinToInterrupt(ULTRASONIC_ECHO),
    onUltrasonicEchoChange,
    CHANGE
  );
  delay(20);

  emergencyStop();
  lastCommandMs = lastTelemetryMs = millis();
  lastRampUs = micros();
  lastEncoderSampleUs = micros() - ENCODER_SAMPLE_PERIOD_US;
  lastUltrasonicTriggerUs = micros() - ULTRASONIC_TRIGGER_PERIOD_US;

  // Prime the encoder before the first telemetry packet.
  for (int i = 0; i < 3; ++i) {
    updateEncoders();
    delay(10);
  }

  Serial.println("STAT,0,READY,0,0");
}

void loop() {
  readSerial();
  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS &&
      (targetLeftSps != 0.0F || targetRightSps != 0.0F)) {
    emergencyStop();
    driveState = "FAILSAFE";
    Serial.println("ERR,COMMAND_TIMEOUT_STOP");
  }
  updateSpeedRamp();
  leftMotor.runSpeed();
  rightMotor.runSpeed();
  updateEncoders();
  updateUltrasonic();
  if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = millis();
    sendTelemetry();
  }
}
