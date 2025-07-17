#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MS5611.h>
#include <Adafruit_BNO055.h>
#include <arm_math.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Sensor objects
TinyGPSPlus gps;
Adafruit_BME280 bme;
MS5611 ms5611;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
#define SEALEVELPRESSSURE_HPA (1013.25)

// Teensy 4.0: Use Serial1 for GPS (pins 0-RX, 1-TX)
#define gpsSerial Serial1
static const uint32_t GPSBaud = 9600;

// RFD900x Configuration
#define RFD_RX_PIN 24    // Teensy 4.0 RX -> RFD900x TX
#define RFD_TX_PIN 25    // Teensy 4.0 TX -> RFD900x RX
#define RFD_BAUD 57600   // Standard baud rate

SoftwareSerial rfdSerial(RFD_RX_PIN, RFD_TX_PIN);

// Flight parameters
#define FIRST_STAGE_ALTITUDE 4000    // Altitude threshold for first stage separation (m)
#define SECOND_STAGE_ALTITUDE 8000   // Altitude threshold for second stage apogee (m)
#define ACCELERATION_THRESHOLD 10    // m/s² for stage separation
#define STAGE_SEPARATION_DELAY 500   // ms delay between separation and ignition
#define MAIN_CHUTE_ALTITUDE 1000     // Main parachute deployment altitude (m)
#define DROGUE_CHUTE_ALTITUDE 3000   // Drogue chute deployment altitude (m)

// Pin definitions
#define SYSTEM_READY_PIN 8
#define LAUNCH_DETECT_PIN 7
#define FIRST_STAGE_SEP_PIN 5
#define SECOND_STAGE_IGNITE_PIN 6
#define MAIN_CHUTE_PIN 4
#define DROGUE_CHUTE_PIN 3

// UKF Configuration
const int n = 2;                    // State dimension (altitude, velocity)
const int sigmaCount = 2 * n + 1;   // Number of sigma points
const float dt = 0.035;             // Time step
float lambda = 1;                   // Scaling parameter
float alpha = 1e-3, beta = 2, kappa = 0; // UKF tuning parameters

// State variables
arm_matrix_instance_f32 x;          // State vector [n x 1]
arm_matrix_instance_f32 P;          // Covariance [n x n]
arm_matrix_instance_f32 Q;          // Process noise [n x n]
float R_bme = 1.5;                 // BME measurement noise
float R_ms = 1.5;                  // MS measurement noise

// Flight data
float usedAlt;
bool peakDetected = false;
unsigned long peakDetectedTime = 0;
float peakAlt = 0;
bool firstStageSeparated = false;
bool secondStageIgnited = false;
unsigned long separationTime = 0;

// Sensor fusion
#define VOTING_WINDOW 10
float h_history[VOTING_WINDOW] = {0};
int history_index = 0;

// System state
unsigned long lastUpdate = 0;
float df = 0;                       // Time difference
float acx = 0, acy = 0, acz = 0;    // Acceleration
float orx = 0, ory = 0, orz = 0;    // Orientation
int state = 1;                      // State machine state
bool parachute1 = false, parachute2 = false;

// ARM Matrix storage
float x_data[2] = {0, 0};
float P_data[4] = {1, 0, 0, 1};
float Q_data[4] = {0.2, 0, 0, 0.2};
float wm[sigmaCount], wc[sigmaCount]; // Weights

// Telemetry Packet Structure (packed binary)
#pragma pack(push, 1)
struct TelemetryPacket {
  uint32_t timestamp;     // 4 bytes
  float altitude;         // 4
  float velocity;         // 4  
  float acceleration_z;   // 4
  float latitude;         // 4
  float longitude;        // 4
  uint16_t state : 4;     // 4 bits
  uint16_t chutes : 2;    // 2 bits (bit0: drogue, bit1: main)
  uint16_t stages : 2;    // 2 bits (bit0: stage1 sep, bit1: stage2 ign)
  uint16_t checksum;      // 2 bytes
};
#pragma pack(pop)

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  Wire.setClock(100000);  // 100 kHz I2C

  gpsSerial.begin(GPSBaud);

  // Initialize sensors
  if (!bno.begin()) {
    Serial.println("Could not find BNO055!");
    while (1);
  }
  
  if (!ms5611.begin()) {
    Serial.println("Could not find MS5611!");
    while (1);
  }
  
  if (!bme.begin(0x76)) {
    Serial.println("Could not find BME280!");
    while (1);
  }

  // Initialize ARM matrices
  arm_mat_init_f32(&x, n, 1, x_data);
  arm_mat_init_f32(&P, n, n, P_data);
  arm_mat_init_f32(&Q, n, n, Q_data);

  // Initialize weights
  lambda = alpha * alpha * (n + kappa) - n;
  wm[0] = lambda / (n + lambda);
  wc[0] = wm[0] + (1 - alpha * alpha + beta);
  for (int i = 1; i < sigmaCount; i++) {
    wm[i] = 1 / (2.0 * (n + lambda));
    wc[i] = wm[i];
  }

  // Setup pins
  pinMode(SYSTEM_READY_PIN, OUTPUT);
  pinMode(LAUNCH_DETECT_PIN, OUTPUT);
  pinMode(FIRST_STAGE_SEP_PIN, OUTPUT);
  pinMode(SECOND_STAGE_IGNITE_PIN, OUTPUT);
  pinMode(MAIN_CHUTE_PIN, OUTPUT);
  pinMode(DROGUE_CHUTE_PIN, OUTPUT);

  // Calibration delay
  bno.setExtCrystalUse(true);
  delay(1000);
  Serial.println("System initialized");
  digitalWrite(SYSTEM_READY_PIN, HIGH);

  rfdSerial.begin(RFD_BAUD);
  delay(1000);
  
  // Configure radio (optional)
  rfdSerial.println("ATS3=5");   // Set network ID
  delay(100);
  rfdSerial.println("ATS6=20");  // 20dBm transmit power (100mW)
  delay(100);
  rfdSerial.println("AT&W");     // Save settings
  delay(100);
  
  Serial.println("RFD900x Initialized");
}

void loop() {
  unsigned long now = millis();
  float elapsed = (now - lastUpdate) / 1000.0;
  if (elapsed < 0.01) return;
  df = elapsed;
  lastUpdate = now;

  // Update GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  // Read sensor data
  readIMUData();
  float alt_bme = getAltBME();
  float alt_ms = getAltMS();
  usedAlt = fuseAltitudes(alt_bme, alt_ms);

  // UKF Prediction
  float Xsig[n][sigmaCount];        // Sigma points
  float Xsig_pred[n][sigmaCount];   // Predicted sigma points
  
  generate_sigma_points(Xsig);
  predict_sigma_points(Xsig, Xsig_pred);
  
  // Convert to ARM matrix format
  arm_matrix_instance_f32 Xsig_mat, Xsig_pred_mat;
  arm_mat_init_f32(&Xsig_mat, n, sigmaCount, (float*)Xsig);
  arm_mat_init_f32(&Xsig_pred_mat, n, sigmaCount, (float*)Xsig_pred);
  
  // Predict mean and covariance
  predict_mean_cov(&Xsig_pred_mat, wm, wc, &Q, &x, &P);
  
  // Update with measurement
  update_with_measurement(usedAlt);

  // State machine
  runStateMachine(x_data[0], alt_bme, alt_ms, x_data[1], acz, orx);

  // Log data
  logSensorData(usedAlt, alt_bme, alt_ms);

  // Emergency checks
  emergencyProcedures(x_data[0], x_data[1]);

  sendBinaryTelemetry();
  handleRadioCommands();
}

// Sensor functions
void readIMUData() {
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> orient = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  acx = accel.x();
  acy = accel.y();
  acz = accel.z();
  
  orx = orient.x();
  ory = orient.y();
  orz = orient.z();
}

float getAltBME() {
  float alt = bme.readAltitude(SEALEVELPRESSSURE_HPA);
  return isnan(alt) ? x_data[0] : alt;
}

float getAltMS() {
  float pressure = ms5611.readPressure();
  if (isnan(pressure)) return x_data[0];
  float pressure_hPa = pressure / 50.0f;
  float alt = 44330.0f * (1.0f - pow(pressure_hPa / SEALEVELPRESSSURE_HPA, 0.1903f));
  return isnan(alt) ? x_data[0] : alt;
}

float fuseAltitudes(float alt1, float alt2) {
  static float weights[2] = {0.5f, 0.5f};
  static float variances[2] = {1.0f, 1.0f};
  
  // Calculate difference from predicted state
  float diff1 = abs(alt1 - x_data[0]);
  float diff2 = abs(alt2 - x_data[0]);
  
  // Update variance estimates
  variances[0] = 0.95f * variances[0] + 0.05f * diff1 * diff1;
  variances[1] = 0.95f * variances[1] + 0.05f * diff2 * diff2;
  
  // Update weights (inverse variance weighting)
  float sumInvVar = 1.0f/variances[0] + 1.0f/variances[1];
  weights[0] = (1.0f/variances[0]) / sumInvVar;
  weights[1] = (1.0f/variances[1]) / sumInvVar;
  
  // Weighted average
  return weights[0] * alt1 + weights[1] * alt2;
}

// UKF functions
void generate_sigma_points(float Xsig[n][sigmaCount]) {
  float sqrtP[n][n];
  arm_matrix_instance_f32 sqrtP_mat;
  arm_mat_init_f32(&sqrtP_mat, n, n, (float*)sqrtP);
  
  // Compute Cholesky decomposition
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      float val = P_data[i*n + j];
      sqrtP[i][j] = (i == j && val > 0) ? sqrt(val) : 0;
    }
  }
  
  float gamma = sqrt(n + lambda);
  arm_mat_scale_f32(&sqrtP_mat, gamma, &sqrtP_mat);
  
  for (int i = 0; i < n; i++) {
    Xsig[i][0] = x_data[i];
  }
  
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      Xsig[j][i+1] = x_data[j] + sqrtP[j][i];
      Xsig[j][i+1+n] = x_data[j] - sqrtP[j][i];
    }
  }
}

void predict_sigma_points(float Xsig[n][sigmaCount], float Xsig_pred[n][sigmaCount]) {
  for (int i = 0; i < sigmaCount; i++) {
    float h = Xsig[0][i];
    float v = Xsig[1][i];
    
    Xsig_pred[0][i] = h + v * dt;
    Xsig_pred[1][i] = v;
  }
}

void predict_mean_cov(arm_matrix_instance_f32* Xsig_pred,
                     const float* Wm,
                     const float* Wc,
                     arm_matrix_instance_f32* Q,
                     arm_matrix_instance_f32* x_pred,
                     arm_matrix_instance_f32* P_pred) {
  // Predict mean
  for (int i = 0; i < n; i++) {
    x_pred->pData[i] = 0;
    for (int j = 0; j < sigmaCount; j++) {
      x_pred->pData[i] += Wm[j] * Xsig_pred->pData[i * sigmaCount + j];
    }
  }

  // Predict covariance
  float diff[n];
  float diff_T[n];
  float temp[n * n];
  
  arm_matrix_instance_f32 diff_mat, diff_T_mat, temp_mat;
  arm_mat_init_f32(&diff_mat, n, 1, diff);
  arm_mat_init_f32(&diff_T_mat, 1, n, diff_T);
  arm_mat_init_f32(&temp_mat, n, n, temp);
  
  memcpy(P_pred->pData, Q->pData, sizeof(float) * Q->numRows * Q->numCols);
  
  for (int j = 0; j < sigmaCount; j++) {
    for (int i = 0; i < n; i++) {
      diff[i] = Xsig_pred->pData[i * sigmaCount + j] - x_pred->pData[i];
      diff_T[i] = diff[i];
    }
    
    arm_mat_mult_f32(&diff_mat, &diff_T_mat, &temp_mat);
    arm_mat_scale_f32(&temp_mat, Wc[j], &temp_mat);
    arm_mat_add_f32(P_pred, &temp_mat, P_pred);
  }
}

void update_with_measurement(float z_meas) {
  float z_pred = x_data[0];
  float innov = z_meas - z_pred;
  float R_combined = (R_bme + R_ms) * 0.5f;

  float relInnov = 0;
  if(!isnan(z_pred) && !isnan(innov)) {
    relInnov = fabs(innov) / max(fabs(z_pred), 1.0f);
  }

  if (relInnov > 0.05f) {
    R_combined *= 10.0f;
  }
  
  float S = P_data[0] + R_combined;
  float K0 = P_data[0] / S;
  float K1 = P_data[2] / S;
  
  float dz = z_meas - z_pred;
  x_data[0] += K0 * dz;
  x_data[1] += K1 * dz;
  
  float IKH[4] = {1 - K0, 0, -K1, 1};
  float P_temp[4];
  
  arm_matrix_instance_f32 IKH_mat, P_mat, P_temp_mat;
  arm_mat_init_f32(&IKH_mat, n, n, IKH);
  arm_mat_init_f32(&P_mat, n, n, P_data);
  arm_mat_init_f32(&P_temp_mat, n, n, P_temp);
  
  arm_mat_mult_f32(&IKH_mat, &P_mat, &P_temp_mat);
  arm_mat_trans_f32(&IKH_mat, &P_mat);
  arm_mat_mult_f32(&P_temp_mat, &P_mat, &P_mat);
  
  P_data[0] += K0 * R_combined * K0;
  P_data[1] += K0 * R_combined * K1;
  P_data[2] += K1 * R_combined * K0;
  P_data[3] += K1 * R_combined * K1;
}

// State machine
void runStateMachine(float h, float alt1, float alt2, float velocity, float accelZ, float orientationX) {
  switch (state) {
    case 1:  // Sensor check
      if (testSensors()) {
        state = 2;
        digitalWrite(SYSTEM_READY_PIN, HIGH);
      }
      break;
      
    case 2:  // Launch detection
      if (isLaunchDetected()) {
        state = 3;
        digitalWrite(LAUNCH_DETECT_PIN, HIGH);
      }
      break;
      
    case 3:  // First stage ascent
      if (checkAltitudeCondition(h, alt1, alt2, FIRST_STAGE_ALTITUDE) && 
          accelZ < ACCELERATION_THRESHOLD && !firstStageSeparated) {
        firstStageSeparated = true;
        separationTime = millis();
        state = 4;
        digitalWrite(FIRST_STAGE_SEP_PIN, HIGH);
      }
      break;
      
    case 4:  // First stage separation and second stage ignition
      if (millis() - separationTime > STAGE_SEPARATION_DELAY && accelZ < 5.0) {
        secondStageIgnited = true;
        state = 5;
        digitalWrite(SECOND_STAGE_IGNITE_PIN, HIGH);
      }
      break;
      
    case 5:  // Second stage ascent
      if (checkAltitudeCondition(h, alt1, alt2, SECOND_STAGE_ALTITUDE)) {
        state = 6;
      }
      break;
      
    case 6:  // Apogee detection
      if (peakDetect(h, velocity, accelZ, orientationX)) {
        parachute1 = true;
        state = 7;
        digitalWrite(DROGUE_CHUTE_PIN, HIGH);
        peakAlt = h;
        peakDetectedTime = millis();
      }
      break;
      
    case 7:  // Descent
      if (h < MAIN_CHUTE_ALTITUDE && parachute1) {
        parachute2 = true;
        state = 8;
        digitalWrite(MAIN_CHUTE_PIN, HIGH);
      }
      break;
      
    case 8:  // Landing
      if (h < 50 && abs(velocity) < 5) {
        Serial.println("Landing complete");
        state = 9;
      }
      break;
  }
}

bool testSensors() {
  float alt_bme = getAltBME();
  float alt_ms = getAltMS();
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  bool bme_ok = !isnan(alt_bme) && alt_bme > -100 && alt_bme < 10000;
  bool ms_ok = !isnan(alt_ms) && alt_ms > -100 && alt_ms < 10000;
  bool bno_ok = !isnan(accel.x()) && !isnan(accel.y()) && !isnan(accel.z());

  return bme_ok && ms_ok && bno_ok;
}

bool isLaunchDetected() {
  static int counter = 0;
  h_history[history_index] = x_data[0];
  history_index = (history_index + 1) % VOTING_WINDOW;
  
  if ((abs(acz) > 15.0 || x_data[0] - h_history[(history_index + 1) % VOTING_WINDOW] > 2.0) && 
      x_data[1] > 5.0) {
    counter++;
  } else {
    counter = max(0, counter-1);
  }
  return counter > 5;
}

bool checkAltitudeCondition(double h, double altitude_bme, double altitude_ms, double threshold) {
  int validCount = 0;
  if (h > threshold) validCount++;
  if (altitude_bme > threshold) validCount++;
  if (altitude_ms > threshold) validCount++;
  return (validCount >= 2);
}

bool peakDetect(float currentAltitude, float currentVelocity, float currentAccelZ, float orientationX) {
    static float maxAltitude = 0;
    const float velocityThreshold = 2.0f;
    const float accelThreshold = -5.0f;
    
    if (currentAltitude > maxAltitude) {
        maxAltitude = currentAltitude;
    }
    
    if (currentAltitude > 0.95f * maxAltitude) {
        bool descending = (currentVelocity < -velocityThreshold);
        bool decelerating = (currentAccelZ < accelThreshold);
        bool orientationChange = (abs(orientationX) > 30.0f);
        
        if ((descending && decelerating) || orientationChange) {
            return true;
        }
    }
    return false;
}

void emergencyProcedures(float altitude, float velocity) {
  // Emergency drogue chute if descending too fast at high altitude
  if (!parachute1 && altitude > DROGUE_CHUTE_ALTITUDE && velocity < -50.0f) {
    digitalWrite(DROGUE_CHUTE_PIN, HIGH);
    parachute1 = true;
    state = 7;
  }
  
  // Emergency main chute if descending too fast at low altitude
  if (!parachute2 && altitude < MAIN_CHUTE_ALTITUDE && velocity < -30.0f) {
    digitalWrite(MAIN_CHUTE_PIN, HIGH);
    parachute2 = true;
    state = 8;
  }
}

void logSensorData(float usedAlt,float alt_bme,float alt_ms){
    Serial.print("State: "); Serial.println(state);
    Serial.print("usedAlt: "); Serial.println(usedAlt);
    Serial.print("BME alt: "); Serial.println(alt_bme);
    Serial.print("MS alt: "); Serial.println(alt_ms);

    Serial.print("Accel X: "); Serial.print(acx);
    Serial.print(" | Y: "); Serial.print(acy);
    Serial.print(" | Z: "); Serial.println(acz);

    Serial.print("Orientation X: "); Serial.print(orx);
    Serial.print(" | Y: "); Serial.print(ory);
    Serial.print(" | Z: "); Serial.println(orz);

    if (gps.location.isValid()) {
      Serial.print("GPS Lat: "); Serial.print(gps.location.lat(), 6);
      Serial.print(" | Lng: "); Serial.print(gps.location.lng(), 6);
    } else {
      Serial.print("GPS konumu geçersiz");
    }
    Serial.println();
}

void sendBinaryTelemetry() {
  static uint32_t lastSend = 0;
  if (millis() - lastSend < 100) return; // 10Hz update rate

  TelemetryPacket packet;
  packet.timestamp = millis();
  packet.altitude = x_data[0];
  packet.velocity = x_data[1];
  packet.acceleration_z = acz;
  
  // GPS data (if available)
  if (gps.location.isValid()) {
    packet.latitude = gps.location.lat();
    packet.longitude = gps.location.lng();
  } else {
    packet.latitude = 0.0f;
    packet.longitude = 0.0f;
  }
  
  // Pack status bits
  packet.state = state & 0x0F;
  packet.chutes = (parachute1 ? 0x01 : 0x00) | (parachute2 ? 0x02 : 0x00);
  packet.stages = (firstStageSeparated ? 0x01 : 0x00) | (secondStageIgnited ? 0x02 : 0x00);

  // Calculate checksum (simple XOR of all bytes)
  uint8_t* bytes = (uint8_t*)&packet;
  packet.checksum = 0;
  for (size_t i = 0; i < sizeof(packet)-2; i++) {
    packet.checksum ^= bytes[i];
  }

  // Send with framing bytes
  rfdSerial.write(0xAA);  // Start byte
  rfdSerial.write(bytes, sizeof(packet));
  rfdSerial.write(0x55);  // End byte
  
  lastSend = millis();
}

void handleRadioCommands() {
  static uint8_t cmdBuffer[32];
  static uint8_t bufPos = 0;
  
  while (rfdSerial.available()) {
    uint8_t c = rfdSerial.read();
    
    // Simple protocol: 0xBB [CMD] [DATA] 0xCC
    if (bufPos == 0 && c != 0xBB) continue;
    
    cmdBuffer[bufPos++] = c;
    
    if (c == 0xCC && bufPos >= 4) { // Complete command
      processCommand(cmdBuffer[1], cmdBuffer[2]);
      bufPos = 0;
    }
    
    if (bufPos >= sizeof(cmdBuffer)) bufPos = 0;
  }
}

void processCommand(uint8_t cmd, uint8_t data) {
  switch(cmd) {
    case 0x01: // Emergency chute deploy
      if (!parachute1) {
        digitalWrite(DROGUE_CHUTE_PIN, HIGH);
        parachute1 = true;
        state = 7;
      }
      break;
      
    case 0x02: // Reset system (for testing)
      if (data == 0x55) {
        resetSystem();
      }
      break;
  }
}

void resetSystem() {
  // Reset all outputs
  digitalWrite(SYSTEM_READY_PIN, LOW);
  digitalWrite(LAUNCH_DETECT_PIN, LOW);
  digitalWrite(FIRST_STAGE_SEP_PIN, LOW);
  digitalWrite(SECOND_STAGE_IGNITE_PIN, LOW);
  digitalWrite(DROGUE_CHUTE_PIN, LOW);
  digitalWrite(MAIN_CHUTE_PIN, LOW);
  
  // Reset state variables
  state = 1;
  parachute1 = parachute2 = false;
  firstStageSeparated = secondStageIgnited = false;
  
  // Reinitialize sensors
  bno.begin();
  ms5611.begin();
  bme.begin(0x76);
}