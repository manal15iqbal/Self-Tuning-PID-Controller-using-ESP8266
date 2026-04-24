/*
 * FIXED COMPILATION ERRORS
 * Continuous disturbance + Lower threshold
 */

#include <ESP8266WiFi.h>

#define MOTOR_PWM D1
#define MOTOR_IN1 D2
#define MOTOR_IN2 D3
#define SENSOR_PIN A0

#define GREEN_LED D5
#define RED1 D6
#define RED2 D7
#define RED3 D8

const float DIVIDER_RATIO = 4.0;
const float RPM_PER_VOLT = 200.0;

// ========== SYSTEM STATE ==========
enum SystemState {
  STATE_IDLE,
  STATE_RUNNING,
  STATE_TUNING
};

SystemState currentState = STATE_IDLE;
bool motorRunning = false;
bool disturbanceActive = false;

// ========== SENSOR DATA ==========
float motorVoltage = 0;
float motorRPM = 0;
float noiseLevel = 0;
float baselineNoise = 512.0;

// ========== PID VALUES ==========
float Kp = 0.5, Ki = 0.1, Kd = 0.05;
float outputPWM = 450;

// ========== TUNING VARIABLES ==========
unsigned long tuningStart = 0;
float Ku = 0, Pu = 0;
float testKp = 0.1;
int tuningStep = 0;
int peakCount = 0;
float lastNoiseForPeak = 0;
bool wasRising = false;

// ========== DISTURBANCE ==========
unsigned long lastDisturbanceChange = 0;
bool disturbanceForward = true;

// ========== PRINT TIMER ==========
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== FIXED TUNING - LOW THRESHOLD ===");
  Serial.println("Oscillation threshold: noise > 20");
  Serial.println("Disturbance runs until tuning complete");
  
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED1, OUTPUT);
  pinMode(RED2, OUTPUT);
  pinMode(RED3, OUTPUT);
  
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, 0);
  
  Serial.println("\nCommands: s=start/stop, d=tune");
}

void loop() {
  // Read sensors
  int raw = analogRead(SENSOR_PIN);
  motorVoltage = (raw * (1.0 / 1023.0)) * DIVIDER_RATIO;
  motorRPM = motorVoltage * RPM_PER_VOLT;
  noiseLevel = abs(raw - baselineNoise);
  
  // State machine
  switch(currentState) {
    case STATE_IDLE:
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED1, LOW);
      digitalWrite(RED2, LOW);
      digitalWrite(RED3, LOW);
      break;
      
    case STATE_RUNNING:
      runNormal();
      break;
      
    case STATE_TUNING:
      runTuning();
      break;
  }
  
  // Handle commands
  if (Serial.available()) {
    handleCommand(Serial.read());
  }
  
  delay(50);
}

void runNormal() {
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED1, LOW);
  digitalWrite(RED2, LOW);
  digitalWrite(RED3, LOW);

  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_PWM, outputPWM);
  
  // Print PID every 10 seconds
  if (millis() - lastPrintTime > 10000) {
    Serial.println("PID STATUS:");
    Serial.print("Kp = ");
    Serial.println(Kp, 3);
    Serial.print("Ki = ");
    Serial.println(Ki, 3);
    Serial.print("Kd = ");
    Serial.println(Kd, 3);
    lastPrintTime = millis();
  }
}

void runTuning() {
  // TURN OFF GREEN during disturbance
  digitalWrite(GREEN_LED, LOW);

  // Blink red LEDs during disturbance
  blinkRedLEDs();

  // Keep disturbance running
  runContinuousDisturbance();
  
  // Run appropriate tuning step
  switch(tuningStep) {
    case 0: // Initial disturbance (3 seconds)
      if (millis() - tuningStart > 3000) {
        Serial.println("\n>>> STARTING TUNING SEQUENCE");
        Serial.println(">>> Step 1: Finding Ku (Ultimate Gain)");
        Serial.println(">>> Increasing Kp until oscillation detected...");
        tuningStep = 1;
        testKp = 0.1;
      }
      break;
      
    case 1: // Find Ku (oscillation point)
      findKu();
      break;
      
    case 2: // Measure Pu (oscillation period)
      measurePu();
      break;
      
    case 3: // Calculate and apply
      calculatePID();
      break;
  }
}

void blinkRedLEDs() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  if (millis() - lastBlink > 250) {
    lastBlink = millis();
    ledState = !ledState;
    digitalWrite(RED1, ledState);
    digitalWrite(RED2, ledState);
    digitalWrite(RED3, ledState);
  }
}

void runContinuousDisturbance() {
  if (millis() - lastDisturbanceChange > 150) {
    lastDisturbanceChange = millis();
    
    if (disturbanceForward) {
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
      analogWrite(MOTOR_PWM, 900);
      disturbanceForward = false;
    } else {
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, HIGH); // Brake
      analogWrite(MOTOR_PWM, 900);
      disturbanceForward = true;
    }
  }
}

void findKu() {
  static unsigned long lastIncrease = 0;
  
  if (millis() - lastIncrease > 500) {
    lastIncrease = millis();
    
    testKp += 0.1;
    
    Serial.print("Step 1 - Testing Kp=");
    Serial.print(testKp, 2);
    Serial.print(", Noise=");
    Serial.println(noiseLevel, 1);
    
    if (noiseLevel > 20.0) {
      Ku = testKp;
      
      Serial.println("\nWRONG PID VALUES:");
      Serial.print("Kp = ");
      Serial.println(Kp, 3);
      Serial.print("Ki = ");
      Serial.println(Ki, 3);
      Serial.print("Kd = ");
      Serial.println(Kd, 3);
      
      Serial.print("\n>>> OSCILLATION DETECTED! Ku = ");
      Serial.println(Ku, 3);
      Serial.println(">>> Step 2: Measuring oscillation period...");
      
      tuningStep = 2;
      peakCount = 0;
      lastNoiseForPeak = noiseLevel;
      wasRising = false;
    }
    
    if (millis() - tuningStart > 30000) {
      Serial.println(">>> WARNING: Weak oscillation, using estimated Ku=0.8");
      Ku = 0.8;
      tuningStep = 2;
    }
  }
}

void measurePu() {
  static unsigned long firstPeakTime = 0;
  
  if (noiseLevel > lastNoiseForPeak) {
    wasRising = true;
  } else if (noiseLevel < lastNoiseForPeak && wasRising) {
    wasRising = false;
    peakCount++;
    
    Serial.print("Peak ");
    Serial.print(peakCount);
    Serial.print(" at noise=");
    Serial.println(noiseLevel, 1);
    
    if (peakCount == 1) {
      firstPeakTime = millis();
    } else if (peakCount >= 3) {
      Pu = (millis() - firstPeakTime) / 2000.0;
      Serial.print(">>> Period Pu = ");
      Serial.print(Pu, 3);
      Serial.println(" seconds");
      
      tuningStep = 3;
    }
  }
  
  lastNoiseForPeak = noiseLevel;
  
  if (millis() - tuningStart > 40000) {
    Serial.println(">>> Using default Pu=0.5 seconds");
    Pu = 0.5;
    tuningStep = 3;
  }
}

void calculatePID() {
  disturbanceActive = false;
  
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(RED1, LOW);
  digitalWrite(RED2, LOW);
  digitalWrite(RED3, LOW);

  Kp = 0.6 * Ku;
  Ki = (2.0 * Kp) / Pu;
  Kd = (Kp * Pu) / 8.0;
  
  Kp = constrain(Kp, 0.1, 5.0);
  Ki = constrain(Ki, 0.0, 2.0);
  Kd = constrain(Kd, 0.0, 1.0);
  
  Serial.println("\n========================================");
  Serial.println("           TUNING COMPLETE!");
  Serial.println("========================================");
  Serial.print("Ku: ");
  Serial.println(Ku, 3);
  Serial.print("Pu: ");
  Serial.println(Pu, 3);
  Serial.print("New Kp: ");
  Serial.println(Kp, 3);
  Serial.print("New Ki: ");
  Serial.println(Ki, 3);
  Serial.print("New Kd: ");
  Serial.println(Kd, 3);
  Serial.println("========================================");
  
  outputPWM = 450;
  currentState = STATE_RUNNING;
  tuningStep = 0;
  Serial.println(">>> Disturbance stopped. Running with new PID.");
}

void handleCommand(char cmd) {
  switch(cmd) {
    case 's':
      if (!motorRunning) {
        motorRunning = true;
        currentState = STATE_RUNNING;
        Serial.println(">>> Motor STARTED");
      } else {
        motorRunning = false;
        currentState = STATE_IDLE;
        analogWrite(MOTOR_PWM, 0);
        Serial.println(">>> Motor STOPPED");
      }
      break;
      
    case 'd':
      if (motorRunning && currentState == STATE_RUNNING) {
        currentState = STATE_TUNING;
        tuningStart = millis();
        disturbanceActive = true;
        tuningStep = 0;
        
        Serial.println("\n>>> STARTING TUNING SEQUENCE");
        Serial.println(">>> Disturbance will run until tuning complete");
        Serial.println(">>> This may take 30-40 seconds...");
      } else {
        Serial.println(">>> Start motor first (s)");
      }
      break;
      
    case '?':
      Serial.println("\n=== COMMANDS ===");
      Serial.println("s - Start/Stop motor");
      Serial.println("d - Start tuning (disturbance continues until done)");
      Serial.println("? - Help");
      break;
      
    default:
      break;
  }
}