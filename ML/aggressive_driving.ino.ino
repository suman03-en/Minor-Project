#include "features.h"
#include "aggressive_driving_classifier.h"

Eloquent::ML::Port::RandomForest classifier;

#define STEP_SIZE       20
#define SAMPLE_DELAY_MS 10
#define WINDOW_SWITCH   80  // switch driving mode every 80 samples (one full window)

float window_buf[WINDOW_SIZE][NUM_AXES];
float feature_vec[NUM_FEATURES];
int   sample_count  = 0;
int   mode          = 0;   // 0 = Normal, 1 = Aggressive
int   mode_counter  = 0;

void pick_new_mode() {
  mode = random(0, 2);  // 0 or 1 with equal probability
  mode_counter = 0;
  Serial.print("\n--- Switching to: ");
  Serial.print(mode == 1 ? "AGGRESSIVE" : "Normal");
  Serial.println(" mode ---");
}

void read_imu_simulated(float *ax, float *ay, float *az,
                        float *gx, float *gy, float *gz) {
  if (mode == 0) {
    // ── Normal driving ─────────────────────────────────────────────
    // Smooth, low variance, gravity dominant on Z
    *ax = random(-15,  15) / 100.0f;   // ±0.15 m/s²
    *ay = random(-15,  15) / 100.0f;
    *az = 980 + random(-10, 10) / 100.0f;  // ~9.8 ± tiny
    *gx = random(-100, 100) / 1000.0f; // ±0.1 deg/s
    *gy = random(-100, 100) / 1000.0f;
    *gz = random(-100, 100) / 1000.0f;

  } else {
    // ── Aggressive driving ─────────────────────────────────────────
    // High variance, large spikes, rapid direction changes
    *ax = random(-900,  900) / 100.0f;  // ±9 m/s²
    *ay = random(-900,  900) / 100.0f;
    *az = 980 + random(-500, 500) / 100.0f;
    *gx = random(-4000, 4000) / 100.0f; // ±40 deg/s
    *gy = random(-4000, 4000) / 100.0f;
    *gz = random(-4000, 4000) / 100.0f;
  }
}

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));
  delay(2000);
  Serial.println("=== Aggressive Driving Classifier ===");
  Serial.println("Dual-mode simulation | filling buffer...\n");
  pick_new_mode();
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  read_imu_simulated(&ax, &ay, &az, &gx, &gy, &gz);

  // Shift buffer
  memmove(&window_buf[0], &window_buf[1],
          (WINDOW_SIZE - 1) * NUM_AXES * sizeof(float));

  window_buf[WINDOW_SIZE-1][0] = ax;
  window_buf[WINDOW_SIZE-1][1] = ay;
  window_buf[WINDOW_SIZE-1][2] = az;
  window_buf[WINDOW_SIZE-1][3] = gx;
  window_buf[WINDOW_SIZE-1][4] = gy;
  window_buf[WINDOW_SIZE-1][5] = gz;
  sample_count++;
  mode_counter++;

  // Switch mode every WINDOW_SWITCH samples
  if (mode_counter >= WINDOW_SWITCH) {
    pick_new_mode();
  }

  // Run inference every STEP_SIZE samples once buffer is full
  if (sample_count >= WINDOW_SIZE && (sample_count % STEP_SIZE == 0)) {
    extract_features(window_buf, feature_vec);
    int label = classifier.predict(feature_vec);

    // Result matches simulated mode?
    bool correct = (label == mode);

    Serial.print("Sample ");
    Serial.print(sample_count);
    Serial.print("  |  Mode: ");
    Serial.print(mode == 1 ? "AGGRESSIVE" : "Normal    ");
    Serial.print("  |  Predicted: ");
    Serial.print(label == 1 ? "AGGRESSIVE" : "Normal    ");
    Serial.print("  |  ");
    Serial.println(correct ? "✓" : "✗ MISMATCH");
  }

  delay(SAMPLE_DELAY_MS);
}