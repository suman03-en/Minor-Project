// features.h
#pragma once
#include <math.h>
#include <string.h>

#define WINDOW_SIZE  80
#define NUM_AXES      6
#define NUM_FEATURES 123

static inline float _mean(float *d, int n) {
    float s = 0; for (int i = 0; i < n; i++) s += d[i]; return s / n;
}
static inline float _std(float *d, int n, float m) {
    float s = 0; for (int i = 0; i < n; i++) s += (d[i]-m)*(d[i]-m);
    return sqrtf(s / n);
}
static inline float _rms(float *d, int n) {
    float s = 0; for (int i = 0; i < n; i++) s += d[i]*d[i];
    return sqrtf(s / n);
}
static inline float _skew(float *d, int n, float m, float s) {
    if (s < 1e-9f) return 0;
    float v = 0; for (int i = 0; i < n; i++) { float z = (d[i]-m)/s; v += z*z*z; }
    return v / n;
}
static inline float _kurt(float *d, int n, float m, float s) {
    if (s < 1e-9f) return 0;
    float v = 0; for (int i = 0; i < n; i++) { float z = (d[i]-m)/s; v += z*z*z*z; }
    return v / n - 3.0f;  // excess kurtosis — matches scipy default
}
static inline float _pct(float *d, int n, float p) {
    float tmp[WINDOW_SIZE];
    memcpy(tmp, d, n * sizeof(float));
    for (int i = 1; i < n; i++) {           // insertion sort, n=80 is fine
        float k = tmp[i]; int j = i - 1;
        while (j >= 0 && tmp[j] > k) { tmp[j+1] = tmp[j]; j--; }
        tmp[j+1] = k;
    }
    float idx = p * (n - 1) / 100.0f;
    int lo = (int)idx; float fr = idx - lo;
    if (lo >= n - 1) return tmp[n-1];
    return tmp[lo] + fr * (tmp[lo+1] - tmp[lo]);
}
static inline float _corr(float *a, float *b, int n) {
    float ma = _mean(a,n), mb = _mean(b,n);
    float sa = _std(a,n,ma), sb = _std(b,n,mb);
    if (sa < 1e-9f || sb < 1e-9f) return 0;
    float s = 0;
    for (int i = 0; i < n; i++) s += (a[i]-ma) * (b[i]-mb);
    return (s / n) / (sa * sb);
}

// buf[WINDOW_SIZE][6] : AccX, AccY, AccZ, GyroX, GyroY, GyroZ
// feat must be float[NUM_FEATURES]
void extract_features(float buf[WINDOW_SIZE][NUM_AXES], float *feat) {
    int fi = 0;

    // ── Per-axis features (15 × 6 = 90) ─────────────────────────────────
    for (int ax = 0; ax < NUM_AXES; ax++) {
        float d[WINDOW_SIZE];
        for (int i = 0; i < WINDOW_SIZE; i++) d[i] = buf[i][ax];

        float mn = d[0], mx = d[0];
        for (int i = 1; i < WINDOW_SIZE; i++) {
            if (d[i] < mn) mn = d[i];
            if (d[i] > mx) mx = d[i];
        }
        float m  = _mean(d, WINDOW_SIZE);
        float s  = _std(d, WINDOW_SIZE, m);
        float tv = 0, de = 0, mad = 0;
        for (int i = 0; i < WINDOW_SIZE - 1; i++) {
            float diff = d[i+1] - d[i];
            tv  += fabsf(diff);
            de  += diff * diff;
        }
        for (int i = 0; i < WINDOW_SIZE; i++) mad += fabsf(d[i] - m);

        feat[fi++] = m;
        feat[fi++] = s;
        feat[fi++] = mx - mn;
        feat[fi++] = mx;
        feat[fi++] = mn;
        feat[fi++] = _pct(d, WINDOW_SIZE, 50);
        feat[fi++] = _skew(d, WINDOW_SIZE, m, s);
        feat[fi++] = _kurt(d, WINDOW_SIZE, m, s);
        feat[fi++] = _rms(d, WINDOW_SIZE);
        feat[fi++] = _pct(d, WINDOW_SIZE, 25);
        feat[fi++] = _pct(d, WINDOW_SIZE, 75);
        feat[fi++] = tv;
        feat[fi++] = mad / WINDOW_SIZE;
        feat[fi++] = de;
        feat[fi++] = tv / (WINDOW_SIZE - 1);
    }

    // ── Magnitude features (10 × 2 = 20) ────────────────────────────────
    float acc_mag[WINDOW_SIZE], gyro_mag[WINDOW_SIZE];
    for (int i = 0; i < WINDOW_SIZE; i++) {
        acc_mag[i]  = sqrtf(buf[i][0]*buf[i][0] + buf[i][1]*buf[i][1] + buf[i][2]*buf[i][2]);
        gyro_mag[i] = sqrtf(buf[i][3]*buf[i][3] + buf[i][4]*buf[i][4] + buf[i][5]*buf[i][5]);
    }
    float *mags[2] = { acc_mag, gyro_mag };
    for (int m2 = 0; m2 < 2; m2++) {
        float *mg = mags[m2];
        float mm = _mean(mg, WINDOW_SIZE);
        float ms = _std(mg, WINDOW_SIZE, mm);
        float mn2 = mg[0], mx2 = mg[0];
        for (int i = 1; i < WINDOW_SIZE; i++) {
            if (mg[i] < mn2) mn2 = mg[i];
            if (mg[i] > mx2) mx2 = mg[i];
        }
        float tv2 = 0;
        for (int i = 0; i < WINDOW_SIZE - 1; i++) tv2 += fabsf(mg[i+1] - mg[i]);

        feat[fi++] = mm;
        feat[fi++] = ms;
        feat[fi++] = mx2;
        feat[fi++] = mn2;
        feat[fi++] = mx2 - mn2;
        feat[fi++] = _rms(mg, WINDOW_SIZE);
        feat[fi++] = _skew(mg, WINDOW_SIZE, mm, ms);
        feat[fi++] = _kurt(mg, WINDOW_SIZE, mm, ms);
        feat[fi++] = tv2;
        feat[fi++] = tv2 / (WINDOW_SIZE - 1);
    }

    // ── Jerk features (3 × 2 = 6) ───────────────────────────────────────
    float ja[WINDOW_SIZE-1], jg[WINDOW_SIZE-1];
    for (int i = 0; i < WINDOW_SIZE - 1; i++) {
        ja[i] = acc_mag[i+1]  - acc_mag[i];
        jg[i] = gyro_mag[i+1] - gyro_mag[i];
    }
    float *jerks[2] = { ja, jg };
    for (int j = 0; j < 2; j++) {
        float *jk = jerks[j];
        float jm = _mean(jk, WINDOW_SIZE-1);
        float js = _std(jk, WINDOW_SIZE-1, jm);
        float jmax = 0;
        for (int i = 0; i < WINDOW_SIZE - 1; i++) if (fabsf(jk[i]) > jmax) jmax = fabsf(jk[i]);
        feat[fi++] = jm;
        feat[fi++] = js;
        feat[fi++] = jmax;
    }

    // ── Cross-axis correlations (6 + 1 = 7) ────────────────────────────
    int pairs[6][2] = { {0,1},{0,2},{1,2},{3,4},{3,5},{4,5} };
    for (int p = 0; p < 6; p++) {
        float a[WINDOW_SIZE], b[WINDOW_SIZE];
        for (int i = 0; i < WINDOW_SIZE; i++) {
            a[i] = buf[i][pairs[p][0]];
            b[i] = buf[i][pairs[p][1]];
        }
        feat[fi++] = _corr(a, b, WINDOW_SIZE);
    }
    feat[fi++] = _corr(acc_mag, gyro_mag, WINDOW_SIZE);
    // fi == 123
}