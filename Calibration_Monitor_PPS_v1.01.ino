
/*
 GPS PPS vs RTC SQW calibration monitor v1.01
Phase regression with phase unwrapping and quality rejection.
RTC aging register is READ ONLY. No RTC adjustment writes.

There is no need to connect your GPS module serial pins to the microcontroller (ESP8266) in your device. BUT 
you will need to know that your module has a satellite lock and a working PPS signal (and well, this is done through
a serial connection). You can connect your serial pins to a USB-UART adapter for purposes of checking a GNSS satellite fix
(GPS/GLONASS/GALILEO/BEIDOU or any other constellation you may want to use, this is NOT restricted to GPS). 
Do not connect your PPS signal pin to this USB-UART adapter, the pps signal does go to your ESP8266. For a better
quality PPS signal, it is recommended to set your sat module to a timing stationary setting, instead of a moving/automotive
setting. Some modules do not support these features. But basically, any module with a PPS pin will do but their accuracy will 
vary in nanoseconds (we are really splitting hair here). I tested this firmware with a LEA-M8T, ATGM-336H, and GU-U7. The 
ATGM-336H is fine and inexpensive for this application.

Disconnect GPS PPS from GPIO2 during boot. Connect your PPS signal after setup is complete. If you are loading firmware do 
the same or you will have a hard time loading the firmware when the signal is altenating between HIGH and LOW.

Confirm a GPS fix and a stable PPS signal (in some receivers, this can take 15 mins or more).
Make the connection of the GPS PPS to GPIO2 immediately after you see SETUP COMPLETE after boot.
Keep RTC SQW connected to GPIO12.

You will need a min of 5 samples before you see an aging recommendation on the console. 

VERY IMPORTANT: Make sure you have a common ground and a 3.3V-compatible PPS signal, DO NOT use 5V. If you power your 
GPS antenna with 5V that is fine (this is my case with my LEA-M8T) BUT make sure the PPS signal is 3.3V.

In order to get the most accurate suggestion, no wifi and no LED matrix is enabled. This method suffers from micros timer, 
timing capturing issues, temperature, etc. The ESP8266 has no dedicated input capture pins like the ESP32 does. This is 
probably the best you can do for calibrating your RTC without expensive lab equipment.

USE AT YOUR OWN RISK!
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <ESP8266WiFi.h>


static const char FIRMWARE_NAME[] = "Calibration_Monitor_PPS_v1.01";

static const uint8_t RTC_I2C_ADDRESS = 0x68;
static const uint8_t RTC_CONTROL_REGISTER = 0x0E;
static const uint8_t RTC_AGING_REGISTER = 0x10;

static const uint8_t GPS_PPS_PIN = 2;
static const uint8_t RTC_SQW_PIN = 12;
static const uint8_t I2C_SDA_PIN = 5;
static const uint8_t I2C_SCL_PIN = 4;

static const uint8_t LED_MATRIX_DIN_PIN = 13;
static const uint8_t LED_MATRIX_CLK_PIN = 14;
static const uint8_t LED_MATRIX_CS_PIN = 15;
static const uint8_t LED_MATRIX_DEVICES = 4;

// Measurement window
static const uint32_t INTERVALS = 600UL;
static const uint16_t SAMPLE_COUNT = INTERVALS + 1;

// Edge validation
static const uint32_t MIN_GAP_US = 500000UL;
static const uint32_t MAX_GAP_US = 1500000UL;

// Plausible measurement limits
static const double MIN_PERIOD_US = 990000.0;
static const double MAX_PERIOD_US = 1010000.0;
static const double MAX_DRIFT_PPM = 100.0;

// Phase is measured modulo one second.
static const int32_t PHASE_WRAP_US = 1000000L;
static const int32_t PHASE_HALF_WRAP_US = 500000L;

// Phase-quality rejection thresholds.
// These are initial safeguards, not guaranteed hardware specifications.
static const int32_t MAX_PHASE_STEP_US = 10000L;
static const double MAX_PHASE_RESIDUAL_SIGMA_US = 1000.0;
static const double MAX_ABS_PHASE_RESIDUAL_US = 5000.0;

static const double AGING_PPM_PER_STEP = 0.12;

static const uint32_t STATUS_INTERVAL_MS = 10000UL;
static const uint32_t MIN_WINDOWS_FOR_RECOMMENDATION = 5UL;

// ---------- Edge capture state ----------

volatile uint32_t gpsLast = 0;
volatile uint32_t rtcLast = 0;

volatile bool gpsHasLast = false;
volatile bool rtcHasLast = false;
volatile bool rtcPeriodValid = false;

volatile uint32_t gpsEdges = 0;
volatile uint32_t rtcEdges = 0;
volatile uint32_t gpsResets = 0;
volatile uint32_t rtcResets = 0;

// Phase = RTC edge timestamp - GPS edge timestamp,
// wrapped into approximately +/- 500 ms.
volatile int32_t phaseSamples[SAMPLE_COUNT];
volatile uint16_t sampleCount = 0;
volatile bool windowActive = false;
volatile bool measurementReady = false;

volatile uint32_t windowStartGps = 0;
volatile uint32_t windowEndGps = 0;

// ---------- Statistics ----------

uint32_t acceptedWindows = 0;
uint32_t rejectedWindows = 0;

double lastDriftPPM = NAN;
double averageDriftPPM = NAN;
double driftM2 = 0.0;
double driftStdDevPPM = NAN;

bool rtcReady = false;
bool agingReadValid = false;
int16_t agingOffset = 0;

unsigned long lastStatus = 0;

// ---------- MAX7219 shutdown ----------

void max7219WriteAll(uint8_t reg, uint8_t data)
{
  digitalWrite(LED_MATRIX_CS_PIN, LOW);

  for (uint8_t i = 0; i < LED_MATRIX_DEVICES; i++) {
    shiftOut(LED_MATRIX_DIN_PIN, LED_MATRIX_CLK_PIN, MSBFIRST, reg);
    shiftOut(LED_MATRIX_DIN_PIN, LED_MATRIX_CLK_PIN, MSBFIRST, data);
  }

  digitalWrite(LED_MATRIX_CS_PIN, HIGH);
}

void blankAndDisableLedMatrix()
{
  pinMode(LED_MATRIX_CS_PIN, OUTPUT);
  digitalWrite(LED_MATRIX_CS_PIN, HIGH);

  pinMode(LED_MATRIX_DIN_PIN, OUTPUT);
  pinMode(LED_MATRIX_CLK_PIN, OUTPUT);

  digitalWrite(LED_MATRIX_DIN_PIN, LOW);
  digitalWrite(LED_MATRIX_CLK_PIN, LOW);

  max7219WriteAll(0x0F, 0);
  max7219WriteAll(0x09, 0);
  max7219WriteAll(0x0B, 7);
  max7219WriteAll(0x0A, 0);

  for (uint8_t reg = 1; reg <= 8; reg++) {
    max7219WriteAll(reg, 0);
  }

  max7219WriteAll(0x0C, 0);
}

// ---------- RTC I2C ----------

bool readRtcRegister(uint8_t reg, uint8_t &value)
{
  Wire.beginTransmission(RTC_I2C_ADDRESS);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received = Wire.requestFrom(
      (uint8_t)RTC_I2C_ADDRESS, (uint8_t)1);

  if (received != 1 || !Wire.available()) {
    return false;
  }

  value = Wire.read();
  return true;
}

bool configureRtcSqwFor1Hz()
{
  uint8_t control = 0;

  if (!readRtcRegister(RTC_CONTROL_REGISTER, control)) {
    Serial.println(F("ERROR: RTC control register read failed."));
    return false;
  }

  // DS3231M: RS2:RS1 = 00 selects 1 Hz.
  // INTCN = 0 selects square-wave output.
  // Preserve unrelated control-register bits.
  uint8_t updated = control & (uint8_t)~0x1C;

  if (updated != control) {
    Wire.beginTransmission(RTC_I2C_ADDRESS);
    Wire.write(RTC_CONTROL_REGISTER);
    Wire.write(updated);

    if (Wire.endTransmission() != 0) {
      Serial.println(F("ERROR: RTC control-register write failed."));
      return false;
    }
  }

  uint8_t verify = 0;

  if (!readRtcRegister(RTC_CONTROL_REGISTER, verify)) {
    Serial.println(F("ERROR: RTC control-register verification failed."));
    return false;
  }

  if ((verify & 0x1C) != 0) {
    Serial.printf("ERROR: Unexpected RTC control value: 0x%02X\n", verify);
    return false;
  }

  Serial.printf("RTC control register: 0x%02X\n", verify);
  return true;
}

bool readRtcAgingOffset()
{
  uint8_t raw = 0;

  if (!readRtcRegister(RTC_AGING_REGISTER, raw)) {
    agingReadValid = false;
    Serial.println(F("Aging register read failed."));
    return false;
  }

  agingOffset = (raw & 0x80)
      ? (int16_t)raw - 256
      : (int16_t)raw;

  agingReadValid = true;

  Serial.printf("Aging register address: 0x%02X\n", raw);
  Serial.printf("Current Aging register: %d\n",
                (int)agingOffset);

  return true;
}

// ---------- Wrapped phase helper ----------

IRAM_ATTR static inline int32_t wrappedPhase(
    uint32_t rtcTime, uint32_t gpsTime)
{
  int32_t phase = (int32_t)(rtcTime - gpsTime);

  if (phase > PHASE_HALF_WRAP_US) {
    phase -= PHASE_WRAP_US;
  } else if (phase < -PHASE_HALF_WRAP_US) {
    phase += PHASE_WRAP_US;
  }

  return phase;
}

// ---------- Phase unwrapping and continuity validation ----------

// Converts the wrapped phase array IN PLACE into a continuous phase series.
//
// Each adjacent difference is corrected for the +/-500 ms wrap.
// Large remaining steps indicate suspicious timing data and reject the window.
//
// The function runs in loop(), never inside an ISR.
// The sample buffer is frozen while measurementReady is true.

bool unwrapPhaseSamplesInPlace(
    uint16_t &badIndex,
    int32_t &badStep)
{
  badIndex = 0;
  badStep = 0;

  if (SAMPLE_COUNT < 2) {
    return false;
  }

  int32_t previousWrapped = phaseSamples[0];
  int64_t unwrapped = previousWrapped;

  for (uint16_t i = 1; i < SAMPLE_COUNT; i++) {
    // Read the original wrapped value before replacing it.
    const int32_t currentWrapped = phaseSamples[i];

    int32_t delta = currentWrapped - previousWrapped;

    // Remove the artificial jump at the +/-500 ms boundary.
    if (delta > PHASE_HALF_WRAP_US) {
      delta -= PHASE_WRAP_US;
    } else if (delta < -PHASE_HALF_WRAP_US) {
      delta += PHASE_WRAP_US;
    }

    const int32_t absDelta = (delta < 0) ? -delta : delta;

    // Reject implausible point-to-point phase changes.
    if (absDelta > MAX_PHASE_STEP_US) {
      badIndex = i;
      badStep = delta;
      return false;
    }

    unwrapped += delta;

    // Protect the int32_t storage from overflow.
    if (unwrapped > 2147483647LL ||
        unwrapped < -2147483648LL) {
      badIndex = i;
      badStep = delta;
      return false;
    }

    phaseSamples[i] = (int32_t)unwrapped;

    // Keep the previous RAW wrapped phase for the next difference.
    previousWrapped = currentWrapped;
  }

  return true;
}

// ---------- GPS PPS interrupt ----------

IRAM_ATTR void gpsPpsISR()
{
  const uint32_t now = micros();
  gpsEdges++;

  if (!gpsHasLast) {
    gpsHasLast = true;
    gpsLast = now;
    return;
  }

  const uint32_t gap = now - gpsLast;

  // Ignore glitches without moving the valid-edge baseline.
  if (gap < MIN_GAP_US) {
    return;
  }

  gpsLast = now;

  if (gap > MAX_GAP_US) {
    gpsResets++;
    windowActive = false;
    sampleCount = 0;
    return;
  }

  // Freeze the completed buffer until loop() processes it.
  if (measurementReady) {
    return;
  }

  if (!rtcPeriodValid) {
    windowActive = false;
    sampleCount = 0;
    return;
  }

  const uint32_t rtcTime = rtcLast;

  if ((uint32_t)(now - rtcTime) > MAX_GAP_US) {
    windowActive = false;
    sampleCount = 0;
    return;
  }

  const int32_t phase = wrappedPhase(rtcTime, now);

  if (!windowActive) {
    phaseSamples[0] = phase;
    sampleCount = 1;
    windowStartGps = now;
    windowActive = true;
    return;
  }

  const uint16_t index = sampleCount;

  if (index >= SAMPLE_COUNT) {
    windowActive = false;
    sampleCount = 0;
    return;
  }

  phaseSamples[index] = phase;
  sampleCount = index + 1;

  if (sampleCount >= SAMPLE_COUNT) {
    windowEndGps = now;
    windowActive = false;
    measurementReady = true;
  }
}

// ---------- RTC SQW interrupt ----------

IRAM_ATTR void rtcSqwISR()
{
  const uint32_t now = micros();
  rtcEdges++;

  // Do not measure SQW until at least one GPS PPS edge has been seen.
  if (gpsEdges == 0) {
    return;
  }

  if (!rtcHasLast) {
    rtcHasLast = true;
    rtcLast = now;
    rtcPeriodValid = false;
    return;
  }

  const uint32_t gap = now - rtcLast;

  if (gap < MIN_GAP_US) {
    return;
  }

  rtcLast = now;

  if (gap > MAX_GAP_US) {
    rtcResets++;
    rtcPeriodValid = false;
    windowActive = false;
    sampleCount = 0;
    return;
  }

  rtcPeriodValid = true;
}

// ---------- Reporting ----------

void printAgingRecommendation(double ppm)
{
  Serial.println();
  Serial.println(F("AGING OFFSET RECOMMENDATION (ADVISORY ONLY)"));

  if (!isfinite(ppm) || !agingReadValid) {
    Serial.println(F("Recommendation unavailable."));
    return;
  }

  if (acceptedWindows < MIN_WINDOWS_FOR_RECOMMENDATION) {
    Serial.printf("Need %lu accepted windows; You have %lu.\n",
        (unsigned long)MIN_WINDOWS_FOR_RECOMMENDATION,
        (unsigned long)acceptedWindows);
    return;
  }

  if (!isfinite(driftStdDevPPM)) {
    Serial.println(F("Window variation unavailable."));
    return;
  }

  const long steps = lround(ppm / AGING_PPM_PER_STEP);
  const long proposed = (long)agingOffset + steps;

  Serial.printf("Mean drift: %+.5f ppm\n", ppm);
  Serial.printf("Window standard deviation: %.5f ppm\n",
                driftStdDevPPM);
  Serial.printf("Nominal sensitivity: %.3f ppm/step\n",
                AGING_PPM_PER_STEP);
  Serial.printf("Current aging value: %+d\n", (int)agingOffset);
  Serial.printf("Suggested adjustment: %+ld steps\n", steps);

  if (proposed < -127 || proposed > 127) {
    Serial.printf("Proposed value %+ld is outside expected range.\n",
                  proposed);
  } else {
    Serial.printf("Proposed register value: %+ld\n", proposed);
  }

  Serial.println(F("No aging-register write is performed."));
}

// ---------- Main calibration processor ----------

void processCalibration()
{
  bool ready = false;
  uint32_t startTime = 0;
  uint32_t endTime = 0;

  // Atomically snapshot the completed window.
  noInterrupts();

  ready = measurementReady;

  if (ready) {
    startTime = windowStartGps;
    endTime = windowEndGps;
  }

  interrupts();

  if (ready) {
    const uint32_t elapsedGps = endTime - startTime;
    const double gpsPeriodUs =
        (double)elapsedGps / (double)INTERVALS;

    uint16_t badPhaseIndex = 0;
    int32_t badPhaseStep = 0;

    // FIX 1: unwrap the phase BEFORE regression.
    const bool phaseUnwrapOK =
        unwrapPhaseSamplesInPlace(badPhaseIndex, badPhaseStep);

    double slope = NAN;
    double intercept = NAN;
    double ppm = NAN;
    double residualSigma = NAN;
    double maxAbsResidual = NAN;

    bool regressionOK = false;

    if (phaseUnwrapOK) {
      const double n = (double)SAMPLE_COUNT;

      double sumX = 0.0;
      double sumY = 0.0;
      double sumXX = 0.0;
      double sumXY = 0.0;

      for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
        const double x = (double)i;
        const double y = (double)phaseSamples[i];

        sumX += x;
        sumY += y;
        sumXX += x * x;
        sumXY += x * y;
      }

      const double denominator = n * sumXX - sumX * sumX;

      if (denominator != 0.0) {
        slope = (n * sumXY - sumX * sumY) / denominator;
        intercept = (sumY - slope * sumX) / n;

        if (isfinite(slope) && isfinite(intercept)) {
          double residualSum = 0.0;
          maxAbsResidual = 0.0;

          for (uint16_t i = 0; i < SAMPLE_COUNT; i++) {
            const double x = (double)i;
            const double predicted = intercept + slope * x;
            const double residual =
                (double)phaseSamples[i] - predicted;

            residualSum += residual * residual;

            const double absResidual = fabs(residual);
            if (absResidual > maxAbsResidual) {
              maxAbsResidual = absResidual;
            }
          }

          residualSigma = sqrt(
              residualSum / (double)(SAMPLE_COUNT - 2));

          // Positive ppm means RTC fast.
          // Positive phase slope means RTC slow.
          ppm = -slope / gpsPeriodUs * 1000000.0;

          regressionOK =
              isfinite(ppm) &&
              isfinite(residualSigma) &&
              isfinite(maxAbsResidual);
        }
      }
    }

    const bool periodOK =
        isfinite(gpsPeriodUs) &&
        gpsPeriodUs >= MIN_PERIOD_US &&
        gpsPeriodUs <= MAX_PERIOD_US;

    const bool driftOK =
        regressionOK &&
        fabs(ppm) <= MAX_DRIFT_PPM;

    // FIX 2: check regression residuals before accepting a window.
    const bool residualQualityOK =
        regressionOK &&
        residualSigma <= MAX_PHASE_RESIDUAL_SIGMA_US &&
        maxAbsResidual <= MAX_ABS_PHASE_RESIDUAL_US;

    if (!phaseUnwrapOK) {
      rejectedWindows++;

      Serial.println();
      Serial.println(F("WINDOW REJECTED: phase discontinuity"));
      Serial.printf("Bad sample index: %u\n",
                    (unsigned)badPhaseIndex);
      Serial.printf("Unwrapped phase step: %+ld us\n",
                    (long)badPhaseStep);
      Serial.printf("Allowed absolute step: %ld us\n",
                    (long)MAX_PHASE_STEP_US);
      Serial.println(F("Running average was NOT changed."));

    } else if (elapsedGps == 0 ||
               !periodOK ||
               !driftOK) {
      rejectedWindows++;

      Serial.println();
      Serial.println(F("WINDOW REJECTED: period/drift validation failed"));
      Serial.printf("GPS elapsed: %lu us\n",
                    (unsigned long)elapsedGps);
      Serial.printf("GPS period: %.3f us\n", gpsPeriodUs);
      Serial.printf("Regression slope: %+.6f us/interval\n", slope);
      Serial.printf("Calculated drift: %+.5f ppm\n", ppm);
      Serial.printf("Phase residual sigma: %.3f us\n", residualSigma);
      Serial.printf("Maximum phase residual: %.3f us\n",
                    maxAbsResidual);
      Serial.println(F("Running average was NOT changed."));

    } else if (!residualQualityOK) {
      rejectedWindows++;

      Serial.println();
      Serial.println(F("WINDOW REJECTED: phase quality limits exceeded"));
      Serial.printf("GPS period: %.3f us\n", gpsPeriodUs);
      Serial.printf("Calculated drift: %+.5f ppm\n", ppm);
      Serial.printf("Phase residual sigma: %.3f us (limit %.3f)\n",
                    residualSigma,
                    MAX_PHASE_RESIDUAL_SIGMA_US);
      Serial.printf("Maximum phase residual: %.3f us (limit %.3f)\n",
                    maxAbsResidual,
                    MAX_ABS_PHASE_RESIDUAL_US);
      Serial.println(F("Running average was NOT changed."));

    } else {
      acceptedWindows++;
      lastDriftPPM = ppm;

      // Welford's running mean and sample variance,
      // using accepted windows only.
      if (acceptedWindows == 1) {
        averageDriftPPM = ppm;
        driftM2 = 0.0;
      } else {
        const double delta = ppm - averageDriftPPM;
        averageDriftPPM += delta / (double)acceptedWindows;
        const double delta2 = ppm - averageDriftPPM;
        driftM2 += delta * delta2;
      }

      driftStdDevPPM = acceptedWindows > 1
          ? sqrt(driftM2 / (double)(acceptedWindows - 1))
          : NAN;

      const int32_t phaseChange =
          phaseSamples[SAMPLE_COUNT - 1] - phaseSamples[0];

      Serial.println();
      Serial.println(F("=============================================="));
      Serial.println(F("GPS PPS vs RTC SQW CALIBRATION"));
      Serial.printf("Accepted windows: %lu\n",
                    (unsigned long)acceptedWindows);
      Serial.printf("Rejected windows: %lu\n",
                    (unsigned long)rejectedWindows);
      Serial.printf("Intervals: %lu\n", (unsigned long)INTERVALS);
      Serial.printf("GPS elapsed: %lu us\n",
                    (unsigned long)elapsedGps);
      Serial.printf("GPS period: %.3f us\n", gpsPeriodUs);

      Serial.printf("Start unwrapped phase: %+ld us\n",
                    (long)phaseSamples[0]);
      Serial.printf("End unwrapped phase: %+ld us\n",
                    (long)phaseSamples[SAMPLE_COUNT - 1]);
      Serial.printf("Phase change (sanity check): %+ld us\n", (long)phaseChange);

      Serial.printf("Regression slope: %+.6f us/interval\n", slope);
      Serial.printf("Regression residual sigma: %.3f us\n",
                    residualSigma);
      Serial.printf("Maximum absolute residual: %.3f us\n",
                    maxAbsResidual);

      Serial.printf("RTC drift vs GPS: %+.5f ppm\n", lastDriftPPM);
      Serial.printf("Running mean: %+.5f ppm\n", averageDriftPPM);

      if (isfinite(driftStdDevPPM)) {
        Serial.printf("Window standard deviation: %.5f ppm\n",
                      driftStdDevPPM);
      }

      Serial.printf("Last drift: %+.5f sec/day\n",
                    lastDriftPPM * 0.0864);
      Serial.printf("Mean drift: %+.5f sec/day\n",
                    averageDriftPPM * 0.0864);

      Serial.println(F("Positive ppm = RTC fast; negative ppm = RTC slow."));
      Serial.println(F("No RTC time or aging-register write is performed."));

      printAgingRecommendation(averageDriftPPM);

      Serial.println(F("=============================================="));
    }

    // Release the completed buffer only after processing is finished.
    noInterrupts();
    measurementReady = false;
    windowActive = false;
    sampleCount = 0;
    interrupts();
  }

  // ---------- Periodic status ----------

  const unsigned long now = millis();

  if (now - lastStatus >= STATUS_INTERVAL_MS) {
    lastStatus = now;

    uint32_t ge, re, gr, rr;
    uint16_t samples;
    bool active, readyCopy, rtcValid;

    noInterrupts();

    ge = gpsEdges;
    re = rtcEdges;
    gr = gpsResets;
    rr = rtcResets;
    samples = sampleCount;
    active = windowActive;
    readyCopy = measurementReady;
    rtcValid = rtcPeriodValid;

    interrupts();

    Serial.printf(
        "Status: GPS=%lu RTC=%lu samples=%u/%u "
        "%s GPSresets=%lu RTCresets=%lu RTCvalid=%d\n",
        (unsigned long)ge,
        (unsigned long)re,
        (unsigned)samples,
        (unsigned)SAMPLE_COUNT,
        readyCopy ? "ready" : (active ? "capturing" : "waiting"),
        (unsigned long)gr,
        (unsigned long)rr,
        rtcValid ? 1 : 0);
  }
}

// ---------- Wi-Fi shutdown ----------

void disableWiFiRadio()
{
  WiFi.persistent(false);

  const bool modeOK = WiFi.mode(WIFI_OFF);
  const bool sleepOK = WiFi.forceSleepBegin();

  delay(10);

  Serial.printf("Wi-Fi off=%d, forced sleep=%d\n",
                modeOK ? 1 : 0,
                sleepOK ? 1 : 0);
}

// ---------- Setup / loop ----------

void setup()
{
  Serial.begin(115200);
  delay(100);

  disableWiFiRadio();
  blankAndDisableLedMatrix();

  Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(FIRMWARE_NAME);
  Serial.println(F("Software timestamps: interrupts + micros()"));
  Serial.println(F("Phase regression with unwrapping and rejection"));
  Serial.println(F("Aging register read-only; no RTC correction."));
  Serial.printf("Window: %lu intervals\n", (unsigned long)INTERVALS);
  Serial.printf("Phase step limit: %ld us\n",
                (long)MAX_PHASE_STEP_US);
  Serial.printf("Residual sigma limit: %.1f us\n",
                MAX_PHASE_RESIDUAL_SIGMA_US);
  Serial.printf("Maximum residual limit: %.1f us\n",
                MAX_ABS_PHASE_RESIDUAL_US);
  Serial.println(F("Disconnect GPS PPS from GPIO2 during boot. Connect your PPS signal after setup is complete"));
  Serial.println(F("=============================================="));

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  if (!configureRtcSqwFor1Hz()) {
    Serial.println(F("FATAL: RTC setup failed; check wiring."));
    rtcReady = false;
    return;
  }

  readRtcAgingOffset();

  pinMode(RTC_SQW_PIN, INPUT_PULLUP);
  pinMode(GPS_PPS_PIN, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(RTC_SQW_PIN), rtcSqwISR, RISING);
  attachInterrupt(
      digitalPinToInterrupt(GPS_PPS_PIN), gpsPpsISR, RISING);

  rtcReady = true;

  Serial.println(F("SETUP COMPLETE"));
  Serial.println(F("Confirm a GPS fix and a stable PPS signal (in some receivers, this can take 15 mins or more)."));
  Serial.println(F("Connect GPS PPS to GPIO2 immediately after you see SETUP COMPLETE after boot."));
  Serial.println(F("Keep RTC SQW connected to GPIO12."));
  Serial.println(F("VERY IMPORTANT: Ensure a common ground and a 3.3V-compatible PPS signal, DO NOT use 5V."));
}

void loop()
{
  if (!rtcReady) {
    delay(1000);
    return;
  }

  processCalibration();
  delay(1);
}