/*
ESP8266 / DS3231M calibration monitor v1.00

PRIMARY INFORMATION
- Wi-Fi connection is managed via WiFiManager.
- NTP is a read-only reference. This firmware does not set or discipline
  the ESP8266 system clock or set the DS3231M time.
- DS3231M 1 Hz SQW measurement on GPIO12. Change DS3231_SQW_PIN if your
  wiring/device uses a different GPIO.

- Method A: ESP TIMER vs SQW INTERVAL.
- Method B: NTP TIMESTAMPS vs SQW PHASE.

- Method A estimates the ESP8266 micros() rate against NTP observations and
  corrects the measured SQW interval error.
- Method B estimates RTC/SQW rate directly using linear regression.

- Both methods provide read-only aging-register recommendations. The
  firmware reads the existing aging register but does NOT write to it.
- A run of at least 24 hours is recommended for calibration. Longer runs
  can improve averaging and reveal longer-term variation, but do not
  automatically guarantee greater accuracy if operating conditions change.
- Both methods can be affected by variable network delay, jitter, software
  timing and interrupt timing. Method B is particularly sensitive to
  network path asymmetry and phase-capture timing.
- No calibration/status web UI or LED matrix is implemented.
- Calibration statistics and regression state are RAM-only. Reboots or
  resets lose the accumulated data. The firmware does not save calibration
  statistics to persistent storage.

NTP SERVER SELECTION AND POLLING
- A local NTP server is HIGHLY recommended.
- The first entry in NTP_SERVERS MUST be set to your local NTP server's
  address. It is the preferred server; the other two are backups.
- The firmware-enforced minimum request intervals are:

    Server 1: local NTP server, 16 seconds
    Server 2: time.nist.gov, 64 seconds
    Server 3: pool.ntp.org, 34 minutes 8 seconds plus 0-120 seconds of random jitter

- These are the intervals configured in this firmware. Please make sure you don't abuse those services by
  entering very short sync intervals.
- If Server 1 fails, a working backup remains active while Server 1 is
  periodically probed for recovery. The primary server is not probed more
  frequently than its configured 16-second interval. A successful probe
  returns the sketch to Server 1.
- Per-server interval limits apply on failover and primary probes. If a server
  returns a verified RATE Kiss-o'-Death response, the firmware backs off that
  server to at least 64 seconds, doubling after each further RATE response up
  to 2 hours. The server's poll field is reported for diagnostics, but is not
  trusted as an arbitrary delay. A working backup remains active while Server
  1 is probed when its backoff interval allows.
- When a previously used server resumes, the firmware checks timestamp
  continuity if sufficient prior ESP timer regression data exists. A
  detected discontinuity exceeding 2 seconds invalidates that server's
  active calibration fit. If there is insufficient prior data to perform
  the check, the discontinuity cannot be verified by that test.
- Each server's NTP regression and associated calibration state are retained
  separately in RAM. NTP observations from different servers are not
  combined into a single regression.
- SQW interval measurements are accumulated globally. Method A uses the
  SQW windows within the selected server's ESP timer regression span,
  including windows collected while a backup server was active.
- Server switching saves and restores the per-server calibration state.
  All retained state is lost if the ESP8266 reboots or resets.
- Serial console report interval is 10 seconds.

METHOD A - ESP TIMER vs SQW INTERVAL
- E is the ESP8266 micros() rate error, estimated by linear regression against
  NTP server timestamps.
- S is the measured SQW interval error in ppm, calculated from micros()
  elapsed time across ten nominal 1 Hz SQW intervals.
- Method A uses the mean of accepted S windows between that server's first
  and latest accepted NTP observations. The window boundaries have
  approximately 10-second granularity.
- The code combines E and S using the rate-ratio relationship:

    R = ((1 + E/1e6) / (1 + S/1e6) - 1) * 1e6 ppm

  For small ppm values, this is approximately R = E - S.
- Method A requires a validated ESP timer estimate and at least 30 accepted
  SQW interval windows (default) within the server's ESP timer regression span.
- ESP timer validation requires at least 30 minutes of observations (default),
  followed by a successful 10-minute stability checkpoint. A change
  exceeding the configured 2 ppm tolerance invalidates the estimate and
  restarts the stability qualification. WARNING: This can extend the time before you see a suggested recommendation.
  This fisrt suggested recommendation will be around the 40 minute mark (if the stability requirement is satisfied).

METHOD B - NTP TIMESTAMPS vs SQW PHASE
- Each eligible observation pairs the midpoint of the SQW phase measured
  just before UDP packet submission and when the response becomes available
  with the midpoint of the NTP server receive/transmit timestamps (t2/t3).
- These are software-side timing approximations, not exact Wi-Fi radio
  transmission or reception timestamps.
- Linear regression of NTP timestamp midpoint against SQW phase estimates
  RTC/SQW rate directly.
- The fractional SQW phase normalizes the stable micros() rate scale, although interrupt and capture timing errors can 
  still affect the result.
- Observations with RTT above 750 ms are rejected for Method B.
  Residual rejection is activated only after at least 30 accepted samples (default)
  and at least 10 minutes of observation span.
- The aging recommendation requires at least 30 accepted phase samples
  spanning at least 30 minutes.

INTERPRETING DRIFT
- Positive ppm means the RTC/SQW rate is fast relative to the NTP reference.
- Negative ppm means the RTC/SQW rate is slow relative to the NTP reference.
- Both methods are ESTIMATES. Agreement between them is useful as a
  cross-check, but does not by itself prove the absolute accuracy of either
  estimate or guarantee the optimal aging-register setting.
  USE AT YOUR OWN RISK!
*/

#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <RTClib.h>
#include <time.h>
#include <math.h>
#include <string.h>

#define FIRMWARE_VERSION "v1.00"
#define DS3231_ADDRESS 0x68
#define DS3231_SQW_PIN 12       // DS3231M SQW 1 Hz output to ESP8266 GPIO12
#define I2C_SDA 5
#define I2C_SCL 4

static const uint32_t MEASURE_INTERVAL_MS = 10000UL; // Serial report and local-server polling interval
static const uint32_t WIFI_RETRY_INTERVAL_MS = 30000UL;
static const uint32_t NTP_TIMEOUT_MS = 2500UL;
static const uint16_t NTP_PORT = 123;
static const uint32_t NTP_FAILOVER_RESCAN_MS = 1000UL;
static const uint32_t PRIMARY_SERVER_PROBE_INTERVAL_MS = 16000UL; // do not probe primary more often than this, in addition to per-server due time
static const uint32_t NTP_RATE_BACKOFF_INITIAL_MS = 64000UL;   // first valid RATE KoD: at least 64 seconds
static const uint32_t NTP_RATE_BACKOFF_MAX_MS = 7200000UL;    // cap adaptive backoff at 2 hours
static const double SERVER_RESUME_MAX_CONTINUITY_RESIDUAL_SECONDS = 2.0; // reject resumed server clock steps
static const uint8_t NTP_SERVER_COUNT = 3;
// Edit these three entries for your installation. Server 1 is the local NTP server.
// Public server poll intervals have per-server rate limits. The sketch uses one
// server at a time and switches only after a request failure.
static const char * const NTP_SERVERS[NTP_SERVER_COUNT] = {
  "192.168.0.1",    // 1: local NTP server. WARNING: This must be changed to your own local NTP server.
  "time.nist.gov",  // 2: public fallback
  "pool.ntp.org"    // 3: public fallback
};
// Configured minimum intervals between requests: local 16 s, NIST 64 s,
// NTP Pool at least 30 minutes. Pool requests add 0-120 s jitter. A verified
// RATE KoD adds a separate per-server backoff that can increase up to 2 hours.
static const uint32_t NTP_SERVER_MIN_INTERVAL_MS[NTP_SERVER_COUNT] = {
  16000UL,    // Local NTP: 16 seconds
  64000UL,    // NIST: 64 seconds
  2048000UL   // NTP Pool: 2048 seconds (34 min 8 sec), plus jitter
};
static const double AGING_PPM_PER_STEP = 0.12; // advisory estimate only
static const uint32_t MIN_REGRESSION_SAMPLES = 3; // ESP timer estimate used by Method A
// ESP timer estimate safeguards
static const double MAX_ESP_TIMER_DRIFT_PPM = 100.0;
static const double MIN_ESP_TIMER_SPAN_SECONDS = 1800.0; // 30 minutes
static const double ESP_TIMER_STABILITY_TOLERANCE_PPM = 2.0;
static const double ESP_TIMER_STABILITY_CHECK_SECONDS = 600.0; // 10 minutes
// Each new valid NTP estimate is checked against both the fixed validation
// baseline and the last checkpoint. Either change exceeding 2 ppm invalidates
// Method A immediately and restarts the 10-minute stability period.

// Do not enable residual rejection until the initial regression has enough
// accepted observations over a meaningful time span. This avoids locking the
// regression onto an early, noisy slope and rejecting every later sample.
static const uint32_t RTC_PHASE_OUTLIER_MIN_SAMPLES = 30;
static const double RTC_PHASE_OUTLIER_MIN_SPAN_SECONDS = 600.0; // 10 minutes before residual gating
static const uint32_t MIN_RTC_RECOMMENDATION_SAMPLES = 30; // require 30 accepted Method B samples
static const double MIN_RTC_RECOMMENDATION_SPAN_SECONDS = 1800.0; // require >= 30 minutes
static const uint32_t MAX_RTC_PHASE_RTT_US = 750000UL; // discard high-delay phase pairs
static const double RTC_PHASE_OUTLIER_FLOOR_SEC = 0.050; // minimum 50 ms residual gate
static const double RTC_PHASE_OUTLIER_SIGMA_MULTIPLIER = 6.0;
// The diagnostic window is exactly ten captured SQW intervals. If it falls
// well outside 10 seconds, treat it as a missed-edge/stale-window condition,
// not as a real clock-rate measurement.
static const uint32_t SQW_DIAG_MIN_ELAPSED_US = 9900000UL;
static const uint32_t SQW_DIAG_MAX_ELAPSED_US = 10100000UL;

// A valid SQW window must have been accepted within this time
// for the report to say lifetime statistics are actively recording.
static const uint32_t SQW_RECORDING_TIMEOUT_MS = 30000UL;

// Require at least 30 accepted SQW windows (~5 minutes at the expected rate)
// within the selected server's ESP-timer regression span before Method A is ready.
static const uint32_t MIN_SQW_CROSSCHECK_WINDOWS = 30;

// Reject SQW diagnostic samples outside this plausible range.
static const double MAX_SQW_DIAG_PPM = 100.0;

WiFiManager wifiManager;
RTC_DS3231 rtc;
WiFiUDP udp;
uint8_t ntpPacket[48];

bool rtcAvailable = false;
bool agingOffsetValid = false;
int8_t agingOffsetValue = 0;

// Continuous SQW phase state. A separate 10-interval diagnostic window is
// measured in micros(); Method B uses edge count + fractional phase.
volatile uint32_t sqwEdgeCount = 0;
volatile uint32_t sqwLastEdgeMicros = 0;
volatile uint32_t sqwPeriodEstimateMicros = 1000000UL; // EWMA of micros() ticks per SQW cycle
volatile bool sqwHaveEdge = false;
volatile uint32_t sqwDiagStartMicros = 0;
volatile uint32_t sqwDiagIntervals = 0;
volatile bool sqwDiagStarted = false;
volatile uint32_t sqwReadyElapsedMicros = 0;
volatile bool sqwWindowReady = false;

double ntpRequestSqwPhase = NAN;
bool ntpRequestSqwPhaseValid = false;
bool ntpRequestTransmitTimestampValid = false;
uint32_t ntpRequestStartedMicros = 0;

// Online least-squares regression of monotonic local time against the active NTP server.
// RAM-only; its full state is saved/restored per server to avoid mixing time offsets.
uint32_t ntpSamplesAccepted = 0;
double regressionMeanLocal = 0.0;
double regressionMeanNtp = 0.0;
double regressionSxx = 0.0;
double regressionSxy = 0.0;
double firstNtpLocalSeconds = NAN;
double lastNtpLocalSeconds = NAN;
uint32_t ntpTimeoutCount = 0;
uint32_t ntpInvalidResponseCount = 0;
uint32_t ntpServerFailures[NTP_SERVER_COUNT] = {0, 0, 0};
uint32_t ntpServerSwitchCount = 0;
uint8_t ntpServerIndex = 0;
uint32_t ntpServerLastRequestMs[NTP_SERVER_COUNT] = {0, 0, 0};
uint32_t ntpServerLastIntervalMs[NTP_SERVER_COUNT] = { 16000UL, 64000UL, 2048000UL };
bool ntpServerEverRequested[NTP_SERVER_COUNT] = {false, false, false};
// RATE KoD handling is kept separately from calibration state. Each server's
// backoff survives failover and is increased only by a verified RATE response.
uint32_t ntpServerRateBackoffMs[NTP_SERVER_COUNT] = {0, 0, 0};
uint32_t ntpServerRateKodCount[NTP_SERVER_COUNT] = {0, 0, 0};
bool ntpFailoverPending = false;
unsigned long lastFailoverScanMs = 0;
unsigned long lastPrimaryProbeAttemptMs = 0;
bool ntpPrimaryProbePending = false;
uint8_t ntpProbeReturnServerIndex = 0;
bool ntpResumeCheckPending = false;
unsigned long ntpRequestStartedMs = 0;
unsigned long lastWifiRetryMs = 0;
unsigned long lastReportMs = 0;
bool ntpPending = false;
double lastNtpRttMs = NAN;
uint32_t lastPairMicros = 0;
double extendedLocalMicros = 0.0;
bool localMicrosInitialized = false;
double espTimerDriftPPM = NAN; // E
bool espEstimateValid = false;

// ESP timer estimate validation state.
// The stability reference remains fixed for a validation period so cumulative
// drift changes cannot be hidden by moving the reference at every checkpoint.
// The previous checkpoint is tracked separately to detect abrupt changes.
double espDriftStabilityReferencePPM = NAN;
double espDriftStabilityReferenceLocalSeconds = NAN;
double espDriftLastCheckpointPPM = NAN;
double espDriftLastCheckpointLocalSeconds = NAN;
bool espDriftStable = false;

// Short SQW interval diagnostic: elapsed micros() time across 10 SQW periods.
// S is approximately E - RTC_rate. Lifetime and current-server summaries are
// retained for diagnostics. Method A instead uses cumulative-sum snapshots at
// each server's first and latest accepted NTP observations, so its S mean spans
// the same observation interval as that server's ESP-timer regression.
double sqwIntervalErrorMeanPPM = NAN;
uint32_t sqwIntervalWindowCount = 0;
uint32_t sqwIntervalRejectedWindowCount = 0;
uint32_t sqwIntervalRejectedNonFiniteCount = 0;
uint32_t sqwIntervalRejectedPlausibilityCount = 0;
double sqwIntervalPpmSum = 0.0;
double sqwIntervalMinPPM = INFINITY;
double sqwIntervalMaxPPM = -INFINITY;
double sqwIntervalLatestPPM = NAN;
double sqwIntervalCurrentServerPpmSum = 0.0;
uint32_t sqwIntervalCurrentServerWindowCount = 0;
uint32_t sqwIntervalCurrentServerFirstWindowMs = 0;
bool sqwIntervalCurrentServerHaveFirstWindow = false;

// Per-active-server snapshots of the lifetime SQW accumulators. The start is
// recorded with the first accepted NTP observation in that server's regression;
// the end is refreshed with every later accepted NTP observation. Differences
// therefore approximate SQW windows spanning that server's regression, with
// boundaries quantized by the ten-SQW-interval diagnostic window.
double sqwIntervalSpanStartPpmSum = 0.0;
uint32_t sqwIntervalSpanStartWindowCount = 0;
double sqwIntervalSpanEndPpmSum = 0.0;
uint32_t sqwIntervalSpanEndWindowCount = 0;
bool sqwIntervalSpanStartValid = false;

uint32_t sqwLastAcceptedWindowMs = 0;
bool sqwHasAcceptedWindow = false;

// Direct RTC-vs-NTP phase regression (one independent RAM-only fit per server).
uint32_t rtcPhaseSamplesAccepted = 0;
uint32_t rtcPhaseRejectedHighRtt = 0;
uint32_t rtcPhaseRejectedNoSqw = 0;
uint32_t rtcPhaseRejectedBadTimestamp = 0;
uint32_t rtcPhaseRejectedNonFinite = 0;
uint32_t rtcPhaseRejectedResidual = 0;
double rtcPhaseBaseRaw = NAN;
double rtcPhaseBaseNtp = NAN;
double rtcPhaseFirstRaw = NAN;
double rtcPhaseLastRaw = NAN;
double rtcPhaseMeanX = 0.0;
double rtcPhaseMeanY = 0.0;
double rtcPhaseSxx = 0.0;
double rtcPhaseSxy = 0.0;
double rtcPhaseSyy = 0.0;
double rtcPhaseSlopeNtpPerSqw = NAN;
double rtcDriftVsNtpPPM = NAN; // positive means RTC/SQW is fast vs NTP
double rtcPhaseResidualSigmaUs = NAN;
double rtcMaxPriorFitResidualUs = 0.0;

// A complete active-calibration snapshot for each NTP server. This keeps each
// server's regression and validation history separate while another server is used.
struct ServerCalibrationState {
  uint32_t ntpSamplesAccepted;
  double regressionMeanLocal;
  double regressionMeanNtp;
  double regressionSxx;
  double regressionSxy;
  double firstNtpLocalSeconds;
  double lastNtpLocalSeconds;
  double espTimerDriftPPM;
  bool espEstimateValid;
  double espDriftStabilityReferencePPM;
  double espDriftStabilityReferenceLocalSeconds;
  double espDriftLastCheckpointPPM;
  double espDriftLastCheckpointLocalSeconds;
  bool espDriftStable;
  double sqwIntervalCurrentServerPpmSum;
  uint32_t sqwIntervalCurrentServerWindowCount;
  uint32_t sqwIntervalCurrentServerFirstWindowMs;
  bool sqwIntervalCurrentServerHaveFirstWindow;
  double sqwIntervalSpanStartPpmSum;
  uint32_t sqwIntervalSpanStartWindowCount;
  double sqwIntervalSpanEndPpmSum;
  uint32_t sqwIntervalSpanEndWindowCount;
  bool sqwIntervalSpanStartValid;
  uint32_t rtcPhaseSamplesAccepted;
  uint32_t rtcPhaseRejectedHighRtt;
  uint32_t rtcPhaseRejectedNoSqw;
  uint32_t rtcPhaseRejectedBadTimestamp;
  uint32_t rtcPhaseRejectedNonFinite;
  uint32_t rtcPhaseRejectedResidual;
  double rtcPhaseBaseRaw;
  double rtcPhaseBaseNtp;
  double rtcPhaseFirstRaw;
  double rtcPhaseLastRaw;
  double rtcPhaseMeanX;
  double rtcPhaseMeanY;
  double rtcPhaseSxx;
  double rtcPhaseSxy;
  double rtcPhaseSyy;
  double rtcPhaseSlopeNtpPerSqw;
  double rtcDriftVsNtpPPM;
  double rtcPhaseResidualSigmaUs;
  double rtcMaxPriorFitResidualUs;
  bool resumeCheckPending;
};

ServerCalibrationState ntpCalibrationState[NTP_SERVER_COUNT];
bool ntpCalibrationStateInitialized[NTP_SERVER_COUNT] = {false, false, false};

// NTP epoch and local monotonic time are retained as a measurement pair only.
// They are never passed to settimeofday(), configTime(), or RTC.adjust().
static double ntpTimestampToUnix(const uint8_t *p) {
  uint32_t seconds = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8) | (uint32_t)p[3];
  uint32_t fraction = ((uint32_t)p[4] << 24) | ((uint32_t)p[5] << 16) |
                      ((uint32_t)p[6] << 8) | (uint32_t)p[7];
  // NTP epoch starts in 1900; Unix epoch starts in 1970. Do not subtract
  // the epoch offset from zero/invalid pre-Unix seconds: unsigned subtraction
  // would wrap and turn an invalid timestamp into a plausible-looking 2036 date.
  // This firmware expects modern timestamps (Unix time after 2020) and does not
  // attempt to handle the NTP seconds-era rollover in 2036.
  const uint32_t NTP_UNIX_EPOCH_OFFSET = 2208988800UL;
  if (seconds < NTP_UNIX_EPOCH_OFFSET) return NAN;
  double unixSeconds = (double)(seconds - NTP_UNIX_EPOCH_OFFSET) +
                       (double)fraction / 4294967296.0;
  if (!isfinite(unixSeconds) || unixSeconds <= 0.0) return NAN;
  return unixSeconds;
}

void IRAM_ATTR sqwISR() {
  uint32_t now = micros();
  uint32_t edgeIncrements = 1;
  if (sqwHaveEdge) {
    uint32_t periodUs = (uint32_t)(now - sqwLastEdgeMicros);
    // A pulse arriving implausibly early is treated as a glitch: do not let it
    // advance the edge count or move the accepted-edge timestamp.
    if (periodUs < 900000UL) return;

    uint32_t estimate = sqwPeriodEstimateMicros;
    // Smooth only plausible ~1-second periods over ~32 cycles.
    if (periodUs <= 1100000UL) {
      sqwPeriodEstimateMicros = estimate - estimate / 32UL + periodUs / 32UL;
    } else if (periodUs > 1500000UL && estimate > 0) {
      // If an interrupt was missed, infer the elapsed SQW cycles from the
      // last trusted period estimate so the whole-cycle phase count can recover.
      uint32_t inferredCycles = periodUs / estimate;
      uint32_t remainderUs = periodUs % estimate;
      if (remainderUs >= (estimate / 2UL)) inferredCycles++;
      if (inferredCycles > 1) edgeIncrements = inferredCycles;
    }
    // Normalizing fractional phase by the period estimate largely cancels
    // micros() scale error; implausible intervals never update that estimate.
  }
  sqwEdgeCount += edgeIncrements;
  sqwLastEdgeMicros = now;
  sqwHaveEdge = true;

  // Independent 10-interval diagnostic window. Never pause edge capture:
  // Method B needs a continuous SQW edge count and latest-edge phase.
  if (!sqwDiagStarted) {
    sqwDiagStartMicros = now;
    sqwDiagIntervals = 0;
    sqwDiagStarted = true;
    return;
  }

  sqwDiagIntervals++;
  if (sqwDiagIntervals >= 10) {
    if (!sqwWindowReady) {
      sqwReadyElapsedMicros = now - sqwDiagStartMicros;
      sqwWindowReady = true;
    }
    sqwDiagStartMicros = now;
    sqwDiagIntervals = 0;
  }
}

// SQW phase coordinate in cycles: whole SQW edges plus fractional phase since
// the latest edge. The fractional phase is (elapsed micros ticks)/(micros ticks
// per SQW period). The ESP timer scale multiplies both numerator and denominator,
// so its rate error cancels in this ratio. Whole cycles come from edge counting.
// Therefore Method B avoids stable ESP micros() rate-scale error; do NOT
// add/subtract E from its direct regression result. E is used by Method A.
bool getSqwPhaseCoordinate(double &phaseOut) {
  uint32_t nowUs;
  uint32_t edges;
  uint32_t lastEdgeUs;
  uint32_t periodEstimateUs;
  bool haveEdge;
  noInterrupts();
  nowUs = micros();
  edges = sqwEdgeCount;
  lastEdgeUs = sqwLastEdgeMicros;
  periodEstimateUs = sqwPeriodEstimateMicros;
  haveEdge = sqwHaveEdge;
  interrupts();
  if (!haveEdge || periodEstimateUs == 0) return false;

  uint32_t sinceEdgeUs = (uint32_t)(nowUs - lastEdgeUs);

  // Reject a potentially stale phase once one estimated SQW period has elapsed.
  // Do not clamp an out-of-period phase to an artificial value near 1.0.
  if (sinceEdgeUs >= periodEstimateUs) return false;

  double fractionalPhase = (double)sinceEdgeUs / (double)periodEstimateUs;
  phaseOut = (double)edges + fractionalPhase;
  return true;
}

void readAgingOffset() {
  agingOffsetValid = false;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write((uint8_t)0x10); // DS3231M aging offset register
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom((uint8_t)DS3231_ADDRESS, (uint8_t)1) == 1) {
    agingOffsetValue = (int8_t)Wire.read();
    agingOffsetValid = true;
  }
}

double currentRtcPhaseResidualSigmaSeconds() {
  if (rtcPhaseSamplesAccepted < 3 || rtcPhaseSxx <= 0.0) return NAN;
  double sse = rtcPhaseSyy - (rtcPhaseSxy * rtcPhaseSxy / rtcPhaseSxx);
  if (sse < 0.0) sse = 0.0;
  return sqrt(sse / (double)(rtcPhaseSamplesAccepted - 2));
}

void resetRtcPhaseRegression() {
  rtcPhaseSamplesAccepted = 0;
  rtcPhaseRejectedHighRtt = 0;
  rtcPhaseRejectedNoSqw = 0;
  rtcPhaseRejectedBadTimestamp = 0;
  rtcPhaseRejectedNonFinite = 0;
  rtcPhaseRejectedResidual = 0;
  rtcPhaseBaseRaw = NAN;
  rtcPhaseBaseNtp = NAN;
  rtcPhaseFirstRaw = NAN;
  rtcPhaseLastRaw = NAN;
  rtcPhaseMeanX = 0.0;
  rtcPhaseMeanY = 0.0;
  rtcPhaseSxx = 0.0;
  rtcPhaseSxy = 0.0;
  rtcPhaseSyy = 0.0;
  rtcPhaseSlopeNtpPerSqw = NAN;
  rtcDriftVsNtpPPM = NAN;
  rtcPhaseResidualSigmaUs = NAN;
  rtcMaxPriorFitResidualUs = 0.0;
}

void addRtcNtpPhasePair(double rawSqwPhase, double ntpMidpointEpoch) {
    // Reject invalid input before it can affect the regression.
  if (!isfinite(rawSqwPhase) || !isfinite(ntpMidpointEpoch)) {
    rtcPhaseRejectedNonFinite++;
    return;
  }
  if (rtcPhaseSamplesAccepted == 0) {
    rtcPhaseBaseRaw = rawSqwPhase;
    rtcPhaseBaseNtp = ntpMidpointEpoch;
    rtcPhaseFirstRaw = rawSqwPhase;
  }

  // Use relative coordinates to keep regression values numerically well-scaled.
  double x = rawSqwPhase - rtcPhaseBaseRaw;
  double y = ntpMidpointEpoch - rtcPhaseBaseNtp;

    // Also reject invalid relative coordinates.
  if (!isfinite(x) || !isfinite(y)) {
    rtcPhaseRejectedNonFinite++;
    return;
  }

  // A very early fit can have the wrong slope because NTP path-delay noise is
  // comparable to the accumulated phase drift over only a few minutes. Keep
  // accepting timestamp-valid samples until we have both 30 observations and
  // 10 minutes of NTP time, then enable the residual gate.
  bool residualGateReady =
      rtcPhaseSamplesAccepted >= RTC_PHASE_OUTLIER_MIN_SAMPLES &&
      isfinite(rtcPhaseBaseNtp) &&
      (ntpMidpointEpoch - rtcPhaseBaseNtp) >= RTC_PHASE_OUTLIER_MIN_SPAN_SECONDS;
  if (residualGateReady &&
      isfinite(rtcPhaseSlopeNtpPerSqw) && rtcPhaseSlopeNtpPerSqw > 0.0) {
    double predictedY = rtcPhaseMeanY + rtcPhaseSlopeNtpPerSqw * (x - rtcPhaseMeanX);
    double residual = y - predictedY;
    double absResidual = fabs(residual);
    // Report normal-sized fit residuals. An extreme multi-second jump is a
    // timestamp/server discontinuity, not useful fit-noise telemetry; it is
    // still rejected by the gate below but must not overflow this diagnostic.
    if (isfinite(absResidual) && absResidual <= 5.0 &&
        absResidual * 1000000.0 > rtcMaxPriorFitResidualUs) {
      rtcMaxPriorFitResidualUs = absResidual * 1000000.0;
    }

    double sigmaSec = currentRtcPhaseResidualSigmaSeconds();
    if (!isfinite(sigmaSec) || sigmaSec < 0.0) sigmaSec = 0.0;
    double limitSec = fmax(RTC_PHASE_OUTLIER_FLOOR_SEC,
                           RTC_PHASE_OUTLIER_SIGMA_MULTIPLIER * sigmaSec);
    if (absResidual > limitSec) {
      rtcPhaseRejectedResidual++;
      return;
    }
  }

  uint32_t nextCount = rtcPhaseSamplesAccepted + 1;
  double dx = x - rtcPhaseMeanX;
  double dy = y - rtcPhaseMeanY;
  rtcPhaseMeanX += dx / (double)nextCount;
  rtcPhaseMeanY += dy / (double)nextCount;
  rtcPhaseSxx += dx * (x - rtcPhaseMeanX);
  rtcPhaseSxy += dx * (y - rtcPhaseMeanY);
  rtcPhaseSyy += dy * (y - rtcPhaseMeanY);
  rtcPhaseSamplesAccepted = nextCount;
  rtcPhaseLastRaw = rawSqwPhase;

  // Clear old fit results first so a failed calculation cannot leave
  // an outdated drift value available for display or recommendation.
  rtcPhaseSlopeNtpPerSqw = NAN;
  rtcDriftVsNtpPPM = NAN;
  rtcPhaseResidualSigmaUs = NAN;

  if (rtcPhaseSamplesAccepted >= 3 &&
      isfinite(rtcPhaseSxx) && rtcPhaseSxx > 0.0 &&
      isfinite(rtcPhaseSxy)) {

    double calculatedSlope = rtcPhaseSxy / rtcPhaseSxx;

    if (isfinite(calculatedSlope) && calculatedSlope > 0.0) {

      // Positive ppm means the RTC/SQW is fast relative to NTP.
      double calculatedDriftPPM =
          (1.0 / calculatedSlope - 1.0) * 1000000.0;

      // Do not publish a non-finite drift result.
      if (isfinite(calculatedDriftPPM)) {
        rtcPhaseSlopeNtpPerSqw = calculatedSlope;
        rtcDriftVsNtpPPM = calculatedDriftPPM;

        double sigmaSec = currentRtcPhaseResidualSigmaSeconds();

        if (isfinite(sigmaSec) &&
            sigmaSec >= 0.0 &&
            isfinite(sigmaSec * 1000000.0)) {
          rtcPhaseResidualSigmaUs = sigmaSec * 1000000.0;
        }
      }
    }
  }
}

void clearActiveServerCalibrationState() {
  resetRtcPhaseRegression();
  ntpSamplesAccepted = 0;
  regressionMeanLocal = 0.0;
  regressionMeanNtp = 0.0;
  regressionSxx = 0.0;
  regressionSxy = 0.0;
  firstNtpLocalSeconds = NAN;
  lastNtpLocalSeconds = NAN;
  espTimerDriftPPM = NAN;
  espEstimateValid = false;
  espDriftStabilityReferencePPM = NAN;
  espDriftStabilityReferenceLocalSeconds = NAN;
  espDriftLastCheckpointPPM = NAN;
  espDriftLastCheckpointLocalSeconds = NAN;
  espDriftStable = false;
  sqwIntervalCurrentServerPpmSum = 0.0;
  sqwIntervalCurrentServerWindowCount = 0;
  sqwIntervalCurrentServerFirstWindowMs = 0;
  sqwIntervalCurrentServerHaveFirstWindow = false;
  sqwIntervalSpanStartPpmSum = 0.0;
  sqwIntervalSpanStartWindowCount = 0;
  sqwIntervalSpanEndPpmSum = 0.0;
  sqwIntervalSpanEndWindowCount = 0;
  sqwIntervalSpanStartValid = false;
}

void saveServerCalibrationState(uint8_t index) {
  if (index >= NTP_SERVER_COUNT) return;
  ServerCalibrationState &s = ntpCalibrationState[index];
  s.ntpSamplesAccepted = ntpSamplesAccepted;
  s.regressionMeanLocal = regressionMeanLocal;
  s.regressionMeanNtp = regressionMeanNtp;
  s.regressionSxx = regressionSxx;
  s.regressionSxy = regressionSxy;
  s.firstNtpLocalSeconds = firstNtpLocalSeconds;
  s.lastNtpLocalSeconds = lastNtpLocalSeconds;
  s.espTimerDriftPPM = espTimerDriftPPM;
  s.espEstimateValid = espEstimateValid;
  s.espDriftStabilityReferencePPM = espDriftStabilityReferencePPM;
  s.espDriftStabilityReferenceLocalSeconds = espDriftStabilityReferenceLocalSeconds;
  s.espDriftLastCheckpointPPM = espDriftLastCheckpointPPM;
  s.espDriftLastCheckpointLocalSeconds = espDriftLastCheckpointLocalSeconds;
  s.espDriftStable = espDriftStable;
  s.sqwIntervalCurrentServerPpmSum = sqwIntervalCurrentServerPpmSum;
  s.sqwIntervalCurrentServerWindowCount = sqwIntervalCurrentServerWindowCount;
  s.sqwIntervalCurrentServerFirstWindowMs = sqwIntervalCurrentServerFirstWindowMs;
  s.sqwIntervalCurrentServerHaveFirstWindow = sqwIntervalCurrentServerHaveFirstWindow;
  s.sqwIntervalSpanStartPpmSum = sqwIntervalSpanStartPpmSum;
  s.sqwIntervalSpanStartWindowCount = sqwIntervalSpanStartWindowCount;
  s.sqwIntervalSpanEndPpmSum = sqwIntervalSpanEndPpmSum;
  s.sqwIntervalSpanEndWindowCount = sqwIntervalSpanEndWindowCount;
  s.sqwIntervalSpanStartValid = sqwIntervalSpanStartValid;
  s.rtcPhaseSamplesAccepted = rtcPhaseSamplesAccepted;
  s.rtcPhaseRejectedHighRtt = rtcPhaseRejectedHighRtt;
  s.rtcPhaseRejectedNoSqw = rtcPhaseRejectedNoSqw;
  s.rtcPhaseRejectedBadTimestamp = rtcPhaseRejectedBadTimestamp;
  s.rtcPhaseRejectedNonFinite = rtcPhaseRejectedNonFinite;
  s.rtcPhaseRejectedResidual = rtcPhaseRejectedResidual;
  s.rtcPhaseBaseRaw = rtcPhaseBaseRaw;
  s.rtcPhaseBaseNtp = rtcPhaseBaseNtp;
  s.rtcPhaseFirstRaw = rtcPhaseFirstRaw;
  s.rtcPhaseLastRaw = rtcPhaseLastRaw;
  s.rtcPhaseMeanX = rtcPhaseMeanX;
  s.rtcPhaseMeanY = rtcPhaseMeanY;
  s.rtcPhaseSxx = rtcPhaseSxx;
  s.rtcPhaseSxy = rtcPhaseSxy;
  s.rtcPhaseSyy = rtcPhaseSyy;
  s.rtcPhaseSlopeNtpPerSqw = rtcPhaseSlopeNtpPerSqw;
  s.rtcDriftVsNtpPPM = rtcDriftVsNtpPPM;
  s.rtcPhaseResidualSigmaUs = rtcPhaseResidualSigmaUs;
  s.rtcMaxPriorFitResidualUs = rtcMaxPriorFitResidualUs;
  s.resumeCheckPending = ntpResumeCheckPending;
  ntpCalibrationStateInitialized[index] = true;
}

void restoreServerCalibrationState(uint8_t index) {
  if (index >= NTP_SERVER_COUNT) return;
  if (!ntpCalibrationStateInitialized[index]) {
    clearActiveServerCalibrationState();
    ntpResumeCheckPending = false;
    ntpCalibrationStateInitialized[index] = true;
    return;
  }

  const ServerCalibrationState &s = ntpCalibrationState[index];
  ntpSamplesAccepted = s.ntpSamplesAccepted;
  regressionMeanLocal = s.regressionMeanLocal;
  regressionMeanNtp = s.regressionMeanNtp;
  regressionSxx = s.regressionSxx;
  regressionSxy = s.regressionSxy;
  firstNtpLocalSeconds = s.firstNtpLocalSeconds;
  lastNtpLocalSeconds = s.lastNtpLocalSeconds;
  espTimerDriftPPM = s.espTimerDriftPPM;
  espEstimateValid = s.espEstimateValid;
  espDriftStabilityReferencePPM = s.espDriftStabilityReferencePPM;
  espDriftStabilityReferenceLocalSeconds = s.espDriftStabilityReferenceLocalSeconds;
  espDriftLastCheckpointPPM = s.espDriftLastCheckpointPPM;
  espDriftLastCheckpointLocalSeconds = s.espDriftLastCheckpointLocalSeconds;
  espDriftStable = s.espDriftStable;
  sqwIntervalCurrentServerPpmSum = s.sqwIntervalCurrentServerPpmSum;
  sqwIntervalCurrentServerWindowCount = s.sqwIntervalCurrentServerWindowCount;
  sqwIntervalCurrentServerFirstWindowMs = s.sqwIntervalCurrentServerFirstWindowMs;
  sqwIntervalCurrentServerHaveFirstWindow = s.sqwIntervalCurrentServerHaveFirstWindow;
  sqwIntervalSpanStartPpmSum = s.sqwIntervalSpanStartPpmSum;
  sqwIntervalSpanStartWindowCount = s.sqwIntervalSpanStartWindowCount;
  sqwIntervalSpanEndPpmSum = s.sqwIntervalSpanEndPpmSum;
  sqwIntervalSpanEndWindowCount = s.sqwIntervalSpanEndWindowCount;
  sqwIntervalSpanStartValid = s.sqwIntervalSpanStartValid;
  rtcPhaseSamplesAccepted = s.rtcPhaseSamplesAccepted;
  rtcPhaseRejectedHighRtt = s.rtcPhaseRejectedHighRtt;
  rtcPhaseRejectedNoSqw = s.rtcPhaseRejectedNoSqw;
  rtcPhaseRejectedBadTimestamp = s.rtcPhaseRejectedBadTimestamp;
  rtcPhaseRejectedNonFinite = s.rtcPhaseRejectedNonFinite;
  rtcPhaseRejectedResidual = s.rtcPhaseRejectedResidual;
  rtcPhaseBaseRaw = s.rtcPhaseBaseRaw;
  rtcPhaseBaseNtp = s.rtcPhaseBaseNtp;
  rtcPhaseFirstRaw = s.rtcPhaseFirstRaw;
  rtcPhaseLastRaw = s.rtcPhaseLastRaw;
  rtcPhaseMeanX = s.rtcPhaseMeanX;
  rtcPhaseMeanY = s.rtcPhaseMeanY;
  rtcPhaseSxx = s.rtcPhaseSxx;
  rtcPhaseSxy = s.rtcPhaseSxy;
  rtcPhaseSyy = s.rtcPhaseSyy;
  rtcPhaseSlopeNtpPerSqw = s.rtcPhaseSlopeNtpPerSqw;
  rtcDriftVsNtpPPM = s.rtcDriftVsNtpPPM;
  rtcPhaseResidualSigmaUs = s.rtcPhaseResidualSigmaUs;
  rtcMaxPriorFitResidualUs = s.rtcMaxPriorFitResidualUs;
  ntpResumeCheckPending = s.resumeCheckPending;
}

void switchActiveNtpServer(uint8_t newIndex, bool validateResume = true) {
  if (newIndex >= NTP_SERVER_COUNT || newIndex == ntpServerIndex) return;

  uint8_t oldIndex = ntpServerIndex;
  saveServerCalibrationState(oldIndex);
  bool hadSavedState = ntpCalibrationStateInitialized[newIndex];
  ntpServerIndex = newIndex;
  restoreServerCalibrationState(newIndex);

  // A previous server's fit is retained, but its first returning sample must
  // demonstrate time continuity before being added to that fit. Probe failure
  // returns to the backup quickly and does not invalidate the backup's stability.
  if (validateResume && hadSavedState) ntpResumeCheckPending = true;

  if (hadSavedState) {
    Serial.printf("Restored retained calibration state for server %u: %lu timer samples, %lu phase samples, %lu SQW windows.\n",
                  (unsigned)(newIndex + 1),
                  (unsigned long)ntpSamplesAccepted,
                  (unsigned long)rtcPhaseSamplesAccepted,
                  (unsigned long)sqwIntervalCurrentServerWindowCount);
  } else {
    Serial.printf("Server %u has no saved calibration state; starting its initial collection.\n",
                  (unsigned)(newIndex + 1));
  }
}

void resetEspTimerStabilityForResume() {
  // Keep the server's historical regression, but require a fresh stability
  // period after returning from another server.
  espEstimateValid = false;
  espDriftStable = false;
  espDriftStabilityReferencePPM = NAN;
  espDriftStabilityReferenceLocalSeconds = NAN;
  espDriftLastCheckpointPPM = NAN;
  espDriftLastCheckpointLocalSeconds = NAN;
}

void returnFromPrimaryProbeFailure(const char *reason) {
  uint8_t fallbackIndex = ntpProbeReturnServerIndex;
  ntpPrimaryProbePending = false;
  ntpFailoverPending = false;
  lastPrimaryProbeAttemptMs = millis();
  Serial.printf("Preferred local NTP server probe failed (%s); returning to backup server %u/%u: %s\n",
                reason, (unsigned)(fallbackIndex + 1), (unsigned)NTP_SERVER_COUNT,
                (fallbackIndex < NTP_SERVER_COUNT) ? NTP_SERVERS[fallbackIndex] : "unknown");
  if (fallbackIndex < NTP_SERVER_COUNT && fallbackIndex != ntpServerIndex) {
    ntpServerSwitchCount++;
    // This was only a short probe detour; restore the backup without restarting
    // its stability validation period.
    switchActiveNtpServer(fallbackIndex, false);
  }
}

bool ntpServerIsDue(uint8_t serverIndex, uint32_t nowMs) {
  if (serverIndex >= NTP_SERVER_COUNT) return false;
  if (!ntpServerEverRequested[serverIndex]) return true;
  return (uint32_t)(nowMs - ntpServerLastRequestMs[serverIndex]) >=
         ntpServerLastIntervalMs[serverIndex];
}

const char *ntpIntervalDescription(uint8_t serverIndex) {
  switch (serverIndex) {
    case 0: return "16 seconds";
    case 1: return "64 seconds";
    case 2: return "34 mins 8 secs minimum + 0-120 s jitter";
    default: return "unknown";
  }
}

bool ntpTimestampBytesAreNonzero(const uint8_t *bytes) {
  for (uint8_t i = 0; i < 8; i++) {
    if (bytes[i] != 0) return true;
  }
  return false;
}

// Apply a conservative client-controlled backoff for a verified RATE KoD.
// The server's poll exponent is logged but not trusted as an arbitrary delay.
uint32_t applyVerifiedRateKodBackoff(uint8_t serverIndex) {
  if (serverIndex >= NTP_SERVER_COUNT) return NTP_RATE_BACKOFF_INITIAL_MS;

  uint32_t previous = ntpServerRateBackoffMs[serverIndex];
  uint32_t nextInterval;
  if (previous < NTP_RATE_BACKOFF_INITIAL_MS) {
    nextInterval = NTP_RATE_BACKOFF_INITIAL_MS;
  } else if (previous >= NTP_RATE_BACKOFF_MAX_MS / 2UL) {
    nextInterval = NTP_RATE_BACKOFF_MAX_MS;
  } else {
    nextInterval = previous * 2UL;
    if (nextInterval > NTP_RATE_BACKOFF_MAX_MS) {
      nextInterval = NTP_RATE_BACKOFF_MAX_MS;
    }
  }

  ntpServerRateBackoffMs[serverIndex] = nextInterval;
  ntpServerRateKodCount[serverIndex]++;

  // Start the new backoff from receipt of the KoD rather than from request send.
  ntpServerLastRequestMs[serverIndex] = millis();
  ntpServerEverRequested[serverIndex] = true;

  uint32_t configuredInterval = NTP_SERVER_MIN_INTERVAL_MS[serverIndex];
  if (serverIndex == 2) {
    configuredInterval += (uint32_t)random(0, 121) * 1000UL;
  }
  ntpServerLastIntervalMs[serverIndex] =
      (nextInterval > configuredInterval) ? nextInterval : configuredInterval;

  return ntpServerLastIntervalMs[serverIndex];
}

void reduceNtpRateBackoffAfterValidResponse(uint8_t serverIndex) {
  if (serverIndex >= NTP_SERVER_COUNT || ntpServerRateBackoffMs[serverIndex] == 0) return;

  uint32_t previous = ntpServerRateBackoffMs[serverIndex];
  uint32_t nextInterval = previous / 2UL;
  if (nextInterval < NTP_RATE_BACKOFF_INITIAL_MS) {
    nextInterval = NTP_RATE_BACKOFF_INITIAL_MS;
  }

  ntpServerRateBackoffMs[serverIndex] = nextInterval;

  // Compute the interval for the next request. Keep the server's configured
  // minimum and the Pool's random jitter even while the RATE backoff decays.
  uint32_t configuredInterval = NTP_SERVER_MIN_INTERVAL_MS[serverIndex];
  if (serverIndex == 2) {
    configuredInterval += (uint32_t)random(0, 121) * 1000UL;
  }
  ntpServerLastIntervalMs[serverIndex] =
      (nextInterval > configuredInterval) ? nextInterval : configuredInterval;

  if (nextInterval < previous) {
    Serial.printf("Valid time response received from server %u; RATE backoff reduced one tier from %.0f to %.0f seconds (not below 64 seconds or its configured minimum).\n",
                  (unsigned)(serverIndex + 1),
                  (double)previous / 1000.0,
                  (double)nextInterval / 1000.0);
  } else {
    Serial.printf("Valid time response received from server %u; RATE backoff remains at %.0f seconds.\n",
                  (unsigned)(serverIndex + 1), (double)nextInterval / 1000.0);
  }
}

void advanceNtpServer(const char *reason) {
  uint8_t failedIndex = ntpServerIndex;
  uint8_t nextIndex = (uint8_t)((ntpServerIndex + 1) % NTP_SERVER_COUNT);
  ntpServerSwitchCount++;
  Serial.printf("NTP server %u failed (%s): %s\n",
                (unsigned)(failedIndex + 1), reason, NTP_SERVERS[failedIndex]);
  switchActiveNtpServer(nextIndex, true);
  Serial.printf("Looking for an eligible fallback starting at server %u/%u: %s\n",
                (unsigned)(ntpServerIndex + 1), (unsigned)NTP_SERVER_COUNT,
                NTP_SERVERS[ntpServerIndex]);
  ntpFailoverPending = true;
  // Permit an immediate scan after a failure. A server is still contacted only
  // if its own per-server minimum interval has elapsed.
  lastFailoverScanMs = millis() - NTP_FAILOVER_RESCAN_MS;
}

bool beginNtpRequestToCurrentServer() {
  if (!WiFi.isConnected() || ntpPending) return false;
  memset(ntpPacket, 0, sizeof(ntpPacket));
  ntpPacket[0] = 0x23; // LI=0, VN=4, mode=3 (client)
  ntpPacket[2] = (uint8_t)lround(
    log2((double)ntpServerLastIntervalMs[ntpServerIndex] / 1000.0)
);
  ntpRequestTransmitTimestampValid = false;
  udp.stop();
  if (!udp.begin(0)) return false;
  if (!udp.beginPacket(NTP_SERVERS[ntpServerIndex], NTP_PORT)) {
    udp.stop();
    return false;
  }

  // Supply the RTC's current whole-second timestamp as the request transmit
  // timestamp when the RTC has a plausible modern date. The NTP server echoes
  // this value in its originate-timestamp field; the calibration uses t2/t3,
  // not this request timestamp, for its rate calculations.
  if (rtcAvailable) {
    DateTime requestTime = rtc.now();
    uint32_t requestUnixSeconds = requestTime.unixtime();
    if (requestUnixSeconds > 1600000000UL) {
      uint32_t requestNtpSeconds = requestUnixSeconds + 2208988800UL;
      ntpPacket[40] = (uint8_t)(requestNtpSeconds >> 24);
      ntpPacket[41] = (uint8_t)(requestNtpSeconds >> 16);
      ntpPacket[42] = (uint8_t)(requestNtpSeconds >> 8);
      ntpPacket[43] = (uint8_t)requestNtpSeconds;
      // RTClib supplies whole-second resolution; fraction remains zero.
      ntpRequestTransmitTimestampValid = true;
    }
  }

  udp.write(ntpPacket, sizeof(ntpPacket));

  // Start timing and capture the SQW phase immediately before submitting the
  // packet to the UDP/Wi-Fi stack. This is a software-side approximation of send time.
  ntpRequestStartedMicros = micros();
  ntpRequestStartedMs = millis();
  ntpRequestSqwPhaseValid = getSqwPhaseCoordinate(ntpRequestSqwPhase);

  if (!udp.endPacket()) {
    ntpRequestSqwPhaseValid = false;
    udp.stop();
    return false;
  }
  ntpServerLastRequestMs[ntpServerIndex] = millis();
  ntpServerEverRequested[ntpServerIndex] = true;
  uint32_t intervalMs = NTP_SERVER_MIN_INTERVAL_MS[ntpServerIndex];
  if (ntpServerIndex == 2) {
    // Add random delay beyond the 30-minute minimum to avoid rigid polling.
    intervalMs += (uint32_t)random(0, 121) * 1000UL;
  }
  if (ntpServerRateBackoffMs[ntpServerIndex] > intervalMs) {
    intervalMs = ntpServerRateBackoffMs[ntpServerIndex];
  }
  ntpServerLastIntervalMs[ntpServerIndex] = intervalMs;
  ntpPending = true;
  Serial.printf("NTP reference server %u/%u: %s (configured minimum: %s; scheduled interval: %.0f seconds)\n",
                (unsigned)(ntpServerIndex + 1), (unsigned)NTP_SERVER_COUNT,
                NTP_SERVERS[ntpServerIndex], ntpIntervalDescription(ntpServerIndex),
                (double)intervalMs / 1000.0);
  if (ntpServerRateBackoffMs[ntpServerIndex] > 0) {
    Serial.printf("  RATE backoff retained for this server: %.0f seconds; verified RATE responses: %lu\n",
                  (double)ntpServerRateBackoffMs[ntpServerIndex] / 1000.0,
                  (unsigned long)ntpServerRateKodCount[ntpServerIndex]);
  }
  return true;
}

void beginNtpRequestWithFailover() {
  if (!WiFi.isConnected() || ntpPending) return;

  uint8_t startIndex = ntpServerIndex;
  uint32_t nowMs = millis();
  bool foundEligibleServer = false;

  // Prefer server 1 first whenever it is eligible, even during failover.
  // If it is not due, try the backups beginning with the current failover order.
  // Never bypass any server's own minimum polling interval.
  for (uint8_t offset = 0; offset < NTP_SERVER_COUNT; offset++) {
    uint8_t candidate;
    if (offset == 0) {
      candidate = 0;
    } else {
      uint8_t backupStart = (startIndex == 0) ? 1 : startIndex;
      candidate = (uint8_t)(1 +
          (((backupStart - 1) + (offset - 1)) % (NTP_SERVER_COUNT - 1)));
    }
    if (!ntpServerIsDue(candidate, nowMs)) continue;
    foundEligibleServer = true;

    if (candidate != ntpServerIndex) {
      switchActiveNtpServer(candidate, true);
      ntpServerSwitchCount++;
      Serial.printf("Selecting eligible NTP fallback %u/%u: %s\n",
                    (unsigned)(ntpServerIndex + 1), (unsigned)NTP_SERVER_COUNT,
                    NTP_SERVERS[ntpServerIndex]);
    }

    if (beginNtpRequestToCurrentServer()) return;

    ntpServerFailures[ntpServerIndex]++;
    Serial.printf("NTP request could not be sent to %s; checking next eligible server.\n",
                  NTP_SERVERS[ntpServerIndex]);
  }

  if (!foundEligibleServer) {
    // All servers were queried recently. Recheck slowly until the next one is due.
    lastFailoverScanMs = nowMs;
  }
  ntpFailoverPending = true;
}

void updateExtendedLocalMicrosAt(uint32_t nowMicros) {
  if (!localMicrosInitialized) {
    lastPairMicros = nowMicros;
    localMicrosInitialized = true;
    return;
  }
  // Called regularly from loop(), so unsigned subtraction safely handles the
  // 32-bit micros() rollover even during long NTP outages or server failover.
  extendedLocalMicros += (uint32_t)(nowMicros - lastPairMicros);
  lastPairMicros = nowMicros;
}

void updateExtendedLocalMicros() {
  updateExtendedLocalMicrosAt(micros());
}

void addNtpPair(double localSeconds, double ntpSeconds) {
  // Online least-squares accumulators (Welford-style update) avoid storing samples.
  // All accepted samples from the current NTP server contribute; no persistent storage is used.
  uint32_t nextCount = ntpSamplesAccepted + 1;
  if (ntpSamplesAccepted == 0) {
    firstNtpLocalSeconds = localSeconds;
    // Exclude SQW windows measured before this server's regression starts.
    // The cumulative counters continue across server switches; this per-server
    // snapshot makes the eventual S mean cover the same overall fit interval.
    sqwIntervalSpanStartPpmSum = sqwIntervalPpmSum;
    sqwIntervalSpanStartWindowCount = sqwIntervalWindowCount;
    sqwIntervalSpanStartValid = true;
  }
  // End at the latest accepted NTP observation for this server. This snapshot
  // includes intervening SQW windows even if a different server was active.
  sqwIntervalSpanEndPpmSum = sqwIntervalPpmSum;
  sqwIntervalSpanEndWindowCount = sqwIntervalWindowCount;

  double dx = localSeconds - regressionMeanLocal;
  double dy = ntpSeconds - regressionMeanNtp;
  regressionMeanLocal += dx / (double)nextCount;
  regressionMeanNtp += dy / (double)nextCount;
  regressionSxx += dx * (localSeconds - regressionMeanLocal);
  regressionSxy += dx * (ntpSeconds - regressionMeanNtp);
  ntpSamplesAccepted = nextCount;
  lastNtpLocalSeconds = localSeconds;

  if (ntpSamplesAccepted < MIN_REGRESSION_SAMPLES || regressionSxx <= 0.0) {
    espEstimateValid = false;
    espDriftStable = false;
    espDriftStabilityReferencePPM = NAN;
    espDriftStabilityReferenceLocalSeconds = NAN;
    espDriftLastCheckpointPPM = NAN;
    espDriftLastCheckpointLocalSeconds = NAN;
    return;
  }

  // Calculate ESP timer drift E.
  double ntpPerLocal = regressionSxy / regressionSxx;

  if (!isfinite(ntpPerLocal) || ntpPerLocal <= 0.0) {
    espEstimateValid = false;
    espDriftStable = false;
    espDriftStabilityReferencePPM = NAN;
    espDriftStabilityReferenceLocalSeconds = NAN;
    espDriftLastCheckpointPPM = NAN;
    espDriftLastCheckpointLocalSeconds = NAN;
    return;
  }

  double calculatedDriftPPM =
      (1.0 / ntpPerLocal - 1.0) * 1000000.0;

  if (!isfinite(calculatedDriftPPM)) {
    espEstimateValid = false;
    espDriftStable = false;
    espDriftStabilityReferencePPM = NAN;
    espDriftStabilityReferenceLocalSeconds = NAN;
    espDriftLastCheckpointPPM = NAN;
    espDriftLastCheckpointLocalSeconds = NAN;
    return;
  }

  // Keep the numerical estimate for diagnostics, but do not use it
  // for Method A unless all validation checks pass.
  espTimerDriftPPM = calculatedDriftPPM;
  espEstimateValid = false;

  // Safeguard 1: reject implausible timer-rate estimates and restart validation.
  if (fabs(espTimerDriftPPM) > MAX_ESP_TIMER_DRIFT_PPM) {
    espDriftStable = false;
    espDriftStabilityReferencePPM = NAN;
    espDriftStabilityReferenceLocalSeconds = NAN;
    espDriftLastCheckpointPPM = NAN;
    espDriftLastCheckpointLocalSeconds = NAN;
    return;
  }

  // Safeguard 2: require at least 30 minutes of observations.
  double measurementSpanSeconds =
      lastNtpLocalSeconds - firstNtpLocalSeconds;

  if (!isfinite(measurementSpanSeconds) ||
      measurementSpanSeconds < MIN_ESP_TIMER_SPAN_SECONDS) {
    espDriftStable = false;
    espDriftStabilityReferencePPM = NAN;
    espDriftStabilityReferenceLocalSeconds = NAN;
    espDriftLastCheckpointPPM = NAN;
    espDriftLastCheckpointLocalSeconds = NAN;
    return;
  }

  // The first eligible estimate starts a new stability-validation period.
  if (!isfinite(espDriftStabilityReferencePPM) ||
      !isfinite(espDriftStabilityReferenceLocalSeconds) ||
      !isfinite(espDriftLastCheckpointPPM) ||
      !isfinite(espDriftLastCheckpointLocalSeconds)) {
    espDriftStabilityReferencePPM = espTimerDriftPPM;
    espDriftStabilityReferenceLocalSeconds = lastNtpLocalSeconds;
    espDriftLastCheckpointPPM = espTimerDriftPPM;
    espDriftLastCheckpointLocalSeconds = lastNtpLocalSeconds;
    espDriftStable = false;
    return;
  }

  // Check every fresh NTP regression estimate, not only at 10-minute
  // checkpoints. A tolerance breach invalidates Method A immediately and
  // starts a fresh 10-minute stability period from the current estimate.
  double changeFromStabilityReferencePPM =
      fabs(espTimerDriftPPM - espDriftStabilityReferencePPM);
  double changeFromLastCheckpointPPM =
      fabs(espTimerDriftPPM - espDriftLastCheckpointPPM);

  if (changeFromStabilityReferencePPM > ESP_TIMER_STABILITY_TOLERANCE_PPM ||
      changeFromLastCheckpointPPM > ESP_TIMER_STABILITY_TOLERANCE_PPM) {
    espDriftStable = false;
    espDriftStabilityReferencePPM = espTimerDriftPPM;
    espDriftStabilityReferenceLocalSeconds = lastNtpLocalSeconds;
    espDriftLastCheckpointPPM = espTimerDriftPPM;
    espDriftLastCheckpointLocalSeconds = lastNtpLocalSeconds;
    return;
  }

  // Keep the stability reference fixed between failures to catch cumulative
  // movement. Advance the separate checkpoint only after a full 10-minute
  // interval has remained within tolerance.
  double checkpointSpan =
      lastNtpLocalSeconds - espDriftLastCheckpointLocalSeconds;

  if (checkpointSpan >= ESP_TIMER_STABILITY_CHECK_SECONDS) {
    espDriftStable = true;
    espDriftLastCheckpointPPM = espTimerDriftPPM;
    espDriftLastCheckpointLocalSeconds = lastNtpLocalSeconds;
  }

  // Only validated estimates may be used by Method A.
  espEstimateValid = espDriftStable;
}

void processNtp() {
  if (ntpPending) {
    int packetSize = udp.parsePacket();

    // A short datagram is a received packet, not a timeout. Diagnose it
    // immediately instead of silently waiting until NTP_TIMEOUT_MS expires.
    if (packetSize > 0 && packetSize < 48) {
      IPAddress responseIp = udp.remoteIP();
      uint16_t responsePort = udp.remotePort();
      uint8_t shortResponse[48] = {0};
      int bytesRead = udp.read(shortResponse, sizeof(shortResponse));
      uint32_t rttUs = (uint32_t)(micros() - ntpRequestStartedMicros);
      ntpPending = false;
      udp.stop();
      ntpInvalidResponseCount++;
      ntpServerFailures[ntpServerIndex]++;
      rtcPhaseRejectedBadTimestamp++;
      ntpRequestSqwPhaseValid = false;

      Serial.println(F("NTP RESPONSE REJECTED: UDP datagram shorter than the 48-byte NTP header."));
      Serial.printf("  Requested server %u/%u: %s\n",
                    (unsigned)(ntpServerIndex + 1), (unsigned)NTP_SERVER_COUNT,
                    NTP_SERVERS[ntpServerIndex]);
      Serial.print(F("  Response source: "));
      Serial.print(responseIp);
      Serial.printf(":%u\n", (unsigned)responsePort);
      Serial.printf("  Packet bytes: %d; bytes read: %d; RTT: %.3f ms\n",
                    packetSize, bytesRead, (double)rttUs / 1000.0);
      Serial.println(F("  Failed check: NTP packet length must be at least 48 bytes."));

      if (ntpPrimaryProbePending) {
        returnFromPrimaryProbeFailure("UDP datagram shorter than 48 bytes");
      } else {
        advanceNtpServer("UDP datagram shorter than 48 bytes");
      }
    } else if (packetSize >= 48) {
      // Record local arrival time and capture SQW phase as soon as the response
      // is available, before packet reading and validation add processing latency.
      uint32_t responseReceivedMicros = micros();
      double phaseAtReceive = NAN;
      bool receivePhaseValid = getSqwPhaseCoordinate(phaseAtReceive);
      IPAddress responseIp = udp.remoteIP();
      uint16_t responsePort = udp.remotePort();

      uint8_t response[48] = {0};
      int bytesRead = udp.read(response, sizeof(response));
      uint32_t rttUs = (uint32_t)(responseReceivedMicros - ntpRequestStartedMicros);
      double rttMs = (double)rttUs / 1000.0;
      lastNtpRttMs = rttMs;
      ntpPending = false;
      udp.stop();

      bool haveFullHeader = bytesRead >= 48;
      uint8_t leapIndicator = haveFullHeader ? (uint8_t)(response[0] >> 6) : 255;
      uint8_t ntpVersion = haveFullHeader ? (uint8_t)((response[0] >> 3) & 0x07) : 0;
      uint8_t ntpMode = haveFullHeader ? (uint8_t)(response[0] & 0x07) : 0;
      uint8_t ntpStratum = haveFullHeader ? response[1] : 255;
      bool validLeapIndicator = haveFullHeader && leapIndicator != 3;
      bool validVersion = haveFullHeader && (ntpVersion == 3 || ntpVersion == 4);
      bool validMode = haveFullHeader && ntpMode == 4;
      bool validStratum = haveFullHeader && ntpStratum >= 1 && ntpStratum <= 15;
      bool validNtpHeader = validLeapIndicator && validVersion && validMode && validStratum;
      bool validOriginateTimestamp =
          !ntpRequestTransmitTimestampValid ||
          (haveFullHeader && memcmp(&response[24], &ntpPacket[40], 8) == 0);
      double serverReceiveEpoch = haveFullHeader ? ntpTimestampToUnix(&response[32]) : NAN;
      double serverTransmitEpoch = haveFullHeader ? ntpTimestampToUnix(&response[40]) : NAN;
      bool validReceiveTimestamp = isfinite(serverReceiveEpoch) &&
                                   serverReceiveEpoch > 1600000000.0;
      bool validTransmitTimestamp = isfinite(serverTransmitEpoch) &&
                                    serverTransmitEpoch > 1600000000.0;
      bool orderedTimestamps = validReceiveTimestamp && validTransmitTimestamp &&
                               serverTransmitEpoch >= serverReceiveEpoch;
      bool saneServerProcessingTime = orderedTimestamps &&
                                      (serverTransmitEpoch - serverReceiveEpoch) <= 2.0;
      bool validServerPair = validNtpHeader && validOriginateTimestamp &&
                             validReceiveTimestamp && validTransmitTimestamp &&
                             orderedTimestamps && saneServerProcessingTime;
      bool responseWithinTimeout = rttUs <= (NTP_TIMEOUT_MS * 1000UL);
      bool isRateKodSignature = haveFullHeader && ntpStratum == 0 &&
                                memcmp(&response[12], "RATE", 4) == 0;
      bool validRateKodOrigin = ntpRequestTransmitTimestampValid &&
                                validOriginateTimestamp &&
                                ntpTimestampBytesAreNonzero(&ntpPacket[40]);
      bool validRateKodHeader = haveFullHeader && validVersion && validMode &&
                                ntpStratum == 0;
      bool validRateKod = isRateKodSignature && validRateKodHeader &&
                          validRateKodOrigin && responseWithinTimeout &&
                          responsePort == NTP_PORT;
      int serverPollExponent = (response[2] < 128) ?
                               (int)response[2] : (int)response[2] - 256;
      double serverPollSeconds = NAN;
      if (serverPollExponent >= 0 && serverPollExponent <= 20) {
        serverPollSeconds = (double)(1UL << serverPollExponent);
      } else if (serverPollExponent < 0 && serverPollExponent >= -20) {
        serverPollSeconds = 1.0 / (double)(1UL << (-serverPollExponent));
      }

      // The responder endpoint is diagnostic. This sketch has historically not
      // rejected a packet solely because its UDP source port differs from 123.
      if (responsePort != NTP_PORT) {
        Serial.printf("NTP diagnostic warning: response source port is %u (standard NTP port is %u).\n",
                      (unsigned)responsePort, (unsigned)NTP_PORT);
      }

      // A RATE Kiss-o'-Death is a valid protocol instruction, not a valid time
      // sample. Honor it only when its NTP header and nonzero originate timestamp
      // match the request, and it arrived promptly from the standard NTP port.
      if (validRateKod) {
        uint8_t rateServerIndex = ntpServerIndex;
        ntpServerFailures[rateServerIndex]++;
        uint32_t scheduledBackoffMs = applyVerifiedRateKodBackoff(rateServerIndex);
        ntpRequestSqwPhaseValid = false;

        Serial.println(F("NTP RATE KISS-O'-DEATH ACCEPTED: server requests a lower query rate."));
        Serial.println(F("  KoD accepted only because response mode/version, UDP port, timeout, and nonzero originate timestamp passed validation."));
        Serial.printf("  Server %u/%u: %s; response source: ",
                      (unsigned)(rateServerIndex + 1), (unsigned)NTP_SERVER_COUNT,
                      NTP_SERVERS[rateServerIndex]);
        Serial.print(responseIp);
        Serial.printf(":%u; RTT %.3f ms\n", (unsigned)responsePort, rttMs);
        Serial.printf("  Header: LI=%u VN=%u mode=%u stratum=%u; refid='RATE'\n",
                      (unsigned)leapIndicator, (unsigned)ntpVersion,
                      (unsigned)ntpMode, (unsigned)ntpStratum);
        if (isfinite(serverPollSeconds)) {
          Serial.printf("  Server poll field: exponent %d (%.0f seconds, diagnostic only).\n",
                        serverPollExponent, serverPollSeconds);
        } else {
          Serial.printf("  Server poll field: exponent %d (outside display range; ignored).\n",
                        serverPollExponent);
        }
        Serial.printf("  Client backoff: %.0f seconds; verified RATE count for this server: %lu.\n",
                      (double)scheduledBackoffMs / 1000.0,
                      (unsigned long)ntpServerRateKodCount[rateServerIndex]);
        Serial.printf("  Backoff policy: starts at 64 seconds, doubles on repeated RATE responses, capped at 7200 seconds.\n");
        Serial.println(F("  The server poll field is not applied directly; the client uses bounded backoff."));

        if (ntpPrimaryProbePending) {
          returnFromPrimaryProbeFailure("verified RATE Kiss-o'-Death; honoring server rate limit");
        } else {
          advanceNtpServer("verified RATE Kiss-o'-Death; honoring server rate limit");
        }
      } else if (validServerPair && responseWithinTimeout) {
        // A synchronized timestamp response is evidence the server accepted the
        // current interval. Reduce a previously applied RATE backoff by one tier,
        // never below the conservative 64-second floor for a server that sent RATE.
        reduceNtpRateBackoffAfterValidResponse(ntpServerIndex);

        // Use the timestamp captured when the UDP response first became available;
        // do not add packet-reading and validation latency to Method A's local time.
        // This remains a software-detection time, not the exact Wi-Fi radio arrival.
        updateExtendedLocalMicrosAt(responseReceivedMicros);
        double localSeconds = extendedLocalMicros / 1000000.0;

        // Before a retained server fit is reused, check the returning timestamp
        // against that server's old NTP/local-time regression. A clear clock step
        // invalidates that server's active fit rather than contaminating it.
        if (ntpResumeCheckPending) {
          bool canCheckContinuity = ntpSamplesAccepted >= MIN_REGRESSION_SAMPLES &&
                                    isfinite(regressionSxx) && regressionSxx > 0.0;
          bool continuityPassed = true;
          double continuityResidualSeconds = NAN;
          if (canCheckContinuity) {
            double priorSlope = regressionSxy / regressionSxx;
            double predictedNtp = regressionMeanNtp +
                                 priorSlope * (localSeconds - regressionMeanLocal);
            continuityResidualSeconds = serverTransmitEpoch - predictedNtp;
            continuityPassed = isfinite(priorSlope) && priorSlope > 0.0 &&
                               isfinite(continuityResidualSeconds) &&
                               fabs(continuityResidualSeconds) <=
                                   SERVER_RESUME_MAX_CONTINUITY_RESIDUAL_SECONDS;
          }

          if (!continuityPassed) {
            Serial.printf("WARNING: server %u resumed with %.3f s timestamp residual; discarding its stale active calibration fit and recollecting.\n",
                          (unsigned)(ntpServerIndex + 1), continuityResidualSeconds);
            clearActiveServerCalibrationState();
          } else if (canCheckContinuity) {
            Serial.printf("Server %u resume continuity check passed (residual %+.3f s); retaining its prior regression.\n",
                          (unsigned)(ntpServerIndex + 1), continuityResidualSeconds);
            resetEspTimerStabilityForResume();
          } else {
            Serial.printf("Server %u resumed without enough prior timer samples for a continuity comparison; restarting its stability validation.\n",
                          (unsigned)(ntpServerIndex + 1));
            resetEspTimerStabilityForResume();
          }
          ntpResumeCheckPending = false;
        }

        addNtpPair(localSeconds, serverTransmitEpoch);

        // Method B associates the mean server receive/transmit timestamp with
        // the midpoint of the SQW phase at client request/response. Low RTT is
        // required because network path asymmetry and jitter otherwise add noise.
        if (!ntpRequestSqwPhaseValid || !receivePhaseValid) {
          rtcPhaseRejectedNoSqw++;
        } else if (rttUs > MAX_RTC_PHASE_RTT_US) {
          rtcPhaseRejectedHighRtt++;
        } else {
          double sqwPhaseMidpoint = (ntpRequestSqwPhase + phaseAtReceive) * 0.5;
          double serverTimeMidpoint =
              serverReceiveEpoch + (serverTransmitEpoch - serverReceiveEpoch) * 0.5;
          addRtcNtpPhasePair(sqwPhaseMidpoint, serverTimeMidpoint);
        }

        if (ntpPrimaryProbePending) {
          ntpPrimaryProbePending = false;
          lastPrimaryProbeAttemptMs = millis();
          Serial.println(F("Preferred local NTP server response passed validation; it is active again."));
          Serial.print(F("  Accepted response source: "));
          Serial.print(responseIp);
          Serial.printf(":%u; RTT %.3f ms; LI=%u VN=%u mode=%u stratum=%u\n",
                        (unsigned)responsePort, rttMs, (unsigned)leapIndicator,
                        (unsigned)ntpVersion, (unsigned)ntpMode, (unsigned)ntpStratum);
        }
        ntpFailoverPending = false; // stay with this working server until it fails
      } else {
        ntpInvalidResponseCount++;
        if (!validServerPair) {
          rtcPhaseRejectedBadTimestamp++;
        } else {
          // A valid timestamp pair that arrives after the request timeout is
          // a transport-delay failure, not a malformed timestamp.
          rtcPhaseRejectedHighRtt++;
        }
        ntpServerFailures[ntpServerIndex]++;
        ntpRequestSqwPhaseValid = false;

        // Explain every failed predicate, rather than collapsing all failures
        // into "invalid/late response". This is diagnostic-only: the acceptance
        // thresholds and failover behavior remain unchanged from Version 11.
        Serial.println(F("NTP RESPONSE REJECTED: detailed validation results"));
        Serial.printf("  Requested server %u/%u: %s\n",
                      (unsigned)(ntpServerIndex + 1), (unsigned)NTP_SERVER_COUNT,
                      NTP_SERVERS[ntpServerIndex]);
        Serial.print(F("  Response source: "));
        Serial.print(responseIp);
        Serial.printf(":%u\n", (unsigned)responsePort);
        Serial.printf("  Datagram size: %d bytes; bytes read: %d\n", packetSize, bytesRead);
        Serial.printf("  RTT: %.3f ms; configured request timeout: %lu ms\n",
                      rttMs, (unsigned long)NTP_TIMEOUT_MS);

        if (!haveFullHeader) {
          Serial.println(F("  FAIL: fewer than 48 bytes were read for the NTP header."));
        } else {
          Serial.printf("  Header: LI=%u, VN=%u, mode=%u, stratum=%u\n",
                        (unsigned)leapIndicator, (unsigned)ntpVersion,
                        (unsigned)ntpMode, (unsigned)ntpStratum);
          if (!validLeapIndicator) {
            Serial.println(F("  FAIL: LI=3 means the server reports itself unsynchronized."));
          }
          if (!validVersion) {
            Serial.println(F("  FAIL: NTP version is neither 3 nor 4."));
          }
          if (!validMode) {
            Serial.println(F("  FAIL: response mode is not 4 (server mode)."));
          }
          if (!validStratum) {
            if (ntpStratum == 0) {
              Serial.printf("  FAIL: stratum=0 (Kiss-o'-Death or unsynchronized server); reference ID bytes: %02X %02X %02X %02X ('%c%c%c%c').\n",
                            response[12], response[13], response[14], response[15],
                            (response[12] >= 32 && response[12] <= 126) ? response[12] : '.',
                            (response[13] >= 32 && response[13] <= 126) ? response[13] : '.',
                            (response[14] >= 32 && response[14] <= 126) ? response[14] : '.',
                            (response[15] >= 32 && response[15] <= 126) ? response[15] : '.');
            } else {
              Serial.printf("  FAIL: stratum=%u is outside the accepted range 1..15.\n",
                            (unsigned)ntpStratum);
            }
          }
          if (!validOriginateTimestamp) {
            Serial.println(F("  FAIL: originate timestamp in the response does not match the request transmit timestamp."));
            Serial.printf("    Expected: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                          ntpPacket[40], ntpPacket[41], ntpPacket[42], ntpPacket[43],
                          ntpPacket[44], ntpPacket[45], ntpPacket[46], ntpPacket[47]);
            Serial.printf("    Received: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                          response[24], response[25], response[26], response[27],
                          response[28], response[29], response[30], response[31]);
          }
          if (!validReceiveTimestamp) {
            Serial.printf("  FAIL: NTP receive timestamp t2 is invalid, pre-2020, or undecodable (decoded %.6f).\n",
                          serverReceiveEpoch);
            Serial.printf("    Raw t2: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                          response[32], response[33], response[34], response[35],
                          response[36], response[37], response[38], response[39]);
          }
          if (!validTransmitTimestamp) {
            Serial.printf("  FAIL: NTP transmit timestamp t3 is invalid, pre-2020, or undecodable (decoded %.6f).\n",
                          serverTransmitEpoch);
            Serial.printf("    Raw t3: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                          response[40], response[41], response[42], response[43],
                          response[44], response[45], response[46], response[47]);
          }
          if (validReceiveTimestamp && validTransmitTimestamp && !orderedTimestamps) {
            Serial.printf("  FAIL: t3 is earlier than t2 (t2=%.6f, t3=%.6f).\n",
                          serverReceiveEpoch, serverTransmitEpoch);
          }
          if (orderedTimestamps && !saneServerProcessingTime) {
            Serial.printf("  FAIL: server t3-t2 processing interval is %.6f seconds; maximum accepted is 2.000 seconds.\n",
                          serverTransmitEpoch - serverReceiveEpoch);
          }
        }
        if (!responseWithinTimeout) {
          Serial.println(F("  FAIL: response arrived after the configured request timeout."));
        }
        if (responsePort != NTP_PORT) {
          Serial.printf("  NOTE: responder source port is %u, not standard NTP port %u; port mismatch is logged but is not itself rejecting an ordinary time response.\n",
                        (unsigned)responsePort, (unsigned)NTP_PORT);
        }
        if (isRateKodSignature) {
          Serial.println(F("  NOTE: packet carries stratum=0 / RATE, but it was NOT honored as a rate instruction."));
          if (!validRateKodHeader) {
            Serial.println(F("    Reason: KoD header failed the required version/mode/stratum checks."));
          }
          if (!ntpRequestTransmitTimestampValid) {
            Serial.println(F("    Reason: this request did not contain a valid nonzero client transmit timestamp; KoD origin cannot be authenticated."));
          } else if (!validOriginateTimestamp || !ntpTimestampBytesAreNonzero(&response[24])) {
            Serial.println(F("    Reason: KoD originate timestamp is zero or does not match the request transmit timestamp."));
          }
          if (!responseWithinTimeout) {
            Serial.println(F("    Reason: KoD arrived after the configured request timeout."));
          }
          if (responsePort != NTP_PORT) {
            Serial.println(F("    Reason: KoD response source port is not UDP 123."));
          }
          Serial.println(F("    No RATE backoff was applied because the instruction was not sufficiently validated."));
          if (isfinite(serverPollSeconds)) {
            Serial.printf("    Server poll field was exponent %d (%.0f seconds) and was ignored.\n",
                          serverPollExponent, serverPollSeconds);
          }
        }
        Serial.println(F("  Action: rejected this response and returned to failover/recovery selection."));

        if (ntpPrimaryProbePending) {
          returnFromPrimaryProbeFailure("see detailed NTP response validation above");
        } else {
          advanceNtpServer("see detailed NTP response validation above");
        }
      }
      ntpRequestSqwPhaseValid = false;
    } else if ((uint32_t)(millis() - ntpRequestStartedMs) >= NTP_TIMEOUT_MS) {
      ntpPending = false;
      ntpTimeoutCount++;
      ntpServerFailures[ntpServerIndex]++;
      udp.stop();
      ntpRequestSqwPhaseValid = false;
      Serial.printf("NTP RESPONSE TIMEOUT: no UDP datagram received from server %u/%u (%s) within %lu ms.\n",
                    (unsigned)(ntpServerIndex + 1), (unsigned)NTP_SERVER_COUNT,
                    NTP_SERVERS[ntpServerIndex], (unsigned long)NTP_TIMEOUT_MS);
      if (ntpPrimaryProbePending) {
        returnFromPrimaryProbeFailure("timeout: no UDP datagram received");
      } else {
        advanceNtpServer("timeout: no UDP datagram received");
      }
    }
  }

  uint32_t now = millis();
  if (!ntpPending && WiFi.isConnected()) {
    if (ntpFailoverPending) {
      if ((uint32_t)(now - lastFailoverScanMs) >= NTP_FAILOVER_RESCAN_MS) {
        lastFailoverScanMs = now;
        beginNtpRequestWithFailover();
      }
    } else if (ntpServerIndex != 0 &&
               ntpServerIsDue(0, now) &&
               (uint32_t)(now - lastPrimaryProbeAttemptMs) >=
                   PRIMARY_SERVER_PROBE_INTERVAL_MS) {
      // Server 1 is preferred. Probe it on its allowed 16-second cadence while
      // a backup is active; a failed probe returns to the working backup.
      ntpProbeReturnServerIndex = ntpServerIndex;
      ntpServerSwitchCount++;
      switchActiveNtpServer(0, true);
      ntpPrimaryProbePending = true;
      lastPrimaryProbeAttemptMs = now;
      Serial.printf("Probing preferred local NTP server 1/%u for recovery: %s\n",
                    (unsigned)NTP_SERVER_COUNT, NTP_SERVERS[0]);
      if (!beginNtpRequestToCurrentServer()) {
        ntpServerFailures[0]++;
        returnFromPrimaryProbeFailure("request could not be sent");
      }
    } else if (ntpServerIsDue(ntpServerIndex, now)) {
      // Poll the active server at its own minimum interval.
      ntpFailoverPending = true;
      beginNtpRequestWithFailover();
    }
  }
}

void processSqw() {
  bool ready = false;
  uint32_t elapsedUs = 0;

  noInterrupts();
  if (sqwWindowReady) {
    elapsedUs = sqwReadyElapsedMicros;
    sqwWindowReady = false;
    ready = true;
  }
  interrupts();

  if (!ready || elapsedUs == 0) return;

  // Existing elapsed-time protection.
  if (elapsedUs < SQW_DIAG_MIN_ELAPSED_US ||
      elapsedUs > SQW_DIAG_MAX_ELAPSED_US) {
    sqwIntervalRejectedWindowCount++;
    return;
  }

  double elapsedSeconds = (double)elapsedUs / 1000000.0;

  // Diagnostic only: ESP micros() timing error plus RTC/SQW rate error.
  double samplePpm =
      ((elapsedSeconds - 10.0) / 10.0) * 1000000.0;

  // Protection 1: never allow NaN or infinity into the statistics.
  if (!isfinite(samplePpm)) {
    sqwIntervalRejectedWindowCount++;
    sqwIntervalRejectedNonFiniteCount++;
    return;
  }

  // Protection 2: reject implausibly large diagnostic readings.
  if (fabs(samplePpm) > MAX_SQW_DIAG_PPM) {
    sqwIntervalRejectedWindowCount++;
    sqwIntervalRejectedPlausibilityCount++;
    return;
  }


  // Mark that a valid SQW window has actually been accepted.
  sqwLastAcceptedWindowMs = millis();
  sqwHasAcceptedWindow = true;
  
  // Only accepted samples reach this point.
  sqwIntervalLatestPPM = samplePpm;
  sqwIntervalPpmSum += samplePpm;
  sqwIntervalWindowCount++;

  if (sqwIntervalCurrentServerWindowCount == 0) {
    sqwIntervalCurrentServerFirstWindowMs = millis();
    sqwIntervalCurrentServerHaveFirstWindow = true;
  }
  sqwIntervalCurrentServerPpmSum += samplePpm;
  sqwIntervalCurrentServerWindowCount++;

  if (samplePpm < sqwIntervalMinPPM) {
    sqwIntervalMinPPM = samplePpm;
  }

  if (samplePpm > sqwIntervalMaxPPM) {
    sqwIntervalMaxPPM = samplePpm;
  }

  sqwIntervalErrorMeanPPM =
      sqwIntervalPpmSum / (double)sqwIntervalWindowCount;
}

// Method A estimates RTC/SQW rate using ESP timer calibration E and measured
// SQW interval error S. If E is the micros() rate error and S is the observed
// micros()-seconds-per-SQW-second error, then (1 + RTC rate) = (1 + E)/(1 + S).
// S is averaged over accepted SQW windows between this server's first and latest
// accepted NTP observations, including windows collected during backup-server use.
uint32_t methodASqwSpanWindowCount() {
  if (!sqwIntervalSpanStartValid ||
      sqwIntervalSpanEndWindowCount < sqwIntervalSpanStartWindowCount) {
    return 0;
  }
  return sqwIntervalSpanEndWindowCount - sqwIntervalSpanStartWindowCount;
}

double methodASqwSpanMeanPPM() {
  uint32_t count = methodASqwSpanWindowCount();
  if (count == 0) return NAN;

  double sum = sqwIntervalSpanEndPpmSum - sqwIntervalSpanStartPpmSum;
  if (!isfinite(sum)) return NAN;
  double mean = sum / (double)count;
  return isfinite(mean) ? mean : NAN;
}

double timerCorrectedSqwIntervalDriftPPM() {
  // Require a validated ESP timer estimate and enough accepted SQW windows
  // spanning this server's saved ESP-timer regression.
  uint32_t spanWindowCount = methodASqwSpanWindowCount();
  double meanS = methodASqwSpanMeanPPM();
  if (!espEstimateValid ||
      !isfinite(espTimerDriftPPM) ||
      spanWindowCount < MIN_SQW_CROSSCHECK_WINDOWS ||
      !isfinite(meanS)) {
    return NAN;
  }

  double timerRate = 1.0 + espTimerDriftPPM / 1000000.0;
  double measuredIntervalRate = 1.0 + meanS / 1000000.0;

  // Reject invalid rates and implausible input estimates.
  if (!isfinite(timerRate) ||
      !isfinite(measuredIntervalRate) ||
      timerRate <= 0.0 ||
      measuredIntervalRate <= 0.0 ||
      fabs(espTimerDriftPPM) > 1000.0 ||
      fabs(meanS) > 1000.0) {
    return NAN;
  }

  // E-corrected RTC rate estimate.
  double correctedPpm =
      (timerRate / measuredIntervalRate - 1.0) * 1000000.0;

  if (!isfinite(correctedPpm)) {
    return NAN;
  }

  return correctedPpm;
}

void printReport() {
  double methodADriftPPM = timerCorrectedSqwIntervalDriftPPM();
  uint32_t methodASqwSpanWindows = methodASqwSpanWindowCount();
  double methodASqwSpanMean = methodASqwSpanMeanPPM();

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("ESP TIMER DRIFT"));
  Serial.printf("NTP server (%u/%u)  : %s\n",
                (unsigned)(ntpServerIndex + 1), (unsigned)NTP_SERVER_COUNT,
                NTP_SERVERS[ntpServerIndex]);
  Serial.printf("NTP configured minimum : %s\n", ntpIntervalDescription(ntpServerIndex));
  Serial.printf("NTP scheduled interval : %.0f seconds\n",
                (double)ntpServerLastIntervalMs[ntpServerIndex] / 1000.0);
  Serial.println(F("Server preference : server 1 is primary; backups trigger primary probes when its backoff permits."));
  if (ntpServerIndex != 0) {
    uint32_t reportNowMs = millis();
    uint32_t primaryElapsedMs = ntpServerEverRequested[0]
        ? (uint32_t)(reportNowMs - ntpServerLastRequestMs[0]) : 0UL;
    uint32_t primaryDueRemainingMs = ntpServerLastIntervalMs[0] > primaryElapsedMs
        ? ntpServerLastIntervalMs[0] - primaryElapsedMs : 0UL;
    uint32_t probeElapsedMs = (uint32_t)(reportNowMs - lastPrimaryProbeAttemptMs);
    uint32_t probeRemainingMs = PRIMARY_SERVER_PROBE_INTERVAL_MS > probeElapsedMs
        ? PRIMARY_SERVER_PROBE_INTERVAL_MS - probeElapsedMs : 0UL;
    uint32_t primaryProbeRemainingMs = primaryDueRemainingMs > probeRemainingMs
        ? primaryDueRemainingMs : probeRemainingMs;
    Serial.printf("Next primary probe eligible in: %.1f seconds (current server-1 interval %.0f seconds).\n",
                  (double)primaryProbeRemainingMs / 1000.0,
                  (double)ntpServerLastIntervalMs[0] / 1000.0);
  }
  Serial.printf("NTP observations  : %lu (current server only)\n", (unsigned long)ntpSamplesAccepted);
  Serial.printf("NTP timeouts      : %lu\n", (unsigned long)ntpTimeoutCount);
  Serial.printf("Invalid/late replies: %lu\n", (unsigned long)ntpInvalidResponseCount);
  if (isfinite(lastNtpRttMs)) Serial.printf("Last NTP RTT      : %.3f ms\n", lastNtpRttMs);
  for (uint8_t i = 0; i < NTP_SERVER_COUNT; i++) {
    Serial.printf("Failures [%-20s] : %lu\n",
                  NTP_SERVERS[i],
                  (unsigned long)ntpServerFailures[i]);
    Serial.printf("RATE KoDs [%-20s] : %lu; retained backoff: %.0f seconds\n",
                  NTP_SERVERS[i],
                  (unsigned long)ntpServerRateKodCount[i],
                  (double)ntpServerRateBackoffMs[i] / 1000.0);
  }
  if (ntpSamplesAccepted >= 2 && isfinite(firstNtpLocalSeconds) && isfinite(lastNtpLocalSeconds)) {
    double spanSeconds = lastNtpLocalSeconds - firstNtpLocalSeconds;
    Serial.printf("Running interval    : %.2f hours\n", spanSeconds / 3600.0);
  } else {
    Serial.println(F("Running interval : collecting samples"));
  }
  if (espEstimateValid) {
    Serial.printf("ESP timer drift (E) : %+.5f ppm (validated)\n", espTimerDriftPPM);
    Serial.printf("ESP timer rate      : %+.5f sec/day\n", espTimerDriftPPM * 0.0864);
    if (espTimerDriftPPM > 0.0) Serial.println(F("ESP timer status    : timer is fast"));
    else if (espTimerDriftPPM < 0.0) Serial.println(F("ESP timer status    : timer is slow"));
    else Serial.println(F("ESP timer status    : nominal"));
  } else if (isfinite(espTimerDriftPPM)) {
    Serial.printf("ESP timer drift (E) : %+.5f ppm (not yet validated)\n", espTimerDriftPPM);
  } else {
    Serial.println(F("ESP timer drift (E) : waiting for valid NTP observations"));
  }

  Serial.println();
  Serial.println(F("SQW DRIFT (LIFETIME DIAGNOSTICS)"));
  Serial.printf("Valid windows       : %lu\n", (unsigned long)sqwIntervalWindowCount);
  Serial.printf("Rejected windows    : %lu\n", (unsigned long)sqwIntervalRejectedWindowCount);
  Serial.printf("Rejected non-finite : %lu\n",
                (unsigned long)sqwIntervalRejectedNonFiniteCount);
  Serial.printf("Rejected > +/-100   : %lu\n",
                (unsigned long)sqwIntervalRejectedPlausibilityCount);
  if (sqwIntervalWindowCount > 0 && isfinite(sqwIntervalErrorMeanPPM)) {
    Serial.printf("Min                 : %+.5f ppm\n", sqwIntervalMinPPM);
    Serial.printf("Max                 : %+.5f ppm\n", sqwIntervalMaxPPM);
    Serial.printf("Lifetime average S  : %+.5f ppm\n", sqwIntervalErrorMeanPPM);
    Serial.printf("Latest              : %+.5f ppm\n", sqwIntervalLatestPPM);
  } else {
    Serial.println(F("Waiting for 10 SQW intervals."));
  }
  if (sqwIntervalWindowCount == 0) {
    Serial.println(F("Status              : NOT recording - no valid window accepted yet"));
  } else if (sqwHasAcceptedWindow &&
             (uint32_t)(millis() - sqwLastAcceptedWindowMs) <= SQW_RECORDING_TIMEOUT_MS) {
    Serial.println(F("Status              : Recording lifetime statistics"));
  } else {
    Serial.println(F("Status              : NOT recording - no valid window in last 30 s"));
  }

  // METHOD A is reported first and independently of Method B's readiness.
  Serial.println();
  Serial.println(F("RTC DRIFT (METHOD A): ESP TIMER vs SQW INTERVAL"));
  Serial.println(F("--------------------------------------------------"));
  if (espEstimateValid) {
    Serial.printf("ESP timer drift E          : %+.5f ppm (validated)\n", espTimerDriftPPM);
  } else {
    Serial.println(F("ESP timer drift E          : NOT YET VALIDATED"));
  }
  if (methodASqwSpanWindows > 0 && isfinite(methodASqwSpanMean)) {
    Serial.printf("SQW interval error S       : %+.5f ppm (ESP-fit span)\n",
                  methodASqwSpanMean);
  } else {
    Serial.println(F("SQW interval error S       : waiting for SQW windows within this server's NTP-fit span"));
  }
  Serial.printf("SQW windows in ESP-fit span: %lu/%lu\n",
                (unsigned long)methodASqwSpanWindows,
                (unsigned long)MIN_SQW_CROSSCHECK_WINDOWS);
  Serial.printf("Current-server SQW windows : %lu (diagnostic only)\n",
                (unsigned long)sqwIntervalCurrentServerWindowCount);
  if (isfinite(methodADriftPPM)) {
    Serial.printf("RTC drift (Method A)       : %+.5f ppm\n", methodADriftPPM);
    Serial.printf("RTC rate                   : %+.5f sec/day\n", methodADriftPPM * 0.0864);
  } else {
    if (methodASqwSpanWindows < MIN_SQW_CROSSCHECK_WINDOWS) {
      Serial.println(F("Method A status            : collecting SQW windows across this server's ESP-fit span"));
    }
    if (!espEstimateValid) {
      Serial.println(F("Method A status            : waiting for a validated ESP timer estimate"));
    }
  }

  // METHOD B is the direct NTP timestamp midpoint vs SQW phase regression.
  Serial.println();
  Serial.println(F("RTC DRIFT (METHOD B): NTP TIMESTAMPS vs SQW PHASE"));
  Serial.println(F("--------------------------------------------------"));
  Serial.printf("Accepted samples   : %lu (current NTP server)\n",
                (unsigned long)rtcPhaseSamplesAccepted);
  Serial.printf("Rejected high RTT  : %lu (limit %.0f ms)\n",
                (unsigned long)rtcPhaseRejectedHighRtt,
                (double)MAX_RTC_PHASE_RTT_US / 1000.0);
  Serial.printf("Rejected residual  : %lu\n", (unsigned long)rtcPhaseRejectedResidual);
  Serial.printf("Rejected no SQW    : %lu\n", (unsigned long)rtcPhaseRejectedNoSqw);
  Serial.printf("Rejected invalid response/t2/t3: %lu\n",
                (unsigned long)rtcPhaseRejectedBadTimestamp);
  Serial.printf("Rejected non-finite: %lu\n",
                (unsigned long)rtcPhaseRejectedNonFinite);

  if (rtcPhaseSamplesAccepted >= 3 && isfinite(rtcPhaseSlopeNtpPerSqw) &&
      isfinite(rtcDriftVsNtpPPM) && isfinite(rtcPhaseFirstRaw) && isfinite(rtcPhaseLastRaw)) {
    double phaseSpanSeconds = rtcPhaseLastRaw - rtcPhaseFirstRaw;
    Serial.printf("SQW phase interval : %.0f cycles (%.3f hours)\n",
                  phaseSpanSeconds, phaseSpanSeconds / 3600.0);
    double slopeErrorUsPerInterval = (rtcPhaseSlopeNtpPerSqw - 1.0) * 1000000.0;
    Serial.printf("Regression slope   : %+.6f us/interval\n", slopeErrorUsPerInterval);
    if (isfinite(rtcPhaseResidualSigmaUs)) {
      Serial.printf("Residual sigma     : %.3f us\n", rtcPhaseResidualSigmaUs);
    } else {
      Serial.println(F("Residual sigma     : collecting samples"));
    }
    Serial.printf("Max gated-fit residual: %.3f us\n", rtcMaxPriorFitResidualUs);
    Serial.printf("RTC drift (Method B): %+.5f ppm\n", rtcDriftVsNtpPPM);
    Serial.printf("RTC rate            : %+.5f sec/day\n", rtcDriftVsNtpPPM * 0.0864);
  } else {
    Serial.println(F("Method B status    : collecting at least 3 valid phase samples"));
  }

  if (isfinite(methodADriftPPM) && isfinite(rtcDriftVsNtpPPM)) {
    Serial.printf("Method A - Method B : %+.5f ppm\n", methodADriftPPM - rtcDriftVsNtpPPM);
  } else {
    Serial.println(F("Method A - Method B: comparison unavailable until both estimates are valid"));
  }

  Serial.println();
  Serial.println(F("AGING OFFSET RECOMMENDATION (METHOD A)"));
  Serial.println(F("--------------------------------------------------"));

  double methodAObservationSpanSeconds = 0.0;
  bool methodAObservationSpanValid =
      ntpSamplesAccepted > 0 &&
      isfinite(firstNtpLocalSeconds) &&
      isfinite(lastNtpLocalSeconds);
  if (methodAObservationSpanValid) {
    methodAObservationSpanSeconds = lastNtpLocalSeconds - firstNtpLocalSeconds;
    if (methodAObservationSpanSeconds < 0.0) methodAObservationSpanSeconds = 0.0;
  }

  double methodAMinimumSpanRemainingSeconds =
      fmax(0.0, MIN_ESP_TIMER_SPAN_SECONDS - methodAObservationSpanSeconds);

  double methodAStabilityEpochSpanSeconds = 0.0;
  bool methodAStabilityReferenceValid =
      isfinite(espDriftStabilityReferencePPM) &&
      isfinite(espDriftStabilityReferenceLocalSeconds) &&
      isfinite(lastNtpLocalSeconds);
  if (methodAStabilityReferenceValid) {
    methodAStabilityEpochSpanSeconds =
        lastNtpLocalSeconds - espDriftStabilityReferenceLocalSeconds;
    if (methodAStabilityEpochSpanSeconds < 0.0) {
      methodAStabilityEpochSpanSeconds = 0.0;
    }
  }

  double methodAStabilityCheckpointSpanSeconds = 0.0;
  bool methodAStabilityCheckpointValid =
      isfinite(espDriftLastCheckpointPPM) &&
      isfinite(espDriftLastCheckpointLocalSeconds) &&
      isfinite(lastNtpLocalSeconds);
  if (methodAStabilityCheckpointValid) {
    methodAStabilityCheckpointSpanSeconds =
        lastNtpLocalSeconds - espDriftLastCheckpointLocalSeconds;
    if (methodAStabilityCheckpointSpanSeconds < 0.0) {
      methodAStabilityCheckpointSpanSeconds = 0.0;
    }
  }

  uint32_t methodASqwWindowsRemaining = 0;
  if (methodASqwSpanWindows < MIN_SQW_CROSSCHECK_WINDOWS) {
    methodASqwWindowsRemaining =
        MIN_SQW_CROSSCHECK_WINDOWS - methodASqwSpanWindows;
  }
  double methodASqwApproxRemainingSeconds =
      (double)methodASqwWindowsRemaining * 10.0;

  Serial.printf("ESP timer NTP samples        : %lu (minimum %lu)\n",
                (unsigned long)ntpSamplesAccepted,
                (unsigned long)MIN_REGRESSION_SAMPLES);
  if (methodAObservationSpanValid) {
    Serial.printf("ESP timer observation span   : %.2f / %.2f hours\n",
                  methodAObservationSpanSeconds / 3600.0,
                  MIN_ESP_TIMER_SPAN_SECONDS / 3600.0);
    Serial.printf("Time to minimum span         : %.2f hours (%.1f minutes)\n",
                  methodAMinimumSpanRemainingSeconds / 3600.0,
                  methodAMinimumSpanRemainingSeconds / 60.0);
  } else {
    Serial.println(F("ESP timer observation span  : not started; waiting for first valid NTP sample"));
    Serial.printf("Minimum span after first sample: %.2f minutes\n",
                  MIN_ESP_TIMER_SPAN_SECONDS / 60.0);
  }

  if (espEstimateValid) {
    Serial.println(F("ESP timer stability check   : VALIDATED; monitoring each new NTP estimate"));
  } else if (ntpSamplesAccepted < MIN_REGRESSION_SAMPLES) {
    Serial.printf("ESP timer stability check    : waiting for at least %lu valid NTP samples\n",
                  (unsigned long)MIN_REGRESSION_SAMPLES);
  } else if (!methodAObservationSpanValid ||
             methodAObservationSpanSeconds < MIN_ESP_TIMER_SPAN_SECONDS) {
    Serial.printf("ESP timer stability check   : starts after %.1f-minute observation span; then needs %.1f stable minutes\n",
                  MIN_ESP_TIMER_SPAN_SECONDS / 60.0,
                  ESP_TIMER_STABILITY_CHECK_SECONDS / 60.0);
  } else if (isfinite(espTimerDriftPPM) &&
             fabs(espTimerDriftPPM) > MAX_ESP_TIMER_DRIFT_PPM) {
    Serial.printf("ESP timer stability check   : waiting for plausible estimate within +/- %.1f ppm\n",
                  MAX_ESP_TIMER_DRIFT_PPM);
  } else if (!methodAStabilityReferenceValid ||
             !methodAStabilityCheckpointValid) {
    Serial.printf("ESP timer stability check   : establishing reference; then needs %.1f stable minutes\n",
                  ESP_TIMER_STABILITY_CHECK_SECONDS / 60.0);
  } else if (!espEstimateValid) {
    Serial.println(F("ESP timer stability check   : WAITING for a full stable interval"));
  }

  // Show the monitoring values even after validation, so later drift changes
  // and the next checkpoint countdown remain visible in the report.
  if (methodAStabilityReferenceValid &&
      methodAStabilityCheckpointValid &&
      methodAObservationSpanValid &&
      methodAObservationSpanSeconds >= MIN_ESP_TIMER_SPAN_SECONDS &&
      isfinite(espTimerDriftPPM)) {
    double methodAStabilityRemainingSeconds = fmax(
        0.0, ESP_TIMER_STABILITY_CHECK_SECONDS -
                 methodAStabilityCheckpointSpanSeconds);
    double changeFromStabilityReferencePPM =
        fabs(espTimerDriftPPM - espDriftStabilityReferencePPM);
    double changeFromLastCheckpointPPM =
        fabs(espTimerDriftPPM - espDriftLastCheckpointPPM);

    Serial.printf("Stability-period age          : %.2f hours\n",
                  methodAStabilityEpochSpanSeconds / 3600.0);
    Serial.printf("Drift change from period base : %.3f / %.3f ppm\n",
                  changeFromStabilityReferencePPM,
                  ESP_TIMER_STABILITY_TOLERANCE_PPM);
    Serial.printf("Drift change from last check  : %.3f / %.3f ppm\n",
                  changeFromLastCheckpointPPM,
                  ESP_TIMER_STABILITY_TOLERANCE_PPM);
    Serial.printf("Stability checkpoint          : %.1f / %.1f minutes\n",
                  methodAStabilityCheckpointSpanSeconds / 60.0,
                  ESP_TIMER_STABILITY_CHECK_SECONDS / 60.0);
    Serial.printf("Time until next checkpoint    : %.1f minutes (tolerance +/- %.1f ppm)\n",
                  methodAStabilityRemainingSeconds / 60.0,
                  ESP_TIMER_STABILITY_TOLERANCE_PPM);
    if (!espEstimateValid) {
      Serial.println(F("Method A remains unavailable until a full stability interval passes."));
    }
  }

  Serial.printf("SQW windows in ESP-fit span  : %lu / %lu\n",
                (unsigned long)methodASqwSpanWindows,
                (unsigned long)MIN_SQW_CROSSCHECK_WINDOWS);
  Serial.printf("Current-server SQW windows   : %lu (diagnostic only)\n",
                (unsigned long)sqwIntervalCurrentServerWindowCount);
  if (sqwIntervalCurrentServerHaveFirstWindow) {
    double currentServerSqwSpanMinutes =
        (double)(uint32_t)(millis() - sqwIntervalCurrentServerFirstWindowMs) / 60000.0;
    Serial.printf("Time since first current-server window: %.2f minutes\n",
                  currentServerSqwSpanMinutes);
  } else {
    Serial.println(F("Time since first current-server window: not started"));
  }
  if (methodASqwWindowsRemaining > 0) {
    Serial.printf("Windows still needed         : %lu (about %.0f seconds at one accepted window per 10 s)\n",
                  (unsigned long)methodASqwWindowsRemaining,
                  methodASqwApproxRemainingSeconds);
  } else {
    Serial.println(F("SQW window-count requirement : MET"));
  }

  if (isfinite(methodADriftPPM)) {
    long stepsA = lround(methodADriftPPM / AGING_PPM_PER_STEP);
    Serial.println(F("Method A qualification      : READY"));
    Serial.printf("RTC drift (Method A)        : %+.5f ppm\n", methodADriftPPM);
    Serial.printf("Nominal ds3231m sensitivity : %.3f ppm/step\n", AGING_PPM_PER_STEP);
    Serial.printf("Suggested adjustment        : %+ld steps\n", stepsA);
    if (agingOffsetValid) {
      long proposedA = (long)agingOffsetValue + stepsA;
      if (proposedA < -128) proposedA = -128;
      if (proposedA > 127) proposedA = 127;
      Serial.printf("Current aging register      : %+d\n", (int)agingOffsetValue);
      Serial.printf("Proposed register           : %+ld\n", proposedA);
    } else {
      Serial.println(F("Current aging register      : unavailable"));
    }
  } else {
    Serial.println(F("Method A qualification       : WAITING"));
    if (!espEstimateValid) {
      Serial.println(F("Waiting for a validated ESP timer estimate."));
    }
    if (methodASqwSpanWindows < MIN_SQW_CROSSCHECK_WINDOWS) {
      Serial.println(F("Waiting for the required SQW interval windows across this server's ESP-fit span."));
    }
    if (agingOffsetValid) {
      Serial.printf("Current aging register       : %+d (read only)\n", (int)agingOffsetValue);
    } else {
      Serial.println(F("Current aging register      : unavailable"));
    }
  }

  Serial.println();
  Serial.println(F("AGING OFFSET RECOMMENDATION (METHOD B)"));
  Serial.println(F("--------------------------------------------------"));

  double recommendationSpanSeconds = 0.0;
  if (isfinite(rtcPhaseFirstRaw) && isfinite(rtcPhaseLastRaw)) {
    recommendationSpanSeconds = rtcPhaseLastRaw - rtcPhaseFirstRaw;
    if (recommendationSpanSeconds < 0.0) recommendationSpanSeconds = 0.0;
  }
  uint32_t methodBSamplesRemaining = 0;
  if (rtcPhaseSamplesAccepted < MIN_RTC_RECOMMENDATION_SAMPLES) {
    methodBSamplesRemaining =
        MIN_RTC_RECOMMENDATION_SAMPLES - rtcPhaseSamplesAccepted;
  }
  double methodBSpanRemainingSeconds =
      fmax(0.0, MIN_RTC_RECOMMENDATION_SPAN_SECONDS - recommendationSpanSeconds);
  bool recommendationReady =
      rtcPhaseSamplesAccepted >= MIN_RTC_RECOMMENDATION_SAMPLES &&
      recommendationSpanSeconds >= MIN_RTC_RECOMMENDATION_SPAN_SECONDS &&
      isfinite(rtcDriftVsNtpPPM);

  Serial.printf("Accepted phase samples       : %lu / %lu\n",
                (unsigned long)rtcPhaseSamplesAccepted,
                (unsigned long)MIN_RTC_RECOMMENDATION_SAMPLES);
  if (rtcPhaseSamplesAccepted > 0 &&
      isfinite(rtcPhaseFirstRaw) && isfinite(rtcPhaseLastRaw)) {
    Serial.printf("Phase observation span       : %.2f / %.2f hours\n",
                  recommendationSpanSeconds / 3600.0,
                  MIN_RTC_RECOMMENDATION_SPAN_SECONDS / 3600.0);
    Serial.printf("Time to minimum phase span   : %.2f hours (%.1f minutes)\n",
                  methodBSpanRemainingSeconds / 3600.0,
                  methodBSpanRemainingSeconds / 60.0);
  } else {
    Serial.println(F("Phase observation span       : not started; waiting for first accepted phase pair"));
    Serial.printf("Minimum span after first pair: %.2f hours (%.1f minutes)\n",
                  MIN_RTC_RECOMMENDATION_SPAN_SECONDS / 3600.0,
                  MIN_RTC_RECOMMENDATION_SPAN_SECONDS / 60.0);
  }
  if (methodBSamplesRemaining > 0) {
    Serial.printf("Accepted samples still needed: %lu (current NTP poll floor: %s; rejected responses can extend this)\n",
                  (unsigned long)methodBSamplesRemaining,
                  ntpIntervalDescription(ntpServerIndex));
  } else {
    Serial.println(F("Phase-sample count requirement: MET"));
  }

  if (recommendationReady) {
    long stepsB = lround(rtcDriftVsNtpPPM / AGING_PPM_PER_STEP);
    Serial.println(F("Method B qualification       : READY"));
    Serial.printf("RTC drift (Method B)        : %+.5f ppm\n", rtcDriftVsNtpPPM);
    Serial.printf("Nominal ds3231m sensitivity : %.3f ppm/step\n", AGING_PPM_PER_STEP);
    Serial.printf("Suggested adjustment        : %+ld steps\n", stepsB);
    if (agingOffsetValid) {
      long proposedB = (long)agingOffsetValue + stepsB;
      if (proposedB < -128) proposedB = -128;
      if (proposedB > 127) proposedB = 127;
      Serial.printf("Current aging register      : %+d\n", (int)agingOffsetValue);
      Serial.printf("Proposed register           : %+ld\n", proposedB);
    } else {
      Serial.println(F("Current aging register      : unavailable"));
    }
  } else {
    Serial.println(F("Method B qualification       : WAITING"));
    if (methodBSamplesRemaining > 0) {
      Serial.printf("Waiting for %lu more accepted NTP/SQW phase samples.\n",
                    (unsigned long)methodBSamplesRemaining);
    }
    if (methodBSpanRemainingSeconds > 0.0) {
      if (rtcPhaseSamplesAccepted == 0) {
        Serial.printf("Phase span timer has not started; it requires %.1f minutes between the first and latest accepted phase samples.\n",
                      MIN_RTC_RECOMMENDATION_SPAN_SECONDS / 60.0);
      } else {
        Serial.printf("Waiting for another %.1f minutes of phase span.\n",
                      methodBSpanRemainingSeconds / 60.0);
      }
    }
    if (!isfinite(rtcDriftVsNtpPPM)) {
      Serial.println(F("Waiting for a valid direct-regression drift estimate."));
    }
    if (agingOffsetValid) {
      Serial.printf("Current aging register       : %+d (read only)\n", (int)agingOffsetValue);
    } else {
      Serial.println(F("Current aging register      : unavailable"));
    }
  }
  Serial.println(F("--------------------------------------------------"));

  Serial.println(F("=================================================="));
}
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("RTC calibration monitor " FIRMWARE_VERSION));
  Serial.println(F("Current NTP servers - Polling intervals. Server 1 is HIGHLY recommended to be a local server."));
  for (uint8_t i = 0; i < NTP_SERVER_COUNT; i++) {
    Serial.printf("  %u: %s - %s\n", (unsigned)(i + 1), NTP_SERVERS[i],
                  ntpIntervalDescription(i));
  }
  randomSeed(ESP.getCycleCount());

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  rtcAvailable = rtc.begin();
  if (rtcAvailable) {
    Serial.println(F("DS3231 RTC detected."));
    DateTime now = rtc.now();
    Serial.printf("RTC time: %04d-%02d-%02d %02d:%02d:%02d (UTC)\n",
                  now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    readAgingOffset();
  } else {
    Serial.println(F("WARNING: DS3231 RTC not detected."));
  }

  pinMode(DS3231_SQW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DS3231_SQW_PIN), sqwISR, FALLING);
  Serial.println(F("SQW input enabled on GPIO12; continuous phase capture is active."));

  WiFi.mode(WIFI_STA);
  // WiFiManager reconnects to saved credentials or opens its setup portal.
  // No custom hostname is set.
  // FOR DEBUGGING ONLY. Uncomment ONLY if you want to erase saved credentials. You must reflash a 2nd time for normal operation.
  //wifiManager.resetSettings();
  
  bool connected = wifiManager.autoConnect("RTC-Cal-Setup");
  if (connected) {
    Serial.print(F("Wi-Fi connected.\n"));
    //Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("Wi-Fi setup portal did not connect; will retry in loop."));
  }

  lastReportMs = millis();
  Serial.println(F("SETUP COMPLETE"));
}
void loop() {
  // Extend micros() continuously, not only on NTP replies. This keeps each
  // server's stored regression time axis valid across long outages and rollover.
  updateExtendedLocalMicros();

  if (!WiFi.isConnected() && (uint32_t)(millis() - lastWifiRetryMs) >= WIFI_RETRY_INTERVAL_MS) {
    lastWifiRetryMs = millis();
    WiFi.reconnect();
  }

  processNtp();
  processSqw();

  if ((uint32_t)(millis() - lastReportMs) >= MEASURE_INTERVAL_MS) {
    lastReportMs = millis();
    printReport();
    if (rtcAvailable) {
      DateTime now = rtc.now();
      Serial.printf("RTC read: %04d-%02d-%02d %02d:%02d:%02d\n",
                    now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    }
  }
  yield();
}
