# MatrixClock-RTC-calibration

A practical calibration for the DS3231M real-time clock used in MatrixClock devices.

Measure RTC frequency error and calculate aging-register corrections using two independent reference methods:

  - NTP-based calibration using the ESP8266/ESP-12S that normally comes with them.

  - GPS-based calibration with PPS (precision pulse-per-second) reference.

Includes phase analysis, statistical regression, measurement validation, and advisory aging-register recommendations.

Both methods suffer from interrupt latency, interrupt jitter, GPS receiver PPS characteristics, electrical signal quality, RTC temperature changes, etc.

No automatic RTC calibration writes are performed.

This repository contains two sketches to help you calibrate the RTC on your MatrixClock. The sketches themselves DO NOT set the aging register. They will give you a suggestion on what to set it to (in the serial console only). A basic Aging Register writer is included here, if you need something more advanced/more informative, you need to look for it on the net. Anyway, it appears that there are several clocks being produced by several companies, but that should not matter. Some of the clocks have an ESP-12S (ESP8266) and a DS3231M in them (please verify the DS3231 version). In fact, this code can be applied to any project that contains a DS3231M and an ESP8266 (or any RTC ICs that have similar characteristics such as aging register features). WARNING: THIS WILL ERASE YOUR FACTORY FIRMWARE. YOU WILL NEED A COPY OF YOUR FIRMWARE IF YOU WANT YOUR CLOCK TO WORK AGAIN. There are other firmware options out there with nicer features. I have my own version that hopefully will release some day when it's fully done. Check out Steve Madden improved matrixclock version in the meantime.

<img width="4032" height="2268" alt="2026-10-29-06-00-35-563" src="https://github.com/user-attachments/assets/014166c5-af0a-4a28-a684-59f52253cd77" />


If you are looking at this repository, you most likely know what you are doing. Therefore, loading firmware using an IDE or identifying your GPIO pins on your device/setup is beyond the scope of this repository. Also, setting up your GPS module is not covered here, and neither setting up a local ntp server (not that difficult). Firmware uploads are done like any other esp8266. Pay close attention to your serial SDA/SCL signal pins. I have several devices and 2 of them have different assigned serial pin mappings. 

You might say, but I have a frequency counter, oscilloscope, etc.; I don’t need this. You are correct. These sketches are meant to be used for when you don’t have expensive lab equipment to calibrate these clocks or so that you incorporate this code into your self-correcting firmware. Although, with the GPS
sketch, you could make the ultimate clock by disciplining the RTC permanently/constantly (instead of using the NTP sync). In my case, I have a device that will run unattended with no WiFi for a long time, and I need to squeeze the best accuracy without breaking the bank on lab equipment (and I don't want to permanently add a GPS module, antenna, yada yada). I would love to see how these sketches perform against lab equipment. I don't currently have these devices or access to them.

The following approaches need you to connect the SQW pin from your DS3231M to your GPIO 12 on your ESP8266.  You must use a pull-up resistor for this implementation. For the first approach, it is HIGHLY recommended to use a local NTP server. You must specify your local IP address (or your ntp.local name if your network resolves it). Running the sketch with public internet-based NTP servers is less accurate. For the second approach, we will be using a GPS PPS signal as a frequency signal generator, in essence. These GPS modules cost as little as $10 US dollars; of course you can also use other inexpensive signal generators such as the transistor testers with signal generation, etc. (I got a GM328 tester that seems OK for this, not covered here though). The PPS signal will be connected to GPIO2 (this is a bootstrap pin, see instructions on this later). 

<img width="4032" height="2268" alt="2026-10-29-05-55-13-247 tmp" src="https://github.com/user-attachments/assets/c1477280-4b9c-448a-ac51-e651ed88b4da" />

Nevermind the temperature sensors (DHT11s), The one at the bottom shows both connections (SQW and GPS PPS, GPIO 12 AND GPIO 2 respectively, yeah no other pins available here). The one at the bottom I got it pretty beat up by now :)

<img width="1002" height="442" alt="Screenshot From 2026-10-29 06-09-59" src="https://github.com/user-attachments/assets/a0ab21fc-b634-492a-8029-af6c0b44c0be" />

A 10KOhms resistor will do for your pull-up.

<img width="653" height="271" alt="Screenshot From 2026-10-30 02-32-48" src="https://github.com/user-attachments/assets/65ee6e3f-8b20-46d5-8437-a4ddd3d38280" />

This is the default interval NTP query/sync (16 secs). If you want to run this default, I would advice you to disable the Kiss-o'-death (KOD) feature in your local server; otherwise, you will be kissed :). Luckily for you, I have incorporated a backoff timing for your queries. The firmware will query at the default until it receives a RATE signal, then it will adjust itself to a higher query timing (64 secs) until it satisfies the server's demands (hmm... this sounds like a girlfriend :) ). Anyway, like everything in life, there is a limit, the max backoff time is 2 hours.


APPROACH I

This approach contains two methods. After limited testing, both methods seem to agree most of the time and are often within +/- 1 steps of each other. Method A seems to arrive at the suggested aging register faster. The recommendation is to run the sketch for a minimum of 24 hours; longer would be better. The first results will be presented at around 40 minutes after start up (if your setup is stable enough, otherwise it will take longer).
  
SQW measurement: Measures the DS3231M's 1 Hz square-wave output connected to ESP8266 GPIO12.

NTP reference: Uses NTP timestamps to compare the RTC's behavior against network time. It does not set the ESP8266 system clock or the RTC's time.

Method A — ESP TIMER vs SQW INTERVAL: Estimates ESP8266 timer-rate error and compares it with measured SQW interval error to estimate the RTC's frequency drift.

Method B — NTP TIMESTAMPS vs SQW PHASE: Uses linear regression of NTP timestamps against SQW phase measurements to independently estimate RTC drift.

NTP failover: Prefers the configured local NTP server and can fall back to backup servers when the preferred server is unavailable.

Calibration recommendations: Calculates suggested DS3231M aging-register adjustments without writing to the register. The adjustment is a recommendation, not an automatic change.

Data validation: Rejects unsuitable NTP responses and applies sample-count, observation-duration, and stability requirements before issuing recommendations.

Runtime data: Keeps calibration statistics in RAM, so accumulated results are lost when the ESP8266 reboots.

Approach I Configurable Variables. Default settings are recommended. 

DS3231_SQW_PIN                 // SQW input pin

I2C_SDA                        // I2C data pin

I2C_SCL                        // I2C clock pin

MEASURE_INTERVAL_MS            // Report/poll interval

WIFI_RETRY_INTERVAL_MS         // Wi-Fi retry interval

NTP_TIMEOUT_MS                 // NTP request timeout

NTP_PORT                       // NTP port

NTP_FAILOVER_RESCAN_MS         // Failover rescan interval

PRIMARY_SERVER_PROBE_INTERVAL_MS // Primary server probe interval

NTP_RATE_BACKOFF_INITIAL_MS    // Initial RATE backoff

NTP_RATE_BACKOFF_MAX_MS        // Maximum RATE backoff

SERVER_RESUME_MAX_CONTINUITY_RESIDUAL_SECONDS // Reject server clock jumps

NTP_SERVER_COUNT               // Number of NTP servers

NTP_SERVERS                    // NTP server addresses

NTP_SERVER_MIN_INTERVAL_MS     // Minimum interval per server

AGING_PPM_PER_STEP             // PPM per aging-register step

MIN_REGRESSION_SAMPLES         // Minimum timer regression samples

MAX_ESP_TIMER_DRIFT_PPM        // Maximum accepted timer drift

MIN_ESP_TIMER_SPAN_SECONDS     // Minimum timer measurement span

ESP_TIMER_STABILITY_TOLERANCE_PPM // Timer stability threshold

ESP_TIMER_STABILITY_CHECK_SECONDS // Stability confirmation period

RTC_PHASE_OUTLIER_MIN_SAMPLES  // Samples before outlier rejection

RTC_PHASE_OUTLIER_MIN_SPAN_SECONDS // Span before outlier rejection

MIN_RTC_RECOMMENDATION_SAMPLES // Minimum Method B samples

MIN_RTC_RECOMMENDATION_SPAN_SECONDS // Minimum Method B time span

MAX_RTC_PHASE_RTT_US           // Maximum accepted NTP round-trip time

RTC_PHASE_OUTLIER_FLOOR_SEC    // Minimum residual threshold

RTC_PHASE_OUTLIER_SIGMA_MULTIPLIER // Residual threshold multiplier

SQW_DIAG_MIN_ELAPSED_US        // Minimum valid SQW diagnostic duration

SQW_DIAG_MAX_ELAPSED_US        // Maximum valid SQW diagnostic duration

SQW_RECORDING_TIMEOUT_MS       // SQW recording status timeout

MIN_SQW_CROSSCHECK_WINDOWS     // Minimum SQW cross-check windows

MAX_SQW_DIAG_PPM               // Maximum accepted SQW diagnostic drift

<img width="599" height="675" alt="Screenshot From 2026-10-30 03-28-50" src="https://github.com/user-attachments/assets/988229bf-86bb-4d93-9c58-88ba79588756" />

1-hour run. 


APPROACH II

The GPS PPS RTC Calibration Sketch is firmware designed to evaluate the timekeeping accuracy of a DS3231M RTC using a GPS receiver's precise 1 pulse-per-second (PPS) signal as a timing reference. The PPS signal will be connected to GPIO2. Because this is a special pin, you need to wait for the firmware to tell you that the setup is complete; only after that you make your connection to your PPS signal. This is due to the fact that the GPIO2 must be HIGH during boot. The signal from your GPS module alternates between HIGH and LOW, and if it is LOW, then your device will go into a firmware download status. This approach produces close results to the first approach, it is often within  +/- 2 steps. However, this is the approach to trust. Additional information available in the sketch.

GPS PPS measurement: Uses the GPS receiver's 1 Hz PPS signal as a precision timing reference.

RTC comparison: Measures timing differences between the GPS PPS (1 Hz) signal and the DS3231M SQW (1 Hz) signal to estimate the RTC's frequency error.

Frequency calibration: Analyzes accumulated timing measurements to determine whether the RTC runs fast or slow relative to the GPS reference.

Aging-register recommendations: Calculates a suggested DS3231M aging-register adjustment to compensate for measured frequency drift.

Diagnostic reporting: Reports measurement results and calibration information to help evaluate RTC performance.

Information is retained in RAM only.


Approach II Configurable Variables. Default settings are recommended.

RTC_I2C_ADDRESS                   // RTC I2C address

RTC_CONTROL_REGISTER              // RTC control register

RTC_AGING_REGISTER                // RTC aging register

GPS_PPS_PIN                       // GPS PPS input pin

RTC_SQW_PIN                       // RTC SQW input pin

I2C_SDA_PIN                       // I2C data pin

I2C_SCL_PIN                       // I2C clock pin

LED_MATRIX_DIN_PIN                // LED matrix data pin

LED_MATRIX_CLK_PIN                // LED matrix clock pin

LED_MATRIX_CS_PIN                 // LED matrix chip select

LED_MATRIX_DEVICES                // Number of LED matrix devices

INTERVALS                         // Measurement intervals

SAMPLE_COUNT                      // Phase samples per window

MIN_GAP_US                        // Minimum valid edge interval

MAX_GAP_US                        // Maximum valid edge interval

MIN_PERIOD_US                     // Minimum plausible GPS period

MAX_PERIOD_US                     // Maximum plausible GPS period

MAX_DRIFT_PPM                     // Maximum accepted drift

PHASE_WRAP_US                     // Phase wrap period

PHASE_HALF_WRAP_US                // Half phase wrap period

MAX_PHASE_STEP_US                 // Maximum phase step

MAX_PHASE_RESIDUAL_SIGMA_US        // Maximum residual standard deviation

MAX_ABS_PHASE_RESIDUAL_US          // Maximum absolute phase residual

AGING_PPM_PER_STEP                // Estimated PPM per aging-register step

STATUS_INTERVAL_MS                // Status report interval

MIN_WINDOWS_FOR_RECOMMENDATION    // Minimum accepted windows for recommendation

<img width="719" height="644" alt="Screenshot From 2026-10-30 02-24-35" src="https://github.com/user-attachments/assets/597d0d04-35d4-4d4c-bec6-4ef780a28ced" />

1-hour run.

KNOWN ISSUES

There is a potential issue where the satiny check (GPS PPS sketch) and the regression slope do not agree closely.  A 10% discrepancy is OK, anything greater than that is cause for concern. The sanity check should be:

Sanity check ≈ regression slope X number of intervals  +/- 10%

Currently working on the issue.

CREDITS and LICENSES

The sketches have been put through an AI engine for code debugging and optimization. Firmware is distributed under GNU GPL v3, without warranty. Libraries retain their original licenses. Installing firmware is at your own risk.
