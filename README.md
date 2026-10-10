# MP65 6-DoF attitude application

This is the main firmware application, integrating the reusable
components. The application-specific code assembles a sensor driver, an
attitude policy, a Kalman estimator, a diagnostic logger, and an optional USB
stream. The boundaries are deliberately the following.

Build and run commands below are executed from this repository root. Firmware
sources are in `main/`, application Octave scripts in `tools/octave/`, and
independent component repositories in `components/`. `CMakeLists.txt` explicitly
disables Component Manager for this local integration; all five checked-out
components must be present. Their own manifests remain usable by other consumers.

The former top-level examples and their generated builds are preserved locally
in `.legacy-examples/` as a backup, excluded from version control. Standalone
sensor and benchmark examples now live in `components/esp_mp65/examples/read_sensor`
and `components/kalman/examples/esp32_benchmark` respectively. Do not reuse an old
build directory after moving the application: configure a fresh root `build/`.

| Layer | Reusable component / code | Responsibility in this application |
|---|---|---|
| Sensor and synchronization | [`esp_mp65`](components/esp_mp65/README.md) | MPU-6500 SPI transactions, register setup, optional managed INT GPIO and one-slot release events. It does not choose the loop policy or interpret attitude. |
| Attitude policy | [`imu_attitude`](components/imu_attitude/README.md) | Warmup, stationary calibration, gyro-bias initialization, acceleration gating/jerk weighting, gap handling, and sequencing of estimator calls. |
| Mathematics | [`kalman_attitude`](components/kalman_attitude/README.md) and [`kalman`](components/kalman/README.md) | Quaternion/gyro prediction and accelerometer correction. They do not read the sensor, schedule tasks, log, or select application thresholds. |
| Generic real-time stream | [`esp_stream`](components/esp_stream/README.md) | Binary framing, metadata, CRC, USB transport, and generic Octave receiver/session helpers. It does not know that the five channels are a quaternion. |
| Application integration | [`main.c`](main/main.c), [`attitude_stream.c`](main/attitude_stream.c) | Core affinity, INT wait/read loop, stale-event policy, application diagnostics, quaternion quantization, channel names, and lifecycle. |
| Application visualization | [`mp65_cube.m`](tools/octave/mp65_cube.m) | MP65 channel interpretation, quaternion normalization, tilt/full-orientation modes, reference handling, and cube rendering. |

### Reporter boundary

The external [`esp_rt_diagnostics`](https://github.com/smartsensingme/esp_rt_diagnostics)
and [`esp_rt_diagnostics_reporter`](https://github.com/smartsensingme/esp_rt_diagnostics_reporter)
components are **not currently linked by this application**. The functions named
`diagnostic_logger()`, `publish_if_due()`, and the type
`diagnostic_report_t` in `main.c` are application code, not the generic
reporter. They currently copy and print application-owned diagnostic records
through a FreeRTOS queue.

The current [`main/CMakeLists.txt`](main/CMakeLists.txt) confirms this
boundary: it links `esp_mp65`, `imu_attitude`, `kalman_attitude`, `esp_timer`,
`esp_driver_gpio`, and `esp_stream`; it does not link either `esp_rt` component.

If those components are added later, `esp_rt_diagnostics` should replace only
the generic timing accumulation, while `esp_rt_diagnostics_reporter` should
replace or sit beside the Core 0 presentation queue. Bias, jerk, innovation,
saturation, calibration, and attitude values remain an application payload.
The migration plan and the exact field mapping are recorded in
[`AGENTS-APP.md`](AGENTS-APP.md); this README describes the current firmware,
not a claimed future integration.

The default 1 kHz profile is unchanged; hardware timing needs revalidation
after changes to the integration. Octave scripts, wiring, and the `esp_stream`
schema are application-facing integrations over the generic transport.

### Data-flow overview

```text
MPU-6500 INT
    │  esp_mp65 ISR: timestamp + overwrite newest release
    ▼
Core 1: main.c
    │  wait event → esp_mp65_read() → imu_attitude_update()
    │  application policy: gaps, stale events, saturation, reports
    ├──────────────► Core 0: diagnostic_logger() [current app logger]
    │                         └─ ESP_LOG output
    └──────────────► attitude_stream.c [current app adapter]
                              └─ esp_stream generic binary USB transport
                                         └─ mp65_cube.m application viewer
```

The current logger and the current stream adapter are therefore two different
application consumers of the same acquisition result. Neither is part of
`esp_mp65`, `imu_attitude`, `kalman_attitude`, or the generic `esp_stream`
protocol implementation.

## Live Octave cube

`CONFIG_MP65_ATTITUDE_STREAM` enables continuous native USB Serial/JTAG output.
Logs stay on UART; disable the secondary USB console as well. Do not run the
buffered capture transport or an IDF monitor on the same USB port.

In an Octave GUI session, with instrument-control installed:

```octave
addpath('/path/to/esp_mp65/components/esp_stream/tools/octave');
addpath('/path/to/esp_mp65/tools/octave');
pkg load instrument-control;
mp65_cube() % list ports, choose a number, then open the cube
```

The text menu lists serial ports without opening them. On macOS it prefers
the matching `/dev/cu.*` interface when available, avoiding duplicate tty/cu
entries. Choose a number and press Enter; R refreshes, 0 cancels. An explicit
port argument remains supported for scripts.

During reception, the terminal menu uses nonblocking single-key input: 1 selects
tilt-only, 2 full orientation, 3 visual reference, 4 reception statistics,
5 menu, and 0 exit. No Enter is needed. Focus the terminal to use these keys.
If the Octave GUI command pane does not deliver live keystrokes, launch
`octave --no-gui` from a desktop terminal (graphics remain enabled), or use the
same numeric shortcuts in the figure. Do not use `--no-window-system`.
Rendering is limited to 25 Hz, independently of receiver polling.

Close the figure or press 0/Q to release the port. T toggles tilt-only (default) and full
orientation. Space establishes a visual reference without changing the filter.
Switching modes clears that reference. The cube rotates, not translates.
Quaternions are right-handed body-to-world, ordered w,x,y,z. Tilt-only removes
Euler heading and retains the usual ambiguity at vertical pitch.

Acquisition remains INT-driven at nominal 1 kHz with a 1 ms deadline. Every
40 ms it quantizes/copies an orientation into a one-slot overwrite queue; Core 0 encodes
and writes USB with a 10 ms timeout. No USB wait or allocation occurs in the
acquisition loop. Publication overhead is included in cycle timing. Old queued
frames are replaced, not retried. USB/host buffering can add latency; no
sensor-to-screen deadline is guaranteed. The GUI marks data stale after 0.5 s
without a valid record. Hardware timing with streaming needs validation.

The reusable [esp_stream](components/esp_stream/README.md) transports five
int16 channels: qw, qx, qy, qz (scale 1/32767, offset 0), and ready (scale 1).
The application rounds/clamps quaternion components to [-32767,32767]; maximum
rounding error is approximately 1.53e-5 per component before host normalization.
Ready is 0 during calibration and 1 after alignment. The Octave adapter locates
these channels by name, applies received scaling and normalizes the quaternion.
Timestamps still represent data-ready ISR time, not hardware conversion time.
The application publishes a frame only after a new sensor sample has been read
and the attitude update succeeds; a read or numerical-update failure therefore
does not reuse the previous quaternion with a newer timestamp. The optional
Octave session layer tracks transport freshness, while this application still
decides whether a received quaternion is valid for rendering.

The original ATT1 ASCII protocol and its decoder were replaced by EST1 binary
framing with CRC-32 and metadata. Update firmware and Octave together. Generic
USB/decoding belongs to esp_stream; quaternion quantization, calibration status,
rotation math and the cube remain in this application. No capture package was
modified. Migration to esp_rt_diagnostics and its reporter remains pending.

Run `octave --no-gui --quiet tools/octave/test_attitude.m` from the repository
root for offline tests. USB and graphical behavior need end-to-end validation.

## Current diagnostics versus optional reporter components

The diagnostic output described below is produced by the application as it is
currently written:

```text
Core 1 main.c
  diagnostic_report_t counters/state
       │ publish_if_due() copies without waiting
       ▼
  FreeRTOS queue (two items)
       ▼
Core 0 diagnostic_logger()
       └─ ESP_LOGI/ESP_LOGW text
```

This path is intentionally visible in the source. `diagnostic_report_t` is not
an `esp_rt_diag_snapshot_t`, `diagnostic_logger()` is not an
`esp_rt_diagnostics_reporter` task, and the queue is not the reporter's
one-slot overwrite queue. The current application also emits separate
calibration records (`kind=1`/`kind=2`) and a calibration-progress record
(`kind=3`).

The optional generic path would be:

```text
Core 1 main.c
  esp_rt_diag_cycle/stage/event/interval calls
       │ close cycle, take immutable snapshot
       ▼
  esp_rt_diag_reporter_publish(snapshot, application_payload)
       ▼
Core 0 reporter task
       └─ generic report + application callback/payload
```

Only the second path uses the two external reporter components. The first path
can remain as a reference during migration. The reporter is useful for bounded,
fresh presentation; it is not a lossless replacement for a recorder and does
not automatically carry the application fields listed below.

## Accuracy and next refinements

Roll/pitch stability at rest does not establish dynamic accuracy. Next steps:
known-angle tests, repeatable slow/fast return trajectories, gyro-only versus
fused quaternion comparison, signed corrected gyro means and temperature, and
evaluation of correction rejection/recovery. Body-frame jerk also responds to
rotating gravity; it is not a pure translation detector. Yaw is relative and
drifts without an external heading reference, but large return errors still
need investigation rather than being attributed solely to 6-DoF.

This ESP-IDF application uses the MPU-6500's INT data-ready signal on GPIO4 to read
both inertial sensors at their common filtered 1 kHz output rate, performs a
stationary startup calibration, runs the portable
`kalman_attitude` ESKF, and prints roll, pitch, and relative yaw every two
seconds.

Keep the board stationary for approximately 2.5 seconds during startup. The
application discards the first 0.5 seconds (500 samples), averages the following
2000 samples over two seconds, and
checks accelerometer/gyroscope dispersion before accepting calibration. Motion
causes the window to be discarded and restarted. The means initialize tilt and
all three gyroscope biases, while the measured variance of each mean initializes
the bias covariance with a 0.05 deg/s uncertainty floor. Normal accelerometer
corrections cannot subsequently modify those calibrated biases, preventing
dynamic acceleration from reintroducing the large yaw drift seen previously.

Roll and pitch remain corrected by gravity. Yaw has an arbitrary zero at
startup and can still drift from residual or temperature-dependent gyro bias
because this 6-DoF sensor provides no absolute heading reference. Acceleration
samples outside the gravity-magnitude or innovation gate are omitted from the
correction step. The application measures windowed acceleration jerk: it averages
20 ms of samples and differentiates rolling means 10 ms apart. A moderate jerk
increases measurement variance, while a jerk of at least 60 m/s³ suppresses
that acceleration correction for one cycle. This avoids treating normal
sample-to-sample sensor noise as dynamics.

The INT connection is `MPU-6500 INT -> ESP32-S3 GPIO4`; GPIO4 is the component
Kconfig default `ESP_MP65_PIN_INT` and can be changed in the **ESP MP65
(MPU-6500)** menu. The application passes that default to
`esp_mp65_data_ready_start()` on Core 1; the component installs the GPIO ISR
before enabling the MPU-6500 raw-data-ready pulse. The ISR timestamps an
edge and overwrites its one-item release queue. The application waits through
`esp_mp65_wait_data_ready()`; all SPI and floating-point work
remains in the Core 1 acquisition task. A sequence gap means the task was not
keeping up and only the newest release is retained. It never processes a
catch-up burst of possibly stale register data.

The gyroscope and accelerometer use DLPF setting 3 and a divider of zero. This
is the highest documented common filtered rate: 1 kHz. The MPU-6500 can expose
faster individual paths (8 kHz gyro and 4 kHz accelerometer), but that does not
establish that both vectors in a single register burst are new at every event.
A future 4 kHz implementation must validate the sensor's event semantics and
should use FIFO framing.

Periodic reports are copied without waiting to a two-item queue and printed by
a separate Core 0 task. If full, the newest report is dropped;
`dropped_reports_total` is cumulative. The Core 1 task runs at priority 10;
the CPU is configured at 240 MHz and dynamic frequency scaling is disabled.
This is still an integration demonstration, not a proven hard-real-time loop.

## Diagnostic reports

After calibration, the current application logger emits one approximately
two-second wall-time window record. These are application snapshots, not
averages of every field and not yet `esp_rt_diagnostics_reporter` snapshots:

| Existing information | Current implementation | Possible future destination |
|---|---|---|
| `work_max_us`, `wake_max_us`, `late_cycles`, `skipped_releases`, `stale_releases` | Counters updated directly in `main.c` | `esp_rt_diagnostics` cycle snapshot |
| `dt_us`, `isr_gap_max_us` | Application timestamps from INT/read flow | `esp_rt_diagnostics` interval, with endpoint semantics documented |
| `read_errors`, `update_errors`, timeouts, correction counts | Application counters and `imu_attitude` results | `esp_rt_diagnostics` events plus optional payload detail |
| gyro peaks, saturation, jerk, innovation, bias, Euler angles | Application-specific fields in `diagnostic_report_t` | Reporter application payload/callback |
| calibration accepted/rejected/progress records | `kind` records in the application queue | Payload state, bounded event path, or retained application logger |

The future destination column is a migration design, not behavior currently
enabled by this application.

- `window`: start/end timestamps in microseconds since boot. Angles and
  `bias_dps` are end-of-window snapshots, not averages. Angles may be stale
  when reads or filter updates fail.
- `gyro_peak_dps`: maximum absolute measured X/Y/Z angular rates, before bias
  subtraction. The configured full scale is printed at startup.
- `near_sat_xyz`: sample counts per axis with absolute raw ADC value at least
  31130 (95% of 32768). `near_sat_any` counts a sample once if any axis is near
  saturation. This indicates proximity to clipping, not proof of clipping;
  unobserved peaks between reads cannot be detected.
- `reads`, `read_errors`: successful and failed acquisitions in this window.
  Reporting continues during acquisition failures, provided reads return.
- `dt_us`: minimum/maximum time between data-ready ISR timestamps for
  consecutive successful reads, including the calibration-to-first-read interval
  and window boundaries.
  A failed acquisition increases the next successful-read gap. `intervals=0`
  means there is no interval measurement in this window (`dt_us=[0,0]`).
  These are host timestamps, not sensor sample timestamps or ISR latency.
- `update_errors`: failed filter updates; prediction may already have committed
  before a correction failure.
- `late_cycles`: completed acquisition/filter iterations whose observed work
  time plus ISR-to-task wake latency reached the nominal 1 ms interval.
- `skipped_releases`: INT edges overwritten in the one-item release queue;
  `stale_releases` are retained edges whose task wake latency was already at
  least 1 ms and were deliberately not read. `no_data_timeouts` counts 100 ms
  waits with no INT edge, which indicates a lost wire, disabled sensor interrupt,
  or halted sensor rather than a synthetic timer release.
- `accel_used`, `accel_rejected`: all accepted and component-rejected
  (magnitude or innovation-hard gate) corrections. `accel_dynamic_rejected`
  counts corrections skipped by the application's jerk-hard gate;
  `accel_dynamic_downweighted` counts calls with jerk-increased variance, and
  `jerk_max_mps3` is the maximum vector jerk of 20 ms rolling acceleration
  means separated by 10 ms. It is unavailable (reported as zero) during the
  first 30 valid samples after calibration or a timing discontinuity.
- `innovation_norm_max`: maximum Euclidean norm of normalized measured
  acceleration minus predicted gravity direction, evaluated after gyro
  prediction and before correction, including magnitude-gate rejections.
  It is dimensionless (0 to approximately 2), not NIS or an angle in degrees.
  `innovation_samples` gives the number of valid evaluations; zero means the
  reported maximum is unavailable.
- `bias_dps`: estimated X/Y/Z bias with five decimal places, to compare against
  startup and subsequent windows. Display precision is not sensor accuracy.

All window counters reset after each report attempt. Timing and peak statistics
are observed values, not worst-case bounds. For the saturation comparison test,
the app explicitly sets this sensor instance to +/-1000 deg/s, overriding the
gyroscope range in saved Kconfig settings. The driver configures both the range
register and the matching unit conversion. Filter tuning, accelerometer range,
diagnostic thresholds are unchanged. The INT-driven comparison uses 1 kHz
acquisition and processing, with the same two-second report interval. Expect
approximately 2000 reads per report and `dt_us` close to `[1000,1000]`, with
zero skipped/stale releases and data timeouts. `isr_gap_max_us` is an observed
interval between GPIO ISR entries; it is not external timing capture or a
worst-case bound.
Check for `gyro full scale=+/- 1000 deg/s` at startup. To return to the Kconfig
range, remove the `sensor_config.gyro_range` assignment in `main.c`.

```sh
idf.py set-target esp32s3
idf.py build flash monitor
```

Wiring and GPIO defaults are documented by the `esp_mp65` component. Change
them under **ESP MP65 (MPU-6500)** in `menuconfig` when required.
