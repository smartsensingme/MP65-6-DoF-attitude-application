# Application guidance: `mp65_attitude`

This file specializes the repository-wide rules in the sibling `AGENTS.md` for
the MPU-6500 attitude application. Keep application names, pins, rates, payloads,
and migration decisions here; do not move them into the repository-wide file.

## Scope and architecture

The application combines:

- `esp_mp65` for MPU-6500 SPI access and the optional managed data-ready INT;
- `imu_attitude` for startup calibration, motion policy, and sequencing;
- `kalman_attitude` for the 6-DoF attitude estimator;
- an acquisition task on Core 1 and a logging/streaming worker on Core 0;
- `esp_stream` for optional native-USB quaternion streaming to Octave.

The reusable components own their generic responsibilities. Application code
owns sensor validity policy, stale-release handling, estimator interpretation,
quaternion quantization, and the meaning of diagnostic fields.

## Timing and INT contract

The current profile uses the MPU-6500 data-ready edge on the GPIO selected by
`ESP_MP65_PIN_INT` (default GPIO4). The application uses
`ESP_MP65_DEFAULT_INT_GPIO` when no per-instance override is required and passes
the resulting GPIO to `esp_mp65_data_ready_start()`. The runtime API remains
per-instance so another application or sensor handle may use another GPIO or no
INT at all.

The nominal common filtered output rate is 1 kHz and the acquisition deadline
is 1 ms. The GPIO ISR timestamps and overwrites a one-item release queue; it
does not perform SPI, floating-point work, logging, or allocation. The Core 1
task must compute sequence gaps, reject stale retained releases, and never run
an unbounded catch-up loop. Logging, USB, and housekeeping remain off Core 1.

The timestamp at the INT ISR is an observed release timestamp, not proof of the
sensor conversion instant. Preserve this distinction in reports and Octave
plots. Hardware timing claims remain empirical and must include competing load,
faults, and reporter/console activity.

## Analysis: migrating the existing diagnostic logs

The present application has a `diagnostic_report_t`, a two-item queue, and a
Core 0 `diagnostic_logger()`. Its approximately two-second reports contain both
generic timing evidence and application-specific estimator/sensor state. The
two reusable diagnostics components can replace the generic timing path, but
they do not automatically discover or reproduce this structure.

### What belongs in `esp_rt_diagnostics`

Create one `esp_rt_diag_t` owned by the Core 1 acquisition task. Configure its
cycle for the INT release event (`expected_period_us = 1000`) and the actual
deadline (`deadline_us = 1000`). Use contiguous IDs for bounded stages, events,
and intervals, for example. A two-second diagnostic window can preserve the
current report cadence, but use a longer window when the reporter's own burst
or host logging is a substantial fraction of two seconds:

- stages: SPI burst read, `imu_attitude_update`, and any other variable-duration
  operation whose budget is actionable;
- events: successful read, read error, successful estimator update, estimator
  error, no-data timeout, skipped release, stale release, accepted correction,
  magnitude/innovation rejection, and jerk rejection;
- intervals: ISR-to-ISR gap or estimator/sample `dt`, with the endpoint
  definition recorded in the application README.

Call `esp_rt_diag_isr_capture()` adjacent to the existing task notification,
pass the full notification count to `esp_rt_diag_cycle_begin_from_isr()`, end
every begun cycle on success and failure paths, and take snapshots only after
the cycle ends. Snapshot extraction must not read the live estimator from the
Core 0 task.

The generic snapshot then provides cycles, missed events, valid wake latency,
processing time, cycle time, deadline overruns, stage statistics, interval
ranges, and generic event counts. It is the authoritative source for timing
claims; do not infer missed releases from the average rate alone.

This is not a byte-for-byte replacement for the current queue. The existing
two-item `xQueueSend(..., 0)` path keeps older reports and rejects a new report
when full; the reporter intentionally has one slot and overwrites an older
pending item with the newest snapshot. Choose the freshness-oriented behavior
deliberately and expose missing reporter `window_id` values in the analysis.

### What remains an application payload

Keep these fields outside generic diagnostic structures and copy them into a
fixed-size reporter payload or callback-owned snapshot:

- `gyro_peak_dps`, `near_sat_xyz`, and `near_sat_any`;
- read/update counts when their exact application denominator matters;
- jerk maximum, innovation maximum, and accepted/rejected correction details;
- Euler angles, relative yaw, gyro bias, calibration means and bias uncertainty;
- calibration state/event and any application-specific error classification.

The payload should be copied only after the Core 1 cycle boundary. If values are
needed at the boundary, copy them into an application-owned staging structure
before the next release. Do not pass pointers to live Kalman or sensor state to
the reporter callback. The callback receives reporter-owned copies valid only
for that invocation. Calibration transitions currently travel as separate
`kind` records; the reporter has no independent event queue, so either encode
the transition and its values in the next application payload, keep a bounded
calibration-event path, or retain the existing logger until that requirement is
settled.

### What belongs in `esp_rt_diagnostics_reporter`

Create one reporter on Core 0 at lower priority than acquisition. Start with
`ESP_RT_DIAG_REPORT_COMPACT` or `ESP_RT_DIAG_REPORT_SILENT` plus a bounded
callback. Publish a completed generic snapshot and the fixed application
payload with `esp_rt_diag_reporter_publish()` without waiting. Count/handle a
publish error outside the deterministic work; do not synchronously log it in
Core 1.

The reporter uses a one-slot overwrite queue. A missing `window_id` means the
presentation task fell behind; it is not evidence that the acquisition task
missed a release. If every two-second report must be retained, use a separate
dimensioned recorder or `esp_stream` transport instead of increasing this queue
without a measured memory/timing budget.

Reporter classification distinguishes windows potentially affected by its own
output from quieter successor windows. Compare repeated quiet windows before
attributing a deadline or jitter change to the reporter. Console logs from
other components, USB polling, and host activity can still perturb a window
classified as quiet.

## Recommended migration sequence

1. Add the accumulator and instrument the existing acquisition loop while
   retaining `diagnostic_report_t` and `diagnostic_logger()` as an oracle.
2. Publish the generic snapshot plus application payload to a reporter in
   `SILENT` mode or a bounded callback. Compare cycle counts, missed releases,
   `dt` ranges, read errors, and maxima against the existing two-second logs.
3. Enable compact output for controlled tests. Record window IDs, quiet versus
   log-affected class, reporter drops, and payload-copy cost.
4. Only after several matched windows, remove duplicated generic fields from
   `diagnostic_report_t`. Keep the application payload for bias, innovation,
   saturation, calibration, and attitude values.
5. Keep the existing logger temporarily if calibration events or a complete
   archival record are still required; the reporter is asynchronous and
   freshness-oriented, not lossless.

Do not change Kalman gains, sensor ranges, task priorities, or bus ownership to
make a diagnostic number look better. Change one observation or load variable
at a time and compare the same physical motion and firmware configuration.

## Component-manager integration

If these external components are adopted, list only
`esp_rt_diagnostics_reporter` as a direct dependency in this application's
`main/idf_component.yml`; its manifest resolves the compatible transitive
`esp_rt_diagnostics` revision. Pin a tested tag or commit for a released test.
For local/offline experiments, document the explicit local override and keep
generated `managed_components/` content out of source control.
