/*
 * MP65 application integration.
 *
 * The file has two deliberately separate responsibilities:
 *
 * 1. The Core 1 application path waits for INT, reads the sensor, applies the
 *    imu_attitude policy, and owns the kalman_attitude estimator.
 * 2. A bounded application-local diagnostic collector copies counters/state to
 *    a Core 0 logger. It is not esp_rt_diagnostics_reporter. If the generic
 *    reporter is adopted, this collector becomes the application payload and
 *    esp_rt_diagnostics supplies the generic timing snapshot.
 *
 * No logger, formatting, USB transfer, or dynamic allocation is intentionally
 * performed in the acquisition loop except for the bounded queue handoffs
 * documented at their call sites.
 */
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "attitude_stream.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mp65.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "imu_attitude.h"
#include "kalman_attitude.h"

#define SAMPLE_PERIOD_US 1000U /* Current MPU-6500 common filtered rate. */
#define DATA_READY_GPIO                                                        \
  ESP_MP65_DEFAULT_INT_GPIO /* Application wiring default. */
_Static_assert(configNUM_CORES == 2, "Acquisition requires two cores");
#define REPORT_PERIOD_US INT64_C(2000000) /* Application logger window. */
/* ceil(0.95 * 32768): test raw signed ADC counts before bias subtraction. */
#define GYRO_NEAR_SATURATION_RAW 31130
#define RAD_TO_DEG 57.295779513082320876f

static const char *TAG = "mp65_attitude";

typedef struct {
  /* Application-window counters. These are not generic reporter counters. */
  uint32_t reads, read_errors, update_errors, accepted, rejected;
  uint32_t accel_dynamic_rejected, accel_dynamic_downweighted;
  uint32_t near_sat[3], near_sat_any, innovation_samples;
  uint32_t interval_samples, dropped_reports, late_cycles;
  uint32_t skipped_releases, no_data_timeouts, stale_releases;
  int64_t work_max_us, wake_max_us, isr_gap_max_us;

  /* kind selects a local logger record: window, calibrated, rejected,
   * or calibrating. The generic reporter has no equivalent event queue. */
  uint32_t kind;
  kalman_real_t calibration_values[4], bias_std[3];

  /* Window boundaries and application-specific estimator payload. */
  int64_t start_us, end_us, dt_min_us, dt_max_us;
  float gyro_peak_dps[3];
  float accel_jerk_max_mps3;
  kalman_real_t innovation_max_squared;
  kalman_real_t roll, pitch, yaw, bias[3];
} diagnostic_report_t;

/* Application-local report record. This is deliberately not
 * esp_rt_diag_snapshot_t: it combines timing counters with MP65/Kalman fields
 * and calibration event kinds. A future generic reporter integration should
 * carry these application fields in a separate fixed-size payload. */

/**
 * @brief Format application-owned diagnostic records on Core 0.
 *
 * Called internally as the `imu_report` task entry point created by
 * acquisition_task(). It blocks on the application queue, never reads live
 * sensor/filter state, and performs all ESP_LOG formatting outside the Core 1
 * acquisition path. It is intentionally not esp_rt_diagnostics_reporter:
 * records contain MP65/Kalman fields and local `kind` events.
 */
static void diagnostic_logger(void *arg) {
  QueueHandle_t queue = (QueueHandle_t)arg;
  diagnostic_report_t r;
  while (true) {
    if (xQueueReceive(queue, &r, portMAX_DELAY) != pdTRUE)
      continue;
    if (r.kind == 1) {
      ESP_LOGI(TAG,
               "stationary calibration complete; bias_dps=[%+.5f,%+.5f,%+.5f] "
               "uncertainty_dps=[%.3f,%.3f,%.3f]; yaw is relative",
               (double)(r.bias[0] * RAD_TO_DEG),
               (double)(r.bias[1] * RAD_TO_DEG),
               (double)(r.bias[2] * RAD_TO_DEG),
               (double)(r.bias_std[0] * RAD_TO_DEG),
               (double)(r.bias_std[1] * RAD_TO_DEG),
               (double)(r.bias_std[2] * RAD_TO_DEG));
      continue;
    }
    if (r.kind == 2) {
      ESP_LOGW(TAG,
               "calibration rejected: |a|=%.3f |gyro|=%.3f deg/s "
               "max_std=[%.3f m/s^2, %.3f deg/s]; hold still",
               (double)r.calibration_values[0],
               (double)(r.calibration_values[1] * RAD_TO_DEG),
               (double)r.calibration_values[2],
               (double)(r.calibration_values[3] * RAD_TO_DEG));
      continue;
    }
    if (r.kind == 3) {
      ESP_LOGI(TAG,
               "calibrating: no_data_timeouts=%" PRIu32 " read_errors=%" PRIu32
               " skipped_releases=%" PRIu32 " late_cycles=%" PRIu32,
               r.no_data_timeouts, r.read_errors, r.skipped_releases,
               r.late_cycles);
      continue;
    }
    ESP_LOGI(TAG,
             "window=[%" PRId64 ",%" PRId64 "] us roll=%+.2f deg "
             "pitch=%+.2f deg relative-yaw=%+.2f deg",
             r.start_us, r.end_us, (double)(r.roll * RAD_TO_DEG),
             (double)(r.pitch * RAD_TO_DEG), (double)(r.yaw * RAD_TO_DEG));
    ESP_LOGI(TAG,
             "gyro_peak_dps=[%.2f,%.2f,%.2f] near_sat_xyz=[%" PRIu32 ",%" PRIu32
             ",%" PRIu32 "] near_sat_any=%" PRIu32,
             (double)r.gyro_peak_dps[0], (double)r.gyro_peak_dps[1],
             (double)r.gyro_peak_dps[2], r.near_sat[0], r.near_sat[1],
             r.near_sat[2], r.near_sat_any);
    ESP_LOGI(TAG,
             "reads=%" PRIu32 " read_errors=%" PRIu32 " dt_us=[%" PRId64
             ",%" PRId64 "] intervals=%" PRIu32 " update_errors=%" PRIu32
             " late_cycles=%" PRIu32 " dropped_reports_total=%" PRIu32,
             r.reads, r.read_errors, r.dt_min_us, r.dt_max_us,
             r.interval_samples, r.update_errors, r.late_cycles,
             r.dropped_reports);
    ESP_LOGI(TAG,
             "timing: work_max_us=%" PRId64 " wake_max_us=%" PRId64
             " isr_gap_max_us=%" PRId64 " skipped_releases=%" PRIu32
             " stale_releases=%" PRIu32 " no_data_timeouts=%" PRIu32,
             r.work_max_us, r.wake_max_us, r.isr_gap_max_us, r.skipped_releases,
             r.stale_releases, r.no_data_timeouts);
    ESP_LOGI(TAG,
             "accel_used=%" PRIu32 " accel_rejected=%" PRIu32
             " accel_dynamic_rejected=%" PRIu32
             " accel_dynamic_downweighted=%" PRIu32 " jerk_max_mps3=%.2f"
             " innovation_norm_max=%.6f innovation_samples=%" PRIu32
             " bias_dps=[%+.5f,%+.5f,%+.5f]",
             r.accepted, r.rejected, r.accel_dynamic_rejected,
             r.accel_dynamic_downweighted, (double)r.accel_jerk_max_mps3,
             (double)KALMAN_REAL_SQRT(r.innovation_max_squared),
             r.innovation_samples, (double)(r.bias[0] * RAD_TO_DEG),
             (double)(r.bias[1] * RAD_TO_DEG),
             (double)(r.bias[2] * RAD_TO_DEG));
  }
}

/**
 * @brief Close and enqueue the current application diagnostic window when due.
 *
 * Called internally by acquisition_task() after a release, timeout, or stale
 * event. It copies the current counters and (when aligned) the latest Euler
 * angles/bias into the application queue using a zero-timeout send. It also
 * clears the window and starts the next one. The queue is separate from the
 * optional generic esp_rt_diagnostics_reporter; this function is the current
 * application's report collector, not a generic snapshot API.
 *
 * @param r Mutable Core 1 application counters/payload.
 * @param queue Core 0 application logger queue; must be non-NULL.
 * @param attitude Borrowed estimator used only for final Euler/bias copies.
 * @param aligned Whether the estimator has reached RUNNING/aligned state.
 * @param dropped Cumulative count of application queue rejections.
 */
static void publish_if_due(diagnostic_report_t *r, QueueHandle_t queue,
                           const kalman_attitude_t *attitude, bool aligned,
                           uint32_t *dropped) {
  const int64_t now = esp_timer_get_time();
  if (now - r->start_us < REPORT_PERIOD_US)
    return;
  r->end_us = now;
  r->kind = aligned ? 0 : 3;
  r->dropped_reports = *dropped;
  if (aligned) {
    (void)kalman_attitude_get_euler(attitude, &r->roll, &r->pitch, &r->yaw);
    (void)kalman_attitude_get_gyro_bias(attitude, r->bias);
  }
  if (xQueueSend(queue, r, 0) != pdTRUE)
    ++*dropped;
  memset(r, 0, sizeof(*r));
  r->start_us = now;
}

/**
 * @brief Own the complete Core 1 sensor, estimator, and application policy.
 *
 * Created and pinned by app_main(). Initialization creates the MP65 handle,
 * estimator, application logger queue/task, and managed data-ready GPIO. The
 * loop then waits for the newest INT release, rejects skipped/stale releases,
 * reads one coherent sensor burst, calls imu_attitude_update(), records local
 * diagnostic fields, and hands fresh attitude snapshots to the optional
 * attitude stream adapter. All sensor/filter state is owned here. The loop
 * deliberately does not run a catch-up burst after an overwritten release.
 *
 * The diagnostics updates in this function are observation bookkeeping, not
 * estimator policy: `imu_attitude` decides correction/gap outcomes, while this
 * task merely counts and copies them for the application logger (or a future
 * reporter payload). The task never calls a generic reporter in the current
 * build.
 */
static void acquisition_task(void *arg) {
  (void)arg;
  esp_mp65_config_t sensor_config = ESP_MP65_CONFIG_DEFAULT();
  /* Comparison test: override the saved Kconfig range for this instance. */
  sensor_config.gyro_range = ESP_MP65_GYRO_RANGE_1000DPS;
  sensor_config.dlpf_cfg = 3; /* Filtered accel + gyro, common 1 kHz output. */
  sensor_config.sample_rate_divider = 0;
  sensor_config.clock_speed_hz = 1000000;
  sensor_config.acquire_bus_on_init =
      true; /* Dedicated host in this demo only. */
  const imu_attitude_config_t imu_config = imu_attitude_config_default();
  esp_mp65_handle_t sensor = NULL;
  imu_attitude_t imu;
  const kalman_attitude_t *attitude;
  diagnostic_report_t diagnostics = {0};
  uint32_t dropped_reports = 0;
  QueueHandle_t report_queue;
  bool aligned = false;

  /* APPLICATION INITIALIZATION: the sensor/filter and the local diagnostic
   * consumer are created before the INT source is enabled. The generic
   * esp_stream adapter itself is initialized by app_main(), before this task.
   */
  ESP_ERROR_CHECK(esp_mp65_init(&sensor_config, &sensor));
  if (imu_attitude_init(&imu, &imu_config) != KALMAN_OK) {
    ESP_LOGE(TAG, "attitude initialization failed");
    ESP_ERROR_CHECK(esp_mp65_deinit(sensor));
    vTaskDelete(NULL);
    return;
  }
  attitude = imu_attitude_filter(&imu);
  report_queue = xQueueCreate(2, sizeof(diagnostic_report_t));
  if (report_queue == NULL) {
    ESP_LOGE(TAG, "diagnostic queue allocation failed");
    ESP_ERROR_CHECK(esp_mp65_deinit(sensor));
    vTaskDelete(NULL);
    return;
  }
  if (xTaskCreatePinnedToCore(diagnostic_logger, "imu_report", 4096,
                              report_queue, 1, NULL, 0) != pdPASS) {
    vQueueDelete(report_queue);
    ESP_LOGE(TAG, "diagnostic logger creation failed");
    ESP_ERROR_CHECK(esp_mp65_deinit(sensor));
    vTaskDelete(NULL);
    return;
  }
  ESP_LOGI(TAG,
           "diagnostics: gyro full scale=+/- %u deg/s, near_sat=95%% "
           "ADC range, INT GPIO%d, nominal period=%u us, acquisition=core1 "
           "logger=core0",
           250U << (unsigned)sensor_config.gyro_range, DATA_READY_GPIO,
           SAMPLE_PERIOD_US);
  ESP_LOGI(TAG, "keep sensor stationary: calibrating in 2.5 seconds");

  const esp_mp65_data_ready_config_t ready_config = {
      .gpio_num = DATA_READY_GPIO,
      .install_isr_service = true,
  };
  diagnostics.start_us = esp_timer_get_time();
  ESP_ERROR_CHECK(esp_mp65_data_ready_start(sensor, &ready_config));
  uint32_t last_sequence = 0;
  while (true) {
    /* ACQUISITION RELEASE: esp_mp65_wait_data_ready() returns the newest
     * observed edge, not a buffered sensor sample. A timeout is a real fault
     * observation; never synthesize a read or estimator step. */
    esp_mp65_data_ready_event_t release;
    if (esp_mp65_wait_data_ready(sensor, &release, 100) != ESP_OK) {
      ++diagnostics.no_data_timeouts;
      imu_attitude_notify_gap(&imu);
      publish_if_due(&diagnostics, report_queue, attitude, aligned,
                     &dropped_reports);
      continue; /* No synthetic reads or integration without a sensor event. */
    }
    const int64_t work_start = esp_timer_get_time();

    /* RELEASE DIAGNOSTICS: these counters describe delivery and scheduling of
     * the application's release event. They are not the estimator result and
     * can later map to generic missed-event/wake/interval diagnostics. */
    const uint32_t skipped = release.sequence - last_sequence - 1U;
    last_sequence = release.sequence;
    diagnostics.skipped_releases += skipped;
    const int64_t wake_us = work_start - release.isr_us;
    if (wake_us > diagnostics.wake_max_us)
      diagnostics.wake_max_us = wake_us;
    if (release.gap_us > diagnostics.isr_gap_max_us)
      diagnostics.isr_gap_max_us = release.gap_us;
    if (skipped || release.gap_us > imu_config.calibration_gap_us) {
      imu_attitude_notify_gap(&imu);
    }
    if (wake_us >= SAMPLE_PERIOD_US) {
      /* APPLICATION VALIDITY POLICY: the retained release is too old for a
       * deterministic read, so report a gap and skip both SPI and fusion. */
      ++diagnostics.stale_releases;
      imu_attitude_notify_gap(&imu);
      publish_if_due(&diagnostics, report_queue, attitude, aligned,
                     &dropped_reports);
      continue;
    }

    /* SENSOR ACQUISITION: this is the only synchronous SPI burst in the
     * periodic path. The driver owns register conversion; this task owns the
     * response to a read failure. */
    esp_mp65_sample_t sample;
    const esp_err_t sensor_status = esp_mp65_read(sensor, &sample);
    const int64_t now_us = esp_timer_get_time();

    bool fresh_estimate = false;
    if (sensor_status != ESP_OK) {
      /* A failed read invalidates this release. imu_attitude is told about a
       * gap, and the stream adapter is not allowed to republish old attitude
       * state under this release timestamp. */
      ++diagnostics.read_errors;
      imu_attitude_notify_gap(&imu);
    } else {
      /* APPLICATION SENSOR OBSERVATION: saturation and peak statistics are
       * collected for the local report payload. They do not change the sample
       * or the filter directly. */
      const bool was_running = aligned;
      bool near_sat = false;
      for (size_t axis = 0; axis < 3; ++axis) {
        int32_t raw = sample.gyro_raw[axis];
        if (raw < 0)
          raw = -raw;
        if (raw >= GYRO_NEAR_SATURATION_RAW) {
          near_sat = true;
          if (was_running)
            ++diagnostics.near_sat[axis];
        }
        if (was_running) {
          const float peak = fabsf(sample.gyro_rads[axis]) * RAD_TO_DEG;
          if (peak > diagnostics.gyro_peak_dps[axis])
            diagnostics.gyro_peak_dps[axis] = peak;
        }
      }
      imu_attitude_sample_t input = {
          .time_us = release.isr_us,
          .discontinuity = skipped != 0,
          .saturated = near_sat,
      };
      memcpy(input.accel_mps2, sample.accel_mps2, sizeof(input.accel_mps2));
      memcpy(input.gyro_rads, sample.gyro_rads, sizeof(input.gyro_rads));
      imu_attitude_result_t result;

      /* ATTITUDE POLICY/ESTIMATION: imu_attitude owns calibration, bias
       * protection, acceleration gating, jerk weighting, gap handling, and
       * the call sequence into kalman_attitude. main.c only supplies the
       * coherent sample and records the returned result. */
      kalman_status_t status = imu_attitude_update(&imu, &input, &result);
      fresh_estimate = status == KALMAN_OK;
      aligned = result.state == IMU_ATTITUDE_RUNNING;
      if (status != KALMAN_OK)
        ++diagnostics.update_errors;
      if (result.event == IMU_ATTITUDE_CALIBRATED ||
          result.event == IMU_ATTITUDE_CALIBRATION_REJECTED) {
        /* APPLICATION EVENT HANDOFF: calibration records are local `kind`
         * messages. They are not generic reporter snapshots and are sent to
         * the same Core 0 logger queue without blocking Core 1. */
        diagnostic_report_t event = {
            .kind = result.event == IMU_ATTITUDE_CALIBRATED ? 1 : 2,
        };
        memcpy(event.calibration_values, result.calibration_values,
               sizeof(event.calibration_values));
        memcpy(event.bias, result.bias, sizeof(event.bias));
        memcpy(event.bias_std, result.bias_std, sizeof(event.bias_std));
        if (aligned)
          diagnostics.start_us = now_us;
        if (xQueueSend(report_queue, &event, 0) != pdTRUE)
          ++dropped_reports;
      }
      if (was_running) {
        /* APPLICATION DIAGNOSTIC COLLECTION: copy policy diagnostics into the
         * current window. These fields are candidates for a future reporter
         * payload; they are not required by the reusable filters. */
        ++diagnostics.reads;
        if (!diagnostics.interval_samples ||
            result.dt_us < diagnostics.dt_min_us)
          diagnostics.dt_min_us = result.dt_us;
        if (result.dt_us > diagnostics.dt_max_us)
          diagnostics.dt_max_us = result.dt_us;
        ++diagnostics.interval_samples;
        if (near_sat)
          ++diagnostics.near_sat_any;
        if (result.jerk_mps3 > diagnostics.accel_jerk_max_mps3)
          diagnostics.accel_jerk_max_mps3 = result.jerk_mps3;
        if (result.innovation_valid) {
          ++diagnostics.innovation_samples;
          if (result.innovation_squared > diagnostics.innovation_max_squared)
            diagnostics.innovation_max_squared = result.innovation_squared;
        }
        diagnostics.accepted +=
            result.correction == IMU_ATTITUDE_CORRECTION_USED;
        diagnostics.rejected +=
            result.correction == IMU_ATTITUDE_REJECT_MAGNITUDE ||
            result.correction == IMU_ATTITUDE_REJECT_INNOVATION;
        diagnostics.accel_dynamic_rejected +=
            result.correction == IMU_ATTITUDE_REJECT_JERK;
        diagnostics.accel_dynamic_downweighted += result.dynamic_downweighted;
      }
    }

    /* OPTIONAL APPLICATION STREAM: this adapter adds channel names,
     * quaternion quantization, and the 25 Hz viewer cadence. esp_stream owns
     * framing/CRC/USB worker behavior. `fresh_estimate` prevents an old
     * quaternion from being assigned the current release timestamp. */
    attitude_stream_publish(attitude, release.isr_us, aligned, fresh_estimate);

    /* APPLICATION REPORT HANDOFF: copy the local diagnostic window to the
     * current Core 0 logger when its two-second wall-time period expires. A
     * future esp_rt_diagnostics integration would take its immutable generic
     * snapshot after cycle end and publish this application's payload here. */
    publish_if_due(&diagnostics, report_queue, attitude, aligned,
                   &dropped_reports);

    /* CYCLE ACCOUNTING/RECOVERY: includes the current application's stream and
     * report-copy overhead. Boundary-cycle timing belongs to the next report.
     * An overrun yields to Idle instead of catch-up loops; normal releases
     * always come from INT, never the RTOS tick. */
    const int64_t work_us = esp_timer_get_time() - work_start;
    if (work_us > diagnostics.work_max_us)
      diagnostics.work_max_us = work_us;
    if (work_us + wake_us >= SAMPLE_PERIOD_US) {
      ++diagnostics.late_cycles;
      if (!aligned) {
        imu_attitude_notify_gap(&imu);
      }
      vTaskDelay(1); /* Fault recovery only; keep watchdog/Idle runnable. */
    }
  }
}

/**
 * @brief Boot entry point: start optional stream resources and Core 1 owner.
 *
 * Called by ESP-IDF once on the startup task. It performs no sensor work
 * itself; acquisition_task() owns all sensor/filter initialization and the
 * lifetime of those objects. The task is pinned to Core 1 so Core 0 remains
 * available for the application logger and optional esp_stream worker.
 */
void app_main(void) {
  attitude_stream_start();
  /* All sensor/filter ownership and peripheral initialization stay on Core 1.
   */
  BaseType_t created = xTaskCreatePinnedToCore(acquisition_task, "imu_acquire",
                                               8192, NULL, 10, NULL, 1);
  ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
