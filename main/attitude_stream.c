#include "attitude_stream.h"
#include "sdkconfig.h"
#if CONFIG_MP65_ATTITUDE_STREAM
#include "esp_stream.h"
#include <math.h>

/* Application adapter: channel names, quaternion quantization and publication
 * cadence belong here; framing, CRC, USB I/O and receiver state belong to the
 * reusable esp_stream component. */
static esp_stream_handle_t stream;

/* External caller: app_main() during boot. Internal callee: esp_stream_create()
 * allocates/configures the generic transport worker on Core 0. */
void attitude_stream_start(void) {
  const esp_stream_channel_t channels[] = {
      {.name = "qw", .unit = "1", .scale = 1.0f / 32767.0f},
      {.name = "qx", .unit = "1", .scale = 1.0f / 32767.0f},
      {.name = "qy", .unit = "1", .scale = 1.0f / 32767.0f},
      {.name = "qz", .unit = "1", .scale = 1.0f / 32767.0f},
      {.name = "ready", .unit = "bool", .scale = 1.0f},
  };
  const esp_stream_config_t config = {
      .channels = channels,
      .channel_count = 5,
      .policy = ESP_STREAM_LATEST,
      .queue_blocks = 1,
      .task_core = 0,
  };
  ESP_ERROR_CHECK(esp_stream_create(&config, &stream));
}

/* External caller: acquisition_task() after a successful fresh estimator
 * update. Internal callees: kalman_attitude_get_quaternion() copies the
 * application estimator state; esp_stream_publish() copies a bounded row for
 * the generic transport. No USB transaction is performed here. */
void attitude_stream_publish(const kalman_attitude_t *filter, int64_t time_us,
                             bool ready, bool fresh) {
  static int64_t last_us;
  /* A failed read/update must not advance last_us or emit an old quaternion
   * under a newer acquisition timestamp. */
  if (!fresh)
    return;
  if (time_us - last_us < 40000)
    return;
  kalman_real_t q[4];
  if (kalman_attitude_get_quaternion(filter, q) != KALMAN_OK)
    return;
  int16_t values[5];
  for (unsigned i = 0; i < 4; ++i) {
    if (!isfinite(q[i]))
      return;
    float scaled = fminf(32767.0f, fmaxf(-32767.0f, (float)q[i] * 32767.0f));
    values[i] = (int16_t)lroundf(scaled);
  }
  values[4] = ready ? 1 : 0;
  /* Valid dimensions and LATEST policy: no FIFO rejection or USB wait. */
  if (esp_stream_publish(stream, values, 1, (uint64_t)time_us, 0) == ESP_OK)
    last_us = time_us;
}
#else
void attitude_stream_start(void) {}
void attitude_stream_publish(const kalman_attitude_t *filter, int64_t time_us,
                             bool ready, bool fresh) {
  (void)filter;
  (void)time_us;
  (void)ready;
  (void)fresh;
}
#endif
