#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "kalman_attitude.h"

/* This header exposes only the application adapter. Framing, CRC, USB
 * transport, and the Octave decoder are implemented by the reusable
 * esp_stream component, not here. */

/**
 * @brief Create the optional application attitude stream.
 *
 * Called externally by app_main() exactly once before the acquisition task is
 * started. In the enabled build it describes the five application channels
 * and creates esp_stream's Core 0 producer/USB resources. It performs setup,
 * allocation, and error logging, so it is boot-only and task-context code.
 * When CONFIG_MP65_ATTITUDE_STREAM is disabled it is a no-op.
 */
void attitude_stream_start(void);

/**
 * @brief Hand off a fresh application attitude snapshot at the viewer rate.
 *
 * Called externally by the Core 1 acquisition task after imu_attitude_update()
 * succeeds. The function calls kalman_attitude_get_quaternion() to copy the
 * caller-owned filter state, quantizes qw/qx/qy/qz to int16, adds the ready
 * flag, and invokes esp_stream_publish(). `fresh` must be false after a sensor
 * read or estimator failure; false suppresses publication so an old quaternion
 * cannot receive a newer timestamp. `time_us` is the data-ready ISR timestamp,
 * not a hardware conversion timestamp. The 40 ms cadence gate and fixed copy
 * are application policy; USB I/O and formatting remain in esp_stream's
 * worker. This function is task-context only and must not be called from an
 * ISR. The filter pointer is borrowed for the duration of the call.
 *
 * @param filter Caller-owned attitude estimator; must remain valid during call.
 * @param time_us Data-ready ISR timestamp in microseconds since boot.
 * @param ready True after application alignment/calibration completes.
 * @param fresh True only after a successful new attitude-policy update.
 */
void attitude_stream_publish(const kalman_attitude_t *filter, int64_t time_us,
                             bool ready, bool fresh);
