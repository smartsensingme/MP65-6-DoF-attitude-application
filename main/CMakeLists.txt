idf_component_register(
    SRCS "main.c" "attitude_stream.c"
    PRIV_REQUIRES esp_mp65 imu_attitude kalman_attitude esp_timer esp_driver_gpio esp_stream
)
