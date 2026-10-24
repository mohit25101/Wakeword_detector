
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        edge_impulse
        esp_timer
        esp_driver_i2s
        esp_driver_gpio
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=missing-field-initializers
    -Wno-error=uninitialized
    -Wno-error=maybe-uninitialized
)

