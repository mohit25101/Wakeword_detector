set(EI_SDK_FOLDER "${CMAKE_CURRENT_LIST_DIR}/edge-impulse-sdk")

include("${EI_SDK_FOLDER}/cmake/utils.cmake")

RECURSIVE_FIND_FILE_APPEND(EI_SOURCE_FILES "${EI_SDK_FOLDER}" "*.cpp")
RECURSIVE_FIND_FILE_APPEND(EI_SOURCE_FILES "${EI_SDK_FOLDER}" "*.cc")
RECURSIVE_FIND_FILE_APPEND(EI_SOURCE_FILES "${EI_SDK_FOLDER}" "*.s")

RECURSIVE_FIND_FILE_APPEND(MODEL_SOURCE
    "${CMAKE_CURRENT_LIST_DIR}/tflite-model" "*.cpp")

list(APPEND EI_SOURCE_FILES
    "${EI_SDK_FOLDER}/tensorflow/lite/c/common.c")
# ESP-DSP FFT implementation required by Edge Impulse
set(EI_DSP_DIR
    "${CMAKE_CURRENT_LIST_DIR}/edge-impulse-sdk/porting/espressif/esp-dsp"
)

list(APPEND EI_SOURCE_FILES
    "${EI_DSP_DIR}/modules/fft/float/dsps_fft2r_fc32_ansi.c"
    "${EI_DSP_DIR}/modules/fft/float/dsps_fft2r_bitrev_tables_fc32.c"
    "${EI_DSP_DIR}/modules/common/misc/dsps_pwroftwo.cpp"
)
idf_component_register(
    SRCS ${EI_SOURCE_FILES} ${MODEL_SOURCE}
    INCLUDE_DIRS "."
"edge-impulse-sdk/porting/espressif/esp-dsp/modules/common/include"
    "edge-impulse-sdk/porting/espressif/esp-dsp/modules/fft/include"
    PRIV_REQUIRES esp_timer esp_nn
)

# The bundled TensorFlow Lite sources have legacy aggregate initializers.
# Keep warnings enabled, but don't fail this component on this warning.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=missing-field-initializers
)
# Keep legacy TensorFlow Lite warnings from failing the build.
# Compatibility with legacy Edge Impulse/TFLite aggregate initializers.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-missing-field-initializers
    -Wno-error=maybe-uninitialized
    -Wno-error=deprecated-declarations
)
