
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "model-parameters/model_metadata.h"

static const char *TAG = "WAKE_WORD";

// ------------------------------------------------------------
// ESP32-S3-EYE microphone pins
// ------------------------------------------------------------
#define MIC_BCLK_GPIO 41
#define MIC_WS_GPIO   42
#define MIC_DIN_GPIO  2

// ------------------------------------------------------------
// Model configuration
// ------------------------------------------------------------
static constexpr uint32_t AUDIO_SAMPLE_RATE = 16000;
static constexpr size_t AUDIO_SLICE_SIZE =
    EI_CLASSIFIER_SLICE_SIZE;

// Adjust this only after verifying the actual I2S sample format.
static constexpr int MIC_SAMPLE_SHIFT = 16;

// Wake-word hysteresis thresholds
static constexpr float WAKE_THRESHOLD = 0.80f;
static constexpr float RESET_THRESHOLD = 0.50f;

// ------------------------------------------------------------
// Buffers and I2S channel
// ------------------------------------------------------------
static i2s_chan_handle_t rx_chan = nullptr;

static int32_t *mic_buffer = nullptr;
static int16_t *audio_slice = nullptr;

static bool wake_latched = false;

// ------------------------------------------------------------
// Compile-time checks
// ------------------------------------------------------------
static_assert(
    EI_CLASSIFIER_FREQUENCY == AUDIO_SAMPLE_RATE,
    "The Edge Impulse model must use 16 kHz audio."
);

static_assert(
    EI_CLASSIFIER_SLICE_SIZE == 4000,
    "This program expects 4000 samples per slice."
);

// ------------------------------------------------------------
// Initialize microphone using ESP-IDF I2S standard driver
// ------------------------------------------------------------
static esp_err_t init_microphone()
{
    i2s_chan_config_t chan_cfg = {};
    chan_cfg.id = I2S_NUM_AUTO;
    chan_cfg.role = I2S_ROLE_MASTER;
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 240;
    chan_cfg.auto_clear = true;
    chan_cfg.intr_priority = 0;
    chan_cfg.allow_pd = false;

    esp_err_t err = i2s_new_channel(
        &chan_cfg,
        nullptr,
        &rx_chan
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    i2s_std_config_t std_cfg = {};

    std_cfg.clk_cfg =
        I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE);

    std_cfg.slot_cfg =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_MONO
        );

    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.bclk = (gpio_num_t)MIC_BCLK_GPIO;
    std_cfg.gpio_cfg.ws = (gpio_num_t)MIC_WS_GPIO;
    std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.din = (gpio_num_t)MIC_DIN_GPIO;

    std_cfg.gpio_cfg.invert_flags.mclk_inv = false;
    std_cfg.gpio_cfg.invert_flags.bclk_inv = false;
    std_cfg.gpio_cfg.invert_flags.ws_inv = false;

    err = i2s_channel_init_std_mode(rx_chan, &std_cfg);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S standard-mode init failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = i2s_channel_enable(rx_chan);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S enable failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Microphone initialized");
    ESP_LOGI(TAG, "Sample rate: %lu Hz",
             (unsigned long)AUDIO_SAMPLE_RATE);
    ESP_LOGI(TAG, "BCLK=%d WS=%d DIN=%d",
             MIC_BCLK_GPIO, MIC_WS_GPIO, MIC_DIN_GPIO);

    return ESP_OK;
}

// ------------------------------------------------------------
// Capture exactly one model slice of 4000 samples
// ------------------------------------------------------------
static esp_err_t capture_audio_slice(int64_t *elapsed_us)
{
    const size_t required_bytes =
        AUDIO_SLICE_SIZE * sizeof(int32_t);

    size_t total_bytes = 0;

    const int64_t start_us = esp_timer_get_time();

    while (total_bytes < required_bytes) {
        size_t bytes_read = 0;

        esp_err_t err = i2s_channel_read(
            rx_chan,
            reinterpret_cast<uint8_t *>(mic_buffer) + total_bytes,
            required_bytes - total_bytes,
            &bytes_read,
            pdMS_TO_TICKS(1000)
        );

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2S read failed: %s",
                     esp_err_to_name(err));
            return err;
        }

        if (bytes_read == 0) {
            ESP_LOGE(TAG, "I2S returned zero bytes");
            return ESP_FAIL;
        }

        total_bytes += bytes_read;
    }

    // Convert 32-bit I2S slots to signed 16-bit samples.
    // The correct shift depends on the microphone's sample alignment.
    for (size_t i = 0; i < AUDIO_SLICE_SIZE; ++i) {
        audio_slice[i] =
            static_cast<int16_t>(
                mic_buffer[i] >> MIC_SAMPLE_SHIFT
            );
    }

    *elapsed_us = esp_timer_get_time() - start_us;

    return ESP_OK;
}

// ------------------------------------------------------------
// Audio diagnostics: range, average amplitude and clipping
// ------------------------------------------------------------
static void log_audio_statistics()
{
    int16_t min_sample = INT16_MAX;
    int16_t max_sample = INT16_MIN;

    uint64_t sum_abs = 0;
    size_t clipped_samples = 0;

    for (size_t i = 0; i < AUDIO_SLICE_SIZE; ++i) {
        const int16_t sample = audio_slice[i];

        if (sample < min_sample) {
            min_sample = sample;
        }

        if (sample > max_sample) {
            max_sample = sample;
        }

        const int32_t wide_sample = sample;

        sum_abs += (wide_sample < 0)
            ? static_cast<uint32_t>(-wide_sample)
            : static_cast<uint32_t>(wide_sample);

        if (sample == INT16_MIN || sample == INT16_MAX) {
            ++clipped_samples;
        }
    }

    const uint64_t mean_abs = sum_abs / AUDIO_SLICE_SIZE;

    ESP_LOGI(
        TAG,
        "Audio min=%d max=%d mean_abs=%llu clipped=%u/%u",
        (int)min_sample,
        (int)max_sample,
        (unsigned long long)mean_abs,
        (unsigned)clipped_samples,
        (unsigned)AUDIO_SLICE_SIZE
    );
}

// ------------------------------------------------------------
// Edge Impulse signal callback
// ------------------------------------------------------------
static int get_signal_data(
    size_t offset,
    size_t length,
    float *out_ptr
)
{
    if (offset > AUDIO_SLICE_SIZE ||
        length > AUDIO_SLICE_SIZE - offset) {
        return -1;
    }

    for (size_t i = 0; i < length; ++i) {
        out_ptr[i] =
            static_cast<float>(audio_slice[offset + i]);
    }

    return 0;
}

// ------------------------------------------------------------
// Classify one captured audio slice
// ------------------------------------------------------------
static void classify_audio_slice()
{
    signal_t signal;
    signal.total_length = AUDIO_SLICE_SIZE;
    signal.get_data = get_signal_data;

    ei_impulse_result_t result = {};

    const int64_t start_us = esp_timer_get_time();

    // This is blocking. It is a diagnostic baseline, not a
    // real-time audio-processing loop.
    EI_IMPULSE_ERROR status =
        run_classifier_continuous(
            &signal,
            &result,
            false
        );

    const int64_t classifier_us =
        esp_timer_get_time() - start_us;

    if (status != EI_IMPULSE_OK) {
        ESP_LOGE(TAG, "Classifier failed: %d", (int)status);
        return;
    }

    ESP_LOGI(
        TAG,
        "Classifier call: %lld ms",
        (long long)(classifier_us / 1000)
    );

    ESP_LOGI(
        TAG,
        "DSP time: %d ms",
        result.timing.dsp
    );

    ESP_LOGI(
        TAG,
        "Classification time: %d ms",
        result.timing.classification
    );

    float hello_score = 0.0f;

    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; ++i) {
        const char *label = result.classification[i].label;
        const float score = result.classification[i].value;

        ESP_LOGI(TAG, "%s: %.3f", label, score);

        if (label != nullptr &&
            strcmp(label, "helloworld") == 0) {
            hello_score = score;
        }
    }

    if (!wake_latched && hello_score >= WAKE_THRESHOLD) {
        wake_latched = true;

        ESP_LOGW(
            TAG,
            "HELLO WORLD DETECTED! Confidence: %.3f",
            hello_score
        );

        // TODO: Trigger your application action here.
    } else if (wake_latched &&
               hello_score < RESET_THRESHOLD) {
        wake_latched = false;
        ESP_LOGI(TAG, "Wake-word detector re-armed");
    }
}

// ------------------------------------------------------------
// ESP-IDF application entry point
// ------------------------------------------------------------
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting Hello World wake-word detector");
    ESP_LOGI(TAG, "Model slice size: %u samples",
             (unsigned)AUDIO_SLICE_SIZE);
    ESP_LOGI(TAG, "Model labels: %u",
             (unsigned)EI_CLASSIFIER_LABEL_COUNT);

    mic_buffer = static_cast<int32_t *>(
        heap_caps_malloc(
            AUDIO_SLICE_SIZE * sizeof(int32_t),
            MALLOC_CAP_8BIT
        )
    );

    audio_slice = static_cast<int16_t *>(
        heap_caps_malloc(
            AUDIO_SLICE_SIZE * sizeof(int16_t),
            MALLOC_CAP_8BIT
        )
    );

    if (mic_buffer == nullptr || audio_slice == nullptr) {
        ESP_LOGE(TAG, "Audio buffer allocation failed");
        abort();
    }

    ESP_ERROR_CHECK(init_microphone());

    run_classifier_init();

    ESP_LOGI(TAG, "Classifier initialized");

    while (true) {
        int64_t capture_us = 0;

        esp_err_t err = capture_audio_slice(&capture_us);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Audio capture failed; retrying");
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        ESP_LOGI(
            TAG,
            "Audio capture: %lld ms",
            (long long)(capture_us / 1000)
        );

        log_audio_statistics();

        classify_audio_slice();
    }
}
