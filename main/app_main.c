#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "audio_codec.h"
#include "audio_route.h"
#include "bt_hfp.h"
#include "recorder.h"

static const char *TAG = "app_main";

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    ESP_LOGI(TAG, "esp-headset boot");
    ESP_LOGI(TAG, "Phase 3: HFP RX playback + mic uplink bring-up");

    init_nvs();

    ESP_ERROR_CHECK(audio_codec_init());
    ESP_LOGI(TAG, "Step check: playing local I2S test tone before Bluetooth init");
    ESP_ERROR_CHECK(audio_codec_play_test_tone(400));
    ESP_ERROR_CHECK(audio_route_init());
    ESP_ERROR_CHECK(recorder_init());
    ESP_ERROR_CHECK(bt_hfp_init());

    ESP_LOGI(TAG, "Initialization complete");
    ESP_LOGI(TAG, "Pair the phone with device name 'ESP-HEADSET'");
    ESP_LOGI(TAG, "Then connect HFP, place a call, and confirm RX audio is routed to the headset output");
    ESP_LOGI(TAG, "Phase 3 adds mic capture and uplink path; confirm the remote side can hear your voice");
}
