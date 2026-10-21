#pragma once
#include <Arduino.h>
#include "esp_timer.h"
#include "config.h"

// Independent blue GPIO LED. Timer-task callbacks keep the indication alive
// through host startup waits, sensor settling/calibration and fatal-error loops.
// Only this LED's PWM duty is written; no motor, serial or I2C work happens here.
namespace status_led {
enum Mode : uint8_t { BOOT, READY, ARMING, BALANCING, STOPPED, MAINTENANCE, ERROR };
static volatile Mode mode = BOOT;
static esp_timer_handle_t timer = nullptr;
static bool running = false;

static void writeDuty(uint32_t brightness) {
    static uint32_t previous = 256;
    const uint32_t duty = STATUS_LED_ACTIVE_HIGH ? brightness : 255-brightness;
    if (duty == previous) return;
    previous = duty;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(STATUS_LED_PIN, duty);
#else
    ledcWrite(STATUS_LED_CH, duty);
#endif
}

static void tick(void*) {
    const uint32_t now = millis();
    uint32_t brightness = 0;
    switch (mode) {
        case BOOT: brightness = now%1000 < 500 ? 255 : 0; break;
        case READY: brightness = 255; break;
        case ARMING: brightness = now%400 < 200 ? 255 : 0; break;
        case ERROR: brightness = now%200 < 100 ? 255 : 0; break;
        case STOPPED: {
            const uint32_t phase = now%1600;
            brightness = phase < 100 || (phase >= 200 && phase < 300) ? 255 : 0;
            break;
        }
        case MAINTENANCE: {
            const uint32_t phase = now%1600;
            brightness = phase < 600 && phase%200 < 100 ? 255 : 0;
            break;
        }
        case BALANCING: {
            const uint32_t phase = now%2400;
            const uint32_t ramp = phase <= 1200 ? phase : 2400-phase;
            // Gamma-shaped 2.4-second breathing cycle, never fully dark.
            brightness = 8 + ramp*ramp*247/(1200UL*1200UL);
            break;
        }
    }
    writeDuty(brightness);
}

inline bool begin() {
    digitalWrite(STATUS_LED_PIN, STATUS_LED_ACTIVE_HIGH ? LOW : HIGH);
    pinMode(STATUS_LED_PIN, OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    if (!ledcAttachChannel(STATUS_LED_PIN, STATUS_LED_PWM_HZ, STATUS_LED_PWM_BITS, STATUS_LED_CH)) return false;
#else
    if (ledcSetup(STATUS_LED_CH, STATUS_LED_PWM_HZ, STATUS_LED_PWM_BITS) == 0) return false;
    ledcAttachPin(STATUS_LED_PIN, STATUS_LED_CH);
#endif
    tick(nullptr);
    esp_timer_create_args_t args = {};
    args.callback = tick;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "status_led";
    if (esp_timer_create(&args, &timer) != ESP_OK) return false;
    running = esp_timer_start_periodic(timer, 20000) == ESP_OK;
    return running;
}

inline void setMode(Mode next) {
    mode = next;
    if (!running) tick(nullptr); // best-effort fallback if timer allocation failed
}
} // namespace status_led
