#include "motion_controller.h"

#include "duybot_pins.h"

#include <algorithm>
#include <cstring>

#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>

namespace duybot {
namespace {

constexpr char kTag[] = "DuyBotMotion";
constexpr uint32_t kPwmFrequencyHz = 5000;
constexpr uint32_t kDefaultTimeoutMs = 1000;
constexpr uint32_t kMaximumTimeoutMs = 10000;
constexpr uint8_t kTurnSpeedFactorPct = 55;
constexpr UBaseType_t kQueueLength = 16;
constexpr TickType_t kControllerPollTicks = pdMS_TO_TICKS(10);

uint8_t PriorityFor(MotionSource source) {
    return static_cast<uint8_t>(source);
}

uint8_t ClampSpeed(int speed_pct) {
    return static_cast<uint8_t>(std::clamp(speed_pct, 0, 100));
}

void SetPwmDuty(uint8_t speed_pct) {
    const uint32_t duty = static_cast<uint32_t>(speed_pct) * 255 / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

}  // namespace

MotionController& MotionController::GetInstance() {
    static MotionController instance;
    return instance;
}

bool MotionController::Initialize() {
    if (task_ != nullptr) {
        return true;
    }
    if (!InitializeHardware()) {
        return false;
    }

    queue_ = xQueueCreate(kQueueLength, sizeof(MotionCommand));
    if (queue_ == nullptr) {
        ESP_LOGE(kTag, "Failed to create motion command queue");
        return false;
    }

    if (xTaskCreate(TaskEntry, "motion_ctrl", 4096, this, 7, &task_) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create motion controller task");
        vQueueDelete(queue_);
        queue_ = nullptr;
        return false;
    }

    ESP_LOGI(kTag, "Motion controller ready (queue=%u, watchdog=%ums)",
             static_cast<unsigned>(kQueueLength), static_cast<unsigned>(kDefaultTimeoutMs));
    return true;
}

bool MotionController::Submit(MotionSource source, MotionDirection direction, int speed_pct,
                              uint32_t duration_ms) {
    if (queue_ == nullptr || direction == MotionDirection::kStop) {
        return direction == MotionDirection::kStop && Stop(source);
    }

    MotionCommand command;
    command.direction = direction;
    command.source = source;
    command.speed_pct = ClampSpeed(speed_pct);
    command.duration_ms = std::clamp<uint32_t>(
        duration_ms == 0 ? kDefaultTimeoutMs : duration_ms,
        1, kMaximumTimeoutMs);

    const BaseType_t sent = PriorityFor(source) >= PriorityFor(MotionSource::kManual)
                                ? xQueueSendToFront(queue_, &command, 0)
                                : xQueueSendToBack(queue_, &command, 0);
    if (sent != pdTRUE) {
        dropped_count_ = dropped_count_ + 1;
        ESP_LOGW(kTag, "Command queue full; dropped source=%u direction=%s",
                 PriorityFor(source), DirectionName(direction));
        return false;
    }
    submitted_count_ = submitted_count_ + 1;
    return true;
}

bool MotionController::Stop(MotionSource source) {
    if (queue_ == nullptr) {
        return false;
    }
    MotionCommand command;
    command.direction = MotionDirection::kStop;
    command.source = source;
    return xQueueSendToFront(queue_, &command, 0) == pdTRUE;
}

void MotionController::EmergencyStop() {
    emergency_stop_requested_ = true;
    if (queue_ == nullptr) {
        return;
    }

    MotionCommand command;
    command.direction = MotionDirection::kStop;
    command.source = MotionSource::kSafety;
    if (xQueueSendToFront(queue_, &command, 0) != pdTRUE) {
        xQueueReset(queue_);
        xQueueSendToFront(queue_, &command, 0);
    }
}

MotionDirection MotionController::GetDirection() const {
    return direction_;
}

MotionSource MotionController::GetActiveSource() const {
    return active_source_;
}

uint8_t MotionController::GetActivePriority() const {
    return active_priority_;
}

bool MotionController::IsMoving() const {
    return direction_ != MotionDirection::kStop;
}

bool MotionController::IsReady() const {
    return queue_ != nullptr && task_ != nullptr;
}

uint32_t MotionController::GetSubmittedCount() const {
    return submitted_count_;
}

uint32_t MotionController::GetAppliedCount() const {
    return applied_count_;
}

uint32_t MotionController::GetDroppedCount() const {
    return dropped_count_;
}

MotionDirection MotionController::GetLastAppliedDirection() const {
    return last_applied_direction_;
}

uint8_t MotionController::GetLastAppliedSpeed() const {
    return last_applied_speed_;
}

MotionDirection MotionController::ParseDirection(const char* direction) {
    if (direction == nullptr) {
        return MotionDirection::kStop;
    }
    if (std::strcmp(direction, "forward") == 0) return MotionDirection::kForward;
    if (std::strcmp(direction, "backward") == 0) return MotionDirection::kBackward;
    if (std::strcmp(direction, "left") == 0) return MotionDirection::kLeft;
    if (std::strcmp(direction, "right") == 0) return MotionDirection::kRight;
    return MotionDirection::kStop;
}

const char* MotionController::DirectionName(MotionDirection direction) {
    switch (direction) {
        case MotionDirection::kForward: return "forward";
        case MotionDirection::kBackward: return "backward";
        case MotionDirection::kLeft: return "left";
        case MotionDirection::kRight: return "right";
        case MotionDirection::kStop: return "stop";
    }
    return "stop";
}

void MotionController::TaskEntry(void* arg) {
    static_cast<MotionController*>(arg)->Run();
}

void MotionController::Run() {
    MotionCommand command;
    while (true) {
        if (emergency_stop_requested_) {
            emergency_stop_requested_ = false;
            xQueueReset(queue_);
            ApplyStop();
        }

        if (xQueueReceive(queue_, &command, kControllerPollTicks) == pdTRUE) {
            ProcessCommand(command, esp_timer_get_time());
        }

        const int64_t now_us = esp_timer_get_time();
        if (deadline_us_ != 0 && now_us >= deadline_us_) {
            ESP_LOGD(kTag, "Motion watchdog expired for source=%u", active_priority_);
            ApplyStop();
        }
    }
}

void MotionController::ProcessCommand(const MotionCommand& command, int64_t now_us) {
    const uint8_t priority = PriorityFor(command.source);
    if (command.direction == MotionDirection::kStop) {
        if (priority >= active_priority_) {
            ApplyStop();
        }
        return;
    }

    if (priority < active_priority_ && direction_ != MotionDirection::kStop &&
        (deadline_us_ == 0 || now_us < deadline_us_)) {
        ESP_LOGD(kTag, "Rejected lower-priority source=%u while source=%u is active",
                 priority, active_priority_);
        return;
    }

    active_source_ = command.source;
    active_priority_ = priority;
    deadline_us_ = now_us + static_cast<int64_t>(command.duration_ms) * 1000;
    ApplyDirection(command.direction, command.speed_pct);
}

void MotionController::ApplyDirection(MotionDirection direction, uint8_t speed_pct) {
    SetPwmDuty(0);
    gpio_set_level(kMotorLeftIn1, 0);
    gpio_set_level(kMotorLeftIn2, 0);
    gpio_set_level(kMotorRightIn1, 0);
    gpio_set_level(kMotorRightIn2, 0);
    esp_rom_delay_us(500);

    switch (direction) {
        case MotionDirection::kForward:
            gpio_set_level(kMotorLeftIn2, 1);
            gpio_set_level(kMotorRightIn1, 1);
            break;
        case MotionDirection::kBackward:
            gpio_set_level(kMotorLeftIn1, 1);
            gpio_set_level(kMotorRightIn2, 1);
            break;
        case MotionDirection::kLeft:
            gpio_set_level(kMotorLeftIn1, 1);
            gpio_set_level(kMotorRightIn1, 1);
            speed_pct = static_cast<uint8_t>(speed_pct * kTurnSpeedFactorPct / 100);
            break;
        case MotionDirection::kRight:
            gpio_set_level(kMotorLeftIn2, 1);
            gpio_set_level(kMotorRightIn2, 1);
            speed_pct = static_cast<uint8_t>(speed_pct * kTurnSpeedFactorPct / 100);
            break;
        case MotionDirection::kStop:
            ApplyStop();
            return;
    }

    direction_ = direction;
    SetPwmDuty(speed_pct);
    last_applied_direction_ = direction;
    last_applied_speed_ = speed_pct;
    applied_count_ = applied_count_ + 1;
}

void MotionController::ApplyStop() {
    SetPwmDuty(0);
    gpio_set_level(kMotorLeftIn1, 0);
    gpio_set_level(kMotorLeftIn2, 0);
    gpio_set_level(kMotorRightIn1, 0);
    gpio_set_level(kMotorRightIn2, 0);
    direction_ = MotionDirection::kStop;
    active_source_ = MotionSource::kIdle;
    active_priority_ = 0;
    deadline_us_ = 0;
}

bool MotionController::InitializeHardware() {
    gpio_config_t output = {};
    output.intr_type = GPIO_INTR_DISABLE;
    output.mode = GPIO_MODE_OUTPUT;
    output.pin_bit_mask = (1ULL << kMotorLeftIn1) | (1ULL << kMotorLeftIn2) |
                          (1ULL << kMotorRightIn1) | (1ULL << kMotorRightIn2);
    output.pull_down_en = GPIO_PULLDOWN_DISABLE;
    output.pull_up_en = GPIO_PULLUP_DISABLE;
    if (gpio_config(&output) != ESP_OK) {
        ESP_LOGE(kTag, "Failed to configure motor direction GPIOs");
        return false;
    }

    gpio_set_level(kMotorLeftIn1, 0);
    gpio_set_level(kMotorLeftIn2, 0);
    gpio_set_level(kMotorRightIn1, 0);
    gpio_set_level(kMotorRightIn2, 0);

    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_8_BIT;
    timer.timer_num = LEDC_TIMER_1;
    timer.freq_hz = kPwmFrequencyHz;
    timer.clk_cfg = LEDC_AUTO_CLK;
    if (ledc_timer_config(&timer) != ESP_OK) {
        ESP_LOGE(kTag, "Failed to configure motor PWM timer");
        return false;
    }

    for (int i = 0; i < 2; ++i) {
        ledc_channel_config_t channel = {};
        channel.gpio_num = i == 0 ? kMotorLeftEnable : kMotorRightEnable;
        channel.speed_mode = LEDC_LOW_SPEED_MODE;
        channel.channel = i == 0 ? LEDC_CHANNEL_2 : LEDC_CHANNEL_3;
        channel.timer_sel = LEDC_TIMER_1;
        channel.duty = 0;
        channel.hpoint = 0;
        if (ledc_channel_config(&channel) != ESP_OK) {
            ESP_LOGE(kTag, "Failed to configure motor PWM channel %d", i);
            return false;
        }
    }

    ApplyStop();
    return true;
}

}  // namespace duybot
