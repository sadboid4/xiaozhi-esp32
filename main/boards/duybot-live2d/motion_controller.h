#pragma once

#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace duybot {

enum class MotionDirection : uint8_t {
    kStop = 0,
    kForward = 1,
    kBackward = 2,
    kLeft = 3,
    kRight = 4,
};

enum class MotionSource : uint8_t {
    kIdle = 20,
    kEmotion = 20,
    kAutonomous = 40,
    kMcp = 60,
    kManual = 80,
    kSafety = 100,
};

struct MotionCommand {
    MotionDirection direction = MotionDirection::kStop;
    MotionSource source = MotionSource::kIdle;
    uint8_t speed_pct = 0;
    uint32_t duration_ms = 0;
};

class MotionController {
public:
    static MotionController& GetInstance();

    bool Initialize();
    bool Submit(MotionSource source, MotionDirection direction, int speed_pct, uint32_t duration_ms);
    bool Stop(MotionSource source);
    void EmergencyStop();

    MotionDirection GetDirection() const;
    MotionSource GetActiveSource() const;
    uint8_t GetActivePriority() const;
    bool IsMoving() const;
    bool IsReady() const;
    uint32_t GetSubmittedCount() const;
    uint32_t GetAppliedCount() const;
    uint32_t GetDroppedCount() const;
    MotionDirection GetLastAppliedDirection() const;
    uint8_t GetLastAppliedSpeed() const;

    static MotionDirection ParseDirection(const char* direction);
    static const char* DirectionName(MotionDirection direction);

private:
    MotionController() = default;
    MotionController(const MotionController&) = delete;
    MotionController& operator=(const MotionController&) = delete;

    static void TaskEntry(void* arg);
    void Run();
    void ProcessCommand(const MotionCommand& command, int64_t now_us);
    void ApplyDirection(MotionDirection direction, uint8_t speed_pct);
    void ApplyStop();
    bool InitializeHardware();

    QueueHandle_t queue_ = nullptr;
    TaskHandle_t task_ = nullptr;
    volatile MotionDirection direction_ = MotionDirection::kStop;
    volatile MotionSource active_source_ = MotionSource::kIdle;
    volatile uint8_t active_priority_ = 0;
    volatile bool emergency_stop_requested_ = false;
    volatile uint32_t submitted_count_ = 0;
    volatile uint32_t applied_count_ = 0;
    volatile uint32_t dropped_count_ = 0;
    volatile MotionDirection last_applied_direction_ = MotionDirection::kStop;
    volatile uint8_t last_applied_speed_ = 0;
    int64_t deadline_us_ = 0;
};

}  // namespace duybot
