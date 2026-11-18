#include "avatar_stream_client.h"
#include "config.h"
#include "duybot_display.h"
#include "duybot_pins.h"
#include "motion_controller.h"

#include "application.h"
#include "button.h"
#include "codecs/no_audio_codec.h"
#include "wifi_board.h"
#include "mcp_server.h"
#include "assets/lang_config.h"

#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <esp_http_server.h>
#include <esp_netif.h>
#include <esp_random.h>
#include <esp_system.h>

#include <freertos/semphr.h>

#include <algorithm>
#include <array>
#include <cstring>

#define TAG "DuyBot"

using duybot::MotionController;
using duybot::MotionDirection;
using duybot::MotionSource;

// ----- Hệ phản xạ tự thân (vệ sĩ chống rơi bàn + né vật cản) -----
static volatile bool g_guard_on = true;     // bảo vệ luôn bật mặc định
static volatile int  g_edge_safe = -1;      // mức dò line khi đứng trên bàn (tự học lần đầu)
static volatile int64_t g_explore_until = 0;
static volatile bool g_hazard_handling = false;
static Display* g_display = nullptr;

// ----- MOOD ENGINE: tâm trạng nền 0-100, trôi về 50 -----
static volatile int g_mood = 60;
static volatile int64_t g_last_activity = 0;   // lần tương tác gần nhất (chạm/lệnh/hội thoại)
static void MoodBump(int delta) {
    int m = g_mood + delta;
    if (m > 100) m = 100;
    if (m < 0) m = 0;
    g_mood = m;
    g_last_activity = esp_timer_get_time();
}

static volatile int g_speed = 80;         // tốc độ chạy thẳng mặc định (%)

enum class SelfCheckState : uint8_t {
    kPending,
    kRunning,
    kPassed,
    kDegraded,
};

enum class AgentState : uint8_t {
    kBooting,
    kDegraded,
    kIdle,
    kEngaged,
    kSocial,
    kCurious,
    kExploring,
    kAlert,
    kMotorTest,
    kActing,
    kGoal,
};

enum class EmoteMotionPattern : uintptr_t {
    kNone = 0,
    kHappy,
    kSurprised,
    kAngry,
    kSad,
    kConfident,
    kThinking,
    kRest,
};

enum class BehaviorState : uint8_t {
    kIdle,
    kFaceOnly,
    kRunning,
    kCompleted,
    kCancelled,
    kBusy,
};

enum class TouchGesture : uint8_t {
    kNone,
    kTap,
    kDoubleTap,
    kLongPress,
};

enum class ControlMode : uint8_t {
    kManual,
    kSmart,
};

enum class AgentGoal : uint8_t {
    kNone,
    kFollow,
    kApproach,
    kInspect,
};

enum class GoalState : uint8_t {
    kIdle,
    kRunning,
    kCompleted,
    kTargetLost,
    kCancelled,
    kBlocked,
};

static volatile SelfCheckState g_self_check_state = SelfCheckState::kPending;
static volatile AgentState g_agent_state = AgentState::kBooting;
static volatile bool g_agent_enabled = true;
static volatile bool g_autonomy_armed = false;
static volatile bool g_ultrasonic_healthy = false;
static volatile bool g_motion_ready = false;
static volatile bool g_motor_test_running = false;
static volatile bool g_motor_test_cancel = false;
static volatile int g_last_distance_cm = -1;
static volatile int g_ultrasonic_success_streak = 0;
static volatile int g_ultrasonic_failure_streak = 0;
static volatile int64_t g_last_sensor_sample_us = 0;
static SemaphoreHandle_t g_ping_mutex = nullptr;
static esp_reset_reason_t g_boot_reset_reason = ESP_RST_UNKNOWN;
static volatile EmoteMotionPattern g_behavior_pattern = EmoteMotionPattern::kNone;
static volatile BehaviorState g_behavior_state = BehaviorState::kIdle;
static volatile bool g_behavior_running = false;
static volatile bool g_behavior_body_executed = false;
static volatile uint32_t g_behavior_sequence = 0;
static volatile uint32_t g_behavior_generation = 0;
static volatile TouchGesture g_last_touch_gesture = TouchGesture::kNone;
static volatile uint32_t g_touch_event_count = 0;
static volatile int64_t g_last_touch_event_us = 0;
static volatile ControlMode g_control_mode = ControlMode::kManual;
static volatile AgentGoal g_agent_goal = AgentGoal::kNone;
static volatile GoalState g_goal_state = GoalState::kIdle;
static volatile bool g_goal_running = false;
static volatile uint32_t g_goal_generation = 0;
static volatile int g_goal_last_distance_cm = -1;
static volatile int g_goal_left_distance_cm = -1;
static volatile int g_goal_right_distance_cm = -1;
static volatile int64_t g_goal_deadline_us = 0;

static bool RequestSynchronizedBehavior(const char* behavior);
static void CancelSynchronizedBehavior();
static void SetControlMode(ControlMode mode);
static bool RequestAgentGoal(const char* goal, int seconds);
static void CancelAgentGoal();

static const char* ResetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON: return "power_on";
        case ESP_RST_EXT: return "external";
        case ESP_RST_SW: return "software";
        case ESP_RST_PANIC: return "panic";
        case ESP_RST_INT_WDT: return "interrupt_watchdog";
        case ESP_RST_TASK_WDT: return "task_watchdog";
        case ESP_RST_WDT: return "watchdog";
        case ESP_RST_DEEPSLEEP: return "deep_sleep";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO: return "sdio";
        case ESP_RST_USB: return "usb";
        case ESP_RST_JTAG: return "jtag";
        case ESP_RST_EFUSE: return "efuse";
        case ESP_RST_PWR_GLITCH: return "power_glitch";
        case ESP_RST_CPU_LOCKUP: return "cpu_lockup";
        default: return "unknown";
    }
}

static bool PowerRecoveryReady() {
    // A brownout means the 3.3 V rail was not trustworthy. Require one minute of
    // continuous uptime before autonomous movement may be armed again.
    return (g_boot_reset_reason != ESP_RST_BROWNOUT &&
            g_boot_reset_reason != ESP_RST_PWR_GLITCH) ||
           esp_timer_get_time() >= 60000000LL;
}

static const char* SelfCheckStateName(SelfCheckState state) {
    switch (state) {
        case SelfCheckState::kPending: return "pending";
        case SelfCheckState::kRunning: return "running";
        case SelfCheckState::kPassed: return "passed";
        case SelfCheckState::kDegraded: return "degraded";
    }
    return "unknown";
}

static const char* AgentStateName(AgentState state) {
    switch (state) {
        case AgentState::kBooting: return "booting";
        case AgentState::kDegraded: return "degraded";
        case AgentState::kIdle: return "idle";
        case AgentState::kEngaged: return "engaged";
        case AgentState::kSocial: return "social";
        case AgentState::kCurious: return "curious";
        case AgentState::kExploring: return "exploring";
        case AgentState::kAlert: return "alert";
        case AgentState::kMotorTest: return "motor_test";
        case AgentState::kActing: return "acting";
        case AgentState::kGoal: return "goal_driven";
    }
    return "unknown";
}

static const char* BehaviorPatternName(EmoteMotionPattern pattern) {
    switch (pattern) {
        case EmoteMotionPattern::kHappy: return "happy";
        case EmoteMotionPattern::kSurprised: return "startled";
        case EmoteMotionPattern::kAngry: return "angry";
        case EmoteMotionPattern::kSad: return "sad";
        case EmoteMotionPattern::kConfident: return "confident";
        case EmoteMotionPattern::kThinking: return "curious";
        case EmoteMotionPattern::kRest: return "rest";
        case EmoteMotionPattern::kNone: return "none";
    }
    return "none";
}

static const char* BehaviorStateName(BehaviorState state) {
    switch (state) {
        case BehaviorState::kIdle: return "idle";
        case BehaviorState::kFaceOnly: return "face_only_autonomy_locked";
        case BehaviorState::kRunning: return "running";
        case BehaviorState::kCompleted: return "completed";
        case BehaviorState::kCancelled: return "cancelled";
        case BehaviorState::kBusy: return "busy";
    }
    return "idle";
}

static const char* TouchGestureName(TouchGesture gesture) {
    switch (gesture) {
        case TouchGesture::kTap: return "tap";
        case TouchGesture::kDoubleTap: return "double_tap";
        case TouchGesture::kLongPress: return "long_press";
        case TouchGesture::kNone: return "none";
    }
    return "none";
}

static const char* ControlModeName(ControlMode mode) {
    return mode == ControlMode::kSmart ? "smart" : "manual";
}

static const char* AgentGoalName(AgentGoal goal) {
    switch (goal) {
        case AgentGoal::kFollow: return "follow";
        case AgentGoal::kApproach: return "approach";
        case AgentGoal::kInspect: return "inspect";
        case AgentGoal::kNone: return "none";
    }
    return "none";
}

static const char* GoalStateName(GoalState state) {
    switch (state) {
        case GoalState::kIdle: return "idle";
        case GoalState::kRunning: return "running";
        case GoalState::kCompleted: return "completed";
        case GoalState::kTargetLost: return "target_lost";
        case GoalState::kCancelled: return "cancelled";
        case GoalState::kBlocked: return "blocked";
    }
    return "idle";
}

static const char* SelfCheckReason() {
    if (g_self_check_state == SelfCheckState::kPending) return "not_run";
    if (g_self_check_state == SelfCheckState::kRunning) return "checking";
    if (!g_motion_ready) return "motion_init_failed";
    if (!g_ultrasonic_healthy) {
        return gpio_get_level(duybot::kUltrasonicEcho) == 0
                   ? "hy_srf05_echo_stuck_low_check_out_floating"
                   : "hy_srf05_echo_stuck_high";
    }
    return "ok";
}

static bool CanArmAutonomy() {
    return g_control_mode == ControlMode::kSmart &&
           g_self_check_state == SelfCheckState::kPassed &&
           g_motion_ready && g_ultrasonic_healthy && g_guard_on && g_edge_safe >= 0 &&
           !g_motor_test_running && PowerRecoveryReady();
}

static bool AutonomousMotionAllowed() {
    return g_agent_enabled && g_autonomy_armed && CanArmAutonomy();
}

static int PingCmRaw() {
    constexpr int kNoObstacleInRangeCm = 451;
    // Reject an ECHO line stuck HIGH before emitting a new trigger pulse.
    int64_t idle_timeout = esp_timer_get_time() + 3000;
    while (gpio_get_level(duybot::kUltrasonicEcho) == 1) {
        if (esp_timer_get_time() >= idle_timeout) return -1;
    }

    gpio_set_level(duybot::kUltrasonicTrigger, 0);
    esp_rom_delay_us(4);
    gpio_set_level(duybot::kUltrasonicTrigger, 1);
    esp_rom_delay_us(12);
    gpio_set_level(duybot::kUltrasonicTrigger, 0);

    // HY-SRF05 dual-pin mode: OUT/MODE must remain floating. The datasheet
    // specifies up to 25 ms ECHO pulses and a 30 ms timeout.
    int64_t timeout = esp_timer_get_time() + 30000;
    while (gpio_get_level(duybot::kUltrasonicEcho) == 0) {
        if (esp_timer_get_time() >= timeout) return -1;
    }
    int64_t t0 = esp_timer_get_time();
    // A HY-SRF05 holds ECHO high until its ~30 ms no-return timeout. Allow
    // margin so that this is classified as "clear" rather than hardware loss.
    timeout = t0 + 35000;
    while (gpio_get_level(duybot::kUltrasonicEcho) == 1) {
        if (esp_timer_get_time() >= timeout) return kNoObstacleInRangeCm;
    }
    const int64_t pulse_us = esp_timer_get_time() - t0;
    if (pulse_us >= 29000) return kNoObstacleInRangeCm;
    const int distance_cm = static_cast<int>((pulse_us + 29) / 58);
    return distance_cm >= 2 && distance_cm <= 450 ? distance_cm
                                                   : kNoObstacleInRangeCm;
}

static void RecordUltrasonicSample(int distance_cm) {
    g_last_sensor_sample_us = esp_timer_get_time();
    if (distance_cm > 0) {
        g_last_distance_cm = distance_cm;
        g_ultrasonic_failure_streak = 0;
        g_ultrasonic_success_streak = g_ultrasonic_success_streak + 1;
        if (g_ultrasonic_success_streak >= 2) g_ultrasonic_healthy = true;
    } else {
        g_ultrasonic_success_streak = 0;
        g_ultrasonic_failure_streak = g_ultrasonic_failure_streak + 1;
        if (g_ultrasonic_failure_streak >= 3) {
            g_ultrasonic_healthy = false;
            g_last_distance_cm = -1;
            g_autonomy_armed = false;
            if (g_self_check_state != SelfCheckState::kRunning) {
                g_self_check_state = SelfCheckState::kDegraded;
            }
            CancelAgentGoal();
            CancelSynchronizedBehavior();
        }
    }
}

static int PingCm() {
    if (g_ping_mutex != nullptr &&
        xSemaphoreTake(g_ping_mutex, pdMS_TO_TICKS(60)) != pdTRUE) {
        return -1;
    }
    const int distance_cm = PingCmRaw();
    if (g_ping_mutex != nullptr) xSemaphoreGive(g_ping_mutex);
    RecordUltrasonicSample(distance_cm);
    return distance_cm;
}

static void SelfCheckTask(void* arg) {
    (void)arg;
    g_self_check_state = SelfCheckState::kRunning;
    g_agent_state = AgentState::kBooting;
    g_autonomy_armed = false;
    CancelAgentGoal();
    CancelSynchronizedBehavior();
    g_explore_until = 0;
    MotionController::GetInstance().EmergencyStop();
    vTaskDelay(pdMS_TO_TICKS(80));

    g_edge_safe = gpio_get_level(duybot::kLineSensor);
    g_ultrasonic_success_streak = 0;
    g_ultrasonic_failure_streak = 0;
    g_ultrasonic_healthy = false;
    g_last_distance_cm = -1;

    std::array<int, 5> valid_distances{};
    int valid_count = 0;
    for (int i = 0; i < 5; ++i) {
        const int distance_cm = PingCm();
        if (distance_cm > 0) valid_distances[valid_count++] = distance_cm;
        vTaskDelay(pdMS_TO_TICKS(70));
    }

    if (valid_count >= 2 && g_motion_ready) {
        for (int i = 0; i < valid_count; ++i) {
            for (int j = i + 1; j < valid_count; ++j) {
                if (valid_distances[j] < valid_distances[i]) {
                    const int temp = valid_distances[i];
                    valid_distances[i] = valid_distances[j];
                    valid_distances[j] = temp;
                }
            }
        }
        g_last_distance_cm = valid_distances[valid_count / 2];
        g_ultrasonic_healthy = true;
        g_self_check_state = SelfCheckState::kPassed;
        g_agent_state = AgentState::kIdle;
        ESP_LOGI(TAG, "Self-check passed: distance=%dcm line=%d touch=%d heap=%u",
                 g_last_distance_cm, g_edge_safe,
                 gpio_get_level(duybot::kTouchSensor),
                 static_cast<unsigned>(esp_get_free_heap_size()));
    } else {
        g_ultrasonic_healthy = false;
        g_self_check_state = SelfCheckState::kDegraded;
        g_agent_state = AgentState::kDegraded;
        ESP_LOGW(TAG,
                 "Self-check degraded: ultrasonic=%d/%u motion=%d echo_idle=%d; autonomy locked",
                 valid_count, static_cast<unsigned>(valid_distances.size()),
                 g_motion_ready, gpio_get_level(duybot::kUltrasonicEcho));
    }
    vTaskDelete(nullptr);
}

static bool RequestSelfCheck() {
    if (g_self_check_state == SelfCheckState::kRunning || g_motor_test_running) return false;
    return xTaskCreate(SelfCheckTask, "self_check", 3072, nullptr, 5, nullptr) == pdPASS;
}

static void MotorTestTask(void* arg) {
    (void)arg;
    auto& motion = MotionController::GetInstance();
    g_motor_test_cancel = false;
    g_motor_test_running = true;
    g_autonomy_armed = false;
    g_explore_until = 0;
    g_hazard_handling = true;
    g_agent_state = AgentState::kMotorTest;
    CancelAgentGoal();
    CancelSynchronizedBehavior();
    motion.EmergencyStop();
    vTaskDelay(pdMS_TO_TICKS(100));

    constexpr std::array<MotionDirection, 4> sequence = {
        MotionDirection::kForward,
        MotionDirection::kBackward,
        MotionDirection::kLeft,
        MotionDirection::kRight,
    };
    for (MotionDirection direction : sequence) {
        if (g_motor_test_cancel) break;
        ESP_LOGI(TAG, "Motor self-test: %s", MotionController::DirectionName(direction));
        motion.Submit(MotionSource::kSafety, direction, 45, 220);
        vTaskDelay(pdMS_TO_TICKS(240));
        motion.Stop(MotionSource::kSafety);
        vTaskDelay(pdMS_TO_TICKS(260));
    }

    motion.EmergencyStop();
    g_hazard_handling = false;
    g_motor_test_running = false;
    g_agent_state = g_ultrasonic_healthy ? AgentState::kIdle : AgentState::kDegraded;
    ESP_LOGI(TAG, "Motor self-test %s", g_motor_test_cancel ? "cancelled" : "finished");
    vTaskDelete(nullptr);
}

static bool RequestMotorTest() {
    if (g_motor_test_running || !g_motion_ready ||
        g_self_check_state == SelfCheckState::kRunning || esp_get_free_heap_size() < 5000) {
        return false;
    }
    g_motor_test_running = true;
    if (xTaskCreate(MotorTestTask, "motor_test", 3072, nullptr, 6, nullptr) != pdPASS) {
        g_motor_test_running = false;
        return false;
    }
    return true;
}

static void AgentSupervisorStep() {
    static int64_t next_sensor_check_us = 0;
    static int64_t next_curiosity_us = esp_timer_get_time() + 120000000LL;
    const int64_t now = esp_timer_get_time();

    if (now >= next_sensor_check_us && !MotionController::GetInstance().IsMoving() &&
        !g_motor_test_running && g_self_check_state != SelfCheckState::kRunning) {
        next_sensor_check_us = now + 5000000LL;
        PingCm();
        if (g_self_check_state == SelfCheckState::kDegraded &&
            g_ultrasonic_healthy && g_motion_ready) {
            g_self_check_state = SelfCheckState::kPassed;
            ESP_LOGI(TAG, "Ultrasonic recovered; self-check promoted to passed");
        }
    }

    if (g_motor_test_running) {
        g_agent_state = AgentState::kMotorTest;
        return;
    }
    if (g_hazard_handling) {
        g_agent_state = AgentState::kAlert;
        return;
    }
    if (g_behavior_running) {
        g_agent_state = AgentState::kActing;
        return;
    }
    if (g_goal_running) {
        g_agent_state = AgentState::kGoal;
        return;
    }
    if (g_self_check_state == SelfCheckState::kPending ||
        g_self_check_state == SelfCheckState::kRunning) {
        g_agent_state = AgentState::kBooting;
        return;
    }
    if (!g_ultrasonic_healthy || !g_motion_ready) {
        g_agent_state = AgentState::kDegraded;
        return;
    }
    if (gpio_get_level(duybot::kTouchSensor) == 1) {
        g_agent_state = AgentState::kSocial;
        return;
    }
    if (Application::GetInstance().GetDeviceState() != kDeviceStateIdle) {
        g_agent_state = AgentState::kEngaged;
        return;
    }
    if (now < g_explore_until) {
        g_agent_state = AgentState::kExploring;
        return;
    }

    g_agent_state = g_mood >= 65 ? AgentState::kCurious : AgentState::kIdle;
    if (AutonomousMotionAllowed() && now >= next_curiosity_us &&
        now - g_last_activity >= 120000000LL) {
        // Short bounded exploration. GuardTask still owns obstacle and edge reactions.
        g_explore_until = now + 6000000LL;
        next_curiosity_us = now + 180000000LL + (esp_random() % 180000000ULL);
        ESP_LOGI(TAG, "Agent curiosity cycle started (6s)");
    }
}

// ===== MANUAL MODE: web dieu khien tren dien thoai =====

static const char MANUAL_HTML[] = R"HTML(<!doctype html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>DuyBot Manual</title><style>
body{font-family:-apple-system,sans-serif;background:#0f172a;color:#e2e8f0;margin:0;padding:16px;text-align:center;user-select:none;-webkit-user-select:none}
h2{margin:8px 0}
.grid{display:grid;grid-template-columns:repeat(3,84px);grid-gap:10px;justify-content:center;margin:18px 0}
button{font-size:30px;padding:18px 0;border:none;border-radius:14px;background:#1e293b;color:#e2e8f0;touch-action:none}
button:active{background:#2563eb}
.stat{display:flex;justify-content:center;gap:14px;flex-wrap:wrap;margin:10px 0}
.card{background:#1e293b;border-radius:10px;padding:10px 16px;min-width:80px}
.v{font-size:22px;font-weight:600}.l{font-size:11px;color:#94a3b8}
.row{margin:10px}
.tgl{font-size:15px;padding:10px 18px}
</style></head><body>
<h2>🤖 DuyBot Manual</h2>
<div class="stat">
<div class="card"><div class="v" id="d">--</div><div class="l">KHOẢNG CÁCH (cm)</div></div>
<div class="card"><div class="v" id="ln">--</div><div class="l">DÒ LINE</div></div>
<div class="card"><div class="v" id="tc">--</div><div class="l">CHẠM</div></div>
<div class="card"><div class="v" id="gd">--</div><div class="l">VỆ SĨ</div></div>
<div class="card"><div class="v" id="ag">--</div><div class="l">AGENT</div></div>
<div class="card"><div class="v" id="sc">--</div><div class="l">SELF-CHECK</div></div>
<div class="card"><div class="v" id="pw">--</div><div class="l">NGUỒN / RESET</div></div>
<div class="card"><div class="v" id="bh">--</div><div class="l">BODY + MIND</div></div>
<div class="card"><div class="v" id="md">--</div><div class="l">CONTROL MODE</div></div>
<div class="card"><div class="v" id="gl">--</div><div class="l">SMART GOAL</div></div>
</div>
<div class="grid">
<span></span><button id="f">▲</button><span></span>
<button id="l">◀</button><button id="s">■</button><button id="r">▶</button>
<span></span><button id="b">▼</button><span></span>
</div>
<div class="row">
<button class="tgl" onclick="setMode('manual')">🎮 Manual Play</button>
<button class="tgl" onclick="setMode('smart')">🧠 Smart Agent</button>
</div>
<div class="row">
<button class="tgl" onclick="fetch('/api/guard?on='+(gON?0:1)).then(_=>{})">Bật/Tắt vệ sĩ</button>
<button class="tgl" onclick="fetch('/api/explore?s=15')">Đi dạo 15s</button>
</div>
<div class="row">
<button class="tgl" onclick="fetch('/api/selftest?run=1')">Chạy self-check</button>
<button class="tgl" onclick="armAgent()">Arm agent</button>
<button class="tgl" onclick="motorTest()">Test motor</button>
</div>
<div class="row">
<button class="tgl" onclick="behavior('greet')">Chào</button>
<button class="tgl" onclick="behavior('curious')">Tò mò</button>
<button class="tgl" onclick="behavior('celebrate')">Ăn mừng</button>
</div>
<div class="row">
<button class="tgl" onclick="goal('follow',20)">Theo tay 20s</button>
<button class="tgl" onclick="goal('approach',12)">Tiến lại gần</button>
<button class="tgl" onclick="goal('inspect',8)">Quan sát</button>
<button class="tgl" onclick="goal('stop',0)">Dừng goal</button>
</div>
<div class="row" style="max-width:280px;margin:6px auto">
⚡ Tốc độ: <span id="spv">80</span>%
<input type="range" min="20" max="100" value="80" style="width:100%"
 oninput="document.getElementById('spv').textContent=this.value"
 onchange="fetch('/api/speed?v='+this.value)">
</div>
<script>
let timer=null,gON=true;
const map={f:'forward',b:'backward',l:'left',r:'right'};
function hold(d){send(d);clearInterval(timer);timer=setInterval(()=>send(d),250);}
function stop(){clearInterval(timer);timer=null;send('stop');}
function send(d){fetch('/api/move?dir='+d).catch(()=>{});}
async function armAgent(){
  if(!confirm('Chỉ arm khi robot ở dưới sàn an toàn và cảm biến đã PASS. Tiếp tục?'))return;
  const r=await fetch('/api/agent?enabled=1&arm=1');alert(await r.text());
}
async function motorTest(){
  if(!confirm('NHẤC HAI BÁNH KHỎI MẶT BÀN. Motor sẽ lần lượt chạy 4 hướng. Tiếp tục?'))return;
  const r=await fetch('/api/motor-test?confirm=LIFTED');alert(await r.text());
}
async function behavior(name){
  const r=await fetch('/api/behavior?name='+name);alert(await r.text());
}
async function setMode(mode){
  if(!confirm('Chuyển sang '+mode+' sẽ STOP và disarm robot. Tiếp tục?'))return;
  const r=await fetch('/api/mode?value='+mode);alert(await r.text());
}
async function goal(name,seconds){
  const r=await fetch('/api/goal?name='+name+'&seconds='+seconds);alert(await r.text());
}
for(const k in map){
  const b=document.getElementById(k);
  b.addEventListener('touchstart',e=>{e.preventDefault();hold(map[k]);});
  b.addEventListener('touchend',e=>{e.preventDefault();stop();});
  b.addEventListener('mousedown',()=>hold(map[k]));
  b.addEventListener('mouseup',stop);
}
document.getElementById('s').addEventListener('click',stop);
setInterval(async()=>{try{
  const j=await(await fetch('/api/status')).json();
  document.getElementById('d').textContent=j.dist<0?'ERR':(j.dist>450?'CLEAR':j.dist);
  document.getElementById('ln').textContent=j.line?'VẠCH':'BÀN';
  document.getElementById('tc').textContent=j.touch?'CÓ':'—';
  gON=j.guard;document.getElementById('gd').textContent=j.guard?'ON':'OFF';
  document.getElementById('ag').textContent=j.agent+(j.autonomy_armed?' ✓':' 🔒');
  document.getElementById('sc').textContent=j.self_check;
  document.getElementById('pw').textContent=j.power_ready?'OK':j.reset_reason;
  document.getElementById('bh').textContent=j.behavior+' / '+j.behavior_state;
  document.getElementById('md').textContent=j.control_mode;
  document.getElementById('gl').textContent=j.goal+' / '+j.goal_state;
}catch(e){}},700);
</script></body></html>)HTML";

static esp_err_t WebRoot(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, MANUAL_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t WebMove(httpd_req_t* req) {
    char q[64] = {0}, dir[16] = {0};
    httpd_req_get_url_query_str(req, q, sizeof(q));
    httpd_query_key_value(q, "dir", dir, sizeof(dir));
    auto& motion = MotionController::GetInstance();
    const MotionDirection direction = MotionController::ParseDirection(dir);
    if (direction == MotionDirection::kStop) {
        g_motor_test_cancel = true;
        CancelAgentGoal();
        CancelSynchronizedBehavior();
        motion.EmergencyStop();
    } else if (g_control_mode != ControlMode::kManual) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "direction buttons require manual mode; STOP always works");
    } else if (g_motor_test_running) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "motor test is running");
    } else {
        // The browser refreshes this command every 250 ms. If keepalive is lost,
        // the controller watchdog stops the chassis after 700 ms.
        motion.Submit(MotionSource::kManual, direction, g_speed, 700);
    }
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t WebStatus(httpd_req_t* req) {
    auto& motion = MotionController::GetInstance();
    const int sensor_age_ms = g_last_sensor_sample_us == 0
                                  ? -1
                                  : static_cast<int>(std::min<int64_t>(
                                        (esp_timer_get_time() - g_last_sensor_sample_us) / 1000,
                                        2147483647));
    const int uptime_ms = static_cast<int>(std::min<int64_t>(
        esp_timer_get_time() / 1000, 2147483647));
    const int touch_age_ms = g_last_touch_event_us == 0
                                 ? -1
                                 : static_cast<int>(std::min<int64_t>(
                                       (esp_timer_get_time() - g_last_touch_event_us) / 1000,
                                       2147483647));
    char buf[1450];
    snprintf(buf, sizeof(buf),
        "{\"dist\":%d,\"ultrasonic_ok\":%s,\"line\":%d,\"touch\":%d,"
        "\"guard\":%s,\"moving\":%d,\"motion_ready\":%s,"
        "\"self_check\":\"%s\",\"self_check_reason\":\"%s\","
        "\"agent\":\"%s\",\"agent_enabled\":%s,"
        "\"autonomy_armed\":%s,\"motor_test\":%s,\"echo\":%d,"
        "\"ultrasonic_success_streak\":%d,\"ultrasonic_failure_streak\":%d,"
        "\"sensor_age_ms\":%d,\"uptime_ms\":%d,"
        "\"reset_reason\":\"%s\",\"power_ready\":%s,"
        "\"motion_submitted\":%u,\"motion_applied\":%u,\"motion_dropped\":%u,"
        "\"last_motion\":\"%s\",\"last_motion_speed\":%u,"
        "\"behavior\":\"%s\",\"behavior_state\":\"%s\","
        "\"behavior_sequence\":%u,\"behavior_body_executed\":%s,"
        "\"touch_gesture\":\"%s\",\"touch_event_count\":%u,\"touch_age_ms\":%d,"
        "\"control_mode\":\"%s\","
        "\"goal\":\"%s\",\"goal_state\":\"%s\",\"goal_running\":%s,"
        "\"goal_distance_cm\":%d,\"goal_left_cm\":%d,\"goal_right_cm\":%d,"
        "\"heap_free\":%u,\"heap_min\":%u}",
        g_last_distance_cm, g_ultrasonic_healthy ? "true" : "false",
        gpio_get_level(duybot::kLineSensor), gpio_get_level(duybot::kTouchSensor),
        g_guard_on ? "true" : "false", static_cast<int>(motion.GetDirection()),
        g_motion_ready ? "true" : "false", SelfCheckStateName(g_self_check_state),
        SelfCheckReason(), AgentStateName(g_agent_state), g_agent_enabled ? "true" : "false",
        g_autonomy_armed ? "true" : "false", g_motor_test_running ? "true" : "false",
        gpio_get_level(duybot::kUltrasonicEcho), g_ultrasonic_success_streak,
        g_ultrasonic_failure_streak, sensor_age_ms, uptime_ms,
        ResetReasonName(g_boot_reset_reason), PowerRecoveryReady() ? "true" : "false",
        static_cast<unsigned>(motion.GetSubmittedCount()),
        static_cast<unsigned>(motion.GetAppliedCount()),
        static_cast<unsigned>(motion.GetDroppedCount()),
        MotionController::DirectionName(motion.GetLastAppliedDirection()),
        static_cast<unsigned>(motion.GetLastAppliedSpeed()),
        BehaviorPatternName(g_behavior_pattern), BehaviorStateName(g_behavior_state),
        static_cast<unsigned>(g_behavior_sequence),
        g_behavior_body_executed ? "true" : "false",
        TouchGestureName(g_last_touch_gesture),
        static_cast<unsigned>(g_touch_event_count), touch_age_ms,
        ControlModeName(g_control_mode),
        AgentGoalName(g_agent_goal), GoalStateName(g_goal_state),
        g_goal_running ? "true" : "false", g_goal_last_distance_cm,
        g_goal_left_distance_cm, g_goal_right_distance_cm,
        static_cast<unsigned>(esp_get_free_heap_size()),
        static_cast<unsigned>(esp_get_minimum_free_heap_size()));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t WebSpeed(httpd_req_t* req) {
    char q[32] = {0}, v[8] = {0};
    httpd_req_get_url_query_str(req, q, sizeof(q));
    httpd_query_key_value(q, "v", v, sizeof(v));
    int s = atoi(v);
    if (s >= 20 && s <= 100) g_speed = s;
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t WebGuard(httpd_req_t* req) {
    char q[32] = {0}, on[4] = {0};
    httpd_req_get_url_query_str(req, q, sizeof(q));
    httpd_query_key_value(q, "on", on, sizeof(on));
    g_guard_on = (on[0] != '0');
    if (!g_guard_on) {
        g_autonomy_armed = false;
        g_explore_until = 0;
        CancelAgentGoal();
        CancelSynchronizedBehavior();
        MotionController::GetInstance().Stop(MotionSource::kAutonomous);
    }
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t WebExplore(httpd_req_t* req) {
    char q[32] = {0}, s[8] = {0};
    httpd_req_get_url_query_str(req, q, sizeof(q));
    httpd_query_key_value(q, "s", s, sizeof(s));
    int sec = atoi(s); if (sec < 5) sec = 15; if (sec > 60) sec = 60;
    if (!AutonomousMotionAllowed()) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "autonomy locked: run self-check and arm agent first");
    }
    g_explore_until = esp_timer_get_time() + (int64_t)sec * 1000000;
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t WebSelfTest(httpd_req_t* req) {
    char q[32] = {0}, run[4] = {0};
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        httpd_query_key_value(q, "run", run, sizeof(run));
    }
    if (run[0] == '1' && !RequestSelfCheck()) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "self-check or motor test already running");
    }
    return WebStatus(req);
}

static esp_err_t WebAgent(httpd_req_t* req) {
    char q[80] = {0}, enabled[4] = {0}, arm[4] = {0};
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        httpd_query_key_value(q, "enabled", enabled, sizeof(enabled));
        httpd_query_key_value(q, "arm", arm, sizeof(arm));
    }
    if (enabled[0] != '\0') g_agent_enabled = enabled[0] != '0';
    if (!g_agent_enabled || arm[0] == '0') {
        g_autonomy_armed = false;
        g_explore_until = 0;
        CancelAgentGoal();
        CancelSynchronizedBehavior();
        MotionController::GetInstance().Stop(MotionSource::kAutonomous);
    }
    if (arm[0] == '1') {
        if (!CanArmAutonomy()) {
            httpd_resp_set_status(req, "409 Conflict");
            return httpd_resp_sendstr(req,
                "cannot arm: switch to smart mode and require self-check, ultrasonic, guard and motion ready");
        }
        g_agent_enabled = true;
        g_autonomy_armed = true;
    }
    return WebStatus(req);
}

static esp_err_t WebMotorTest(httpd_req_t* req) {
    char q[64] = {0}, confirm[16] = {0};
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        httpd_query_key_value(q, "confirm", confirm, sizeof(confirm));
    }
    if (std::strcmp(confirm, "LIFTED") != 0) {
        httpd_resp_set_status(req, "412 Precondition Failed");
        return httpd_resp_sendstr(req, "lift both wheels, then send confirm=LIFTED");
    }
    if (!RequestMotorTest()) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "motor test unavailable or already running");
    }
    return httpd_resp_sendstr(req, "motor test started; STOP cancels it");
}

static esp_err_t WebBehavior(httpd_req_t* req) {
    char q[64] = {0}, name[20] = {0};
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        httpd_query_key_value(q, "name", name, sizeof(name));
    }
    if (!RequestSynchronizedBehavior(name)) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req,
            "unknown behavior or coordinator busy; use greet, celebrate, curious, startled, comfort, confident or rest");
    }
    return WebStatus(req);
}

static esp_err_t WebMode(httpd_req_t* req) {
    char q[48] = {0}, value[12] = {0};
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        httpd_query_key_value(q, "value", value, sizeof(value));
    }
    if (std::strcmp(value, "manual") == 0) SetControlMode(ControlMode::kManual);
    else if (std::strcmp(value, "smart") == 0) SetControlMode(ControlMode::kSmart);
    else {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "value must be manual or smart");
    }
    return WebStatus(req);
}

static esp_err_t WebGoal(httpd_req_t* req) {
    char q[80] = {0}, name[16] = {0}, seconds_text[8] = {0};
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        httpd_query_key_value(q, "name", name, sizeof(name));
        httpd_query_key_value(q, "seconds", seconds_text, sizeof(seconds_text));
    }
    if (std::strcmp(name, "stop") == 0) {
        CancelAgentGoal();
        MotionController::GetInstance().EmergencyStop();
        return WebStatus(req);
    }
    const int seconds = atoi(seconds_text);
    if (!RequestAgentGoal(name, seconds == 0 ? 15 : seconds)) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req,
            "goal blocked: switch to smart mode, arm safely, then use follow, approach or inspect");
    }
    return WebStatus(req);
}

static void StartManualWebServer() {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.max_uri_handlers = 12;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Manual web server start failed");
        return;
    }
    static const httpd_uri_t u1 = {.uri="/", .method=HTTP_GET, .handler=WebRoot, .user_ctx=NULL};
    static const httpd_uri_t u2 = {.uri="/api/move", .method=HTTP_GET, .handler=WebMove, .user_ctx=NULL};
    static const httpd_uri_t u3 = {.uri="/api/status", .method=HTTP_GET, .handler=WebStatus, .user_ctx=NULL};
    static const httpd_uri_t u4 = {.uri="/api/guard", .method=HTTP_GET, .handler=WebGuard, .user_ctx=NULL};
    static const httpd_uri_t u5 = {.uri="/api/explore", .method=HTTP_GET, .handler=WebExplore, .user_ctx=NULL};
    static const httpd_uri_t u6 = {.uri="/api/speed", .method=HTTP_GET, .handler=WebSpeed, .user_ctx=NULL};
    static const httpd_uri_t u7 = {.uri="/api/selftest", .method=HTTP_GET, .handler=WebSelfTest, .user_ctx=NULL};
    static const httpd_uri_t u8 = {.uri="/api/agent", .method=HTTP_GET, .handler=WebAgent, .user_ctx=NULL};
    static const httpd_uri_t u9 = {.uri="/api/motor-test", .method=HTTP_GET, .handler=WebMotorTest, .user_ctx=NULL};
    static const httpd_uri_t u10 = {.uri="/api/behavior", .method=HTTP_GET, .handler=WebBehavior, .user_ctx=NULL};
    static const httpd_uri_t u11 = {.uri="/api/mode", .method=HTTP_GET, .handler=WebMode, .user_ctx=NULL};
    static const httpd_uri_t u12 = {.uri="/api/goal", .method=HTTP_GET, .handler=WebGoal, .user_ctx=NULL};
    httpd_register_uri_handler(server, &u1);
    httpd_register_uri_handler(server, &u2);
    httpd_register_uri_handler(server, &u3);
    httpd_register_uri_handler(server, &u4);
    httpd_register_uri_handler(server, &u5);
    httpd_register_uri_handler(server, &u6);
    httpd_register_uri_handler(server, &u7);
    httpd_register_uri_handler(server, &u8);
    httpd_register_uri_handler(server, &u9);
    httpd_register_uri_handler(server, &u10);
    httpd_register_uri_handler(server, &u11);
    httpd_register_uri_handler(server, &u12);
    ESP_LOGI(TAG, "Manual web server on port 80");
}

// ===== TASK VỆ SĨ: chạy nền 60ms/lần, chỉ can thiệp khi đang di chuyển =====
static void GuardTask(void* arg) {
    auto& motion = MotionController::GetInstance();
    int tick = 0; bool turn_flip = false;
    vTaskDelay(pdMS_TO_TICKS(3000));               // chờ boot xong
    g_edge_safe = gpio_get_level(duybot::kLineSensor);  // học "mặt bàn" lúc khởi động
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(60));
        tick++;
        if (!g_guard_on || g_hazard_handling) continue;
        bool exploring = AutonomousMotionAllowed() && esp_timer_get_time() < g_explore_until;
        MotionDirection current_direction = motion.GetDirection();
        bool moving_fwd = current_direction == MotionDirection::kForward;

        // 1) MÉP BÀN — ưu tiên cao nhất, check mọi lúc đang chạy tới/quay
        if (motion.IsMoving() && g_edge_safe >= 0 &&
            gpio_get_level(duybot::kLineSensor) != g_edge_safe) {
            g_hazard_handling = true;
            MoodBump(-3);
            CancelAgentGoal();
            CancelSynchronizedBehavior();
            motion.EmergencyStop();
            vTaskDelay(pdMS_TO_TICKS(20));
            motion.Submit(MotionSource::kSafety, MotionDirection::kBackward, g_speed, 280);
            vTaskDelay(pdMS_TO_TICKS(300));
            if (exploring) {                        // đổi hướng rồi đi tiếp
                motion.Submit(MotionSource::kSafety,
                              turn_flip ? MotionDirection::kLeft : MotionDirection::kRight,
                              g_speed, 350);
                turn_flip = !turn_flip;
                vTaskDelay(pdMS_TO_TICKS(370));
            } else {
                g_explore_until = 0;
                if (g_display) g_display->SetEmotion("shocked");
            }
            motion.Stop(MotionSource::kSafety);
            g_hazard_handling = false;
            continue;
        }

        // 2) VẬT CẢN — mỗi 3 tick (~180ms) khi đang tiến
        if (moving_fwd && (tick % 3 == 0)) {
            int d = PingCm();
            if (d > 0 && d < 15) {
                g_hazard_handling = true;
                CancelAgentGoal();
                CancelSynchronizedBehavior();
                motion.EmergencyStop();
                vTaskDelay(pdMS_TO_TICKS(20));
                if (exploring) {
                    motion.Submit(MotionSource::kSafety, MotionDirection::kBackward, g_speed, 180);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    motion.Submit(MotionSource::kSafety,
                                  turn_flip ? MotionDirection::kLeft : MotionDirection::kRight,
                                  g_speed, 300);
                    turn_flip = !turn_flip;
                    vTaskDelay(pdMS_TO_TICKS(320));
                } else if (g_display) {
                    g_display->SetEmotion("surprised");
                }
                motion.Stop(MotionSource::kSafety);
                g_hazard_handling = false;
                continue;
            }
        }

        // 3) ĐI DẠO — hết chướng ngại thì tự tiến tiếp
        if (exploring && !motion.IsMoving()) {
            motion.Submit(MotionSource::kAutonomous, MotionDirection::kForward, g_speed, 250);
        }
        if (!exploring &&
            motion.GetDirection() == MotionDirection::kForward &&
            motion.GetActiveSource() == MotionSource::kAutonomous) {
            motion.Stop(MotionSource::kAutonomous);  // hết giờ dạo
            g_explore_until = 0;
        }
    }
}


// ----- Chuyển động nhỏ theo cảm xúc (an toàn trên bàn: mỗi nhịp <= 250ms) -----
static volatile bool emote_busy = false;

static EmoteMotionPattern GetEmoteMotionPattern(const char* emotion) {
    if (emotion == nullptr) return EmoteMotionPattern::kNone;
    if (std::strcmp(emotion, "happy") == 0 || std::strcmp(emotion, "laughing") == 0 ||
        std::strcmp(emotion, "funny") == 0 || std::strcmp(emotion, "loving") == 0 ||
        std::strcmp(emotion, "kissy") == 0) return EmoteMotionPattern::kHappy;
    if (std::strcmp(emotion, "surprised") == 0 || std::strcmp(emotion, "shocked") == 0)
        return EmoteMotionPattern::kSurprised;
    if (std::strcmp(emotion, "angry") == 0) return EmoteMotionPattern::kAngry;
    if (std::strcmp(emotion, "sad") == 0 || std::strcmp(emotion, "crying") == 0)
        return EmoteMotionPattern::kSad;
    if (std::strcmp(emotion, "cool") == 0 || std::strcmp(emotion, "confident") == 0 ||
        std::strcmp(emotion, "winking") == 0) return EmoteMotionPattern::kConfident;
    if (std::strcmp(emotion, "thinking") == 0 || std::strcmp(emotion, "confused") == 0)
        return EmoteMotionPattern::kThinking;
    if (std::strcmp(emotion, "relaxed") == 0 || std::strcmp(emotion, "sleepy") == 0)
        return EmoteMotionPattern::kRest;
    return EmoteMotionPattern::kNone;
}

static void EmoteMotionTask(void* arg) {
    auto& motion = MotionController::GetInstance();
    const auto pattern = static_cast<EmoteMotionPattern>(reinterpret_cast<uintptr_t>(arg));
    const uint32_t generation = g_behavior_generation;
    g_behavior_running = true;
    g_behavior_state = BehaviorState::kRunning;
    g_agent_state = AgentState::kActing;
    auto pulse = [&motion, generation](MotionDirection direction, int ms) {
        if (generation != g_behavior_generation || !AutonomousMotionAllowed()) return false;
        if (!motion.Submit(MotionSource::kEmotion, direction, g_speed, ms)) return false;
        g_behavior_body_executed = true;
        vTaskDelay(pdMS_TO_TICKS(ms));
        if (generation != g_behavior_generation) return false;
        motion.Stop(MotionSource::kEmotion);
        vTaskDelay(pdMS_TO_TICKS(80));
        return true;
    };
    if (pattern == EmoteMotionPattern::kHappy) {
        pulse(MotionDirection::kLeft, 120); pulse(MotionDirection::kRight, 120);
        pulse(MotionDirection::kLeft, 120); pulse(MotionDirection::kRight, 120);  // lắc lư vui
    } else if (pattern == EmoteMotionPattern::kSurprised) {
        pulse(MotionDirection::kBackward, 200);       // giật mình lùi nhẹ
    } else if (pattern == EmoteMotionPattern::kAngry) {
        pulse(MotionDirection::kForward, 150); pulse(MotionDirection::kBackward, 150); // chồm nhẹ
    } else if (pattern == EmoteMotionPattern::kSad) {
        pulse(MotionDirection::kBackward, 250);       // thu mình
    } else if (pattern == EmoteMotionPattern::kConfident) {
        pulse(MotionDirection::kRight, 140); pulse(MotionDirection::kLeft, 140); // xoay điệu
    } else if (pattern == EmoteMotionPattern::kThinking) {
        pulse(MotionDirection::kLeft, 120); pulse(MotionDirection::kRight, 120); // nghiêng ngó
    } else if (pattern == EmoteMotionPattern::kRest) {
        vTaskDelay(pdMS_TO_TICKS(180));      // chỉ đồng bộ nét mặt, không cần di chuyển
    }
    motion.Stop(MotionSource::kEmotion);
    g_behavior_running = false;
    g_behavior_state = generation == g_behavior_generation
                           ? BehaviorState::kCompleted
                           : BehaviorState::kCancelled;
    g_agent_state = g_ultrasonic_healthy ? AgentState::kIdle : AgentState::kDegraded;
    emote_busy = false;
    vTaskDelete(nullptr);
}

static void TriggerEmoteMotion(const char* emotion) {
    const EmoteMotionPattern pattern = GetEmoteMotionPattern(emotion);
    if (pattern == EmoteMotionPattern::kNone) return;
    g_behavior_pattern = pattern;
    g_behavior_sequence = g_behavior_sequence + 1;
    g_behavior_body_executed = false;
    if (emote_busy || g_goal_running || MotionController::GetInstance().IsMoving()) {
        g_behavior_state = BehaviorState::kBusy;
        return;
    }
    if (!AutonomousMotionAllowed()) {
        // The mind/face response is still valid while wheels are locked. Once
        // armed, the same emotion hook will execute the coordinated body pulse.
        g_behavior_state = BehaviorState::kFaceOnly;
        return;
    }
    emote_busy = true;
    if (xTaskCreate(EmoteMotionTask, "emote_motion", 3072,
                    reinterpret_cast<void*>(static_cast<uintptr_t>(pattern)), 4, nullptr) != pdPASS) {
        emote_busy = false;
        g_behavior_state = BehaviorState::kCancelled;
    }
}

static void CancelSynchronizedBehavior() {
    g_behavior_generation = g_behavior_generation + 1;
    g_behavior_running = false;
    if (g_behavior_pattern != EmoteMotionPattern::kNone) {
        g_behavior_state = BehaviorState::kCancelled;
    }
    MotionController::GetInstance().Stop(MotionSource::kEmotion);
}

static void SetControlMode(ControlMode mode) {
    g_autonomy_armed = false;
    g_explore_until = 0;
    g_motor_test_cancel = true;
    CancelAgentGoal();
    CancelSynchronizedBehavior();
    MotionController::GetInstance().EmergencyStop();
    g_control_mode = mode;
    ESP_LOGI(TAG, "Control mode changed to %s; robot stopped and disarmed",
             ControlModeName(mode));
}

static void AgentGoalTask(void* arg) {
    const AgentGoal goal = static_cast<AgentGoal>(reinterpret_cast<uintptr_t>(arg));
    const uint32_t generation = g_goal_generation;
    auto& motion = MotionController::GetInstance();
    g_goal_state = GoalState::kRunning;
    g_agent_state = AgentState::kGoal;
    int stable_samples = 0;
    int lost_samples = 0;

    auto allowed = [generation]() {
        return generation == g_goal_generation && g_goal_running &&
               AutonomousMotionAllowed() && !g_hazard_handling;
    };
    auto pulse = [&motion, &allowed](MotionDirection direction, int speed, int duration_ms) {
        if (!allowed()) return false;
        if (!motion.Submit(MotionSource::kAutonomous, direction, speed, duration_ms)) return false;
        vTaskDelay(pdMS_TO_TICKS(duration_ms + 20));
        motion.Stop(MotionSource::kAutonomous);
        return allowed();
    };
    auto sample = []() {
        int values[3];
        int count = 0;
        for (int i = 0; i < 3; ++i) {
            const int distance_cm = PingCm();
            if (distance_cm > 0) values[count++] = distance_cm;
            vTaskDelay(pdMS_TO_TICKS(55));
        }
        if (count == 0) return -1;
        for (int i = 0; i < count; ++i)
            for (int j = i + 1; j < count; ++j)
                if (values[j] < values[i]) std::swap(values[i], values[j]);
        return values[count / 2];
    };

    GoalState final_state = GoalState::kCompleted;
    if (goal == AgentGoal::kInspect) {
        if (g_display) g_display->SetEmotion("thinking");
        if (!pulse(MotionDirection::kLeft, 42, 320)) final_state = GoalState::kCancelled;
        if (final_state == GoalState::kCompleted) {
            g_goal_left_distance_cm = sample();
            if (!pulse(MotionDirection::kRight, 42, 640)) final_state = GoalState::kCancelled;
        }
        if (final_state == GoalState::kCompleted) {
            g_goal_right_distance_cm = sample();
            if (!pulse(MotionDirection::kLeft, 42, 320)) final_state = GoalState::kCancelled;
        }
        if (final_state == GoalState::kCompleted) {
            const bool choose_left = g_goal_left_distance_cm >= g_goal_right_distance_cm;
            pulse(choose_left ? MotionDirection::kLeft : MotionDirection::kRight, 38, 220);
            g_goal_last_distance_cm = choose_left ? g_goal_left_distance_cm
                                                  : g_goal_right_distance_cm;
        }
    } else {
        while (allowed() && esp_timer_get_time() < g_goal_deadline_us) {
            const int distance_cm = PingCm();
            g_goal_last_distance_cm = distance_cm;
            if (distance_cm < 0 || distance_cm > 160) {
                motion.Stop(MotionSource::kAutonomous);
                stable_samples = 0;
                if (++lost_samples >= 8) {
                    final_state = GoalState::kTargetLost;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(180));
                continue;
            }
            lost_samples = 0;

            if (goal == AgentGoal::kFollow) {
                if (distance_cm > 38) {
                    stable_samples = 0;
                    pulse(MotionDirection::kForward, 42, 220);
                } else if (distance_cm < 18) {
                    stable_samples = 0;
                    pulse(MotionDirection::kBackward, 38, 160);
                } else {
                    motion.Stop(MotionSource::kAutonomous);
                    stable_samples++;
                    vTaskDelay(pdMS_TO_TICKS(180));
                }
            } else if (goal == AgentGoal::kApproach) {
                if (distance_cm > 32) {
                    stable_samples = 0;
                    pulse(MotionDirection::kForward, 40, 220);
                } else if (distance_cm < 20) {
                    stable_samples = 0;
                    pulse(MotionDirection::kBackward, 36, 140);
                } else if (++stable_samples >= 2) {
                    motion.Stop(MotionSource::kAutonomous);
                    final_state = GoalState::kCompleted;
                    break;
                }
            }
        }
        if (generation != g_goal_generation || !g_goal_running) {
            final_state = GoalState::kCancelled;
        }
    }

    motion.Stop(MotionSource::kAutonomous);
    if (generation == g_goal_generation) {
        g_goal_running = false;
        g_goal_state = final_state;
        g_agent_state = g_ultrasonic_healthy ? AgentState::kIdle : AgentState::kDegraded;
        if (final_state == GoalState::kCompleted && goal == AgentGoal::kApproach) {
            RequestSynchronizedBehavior("greet");
        } else if (final_state == GoalState::kTargetLost && g_display) {
            g_display->SetEmotion("confused");
        }
    }
    vTaskDelete(nullptr);
}

static void CancelAgentGoal() {
    g_goal_generation = g_goal_generation + 1;
    if (g_goal_running) g_goal_state = GoalState::kCancelled;
    g_goal_running = false;
    g_goal_deadline_us = 0;
    g_explore_until = 0;
    MotionController::GetInstance().Stop(MotionSource::kAutonomous);
}

static bool RequestAgentGoal(const char* goal_name, int seconds) {
    AgentGoal goal = AgentGoal::kNone;
    if (goal_name != nullptr && std::strcmp(goal_name, "follow") == 0) goal = AgentGoal::kFollow;
    else if (goal_name != nullptr && std::strcmp(goal_name, "approach") == 0) goal = AgentGoal::kApproach;
    else if (goal_name != nullptr && std::strcmp(goal_name, "inspect") == 0) goal = AgentGoal::kInspect;
    else return false;

    if (!AutonomousMotionAllowed() || g_goal_running || g_motor_test_running) {
        g_goal_state = GoalState::kBlocked;
        return false;
    }
    CancelSynchronizedBehavior();
    g_goal_generation = g_goal_generation + 1;
    g_agent_goal = goal;
    g_goal_state = GoalState::kRunning;
    g_goal_running = true;
    g_goal_last_distance_cm = -1;
    g_goal_left_distance_cm = -1;
    g_goal_right_distance_cm = -1;
    seconds = std::clamp(seconds, 5, 60);
    g_goal_deadline_us = esp_timer_get_time() + static_cast<int64_t>(seconds) * 1000000;
    if (xTaskCreate(AgentGoalTask, "agent_goal", 4096,
                    reinterpret_cast<void*>(static_cast<uintptr_t>(goal)), 5, nullptr) != pdPASS) {
        g_goal_running = false;
        g_goal_state = GoalState::kBlocked;
        return false;
    }
    return true;
}

static bool RequestSynchronizedBehavior(const char* behavior) {
    if (behavior == nullptr || behavior[0] == '\0' || g_display == nullptr || emote_busy) {
        return false;
    }
    const char* emotion = nullptr;
    if (std::strcmp(behavior, "greet") == 0) emotion = "happy";
    else if (std::strcmp(behavior, "celebrate") == 0) emotion = "laughing";
    else if (std::strcmp(behavior, "curious") == 0) emotion = "thinking";
    else if (std::strcmp(behavior, "startled") == 0) emotion = "surprised";
    else if (std::strcmp(behavior, "comfort") == 0) emotion = "loving";
    else if (std::strcmp(behavior, "confident") == 0) emotion = "cool";
    else if (std::strcmp(behavior, "rest") == 0) emotion = "relaxed";
    else return false;

    g_last_activity = esp_timer_get_time();
    g_display->SetEmotion(emotion);
    return true;
}

class FixedBacklight : public Backlight {
protected:
    void SetBrightnessImpl(uint8_t brightness) override {
        (void)brightness;
    }
};

class DuyBotLive2D : public WifiBoard {
public:
    DuyBotLive2D() : boot_button_(BOOT_BUTTON_GPIO) {
        g_boot_reset_reason = esp_reset_reason();
        if (!PowerRecoveryReady()) {
            ESP_LOGW(TAG, "Previous reset was %s; autonomous motion locked for 60s",
                     ResetReasonName(g_boot_reset_reason));
        }
        InitializeSpi();
        InitializeDisplay();
        g_display = display_;
        InitializeButton();
        InitializeMotors();
        RegisterMcpTools();
        RequestSelfCheck();
        xTaskCreate([](void* arg) {
            (void)arg;
            int64_t press_start = 0, last_release = 0, last_react = 0;
            bool was_touching = false, pending_single = false, suppress_release = false;
            bool last_raw = false;
            int stable_samples = 0;
            while (true) {
                vTaskDelay(pdMS_TO_TICKS(40));
                const bool raw = gpio_get_level(duybot::kTouchSensor) == 1;
                if (raw == last_raw) stable_samples++;
                else { last_raw = raw; stable_samples = 1; }
                if (stable_samples < 2) continue;             // debounce 80 ms
                const bool touching = last_raw;
                int64_t now = esp_timer_get_time();
                auto react = [&](TouchGesture gesture, const char* behavior,
                                 int mood_delta, const std::string_view& sound) {
                    if (now - last_react < 800000) return;    // tránh rung/nhiễu phát lặp
                    last_react = now;
                    g_last_touch_gesture = gesture;
                    g_touch_event_count = g_touch_event_count + 1;
                    g_last_touch_event_us = now;
                    MoodBump(mood_delta);
                    RequestSynchronizedBehavior(behavior);
                    auto& app = Application::GetInstance();
                    if (!sound.empty() && app.GetDeviceState() == kDeviceStateIdle) {
                        app.PlaySound(sound);                 // âm thanh bắt đầu cùng behavior
                    }
                };
                if (touching && !was_touching) {              // bắt đầu chạm
                    press_start = now;
                    if (pending_single && now - last_release < 400000) {
                        pending_single = false;
                        suppress_release = true;
                        react(TouchGesture::kDoubleTap, "celebrate", 5,
                              Lang::Sounds::OGG_TOUCH_ARA);
                    }
                } else if (!touching && was_touching) {       // thả tay
                    if (suppress_release) {
                        suppress_release = false;              // không phát dư tap sau double-tap
                        pending_single = false;
                    } else {
                        const int64_t held = now - press_start;
                        if (held >= 900000) {
                            react(TouchGesture::kLongPress, "comfort", 8,
                                  Lang::Sounds::OGG_TOUCH_LONG);
                            pending_single = false;
                        } else {
                            pending_single = true;            // chờ tap thứ hai
                            last_release = now;
                        }
                    }
                } else if (pending_single && now - last_release >= 400000) {
                    pending_single = false;
                    react(TouchGesture::kTap, "greet", 3, Lang::Sounds::OGG_TOUCH_ARA);
                }
                was_touching = touching;
            }
        }, "touch_poll", 3072, this, 3, NULL);

        // ===== IDLE + MOOD TASK: sự sống khi rảnh =====
        xTaskCreate([](void* arg) {
            auto* self = (DuyBotLive2D*)arg;
            int64_t last_decay = 0, next_idle_act = 0;
            g_last_activity = esp_timer_get_time();
            while (true) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                AgentSupervisorStep();
                int64_t now = esp_timer_get_time();
                auto& app = Application::GetInstance();
                bool busy = app.GetDeviceState() != kDeviceStateIdle;
                if (busy) {
                    g_last_activity = now;                    // đang hội thoại = có tương tác
                    continue;
                }
                auto& motion = MotionController::GetInstance();
                if (motion.IsMoving() || g_hazard_handling) continue;

                // mood trôi về 50 (1 nấc / 3 phút)
                if (now - last_decay > 180000000LL) {
                    last_decay = now;
                    if (g_mood > 50) g_mood = g_mood - 1;
                    else if (g_mood < 50) g_mood = g_mood + 1;
                }

                int64_t idle_us = now - g_last_activity;
                if (idle_us < 45000000LL) continue;           // chưa rảnh đủ 45s
                if (now < next_idle_act) continue;
                // hẹn lần ngẫu hứng kế: 40-100s
                next_idle_act = now + 40000000LL + (esp_random() % 60000000LL);

                int roll = esp_random() % 100;
                if (idle_us > 600000000LL && roll < 40) {     // bỏ rơi >10 phút: ngủ gật
                    if (self->display_) self->display_->SetEmotion("sleepy");
                } else if (g_mood >= 65) {
                    if (roll < 50 && self->display_) self->display_->SetEmotion("happy");
                    else if (AutonomousMotionAllowed()) {     // supervisor quyết định có được di chuyển
                        motion.Submit(MotionSource::kIdle,
                                      roll % 2 ? MotionDirection::kLeft : MotionDirection::kRight,
                                      g_speed, 110);
                        vTaskDelay(pdMS_TO_TICKS(110));
                        motion.Stop(MotionSource::kIdle);
                        vTaskDelay(pdMS_TO_TICKS(400));
                        motion.Submit(MotionSource::kIdle,
                                      roll % 2 ? MotionDirection::kRight : MotionDirection::kLeft,
                                      g_speed, 110);
                        vTaskDelay(pdMS_TO_TICKS(110));
                        motion.Stop(MotionSource::kIdle);
                    }
                } else if (g_mood <= 35) {
                    if (self->display_) self->display_->SetEmotion(roll < 50 ? "sad" : "confused");
                } else {
                    if (roll < 40 && self->display_) self->display_->SetEmotion("thinking");
                    else if (roll < 70 && self->display_) self->display_->SetEmotion("relaxed");
                }
            }
        }, "idle_mood", 4096, this, 3, NULL);

        xTaskCreate([](void* arg) {
            vTaskDelay(pdMS_TO_TICKS(12000));  // cho WiFi len
            StartManualWebServer();
            vTaskDelete(NULL);
        }, "web_start", 4096, NULL, 3, NULL);
        xTaskCreatePinnedToCore(GuardTask, "chassis_guard", 4096, NULL, 6, NULL, 1);
        display_->SetEmotionHook([](const char* emotion) {
            TriggerEmoteMotion(emotion);
        });
#if AVATAR_STREAM_ENABLED
        avatar_stream_ = std::make_unique<AvatarStreamClient>(display_);
        if (!avatar_stream_->Start()) {
            ESP_LOGE(TAG, "Avatar stream initialization failed; Xiaozhi UI remains available");
        }
#else
        ESP_LOGI(TAG, "Avatar stream disabled; Xiaozhi UI remains active");
#endif
        GetBacklight()->RestoreBrightness();
    }

    AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex codec(
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK,
            AUDIO_I2S_SPK_GPIO_LRCK,
            AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK,
            AUDIO_I2S_MIC_GPIO_WS,
            AUDIO_I2S_MIC_GPIO_DIN);
        return &codec;
    }

    Display* GetDisplay() override {
        return display_;
    }

    Backlight* GetBacklight() override {
        static FixedBacklight backlight;
        return &backlight;
    }

private:
    Button boot_button_;
    DuyBotDisplay* display_ = nullptr;
    std::unique_ptr<AvatarStreamClient> avatar_stream_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;

    void InitializeSpi() {
        spi_bus_config_t bus_config = {};
        bus_config.mosi_io_num = DISPLAY_SDA;
        bus_config.miso_io_num = GPIO_NUM_NC;
        bus_config.sclk_io_num = DISPLAY_SCL;
        bus_config.quadwp_io_num = GPIO_NUM_NC;
        bus_config.quadhd_io_num = GPIO_NUM_NC;
        bus_config.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO));
    }

    void InitializeDisplay() {
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS;
        io_config.dc_gpio_num = DISPLAY_DC;
        io_config.spi_mode = 3;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io_));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RES;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io_, &panel_config, &panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, true));

        display_ = new DuyBotDisplay(
            panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButton() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
            } else {
                app.ToggleChatState();
            }
        });
        boot_button_.OnLongPress([this]() {
            EnterWifiConfigMode();
        });
    }

    void InitializeMotors() {
        // The MotionController owns all motor GPIO and PWM. This board class
        // only configures chassis sensors and the ultrasonic trigger.
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << duybot::kUltrasonicTrigger);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        // ECHO siêu âm làm input
        gpio_config_t echo_conf = {};
        echo_conf.intr_type = GPIO_INTR_DISABLE;
        echo_conf.mode = GPIO_MODE_INPUT;
        echo_conf.pin_bit_mask = (1ULL << duybot::kUltrasonicEcho);
        echo_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        echo_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&echo_conf);

        // Dò line làm input pull-up
        gpio_config_t line_conf = {};
        line_conf.intr_type = GPIO_INTR_DISABLE;
        line_conf.mode = GPIO_MODE_INPUT;
        line_conf.pin_bit_mask = (1ULL << duybot::kLineSensor);
        line_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        line_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&line_conf);

        // Cảm biến chạm TTP223: OUT -> GPIO 8 (HIGH khi chạm)
        gpio_config_t touch_conf = {};
        touch_conf.intr_type = GPIO_INTR_DISABLE;
        touch_conf.mode = GPIO_MODE_INPUT;
        touch_conf.pin_bit_mask = (1ULL << duybot::kTouchSensor);
        touch_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        touch_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&touch_conf);

        gpio_set_level(duybot::kUltrasonicTrigger, 0);
        if (g_ping_mutex == nullptr) g_ping_mutex = xSemaphoreCreateMutex();
        g_motion_ready = MotionController::GetInstance().Initialize();
        if (!g_motion_ready) {
            ESP_LOGE(TAG, "Motion controller initialization failed; chassis remains disabled");
        }
        ESP_LOGI(TAG, "Chassis GPIOs initialized (motor 11-14, trig 10, echo 18, line 3, touch 8)");
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.chassis.move", 
            "Di chuyển robot DuyBot: tiến (forward), lùi (backward), rẽ trái (left), rẽ phải (right), dừng lại (stop). "
            "duration_ms là thời gian chạy. speed là tốc độ 20-100%% (mặc định 80; đi chậm dùng 40-50). "
            "Rẽ trái/phải tự động chạy ở 55%% tốc độ; rẽ nhẹ 300-500ms, quay 90 độ khoảng 800ms.",
            PropertyList({
                Property("direction", kPropertyTypeString, "forward"),
                Property("duration_ms", kPropertyTypeInteger, 1000, 100, 10000),
                Property("speed", kPropertyTypeInteger, 80, 20, 100)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                std::string direction = properties["direction"].value<std::string>();
                int duration_ms = properties["duration_ms"].value<int>();
                g_speed = properties["speed"].value<int>();
                
                ESP_LOGI(TAG, "Chassis move command: direction=%s duration=%d ms", direction.c_str(), duration_ms);
                
                g_last_activity = esp_timer_get_time();
                auto& motion = MotionController::GetInstance();
                if (direction == "stop") {
                    g_motor_test_cancel = true;
                    CancelSynchronizedBehavior();
                    motion.EmergencyStop();
                    return true;
                }
                MotionDirection parsed = MotionController::ParseDirection(direction.c_str());
                if (parsed == MotionDirection::kStop) {
                    return "Huong khong hop le: " + direction;
                }
                if (!AutonomousMotionAllowed()) {
                    return "Chuyen dong AI dang bi khoa. Hay chay self-check va arm agent khi robot o vi tri an toan.";
                }
                return motion.Submit(MotionSource::kMcp, parsed, g_speed, duration_ms);
            });
        mcp_server.AddTool("self.chassis.get_distance",
            "Đo khoảng cách (cm) tới vật cản phía trước robot bằng cảm biến HY-SRF05 ở chế độ TRIG/ECHO riêng; chân OUT phải để hở. Trả về -1 nếu ngoài tầm đo.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                int vals[3];
                int ok = 0;
                for (int i = 0; i < 3; i++) {
                    int v = PingCm();
                    if (v > 0) vals[ok++] = v;
                    vTaskDelay(pdMS_TO_TICKS(70));
                }
                if (ok == 0) return -1;
                // lấy trung vị của các lần đo hợp lệ
                for (int i = 0; i < ok; i++)
                    for (int j = i + 1; j < ok; j++)
                        if (vals[j] < vals[i]) { int t = vals[i]; vals[i] = vals[j]; vals[j] = t; }
                return vals[ok / 2];
            });

        mcp_server.AddTool("self.body.get_mood",
            "Xem tâm trạng hiện tại của robot (0-100): >65 vui vẻ, 35-65 bình thường, <35 buồn/dỗi. "
            "Tăng khi được vuốt ve, giảm khi va chạm hoặc bị bỏ rơi lâu.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                int m = g_mood;
                std::string label = m >= 65 ? "vui vẻ" : (m <= 35 ? "buồn/dỗi" : "bình thường");
                return "Mood " + std::to_string(m) + "/100 (" + label + ")";
            });

        mcp_server.AddTool("self.body.get_ip",
            "Lấy địa chỉ IP của robot trong mạng WiFi. Mở trình duyệt điện thoại vào http://<IP> để điều khiển manual.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
                if (netif == nullptr) return "Chua co mang";
                esp_netif_ip_info_t ip;
                if (esp_netif_get_ip_info(netif, &ip) != ESP_OK) return "Chua co IP";
                char buf[32];
                snprintf(buf, sizeof(buf), IPSTR, IP2STR(&ip.ip));
                return std::string(buf);
            });

        mcp_server.AddTool("self.chassis.explore",
            "Cho robot đi dạo thông minh trong N giây (5-60): tự tiến, tự né vật cản, "
            "tự tránh rơi khỏi mép bàn, hết giờ tự dừng.",
            PropertyList({
                Property("seconds", kPropertyTypeInteger, 15, 5, 60)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                int s = properties["seconds"].value<int>();
                if (!AutonomousMotionAllowed()) {
                    return "Khong the di dao: self-check phai PASS va agent phai duoc arm.";
                }
                g_explore_until = esp_timer_get_time() + (int64_t)s * 1000000;
                return "Bat dau di dao " + std::to_string(s) + " giay";
            });

        mcp_server.AddTool("self.chassis.set_guard",
            "Bật/tắt hệ thống bảo vệ tự động (chống rơi mép bàn + tự né vật cản khi di chuyển).",
            PropertyList({
                Property("enabled", kPropertyTypeBoolean, true)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                g_guard_on = properties["enabled"].value<bool>();
                if (!g_guard_on) {
                    g_autonomy_armed = false;
                    g_explore_until = 0;
                    CancelSynchronizedBehavior();
                    MotionController::GetInstance().Stop(MotionSource::kAutonomous);
                }
                return g_guard_on ? "Ve si da bat" : "Ve si da tat";
            });

        mcp_server.AddTool("self.chassis.calibrate_edge",
            "Học lại mức cảm biến dò line của mặt bàn hiện tại (gọi khi đặt robot lên mặt bàn mới).",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                g_edge_safe = gpio_get_level(duybot::kLineSensor);
                return "Da hoc mat ban: muc " + std::to_string(g_edge_safe);
            });

        mcp_server.AddTool("self.body.get_touch_status",
            "Kiểm tra cảm biến chạm GPIO8, gesture gần nhất và tổng số sự kiện touch đã nhận.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                std::string result = "raw=";
                result += gpio_get_level(duybot::kTouchSensor) ? "1" : "0";
                result += ", last_gesture=";
                result += TouchGestureName(g_last_touch_gesture);
                result += ", event_count=" + std::to_string(g_touch_event_count);
                return result;
            });

        mcp_server.AddTool("self.body.perform_behavior",
            "Đồng bộ mind và body bằng một hành vi cấp cao: greet, celebrate, curious, startled, comfort, confident hoặc rest. "
            "Biểu cảm luôn chạy; chuyển động cơ thể chỉ chạy khi agent đã arm và supervisor xác nhận an toàn.",
            PropertyList({
                Property("behavior", kPropertyTypeString, "greet")
            }),
            [](const PropertyList& properties) -> ReturnValue {
                const std::string behavior = properties["behavior"].value<std::string>();
                if (!RequestSynchronizedBehavior(behavior.c_str())) {
                    return "Behavior khong hop le hoac coordinator dang ban";
                }
                std::string result = "behavior=";
                result += BehaviorPatternName(g_behavior_pattern);
                result += ", state=";
                result += BehaviorStateName(g_behavior_state);
                result += ", body_allowed=";
                result += AutonomousMotionAllowed() ? "true" : "false";
                return result;
            });

        mcp_server.AddTool("self.agent.execute_goal",
            "Giao mục tiêu đóng vòng cho robot: follow giữ khoảng cách với tay/vật, approach tiến tới khoảng 20-32cm, "
            "inspect quay nhìn hai phía và chọn phía thoáng. ESP32 tự quyết định từng nhịp từ sensor; cần smart mode và đã arm.",
            PropertyList({
                Property("goal", kPropertyTypeString, "follow"),
                Property("seconds", kPropertyTypeInteger, 15, 5, 60)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                const std::string goal = properties["goal"].value<std::string>();
                const int seconds = properties["seconds"].value<int>();
                if (!RequestAgentGoal(goal.c_str(), seconds)) {
                    return "Goal bi khoa: can smart mode, arm va sensor PASS; goal hop le follow/approach/inspect";
                }
                return "goal=" + goal + ", state=running, closed_loop=true";
            });

        mcp_server.AddTool("self.chassis.echo_diag",
            "Chẩn đoán cảm biến siêu âm: bắn xung TRIG rồi ghi sóng thô trên chân ECHO trong 100ms. "
            "Trả về mức nghỉ, số lần đổi mức và độ rộng xung — dùng khi cảm biến không đo được.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                if (g_ping_mutex != nullptr &&
                    xSemaphoreTake(g_ping_mutex, pdMS_TO_TICKS(150)) != pdTRUE) {
                    return "Cam bien dang ban; thu lai sau";
                }
                int idle = gpio_get_level(duybot::kUltrasonicEcho);
                gpio_set_level(duybot::kUltrasonicTrigger, 0);
                esp_rom_delay_us(4);
                gpio_set_level(duybot::kUltrasonicTrigger, 1);
                esp_rom_delay_us(12);
                gpio_set_level(duybot::kUltrasonicTrigger, 0);
                int last = gpio_get_level(duybot::kUltrasonicEcho);
                int transitions = 0;
                int64_t high_us = 0, t0 = esp_timer_get_time(), last_change = t0;
                while (esp_timer_get_time() - t0 < 100000) {  // quan sát 100ms
                    int lv = gpio_get_level(duybot::kUltrasonicEcho);
                    if (lv != last) {
                        transitions++;
                        if (last == 1) high_us += esp_timer_get_time() - last_change;
                        last_change = esp_timer_get_time();
                        last = lv;
                    }
                }
                char buf[160];
                snprintf(buf, sizeof(buf),
                    "ECHO idle=%d, doi muc=%d lan, tong xung HIGH=%lld us. "
                    "(0 lan doi muc = khong co tin hieu ve: kiem tra nguon 5V/day/chan)",
                    idle, transitions, (long long)high_us);
                if (g_ping_mutex != nullptr) xSemaphoreGive(g_ping_mutex);
                return std::string(buf);
            });

        mcp_server.AddTool("self.chassis.get_line_status",
            "Đọc cảm biến dò line MH-B dưới gầm xe. Trả về 1 nếu thấy vạch đen, 0 nếu nền trắng.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                return gpio_get_level(duybot::kLineSensor);
            });

        mcp_server.AddTool("self.body.self_check",
            "Xem sức khỏe phần cứng của robot. Đặt rerun=true để chạy lại kiểm tra an toàn; self-check không làm motor quay.",
            PropertyList({
                Property("rerun", kPropertyTypeBoolean, false)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                if (properties["rerun"].value<bool>()) RequestSelfCheck();
                std::string result = "self_check=";
                result += SelfCheckStateName(g_self_check_state);
                result += ", ultrasonic=";
                result += g_ultrasonic_healthy ? "ok" : "fail";
                result += ", reason=";
                result += SelfCheckReason();
                result += ", distance_cm=" + std::to_string(g_last_distance_cm);
                result += ", line=" + std::to_string(gpio_get_level(duybot::kLineSensor));
                result += ", touch=" + std::to_string(gpio_get_level(duybot::kTouchSensor));
                result += ", motion=";
                result += g_motion_ready ? "ready" : "fail";
                return result;
            });

        mcp_server.AddTool("self.agent.get_state",
            "Xem trạng thái supervisor agent, self-check và khóa chuyển động tự hành.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                std::string result = "agent=";
                result += AgentStateName(g_agent_state);
                result += ", enabled=";
                result += g_agent_enabled ? "true" : "false";
                result += ", autonomy_armed=";
                result += g_autonomy_armed ? "true" : "false";
                result += ", can_arm=";
                result += CanArmAutonomy() ? "true" : "false";
                result += ", control_mode=";
                result += ControlModeName(g_control_mode);
                result += ", reset_reason=";
                result += ResetReasonName(g_boot_reset_reason);
                result += ", power_ready=";
                result += PowerRecoveryReady() ? "true" : "false";
                return result;
            });

        mcp_server.AddTool("self.agent.set_control_mode",
            "Chuyển chế độ điều khiển manual hoặc smart. Mỗi lần chuyển mode robot sẽ STOP, hủy behavior và disarm.",
            PropertyList({
                Property("mode", kPropertyTypeString, "manual")
            }),
            [](const PropertyList& properties) -> ReturnValue {
                const std::string mode = properties["mode"].value<std::string>();
                if (mode == "manual") SetControlMode(ControlMode::kManual);
                else if (mode == "smart") SetControlMode(ControlMode::kSmart);
                else return "Mode khong hop le; dung manual hoac smart";
                return "control_mode=" + mode + ", stopped=true, autonomy_armed=false";
            });

        mcp_server.AddTool("self.agent.set_autonomy",
            "Bật/tắt supervisor agent. Chỉ đặt confirmed_safe=true sau khi người dùng xác nhận robot ở dưới sàn an toàn; "
            "agent sẽ từ chối arm nếu self-check hoặc HY-SRF05 chưa đạt.",
            PropertyList({
                Property("enabled", kPropertyTypeBoolean, true),
                Property("confirmed_safe", kPropertyTypeBoolean, false)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                g_agent_enabled = properties["enabled"].value<bool>();
                if (!g_agent_enabled) {
                    g_autonomy_armed = false;
                    g_explore_until = 0;
                    CancelAgentGoal();
                    CancelSynchronizedBehavior();
                    MotionController::GetInstance().EmergencyStop();
                    return "Agent da tat va robot da dung";
                }
                if (!properties["confirmed_safe"].value<bool>()) {
                    g_autonomy_armed = false;
                    CancelAgentGoal();
                    CancelSynchronizedBehavior();
                    return "Agent nhan thuc da bat, nhung chuyen dong van khoa cho den khi nguoi dung xac nhan an toan";
                }
                if (!CanArmAutonomy()) {
                    g_autonomy_armed = false;
                    return "Khong arm duoc: phai o smart mode va self-check/ultrasonic/guard/motion san sang";
                }
                g_autonomy_armed = true;
                return "Agent da arm; supervisor se tu chan hanh dong khong an toan";
            });

        ESP_LOGI(TAG, "DuyBot MCP tools registered: chassis, self-check and agent supervisor");
    }
};

DECLARE_BOARD(DuyBotLive2D);
