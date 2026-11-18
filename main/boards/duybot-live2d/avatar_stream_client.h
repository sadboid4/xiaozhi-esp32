#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

#include <esp_jpeg_dec.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <web_socket.h>

#include <vector>

class DuyBotDisplay;

class AvatarStreamClient {
public:
    explicit AvatarStreamClient(DuyBotDisplay* display);
    ~AvatarStreamClient();

    bool Start();

private:
    static void TaskEntry(void* arg);
    void Run();
    void DecodeAndRender(const uint8_t* data, size_t size);

    DuyBotDisplay* display_;
    std::unique_ptr<WebSocket> websocket_;
    jpeg_dec_handle_t decoder_ = nullptr;
    std::array<uint8_t*, 2> frame_buffers_{};
    TaskHandle_t task_ = nullptr;
    std::atomic<bool> connected_{false};
    std::atomic<bool> stopping_{false};
    uint32_t decoded_frames_ = 0;
    uint32_t dropped_frames_ = 0;
    uint32_t decode_errors_ = 0;
    int active_buffer_ = 0;
    int64_t last_frame_time_us_ = 0;

    std::vector<uint8_t> jpeg_buffer_;
    bool new_frame_available_ = false;
    SemaphoreHandle_t frame_mutex_ = nullptr;
};
