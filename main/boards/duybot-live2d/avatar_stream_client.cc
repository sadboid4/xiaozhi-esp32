#include "avatar_stream_client.h"

#include "board.h"
#include "config.h"
#include "duybot_display.h"
#include "settings.h"

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>

#define TAG "AvatarStream"

namespace {
constexpr size_t kFrameBufferBytes = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
constexpr int64_t kMinimumFrameIntervalUs = 1000000 / AVATAR_MAX_FPS;
}

AvatarStreamClient::AvatarStreamClient(DuyBotDisplay* display) : display_(display) {}

AvatarStreamClient::~AvatarStreamClient() {
    stopping_ = true;
    if (websocket_) {
        websocket_->Close();
    }
    if (task_) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (decoder_) {
        jpeg_dec_close(decoder_);
    }
    if (frame_mutex_) {
        vSemaphoreDelete(frame_mutex_);
        frame_mutex_ = nullptr;
    }
    for (auto* buffer : frame_buffers_) {
        heap_caps_free(buffer);
    }
}

bool AvatarStreamClient::Start() {
    frame_mutex_ = xSemaphoreCreateMutex();
    if (frame_mutex_ == nullptr) {
        ESP_LOGE(TAG, "Unable to create avatar frame mutex");
        return false;
    }

    for (auto& buffer : frame_buffers_) {
        buffer = static_cast<uint8_t*>(heap_caps_aligned_alloc(16, kFrameBufferBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (buffer == nullptr) {
            ESP_LOGE(TAG, "Unable to allocate %zu-byte PSRAM framebuffer", kFrameBufferBytes);
            return false;
        }
    }

    jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
    config.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;
    if (jpeg_dec_open(&config, &decoder_) != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "Unable to initialize JPEG decoder");
        return false;
    }

    if (xTaskCreate(TaskEntry, "avatar_stream", 6144, this, 6, &task_) != pdPASS) {
        ESP_LOGE(TAG, "Unable to create avatar task");
        task_ = nullptr;
        return false;
    }
    return true;
}

void AvatarStreamClient::TaskEntry(void* arg) {
    static_cast<AvatarStreamClient*>(arg)->Run();
    vTaskDelete(nullptr);
}

void AvatarStreamClient::Run() {
    vTaskDelay(pdMS_TO_TICKS(5000));
    Settings settings("avatar", false);
    const std::string url = settings.GetString("url", AVATAR_STREAM_URL);

    while (!stopping_) {
        if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < AVATAR_MIN_INTERNAL_HEAP) {
            ESP_LOGW(TAG, "Waiting for internal heap before reconnect");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        websocket_ = Board::GetInstance().GetNetwork()->CreateWebSocket(2);
        if (!websocket_) {
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        websocket_->OnData([this](const char* data, size_t size, bool binary) {
            if (binary && size > 128 && size <= AVATAR_MAX_JPEG_BYTES &&
                static_cast<uint8_t>(data[0]) == 0xff && static_cast<uint8_t>(data[1]) == 0xd8 &&
                static_cast<uint8_t>(data[size - 2]) == 0xff && static_cast<uint8_t>(data[size - 1]) == 0xd9) {
                if (xSemaphoreTake(frame_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                    // This may copy tens of kilobytes from the network. A mutex keeps
                    // interrupts enabled, unlike the former spinlock critical section.
                    jpeg_buffer_.assign(data, data + size);
                    new_frame_available_ = true;
                    xSemaphoreGive(frame_mutex_);
                } else {
                    dropped_frames_++;
                }
            } else {
                dropped_frames_++;
            }
        });
        websocket_->OnDisconnected([this]() {
            connected_ = false;
            ESP_LOGW(TAG, "Avatar stream disconnected");
        });
        websocket_->OnError([this](int error) {
            connected_ = false;
            ESP_LOGW(TAG, "Avatar stream error: %d", error);
        });

        ESP_LOGI(TAG, "Connecting to %s", url.c_str());
        connected_ = websocket_->Connect(url.c_str());
        if (connected_) {
            ESP_LOGI(TAG, "Avatar stream connected");
            last_frame_time_us_ = esp_timer_get_time();
            if (xSemaphoreTake(frame_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                new_frame_available_ = false;
                xSemaphoreGive(frame_mutex_);
            }
        }
        while (connected_ && !stopping_) {
            vTaskDelay(pdMS_TO_TICKS(10)); // Poll every 10ms for low latency
            
            bool process_frame = false;
            std::vector<uint8_t> local_buffer;
            
            if (xSemaphoreTake(frame_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (new_frame_available_) {
                    local_buffer = std::move(jpeg_buffer_);
                    new_frame_available_ = false;
                    process_frame = true;
                }
                xSemaphoreGive(frame_mutex_);
            }
            
            if (process_frame) {
                DecodeAndRender(local_buffer.data(), local_buffer.size());
            }
            
            // Connection Watchdog: If no frames are received for 5 seconds, force a reconnection
            if (esp_timer_get_time() - last_frame_time_us_ > 5000000) {
                ESP_LOGW(TAG, "Avatar stream read timeout (5s), reconnecting");
                connected_ = false;
                websocket_->Close();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(250));
        websocket_.reset();
        if (!stopping_) {
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
    task_ = nullptr;
}

void AvatarStreamClient::DecodeAndRender(const uint8_t* data, size_t size) {
    const int64_t now = esp_timer_get_time();
    last_frame_time_us_ = now;

    jpeg_dec_io_t io = {};
    jpeg_dec_header_info_t header = {};
    io.inbuf = const_cast<uint8_t*>(data);
    io.inbuf_len = static_cast<int>(size);

    if (jpeg_dec_parse_header(decoder_, &io, &header) != JPEG_ERR_OK ||
        header.width != DISPLAY_WIDTH || header.height != DISPLAY_HEIGHT) {
        ++decode_errors_;
        return;
    }

    io.outbuf = frame_buffers_[active_buffer_];
    if (jpeg_dec_process(decoder_, &io) != JPEG_ERR_OK || io.out_size != kFrameBufferBytes) {
        ++decode_errors_;
        return;
    }

    if (display_->ShowAvatarFrame(frame_buffers_[active_buffer_], kFrameBufferBytes, active_buffer_)) {
        active_buffer_ ^= 1;
        ++decoded_frames_;
    } else {
        ++dropped_frames_;
    }

    if ((decoded_frames_ + dropped_frames_) % 80 == 0) {
        ESP_LOGI(TAG, "decoded=%" PRIu32 " dropped=%" PRIu32 " errors=%" PRIu32 " internal=%zu psram=%zu",
                 decoded_frames_, dropped_frames_, decode_errors_,
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
}
