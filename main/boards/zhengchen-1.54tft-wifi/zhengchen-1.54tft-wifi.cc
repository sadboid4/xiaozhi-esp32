#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "zhengchen_lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "power_save_timer.h"
#include "led/single_led.h"
#include "assets/lang_config.h"
#include "power_manager.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>

#include <driver/rtc_io.h>
#include <esp_sleep.h>

#define TAG "ZHENGCHEN_1_54TFT_WIFI"

class ZHENGCHEN_1_54TFT_WIFI : public WifiBoard {
private:
    Button boot_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    ZHENGCHEN_LcdDisplay* display_;
    PowerSaveTimer* power_save_timer_;
    PowerManager* power_manager_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;

    void InitializePowerManager() {
        power_manager_ = new PowerManager(GPIO_NUM_9);
        power_manager_->OnTemperatureChanged([this](float chip_temp) {
            display_->UpdateHighTempWarning(chip_temp);
        });

        power_manager_->OnChargingStatusChanged([this](bool is_charging) {
            if (is_charging) {
                power_save_timer_->SetEnabled(false);
                ESP_LOGI("PowerManager", "Charging started");
            } else {
                power_save_timer_->SetEnabled(true);
                ESP_LOGI("PowerManager", "Charging stopped");
            }
        });
    
    }

    void InitializePowerSaveTimer() {
        rtc_gpio_init(GPIO_NUM_2);
        rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_set_level(GPIO_NUM_2, 1);

        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(1);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_SDA;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_SCL;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        
        boot_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        // 设置开机按钮的长按事件（直接进入配网模式）
        boot_button_.OnLongPress([this]() {
            // 唤醒电源保存定时器
            power_save_timer_->WakeUp();
            // 获取应用程序实例
            auto& app = Application::GetInstance();
            
            // 进入配网模式
            app.SetDeviceState(kDeviceStateWifiConfiguring);
            
            // 重置WiFi配置以确保进入配网模式
            EnterWifiConfigMode();
        });

        volume_up_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume/10));
        });

        volume_up_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume/10));
        });

        volume_down_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    void InitializeSt7789Display() {
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS;
        io_config.dc_gpio_num = DISPLAY_DC;
        io_config.spi_mode = 3;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io_));

        ESP_LOGD(TAG, "Install LCD driver");
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

        display_ = new ZHENGCHEN_LcdDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, 
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        display_->SetupHighTempWarningPopup();
    }

    void InitializeTools() {
        // Cấu hình các chân GPIO làm Output cho motor và siêu âm
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << GPIO_NUM_11) | (1ULL << GPIO_NUM_12) | (1ULL << GPIO_NUM_13) | (1ULL << GPIO_NUM_14) | (1ULL << GPIO_NUM_10);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);

        // Cấu hình chân ECHO (GPIO 18) làm Input
        gpio_config_t echo_conf = {};
        echo_conf.intr_type = GPIO_INTR_DISABLE;
        echo_conf.mode = GPIO_MODE_INPUT;
        echo_conf.pin_bit_mask = (1ULL << GPIO_NUM_18);
        echo_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        echo_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&echo_conf);

        // Cấu hình chân Dò Line (GPIO 3) làm Input với Pull-up
        gpio_config_t line_conf = {};
        line_conf.intr_type = GPIO_INTR_DISABLE;
        line_conf.mode = GPIO_MODE_INPUT;
        line_conf.pin_bit_mask = (1ULL << GPIO_NUM_3);
        line_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        line_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&line_conf);

        // Đảm bảo ban đầu tắt hết động cơ và Trig
        gpio_set_level(GPIO_NUM_11, 0);
        gpio_set_level(GPIO_NUM_12, 0);
        gpio_set_level(GPIO_NUM_13, 0);
        gpio_set_level(GPIO_NUM_14, 0);
        gpio_set_level(GPIO_NUM_10, 0);

        auto& mcp_server = McpServer::GetInstance();

        // Đăng ký tool robot_chassis.move
        mcp_server.AddTool("self.robot_chassis.move",
            "Control the movement of the robot companion car (DuyBot). Directions: 'forward', 'backward', 'left' (turn left in place), 'right' (turn right in place), 'stop'. duration_ms is the time to run in milliseconds (default: 1000, max: 10000).",
            PropertyList({
                Property("direction", kPropertyTypeString),
                Property("duration_ms", kPropertyTypeInteger, 1000)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                auto direction = properties["direction"].value<std::string>();
                int duration_ms = properties["duration_ms"].value<int>();

                // Điều khiển GPIO
                if (direction == "forward") {
                    gpio_set_level(GPIO_NUM_11, 1);
                    gpio_set_level(GPIO_NUM_12, 0);
                    gpio_set_level(GPIO_NUM_13, 1);
                    gpio_set_level(GPIO_NUM_14, 0);
                } else if (direction == "backward") {
                    gpio_set_level(GPIO_NUM_11, 0);
                    gpio_set_level(GPIO_NUM_12, 1);
                    gpio_set_level(GPIO_NUM_13, 0);
                    gpio_set_level(GPIO_NUM_14, 1);
                } else if (direction == "left") {
                    gpio_set_level(GPIO_NUM_11, 0);
                    gpio_set_level(GPIO_NUM_12, 1);
                    gpio_set_level(GPIO_NUM_13, 1);
                    gpio_set_level(GPIO_NUM_14, 0);
                } else if (direction == "right") {
                    gpio_set_level(GPIO_NUM_11, 1);
                    gpio_set_level(GPIO_NUM_12, 0);
                    gpio_set_level(GPIO_NUM_13, 0);
                    gpio_set_level(GPIO_NUM_14, 1);
                } else { // "stop"
                    gpio_set_level(GPIO_NUM_11, 0);
                    gpio_set_level(GPIO_NUM_12, 0);
                    gpio_set_level(GPIO_NUM_13, 0);
                    gpio_set_level(GPIO_NUM_14, 0);
                    return "Stopped";
                }

                // Nếu có duration, tạo task tự động tắt
                if (duration_ms > 0) {
                    if (duration_ms > 10000) duration_ms = 10000; // Giới hạn max 10 giây để an toàn
                    xTaskCreate([](void* arg) {
                        uint32_t delay = (uint32_t)(uintptr_t)arg;
                        vTaskDelay(pdMS_TO_TICKS(delay));
                        gpio_set_level(GPIO_NUM_11, 0);
                        gpio_set_level(GPIO_NUM_12, 0);
                        gpio_set_level(GPIO_NUM_13, 0);
                        gpio_set_level(GPIO_NUM_14, 0);
                        vTaskDelete(NULL);
                    }, "stop_motor", 2048, (void*)(uintptr_t)duration_ms, 5, NULL);
                }

                return "Moving " + direction + " for " + std::to_string(duration_ms) + " ms";
            });

        // Đăng ký tool robot_chassis.get_distance
        mcp_server.AddTool("self.robot_chassis.get_distance",
            "Read the current distance (in centimeters) to any obstacle in front of the robot companion car using the HC-SR04 ultrasonic sensor. Returns -1 if out of range or timeout.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                gpio_set_level(GPIO_NUM_10, 0);
                esp_rom_delay_us(2);
                gpio_set_level(GPIO_NUM_10, 1);
                esp_rom_delay_us(10);
                gpio_set_level(GPIO_NUM_10, 0);

                int64_t start = esp_timer_get_time();
                int64_t timeout = start + 30000; // 30ms timeout
                while (gpio_get_level(GPIO_NUM_18) == 0) {
                    if (esp_timer_get_time() > timeout) return -1.0;
                }
                int64_t t_start = esp_timer_get_time();
                while (gpio_get_level(GPIO_NUM_18) == 1) {
                    if (esp_timer_get_time() > timeout) return -1.0;
                }
                int64_t t_end = esp_timer_get_time();
                double duration = (double)(t_end - t_start);
                double distance = duration * 0.017;
                return distance;
            });

        // Đăng ký tool robot_chassis.get_line_status
        mcp_server.AddTool("self.robot_chassis.get_line_status",
            "Read the state of the line tracking sensor (MH-B). Returns 1 if a black line is detected, and 0 if a white background is detected.",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                int val = gpio_get_level(GPIO_NUM_3);
                return val;
            });
    }

public:
    ZHENGCHEN_1_54TFT_WIFI() :
        boot_button_(BOOT_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializePowerManager();
        InitializePowerSaveTimer();
        InitializeSpi();
        InitializeButtons();
        InitializeSt7789Display();  
        InitializeTools();
        GetBacklight()->RestoreBrightness();
    }

    // 获取音频编解码器
    virtual AudioCodec* GetAudioCodec() override {
        // 静态实例化NoAudioCodecSimplex类
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        // 返回音频编解码器
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = power_manager_->IsCharging();
        discharging = power_manager_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }
        level = std::max<uint32_t>(power_manager_->GetBatteryLevel(), 20);
        return true;
    }

    virtual bool GetTemperature(float& esp32temp)  override {
        esp32temp = power_manager_->GetTemperature();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(ZHENGCHEN_1_54TFT_WIFI);
