#pragma once

#include "display/lcd_display.h"

#include <array>
#include <functional>

class DuyBotDisplay : public SpiLcdDisplay {
public:
    using SpiLcdDisplay::SpiLcdDisplay;
    ~DuyBotDisplay() override;

    void SetupUI() override;
    bool ShowAvatarFrame(uint8_t* pixels, size_t size, int buffer_index);

    void SetEmotion(const char* emotion) override;
    void SetEmotionHook(std::function<void(const char*)> hook) { emotion_hook_ = std::move(hook); }

private:
    lv_obj_t* avatar_image_ = nullptr;
    std::array<lv_image_dsc_t, 2> avatar_descriptors_{};
    std::function<void(const char*)> emotion_hook_;
};
