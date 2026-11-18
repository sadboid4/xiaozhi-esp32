#include "duybot_display.h"
#include "config.h"

#include <cstring>
#include <src/misc/cache/instance/lv_image_cache.h>

void DuyBotDisplay::SetEmotion(const char* emotion) {
    SpiLcdDisplay::SetEmotion(emotion);
    if (emotion_hook_ && emotion != nullptr) {
        emotion_hook_(emotion);
    }
}

DuyBotDisplay::~DuyBotDisplay() {
    if (avatar_image_ != nullptr) {
        DisplayLockGuard lock(this);
        lv_obj_delete(avatar_image_);
        avatar_image_ = nullptr;
    }
}

void DuyBotDisplay::SetupUI() {
    if (setup_ui_called_) {
        return;
    }

    SpiLcdDisplay::SetupUI();
    DisplayLockGuard lock(this);

    for (auto& descriptor : avatar_descriptors_) {
        std::memset(&descriptor, 0, sizeof(descriptor));
        descriptor.header.magic = LV_IMAGE_HEADER_MAGIC;
        descriptor.header.cf = LV_COLOR_FORMAT_RGB565;
        descriptor.header.w = DISPLAY_WIDTH;
        descriptor.header.h = DISPLAY_HEIGHT;
        descriptor.header.stride = DISPLAY_WIDTH * sizeof(uint16_t);
        descriptor.data_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
    }

    avatar_image_ = lv_image_create(lv_screen_active());
    lv_obj_set_size(avatar_image_, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    lv_obj_align(avatar_image_, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(avatar_image_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_to_index(avatar_image_, 1);
}

bool DuyBotDisplay::ShowAvatarFrame(uint8_t* pixels, size_t size, int buffer_index) {
    if (avatar_image_ == nullptr || pixels == nullptr || size != DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t)) {
        return false;
    }

    auto& descriptor = avatar_descriptors_[buffer_index & 1];
    DisplayLockGuard lock(this);
    lv_image_cache_drop(&descriptor);
    descriptor.data = pixels;
    lv_image_set_src(avatar_image_, &descriptor);
    if (gif_controller_) {
        gif_controller_->Stop();
    }
    lv_obj_add_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(preview_image_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(avatar_image_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(avatar_image_);
    return true;
}
