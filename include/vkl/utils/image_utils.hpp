#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace vkl {

struct ImageUtils {

   static uint8_t toSRGB(float value) {
       value = std::clamp(value, 0.0f, 1.0f);  // Clamp to [0, 1]
       if (value <= 0.0031308f)
           return static_cast<uint8_t>(value * 12.92f * 255.0f);
       else
           return static_cast<uint8_t>((1.055f * std::pow(value, 1.0f / 2.4f) - 0.055f) * 255.0f);
   }
};

}