#pragma once

struct ParametricSpaceUBO {
    float zoom = 1.0f;
    float offset_x = 0.0f;
    float offset_y = 0.0f;
};

struct UIState {
    ParametricSpaceUBO ubo;
};