#pragma once

namespace GraphicsLab {
struct IGraphicsLabImguiComponent {
    virtual ~IGraphicsLabImguiComponent() {
    }
    virtual void render() = 0;
};
} // namespace GraphicsLab