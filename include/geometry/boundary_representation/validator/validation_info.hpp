#pragma once

#include <string>
#include <vector>

namespace GraphicsLab::Geometry::BRep {

struct ValidationInfo {
    std::vector<std::string> error_messages;
};

} // namespace GraphicsLab::Geometry::BRep
