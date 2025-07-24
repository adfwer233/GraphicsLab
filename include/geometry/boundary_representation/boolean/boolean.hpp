#pragma once

namespace GraphicsLab::Geometry::BRep {

/**
 * @brief Boolean operation between solids
 *
 * @note
 *
 * 1. intersect between solids, get the intersection graph of each face.
 * 2. split faces
 * 3. inside/outside classification
 * 4. rebuild topology
 *
 */
struct Boolean {};

} // namespace GraphicsLab::Geometry::BRep