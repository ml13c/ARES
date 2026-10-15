#pragma once

#include "protocol.hpp"

#include <cstdint>
#include <map>
#include <string>

namespace ares {

class GridMap {
public:
    GridMap(int width, int height);

    // Return false if a coordinate falls outside the field.
    bool updateRover(int x, int y);
    bool addHazard(const Message& hazard);
    std::string render() const;

private:
    bool withinBounds(int x, int y) const;

    int width_;
    int height_;
    bool has_rover_ = false;
    int rover_x_ = 0;
    int rover_y_ = 0;
    std::map<std::uint32_t, Message> hazards_;
};

} // namespace ares
