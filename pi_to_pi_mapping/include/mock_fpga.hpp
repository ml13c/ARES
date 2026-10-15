#pragma once

#include "protocol.hpp"

#include <cstdint>
#include <optional>
#include <random>

namespace ares {

// Only simulates the FPGA's DETECTION/CLASSIFICATION output.
// No grid coordinates, GPS, ranges, or camera images are generated here.
struct MockDetection {
    std::uint32_t id = 0;
    ObjectType type = ObjectType::None;
};

class MockFPGA {
public:
    MockFPGA();
    std::optional<MockDetection> poll(std::uint32_t rover_step);

private:
    std::mt19937 rng_;
    std::uint32_t next_detection_step_ = 0;
    std::uint32_t next_hazard_id_ = 101;
};

} // namespace ares
