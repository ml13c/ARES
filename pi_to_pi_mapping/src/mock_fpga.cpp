#include "mock_fpga.hpp"

#include <random>

namespace ares {

MockFPGA::MockFPGA() : rng_(std::random_device{}()) {
    // First detection occurs after 3 to 7 rover-position messages.
    next_detection_step_ = std::uniform_int_distribution<std::uint32_t>(3, 7)(rng_);
}

std::optional<MockDetection> MockFPGA::poll(std::uint32_t rover_step) {
    if (rover_step < next_detection_step_) {
        return std::nullopt;
    }

    // Subsequent detections every 3-7 rover updates, chosen randomly.
    next_detection_step_ = rover_step +
        std::uniform_int_distribution<std::uint32_t>(3, 7)(rng_);

    const int choice = std::uniform_int_distribution<int>(0, 2)(rng_);
    MockDetection detection;
    detection.id = next_hazard_id_++;
    detection.type = (choice == 0) ? ObjectType::Rock
                   : (choice == 1) ? ObjectType::Weed
                                   : ObjectType::Other;
    return detection;
}

} // namespace ares
