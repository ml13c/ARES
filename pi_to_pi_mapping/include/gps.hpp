#pragma once

#include "localization.hpp"

#include <optional>
#include <string>

namespace ares {

// One valid NMEA GPS fix. Course is NOT a compass heading. It is only exposed
// when the receiver reports enough motion for course-over-ground to be useful.
struct GpsFix {
    GeoPoint position;
    std::optional<double> course_deg;
};

// Accepts checksummed GxRMC and GxGGA sentences with a valid fix.
// Invalid checksums, no-fix statuses and unsupported sentences yield nullopt.
std::optional<GpsFix> parseNmeaSentence(const std::string& line);

// POSIX serial reader for a USB/UART NMEA GPS connected to the rover Pi.
// No Python packages or GPS daemon required.
class GpsSerial {
public:
    GpsSerial() = default;
    ~GpsSerial();
    GpsSerial(const GpsSerial&) = delete;
    GpsSerial& operator=(const GpsSerial&) = delete;

    bool openDevice(const std::string& device, int baud, std::string& error);
    // Polls serial input for up to timeout_ms. nullopt means no valid fix yet.
    std::optional<GpsFix> nextFix(int timeout_ms);
    bool connected() const { return fd_ >= 0; }

private:
    int fd_ = -1;
    std::string pending_;
};

} // namespace ares
