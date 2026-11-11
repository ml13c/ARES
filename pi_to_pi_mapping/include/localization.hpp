#pragma once

#include <optional>

namespace ares {

// Local, small-field approximation. Intended for simulations and short ranges,
// NOT centimeter-level GPS/RTK positioning or global navigation.
struct GeoPoint {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
};

struct RobotPose {
    GeoPoint location;
    // Compass heading: 0 = North, 90 = East, 180 = South, 270 = West.
    double heading_deg = 0.0;
};

struct LocalOffset {
    double east_m = 0.0;
    double north_m = 0.0;
};

// Convert offsets relative to a nearby latitude/longitude reference.
std::optional<GeoPoint> fromLocalOffset(GeoPoint origin, LocalOffset meters);
std::optional<LocalOffset> toLocalOffset(GeoPoint origin, GeoPoint point);

// The relative bearing is measured clockwise from the rover's forward axis.
// A 0-degree bearing means the object is straight ahead of the rover.
std::optional<GeoPoint> localizeObject(const RobotPose& rover,
                                      double range_m,
                                      double relative_bearing_deg);

bool validGeoPoint(GeoPoint p);

} // namespace ares
