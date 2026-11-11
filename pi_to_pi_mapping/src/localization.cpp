#include "localization.hpp"

#include <cmath>

namespace ares {
namespace {
constexpr double kEarthRadiusMeters = 6371000.0;
constexpr double kPi = 3.14159265358979323846;
double radians(double degrees) { return degrees * kPi / 180.0; }
double degrees(double rad) { return rad * 180.0 / kPi; }
} // namespace

bool validGeoPoint(GeoPoint p) {
    return std::isfinite(p.latitude_deg) && std::isfinite(p.longitude_deg) &&
           p.latitude_deg >= -90.0 && p.latitude_deg <= 90.0 &&
           p.longitude_deg >= -180.0 && p.longitude_deg <= 180.0;
}

std::optional<GeoPoint> fromLocalOffset(GeoPoint origin, LocalOffset meters) {
    if (!validGeoPoint(origin) || !std::isfinite(meters.east_m) ||
        !std::isfinite(meters.north_m)) return std::nullopt;

    const double latitude = origin.latitude_deg +
                            degrees(meters.north_m / kEarthRadiusMeters);
    const double mid_latitude = (origin.latitude_deg + latitude) / 2.0;
    const double scale = std::cos(radians(mid_latitude));
    if (std::abs(scale) < 1e-8) return std::nullopt;

    const double longitude = origin.longitude_deg +
                             degrees(meters.east_m / (kEarthRadiusMeters * scale));
    GeoPoint result{latitude, longitude};
    if (!validGeoPoint(result)) return std::nullopt;
    return result;
}

std::optional<LocalOffset> toLocalOffset(GeoPoint origin, GeoPoint point) {
    if (!validGeoPoint(origin) || !validGeoPoint(point)) return std::nullopt;
    // The short-field approximation does not support crossing the antimeridian.
    if (std::abs(point.longitude_deg - origin.longitude_deg) > 180.0) return std::nullopt;
    const double mid_latitude = (origin.latitude_deg + point.latitude_deg) / 2.0;
    const double scale = std::cos(radians(mid_latitude));
    if (std::abs(scale) < 1e-8) return std::nullopt;
    return LocalOffset{
        radians(point.longitude_deg - origin.longitude_deg) *
            kEarthRadiusMeters * scale,
        radians(point.latitude_deg - origin.latitude_deg) * kEarthRadiusMeters
    };
}

std::optional<GeoPoint> localizeObject(const RobotPose& rover,
                                      double range_m,
                                      double relative_bearing_deg) {
    if (!validGeoPoint(rover.location) || !std::isfinite(rover.heading_deg) ||
        !std::isfinite(range_m) || range_m < 0.0 ||
        !std::isfinite(relative_bearing_deg)) return std::nullopt;
    const double absolute_bearing = radians(rover.heading_deg + relative_bearing_deg);
    return fromLocalOffset(rover.location,
                           {range_m * std::sin(absolute_bearing),
                            range_m * std::cos(absolute_bearing)});
}

} // namespace ares
