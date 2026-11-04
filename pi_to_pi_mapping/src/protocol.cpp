#include "protocol.hpp"
#include "localization.hpp"

#include <charconv>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace ares {
namespace {
std::vector<std::string> splitCommaSeparated(const std::string& line) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true) {
        const std::size_t end = line.find(',', start);
        if (end == std::string::npos) {
            parts.push_back(line.substr(start));
            return parts;
        }
        parts.push_back(line.substr(start, end - start));
        start = end + 1;
    }
}

template <typename T>
bool parseInteger(const std::string& value, T& parsed) {
    if (value.empty()) return false;
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    return result.ec == std::errc{} && result.ptr == value.data() + value.size();
}

bool parseDouble(const std::string& value, double& parsed) {
    if (value.empty()) return false;
    // stod rejects some input, so also check full consumption and finiteness.
    try {
        std::size_t consumed = 0;
        parsed = std::stod(value, &consumed);
        return consumed == value.size() && std::isfinite(parsed);
    } catch (...) {
        return false;
    }
}

bool parseGeo(const std::string& lat, const std::string& lon, Message& m) {
    if (!parseDouble(lat, m.latitude_deg) || !parseDouble(lon, m.longitude_deg) ||
        !validGeoPoint({m.latitude_deg, m.longitude_deg})) return false;
    m.has_geo = true;
    return true;
}
} // namespace

const char* objectTypeName(ObjectType type) {
    switch (type) {
        case ObjectType::Rock: return "ROCK";
        case ObjectType::Weed: return "WEED";
        case ObjectType::Other: return "OTHER";
        case ObjectType::None: return "NONE";
    }
    return "UNKNOWN";
}

const char* navigationStateName(NavigationState state) {
    switch (state) {
        case NavigationState::Running: return "RUNNING";
        case NavigationState::Complete: return "COMPLETE";
        case NavigationState::Fault: return "FAULT";
    }
    return "FAULT";
}

std::string encodeMessage(const Message& message) {
    std::ostringstream out;
    out << std::setprecision(12);
    if (message.message_type == MessageType::FieldBoundary ||
        message.message_type == MessageType::PathWaypoint) {
        out << (message.message_type == MessageType::FieldBoundary ? "BOUNDARY" : "WAYPOINT")
            << ',' << message.id << ',' << message.latitude_deg
            << ',' << message.longitude_deg;
    } else if (message.message_type == MessageType::NavigationStatus) {
        out << "NAV," << navigationStateName(message.nav_state) << ',' << message.id;
    } else if (message.message_type == MessageType::GpsStatus) {
        out << "GPS_STATUS,NO_FIX";
    } else if (message.message_type == MessageType::GpsPosition) {
        out << "GPS," << message.id << ',' << message.latitude_deg
            << ',' << message.longitude_deg;
        if (message.has_heading) out << ',' << message.heading_deg;
    } else if (message.message_type == MessageType::Sighting) {
        out << "SIGHTING," << message.id << ',' << objectTypeName(message.object_type)
            << ',' << message.latitude_deg << ',' << message.longitude_deg;
    } else if (message.message_type == MessageType::Position) {
        out << "POS," << message.id << ',' << message.x << ',' << message.y;
        if (message.has_geo) {
            out << ',' << message.latitude_deg << ',' << message.longitude_deg
                << ',' << message.heading_deg;
        }
    } else {
        out << "HAZARD," << message.id << ',' << objectTypeName(message.object_type)
            << ',' << message.x << ',' << message.y;
        if (message.has_geo) {
            out << ',' << message.latitude_deg << ',' << message.longitude_deg
                << ',' << message.range_m << ',' << message.relative_bearing_deg;
        }
    }
    return out.str() + '\n';
}

bool decodeMessage(const std::string& raw, Message& output, std::string& error) {
    std::string line = raw;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const auto parts = splitCommaSeparated(line);
    Message parsed;

    if (parts.size() == 4 && (parts[0] == "BOUNDARY" || parts[0] == "WAYPOINT")) {
        parsed.message_type = parts[0] == "BOUNDARY" ? MessageType::FieldBoundary
                                                       : MessageType::PathWaypoint;
        if (!parseInteger(parts[1],parsed.id) || parsed.id > 1024 ||
            !parseGeo(parts[2],parts[3],parsed)) {
            error = "Invalid mission point";
            return false;
        }
    } else if (parts.size() == 3 && parts[0] == "NAV") {
        parsed.message_type = MessageType::NavigationStatus;
        if (parts[1] == "RUNNING") parsed.nav_state = NavigationState::Running;
        else if (parts[1] == "COMPLETE") parsed.nav_state = NavigationState::Complete;
        else if (parts[1] == "FAULT") parsed.nav_state = NavigationState::Fault;
        else { error = "Unknown navigation state"; return false; }
        if (!parseInteger(parts[2],parsed.id) || parsed.id > 1024) {
            error = "Invalid next waypoint index"; return false;
        }
    } else if (parts.size() == 2 && parts[0] == "GPS_STATUS" && parts[1] == "NO_FIX") {
        parsed.message_type = MessageType::GpsStatus;
    } else if (!parts.empty() && parts[0] == "GPS" &&
        (parts.size() == 4 || parts.size() == 5)) {
        parsed.message_type = MessageType::GpsPosition;
        if (!parseInteger(parts[1], parsed.id) ||
            !parseGeo(parts[2], parts[3], parsed)) {
            error = "Invalid GPS position";
            return false;
        }
        if (parts.size() == 5) {
            if (!parseDouble(parts[4], parsed.heading_deg) ||
                parsed.heading_deg < 0 || parsed.heading_deg >= 360) {
                error = "Invalid GPS course";
                return false;
            }
            parsed.has_heading = true;
        }
    } else if (!parts.empty() && parts[0] == "SIGHTING" && parts.size() == 5) {
        parsed.message_type = MessageType::Sighting;
        if (parts[2] == "ROCK") parsed.object_type = ObjectType::Rock;
        else if (parts[2] == "WEED") parsed.object_type = ObjectType::Weed;
        else if (parts[2] == "OTHER") parsed.object_type = ObjectType::Other;
        else {
            error = "Unknown sighting class";
            return false;
        }
        if (!parseInteger(parts[1], parsed.id) ||
            !parseGeo(parts[3], parts[4], parsed)) {
            error = "Invalid sighting position";
            return false;
        }
    } else if (!parts.empty() && parts[0] == "POS" && (parts.size() == 4 || parts.size() == 7)) {
        parsed.message_type = MessageType::Position;
        if (!parseInteger(parts[1], parsed.id) || !parseInteger(parts[2], parsed.x) ||
            !parseInteger(parts[3], parsed.y)) {
            error = "Invalid POS numeric field";
            return false;
        }
        if (parts.size() == 7 &&
            (!parseGeo(parts[4], parts[5], parsed) ||
             !parseDouble(parts[6], parsed.heading_deg) ||
             parsed.heading_deg < 0.0 || parsed.heading_deg >= 360.0)) {
            error = "Invalid POS latitude/longitude/heading";
            return false;
        }
    } else if (!parts.empty() && parts[0] == "HAZARD" &&
               (parts.size() == 5 || parts.size() == 9)) {
        parsed.message_type = MessageType::Hazard;
        if (parts[2] == "ROCK") parsed.object_type = ObjectType::Rock;
        else if (parts[2] == "WEED") parsed.object_type = ObjectType::Weed;
        else if (parts[2] == "OTHER") parsed.object_type = ObjectType::Other;
        else {
            error = "Unknown hazard class";
            return false;
        }
        if (!parseInteger(parts[1], parsed.id) || !parseInteger(parts[3], parsed.x) ||
            !parseInteger(parts[4], parsed.y)) {
            error = "Invalid HAZARD numeric field";
            return false;
        }
        if (parts.size() == 9 &&
            (!parseGeo(parts[5], parts[6], parsed) ||
             !parseDouble(parts[7], parsed.range_m) ||
             parsed.range_m < 0.0 ||
             !parseDouble(parts[8], parsed.relative_bearing_deg) ||
             parsed.relative_bearing_deg < -180.0 ||
             parsed.relative_bearing_deg > 180.0)) {
            error = "Invalid HAZARD location/range/bearing";
            return false;
        }
    } else {
        error = "Expected GPS, SIGHTING, POS, HAZARD, BOUNDARY, WAYPOINT or NAV";
        return false;
    }

    output = parsed;
    return true;
}

} // namespace ares
