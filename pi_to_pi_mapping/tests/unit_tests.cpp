#include "grid_map.hpp"
#include "localization.hpp"
#include "protocol.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {
int failures = 0;

void check(bool expression, const char* label, int line) {
    if (!expression) {
        std::cerr << "FAIL line " << line << ": " << label << '\n';
        ++failures;
    }
}
#define CHECK(expression) check((expression), #expression, __LINE__)

void testLocalization() {
    const ares::GeoPoint origin{37.0, -122.0};
    const ares::RobotPose facing_north{origin, 0.0};
    const ares::RobotPose facing_east{origin, 90.0};
    const auto north = ares::localizeObject(facing_north, 10.0, 0.0);
    const auto east = ares::localizeObject(facing_east, 10.0, 0.0);
    CHECK(north.has_value());
    CHECK(east.has_value());
    if (north && east) {
        const auto n = ares::toLocalOffset(origin, *north);
        const auto e = ares::toLocalOffset(origin, *east);
        CHECK(n.has_value());
        CHECK(e.has_value());
        if (n && e) {
            CHECK(std::abs(n->north_m - 10.0) < 0.02);
            CHECK(std::abs(n->east_m) < 0.02);
            CHECK(std::abs(e->east_m - 10.0) < 0.02);
            CHECK(std::abs(e->north_m) < 0.02);
        }
    }

    // A -90 degree relative bearing from an East-facing rover points North.
    const auto left = ares::localizeObject(facing_east, 8.0, -90.0);
    CHECK(left.has_value());
    if (left) {
        const auto local = ares::toLocalOffset(origin, *left);
        CHECK(local.has_value());
        if (local) {
            CHECK(std::abs(local->east_m) < 0.02);
            CHECK(std::abs(local->north_m - 8.0) < 0.02);
        }
    }

    const auto back = ares::localizeObject(facing_north, 5.0, 180.0);
    CHECK(back.has_value());
    if (back) {
        const auto local = ares::toLocalOffset(origin, *back);
        CHECK(local.has_value());
        if (local) CHECK(std::abs(local->north_m + 5.0) < 0.02);
    }

    CHECK(!ares::localizeObject(facing_north, -1.0, 0.0));
    CHECK(!ares::localizeObject(facing_north,
                               std::numeric_limits<double>::infinity(), 0.0));
    CHECK(!ares::localizeObject({{91.0, 0.0}, 0.0}, 1.0, 0.0));
    CHECK(!ares::fromLocalOffset({91.0, 0.0}, {1.0, 1.0}));
    CHECK(!ares::toLocalOffset(origin, {0.0, 200.0}));
}

void testProtocol() {
    ares::Message pos;
    pos.message_type = ares::MessageType::Position;
    pos.id = 25;
    pos.x = 5;
    pos.y = 2;
    pos.has_geo = true;
    pos.latitude_deg = 37.123456789;
    pos.longitude_deg = -122.123456789;
    pos.heading_deg = 90.0;

    std::string err;
    ares::Message decoded;
    const std::string encoded_pos = ares::encodeMessage(pos);
    CHECK(!encoded_pos.empty() && encoded_pos.back() == '\n');
    CHECK(ares::decodeMessage(encoded_pos.substr(0, encoded_pos.size() - 1), decoded, err));
    CHECK(decoded.message_type == ares::MessageType::Position);
    CHECK(decoded.id == 25 && decoded.x == 5 && decoded.y == 2);
    CHECK(decoded.has_geo);
    CHECK(std::abs(decoded.latitude_deg - pos.latitude_deg) < 1e-9);
    CHECK(std::abs(decoded.longitude_deg - pos.longitude_deg) < 1e-9);
    CHECK(std::abs(decoded.heading_deg - 90.0) < 1e-9);

    ares::Message hazard;
    hazard.message_type = ares::MessageType::Hazard;
    hazard.id = 101;
    hazard.object_type = ares::ObjectType::Rock;
    hazard.x = 6;
    hazard.y = 2;
    hazard.has_geo = true;
    hazard.latitude_deg = 37.0;
    hazard.longitude_deg = -122.0;
    hazard.range_m = 1.5;
    hazard.relative_bearing_deg = -15;
    const std::string encoded_hazard = ares::encodeMessage(hazard);
    CHECK(ares::decodeMessage(encoded_hazard.substr(0, encoded_hazard.size() - 1),
                              decoded, err));
    CHECK(decoded.message_type == ares::MessageType::Hazard);
    CHECK(decoded.object_type == ares::ObjectType::Rock);
    CHECK(decoded.id == 101 && decoded.has_geo);
    CHECK(std::abs(decoded.range_m - 1.5) < 1e-9);
    CHECK(std::abs(decoded.relative_bearing_deg + 15) < 1e-9);

    // Legacy messages are still decoded for compatibility.
    CHECK(ares::decodeMessage("POS,3,1,2", decoded, err));
    CHECK(!decoded.has_geo && decoded.id == 3);
    CHECK(ares::decodeMessage("HAZARD,7,WEED,3,4", decoded, err));
    CHECK(!decoded.has_geo && decoded.object_type == ares::ObjectType::Weed);

    CHECK(!ares::decodeMessage("POS,3,1,2,200,0,90", decoded, err));
    CHECK(!ares::decodeMessage("POS,3,1,2,nan,-122,90", decoded, err));
    CHECK(!ares::decodeMessage("POS,3,1,2,37,-122,360", decoded, err));
    CHECK(!ares::decodeMessage("HAZARD,7,WEED,3,4,37,-122,-2,5", decoded, err));
    CHECK(!ares::decodeMessage("HAZARD,7,WEED,3,4,37,-122,2,181", decoded, err));
    CHECK(!ares::decodeMessage("HAZARD,7,UNKNOWN,3,4", decoded, err));
    CHECK(!ares::decodeMessage("HAZARD,7,ROCK,999999999999999999999,4", decoded, err));
    CHECK(!ares::decodeMessage("POS,4,2,3,5", decoded, err));
}

void testGrid() {
    ares::GridMap map(20, 10);
    CHECK(map.updateRover(0, 0));
    CHECK(!map.updateRover(20, 0));
    ares::Message m;
    m.message_type = ares::MessageType::Hazard;
    m.id = 101;
    m.object_type = ares::ObjectType::Rock;
    m.x = 1;
    m.y = 0;
    m.has_geo = true;
    m.latitude_deg = 37;
    m.longitude_deg = -122;
    CHECK(map.addHazard(m));
    CHECK(map.render().find("GPS (37.0000000, -122.0000000)") != std::string::npos);
    m.x = -1;
    CHECK(!map.addHazard(m));
}
} // namespace

int main() {
    testLocalization();
    testProtocol();
    testGrid();
    if (failures != 0) {
        std::cerr << failures << " checks failed\n";
        return 1;
    }
    std::cout << "PASS: localization, message protocol, invalid inputs and grid map\n";
    return 0;
}
