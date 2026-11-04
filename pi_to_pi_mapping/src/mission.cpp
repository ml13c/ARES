#include "mission.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace ares {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;
struct Vec { double x, y; };

Vec operator-(Vec a, Vec b) { return {a.x - b.x, a.y - b.y}; }
double cross(Vec a, Vec b) { return a.x * b.y - a.y * b.x; }
double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y; }
double length(Vec a) { return std::hypot(a.x, a.y); }

double pointSegmentDistance(Vec p, Vec a, Vec b) {
    const Vec d = b - a;
    const double denom = dot(d, d);
    if (denom <= kEps) return length(p - a);
    const double t = std::clamp(dot(p - a, d) / denom, 0.0, 1.0);
    return length(p - Vec{a.x + t * d.x, a.y + t * d.y});
}

int orientation(Vec a, Vec b, Vec c) {
    const double area = cross(b - a, c - a);
    return area > kEps ? 1 : (area < -kEps ? -1 : 0);
}
bool onSegment(Vec a, Vec b, Vec p) {
    return std::min(a.x, b.x) - kEps <= p.x && p.x <= std::max(a.x, b.x) + kEps &&
           std::min(a.y, b.y) - kEps <= p.y && p.y <= std::max(a.y, b.y) + kEps;
}
bool segmentsIntersect(Vec a, Vec b, Vec c, Vec d) {
    const int o1 = orientation(a, b, c), o2 = orientation(a, b, d);
    const int o3 = orientation(c, d, a), o4 = orientation(c, d, b);
    if (o1 != o2 && o3 != o4) return true;
    return (o1 == 0 && onSegment(a,b,c)) || (o2 == 0 && onSegment(a,b,d)) ||
           (o3 == 0 && onSegment(c,d,a)) || (o4 == 0 && onSegment(c,d,b));
}
double segmentDistance(Vec a, Vec b, Vec c, Vec d) {
    if (segmentsIntersect(a,b,c,d)) return 0.0;
    return std::min({pointSegmentDistance(a,c,d), pointSegmentDistance(b,c,d),
                     pointSegmentDistance(c,a,b), pointSegmentDistance(d,a,b)});
}

bool toLocalPolygon(const Mission& mission, std::vector<Vec>& polygon) {
    if (mission.boundary.size() < 3) return false;
    polygon.clear();
    for (GeoPoint geo : mission.boundary) {
        const auto p = toLocalOffset(mission.boundary[0], geo);
        if (!p || std::abs(p->east_m) > 10000 || std::abs(p->north_m) > 10000)
            return false; // single, small local field; no world-spanning polygons
        polygon.push_back({p->east_m, p->north_m});
    }
    return true;
}

bool insidePolygon(const std::vector<Vec>& poly, Vec p) {
    bool inside = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const Vec a = poly[i], b = poly[j];
        if (pointSegmentDistance(p, a, b) < 1e-7) return false; // boundary not safe
        if ((a.y > p.y) != (b.y > p.y)) {
            const double x = (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x;
            if (p.x < x) inside = !inside;
        }
    }
    return inside;
}

bool safePoint(const std::vector<Vec>& poly, Vec p, double clearance_m) {
    if (!insidePolygon(poly,p)) return false;
    for (std::size_t i = 0; i < poly.size(); ++i)
        if (pointSegmentDistance(p,poly[i],poly[(i+1)%poly.size()]) < clearance_m-1e-7)
            return false;
    return true;
}

bool safeSegment(const std::vector<Vec>& poly, Vec a, Vec b, double clearance_m) {
    if (!safePoint(poly,a,clearance_m) || !safePoint(poly,b,clearance_m)) return false;
    for (std::size_t i = 0; i < poly.size(); ++i)
        if (segmentDistance(a,b,poly[i],poly[(i+1)%poly.size()]) < clearance_m-1e-7)
            return false;
    return true;
}

std::string trim(std::string s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last-first+1);
}
bool parseNumber(const std::string& text, double& result) {
    try {
        std::size_t consumed = 0;
        result = std::stod(text,&consumed);
        return consumed == text.size() && std::isfinite(result);
    } catch (...) { return false; }
}
} // namespace

bool loadMissionCsv(const std::string& filename, Mission& mission, std::string& error) {
    std::ifstream in(filename);
    if (!in) { error = "Cannot open mission file: " + filename; return false; }
    Mission candidate;
    std::string line;
    std::size_t line_num = 0;
    while (std::getline(in,line)) {
        ++line_num;
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string type, lat, lon, extra;
        if (!std::getline(ss,type,',') || !std::getline(ss,lat,',') ||
            !std::getline(ss,lon,',') || std::getline(ss,extra,',')) {
            error = "Invalid mission CSV at line " + std::to_string(line_num);
            return false;
        }
        type = trim(type); lat = trim(lat); lon = trim(lon);
        GeoPoint point;
        if (!parseNumber(lat,point.latitude_deg) ||
            !parseNumber(lon,point.longitude_deg) || !validGeoPoint(point)) {
            error = "Invalid coordinate at line " + std::to_string(line_num);
            return false;
        }
        if (type == "BOUNDARY") candidate.boundary.push_back(point);
        else if (type == "WAYPOINT") candidate.waypoints.push_back(point);
        else { error = "Expected BOUNDARY or WAYPOINT at line " + std::to_string(line_num); return false; }
        if (candidate.boundary.size() > 128 || candidate.waypoints.size() > 1024) {
            error = "Mission exceeds allowed point count"; return false;
        }
    }
    if (!in.eof() && in.fail()) { error = "Mission file read error"; return false; }
    mission = std::move(candidate);
    return true;
}

bool validateMission(const Mission& mission, double clearance_m, std::string& error) {
    if (!std::isfinite(clearance_m) || clearance_m < 0.25 || clearance_m > 100) {
        error = "Geofence safety margin must be between 0.25 and 100 meters";
        return false;
    }
    if (mission.boundary.size() < 3 || mission.waypoints.size() < 2) {
        error = "Mission requires at least 3 BOUNDARY points and 2 WAYPOINT points";
        return false;
    }
    std::vector<Vec> poly;
    if (!toLocalPolygon(mission,poly)) { error = "Unsupported/invalid field coordinates"; return false; }
    for (std::size_t i = 0; i < poly.size(); ++i) {
        if (length(poly[(i+1)%poly.size()] - poly[i]) < 0.1) {
            error = "Repeated or degenerate field boundary vertices";
            return false;
        }
        for (std::size_t j = i+1; j < poly.size(); ++j) {
            // Adjacent polygon sides necessarily touch at a shared vertex.
            if (j == i+1 || (i == 0 && j+1 == poly.size())) continue;
            if (segmentsIntersect(poly[i],poly[(i+1)%poly.size()],
                                  poly[j],poly[(j+1)%poly.size()])) {
                error = "Self-intersecting field boundary";
                return false;
            }
        }
    }
    double area2 = 0;
    for (std::size_t i=0;i<poly.size();++i) area2 += cross(poly[i],poly[(i+1)%poly.size()]);
    if (std::abs(area2) < 1.0) { error = "Field polygon area too small"; return false; }

    std::vector<Vec> points;
    for (const auto& geo : mission.waypoints) {
        const auto local = toLocalOffset(mission.boundary[0],geo);
        if (!local || std::abs(local->east_m) > 10000 || std::abs(local->north_m) > 10000) {
            error = "Invalid waypoint coordinates"; return false;
        }
        Vec p{local->east_m,local->north_m};
        if (!safePoint(poly,p,clearance_m)) {
            error = "Waypoint outside field or within geofence safety margin"; return false;
        }
        points.push_back(p);
    }
    for (std::size_t i=1;i<points.size();++i) {
        if (!safeSegment(poly,points[i-1],points[i],clearance_m)) {
            error = "Waypoint segment leaves field or violates geofence margin";
            return false;
        }
    }
    return true;
}

bool insideFieldWithMargin(const Mission& mission, GeoPoint p, double clearance_m) {
    std::vector<Vec> poly;
    if (clearance_m < 0.25 || !toLocalPolygon(mission,poly)) return false;
    const auto position = toLocalOffset(mission.boundary[0],p);
    if (!position) return false;
    return safePoint(poly,{position->east_m,position->north_m},clearance_m);
}

bool stepSimulatedNavigation(const Mission& mission, SimulatedNavigation& state,
                             double step_m, double clearance_m) {
    if (state.complete || state.fault) return !state.fault;
    if (!std::isfinite(step_m) || step_m <= 0 ||
        !insideFieldWithMargin(mission,state.position,clearance_m)) {
        state.fault = true;
        return false;
    }
    double remaining = step_m;
    while (remaining > 1e-8 && state.next_waypoint < mission.waypoints.size()) {
        const auto offset = toLocalOffset(state.position,mission.waypoints[state.next_waypoint]);
        if (!offset) { state.fault = true; return false; }
        const double distance = std::hypot(offset->east_m,offset->north_m);
        if (distance < 0.02) { ++state.next_waypoint; continue; }
        const double move = std::min(remaining,distance);
        const double ratio = move/distance;
        const auto target = fromLocalOffset(state.position,
            {offset->east_m*ratio,offset->north_m*ratio});
        if (!target || !insideFieldWithMargin(mission,*target,clearance_m)) {
            state.fault = true;
            return false;
        }
        state.position = *target;
        const double angle = std::atan2(offset->east_m,offset->north_m)*180.0/kPi;
        state.heading_deg = std::fmod(angle+360.0,360.0);
        remaining -= move;
        if (distance-move < 0.02) ++state.next_waypoint;
    }
    if (state.next_waypoint >= mission.waypoints.size()) state.complete = true;
    return true;
}
} // namespace ares
