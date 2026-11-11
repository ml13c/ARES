#include "grid_map.hpp"
#include "localization.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
//temporary map for this test
namespace ares {

GridMap::GridMap(int width, int height) : width_(width), height_(height) {}

bool GridMap::withinBounds(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool GridMap::updateRover(int x, int y) {
    if (!withinBounds(x, y)) return false;
    rover_x_ = x;
    rover_y_ = y;
    has_rover_ = true;
    return true;
}

bool GridMap::addHazard(const Message& hazard) {
    if (hazard.message_type != MessageType::Hazard ||
        !withinBounds(hazard.x, hazard.y)) return false;
    hazards_[hazard.id] = hazard;
    return true;
}

bool GridMap::updateGps(const Message& position) {
    if (position.message_type != MessageType::GpsPosition ||
        !position.has_geo ||
        !validGeoPoint({position.latitude_deg, position.longitude_deg})) return false;
    live_gps_ = position;
    gps_stale_ = false;
    if (!boundary_.empty() && gps_trace_.size() < 10000)
        gps_trace_.push_back({position.latitude_deg,position.longitude_deg});
    return true;
}

void GridMap::markGpsStale() { gps_stale_ = true; }

bool GridMap::addSighting(const Message& sighting) {
    if (sighting.message_type != MessageType::Sighting ||
        !sighting.has_geo ||
        !validGeoPoint({sighting.latitude_deg, sighting.longitude_deg})) return false;
    sightings_[sighting.id] = sighting;
    return true;
}

bool GridMap::addMissionPoint(const Message& point) {
    if (!point.has_geo || !validGeoPoint({point.latitude_deg,point.longitude_deg}))
        return false;
    if (point.message_type == MessageType::FieldBoundary) {
        if (point.id == 0 && !boundary_.empty()) resetMissionDisplay();
        if (point.id != boundary_.size() || boundary_.size() >= 128) return false;
        boundary_.push_back({point.latitude_deg,point.longitude_deg});
        return true;
    }
    if (point.message_type == MessageType::PathWaypoint) {
        if (point.id != waypoints_.size() || waypoints_.size() >= 1024) return false;
        waypoints_.push_back({point.latitude_deg,point.longitude_deg});
        return true;
    }
    return false;
}

bool GridMap::updateNavigation(const Message& status) {
    if (status.message_type != MessageType::NavigationStatus) return false;
    navigation_state_ = status.nav_state;
    navigation_target_ = status.id;
    has_navigation_status_ = true;
    return true;
}

void GridMap::resetMissionDisplay() {
    boundary_.clear();
    waypoints_.clear();
    gps_trace_.clear();
    live_gps_.reset();
    gps_stale_ = false;
    has_navigation_status_ = false;
}

std::string GridMap::renderGeoField() const {
    // Terminal-only schematic projection for small fields. Not a GIS app,
    // survey-grade display, or actual sensor/motor navigation.
    constexpr int width=52, height=18;
    std::vector<std::string> canvas(height,std::string(width,' '));
    const GeoPoint origin=boundary_[0];
    struct P {double x,y;};
    auto project = [&](GeoPoint geo, P& projected) -> bool {
        const auto p = toLocalOffset(origin,geo);
        if (!p) return false;
        projected={p->east_m,p->north_m};
        return true;
    };
    std::vector<P> boundary, route;
    for (auto g: boundary_) { P p{}; if (!project(g,p)) return "Invalid boundary projection\n"; boundary.push_back(p); }
    for (auto g: waypoints_) { P p{}; if (!project(g,p)) return "Invalid route projection\n"; route.push_back(p); }
    double xmin=boundary[0].x,xmax=xmin,ymin=boundary[0].y,ymax=ymin;
    for (const auto& p:boundary) {
        xmin=std::min(xmin,p.x); xmax=std::max(xmax,p.x);
        ymin=std::min(ymin,p.y); ymax=std::max(ymax,p.y);
    }
    if (xmax-xmin<0.1 || ymax-ymin<0.1) return "Degenerate field projection\n";
    xmin-=2.0; xmax+=2.0; ymin-=2.0; ymax+=2.0;
    auto cell=[&](P p) {
        const int col=std::clamp(static_cast<int>(std::lround((p.x-xmin)/(xmax-xmin)*(width-1))),0,width-1);
        const int row=std::clamp(static_cast<int>(std::lround((ymax-p.y)/(ymax-ymin)*(height-1))),0,height-1);
        return std::pair<int,int>{col,row};
    };
    auto put=[&](P p,char symbol) { const auto c=cell(p); canvas[c.second][c.first]=symbol; };
    auto line=[&](P a,P b,char symbol) {
        const auto u=cell(a), v=cell(b);
        const int steps=std::max(1,2*std::max(std::abs(u.first-v.first),std::abs(u.second-v.second)));
        for (int i=0;i<=steps;++i) {
            const double t=static_cast<double>(i)/steps;
            put({a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t},symbol);
        }
    };
    for (std::size_t i=0;i<boundary.size();++i)
        line(boundary[i],boundary[(i+1)%boundary.size()],'#');
    for (std::size_t i=1;i<route.size();++i)
        line(route[i-1],route[i],'.');
    for (auto p:route) put(p,'W');
    P last{};
    bool has_last=false;
    for (auto gps:gps_trace_) {
        P p{};
        if (!project(gps,p)) continue;
        if (has_last) line(last,p,'+');
        last=p; has_last=true;
    }
    for (const auto& entry:sightings_) {
        P p{};
        if (project({entry.second.latitude_deg,entry.second.longitude_deg},p)) put(p,'?');
    }
    if (live_gps_) {
        P p{};
        if (project({live_gps_->latitude_deg,live_gps_->longitude_deg},p)) put(p,'R');
    }
    std::ostringstream out;
    out << "ARES - GPS FIELD VIEW (schematic, SIMULATED mission)\n"
        << "# field boundary   . planned path   W waypoint   + rover trail\n"
        << "R rover GPS        ? sighting (object position UNKNOWN)\n\n";
    for (const auto& row:canvas) out << row << '\n';
    out << "\nField vertices: " << boundary_.size()
        << " | waypoints: " << waypoints_.size()
        << " | recorded GPS samples: " << gps_trace_.size() << '\n';
    if (has_navigation_status_)
        out << "Simulation navigation: " << navigationStateName(navigation_state_)
            << " | next waypoint index: " << navigation_target_ << '\n';
    if (live_gps_) {
        out << (gps_stale_ ? "LAST GPS rover (STALE - DO NOT NAVIGATE): " : "LIVE GPS rover: ")
            << std::fixed << std::setprecision(8)
            << live_gps_->latitude_deg << ", " << live_gps_->longitude_deg;
        if (live_gps_->has_heading) out << " | heading/course " << live_gps_->heading_deg << " deg";
        out << '\n';
    }
    out << "Sightings (rover GPS at detection; object location UNKNOWN): "
        << sightings_.size() << '\n';
    for (const auto& entry:sightings_) {
        const auto& s=entry.second;
        out << "  #" << s.id << ' ' << objectTypeName(s.object_type)
            << " seen at rover GPS (" << std::fixed << std::setprecision(8)
            << s.latitude_deg << ", " << s.longitude_deg << ")\n";
    }
    return out.str();
}

std::string GridMap::render() const {
    if (boundary_.size() >= 3 && waypoints_.size() >= 2) return renderGeoField();
    std::ostringstream out;
    out << "ARES — SIMULATED FIELD (" << width_ << " x " << height_ << ")\n";
    out << "X = rover, H = saved hazard, - = empty\n\n";

    // x increases to the right. y increases downward. Origin at top-left.
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            char marker = '-';
            for (const auto& entry : hazards_) {
                const Message& hazard = entry.second;
                if (hazard.x == x && hazard.y == y) {
                    marker = 'H';
                    break;
                }
            }
            if (has_rover_ && rover_x_ == x && rover_y_ == y) marker = 'X';
            out << marker;
        }
        out << '\n';
    }

    if (has_rover_) {
        out << "\nRover: (" << rover_x_ << ", " << rover_y_ << ")\n";
    } else {
        out << "\nRover: waiting for first position...\n";
    }
    out << "Saved hazards: " << hazards_.size() << '\n';
    for (const auto& entry : hazards_) {
        const Message& hazard = entry.second;
        out << "  #" << hazard.id << " " << objectTypeName(hazard.object_type)
            << " at grid (" << hazard.x << ", " << hazard.y << ")";
        if (hazard.has_geo) {
            out << std::fixed << std::setprecision(7)
                << " GPS (" << hazard.latitude_deg << ", "
                << hazard.longitude_deg << ")";
        }
        out << '\n';
    }
    if (live_gps_) {
        out << "\n" << (gps_stale_ ? "LAST GPS rover (STALE - DO NOT NAVIGATE): "
                                     : "LIVE GPS rover (not drawn on simulation grid): ")
            << std::fixed << std::setprecision(8)
            << live_gps_->latitude_deg << ", " << live_gps_->longitude_deg;
        if (live_gps_->has_heading)
            out << " | course over ground: " << live_gps_->heading_deg << " deg";
        else out << " | course: unavailable (stationary/unknown)";
        out << '\n';
    }
    if (gps_stale_ && !live_gps_)
        out << "\nGPS: NO VALID FIX - navigation must remain disabled\n";
    out << "Sightings (rover GPS at detection; object location UNKNOWN): "
        << sightings_.size() << '\n';
    for (const auto& entry : sightings_) {
        const auto& sighting = entry.second;
        out << "  #" << sighting.id << ' ' << objectTypeName(sighting.object_type)
            << " seen near rover GPS (" << std::fixed << std::setprecision(8)
            << sighting.latitude_deg << ", " << sighting.longitude_deg << ")\n";
    }
    return out.str();
}

} // namespace ares
