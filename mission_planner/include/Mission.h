#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include <string>

#include "Waypoint.h"
#include "RobotConfig.h"

class Mission
{
private:

    std::vector<Waypoint> fieldBoundary;

    std::vector<Waypoint> route;

    Waypoint robotPosition;

    Waypoint startPoint;

    RobotConfig robotConfig;
    
    double minLatitude() const;
    double maxLatitude() const;

    double minLongitude() const;
    double maxLongitude() const;

public:

    Mission(const RobotConfig& config);
    
    void addBoundaryPoint(const Waypoint& point);

    void setRobotPosition(const Waypoint& point);

    void determineStartPoint();

    void generateRoute();

    void exportRoute(const std::string& filename) const;

    bool validateRoute() const;

    const std::vector<Waypoint>& getRoute() const;

    const std::vector<Waypoint>& getBoundary() const;

    double getMinLatitude() const;
    double getMaxLatitude() const;

    double getMinLongitude() const;
    double getMaxLongitude() const;

    double getSweepSpacing() const;

    double metersToLatitude(double meters) const;
    double metersToLongitude(double meters) const;
};

#endif