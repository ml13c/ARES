#ifndef MISSION_H
#define MISSION_H

#include <vector>

#include "Waypoint.h"

class Mission
{
private:

    std::vector<Waypoint> fieldBoundary;

    std::vector<Waypoint> route;

    Waypoint robotPosition;

    Waypoint startPoint;

public:

    void addBoundaryPoint(const Waypoint& point);

    void setRobotPosition(const Waypoint& point);

    void determineStartPoint();

    void generateRoute();

    bool validateRoute() const;

    const std::vector<Waypoint>& getRoute() const;

    const std::vector<Waypoint>& getBoundary() const;
};

#endif