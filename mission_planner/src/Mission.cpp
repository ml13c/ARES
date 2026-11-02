#include "Mission.h"

void Mission::addBoundaryPoint(const Waypoint& point)
{
    fieldBoundary.push_back(point);
}

void Mission::setRobotPosition(const Waypoint& point)
{
    robotPosition = point;
}

void Mission::determineStartPoint()
{
    // Temporary implementation
    startPoint = robotPosition;
}

void Mission::generateRoute()
{
    route.clear();

    // Temporary route: just start at the robot's position
    route.push_back(startPoint);
}

bool Mission::validateRoute() const
{
    // Temporary implementation
    return true;
}

const std::vector<Waypoint>& Mission::getRoute() const
{
    return route;
}

const std::vector<Waypoint>& Mission::getBoundary() const
{
    return fieldBoundary;
}