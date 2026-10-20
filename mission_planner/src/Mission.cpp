#include "Mission.h"


void Mission::addWaypoint(Waypoint point)
{
    waypoints.push_back(point);
}


std::vector<Waypoint> Mission::getWaypoints() const
{
    return waypoints;
}