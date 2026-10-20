#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "Waypoint.h"


class Mission
{

private:

    std::vector<Waypoint> waypoints;


public:

    void addWaypoint(Waypoint point);

    std::vector<Waypoint> getWaypoints();

};


#endif