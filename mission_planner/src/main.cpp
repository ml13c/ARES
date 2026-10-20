#include <iostream>

#include "Mission.h"
#include "RobotConfig.h"


int main()
{

    std::cout << "ARES Mission Planner Started\n";


    Mission mission;


    Waypoint startPoint;

    startPoint.latitude = 33.2100;
    startPoint.longitude = -97.1500;
    startPoint.order = 1;


    mission.addWaypoint(startPoint);


    std::cout 
        << "Number of waypoints: "
        << mission.getWaypoints().size()
        << std::endl;


    return 0;
}