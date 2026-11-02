#include <iostream>
#include "Mission.h"

int main()
{
    std::cout << "ARES Mission Planner Started\n\n";

    Mission mission;

    // Create a simple rectangular field
    mission.addBoundaryPoint({33.2100, -97.1500, 1});
    mission.addBoundaryPoint({33.2100, -97.1450, 2});
    mission.addBoundaryPoint({33.2150, -97.1450, 3});
    mission.addBoundaryPoint({33.2150, -97.1500, 4});

    std::cout << "Field Boundary:\n\n";

    for (const Waypoint& point : mission.getBoundary())
    {
        std::cout << "Point " << point.order << "\n";
        std::cout << "Latitude: " << point.latitude << "\n";
        std::cout << "Longitude: " << point.longitude << "\n\n";
    }

    // Robot starts here
    mission.setRobotPosition({33.2110, -97.1490, 1});

    mission.determineStartPoint();

    mission.generateRoute();

    std::cout << "Route contains "
              << mission.getRoute().size()
              << " waypoint(s).\n";

    return 0;
}