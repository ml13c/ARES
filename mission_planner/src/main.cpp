#include <iostream>
#include "Mission.h"

int main()
{
    std::cout << "ARES Mission Planner Started\n\n";

    RobotConfig robot;

    robot.robotWidth = 10.0;      // meters
    robot.safetyMargin = 5.0;    // meters
    robot.maxSpeed = 1.0;         // meters/sec


    Mission mission(robot);

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

    mission.exportRoute("mission_map.csv");

    std::cout << "Route contains "
              << mission.getRoute().size()
              << " waypoint(s).\n";

    std::cout << "\nGenerated Route:\n\n";


    for(const Waypoint& point : mission.getRoute())
    {
    std::cout << "Waypoint "
              << point.order
              << "\n";

    std::cout << "Latitude: "
              << point.latitude
              << "\n";

    std::cout << "Longitude: "
              << point.longitude
              << "\n\n";
    }

    std::cout << "Field Limits:\n\n";

    std::cout << "Robot Sweep Spacing: "
          << mission.getSweepSpacing()
          << " meters\n\n";

    std::cout << "North: "
          << mission.getMaxLatitude()
          << "\n";

    std::cout << "South: "
          << mission.getMinLatitude()
          << "\n";

    std::cout << "East: "
          << mission.getMaxLongitude()
          << "\n";

    std::cout << "West: "
          << mission.getMinLongitude()
          << "\n\n";

    return 0;
}