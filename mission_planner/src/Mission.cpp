#include "Mission.h"
#include <fstream>

Mission::Mission(const RobotConfig& config)
{
    robotConfig = config;
}

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


    double spacing = getSweepSpacing();


    double latStep = metersToLatitude(spacing);


    double south = getMinLatitude();
    double north = getMaxLatitude();

    double west = getMinLongitude();
    double east = getMaxLongitude();


    double currentLat = south;

    bool leftToRight = true;

    int order = 1;


    while(currentLat <= north)
    {

        if(leftToRight)
        {
            route.push_back(
                {
                    currentLat,
                    west,
                    order++
                }
            );


            route.push_back(
                {
                    currentLat,
                    east,
                    order++
                }
            );
        }

        else
        {
            route.push_back(
                {
                    currentLat,
                    east,
                    order++
                }
            );


            route.push_back(
                {
                    currentLat,
                    west,
                    order++
                }
            );
        }


        currentLat += latStep;

        leftToRight = !leftToRight;
    }
}

void Mission::exportRoute(const std::string& filename) const
{
    std::ofstream file(filename);

    file << "latitude,longitude,type\n";


    // Boundary
    for(const auto& point : fieldBoundary)
    {
        file << point.latitude << ","
             << point.longitude
             << ",boundary\n";
    }


    // Route
    for(const auto& point : route)
    {
        file << point.latitude << ","
             << point.longitude
             << ",route\n";
    }


    file.close();
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

double Mission::getMinLatitude() const
{
    return minLatitude();
}


double Mission::getMaxLatitude() const
{
    return maxLatitude();
}


double Mission::getMinLongitude() const
{
    return minLongitude();
}


double Mission::getMaxLongitude() const
{
    return maxLongitude();
}

double Mission::minLatitude() const
{
    double min = fieldBoundary[0].latitude;

    for(const auto& point : fieldBoundary)
    {
        if(point.latitude < min)
            min = point.latitude;
    }

    return min;
}


double Mission::maxLatitude() const
{
    double max = fieldBoundary[0].latitude;

    for(const auto& point : fieldBoundary)
    {
        if(point.latitude > max)
            max = point.latitude;
    }

    return max;
}


double Mission::minLongitude() const
{
    double min = fieldBoundary[0].longitude;

    for(const auto& point : fieldBoundary)
    {
        if(point.longitude < min)
            min = point.longitude;
    }

    return min;
}


double Mission::maxLongitude() const
{
    double max = fieldBoundary[0].longitude;

    for(const auto& point : fieldBoundary)
    {
        if(point.longitude > max)
            max = point.longitude;
    }

    return max;
}

double Mission::getSweepSpacing() const
{
    return robotConfig.robotWidth + robotConfig.safetyMargin;
}

double Mission::metersToLatitude(double meters) const
{
    // Approximation:
    // 1 degree latitude ≈ 111,000 meters

    return meters / 111000.0;
}


double Mission::metersToLongitude(double meters) const
{
    // Approximation for Texas latitude
    // 1 degree longitude ≈ 94,000 meters

    return meters / 94000.0;
}