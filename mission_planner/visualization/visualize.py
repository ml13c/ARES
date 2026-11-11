import csv
import matplotlib.pyplot as plt


boundary_lat = []
boundary_lon = []

route_lat = []
route_lon = []


with open("mission_map.csv", "r") as file:

    reader = csv.DictReader(file)

    for row in reader:

        if row["type"] == "boundary":
            boundary_lat.append(float(row["latitude"]))
            boundary_lon.append(float(row["longitude"]))

        elif row["type"] == "route":
            route_lat.append(float(row["latitude"]))
            route_lon.append(float(row["longitude"]))


# Close the boundary shape
boundary_lat.append(boundary_lat[0])
boundary_lon.append(boundary_lon[0])


plt.figure(figsize=(8,8))


# Draw field boundary
plt.plot(
    boundary_lon,
    boundary_lat,
    marker="o",
    label="Field Boundary"
)


# Draw robot route
plt.plot(
    route_lon,
    route_lat,
    marker=".",
    label="Robot Route"
)


# Robot starting location
plt.scatter(
    route_lon[0],
    route_lat[0],
    marker="*",
    s=200,
    label="Robot Start"
)


plt.xlabel("Longitude")
plt.ylabel("Latitude")

plt.title("ARES Mission Planner - Scout Route")

plt.legend()

plt.grid(True)


plt.show()