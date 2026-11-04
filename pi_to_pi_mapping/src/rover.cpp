#include "protocol.hpp"
#include "mock_fpga.hpp"
#include "localization.hpp"
#include "gps.hpp"

#include <algorithm>
#include <cmath>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>
#include <memory>
#include <limits>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace {

volatile std::sig_atomic_t keep_running = 1;
void stopOnInterrupt(int) { keep_running = 0; }

bool parsePort(const char* text, int& port) {
    try {
        const std::string raw(text);
        std::size_t consumed = 0;
        const int value = std::stoi(raw, &consumed);
        if (consumed != raw.size() || value < 1 || value > 65535) return false;
        port = value;
        return true;
    } catch (...) {
        return false;
    }
}

bool parsePositiveCount(const char* text, std::uint32_t& value) {
    try {
        const std::string raw(text);
        std::size_t consumed = 0;
        const unsigned long long parsed = std::stoull(raw, &consumed);
        if (consumed != raw.size() || parsed == 0 ||
            parsed > std::numeric_limits<std::uint32_t>::max()) return false;
        value = static_cast<std::uint32_t>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

// TCP is a byte stream: one send() is not guaranteed to send all bytes.
bool sendAll(int fd, const std::string& message) {
    std::size_t sent = 0;
    while (sent < message.size()) {
        const ssize_t count = ::send(fd, message.data() + sent,
                                     message.size() - sent, MSG_NOSIGNAL);
        if (count > 0) {
            sent += static_cast<std::size_t>(count);
        } else if (count == -1 && errno == EINTR) {
            if (!keep_running) return false;
        } else {
            return false;
        }
    }
    return true;
}

struct GpsOptions {
    bool enabled = false;
    bool serial = false;
    std::string source;
    int baud = 9600;
    bool mock_detections = false;
    std::uint32_t steps = 0;
    std::uint32_t interval_ms = 1000;
};

bool gpsOptions(int argc, char* argv[], GpsOptions& config) {
    // GPS mode keeps the legacy 4-positional-argument rover command intact.
    if (argc < 5 || (std::string(argv[3]) != "--gps-replay" &&
                     std::string(argv[3]) != "--gps-serial")) return false;
    config.enabled = true;
    config.serial = std::string(argv[3]) == "--gps-serial";
    config.source = argv[4];
    for (int i = 5; i < argc; ++i) {
        const std::string flag = argv[i];
        if (flag == "--mock-detections") {
            config.mock_detections = true;
        } else if ((flag == "--baud" || flag == "--steps" ||
                    flag == "--interval-ms") && i + 1 < argc) {
            const char* val = argv[++i];
            if (flag == "--baud") {
                int baud = 0;
                if (!parsePort(val, baud)) return false;
                config.baud = baud;
            } else if (flag == "--steps") {
                if (!parsePositiveCount(val, config.steps)) return false;
            } else if (!parsePositiveCount(val, config.interval_ms)) {
                return false;
            }
        } else {
            return false;
        }
    }
    return !(config.serial && config.interval_ms != 1000);
}

void sleepInterval(std::uint32_t interval_ms) {
    std::uint32_t slept_ms = 0;
    while (slept_ms < interval_ms && keep_running) {
        const auto chunk = std::min(100u, interval_ms - slept_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(chunk));
        slept_ms += chunk;
    }
}

bool transmit(int fd, const ares::Message& message) {
    const std::string record = ares::encodeMessage(message);
    if (!sendAll(fd, record)) return false;
    std::cout << "Sent: " << record << std::flush;
    return true;
}

// True GPS/replay mode intentionally does not invent grid coordinates or
// hazard range. Position messages and sightings have their own protocol types.
bool runGpsMode(int fd, const GpsOptions& options) {
    std::ifstream replay;
    ares::GpsSerial serial;
    if (options.serial) {
        std::string error;
        if (!serial.openDevice(options.source, options.baud, error)) {
            std::cerr << "GPS serial: " << error << '\n';
            return false;
        }
        std::cout << "Live NMEA GPS on " << options.source << " at "
                  << options.baud << " baud. Waiting for a valid fix...\n";
    } else {
        replay.open(options.source);
        if (!replay) {
            std::cerr << "Cannot open NMEA replay file: " << options.source << '\n';
            return false;
        }
        std::cout << "Replaying NMEA GPS from " << options.source << '\n';
    }
    ares::MockFPGA mock;
    std::uint32_t sequence = 0;
    bool warned_no_fix = false;
    auto last_fix = std::chrono::steady_clock::now();
    while (keep_running) {
        std::optional<ares::GpsFix> fix;
        if (options.serial) {
            fix = serial.nextFix(250);
            if (!serial.connected()) {
                std::cerr << "GPS device disconnected; stopping rover feed\n";
                ares::Message lost;
                lost.message_type = ares::MessageType::GpsStatus;
                transmit(fd, lost);
                return false;
            }
            const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - last_fix).count();
            if (!fix && elapsed >= 3 && !warned_no_fix) {
                std::cerr << "[GPS] No valid fix received for 3 seconds; "
                             "position transmission paused\n";
                warned_no_fix = true;
                ares::Message lost;
                lost.message_type = ares::MessageType::GpsStatus;
                if (!transmit(fd, lost)) return false;
            }
        } else {
            std::string line;
            while (std::getline(replay, line)) {
                fix = ares::parseNmeaSentence(line);
                if (fix) break;
            }
            if (!fix) break; // replay ended without a usable new fix
        }
        if (!fix) continue;
        last_fix = std::chrono::steady_clock::now();
        warned_no_fix = false;

        ares::Message position;
        position.message_type = ares::MessageType::GpsPosition;
        position.id = sequence;
        position.has_geo = true;
        position.latitude_deg = fix->position.latitude_deg;
        position.longitude_deg = fix->position.longitude_deg;
        position.has_heading = fix->course_deg.has_value();
        if (fix->course_deg) position.heading_deg = *fix->course_deg;
        if (!transmit(fd, position)) return false;

        if (options.mock_detections) {
            if (const auto detection = mock.poll(sequence)) {
                // Classification-only observations: these are *rover* GPS
                // coordinates at detection time, NOT localized object GPS.
                ares::Message sighting;
                sighting.message_type = ares::MessageType::Sighting;
                sighting.id = detection->id;
                sighting.object_type = detection->type;
                sighting.has_geo = true;
                sighting.latitude_deg = fix->position.latitude_deg;
                sighting.longitude_deg = fix->position.longitude_deg;
                if (!transmit(fd, sighting)) return false;
            }
        }
        ++sequence;
        if (options.steps > 0 && sequence >= options.steps) break;
        if (!options.serial) sleepInterval(options.interval_ms);
    }
    std::cout << "GPS rover feed completed: " << sequence << " position fixes sent.\n";
    return true;
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <operator-ipv4> [port] [steps] [interval-ms]\n"
                  << "  or: " << argv[0] << " <operator-ipv4> <port> --gps-replay FILE "
                     "[--steps N] [--interval-ms N] [--mock-detections]\n"
                  << "  or: " << argv[0] << " <operator-ipv4> <port> --gps-serial DEVICE "
                     "[--baud 9600] [--steps N] [--mock-detections]\n";
        return 1;
    }
    GpsOptions gps;
    const bool gps_mode = argc >= 4 && std::string(argv[3]).rfind("--gps-", 0) == 0;
    if (gps_mode && !gpsOptions(argc, argv, gps)) {
        std::cerr << "Invalid GPS arguments. Use --gps-replay FILE or --gps-serial DEVICE.\n";
        return 1;
    }
    if (!gps_mode && argc > 5) {
        std::cerr << "Too many positional arguments for simulated rover\n";
        return 1;
    }

    int port = ares::kDefaultPort;
    if (argc >= 3 && !parsePort(argv[2], port)) {
        std::cerr << "Invalid port (expected 1-65535)\n";
        return 1;
    }

    // steps=0 means run until Ctrl+C (default). Test mode uses a finite count.
    std::uint32_t max_steps = 0;
    std::uint32_t interval_ms = 1000;
    if (!gps_mode && argc >= 4 && !parsePositiveCount(argv[3], max_steps)) {
        std::cerr << "Invalid steps (expected positive integer)\n";
        return 1;
    }
    if (!gps_mode && argc >= 5 && !parsePositiveCount(argv[4], interval_ms)) {
        std::cerr << "Invalid interval-ms (expected positive integer)\n";
        return 1;
    }

    struct sigaction interrupt_action{};
    interrupt_action.sa_handler = stopOnInterrupt;
    sigemptyset(&interrupt_action.sa_mask);
    interrupt_action.sa_flags = 0;
    sigaction(SIGINT, &interrupt_action, nullptr);
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        std::perror("socket");
        return 1;
    }

    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::inet_pton(AF_INET, argv[1], &server.sin_addr) != 1) {
        std::cerr << "Invalid IPv4 address: " << argv[1] << '\n';
        ::close(fd);
        return 1;
    }

    if (::connect(fd, reinterpret_cast<const sockaddr*>(&server), sizeof(server)) < 0) {
        std::perror("connect (check operator IP, port, Wi-Fi and firewall)");
        ::close(fd);
        return 1;
    }

    std::cout << "Connected to operator " << argv[1] << ':' << port << '\n';
    if (gps_mode) {
        const bool okay = runGpsMode(fd, gps);
        ::close(fd);
        return okay ? 0 : 1;
    }
    std::cout << "Simulated scout mode: GPS coordinates + mock detections. "
              << "Ctrl+C to stop.\n";

    ares::MockFPGA mock_fpga;
    const ares::GeoPoint anchor{ares::kDemoOriginLat, ares::kDemoOriginLon};

    std::uint32_t sequence = 0;
    while (keep_running) {
        // Snake-shaped path inside a 20x10 grid; no real GPS is involved.
        const int column = static_cast<int>(sequence % ares::kGridWidth);
        const int row = static_cast<int>((sequence / ares::kGridWidth) % ares::kGridHeight);
        const bool left_to_right = (row % 2 == 0);
        const int x = left_to_right ? column : (ares::kGridWidth - 1 - column);
        const int y = row;

        // Demonstration-only GPS from grid cell coordinates, not an actual GPS receiver.
        const auto rover_geo = ares::fromLocalOffset(anchor,
            {x * ares::kDemoMetersPerCell, -y * ares::kDemoMetersPerCell});
        if (!rover_geo) {
            std::cerr << "Unable to calculate simulated rover GPS position\n";
            break;
        }
        const double heading_deg = left_to_right ? 90.0 : 270.0;
        const ares::RobotPose rover_pose{*rover_geo, heading_deg};

        ares::Message position;
        position.has_geo = true;
        position.latitude_deg = rover_geo->latitude_deg;
        position.longitude_deg = rover_geo->longitude_deg;
        position.heading_deg = heading_deg;
        position.message_type = ares::MessageType::Position;
        position.id = sequence;
        position.x = x;
        position.y = y;
        if (!transmit(fd, position)) {
            std::perror("send position");
            break;
        }

        // An event is generated by a simulated FPGA, *not* from a preloaded field map.
        // The FPGA produces only type/id; rover-localization code supplies coordinates.
        if (const auto detection = mock_fpga.poll(sequence)) {
            std::cout << "[Mock FPGA] Detected "
                      << ares::objectTypeName(detection->type)
                      << " (#" << detection->id << ")\n";

            // SIMULATED measurement: one meter directly ahead of the rover.
            // A real FPGA/classifier output is not enough to know distance;
            // real use requires camera calibration/depth, range sensor, etc.
            const double range_m = 1.0;
            const double relative_bearing_deg = 0.0;
            const auto hazard_geo = ares::localizeObject(
                rover_pose, range_m, relative_bearing_deg);
            if (!hazard_geo) {
                std::cerr << "Invalid mock detection geometry; skipping\n";
            } else {
                const auto local = ares::toLocalOffset(anchor, *hazard_geo);
                if (!local) {
                    std::cerr << "Unable to map localized hazard; skipping\n";
                } else {
                    // The ASCII grid is only a display of GPS-based coordinates.
                    const int hazard_x = static_cast<int>(std::lround(
                        local->east_m / ares::kDemoMetersPerCell));
                    const int hazard_y = static_cast<int>(std::lround(
                        -local->north_m / ares::kDemoMetersPerCell));
                    if (hazard_x < 0 || hazard_x >= ares::kGridWidth ||
                        hazard_y < 0 || hazard_y >= ares::kGridHeight) {
                        std::cout << "[Mock detection outside field grid; skipped]\n";
                    } else {
                        ares::Message hazard;
                        hazard.message_type = ares::MessageType::Hazard;
                        hazard.id = detection->id;
                        hazard.object_type = detection->type;
                        hazard.x = hazard_x;
                        hazard.y = hazard_y;
                        hazard.has_geo = true;
                        hazard.latitude_deg = hazard_geo->latitude_deg;
                        hazard.longitude_deg = hazard_geo->longitude_deg;
                        hazard.range_m = range_m;
                        hazard.relative_bearing_deg = relative_bearing_deg;
                        if (!transmit(fd, hazard)) {
                            std::perror("send hazard");
                            break;
                        }
                    }
                }
            }
        }

        ++sequence;
        if (max_steps > 0 && sequence >= max_steps) break;
        // Short sleep chunks allow Ctrl+C to stop the demo quickly.
        std::uint32_t slept_ms = 0;
        while (slept_ms < interval_ms && keep_running) {
            const std::uint32_t chunk = std::min(100u, interval_ms - slept_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(chunk));
            slept_ms += chunk;
        }
    }

    ::close(fd);
    std::cout << "Rover stopped.\n";
    return 0;
}
