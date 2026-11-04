#include "gps.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace ares {
namespace {
std::vector<std::string> split(const std::string& text) {
    std::vector<std::string> fields;
    std::size_t start = 0;
    for (;;) {
        const auto end = text.find(',', start);
        fields.push_back(text.substr(start, end == std::string::npos ? end : end - start));
        if (end == std::string::npos) break;
        start = end + 1;
    }
    return fields;
}

std::optional<double> number(const std::string& value) {
    if (value.empty()) return std::nullopt;
    try {
        std::size_t used = 0;
        const double parsed = std::stod(value, &used);
        if (used != value.size() || !std::isfinite(parsed)) return std::nullopt;
        return parsed;
    } catch (...) {
        return std::nullopt;
    }
}

// NMEA ddmm.mmmm (latitude) / dddmm.mmmm (longitude) into decimal degrees.
std::optional<double> coordinate(const std::string& raw,
                                 const std::string& hemisphere, bool latitude) {
    if (hemisphere.size() != 1 || raw.size() < (latitude ? 4u : 5u))
        return std::nullopt;
    const char hemi = hemisphere[0];
    if (latitude ? (hemi != 'N' && hemi != 'S') :
                   (hemi != 'E' && hemi != 'W')) return std::nullopt;
    const std::size_t degree_digits = latitude ? 2 : 3;
    const auto degrees = number(raw.substr(0, degree_digits));
    const auto minutes = number(raw.substr(degree_digits));
    if (!degrees || !minutes || std::floor(*degrees) != *degrees ||
        *minutes < 0 || *minutes >= 60 ||
        *degrees > (latitude ? 90 : 180) ||
        (*degrees == (latitude ? 90 : 180) && *minutes != 0))
        return std::nullopt;
    double result = *degrees + *minutes / 60.0;
    if (hemi == 'S' || hemi == 'W') result = -result;
    return result;
}

int hexDigit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

speed_t baudConstant(int baud) {
    switch (baud) {
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return 0;
    }
}
} // namespace

std::optional<GpsFix> parseNmeaSentence(const std::string& raw) {
    std::string line = raw;
    if (!line.empty() && line.back() == '\n') line.pop_back();
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line.front() != '$' || line.size() > 128) return std::nullopt;
    const auto star = line.find('*');
    if (star == std::string::npos || star + 3 != line.size()) return std::nullopt;
    const int hi = hexDigit(line[star + 1]);
    const int lo = hexDigit(line[star + 2]);
    if (hi < 0 || lo < 0) return std::nullopt;
    unsigned int checksum = 0;
    for (std::size_t i = 1; i < star; ++i)
        checksum ^= static_cast<unsigned char>(line[i]);
    if (checksum != static_cast<unsigned int>((hi << 4) | lo)) return std::nullopt;

    const auto fields = split(line.substr(1, star - 1));
    if (fields.empty() || fields[0].size() != 5) return std::nullopt;
    const std::string kind = fields[0].substr(2);
    GpsFix fix;
    if (kind == "RMC") {
        // $GNRMC,time,A,latitude,N,longitude,E,speed_knots,course,...
        if (fields.size() < 9 || fields[2] != "A") return std::nullopt;
        const auto lat = coordinate(fields[3], fields[4], true);
        const auto lon = coordinate(fields[5], fields[6], false);
        if (!lat || !lon) return std::nullopt;
        fix.position = {*lat, *lon};
        const auto speed_knots = number(fields[7]);
        const auto course = number(fields[8]);
        // Ground track is unreliable when stationary / moving slowly.
        if (speed_knots && *speed_knots > 0.972 && course &&
            *course >= 0 && *course < 360) fix.course_deg = *course;
    } else if (kind == "GGA") {
        // $GNGGA,time,latitude,N,longitude,E,fix_quality,...
        if (fields.size() < 7) return std::nullopt;
        const auto quality = number(fields[6]);
        if (!quality || *quality < 1 || *quality > 8 ||
            std::floor(*quality) != *quality) return std::nullopt;
        const auto lat = coordinate(fields[2], fields[3], true);
        const auto lon = coordinate(fields[4], fields[5], false);
        if (!lat || !lon) return std::nullopt;
        fix.position = {*lat, *lon};
    } else {
        return std::nullopt;
    }
    if (!validGeoPoint(fix.position)) return std::nullopt;
    return fix;
}

GpsSerial::~GpsSerial() {
    if (fd_ >= 0) ::close(fd_);
}

bool GpsSerial::openDevice(const std::string& device, int baud, std::string& error) {
    const speed_t speed = baudConstant(baud);
    if (!speed) {
        error = "Unsupported baud rate (use 4800/9600/19200/38400/57600/115200)";
        return false;
    }
    const int fd = ::open(device.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        error = "Cannot open GPS device " + device + ": " + std::to_string(errno);
        return false;
    }
    termios settings{};
    if (::tcgetattr(fd, &settings) < 0) {
        error = "Cannot read serial configuration";
        ::close(fd);
        return false;
    }
    ::cfmakeraw(&settings);
    ::cfsetispeed(&settings, speed);
    ::cfsetospeed(&settings, speed);
    settings.c_cflag |= (CLOCAL | CREAD);
    settings.c_cflag &= ~CRTSCTS;
    settings.c_cflag &= ~CSTOPB;
    settings.c_cflag &= ~PARENB;
    settings.c_cflag &= ~CSIZE;
    settings.c_cflag |= CS8;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (::tcsetattr(fd, TCSANOW, &settings) < 0) {
        error = "Cannot configure GPS serial device";
        ::close(fd);
        return false;
    }
    if (fd_ >= 0) ::close(fd_);
    fd_ = fd;
    pending_.clear();
    return true;
}

std::optional<GpsFix> GpsSerial::nextFix(int timeout_ms) {
    if (fd_ < 0) return std::nullopt;
    // Parse already-buffered complete sentences BEFORE waiting for more bytes.
    auto extract = [this]() -> std::optional<GpsFix> {
        std::size_t line_end;
        while ((line_end = pending_.find('\n')) != std::string::npos) {
            const std::string line = pending_.substr(0, line_end);
            pending_.erase(0, line_end + 1);
            if (auto fix = parseNmeaSentence(line)) return fix;
        }
        if (pending_.size() > 1024) pending_.clear();
        return std::nullopt;
    };
    if (auto fix = extract()) return fix;
    pollfd watch{fd_, POLLIN, 0};
    const int ready = ::poll(&watch, 1, timeout_ms);
    if (ready <= 0) return std::nullopt;
    if (watch.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        ::close(fd_);
        fd_ = -1;
        return std::nullopt;
    }
    char buffer[512];
    const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
    if (count <= 0) return std::nullopt;
    pending_.append(buffer, static_cast<std::size_t>(count));
    return extract();
}
} // namespace ares
