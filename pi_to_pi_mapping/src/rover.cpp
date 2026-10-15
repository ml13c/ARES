#include "protocol.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
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

bool transmit(int fd, const ares::Message& message) {
    const std::string record = ares::encodeMessage(message);
    if (!sendAll(fd, record)) return false;
    std::cout << "Sent: " << record << std::flush;
    return true;
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: " << argv[0] << " <operator-ipv4> [port]\n";
        return 1;
    }

    int port = ares::kDefaultPort;
    if (argc == 3 && !parsePort(argv[2], port)) {
        std::cerr << "Invalid port (expected 1-65535)\n";
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

    std::cout << "Connected to operator " << argv[1] << ':' << port << '\n'
              << "Generating simulated positions every second. Ctrl+C to stop.\n";

    std::uint32_t sequence = 0;
    while (keep_running) {
        // Snake-shaped path inside a 20x10 grid; no real GPS is involved.
        const int column = static_cast<int>(sequence % ares::kGridWidth);
        const int row = static_cast<int>((sequence / ares::kGridWidth) % ares::kGridHeight);
        const bool left_to_right = (row % 2 == 0);
        const int x = left_to_right ? column : (ares::kGridWidth - 1 - column);
        const int y = row;

        ares::Message position;
        position.message_type = ares::MessageType::Position;
        position.id = sequence;
        position.x = x;
        position.y = y;
        if (!transmit(fd, position)) {
            std::perror("send position");
            break;
        }

        // Two mock FPGA detections. They remain plotted after rover moves.
        if (sequence == 3 || sequence == 8) {
            ares::Message hazard;
            hazard.message_type = ares::MessageType::Hazard;
            hazard.id = (sequence == 3) ? 101 : 102;
            hazard.object_type = (sequence == 3) ? ares::ObjectType::Rock
                                                : ares::ObjectType::Weed;
            hazard.x = (sequence == 3) ? 5 : 13;
            hazard.y = (sequence == 3) ? 4 : 7;
            if (!transmit(fd, hazard)) {
                std::perror("send hazard");
                break;
            }
        }

        ++sequence;
        // Sleep in small intervals so Ctrl+C exits promptly.
        for (int i = 0; i < 10 && keep_running; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ::close(fd);
    std::cout << "Rover stopped.\n";
    return 0;
}
