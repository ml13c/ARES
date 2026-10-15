#include "grid_map.hpp"
#include "protocol.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
// Operator implementation for handling rover communications and grid visualization this is just a temp
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

void showMap(const ares::GridMap& map, const std::string& status) {
    std::cout << "\033[2J\033[H"; // clear and home, works in most SSH terminals
    std::cout << map.render() << "\n" << status << "\nCtrl+C to stop.\n" << std::flush;
}

void applyLine(const std::string& line, ares::GridMap& map, std::string& status) {
    ares::Message msg;
    std::string error;
    if (!ares::decodeMessage(line, msg, error)) {
        status = "Discarded malformed message: " + error;
    } else if (msg.message_type == ares::MessageType::Position) {
        if (map.updateRover(msg.x, msg.y)) {
            status = "Last update: rover position #" + std::to_string(msg.id);
        } else {
            status = "Discarded position outside grid: (" + std::to_string(msg.x) +
                     ", " + std::to_string(msg.y) + ")";
        }
    } else {
        if (map.addHazard(msg)) {
            status = "Last update: hazard #" + std::to_string(msg.id) + " (" +
                     ares::objectTypeName(msg.object_type) + ")";
        } else {
            status = "Discarded hazard outside grid";
        }
    }
    showMap(map, status);
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc > 2) {
        std::cerr << "Usage: " << argv[0] << " [port]\n";
        return 1;
    }

    int port = ares::kDefaultPort;
    if (argc == 2 && !parsePort(argv[1], port)) {
        std::cerr << "Invalid port (expected 1-65535)\n";
        return 1;
    }
    
    struct sigaction interrupt_action{};
    interrupt_action.sa_handler = stopOnInterrupt;
    sigemptyset(&interrupt_action.sa_mask);
    interrupt_action.sa_flags = 0;
    sigaction(SIGINT, &interrupt_action, nullptr);

    const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listener < 0) {
        std::perror("socket");
        return 1;
    }
    const int reuse = 1;
    ::setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<std::uint16_t>(port));

    if (::bind(listener, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
        std::perror("bind");
        ::close(listener);
        return 1;
    }
    if (::listen(listener, 1) < 0) {
        std::perror("listen");
        ::close(listener);
        return 1;
    }

    ares::GridMap map(ares::kGridWidth, ares::kGridHeight);
    showMap(map, "Waiting for rover on TCP port " + std::to_string(port) + "...");

    while (keep_running) {
        sockaddr_in rover_addr{};
        socklen_t rover_len = sizeof(rover_addr);
        const int connection = ::accept(listener,
                       reinterpret_cast<sockaddr*>(&rover_addr), &rover_len);
        if (connection < 0) {
            if (errno == EINTR && !keep_running) break;
            if (errno == EINTR) continue;
            std::perror("accept");
            break;
        }

        showMap(map, "Rover connected. Waiting for messages...");
        std::string pending;
        char buffer[1024];
        while (keep_running) {
            const ssize_t size = ::recv(connection, buffer, sizeof(buffer), 0);
            if (size == 0) break;
            if (size < 0) {
                if (errno == EINTR) continue;
                std::perror("recv");
                break;
            }
            pending.append(buffer, static_cast<std::size_t>(size));
            std::size_t line_end;
            while ((line_end = pending.find('\n')) != std::string::npos) {
                const std::string line = pending.substr(0, line_end);
                pending.erase(0, line_end + 1);
                std::string status;
                applyLine(line, map, status);
            }
            if (pending.size() > 2048) {
                pending.clear();
                showMap(map, "Discarded oversized/incomplete record");
            }
        }
        ::close(connection);
        if (keep_running) showMap(map, "Rover disconnected. Waiting for reconnect...");
    }

    ::close(listener);
    std::cout << "\nOperator stopped.\n";
    return 0;
}
