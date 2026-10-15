#include "protocol.hpp"

#include <charconv>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
// thisi simulates what will be sent to the operator from the rover
namespace ares {
namespace {

std::vector<std::string> splitCommaSeparated(const std::string& line) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true) {
        const std::size_t end = line.find(',', start);
        if (end == std::string::npos) {
            parts.push_back(line.substr(start));
            return parts;
        }
        parts.push_back(line.substr(start, end - start));
        start = end + 1;
    }
}

template <typename T>
bool parseInteger(const std::string& value, T& parsed) {
    if (value.empty()) return false;
    const char* begin = value.data();
    const char* end = begin + value.size();
    const auto result = std::from_chars(begin, end, parsed);
    return result.ec == std::errc{} && result.ptr == end;
}

} // namespace

const char* objectTypeName(ObjectType type) {
    switch (type) {
        case ObjectType::Rock: return "ROCK";
        case ObjectType::Weed: return "WEED";
        case ObjectType::Other: return "OTHER";
        case ObjectType::None: return "NONE";
    }
    return "UNKNOWN";
}

std::string encodeMessage(const Message& message) {
    std::ostringstream out;
    if (message.message_type == MessageType::Position) {
        out << "POS," << message.id << ',' << message.x << ',' << message.y << '\n';
    } else {
        out << "HAZARD," << message.id << ',' << objectTypeName(message.object_type)
            << ',' << message.x << ',' << message.y << '\n';
    }
    return out.str();
}

bool decodeMessage(const std::string& raw, Message& output, std::string& error) {
    std::string line = raw;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const auto parts = splitCommaSeparated(line);
    Message parsed;

    if (!parts.empty() && parts[0] == "POS" && parts.size() == 4) {
        parsed.message_type = MessageType::Position;
        if (!parseInteger(parts[1], parsed.id) ||
            !parseInteger(parts[2], parsed.x) ||
            !parseInteger(parts[3], parsed.y)) {
            error = "Invalid POS numeric field";
            return false;
        }
    } else if (!parts.empty() && parts[0] == "HAZARD" && parts.size() == 5) {
        parsed.message_type = MessageType::Hazard;
        if (parts[2] == "ROCK") parsed.object_type = ObjectType::Rock;
        else if (parts[2] == "WEED") parsed.object_type = ObjectType::Weed;
        else if (parts[2] == "OTHER") parsed.object_type = ObjectType::Other;
        else {
            error = "Unknown hazard class";
            return false;
        }
        if (!parseInteger(parts[1], parsed.id) ||
            !parseInteger(parts[3], parsed.x) ||
            !parseInteger(parts[4], parsed.y)) {
            error = "Invalid HAZARD numeric field";
            return false;
        }
    } else {
        error = "Expected POS,id,x,y or HAZARD,id,type,x,y";
        return false;
    }

    output = parsed;
    return true;
}

} // namespace ares
