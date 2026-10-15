#pragma once

#include <cstdint>
#include <string>

namespace ares {

constexpr int kGridWidth = 20;
constexpr int kGridHeight = 10;
constexpr int kDefaultPort = 5000;

enum class MessageType {
    Position,
    Hazard
};

enum class ObjectType {
    None,
    Rock,
    Weed,
    Other
};

// Shared application data type. This is not a raw wire format.
struct Message {
    MessageType message_type = MessageType::Position;
    std::uint32_t id = 0;
    int x = 0;
    int y = 0;
    ObjectType object_type = ObjectType::None;
};

// One newline-terminated ASCII record per message:
// POS,id,x,y\n
// HAZARD,id,ROCK|WEED|OTHER,x,y\n
std::string encodeMessage(const Message& message);
bool decodeMessage(const std::string& line, Message& output, std::string& error);
const char* objectTypeName(ObjectType type);

} // namespace ares
