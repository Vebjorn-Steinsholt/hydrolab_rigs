#include "linmot/protocol.h"

#include <cstring>

namespace linmot {
namespace {

void put_u16(std::uint8_t* p, std::uint16_t v) {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
}

void put_u32(std::uint8_t* p, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<std::uint8_t>(v >> (8 * i));
}

std::uint16_t get_u16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

std::uint32_t get_u32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

}  // namespace

std::uint16_t crc16(const std::uint8_t* data, std::size_t length) {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= static_cast<std::uint16_t>(data[i] << 8);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<std::uint16_t>(crc << 1);
        }
    }
    return crc;
}

std::array<std::uint8_t, kCommandFrameSize> encode(const CommandFrame& frame) {
    std::array<std::uint8_t, kCommandFrameSize> out{};
    out[0] = kSync1;
    out[1] = kSync2;
    out[2] = static_cast<std::uint8_t>(frame.command);
    out[3] = frame.sequence;
    put_u32(&out[4], static_cast<std::uint32_t>(frame.position));
    put_u32(&out[8], frame.velocity);
    put_u32(&out[12], frame.acceleration);
    put_u32(&out[16], frame.deceleration);
    put_u16(&out[20], crc16(&out[2], kCommandFrameSize - 4));
    return out;
}

std::array<std::uint8_t, kReplyFrameSize> encode_reply(const Reply& reply) {
    std::array<std::uint8_t, kReplyFrameSize> out{};
    out[0] = kSync1;
    out[1] = kSync2;
    out[2] = static_cast<std::uint8_t>(reply.type);
    out[3] = reply.sequence;
    put_u32(&out[4], static_cast<std::uint32_t>(reply.position));
    put_u16(&out[8], reply.status_word);
    out[10] = reply.flags;
    out[11] = reply.bad_frames;
    put_u16(&out[12], crc16(&out[2], kReplyFrameSize - 4));
    return out;
}

bool decode_command(const std::uint8_t* frame, CommandFrame& out) {
    if (frame[0] != kSync1 || frame[1] != kSync2) return false;
    if (crc16(&frame[2], kCommandFrameSize - 4) != get_u16(&frame[20])) return false;
    out.command = static_cast<Command>(frame[2]);
    out.sequence = frame[3];
    out.position = static_cast<std::int32_t>(get_u32(&frame[4]));
    out.velocity = get_u32(&frame[8]);
    out.acceleration = get_u32(&frame[12]);
    out.deceleration = get_u32(&frame[16]);
    return true;
}

void ReplyParser::feed(const std::uint8_t* data, std::size_t length, std::vector<Reply>& out) {
    buffer_.insert(buffer_.end(), data, data + length);
    std::size_t pos = 0;
    while (true) {
        // Find the next sync pair from pos.
        std::size_t start = pos;
        while (start + 1 < buffer_.size() &&
               !(buffer_[start] == kSync1 && buffer_[start + 1] == kSync2)) {
            ++start;
        }
        if (start + 1 >= buffer_.size()) {
            // No sync pair; keep a trailing first sync byte, it may start the next frame.
            const bool keep_last = !buffer_.empty() && start < buffer_.size() &&
                                   buffer_[buffer_.size() - 1] == kSync1;
            const std::size_t keep_from = keep_last ? buffer_.size() - 1 : buffer_.size();
            dropped_bytes_ += keep_from - pos;
            pos = keep_from;
            break;
        }
        dropped_bytes_ += start - pos;
        pos = start;
        if (buffer_.size() - pos < kReplyFrameSize) break;

        const std::uint8_t* f = &buffer_[pos];
        if (crc16(&f[2], kReplyFrameSize - 4) != get_u16(&f[12])) {
            ++bad_crc_;
            ++dropped_bytes_;
            pos += 1;  // look for the next sync inside this frame
            continue;
        }
        Reply r;
        r.type = static_cast<ReplyType>(f[2]);
        r.sequence = f[3];
        r.position = static_cast<std::int32_t>(get_u32(&f[4]));
        r.status_word = get_u16(&f[8]);
        r.flags = f[10];
        r.bad_frames = f[11];
        out.push_back(r);
        pos += kReplyFrameSize;
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(pos));
}

}  // namespace linmot
