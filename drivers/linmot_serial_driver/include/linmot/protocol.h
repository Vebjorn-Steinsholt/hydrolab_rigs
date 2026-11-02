// protocol.h - frames of the LinMot serial-to-CAN bridge (firmware/linmot_serial_bridge).
//
// The firmware's header comment is the reference; this mirrors it:
//
//   computer -> Arduino, 22 bytes          Arduino -> computer, 14 bytes
//   0  u8   0xA5  sync                     0  u8   0xA5  sync
//   1  u8   0x5A  sync                     1  u8   0x5A  sync
//   2  u8   command                        2  u8   type (READY or FEEDBACK)
//   3  u8   sequence                       3  u8   sequence of the last valid command
//   4  i32  position      0.1 um           4  i32  actual position  0.1 um
//   8  u32  max velocity  1e-6 m/s         8  u16  status word
//   12 u32  acceleration  1e-5 m/s^2       10 u8   flags
//   16 u32  deceleration  1e-5 m/s^2       11 u8   bad frames (wraps at 255)
//   20 u16  CRC                            12 u16  CRC
//
// Little-endian; CRC-16/CCITT-FALSE over every byte after the sync bytes and before the CRC.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace linmot {

constexpr std::uint8_t kSync1 = 0xA5;
constexpr std::uint8_t kSync2 = 0x5A;
constexpr std::size_t kCommandFrameSize = 22;
constexpr std::size_t kReplyFrameSize = 14;

enum class Command : std::uint8_t {
    Ping = 0,           // reply READY
    Operational = 1,    // CANopen NMT start
    Home = 2,           // start homing (the slide moves to its home stop)
    GoToPosition = 3,   // VAI go to position
    SwitchOn = 4,       // from any state; clears control word bit 0 first if needed
    SwitchOff = 5,
    RequestStatus = 6,  // SYNC only, so the drive sends position and status
    AckError = 7,       // rising edge of control word bit 7, drive left switched off
    PreOperational = 8, // CANopen NMT: the drive ignores PDOs and its sync watchdog is off;
                        // no reply, because the drive sends no feedback in that state
};

enum class ReplyType : std::uint8_t {
    Feedback = 0x01,
    Ready = 0x10,
};

// Flags in replies.
namespace flag {
constexpr std::uint8_t kWatchdogTripped = 0x01;  // motion aborted: no frame within 200 ms
constexpr std::uint8_t kCanSendError = 0x02;     // a CAN frame could not be sent (reported once)
constexpr std::uint8_t kRejected = 0x04;         // last command unknown or out of range (once)
}  // namespace flag

// Units on the wire.
constexpr double kPositionPerMetre = 1e7;        // 0.1 um
constexpr double kVelocityPerMps = 1e6;          // 1e-6 m/s
constexpr double kAccelerationPerMps2 = 1e5;     // 1e-5 m/s^2

struct CommandFrame {
    Command command = Command::Ping;
    std::uint8_t sequence = 0;
    std::int32_t position = 0;
    std::uint32_t velocity = 0;
    std::uint32_t acceleration = 0;
    std::uint32_t deceleration = 0;
};

struct Reply {
    ReplyType type = ReplyType::Feedback;
    std::uint8_t sequence = 0;
    std::int32_t position = 0;
    std::uint16_t status_word = 0;
    std::uint8_t flags = 0;
    std::uint8_t bad_frames = 0;
};

std::uint16_t crc16(const std::uint8_t* data, std::size_t length);

std::array<std::uint8_t, kCommandFrameSize> encode(const CommandFrame& frame);

// For tests and simulators: the bridge's side of the link.
std::array<std::uint8_t, kReplyFrameSize> encode_reply(const Reply& reply);
bool decode_command(const std::uint8_t* frame, CommandFrame& out);

// Collects reply frames from a byte stream. Bytes before a sync and frames with a bad CRC are
// dropped and counted; after a bad CRC the search for the next sync starts one byte later, so a
// lost byte costs only the frames it touches. The Arduino's start-up text ends up here as
// dropped bytes.
class ReplyParser {
public:
    // Appends the complete, valid replies in data to out.
    void feed(const std::uint8_t* data, std::size_t length, std::vector<Reply>& out);

    std::uint64_t dropped_bytes() const { return dropped_bytes_; }
    std::uint64_t bad_crc() const { return bad_crc_; }

private:
    std::vector<std::uint8_t> buffer_;
    std::uint64_t dropped_bytes_ = 0;
    std::uint64_t bad_crc_ = 0;
};

}  // namespace linmot
