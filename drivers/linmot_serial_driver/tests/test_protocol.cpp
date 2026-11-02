// Frame layout, CRC and the reply parser, checked against the firmware's definitions.
#include <cstdio>
#include <cstring>
#include <vector>

#include "linmot/protocol.h"

using namespace linmot;

int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

int main() {
    // CRC-16/CCITT-FALSE check value; Python: binascii.crc_hqx(b"123456789", 0xFFFF).
    const char* check = "123456789";
    CHECK(crc16(reinterpret_cast<const std::uint8_t*>(check), 9) == 0x29B1);

    // A go-to-position command as link_test.py builds it: 100 mm at 20 mm/s, 0.2 m/s^2.
    CommandFrame f;
    f.command = Command::GoToPosition;
    f.sequence = 7;
    f.position = 1000000;
    f.velocity = 20000;
    f.acceleration = 20000;
    f.deceleration = 20000;
    const auto bytes = encode(f);
    CHECK(bytes.size() == 22);
    CHECK(bytes[0] == 0xA5 && bytes[1] == 0x5A);
    CHECK(bytes[2] == 3 && bytes[3] == 7);
    CHECK(bytes[4] == 0x40 && bytes[5] == 0x42 && bytes[6] == 0x0F && bytes[7] == 0x00);
    CHECK(bytes[8] == 0x20 && bytes[9] == 0x4E);
    const std::uint16_t crc = crc16(&bytes[2], 18);
    CHECK(bytes[20] == (crc & 0xFF) && bytes[21] == (crc >> 8));
    CommandFrame back;
    CHECK(decode_command(bytes.data(), back));
    CHECK(back.command == Command::GoToPosition && back.position == 1000000 && back.sequence == 7);

    // Negative positions survive the round trip.
    f.position = -12345;
    CHECK(decode_command(encode(f).data(), back) && back.position == -12345);

    // A reply as the drive sent it on the rig: 100.3 mm, status 0x0830.
    Reply r;
    r.type = ReplyType::Feedback;
    r.sequence = 42;
    r.position = 1003000;
    r.status_word = 0x0830;
    r.flags = flag::kWatchdogTripped;
    r.bad_frames = 3;
    const auto rb = encode_reply(r);

    // Start-up text, a reply split over two reads, a corrupted reply, then a good one.
    std::vector<std::uint8_t> stream;
    const char text[] = "Entering Configuration Mode Successful!\r\n";
    stream.insert(stream.end(), text, text + sizeof(text) - 1);
    stream.insert(stream.end(), rb.begin(), rb.end());
    auto corrupt = rb;
    corrupt[6] ^= 0xFF;
    stream.insert(stream.end(), corrupt.begin(), corrupt.end());
    stream.insert(stream.end(), rb.begin(), rb.end());

    ReplyParser parser;
    std::vector<Reply> out;
    parser.feed(stream.data(), 50, out);
    CHECK(out.size() == 0);  // the first reply is not complete yet
    parser.feed(stream.data() + 50, stream.size() - 50, out);
    CHECK(out.size() == 2);
    CHECK(parser.bad_crc() == 1);
    CHECK(parser.dropped_bytes() == (sizeof(text) - 1) + 14);
    if (out.size() == 2) {
        CHECK(out[0].type == ReplyType::Feedback);
        CHECK(out[0].sequence == 42 && out[0].position == 1003000);
        CHECK(out[0].status_word == 0x0830 && out[0].flags == flag::kWatchdogTripped);
        CHECK(out[0].bad_frames == 3);
        CHECK(out[1].position == 1003000);
    }

    // A frame fed one byte at a time, with a stray 0xA5 in front.
    ReplyParser slow;
    out.clear();
    const std::uint8_t stray = 0xA5;
    slow.feed(&stray, 1, out);
    for (std::uint8_t b : rb) slow.feed(&b, 1, out);
    CHECK(out.size() == 1);
    CHECK(slow.dropped_bytes() == 1);

    if (failures == 0) std::printf("test_protocol: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
