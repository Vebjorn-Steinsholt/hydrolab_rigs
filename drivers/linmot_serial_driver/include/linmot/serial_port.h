// serial_port.h - raw 8N1 serial port on Linux (termios), non-blocking.
//
// Opening the port of an Arduino Uno resets the board (DTR); the bridge waits for READY instead
// of avoiding that.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace linmot {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Returns false and sets last_error() on failure.
    bool open(const std::string& device, int baud = 115200);
    void close();
    bool is_open() const { return fd_ >= 0; }

    // Writes all bytes or fails.
    bool write(const std::uint8_t* data, std::size_t length);
    // Reads what is available, waiting at most timeout_ms for the first byte.
    // Returns the number of bytes read, 0 on timeout, -1 on error.
    long read(std::uint8_t* data, std::size_t capacity, int timeout_ms);

    const std::string& last_error() const { return last_error_; }

private:
    int fd_ = -1;
    std::string last_error_;
};

}  // namespace linmot
