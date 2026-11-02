#include "linmot/serial_port.h"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace linmot {
namespace {

speed_t to_speed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return 0;
    }
}

}  // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& device, int baud) {
    close();
    const speed_t speed = to_speed(baud);
    if (speed == 0) {
        last_error_ = "unsupported baud rate " + std::to_string(baud);
        return false;
    }
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
        last_error_ = "cannot open " + device + ": " + std::strerror(errno);
        return false;
    }
    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        last_error_ = device + ": tcgetattr: " + std::strerror(errno);
        close();
        return false;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        last_error_ = device + ": tcsetattr: " + std::strerror(errno);
        close();
        return false;
    }
    tcflush(fd_, TCIOFLUSH);
    last_error_.clear();
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::write(const std::uint8_t* data, std::size_t length) {
    std::size_t done = 0;
    while (done < length) {
        const ssize_t n = ::write(fd_, data + done, length - done);
        if (n > 0) {
            done += static_cast<std::size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd p{fd_, POLLOUT, 0};
            if (::poll(&p, 1, 100) > 0) continue;
            last_error_ = "serial write timed out";
            return false;
        }
        last_error_ = std::string("serial write: ") + std::strerror(errno);
        return false;
    }
    return true;
}

long SerialPort::read(std::uint8_t* data, std::size_t capacity, int timeout_ms) {
    pollfd p{fd_, POLLIN, 0};
    const int ready = ::poll(&p, 1, timeout_ms);
    if (ready < 0) {
        if (errno == EINTR) return 0;
        last_error_ = std::string("serial poll: ") + std::strerror(errno);
        return -1;
    }
    if (ready == 0) return 0;
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        last_error_ = "serial port closed or unplugged";
        return -1;
    }
    const ssize_t n = ::read(fd_, data, capacity);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
        last_error_ = std::string("serial read: ") + std::strerror(errno);
        return -1;
    }
    return static_cast<long>(n);
}

}  // namespace linmot
