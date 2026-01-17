#pragma once
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class SerialPort {
    int fd = -1;
public:
    bool Connect(const std::string& port) {
        if (fd != -1) close(fd);
        fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1) return false;
        
        fcntl(fd, F_SETFL, 0); // 恢复阻塞模式 (带超时)

        termios options{};
        tcgetattr(fd, &options);
        cfsetispeed(&options, B115200); // 必须匹配 ESP32
        cfsetospeed(&options, B115200);
        
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw mode
        options.c_oflag &= ~OPOST;
        
        // 关键：设置写入超时，防止拔线卡死
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 1; // 0.1s 超时

        tcsetattr(fd, TCSANOW, &options);
        return true;
    }

    bool Write(const void* data, const size_t len) {
        if (fd == -1) return false;
        const ssize_t n = write(fd, data, len);
        if (n < 0 || n != static_cast<ssize_t>(len)) {
            close(fd); fd = -1; // 自动断开
            return false;
        }
        return true;
    }
    
    bool IsConnected() const { return fd != -1; }
};