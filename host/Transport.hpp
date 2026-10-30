#pragma once
// The two ways to the station, as a file descriptor: SERCOM1's UART on PA18/PA19 (a serial device,
// 8N1) or uc_log_printer's duplex socket for the "control" channel
// (<build>/rtt_log/<target>/duplex.0.sock, over the probe). psu302's host/Transport.hpp, with the
// baud rate an argument and the printer's short-path rule for socket paths too long for sun_path.
#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <filesystem>
#include <optional>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <termios.h>
#include <unistd.h>

namespace Host {

/// What opening a path came to: the descriptor, or why there is none - a path that is not there
/// (no printer ran yet, the adapter unplugged) or one that is there but did not open (a socket left
/// behind by a printer that is gone, a device somebody else holds).
struct Opened {
    enum class Why { ok, missing, refused };
    int fd{-1};
    Why why{Why::missing};
};

/// termios' constant for a baud rate; nullopt for one it has none for.
inline std::optional<speed_t> baudConstant(unsigned baud) {
    switch(baud) {
    case 9'600:   return B9600;
    case 19'200:  return B19200;
    case 38'400:  return B38400;
    case 57'600:  return B57600;
    case 115'200: return B115200;
    case 230'400: return B230400;
    case 460'800: return B460800;
    case 921'600: return B921600;
    default:      return std::nullopt;
    }
}

inline Opened openSerial(std::string const& path,
                         unsigned           baud = 115'200) {
    auto const speed = baudConstant(baud);
    if(!speed) { return {-1, Opened::Why::refused}; }
    int const fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if(fd < 0) { return {-1, errno == ENOENT ? Opened::Why::missing : Opened::Why::refused}; }
    termios t{};
    if(::tcgetattr(fd, &t) != 0) {
        ::close(fd);
        return {-1, Opened::Why::refused};
    }
    ::cfmakeraw(&t);
    ::cfsetispeed(&t, *speed);
    ::cfsetospeed(&t, *speed);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
    ::tcsetattr(fd, TCSANOW, &t);
    return {fd, Opened::Why::ok};
}

inline Opened openSocket(std::string const& path) {
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if(path.size() >= sizeof(address.sun_path)) { return {-1, Opened::Why::refused}; }
    std::copy(path.begin(), path.end(), address.sun_path);
    int const fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if(fd < 0) { return {-1, Opened::Why::refused}; }
    if(::connect(fd, reinterpret_cast<sockaddr const*>(&address), sizeof(address)) == 0) {
        return {fd, Opened::Why::ok};
    }
    ::close(fd);
    return {-1, Opened::Why::refused};
}

/// A socket is connected to, anything else is opened as a serial device.
inline Opened openLink(std::string const& path,
                       unsigned           baud = 115'200) {
    struct stat st{};
    if(::stat(path.c_str(), &st) != 0) { return {-1, Opened::Why::missing}; }
    return S_ISSOCK(st.st_mode) ? openSocket(path) : openSerial(path, baud);
}

/// Where a printer puts its socket `name` for the log directory `dir`: dir/name, or
/// /tmp/uc_log_<uid>/<FNV-1a of dir>.name when that is too long for sun_path (uc_log's
/// host/uc_log/printer/SocketPath.hpp, whose rule doc/control_protocol/socket_paths.json pins).
inline std::filesystem::path printerSocketPath(std::filesystem::path const& dir,
                                               std::string const&           name) {
    auto absolute = std::filesystem::absolute(dir).lexically_normal();
    if(!absolute.has_filename() && absolute.has_parent_path()) {
        absolute = absolute.parent_path();
    }
    auto direct = absolute / name;
    if(direct.native().size() < sizeof(sockaddr_un{}.sun_path)) { return direct; }
    std::uint64_t hash = 0xcbf29ce484222325ULL;
    for(char const c : absolute.native()) {
        hash ^= static_cast<unsigned char>(c);
        hash *= 0x100000001b3ULL;
    }
    char tail[64];
    std::snprintf(tail,
                  sizeof(tail),
                  "%016llx.%s",
                  static_cast<unsigned long long>(hash),
                  name.c_str());
    return std::filesystem::path{"/tmp"} / ("uc_log_" + std::to_string(::getuid())) / tail;
}

}   // namespace Host
