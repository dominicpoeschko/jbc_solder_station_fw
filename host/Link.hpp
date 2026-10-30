#pragma once
// The remote's link to the station on a thread of its own: opens the path (a serial device or the
// printer's duplex socket; none given: the first duplex.0.sock under `logDir`), hands every report
// to `onReport` and every change of the connection to `onLink`, both on this thread, and opens it
// again after it dropped. As psu_hmi's and water_mix's tui Link, over host/Transport.hpp.
#include "Frames.hpp"
#include "Transport.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <mutex>
#include <poll.h>
#include <string>
#include <system_error>
#include <thread>
#include <unistd.h>
#include <utility>

namespace Host {

/// What the link is doing: no path to open, a path that did not open, open.
enum class LinkEvent { missing, refused, up };

class Link {
public:
    Link(std::string                                  path,
         std::filesystem::path                        logDir,
         unsigned                                     baud,
         std::function<void(Protocol::Report const&)> onReport,
         std::function<void(LinkEvent,
                            std::string const&)>      onLink)
      : path_{std::move(path)}
      , logDir_{std::move(logDir)}
      , baud_{baud}
      , onReport_{std::move(onReport)}
      , onLink_{std::move(onLink)}
      , thread_{[this] { run(); }} {}

    Link(Link const&)            = delete;
    Link& operator=(Link const&) = delete;

    ~Link() {
        stop_ = true;
        thread_.join();
    }

    [[nodiscard]] bool up() const { return up_; }

    /// False when there is no connection to send it on.
    bool send(Protocol::Command const& command) {
        auto const            bytes = frame(command);
        std::lock_guard const lock{mutex_};
        if(fd_ < 0 || bytes.empty()) { return false; }
        return ::write(fd_, bytes.data(), bytes.size()) == static_cast<ssize_t>(bytes.size());
    }

private:
    struct Attempt {
        Opened      opened;
        std::string path;
    };

    [[nodiscard]] Attempt open() const {
        if(!path_.empty()) { return {openLink(path_, baud_), path_}; }
        Attempt last{
          {-1, Opened::Why::missing},
          (logDir_ / "*" / "duplex.0.sock").string()
        };
        std::error_code ec;
        for(auto const& target : std::filesystem::directory_iterator{logDir_, ec}) {
            auto const socket = printerSocketPath(target.path(), "duplex.0.sock").string();
            auto const opened = openLink(socket, baud_);
            if(opened.why == Opened::Why::ok) { return {opened, socket}; }
            if(opened.why == Opened::Why::refused) { last = {opened, socket}; }
        }
        return last;
    }

    void run() {
        using namespace std::chrono_literals;
        while(!stop_) {
            auto const attempt = open();
            if(attempt.opened.why != Opened::Why::ok) {
                onLink_(attempt.opened.why == Opened::Why::missing ? LinkEvent::missing
                                                                   : LinkEvent::refused,
                        attempt.path);
                for(int i = 0; i < 5 && !stop_; ++i) { std::this_thread::sleep_for(100ms); }
                continue;
            }
            {
                std::lock_guard const lock{mutex_};
                fd_ = attempt.opened.fd;
            }
            up_ = true;
            onLink_(LinkEvent::up, attempt.path);
            send(Protocol::Command{Protocol::Get{0}});
            readUntilDown(attempt.opened.fd);
            {
                std::lock_guard const lock{mutex_};
                ::close(fd_);
                fd_ = -1;
            }
            up_ = false;
        }
    }

    void readUntilDown(int fd) {
        FrameReader<Protocol::Report> reader;
        std::array<std::byte, 256>    chunk{};
        while(!stop_) {
            pollfd     p{.fd = fd, .events = POLLIN, .revents = 0};
            auto const ready = ::poll(&p, 1, 100);
            if(ready < 0) { return; }
            if(ready == 0) { continue; }
            auto const n = ::read(fd, chunk.data(), chunk.size());
            if(n <= 0) { return; }   // the printer went, the adapter was unplugged
            reader.append(std::span{chunk}.first(static_cast<std::size_t>(n)));
            while(auto const report = reader.next()) { onReport_(*report); }
        }
    }

    std::string                                        path_;
    std::filesystem::path                              logDir_;
    unsigned                                           baud_;
    std::function<void(Protocol::Report const&)>       onReport_;
    std::function<void(LinkEvent, std::string const&)> onLink_;
    std::mutex                                         mutex_;
    int                                                fd_{-1};
    std::atomic<bool>                                  up_{false};
    std::atomic<bool>                                  stop_{false};
    std::thread                                        thread_;
};

}   // namespace Host
