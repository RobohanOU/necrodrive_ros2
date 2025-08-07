#pragma once

#include <mutex>
#include <memory>
#include <queue>
#include <future>
#include <variant>
#include <unordered_map>
#include <optional>

// socketcan
#include "ros2_socketcan/socket_can_sender.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_id.hpp"



namespace necrodrive_system
{

namespace ros2socketcan = drivers::socketcan;

enum Health
{
    GREEN,
    RED
};

struct CanRequestBase
{
    virtual ~CanRequestBase() = default;
    virtual size_t get_size() = 0;
    virtual void fulfill_from_raw(std::array<uint8_t, 8>& data) = 0;

    ros2socketcan::CanId id;

    explicit CanRequestBase(ros2socketcan::CanId id) : id(id) {}
};

template <typename T>
class CanRequest : public CanRequestBase
{
public:
    inline CanRequest(ros2socketcan::CanId id) : CanRequestBase(id) {};

    inline std::promise<T>& get_promise() { return promise_; }
    inline std::future<T>   get_future() { return promise_.get_future(); }

    inline size_t get_size() { return sizeof(T); }
    inline void fulfill_from_raw(std::array<uint8_t, 8>& data) override {
        T value;
        std::memcpy(&value, data.data(), sizeof(T));
        promise_.set_value(value);
    }
private:
    std::promise<T> promise_;
};

class CanReader 
{
public:
    CanReader(
        std::shared_ptr<ros2socketcan::SocketCanSender> pSender,
        std::shared_ptr<ros2socketcan::SocketCanReceiver> pReceiver,
        std::chrono::nanoseconds write_timeout,
        std::chrono::nanoseconds read_timeout,
        std::chrono::milliseconds heartbeat_timeout
    );
    ~CanReader();

    void stop();
    void run();
    void start();

    inline Health get_health() {
        std::lock_guard lock(request_mutex_);
        return health_;
    }
    
    template <typename T>
    std::future<T> send_request(std::shared_ptr<CanRequest<T>> pRequest);
    private:
    void handle_frame(ros2socketcan::CanId& id, std::array<uint8_t, 8>& data);
    
    std::shared_ptr<ros2socketcan::SocketCanSender> pSender_;
    std::shared_ptr<ros2socketcan::SocketCanReceiver> pReceiver_;
    
    std::atomic<bool> running_ = false;
    std::thread reader_thread_;
    
    std::mutex request_mutex_;
    std::queue<std::shared_ptr<CanRequestBase>> request_queue_;
    std::unordered_map<
    ros2socketcan::CanId::IdT, 
    std::queue<std::shared_ptr<CanRequestBase>>> 
    pending_list_;
    
    std::chrono::nanoseconds write_timeout_;
    std::chrono::nanoseconds read_timeout_;
    std::chrono::milliseconds heartbeat_timeout_;
    std::optional<std::chrono::time_point<std::chrono::steady_clock>> last_heartbeat_;
    Health health_ = RED;
};

}