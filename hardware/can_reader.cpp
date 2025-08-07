#include "necrodrive_ros2/can_reader.hpp"
#include "necrodrive_ros2/odrive_cmds.hpp"

namespace necrodrive_system {

CanReader::CanReader(
    std::shared_ptr<ros2socketcan::SocketCanSender> pSender,
    std::shared_ptr<ros2socketcan::SocketCanReceiver> pReceiver,
    std::chrono::nanoseconds write_timeout,
    std::chrono::nanoseconds read_timeout,
    std::chrono::milliseconds heartbeat_timeout
) : pSender_ { pSender }, pReceiver_ { pReceiver }, 
    write_timeout_ { write_timeout }, read_timeout_ { read_timeout },
    heartbeat_timeout_ { heartbeat_timeout }
{ }

CanReader::~CanReader()
{
    stop();
}

template <typename T>
std::future<T> CanReader::send_request(std::shared_ptr<CanRequest<T>> pRequest)
{
    auto future = pRequest->get_future();
    {
        std::lock_guard lock(request_mutex_);
        pending_list_[pRequest->id.get()].push(pRequest);
        // send request
        uint64_t raw_data = 0;
        pSender_->send(raw_data, pRequest->get_size(), pRequest->id, write_timeout_);
    }
    
    return future;
}

void CanReader::start()
{
    running_ = true;
    reader_thread_ = std::thread(&CanReader::run, this);
}

void CanReader::stop()
{
    running_ = false;
    if (reader_thread_.joinable())
    {
        reader_thread_.join();
    }
}

void CanReader::handle_frame(ros2socketcan::CanId& id, std::array<uint8_t, 8>& data)
{
    std::shared_ptr<CanRequestBase> pRequest;
    std::lock_guard lock(request_mutex_);

    // heartbeat check
    if (id.get() == odrive_cmds::ODRV_HEARTBEAT)
    {
        last_heartbeat_ = std::chrono::steady_clock::now();
    }

    auto it = pending_list_.find(id.get());
    if (it != pending_list_.end() and !it->second.empty()) // request match!
    {
        pRequest = it->second.front();
        it->second.pop();
        
        if (it->second.empty())
            pending_list_.erase(it);
        
        pRequest->fulfill_from_raw(data);
    }
}

void CanReader::run()
{
    while (running_)
    {
        {
            std::lock_guard lock(request_mutex_);
            if (last_heartbeat_.has_value())
            {
                auto now = std::chrono::steady_clock::now();
                if (now - last_heartbeat_.value() > heartbeat_timeout_)
                {
                    // uh oh.. no heartbeat, probably disconnected
                    health_ = RED;
                } else
                {
                    health_ = GREEN;
                }
            }
        }

        std::array<uint8_t, 8> data;
        ros2socketcan::CanId read_id {};
        try
        {
            read_id = pReceiver_->receive(data.data(), read_timeout_);
        } 
        catch (...)
        {
            continue;
        }

        // got something!
        handle_frame(read_id, data);
    }
}

}