/****************************NATSUROBOCON ROBOHAN*****************************
    🮕             🮘  🭦🮄🮄🮄🮄🮃🮃🮃🮃🮃🮃🮂🮂🮂🮂🭛   🮘 🭦🭏▂▂▂▂▃▃▃█🭍▄▅▅▆▆   🭔🭀  🮘       🮕
         🮘     🮕    🮕 ⎞ 🭦🮄🮄🮄║🮃🮃🮂🮂🮂 ⎛🮘           🭅🭛  🭥🭔🭍🭑▂▁▂🭐 🭭     🮕
             🮘    🮕   ║ 🭦🮄🮄🮃║🮃🮃🮂🮂🭛 ║  🮕   🮘 ║🮀🮀🮀🮀🮀🮀🮀🮀🮀🮀▌ 🮂🮂   🮕              🮕
        🮕          🮘 🭵🭱  ║  ║   ║ 🭵🭱        ║🬋🬋🬋🬋🬋🬋🬋🬋🬋🬍          🮘 
             🮕       🭴🭰 🭋▂▂▃║▃▄▄🭛  ║  🮘     ║🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋🬋   🮕        🮕
      🮕        🮘   🮕 ║     🭵🭱      🭴🭰    🮕  🭦🮄🮄🮄🮄🮄🮃🮃🮃🮃🮂🮂🮂🮂🮂🭌  🮘     🮕
                  🮘  ║     ║▁▂▂▃🭎   ║🭋🭡 🮘  🭅🭀 🭃🭌  🭃🭌  🭃🭌   ▊   
        🮕      🮕    ⎠⎠ 🭦🮅🮅🮄🮄🮂🮂   🭦   V     🭒🭡 🭦🭡  🭦🭡  🭦🭡 🭦🭩🭡     🮘    🮕   
********************************can_reader.cpp*******************************/ 
/**
 * Maintainer: Oz
 * Description: 'Can Reader' helper class for handling RTR requests
 * Robohan 2025
 * 
 */

#include "necrodrive_ros2/can_reader.hpp"
#include "necrodrive_ros2/odrive_cmds.hpp"

namespace necrodrive_system {

CanReader::CanReader(
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr pClock,
    std::shared_ptr<ros2socketcan::SocketCanSender> pSender,
    std::shared_ptr<ros2socketcan::SocketCanReceiver> pReceiver,
    std::chrono::nanoseconds write_timeout,
    std::chrono::nanoseconds read_timeout,
    std::chrono::milliseconds heartbeat_timeout
) : logger_ { logger }, pClock_ { pClock }, pSender_ { pSender }, pReceiver_ { pReceiver }, 
    write_timeout_ { write_timeout }, read_timeout_ { read_timeout },
    heartbeat_timeout_ { heartbeat_timeout }
{ }

CanReader::~CanReader()
{
    stop();
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
    if (id.frame_type() != ros2socketcan::FrameType::DATA)
        return;
    /*
    RCLCPP_INFO_THROTTLE(
        logger_,
        *pClock_,
        1000,
        "Handling frame, id: 0x%X", id.identifier()
    );
    if (pending_list_.empty()) {
        RCLCPP_INFO_THROTTLE(logger_, *pClock_, 1000, "No pending requests");
    } else {
        std::stringstream ss;
        ss << "Pending request IDs: ";
        for (const auto& pair : pending_list_) {
            ss << "0x" << std::hex << pair.first << " (queue size: " << pair.second.size() << "), ";
        }
        RCLCPP_INFO_THROTTLE(logger_, *pClock_, 1000, "Pending requests: %s", ss.str().c_str());
    }
    */
    std::shared_ptr<CanRequestBase> pRequest;
    std::lock_guard lock(request_mutex_);

    // heartbeat check
    if (id.identifier() == odrive_cmds::ODRV_HEARTBEAT)
    {
        last_heartbeat_ = std::chrono::steady_clock::now();
        return;
    }

    auto it = pending_list_.find(id.identifier());
    if (it != pending_list_.end() and !it->second.empty()) // request match!
    {
        /*
        RCLCPP_INFO(
            logger_,
            "Matched CAN ID: 0x%X (queue size for this ID: %zu)",
            id.identifier(),
            it->second.size()
        );
        */

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
            /*
            RCLCPP_INFO_THROTTLE(
                logger_,
                *pClock_,
                1000,
                "Reading..."
            );
            */
            read_id = pReceiver_->receive(static_cast<void *>(data.data()), read_timeout_);
        } 
        catch (const std::exception &e)
        {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *pClock_,  // clock instance (ROS time or system time)
                1000,                           // throttle period in ms (here: 2 seconds)
                "CAN receive error: %s", e.what()
            );
            continue;
        }
        /*
        
        RCLCPP_INFO_THROTTLE(
            logger_,
            *pClock_,
            1000,
            "Received!"
        );
        */
        // got something!
        handle_frame(read_id, data);
    }
}

}