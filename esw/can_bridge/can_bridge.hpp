#pragma once

#include "pch.hpp"

#include "can_net_link.hpp"

namespace mrover {

    // [0-28]: CAN identifier (11/29bit)
    // [29]: Error frame flag (0 = data frame, 1 = error frame)
    // [30]: Remote transmission request flag (1 = rtr frame)
    // [31]: Frame format flag (0 = standard 11bit, 1 = extended 29bit)
    // In the future, if we want to send different types of messages,
    // we should have logic for switching bits such as errorFrameFlag.
    struct RawCanFdId {
        std::uint32_t identifier : 29 {};
        bool isErrorFrame : 1 {};
        bool isRemoteTransmissionRequest : 1 {};
        bool isExtendedFrame : 1 {};
    };
    static_assert(sizeof(RawCanFdId) == sizeof(canid_t));

    // Our custom message ID format, this is not part of the standard
    // Based on the mjbots protocol
    // By default CAN is broadcast, but this allows for directed messages
    // Devices can set up filters (usually in hardware) to only receive messages meant for them
    struct CanFdMessageId {
        std::uint8_t destination{};
        std::uint8_t source : 7 {};
        bool replyRequired : 1 {};
    };
    static_assert(sizeof(CanFdMessageId) == 2);

    struct CanFdPubSub {
        rclcpp::Publisher<msg::CAN>::SharedPtr publisher;
        rclcpp::Subscription<msg::CAN>::SharedPtr subscriber;
    };

    class CanBridge : public rclcpp::Node {
    public:
        CanBridge();

        ~CanBridge() override;

    private:
        std::string mInterface;

        canfd_frame mReadFrame{};
        CanNetLink mCanNetLink;
        std::optional<boost::asio::posix::basic_stream_descriptor<>> mStream;
        std::jthread mIoThread;
        boost::asio::io_service mIoService;

        boost::bimap<std::string, std::uint8_t> mDevices;
        std::unordered_map<std::string, CanFdPubSub> mDevicesPubSub;

        [[nodiscard]] auto setupSocket() const -> int;

        void readFrameAsync();

        void frameReadCallback();

        void frameSendRequestCallback(msg::CAN::ConstSharedPtr const& msg);
    };

} // namespace mrover
