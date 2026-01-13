#pragma once

#include "steadywin/can_port_interface.h"
#include <memory>
#include <string>
#include <vector>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#include <windows.h>
#include <winusb.h>
#include <setupapi.h>
#endif

namespace steadywin {

namespace gs_usb {
    // Правильные коды команд для протокола CandleLight/GS-USB
    constexpr uint8_t HOST_CONFIG_FORMAT     = 0;
    constexpr uint8_t HOST_CONFIG_BITRATE    = 1;
    constexpr uint8_t HOST_CONFIG_MODE       = 2;

    constexpr uint32_t MODE_RESET            = 0;
    constexpr uint32_t MODE_NORMAL           = 1;
    
    #pragma pack(push, 1)
    struct HostConfig {
        uint32_t byte_order;
    };

    struct Bitrate {
        uint32_t bitrate;
    };

    struct Mode {
        uint32_t mode;
        uint32_t flags;
    };

    struct HostFrame {
        uint32_t echo_id;
        uint32_t can_id;
        uint8_t  can_dlc;
        uint8_t  channel;
        uint8_t  flags;
        uint8_t  reserved;
        uint8_t  data[8];
    };
    #pragma pack(pop)
}

struct GsUsbDeviceInfo {
    std::string path;
    std::string description;
};

class GsUsbCanPort : public ICanPort {
public:
    GsUsbCanPort();
    ~GsUsbCanPort() override;

    bool open(const char* device_path, unsigned int baud_rate) override;
    void close() override;
    bool isOpen() const override;
    bool write(const CanFrame& frame) override;
    bool read(CanFrame& frame, unsigned int timeout_ms) override;

    static std::vector<GsUsbDeviceInfo> enumerateDevices();

private:
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
    HANDLE device_handle_{INVALID_HANDLE_VALUE};
    WINUSB_INTERFACE_HANDLE usb_handle_{NULL};
    uint8_t bulk_in_pipe_{0};
    uint8_t bulk_out_pipe_{0};
#endif
    bool is_open_{false};
};

} // namespace steadywin