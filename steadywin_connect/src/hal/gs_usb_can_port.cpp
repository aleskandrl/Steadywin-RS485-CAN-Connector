#include "steadywin/hal/gs_usb_can_port.h"
#include "libusb.h"
#include <iostream>
#include <array>
#include <cstring>
#include <sstream>

namespace steadywin {

// GS_USB definitions
namespace {
    constexpr uint8_t GS_USB_BREQ_HOST_FORMAT    = 0;
    constexpr uint8_t GS_USB_BREQ_BITTIMING      = 1;
    constexpr uint8_t GS_USB_BREQ_MODE           = 2;
    constexpr uint8_t GS_USB_BREQ_BERR           = 3;
    constexpr uint8_t GS_USB_BREQ_BT_CONST       = 4;
    constexpr uint8_t GS_USB_BREQ_DEVICE_CONFIG  = 5;
    constexpr uint8_t GS_USB_BREQ_TIMESTAMP      = 6;
    constexpr uint8_t GS_USB_BREQ_IDENTIFY       = 7;

    constexpr uint32_t GS_USB_MODE_RESET          = 0;
    constexpr uint32_t GS_USB_MODE_START          = 1;

    struct GsUsbFrame {
        uint32_t echo_id;
        uint32_t can_id;
        uint8_t can_dlc;
        uint8_t channel;
        uint8_t flags;
        uint8_t reserved;
        uint8_t data[8];
    };
}

struct GsUsbCanPort::Impl {
    libusb_context* ctx = nullptr;
    libusb_device_handle* dev = nullptr;
    uint8_t intf_num = 0;
    
    // Internal buffer for receiving
    uint8_t rx_buf[2048]; // Enough for multiple frames

    // Helper to read/write LE
    static uint32_t read_u32(const uint8_t* p) {
        return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
    }

    static void write_u32(uint8_t* p, uint32_t v) {
        p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
    }
    
    bool set_mode(uint32_t mode, uint32_t flags) {
        struct {
            uint32_t mode;
            uint32_t flags;
        } m = {mode, flags};
        
        return libusb_control_transfer(
            dev,
            LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
            GS_USB_BREQ_MODE,
            0,
            intf_num,
            reinterpret_cast<uint8_t*>(&m),
            sizeof(m),
            1000) >= 0;
    }

    Impl() {
        libusb_init(&ctx);
    }

    ~Impl() {
        if (dev) {
            libusb_release_interface(dev, intf_num);
            libusb_close(dev);
        }
        if (ctx) {
            libusb_exit(ctx);
        }
    }
};

GsUsbCanPort::GsUsbCanPort() : impl_(std::make_unique<Impl>()) {}

GsUsbCanPort::~GsUsbCanPort() = default;

bool GsUsbCanPort::isOpen() const {
    return impl_->dev != nullptr;
}

void GsUsbCanPort::close() {
    if (impl_->dev) {
        // Reset mode
        impl_->set_mode(GS_USB_MODE_RESET, 0);
        
        libusb_release_interface(impl_->dev, impl_->intf_num);
        libusb_close(impl_->dev);
        impl_->dev = nullptr;
    }
}

bool GsUsbCanPort::open(const char* device_path, unsigned int baud_rate) {
    if (isOpen()) close();
    
    // Find device
    libusb_device** devs;
    ssize_t cnt = libusb_get_device_list(impl_->ctx, &devs);
    if (cnt < 0) return false;
    
    libusb_device* target = nullptr;
    
    // If device_path is provided, try to match it.
    // Logic: if device_path is empty/null, pick first. 
    // If provided, we assume it's in format "usb:bus:addr" or similar, but for now let's just pick first 
    // OR try to match based on user code if possible. 
    // Implementation Plan said: "Main uses enumerateDevices".
    // enumerateDevices likely returns paths. We should match that path.
    // Let's implement robust matching.
    
    std::string target_path = (device_path ? device_path : "");

    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device_descriptor desc;
        libusb_get_device_descriptor(devs[i], &desc);
        if (desc.idVendor == 0x1d50 && desc.idProduct == 0x606f) {
            if (target_path.empty()) {
                target = devs[i];
                break;
            } else {
                // Construct path for this device and compare
                uint8_t bus = libusb_get_bus_number(devs[i]);
                uint8_t addr = libusb_get_device_address(devs[i]);
                std::stringstream ss;
                ss << "usb:" << (int)bus << ":" << (int)addr;
                if (ss.str() == target_path) {
                    target = devs[i];
                    break;
                }
            }
        }
    }
    
    if (!target) {
        libusb_free_device_list(devs, 1);
        std::cerr << "GS_USB device not found\n";
        return false;
    }
    
    int r = libusb_open(target, &impl_->dev);
    libusb_free_device_list(devs, 1);
    if (r < 0) {
        std::cerr << "Failed to open device\n";
        return false;
    }
    
    if (libusb_kernel_driver_active(impl_->dev, 0) == 1) {
        libusb_detach_kernel_driver(impl_->dev, 0);
    }
    
    if (libusb_claim_interface(impl_->dev, 0) < 0) {
        std::cerr << "Failed to claim interface\n";
        close();
        return false;
    }
    
    // Prepare device
    uint32_t host_format = 0x0000BEEF; // LE
    libusb_control_transfer(
        impl_->dev,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
        GS_USB_BREQ_HOST_FORMAT,
        1, // value
        impl_->intf_num,
        reinterpret_cast<uint8_t*>(&host_format),
        sizeof(host_format),
        1000
    );
    
    // Read bit timing constants to get clock
    uint8_t bt_const[40];
    if (libusb_control_transfer(
            impl_->dev,
            LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
            GS_USB_BREQ_BT_CONST,
            0,
            impl_->intf_num,
            bt_const,
            sizeof(bt_const),
            1000) < 0) {
        std::cerr << "Failed to read BT constants\n";
        close();
        return false;
    }
    
    const uint32_t fclk_can = Impl::read_u32(bt_const + 4);
    
    // Bit timing calculation
    std::array<uint8_t, 20> bit_timing{};
    uint32_t prop = 1;
    uint32_t ph1 = 12;
    uint32_t ph2 = 2;
    uint32_t sjw = 1;
    uint32_t brp = 3;
    
    if (fclk_can == 80000000u) {
        brp = 5;
    } else if (fclk_can == 170000000u) {
        // Fix for 170MHz clock
        brp = 10;
        prop = 6;
        ph1 = 8;
        ph2 = 2;
        sjw = 1;
    } else if (fclk_can == 48000000u) {
        brp = 3;
    }
    
    Impl::write_u32(bit_timing.data() + 0, prop);
    Impl::write_u32(bit_timing.data() + 4, ph1);
    Impl::write_u32(bit_timing.data() + 8, ph2);
    Impl::write_u32(bit_timing.data() + 12, sjw);
    Impl::write_u32(bit_timing.data() + 16, brp);
    
    if (libusb_control_transfer(
            impl_->dev,
            LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
            GS_USB_BREQ_BITTIMING,
            0,
            impl_->intf_num,
            bit_timing.data(),
            bit_timing.size(),
            1000) < 0) {
        std::cerr << "Failed to set bit timing\n";
        close();
        return false;
    }
    
    if (!impl_->set_mode(GS_USB_MODE_START, 0)) {
        std::cerr << "Failed to start mode\n";
        close();
        return false;
    }
    
    return true;
}

bool GsUsbCanPort::write(const CanFrame& frame) {
    if (!isOpen()) return false;
    
    GsUsbFrame uframe{};
    uframe.echo_id = frame.id;
    uframe.can_id = frame.id;
    uframe.can_dlc = frame.dlc;
    uframe.channel = 0;
    uframe.flags = 0;
    uframe.reserved = 0;
    std::memcpy(uframe.data, frame.data, frame.dlc);
    
    int transferred = 0;
    int r = libusb_bulk_transfer(
        impl_->dev,
        0x02 | LIBUSB_ENDPOINT_OUT, // Endpoint 2 OUT
        reinterpret_cast<uint8_t*>(&uframe),
        sizeof(uframe),
        &transferred,
        100
    );
    
    return (r == 0 && transferred == sizeof(uframe));
}

bool GsUsbCanPort::read(CanFrame& frame, unsigned int timeout_ms) {
    if (!isOpen()) return false;
    
    int transferred = 0;
    // Expected frame size is 32 bytes (sizeof GsUsbFrame)
    int r = libusb_bulk_transfer(
        impl_->dev,
        0x81 | LIBUSB_ENDPOINT_IN, // Endpoint 1 IN
        impl_->rx_buf,
        sizeof(GsUsbFrame), // Try to read one frame
        &transferred,
        timeout_ms
    );
    
    if (r == 0 && transferred == sizeof(GsUsbFrame)) {
        GsUsbFrame* uframe = reinterpret_cast<GsUsbFrame*>(impl_->rx_buf);
        
        // Filter logic from SteadyWinCAN.cpp
        if (uframe->can_dlc > 8 || (uframe->can_id & 0x80000000u)) {
             return false;
        }
        
        frame.id = uframe->can_id;
        frame.dlc = uframe->can_dlc;
        std::memcpy(frame.data, uframe->data, 8);
        frame.is_extended = false; // Simple assumption
        frame.is_rtr = false;
        return true;
    }
    
    return false;
}

std::vector<GsUsbDeviceInfo> GsUsbCanPort::enumerateDevices() {
    std::vector<GsUsbDeviceInfo> devices;
    
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) < 0) return devices;
    
    libusb_device** devs;
    ssize_t cnt = libusb_get_device_list(ctx, &devs);
    if (cnt < 0) {
        libusb_exit(ctx);
        return devices;
    }
    
    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device_descriptor desc;
        libusb_get_device_descriptor(devs[i], &desc);
        if (desc.idVendor == 0x1d50 && desc.idProduct == 0x606f) {
            GsUsbDeviceInfo info;
            
            uint8_t bus = libusb_get_bus_number(devs[i]);
            uint8_t addr = libusb_get_device_address(devs[i]);
            std::stringstream ss;
            ss << "usb:" << (int)bus << ":" << (int)addr;
            info.path = ss.str();
            
            // Try to get product string
            libusb_device_handle* handle = nullptr;
            if (libusb_open(devs[i], &handle) == 0) {
                char product[256];
                if (libusb_get_string_descriptor_ascii(handle, desc.iProduct, (unsigned char*)product, sizeof(product)) > 0) {
                   info.description = product;
                } else {
                   info.description = "GS-USB CAN Device";
                }
                libusb_close(handle);
            } else {
                info.description = "GS-USB CAN Device (Protected)";
            }
            
            devices.push_back(info);
        }
    }
    
    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);
    return devices;
}


} // namespace steadywin
