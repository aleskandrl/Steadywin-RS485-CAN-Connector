#include "steadywin/gs_usb_can_port.h"
#include <iostream>
#include <iomanip>
#include <mutex>
#include <algorithm>
#include <vector>
#include <string>
#include <cstdint>
#include <cstring> // <--- Добавлено для memcpy

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#include <windows.h>
#include <initguid.h>
#include <usbiodef.h>
#include <setupapi.h>

// Standard GUID for WinUSB (Zadig)
// {dee824ef-729b-4a0e-9c14-b7117d33a817}
DEFINE_GUID(GUID_DEVINTERFACE_WINUSB, 
    0xdee824ef, 0x729b, 0x4a0e, 0x9c, 0x14, 0xb7, 0x11, 0x7d, 0x33, 0xa8, 0x17);

namespace steadywin {

static std::recursive_mutex g_usb_mutex;

GsUsbCanPort::GsUsbCanPort() : is_open_(false) {}

GsUsbCanPort::~GsUsbCanPort() {
    close();
}

bool GsUsbCanPort::open(const char* device_path, unsigned int baud_rate) {
    std::lock_guard<std::recursive_mutex> lock(g_usb_mutex);
    
    device_handle_ = CreateFileA(device_path,
                                GENERIC_READ | GENERIC_WRITE,
                                FILE_SHARE_READ | FILE_SHARE_WRITE,
                                NULL,
                                OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
                                NULL);

    if (device_handle_ == INVALID_HANDLE_VALUE) {
        return false;
    }

    if (!WinUsb_Initialize(device_handle_, &usb_handle_)) {
        CloseHandle(device_handle_);
        device_handle_ = INVALID_HANDLE_VALUE;
        return false;
    }

    USB_INTERFACE_DESCRIPTOR ifaceDesc;
    if (!WinUsb_QueryInterfaceSettings(usb_handle_, 0, &ifaceDesc)) {
        close();
        return false;
    }

    bulk_in_pipe_ = 0;
    bulk_out_pipe_ = 0;
    for (int i = 0; i < ifaceDesc.bNumEndpoints; i++) {
        WINUSB_PIPE_INFORMATION pipeInfo;
        if (WinUsb_QueryPipe(usb_handle_, 0, (UCHAR)i, &pipeInfo)) {
            if (USB_ENDPOINT_DIRECTION_IN(pipeInfo.PipeId)) bulk_in_pipe_ = pipeInfo.PipeId;
            else if (USB_ENDPOINT_DIRECTION_OUT(pipeInfo.PipeId)) bulk_out_pipe_ = pipeInfo.PipeId;
        }
    }

    if (bulk_in_pipe_ == 0 || bulk_out_pipe_ == 0) {
        close();
        return false;
    }

    // --- GS-USB CONFIGURATION ---
    ULONG transferred;
    WINUSB_SETUP_PACKET setup;

    gs_usb::HostConfig hconf = { 0x0000BEEF };
    setup = { 0x41, gs_usb::HOST_CONFIG_FORMAT, 0, 0, sizeof(hconf) };
    WinUsb_ControlTransfer(usb_handle_, setup, (PUCHAR)&hconf, sizeof(hconf), &transferred, NULL);

    gs_usb::Mode m_reset = { gs_usb::MODE_RESET, 0 };
    setup = { 0x41, gs_usb::HOST_CONFIG_MODE, 0, 0, sizeof(m_reset) };
    WinUsb_ControlTransfer(usb_handle_, setup, (PUCHAR)&m_reset, sizeof(m_reset), &transferred, NULL);

    gs_usb::Bitrate br = { baud_rate };
    setup = { 0x41, gs_usb::HOST_CONFIG_BITRATE, 0, 0, sizeof(br) };
    if (!WinUsb_ControlTransfer(usb_handle_, setup, (PUCHAR)&br, sizeof(br), &transferred, NULL)) {
        close();
        return false;
    }

    gs_usb::Mode m_start = { gs_usb::MODE_NORMAL, 0 };
    setup = { 0x41, gs_usb::HOST_CONFIG_MODE, 0, 0, sizeof(m_start) };
    if (!WinUsb_ControlTransfer(usb_handle_, setup, (PUCHAR)&m_start, sizeof(m_start), &transferred, NULL)) {
        close();
        return false;
    }

    is_open_ = true;
    return true;
}

void GsUsbCanPort::close() {
    std::lock_guard<std::recursive_mutex> lock(g_usb_mutex);
    if (usb_handle_) {
        gs_usb::Mode m_reset = { gs_usb::MODE_RESET, 0 };
        WINUSB_SETUP_PACKET setup = { 0x41, gs_usb::HOST_CONFIG_MODE, 0, 0, sizeof(m_reset) };
        ULONG bytes;
        WinUsb_ControlTransfer(usb_handle_, setup, (PUCHAR)&m_reset, sizeof(m_reset), &bytes, NULL);
        WinUsb_Free(usb_handle_);
        usb_handle_ = NULL;
    }
    if (device_handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(device_handle_);
        device_handle_ = INVALID_HANDLE_VALUE;
    }
    is_open_ = false;
}

bool GsUsbCanPort::isOpen() const { return is_open_; }

bool GsUsbCanPort::write(const CanFrame& frame) {
    if (!is_open_ || !usb_handle_) return false;
    std::lock_guard<std::recursive_mutex> lock(g_usb_mutex);

    gs_usb::HostFrame hf = {0};
    hf.echo_id = 0xFFFFFFFF;
    hf.can_id = frame.id;
    if (frame.is_extended) hf.can_id |= 0x80000000U;
    if (frame.is_rtr) hf.can_id |= 0x40000000U;
    hf.can_dlc = frame.dlc;
    std::memcpy(hf.data, frame.data, 8);

    ULONG written;
    return WinUsb_WritePipe(usb_handle_, bulk_out_pipe_, (PUCHAR)&hf, sizeof(hf), &written, NULL) == TRUE;
}

bool GsUsbCanPort::read(CanFrame& frame, unsigned int timeout_ms) {
    if (!is_open_ || !usb_handle_) return false;
    std::lock_guard<std::recursive_mutex> lock(g_usb_mutex);

    WinUsb_SetPipePolicy(usb_handle_, bulk_in_pipe_, PIPE_TRANSFER_TIMEOUT, sizeof(timeout_ms), &timeout_ms);

    gs_usb::HostFrame hf;
    ULONG read_bytes;
    if (WinUsb_ReadPipe(usb_handle_, bulk_in_pipe_, (PUCHAR)&hf, sizeof(hf), &read_bytes, NULL)) {
        if (read_bytes >= 12) {
            frame.id = hf.can_id & 0x1FFFFFFF;
            frame.is_extended = (hf.can_id & 0x80000000U) != 0;
            frame.is_rtr = (hf.can_id & 0x40000000U) != 0;
            frame.dlc = hf.can_dlc;
            // BUGFIX: was wmemcpy
            std::memcpy(frame.data, hf.data, 8);
            return true;
        }
    }
    return false;
}

std::vector<GsUsbDeviceInfo> GsUsbCanPort::enumerateDevices() {
    std::vector<GsUsbDeviceInfo> devices;
    HDEVINFO hDevInfo = SetupDiGetClassDevsA(&GUID_DEVINTERFACE_WINUSB, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (hDevInfo == INVALID_HANDLE_VALUE) return devices;

    SP_DEVICE_INTERFACE_DATA ifaceData = { sizeof(SP_DEVICE_INTERFACE_DATA) };
    for (DWORD i = 0; SetupDiEnumDeviceInterfaces(hDevInfo, NULL, &GUID_DEVINTERFACE_WINUSB, i, &ifaceData); i++) {
        DWORD size = 0;
        SetupDiGetDeviceInterfaceDetailA(hDevInfo, &ifaceData, NULL, 0, &size, NULL);
        std::vector<char> buf(size);
        PSP_DEVICE_INTERFACE_DETAIL_DATA_A detail = (PSP_DEVICE_INTERFACE_DETAIL_DATA_A)buf.data();
        detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);

        if (SetupDiGetDeviceInterfaceDetailA(hDevInfo, &ifaceData, detail, size, NULL, NULL)) {
            std::string path = detail->DevicePath;
            std::string lowPath = path;
            std::transform(lowPath.begin(), lowPath.end(), lowPath.begin(), ::tolower);

            if (lowPath.find("vid_1d50") != std::string::npos && lowPath.find("pid_606f") != std::string::npos) {
                std::string desc = "CandleLight USB CAN";
                if (lowPath.find("mi_00") != std::string::npos) desc += " (Interface 0)";
                devices.push_back({path, desc});
            }
        }
    }
    SetupDiDestroyDeviceInfoList(hDevInfo);
    return devices;
}

} // namespace steadywin

#else
namespace steadywin {
    GsUsbCanPort::GsUsbCanPort() : is_open_(false) {}
    GsUsbCanPort::~GsUsbCanPort() {}
    bool GsUsbCanPort::open(const char*, unsigned int) { return false; }
    void GsUsbCanPort::close() {}
    bool GsUsbCanPort::isOpen() const { return false; }
    bool GsUsbCanPort::write(const CanFrame&) { return false; }
    bool GsUsbCanPort::read(CanFrame&, unsigned int) { return false; }
    std::vector<GsUsbDeviceInfo> GsUsbCanPort::enumerateDevices() { return {}; }
}
#endif
