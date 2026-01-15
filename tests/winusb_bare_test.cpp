#include <windows.h>
#include <winusb.h>
#include <setupapi.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <chrono>
#include <thread>

#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "winusb.lib")

DEFINE_GUID(GUID_DEVINTERFACE_WINUSB, 0xdee824ef, 0x729b, 0x4a0e, 0x9c, 0x14, 0xb7, 0x11, 0x7d, 0x33, 0xa8, 0x17);

namespace gs_usb {
    enum gs_usb_breq {
        HOST_CONFIG_FORMAT = 1,
        HOST_CONFIG_MODE,
        HOST_CONFIG_BITRATE,
    };
    enum gs_can_mode { MODE_RESET = 0, MODE_NORMAL = 1 };
    struct Bitrate { uint32_t bitrate; };
    struct Mode { uint32_t mode; uint32_t flags; };
    struct HostConfig { uint32_t byte_order; };
    
    // Структура кадра gs_usb
    // ВАЖНО: Выравнивание должно быть 1 байт!
#pragma pack(push, 1)
    struct HostFrame {
        uint32_t echo_id;
        uint32_t can_id;
        uint8_t can_dlc;
        uint8_t channel;
        uint8_t flags;
        uint8_t reserved;
        uint8_t data[8];
    };
#pragma pack(pop)
}

int main() {
    std::cout << "=== BARE WINUSB GS_USB TEST V2 ===" << std::endl;
    std::cout << "HostFrame size: " << sizeof(gs_usb::HostFrame) << " (should be 19-20)" << std::endl;

    HDEVINFO hDevInfo = SetupDiGetClassDevsA(&GUID_DEVINTERFACE_WINUSB, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (hDevInfo == INVALID_HANDLE_VALUE) return 1;

    SP_DEVICE_INTERFACE_DATA ifaceData = { sizeof(SP_DEVICE_INTERFACE_DATA) };
    if (!SetupDiEnumDeviceInterfaces(hDevInfo, NULL, &GUID_DEVINTERFACE_WINUSB, 0, &ifaceData)) return 1;

    DWORD size = 0;
    SetupDiGetDeviceInterfaceDetailA(hDevInfo, &ifaceData, NULL, 0, &size, NULL);
    std::vector<char> buf(size);
    PSP_DEVICE_INTERFACE_DETAIL_DATA_A detail = (PSP_DEVICE_INTERFACE_DETAIL_DATA_A)buf.data();
    detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
    SetupDiGetDeviceInterfaceDetailA(hDevInfo, &ifaceData, detail, size, NULL, NULL);

    HANDLE hDev = CreateFileA(detail->DevicePath, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
    WINUSB_INTERFACE_HANDLE hWin;
    WinUsb_Initialize(hDev, &hWin);

    USB_INTERFACE_DESCRIPTOR ifaceDesc;
    WinUsb_QueryInterfaceSettings(hWin, 0, &ifaceDesc);
    UCHAR inPipe = 0, outPipe = 0;
    for (int i = 0; i < ifaceDesc.bNumEndpoints; i++) {
        WINUSB_PIPE_INFORMATION pipeInfo;
        WinUsb_QueryPipe(hWin, 0, (UCHAR)i, &pipeInfo);
        if (USB_ENDPOINT_DIRECTION_IN(pipeInfo.PipeId)) inPipe = pipeInfo.PipeId;
        else if (USB_ENDPOINT_DIRECTION_OUT(pipeInfo.PipeId)) outPipe = pipeInfo.PipeId;
    }

    ULONG transferred;
    WINUSB_SETUP_PACKET setup;

    // 1. Host Config (byte order)
    gs_usb::HostConfig hconf = { 0x0000BEEF };
    setup = { 0x41, gs_usb::HOST_CONFIG_FORMAT, 0, 0, sizeof(hconf) };
    WinUsb_ControlTransfer(hWin, setup, (PUCHAR)&hconf, sizeof(hconf), &transferred, NULL);

    // 2. Reset
    gs_usb::Mode m_reset = { gs_usb::MODE_RESET, 0 };
    setup = { 0x41, gs_usb::HOST_CONFIG_MODE, 0, 0, sizeof(m_reset) };
    WinUsb_ControlTransfer(hWin, setup, (PUCHAR)&m_reset, sizeof(m_reset), &transferred, NULL);

    // 3. Bitrate 1M
    gs_usb::Bitrate br = { 1000000 };
    setup = { 0x41, gs_usb::HOST_CONFIG_BITRATE, 0, 0, sizeof(br) };
    WinUsb_ControlTransfer(hWin, setup, (PUCHAR)&br, sizeof(br), &transferred, NULL);

    // 4. Start (Normal Mode)
    gs_usb::Mode m_start = { gs_usb::MODE_NORMAL, 0 };
    setup = { 0x41, gs_usb::HOST_CONFIG_MODE, 0, 0, sizeof(m_start) };
    WinUsb_ControlTransfer(hWin, setup, (PUCHAR)&m_start, sizeof(m_start), &transferred, NULL);

    std::cout << "Initialization commands sent. Check LEDs on Canable2!" << std::endl;

    gs_usb::HostFrame hf = {0};
    hf.echo_id = 0xFFFFFFFF;
    hf.can_id = 0x101;
    hf.can_dlc = 1;
    hf.data[0] = 0xA3;

    for(int i=0; i<20; i++) {
        if (WinUsb_WritePipe(hWin, outPipe, (PUCHAR)&hf, sizeof(hf), &transferred, NULL)) {
            std::cout << "TX[0x101] OK (" << transferred << " bytes)" << std::endl;
        } else {
            std::cerr << "TX FAILED: " << GetLastError() << std::endl;
        }

        gs_usb::HostFrame rx_hf;
        ULONG read_bytes;
        ULONG timeout = 200;
        WinUsb_SetPipePolicy(hWin, inPipe, PIPE_TRANSFER_TIMEOUT, sizeof(timeout), &timeout);
        if (WinUsb_ReadPipe(hWin, inPipe, (PUCHAR)&rx_hf, sizeof(rx_hf), &read_bytes, NULL)) {
            std::cout << "RX ID: 0x" << std::hex << rx_hf.can_id << " Data: ";
            for(int j=0; j<rx_hf.can_dlc; j++) std::cout << std::setw(2) << std::setfill('0') << (int)rx_hf.data[j] << " ";
            std::cout << std::dec << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    WinUsb_Free(hWin);
    CloseHandle(hDev);
    return 0;
}
