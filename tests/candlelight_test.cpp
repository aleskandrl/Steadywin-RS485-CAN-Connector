#include <windows.h>
#include <winusb.h>
#include <setupapi.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <conio.h>
#include <algorithm>

#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "winusb.lib")

// Standard USB Device Interface GUID
DEFINE_GUID(GUID_DEVINTERFACE_USB_DEVICE, 0xA5DCBF10L, 0x6530, 0x11D2, 0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED);

void run_test() {
    std::cout << "=== CandleLight Full Node Scan ===" << std::endl;

    // Use DIGCF_ALLCLASSES to see EVERY device in the system
    HDEVINFO hDevInfo = SetupDiGetClassDevsA(NULL, NULL, NULL, DIGCF_PRESENT | DIGCF_ALLCLASSES);
    if (hDevInfo == INVALID_HANDLE_VALUE) {
        std::cerr << "Failed to get device info set." << std::endl;
        return;
    }

    SP_DEVINFO_DATA devInfoData;
    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

    for (DWORD i = 0; SetupDiEnumDeviceInfo(hDevInfo, i, &devInfoData); i++) {
        char buffer[2048];
        if (SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfoData, SPDRP_HARDWAREID, NULL, (PBYTE)buffer, sizeof(buffer), NULL)) {
            std::string hwId = buffer;
            std::string hwIdLower = hwId;
            std::transform(hwIdLower.begin(), hwIdLower.end(), hwIdLower.begin(), ::tolower);

            if (hwIdLower.find("vid_1d50") != std::string::npos && hwIdLower.find("pid_606f") != std::string::npos) {
                std::cout << "\nFound Device Instance: " << hwId << std::endl;
                
                char friendlyName[1024] = "Unknown";
                SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfoData, SPDRP_FRIENDLYNAME, NULL, (PBYTE)friendlyName, sizeof(friendlyName), NULL);
                std::cout << "  Friendly Name: " << friendlyName << std::endl;

                // Try to find ANY interface for this node
                // Since we don't have the GUID, we'll try to find interfaces by enumerating all interfaces 
                // and checking if they belong to this device instance ID.
                char instanceId[1024];
                if (SetupDiGetDeviceInstanceIdA(hDevInfo, &devInfoData, instanceId, sizeof(instanceId), NULL)) {
                    std::cout << "  Instance ID: " << instanceId << std::endl;
                    
                    // We will now scan for all interfaces in the system and see if they link to this instance
                    HDEVINFO hIntDevInfo = SetupDiGetClassDevsA(NULL, NULL, NULL, DIGCF_PRESENT | DIGCF_ALLCLASSES | DIGCF_DEVICEINTERFACE);
                    if (hIntDevInfo != INVALID_HANDLE_VALUE) {
                        SP_DEVICE_INTERFACE_DATA interfaceData;
                        interfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
                        
                        for (DWORD j = 0; SetupDiEnumDeviceInterfaces(hIntDevInfo, NULL, NULL, j, &interfaceData); j++) {
                            // Check if this interface belongs to our device instance
                            // We do this by getting the interface detail and checking its device info
                            SP_DEVINFO_DATA intDevInfoData;
                            intDevInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
                            
                            DWORD detailSize = 0;
                            SetupDiGetDeviceInterfaceDetailA(hIntDevInfo, &interfaceData, NULL, 0, &detailSize, &intDevInfoData);
                            if (GetLastError() == ERROR_INSUFFICIENT_BUFFER) {
                                PSP_DEVICE_INTERFACE_DETAIL_DATA_A detailData = (PSP_DEVICE_INTERFACE_DETAIL_DATA_A)malloc(detailSize);
                                detailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
                                if (SetupDiGetDeviceInterfaceDetailA(hIntDevInfo, &interfaceData, detailData, detailSize, NULL, NULL)) {
                                    char intInstanceId[1024];
                                    if (SetupDiGetDeviceInstanceIdA(hIntDevInfo, &intDevInfoData, intInstanceId, sizeof(intInstanceId), NULL)) {
                                        if (strcmp(instanceId, intInstanceId) == 0) {
                                            std::cout << "  -> Interface found: " << detailData->DevicePath << std::endl;
                                            
                                            // Attempt to open and initialize WinUSB
                                            HANDLE hDev = CreateFileA(detailData->DevicePath, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
                                            if (hDev != INVALID_HANDLE_VALUE) {
                                                WINUSB_INTERFACE_HANDLE hWin;
                                                if (WinUsb_Initialize(hDev, &hWin)) {
                                                    std::cout << "     *** WinUsb_Initialize: SUCCESS! ***" << std::endl;
                                                    WinUsb_Free(hWin);
                                                } else {
                                                    std::cout << "     WinUsb_Initialize FAILED: " << GetLastError() << std::endl;
                                                }
                                                CloseHandle(hDev);
                                            } else {
                                                std::cout << "     CreateFile FAILED: " << GetLastError() << std::endl;
                                            }
                                        }
                                    }
                                }
                                free(detailData);
                            }
                        }
                        SetupDiDestroyDeviceInfoList(hIntDevInfo);
                    }
                }
            }
        }
    }
    SetupDiDestroyDeviceInfoList(hDevInfo);
    std::cout << "\nScan complete." << std::endl;
}

int main() {
    run_test();
    return 0;
}
