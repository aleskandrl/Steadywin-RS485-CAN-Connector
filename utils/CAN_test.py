import usb.core
import usb.util
import usb.backend.libusb1
import sys
import os

# --- BACKEND SETUP ---
# Get the absolute path to libusb-1.0.dll in the script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
libusb_path = os.path.join(script_dir, 'libusb-1.0.dll')

# Check if the DLL exists
if not os.path.exists(libusb_path):
    print(f"ERROR: File not found: {libusb_path}")
    print("Please ensure libusb-1.0.dll is in the same folder as this script.")
    sys.exit(1)

# Force load the backend
try:
    backend = usb.backend.libusb1.get_backend(find_library=lambda x: libusb_path)
except Exception as e:
    print(f"ERROR loading DLL: {e}")
    sys.exit(1)
# -------------------------

# CandleLight / GS-USB IDs
VID = 0x1d50
PID = 0x606f

print(f"--- Driver Test for VID:0x{VID:04x} PID:0x{PID:04x} ---")

# Find the device
dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)

if dev is None:
    print("Device not found.")
    sys.exit(1)

print(f"Device found: {dev.manufacturer} {dev.product}")

# --- IMPORTANT FOR WINDOWS ---
try:
    # In Windows, we must explicitly set the configuration before using the device
    dev.set_configuration()
    print("Configuration set successfully.")
except Exception as e:
    print(f"Warning during configuration setup (this can be normal): {e}")

# --- TEST INTERFACE 0 (CAN) ---
print("\nAttempting to open Interface 0 (CAN)...")
try:
    # Try to claim the interface. If the driver is not WinUSB, this will fail.
    usb.util.claim_interface(dev, 0)
    print("✅ SUCCESS! Interface 0 is accessible.")
    print("   This means the WinUSB driver is installed CORRECTLY.")
    print("   The issue is in your C++ code: it is likely opening the wrong device path.")

    # Release the interface
    usb.util.release_interface(dev, 0)

except usb.core.USBError as e:
    print(f"❌ ERROR accessing the interface: {e}")
    print("-" * 40)
    print("DIAGNOSIS:")
    if "Access denied" in str(e) or "Entity not found" in str(e):
        print("1. WinUSB driver is installed on the WRONG device level (Parent instead of Interface).")
        print("2. Or the device is being used by another program.")
        print("SOLUTION: Use Zadig, select 'Interface 0' (MI:00), and reinstall the driver.")
    else:
        print("Unknown driver error.")

input("\nPress Enter to exit...")