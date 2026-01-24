import sys
import os
import glob
import time

# Add project root to path
sys.path.append(os.getcwd())
try:
    sys.path.append(os.path.join(os.getcwd(), 'lerobot', 'src'))
except:
    pass

from lerobot.motors.feetech.feetech import FeetechMotorsBus, Motor, MotorNormMode

def scan_ports():
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/robot_acm*'))
    print(f"Scanning ports: {ports}")
    
    for port in ports:
        print(f"\nChecking port: {port}")
        try:
            # Initialize with NO motors to avoid immediate connect() checks
            # We will manually open the port
            bus = FeetechMotorsBus(port=port, motors={})
            
            # Manually open packet handler without full bus validation
            bus.packet_handler.open_port(port, bus.baudrate)
            
            print("  -> Port opened. Scanning IDs 1-20...")
            found = []
            for mid in range(1, 21):
                try:
                    # Try to ping via read
                    model = bus.read("Model_Number", mid)
                    if model is not None:
                        found.append(f"ID {mid} (Model {model})")
                except Exception:
                    pass
            
            if found:
                print(f"  -> FOUND MOTORS: {found}")
            else:
                print("  -> No motors responded.")
                
            bus.disconnect()
            
        except Exception as e:
            print(f"  -> Scan failed: {e}")

if __name__ == "__main__":
    scan_ports()
