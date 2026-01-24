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

from lerobot.motors.feetech.feetech import FeetechMotorsBus

def scan_ports():
    ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/robot_acm*'))
    baudrates = [1000000, 115200, 57600, 500000, 230400]
    
    for port in ports:
        print(f"\nChecking port: {port}")
        
        for baud in baudrates:
            print(f"  Testing baudrate: {baud}...")
            bus = None
            try:
                # Initialize with empty motors dict
                bus = FeetechMotorsBus(port=port, motors={})
                
                if hasattr(bus, 'port_handler'):
                    bus.port_handler.openPort()
                    bus.port_handler.setBaudRate(baud)
                else:
                    # Fallback
                    bus.connect()

                # Scan
                found = []
                for mid in range(1, 21):
                    try:
                        model = bus.read("Model_Number", mid)
                        if model is not None:
                            found.append(f"ID {mid} (Model {model})")
                    except:
                        pass
                
                if found:
                    print(f"  -> [SUCCESS] FOUND MOTORS at {baud}: {found}")
                    bus.disconnect()
                    break # Stop trying other baudrates for this port
                else:
                    pass # silent if nothing found
                    
                bus.disconnect()
                
            except Exception as e:
                # print(f"    Error at {baud}: {e}")
                pass
        else:
            print("  -> No motors responded on any baudrate.")

if __name__ == "__main__":
    scan_ports()
