import sys
import os
import glob
import time

# Add project root to path
sys.path.append(os.getcwd())

try:
    from lerobot.motors.feetech import FeetechMotorsBus
except ImportError:
    # Try adding lerobot src to path if standard import fails
    sys.path.append(os.path.join(os.getcwd(), 'lerobot', 'src'))
    try:
        from lerobot.motors.feetech import FeetechMotorsBus
    except ImportError:
        print("Could not import FeetechMotorsBus. Make sure you are in the project root.")
        sys.exit(1)

def scan_ports():
    # Look for both standard ACM and mapped robot_acm ports
    patterns = ['/dev/ttyACM*', '/dev/robot_acm*', '/dev/ttyUSB*']
    ports = []
    for p in patterns:
        ports.extend(glob.glob(p))
    ports = sorted(list(set(ports))) # unique
    
    print(f"Scanning ports: {ports}")
    
    for port in ports:
        print(f"\nChecking port: {port}")
        bus = None
        try:
            # Initialize with empty motors dict to just open the port
            bus = FeetechMotorsBus(port=port, motors={})
            bus.connect()
            
            print("  -> Port opened successfully. Scanning IDs 1-20...")
            found_motors = []
            
            # Manually scan IDs
            for mid in range(1, 21):
                try:
                    # Generic read attempts. 
                    # Model_Number is usually at address 0, length 2
                    # FeetechMotorsBus has low level read methods
                    # we can use read_model_number if available or _read
                    
                    # Try reading model number
                    model = bus.read("Model_Number", mid)
                    if model:
                        found_motors.append(f"ID {mid} (Model {model})")
                except Exception:
                    pass
            
            if found_motors:
                print(f"  -> FOUND MOTORS: {found_motors}")
            else:
                print("  -> No motors responded on this port.")
                
        except Exception as e:
            print(f"  -> Failed to connect/scan: {e}")
        finally:
            if bus:
                try:
                    bus.disconnect()
                except:
                    pass

if __name__ == "__main__":
    scan_ports()
