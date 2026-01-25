"""
LD19 360° LIDAR Sensor Driver
Full 360° distance scanning via USB serial (UART at 230400 baud).
Based on LDROBOT LD19/LD06 protocol.
"""
import logging
import struct
import threading
import time
from typing import Optional, List, Tuple, Dict

logger = logging.getLogger(__name__)


class Lidar360:
    """LDROBOT LD19 360° LIDAR sensor driver."""
    
    # LD19 Protocol Constants
    HEADER = 0x54
    PKT_LENGTH = 47  # Total packet size
    POINT_COUNT = 12  # Points per packet
    
    def __init__(self, port: str = "", baud_rate: int = 230400):
        self.port = port
        self.baud_rate = baud_rate
        
        self._serial = None
        self._connected = False
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        
        # Full 360° scan buffer: dict of angle -> distance
        self._scan_data: Dict[int, float] = {}
        self._last_scan_time = 0.0
        
        # Forward distance for compatibility
        self.forward_distance: Optional[float] = None
    
    @property
    def connected(self) -> bool:
        return self._connected
    
    def connect(self) -> bool:
        """Attempt to connect to the LIDAR sensor."""
        if not self.port:
            logger.warning("Lidar360: No port configured")
            return False
        
        try:
            import serial
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            self._connected = True
            logger.info(f"Lidar360: Connected on {self.port} @ {self.baud_rate}")
            return True
        except Exception as e:
            logger.error(f"Lidar360: Connection failed - {e}")
            self._connected = False
            return False
    
    def disconnect(self):
        """Disconnect from the sensor."""
        self.stop()
        
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        
        self._connected = False
        logger.info("Lidar360: Disconnected")
    
    def start(self):
        """Start continuous reading (motor spins up)."""
        if self._running:
            return
        
        if not self._connected and not self.connect():
            logger.warning("Lidar360: Cannot start - not connected")
            return
        
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        logger.info("Lidar360: Started scanning")
    
    def stop(self):
        """Stop continuous reading (motor spins down)."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("Lidar360: Stopped scanning")
    
    def get_decimated_scan(self, step_degrees: int = 10) -> Dict[int, float]:
        """Get a simplified scan with points every n degrees."""
        with self._lock:
            if not self._scan_data:
                return {}
            
            # Return sparse data for visualization
            sparse = {}
            for angle, dist in self._scan_data.items():
                if angle % step_degrees == 0:
                    sparse[angle] = round(dist, 1)
            return sparse

    def _read_loop(self):
        """Background reading loop - parses LD19 packets."""
        PACKET_SIZE = 47
        buffer = bytearray()
        
        while self._running and self._serial:
            try:
                data = self._serial.read(128)
                if not data:
                    continue
                
                buffer.extend(data)
                
                while len(buffer) >= PACKET_SIZE:
                    # Search for Header (0x54) + VerLen (0x2C)
                    header_idx = -1
                    for i in range(len(buffer) - 1):
                        if buffer[i] == 0x54 and buffer[i + 1] == 0x2C:
                            header_idx = i
                            break
                    
                    if header_idx == -1:
                        # Keep last byte just in case
                        buffer = buffer[-1:] if buffer else bytearray()
                        break
                    
                    if header_idx > 0:
                        buffer = buffer[header_idx:]
                    
                    if len(buffer) < PACKET_SIZE:
                        break
                    
                    packet = bytes(buffer[:PACKET_SIZE])
                    buffer = buffer[PACKET_SIZE:]
                    
                    self._parse_packet(packet)
                    
            except Exception as e:
                logger.debug(f"Lidar360: Read error - {e}")
                time.sleep(0.1)
    
    def _parse_packet(self, packet: bytes):
        """Parse a single LD19 data packet (47 bytes)."""
        if len(packet) != 47 or packet[0] != 0x54 or packet[1] != 0x2C:
            return
        
        # Parse angles (little endian, 0.01 degree units)
        fsa = (packet[5] << 8 | packet[4]) / 100.0  # Start angle
        lsa = (packet[43] << 8 | packet[42]) / 100.0  # End angle
        
        # Handle angle wraparound
        if lsa < fsa:
            lsa += 360.0
        
        # Calculate angle step for 12 points
        angle_step = (lsa - fsa) / 11.0 if lsa != fsa else 0
        
        with self._lock:
            for i in range(12):
                offset = 6 + i * 3  # Each point is 3 bytes
                
                # Distance is little endian, in mm
                distance_mm = packet[offset] | (packet[offset + 1] << 8)
                # Intensity is third byte (we don't use it but could for confidence)
                # intensity = packet[offset + 2]
                
                # Calculate angle for this point
                angle_deg = (fsa + i * angle_step) % 360.0
                angle_int = int(round(angle_deg))
                
                # Store distance in cm (convert from mm)
                if distance_mm > 0:  # Only store valid readings
                    self._scan_data[angle_int] = distance_mm / 10.0
            
            self._last_scan_time = time.time()
            
            # Update forward distance (angle 0° ± 5 degrees, with wraparound)
            # Front is 0°, so we check 355-360 and 0-5
            fwd_distances = []
            for a in list(range(0, 6)) + list(range(355, 360)):
                if a in self._scan_data:
                    fwd_distances.append(self._scan_data[a])
            if fwd_distances:
                self.forward_distance = min(fwd_distances)
    
    def get_full_scan(self) -> List[Tuple[int, float]]:
        """Get the full 360° scan as sorted (angle, distance_cm) list."""
        with self._lock:
            return sorted(self._scan_data.items())
    
    def get_distance_at_angle(self, angle: float, tolerance: float = 5.0) -> Optional[float]:
        """Get distance at specific angle (±tolerance degrees)."""
        angle = angle % 360
        with self._lock:
            for offset in range(int(tolerance) + 1):
                for a in [int(angle + offset) % 360, int(angle - offset) % 360]:
                    if a in self._scan_data:
                        return self._scan_data[a]
        return None
    
    def get_forward_distance(self) -> Optional[float]:
        """Get distance straight ahead (0°)."""
        return self.forward_distance
    
    def get_distances_in_range(self, start_angle: float, end_angle: float) -> List[Tuple[int, float]]:
        """Get distances in an angle range. Returns (angle, distance_cm) pairs."""
        start_angle = start_angle % 360
        end_angle = end_angle % 360
        
        with self._lock:
            results = []
            for angle, dist in self._scan_data.items():
                if start_angle <= end_angle:
                    if start_angle <= angle <= end_angle:
                        results.append((angle, dist))
                else:
                    # Wraparound case (e.g., 350 to 10)
                    if angle >= start_angle or angle <= end_angle:
                        results.append((angle, dist))
            return sorted(results)
    
    def get_min_distance_in_range(self, start_angle: float, end_angle: float) -> Optional[float]:
        """Get minimum distance in an angle range."""
        distances = self.get_distances_in_range(start_angle, end_angle)
        if not distances:
            return None
        return min(d for _, d in distances)
    
    def find_gap_in_range(self, start_angle: float, end_angle: float, 
                          min_gap_width_deg: float = 15.0, 
                          min_gap_depth_cm: float = 100.0) -> dict:
        """
        Find passable gaps in a given angle range.
        
        Returns:
            dict with 'found', 'center_angle', 'width_deg', 'left_edge', 'right_edge'
        """
        # Normalize angles to 0-360 for internal use
        # But keep the semantics: negative = right, positive = left
        distances = []
        
        with self._lock:
            for angle in range(int(start_angle), int(end_angle) + 1):
                a_normalized = angle % 360
                if a_normalized in self._scan_data:
                    distances.append((angle, self._scan_data[a_normalized]))
        
        if len(distances) < 10:
            return {'found': False, 'reason': 'insufficient_data'}
        
        distances.sort(key=lambda x: x[0])
        
        # Calculate depth threshold dynamically
        dist_values = [d for _, d in distances]
        p20 = sorted(dist_values)[len(dist_values) // 5]
        p80 = sorted(dist_values)[len(dist_values) * 4 // 5]
        
        if (p80 - p20) < 20:
            return {'found': False, 'reason': 'no_depth_contrast'}
        
        threshold = (p20 + p80) / 2
        
        # Find gap segments and smooth noise
        raw_gap_mask = [d > threshold and d > min_gap_depth_cm for _, d in distances]
        
        noise_fill_limit = 3 
        smoothed_mask = raw_gap_mask[:]
        
        i = 0
        while i < len(smoothed_mask):
            if not smoothed_mask[i]:
                j = i + 1
                while j < len(smoothed_mask) and not smoothed_mask[j]:
                    j += 1
                
                if j - i <= noise_fill_limit:
                    for k in range(i, j):
                        smoothed_mask[k] = True
                
                i = j
            else:
                i += 1

        # Use smoothed mask to identify segments
        is_gap = [(distances[i][0], smoothed_mask[i]) for i in range(len(distances))]
        
        # Find longest gap segment
        best_start = None
        best_end = None
        best_width = 0
        
        current_start = None
        for i, (angle, is_far) in enumerate(is_gap):
            if is_far:
                if current_start is None:
                    current_start = angle
            else:
                if current_start is not None:
                    width = angle - current_start
                    if width > best_width:
                        best_width = width
                        best_start = current_start
                        best_end = angle
                    current_start = None
        
        # Check if gap extends to end
        if current_start is not None:
            width = is_gap[-1][0] - current_start
            if width > best_width:
                best_width = width
                best_start = current_start
                best_end = is_gap[-1][0]
        
        if best_width < min_gap_width_deg:
            return {'found': False, 'reason': 'gap_too_narrow', 'width_deg': best_width}
        
        center_angle = (best_start + best_end) / 2
        
        # Extract raw profile for visualization
        # Unzip the (angle, dist) tuples
        raw_angles = [d[0] for d in distances]
        raw_dists = [d[1] for d in distances]

        return {
            'found': True,
            'center_angle': center_angle,
            'width_deg': best_width,
            'left_edge_angle': best_start,
            'right_edge_angle': best_end,
            'threshold': threshold,
            'raw_profile': (raw_angles, raw_dists)
        }


# Singleton instance (lazy initialized)
_lidar_instance: Optional[Lidar360] = None


def get_lidar360() -> Optional[Lidar360]:
    """Get or create the 360° lidar instance from config."""
    global _lidar_instance
    
    if _lidar_instance is not None:
        return _lidar_instance
    
    from core.config_manager import get_config
    
    port = get_config("LIDAR_PORT", "")
    baud_rate = get_config("LIDAR_BAUD_RATE", 230400)
    
    if not port:
        logger.debug("Lidar360: No port configured, skipping initialization")
        return None
    
    _lidar_instance = Lidar360(
        port=port,
        baud_rate=int(baud_rate)
    )
    
    return _lidar_instance


def start_lidar() -> bool:
    """Start the 360° LIDAR (call when AI navigation starts)."""
    from state import state
    
    lidar = get_lidar360()
    if lidar is None:
        return False
    
    lidar.start()
    state.lidar360 = lidar
    
    # Start update thread for state sync
    def update_state():
        while lidar._running:
            state.lidar_distance = lidar.get_forward_distance()
            state.lidar_scan = lidar.get_full_scan()
            time.sleep(0.05)
    
    threading.Thread(target=update_state, daemon=True).start()
    return True


def stop_lidar():
    """Stop the 360° LIDAR (call when AI navigation stops)."""
    from state import state
    
    if state.lidar360:
        state.lidar360.stop()
        state.lidar_distance = None
        state.lidar_scan = None
