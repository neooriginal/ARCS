"""
TF-Luna Lidar Sensor Driver
Single-point distance measurement (0.2-8m range).
Supports UART and I2C communication protocols.
"""
import logging
import threading
import time
from typing import Optional

logger = logging.getLogger(__name__)


class TFLunaLidar:
    """TF-Luna single-point lidar sensor driver."""
    
    # TF-Luna frame format (UART): 0x59 0x59 DIST_L DIST_H STRENGTH_L STRENGTH_H TEMP_L TEMP_H CHECKSUM
    HEADER = 0x59
    FRAME_SIZE = 9
    
    def __init__(
        self,
        port: str = "",
        protocol: str = "uart",
        baud_rate: int = 115200,
        i2c_address: int = 0x10
    ):
        self.port = port
        self.protocol = protocol
        self.baud_rate = baud_rate
        self.i2c_address = i2c_address
        
        self._serial = None
        self._i2c = None
        self._connected = False
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        
        # Latest readings
        self.distance_cm: Optional[int] = None
        self.strength: Optional[int] = None
        self.temperature: Optional[float] = None
    
    @property
    def connected(self) -> bool:
        return self._connected
    
    def connect(self) -> bool:
        """Attempt to connect to the lidar sensor."""
        if not self.port and self.protocol == "uart":
            logger.warning("Lidar: No port configured")
            return False
        
        try:
            if self.protocol == "uart":
                return self._connect_uart()
            elif self.protocol == "i2c":
                return self._connect_i2c()
            else:
                logger.error(f"Lidar: Unknown protocol {self.protocol}")
                return False
        except Exception as e:
            logger.error(f"Lidar: Connection failed - {e}")
            return False
    
    def _connect_uart(self) -> bool:
        """Connect via UART/Serial."""
        try:
            import serial
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            self._connected = True
            logger.info(f"Lidar: Connected via UART on {self.port} @ {self.baud_rate}")
            return True
        except Exception as e:
            logger.error(f"Lidar: UART connection failed - {e}")
            self._connected = False
            return False
    
    def _connect_i2c(self) -> bool:
        """Connect via I2C (Linux/RPi only)."""
        try:
            import smbus2
            self._i2c = smbus2.SMBus(1)  # I2C bus 1 (standard on RPi)
            self._connected = True
            logger.info(f"Lidar: Connected via I2C at address 0x{self.i2c_address:02X}")
            return True
        except ImportError:
            logger.error("Lidar: smbus2 not installed (pip install smbus2)")
            return False
        except Exception as e:
            logger.error(f"Lidar: I2C connection failed - {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the sensor."""
        self.stop_reading()
        
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        
        if self._i2c:
            try:
                self._i2c.close()
            except Exception:
                pass
            self._i2c = None
        
        self._connected = False
        logger.info("Lidar: Disconnected")
    
    def read_once(self) -> Optional[int]:
        """Read a single distance measurement. Returns distance in cm or None."""
        if not self._connected:
            return None
        
        try:
            if self.protocol == "uart":
                return self._read_uart()
            elif self.protocol == "i2c":
                return self._read_i2c()
        except Exception as e:
            logger.debug(f"Lidar: Read error - {e}")
            return None
    
    def _read_uart(self) -> Optional[int]:
        """Read distance from UART."""
        if not self._serial:
            return None
        
        # Clear buffer and find frame start
        self._serial.reset_input_buffer()
        
        # Wait for frame header (0x59 0x59)
        header_count = 0
        timeout = time.time() + 0.5
        
        while time.time() < timeout:
            byte = self._serial.read(1)
            if not byte:
                continue
            if byte[0] == self.HEADER:
                header_count += 1
                if header_count >= 2:
                    break
            else:
                header_count = 0
        
        if header_count < 2:
            return None
        
        # Read remaining frame data
        data = self._serial.read(self.FRAME_SIZE - 2)
        if len(data) != self.FRAME_SIZE - 2:
            return None
        
        # Parse frame
        dist_l, dist_h, str_l, str_h, temp_l, temp_h, checksum = data
        
        # Verify checksum
        calc_checksum = (self.HEADER + self.HEADER + sum(data[:-1])) & 0xFF
        if calc_checksum != checksum:
            return None
        
        # Extract values
        with self._lock:
            self.distance_cm = dist_l + (dist_h << 8)
            self.strength = str_l + (str_h << 8)
            self.temperature = (temp_l + (temp_h << 8)) / 8.0 - 256
        
        return self.distance_cm
    
    def _read_i2c(self) -> Optional[int]:
        """Read distance from I2C."""
        if not self._i2c:
            return None
        
        try:
            # TF-Luna I2C: Read 2 bytes from register 0x00 for distance
            data = self._i2c.read_i2c_block_data(self.i2c_address, 0x00, 2)
            with self._lock:
                self.distance_cm = data[0] + (data[1] << 8)
            return self.distance_cm
        except Exception:
            return None
    
    def start_reading(self, callback=None, interval: float = 0.05):
        """Start continuous reading in background thread."""
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(
            target=self._read_loop,
            args=(callback, interval),
            daemon=True
        )
        self._thread.start()
        logger.info("Lidar: Started continuous reading")
    
    def stop_reading(self):
        """Stop continuous reading."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
    
    def _read_loop(self, callback, interval: float):
        """Background reading loop."""
        while self._running:
            distance = self.read_once()
            if distance is not None and callback:
                try:
                    callback(distance)
                except Exception as e:
                    logger.debug(f"Lidar: Callback error - {e}")
            time.sleep(interval)
    
    def get_distance(self) -> Optional[int]:
        """Get the last read distance in cm."""
        with self._lock:
            return self.distance_cm


# Singleton instance (lazy initialized)
_lidar_instance: Optional[TFLunaLidar] = None


def get_lidar() -> Optional[TFLunaLidar]:
    """Get or create the lidar instance from config."""
    global _lidar_instance
    
    if _lidar_instance is not None:
        return _lidar_instance
    
    from core.config_manager import get_config
    
    port = get_config("LIDAR_PORT", "")
    protocol = get_config("LIDAR_PROTOCOL", "uart")
    baud_rate = get_config("LIDAR_BAUD_RATE", 115200)
    i2c_address = get_config("LIDAR_I2C_ADDRESS", 16)
    
    # Only create if port is configured
    if not port and protocol == "uart":
        logger.debug("Lidar: No port configured, skipping initialization")
        return None
    
    _lidar_instance = TFLunaLidar(
        port=port,
        protocol=protocol,
        baud_rate=int(baud_rate),
        i2c_address=int(i2c_address)
    )
    
    if _lidar_instance.connect():
        logger.info("Lidar: Initialized successfully")
    else:
        logger.warning("Lidar: Failed to connect, will retry on demand")
    
    return _lidar_instance


def init_lidar():
    """Initialize lidar from config (called at startup)."""
    lidar = get_lidar()
    if lidar and lidar.connected:
        from state import state
        state.lidar = lidar
        state.lidar_distance = lidar.get_distance()
        return True
    return False
