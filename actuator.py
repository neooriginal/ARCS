"""
ARCS - Linear Actuator Module
Controls a linear actuator via a Raspberry Pi Pico USB bridge.

Protocol (same as pico_motor.py firmware):
  forward <speed>   — extend at speed 0-100
  backward <speed>  — retract at speed 0-100
  stop              — stop
  stream <ms>       — start telemetry stream (JSON lines)
  stream off        — stop telemetry stream
"""

import threading
import time
import json
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class LinearActuator:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._serial = None
        self._lock = threading.Lock()
        self._telemetry: dict = {}
        self._stream_thread: Optional[threading.Thread] = None
        self.connected = False

    def connect(self) -> bool:
        try:
            import serial
            self._serial = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(0.5)
            self._serial.reset_input_buffer()
            self.connected = True

            self._stream_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._stream_thread.start()

            self._send("stream 100")
            logger.info(f"[Actuator] Connected on {self.port}")
            return True
        except Exception as e:
            logger.warning(f"[Actuator] Connect failed: {e}")
            self.connected = False
            return False

    def disconnect(self) -> None:
        self.connected = False
        if self._serial and self._serial.is_open:
            try:
                self._send("stop")
                self._send("stream off")
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def extend(self, speed: int = 100) -> bool:
        return self._send(f"forward {max(0, min(100, int(speed)))}")

    def retract(self, speed: int = 100) -> bool:
        return self._send(f"backward {max(0, min(100, int(speed)))}")

    def stop(self) -> bool:
        return self._send("stop")

    def get_telemetry(self) -> dict:
        with self._lock:
            return self._telemetry.copy()

    def _send(self, cmd: str) -> bool:
        if not self._serial or not self._serial.is_open:
            return False
        try:
            with self._lock:
                self._serial.write(f"{cmd}\n".encode())
            return True
        except Exception as e:
            logger.warning(f"[Actuator] Send error: {e}")
            self.connected = False
            return False

    def _read_loop(self) -> None:
        while self.connected and self._serial and self._serial.is_open:
            try:
                line = self._serial.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                if line.startswith('{'):
                    try:
                        data = json.loads(line)
                        with self._lock:
                            self._telemetry = data
                    except json.JSONDecodeError:
                        pass
            except Exception as e:
                logger.warning(f"[Actuator] Read error: {e}")
                time.sleep(0.1)


# Global singleton — initialized in main.py if ACTUATOR_USB is set
actuator: Optional[LinearActuator] = None


def init_actuator() -> bool:
    global actuator
    from core.config_manager import get_config
    from state import state

    port = get_config("ACTUATOR_USB")
    if not port:
        logger.info("[Actuator] No USB port configured, skipping")
        return False

    actuator = LinearActuator(port)
    ok = actuator.connect()
    state.actuator = actuator
    state.actuator_connected = ok
    return ok
