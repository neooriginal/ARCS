"""
ARCS - JBD BMS (Jiabaida DH04SA01) Bluetooth LE Client

JBD BLE protocol over UART-bridge:
  Service:   0000ff00-0000-1000-8000-00805f9b34fb
  Write:     0000fff0-0000-1000-8000-00805f9b34fb
  Notify:    0000fff1-0000-1000-8000-00805f9b34fb

Basic info request:  DD A5 03 00 FF FD 77  (reads SOC, voltage, current, temp, MOSFET)
Cell voltage request: DD A5 04 00 FF FC 77
"""

import asyncio
import logging
import struct
import threading
import time
from typing import Optional

logger = logging.getLogger(__name__)

SERVICE_UUID  = "0000ff00-0000-1000-8000-00805f9b34fb"
WRITE_UUID    = "0000fff0-0000-1000-8000-00805f9b34fb"
NOTIFY_UUID   = "0000fff1-0000-1000-8000-00805f9b34fb"

CMD_BASIC_INFO  = bytes([0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77])
CMD_CELL_VOLT   = bytes([0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77])

# MOSFET state byte: bit0 = charge MOSFET on, bit1 = discharge MOSFET on
MOSFET_BOTH_ON    = 0x03
MOSFET_CHARGE_ON  = 0x01
MOSFET_BOTH_OFF   = 0x00


def _jbd_checksum(data: bytes) -> int:
    chk = 0
    for b in data:
        chk += b
    return (~chk + 1) & 0xFFFF


def _build_write_cmd(cmd: int, data: bytes) -> bytes:
    payload = bytes([cmd, len(data)]) + data
    chk = _jbd_checksum(payload)
    return bytes([0xDD, 0x5A]) + payload + bytes([(chk >> 8) & 0xFF, chk & 0xFF, 0x77])


class BMSClient:
    def __init__(self, address: str, poll_interval: int = 30):
        self.address = address
        self.poll_interval = poll_interval
        self._client = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._buf = bytearray()
        self._running = False

    # --- Public control methods (thread-safe, called from Flask) ---

    def set_charging(self, enabled: bool) -> bool:
        """Enable or disable the charge MOSFET. Discharge is left on."""
        if not self._loop or not self._running:
            return False
        state = MOSFET_BOTH_ON if enabled else MOSFET_CHARGE_ON ^ MOSFET_CHARGE_ON  # charge off = 0x02 dis only
        # State: 0x01=charge on, 0x02=discharge on, 0x03=both on, 0x00=both off
        new_state = MOSFET_BOTH_ON if enabled else 0x02  # keep discharge on, toggle charge
        cmd = _build_write_cmd(0xE1, bytes([new_state]))
        future = asyncio.run_coroutine_threadsafe(self._write(cmd), self._loop)
        try:
            return future.result(timeout=3)
        except Exception as e:
            logger.warning(f"[BMS] set_charging error: {e}")
            return False

    def disconnect(self) -> None:
        self._running = False
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop)

    # --- Internal async implementation ---

    async def _write(self, data: bytes) -> bool:
        if not self._client or not self._client.is_connected:
            return False
        try:
            await self._client.write_gatt_char(WRITE_UUID, data, response=False)
            return True
        except Exception as e:
            logger.warning(f"[BMS] Write error: {e}")
            return False

    async def _disconnect(self) -> None:
        if self._client and self._client.is_connected:
            try:
                await self._client.disconnect()
            except Exception:
                pass

    def _on_notify(self, _sender, data: bytes) -> None:
        self._buf.extend(data)
        self._process_buffer()

    def _process_buffer(self) -> None:
        from state import state

        while len(self._buf) >= 4:
            if self._buf[0] != 0xDD:
                self._buf.pop(0)
                continue

            if self._buf[1] == 0x03 and len(self._buf) >= 4:
                length = self._buf[3]
                total = length + 7  # header(2) + status(1) + len(1) + data + chk(2) + end(1)
                if len(self._buf) < total:
                    break
                packet = bytes(self._buf[:total])
                self._buf = self._buf[total:]
                self._parse_basic(packet, state)

            elif self._buf[1] == 0x04 and len(self._buf) >= 4:
                length = self._buf[3]
                total = length + 7
                if len(self._buf) < total:
                    break
                packet = bytes(self._buf[:total])
                self._buf = self._buf[total:]
                self._parse_cells(packet, state)
            else:
                self._buf.pop(0)

    def _parse_basic(self, pkt: bytes, state) -> None:
        try:
            # pkt[0]=DD, pkt[1]=03, pkt[2]=status(00=OK), pkt[3]=length, pkt[4..]=data
            if pkt[2] != 0x00:
                state.bms_error = f"BMS returned error status 0x{pkt[2]:02X}"
                return

            d = pkt[4:]
            total_voltage = struct.unpack_from('>H', d, 0)[0] / 100.0  # V
            current_raw   = struct.unpack_from('>h', d, 2)[0]          # signed
            current        = current_raw / 100.0                        # A
            remain_cap     = struct.unpack_from('>H', d, 4)[0] / 100.0 # Ah
            typ_cap        = struct.unpack_from('>H', d, 6)[0] / 100.0 # Ah
            cycles         = struct.unpack_from('>H', d, 8)[0]
            # protection status at offset 14 (2 bytes), skip production date (2)
            protect_status = struct.unpack_from('>H', d, 14)[0]
            soc            = d[19]                                       # 0-100%
            mosfet_status  = d[20]                                       # bit0=charge, bit1=discharge
            num_cells      = d[21]
            num_temps      = d[22]

            temps = []
            for i in range(num_temps):
                raw_t = struct.unpack_from('>H', d, 23 + i * 2)[0]
                temps.append(round((raw_t - 2731) / 10.0, 1))

            state.bms_soc        = soc
            state.bms_voltage    = round(total_voltage, 2)
            state.bms_current    = round(current, 2)
            state.bms_temp       = temps[0] if temps else None
            state.bms_temps      = temps
            state.bms_cycles     = cycles
            state.bms_remain_cap = round(remain_cap, 2)
            state.bms_charging   = bool(mosfet_status & 0x01)
            state.bms_discharging = bool(mosfet_status & 0x02)
            state.bms_connected  = True
            state.bms_error      = None
            state.bms_last_update = time.time()

            logger.debug(f"[BMS] SOC={soc}% V={total_voltage:.2f}V I={current:.2f}A T={temps}")

        except Exception as e:
            state.bms_error = f"Parse error: {e}"
            logger.warning(f"[BMS] Parse basic failed: {e}")

    def _parse_cells(self, pkt: bytes, state) -> None:
        try:
            if pkt[2] != 0x00:
                return
            d = pkt[4:]
            num = len(d) // 2
            cells = [struct.unpack_from('>H', d, i * 2)[0] / 1000.0 for i in range(num)]
            state.bms_cells = cells
        except Exception as e:
            logger.warning(f"[BMS] Parse cells failed: {e}")

    async def _run(self) -> None:
        from state import state
        try:
            from bleak import BleakClient, BleakError
        except ImportError:
            logger.warning("[BMS] 'bleak' not installed — BMS disabled. Run: pip install bleak")
            state.bms_error = "bleak not installed"
            return

        while self._running:
            try:
                logger.info(f"[BMS] Connecting to {self.address}...")
                async with BleakClient(self.address, timeout=10.0) as client:
                    self._client = client
                    state.bms_connected = True
                    state.bms_error = None
                    logger.info("[BMS] Connected")

                    await client.start_notify(NOTIFY_UUID, self._on_notify)

                    while self._running and client.is_connected:
                        await self._write(CMD_BASIC_INFO)
                        await asyncio.sleep(0.5)
                        await self._write(CMD_CELL_VOLT)
                        await asyncio.sleep(self.poll_interval)

                    await client.stop_notify(NOTIFY_UUID)

            except Exception as e:
                logger.warning(f"[BMS] Connection error: {e}")
                state.bms_connected = False
                state.bms_error = str(e)
                self._client = None

                if self._running:
                    logger.info("[BMS] Retrying in 15s...")
                    await asyncio.sleep(15)

        state.bms_connected = False
        logger.info("[BMS] Loop stopped")

    def start(self) -> None:
        self._running = True
        self._loop = asyncio.new_event_loop()

        def _thread_main():
            asyncio.set_event_loop(self._loop)
            self._loop.run_until_complete(self._run())
            self._loop.close()

        self._thread = threading.Thread(target=_thread_main, daemon=True, name="bms-ble")
        self._thread.start()


# Global singleton
bms_client: Optional[BMSClient] = None


def init_bms() -> bool:
    global bms_client
    from core.config_manager import get_config
    from state import state

    try:
        from bleak import BleakClient  # noqa: F401 — early check
    except ImportError:
        logger.warning("[BMS] 'bleak' not installed — skipping. Install with: pip install bleak")
        state.bms_error = "bleak not installed"
        return False

    address = get_config("BMS_BT_ADDRESS", "")
    if not address:
        logger.info("[BMS] No BT address configured, skipping")
        return False

    poll = int(get_config("BMS_POLL_INTERVAL", 30))
    bms_client = BMSClient(address, poll_interval=poll)
    bms_client.start()
    state.bms_client = bms_client
    return True


async def scan_bms_devices(timeout: float = 5.0) -> list[dict]:
    """Scan for nearby BLE devices — used by the settings scan button.

    Returns devices ordered by signal strength. Devices that advertise the JBD
    BMS service UUID are flagged with matched=True so the UI can highlight them.
    If no JBD devices are found, all visible devices are returned so the user
    can still select manually.
    """
    try:
        from bleak import BleakScanner

        devices = await BleakScanner.discover(timeout=timeout, return_adv=True)

        results = []
        for device, adv in devices.values():
            service_uuids = [str(u).lower() for u in (adv.service_uuids or [])]
            matched = SERVICE_UUID in service_uuids
            results.append({
                "address": device.address,
                "name": device.name or "Unknown",
                "rssi": adv.rssi,
                "matched": matched,
            })

        # Sort: matched devices first, then by signal strength
        results.sort(key=lambda x: (not x["matched"], -(x["rssi"] or -999)))

        # If any matched, only return matched devices
        matched_only = [r for r in results if r["matched"]]
        return matched_only if matched_only else results

    except Exception as e:
        logger.warning(f"[BMS] Scan failed: {e}")
        return []
