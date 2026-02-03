"""Serial client for the VW frequency controller used by the legacy VB app."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Sequence, Tuple

import serial
from serial import Serial
from serial.serialutil import SerialException
from serial.tools import list_ports

FRAME_HEADER = b"\xFC\xFD"
FRAME_PAYLOAD_SIZE = 16  # hardware reports 16 data bytes after the header
HEADER_SIZE = len(FRAME_HEADER)
TICK_SECONDS = 1.25e-7  # 125 ns per hardware tick
DEFAULT_CALIBRATION = 2
DEFAULT_TIMER_MP = 0.008192  # seconds
DEFAULT_BAUDRATE = 38400


class ControllerError(RuntimeError):
    """Raised when the controller cannot be reached or returns bad data."""


@dataclass
class ChannelMeasurement:
    """Decoded information for a single measurement channel."""

    count: int
    wait_cycles: int
    gate_bytes: Tuple[int, int, int, int]
    frequency_hz: float


@dataclass
class FrequencyFrame:
    """Represents a pair of channel measurements captured at the same instant."""

    channel1: ChannelMeasurement
    channel2: ChannelMeasurement
    raw_payload: bytes




BytesLogger = Callable[[str, bytes], None]


class VWFrequencyController:
    """Python port of the serial protocol used by the VB controller."""

    def __init__(
        self,
        baudrate: int = DEFAULT_BAUDRATE,
        read_timeout: float = 1.0,
        calibration: int = DEFAULT_CALIBRATION,
        timer_mp: float = DEFAULT_TIMER_MP,
    ) -> None:
        self.baudrate = baudrate
        self.read_timeout = read_timeout
        self.calibration = calibration
        self.timer_mp = timer_mp
        self._serial: Optional[Serial] = None
        self.mode: int = 3
        self.k_value: int = 31
        self._raw_k_value: int = 31
        self._extra_delay: float = 0.0
        # Optional callback called when bytes are sent or received.
        # Signature: (direction: str, data: bytes) where direction is '>' for
        # sent bytes and '<' for received bytes.
        self._bytes_logger: Optional[BytesLogger] = None

    # ------------------------------------------------------------------
    def _log_bytes(self, direction: str, data: bytes) -> None:
        """Call the optional logger without affecting controller flow."""

        if not self._bytes_logger or not data:
            return
        try:
            self._bytes_logger(direction, data)
        except Exception:
            # Logging should never abort the main control path.
            pass

    def _read_available(self) -> bytes:
        """Return currently buffered inbound bytes without blocking."""

        assert self._serial is not None
        waiting = getattr(self._serial, "in_waiting", None)
        if waiting is not None and waiting > 0:
            return self._serial.read(waiting)
        try:
            return self._serial.read_all()
        except AttributeError:
            return b""

    def _reset_like_vb(self) -> None:
        """Toggle DTR lines the same way the legacy VB app does."""

        assert self._serial is not None
        self._serial.dtr = True
        time.sleep(0.25)
        self._serial.dtr = False

    def _drain_echo(self, label: str = "") -> None:
        """Drop small status bursts sent by the hardware after control packets."""

        assert self._serial is not None
        drained = bytearray()
        for _ in range(4):
            chunk = self._read_available()
            if not chunk:
                break
            drained.extend(chunk)
            time.sleep(0.02)
        if drained:
            self._log_bytes('<', bytes(drained))
        self._serial.reset_input_buffer()

    # ---------------------------------------------------------------------
    @staticmethod
    def available_ports(max_index: int = 30) -> List[str]:
        """Return a list of candidate COM ports."""

        detected = [port.device for port in list_ports.comports()]
        if detected:
            return detected
        return [f"COM{i}" for i in range(1, max_index + 1)]

    def auto_connect(self, max_index: int = 30) -> str:
        """Try to connect to the first reachable port."""

        last_error: Optional[Exception] = None
        for port in self.available_ports(max_index):
            try:
                self.connect(port)
            except (OSError, SerialException) as exc:
                last_error = exc
                continue
            return port
        if last_error is None:
            raise ControllerError("No serial ports detected.")
        raise ControllerError(f"Could not open controller on any port: {last_error}")

    def connect(self, port: str) -> None:
        """Open the serial port and prepare the buffers."""

        self.close()
        try:
            self._serial = Serial(
                port=port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.read_timeout,
                write_timeout=self.read_timeout,
            )
        except SerialException as exc:
            raise ControllerError(f"Failed to open {port}: {exc}") from exc

        self._serial.reset_output_buffer()
        self._serial.reset_input_buffer()
        self._serial.rts = False
        self._reset_like_vb()

    def set_bytes_logger(self, logger: BytesLogger | None) -> None:
        """Set a callback to receive sent/received byte notifications.

        The callback will be called with a direction string ('>' or '<')
        and the bytes object that was sent or received.
        """
        self._bytes_logger = logger

    def close(self) -> None:
        """Close the serial connection if it is open."""

        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None

    # ------------------------------------------------------------------
    def configure_measurement(self, mode: int = 3, k_value: int = 31) -> None:
        """Match the VB app handshake by sending mode and integration counter."""

        if not self._serial or not self._serial.is_open:
            raise ControllerError("Serial port is not open.")

        self.mode = mode & 0xFF
        raw_k = max(1, int(k_value))
        effective_k = min(255, raw_k)
        self._raw_k_value = raw_k
        self.k_value = effective_k
        self._extra_delay = (raw_k - effective_k) * self.timer_mp

        payload = bytes((self.mode, self.k_value & 0xFF))
        self._serial.reset_output_buffer()
        self._serial.reset_input_buffer()
        self._serial.write(payload)
        self._log_bytes('>', payload)
        self._serial.flush()
        time.sleep(0.7)  # VB introduces ~700 ms pause after this packet
        self._drain_echo("handshake")

    def trigger_measurement(self) -> None:
        """Send the single-byte trigger that starts a new measurement window."""

        if not self._serial or not self._serial.is_open:
            raise ControllerError("Serial port is not open.")
        data = bytes((self.k_value,))
        self._serial.write(data)
        self._log_bytes('>', data)
        try:
            self._serial.flush()
        except Exception:
            raise ControllerError("Failed to flush serial output buffer.")

    # ------------------------------------------------------------------
    def iter_frames(
        self,
        count: Optional[int] = None,
        timeout: float = 3.0,
        auto_trigger: bool = True,
        interval: float = 0.0,
    ) -> Iterable[FrequencyFrame]:
        """Yield decoded frames until count is reached (or forever when None)."""

        produced = 0
        first_cycle = True
        while count is None or produced < count:
            if auto_trigger:
                if not first_cycle:
                    gap = self.measurement_gap_seconds()
                    if gap > 0:
                        time.sleep(gap)
                self.trigger_measurement()
            frame = self.read_frame(timeout)
            produced += 1
            yield frame
            first_cycle = False
            if interval > 0:
                time.sleep(interval)

    def read_frame(self, timeout: float = 3.0) -> FrequencyFrame:
        """Read a single frame and decode both channels."""

        if not self._serial or not self._serial.is_open:
            raise ControllerError("Serial port is not open.")

        deadline = time.monotonic() + timeout
        buffer = bytearray()
        state = 0
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ControllerError("Timed out waiting for frame header.")
            chunk = self._serial.read(1)
            if not chunk:
                continue

            self._log_bytes('<', chunk)

            byte = chunk[0]
            if state == 0:
                buffer.clear()
                if byte == FRAME_HEADER[0]:
                    state = 1
                continue
            if state == 1:
                if byte == FRAME_HEADER[1]:
                    while len(buffer) < FRAME_PAYLOAD_SIZE:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise ControllerError("Timed out while reading frame payload.")
                        chunk_payload = self._serial.read(FRAME_PAYLOAD_SIZE - len(buffer))
                        if not chunk_payload:
                            continue
                        self._log_bytes('<', chunk_payload)
                        buffer.extend(chunk_payload)
                    return self._decode_frame(bytes(buffer))
                state = 1 if byte == FRAME_HEADER[0] else 0
                if state == 0:
                    buffer.clear()

    # ------------------------------------------------------------------
    def _decode_frame(self, payload: bytes) -> FrequencyFrame:
        """Convert the 16-byte payload into a pair of channel measurements."""

        if len(payload) != FRAME_PAYLOAD_SIZE:
            raise ControllerError(f"Unexpected frame size: {len(payload)}")

        ch1_count = (payload[0] << 8) | payload[1]
        ch1_wait = (payload[2] << 8) | payload[3]
        ch1_gate = tuple(payload[4:8])

        ch2_count = (payload[8] << 8) | payload[9]
        ch2_wait = (payload[10] << 8) | payload[11]
        ch2_gate = tuple(payload[12:16])

        ch1_freq = self._compute_frequency(ch1_count, ch1_wait)
        ch2_freq = self._compute_frequency(ch2_count, ch2_wait)

        channel1 = ChannelMeasurement(ch1_count, ch1_wait, ch1_gate, ch1_freq)
        channel2 = ChannelMeasurement(ch2_count, ch2_wait, ch2_gate, ch2_freq)
        return FrequencyFrame(channel1, channel2, payload)

    def _compute_frequency(self, count: int, wait_cycles: int) -> float:
        """Replicate the VB timing formula to derive frequency in Hz."""

        measurement_window = self._measurement_window()
        extra_delay = (wait_cycles + self.calibration) * TICK_SECONDS
        denominator = measurement_window + extra_delay
        if denominator <= 0:
            raise ControllerError("Invalid timing data (non-positive denominator).")
        return count / denominator

    def _measurement_window(self) -> float:
        """Mirror the VB computation of the time window for one capture."""

        k = max(1, self.k_value)
        if self.mode == 3:
            if k % 2 == 1:
                return ((k - 1) // 2) * (self.timer_mp * 2.0) + 0.008193125
            return (k // 2) * (self.timer_mp * 2.0) + 0.000001
        return (k - 1) * self.timer_mp + 0.0081939

    def measurement_window_seconds(self) -> float:
        """Public helper exposing the current measurement window duration."""

        return self._measurement_window()

    def measurement_gap_seconds(self) -> float:
        """Total delay enforced between automatic triggers (window + guard time)."""

        extra = 0.02 if self.mode == 3 else 0.01
        return self._measurement_window() + self._extra_delay + extra

    def extra_delay_seconds(self) -> float:
        """Additional wait introduced when the requested k exceeds 255."""

        return self._extra_delay

    @property
    def requested_k_value(self) -> int:
        """Return the integration counter requested by the caller (before clamping)."""

        return self._raw_k_value

    # ------------------------------------------------------------------
    def __enter__(self) -> "VWFrequencyController":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def format_frame(frame: FrequencyFrame) -> str:
    """Pretty-print a frame similar to the VB diagnostics textbox."""

    ch1 = frame.channel1
    ch2 = frame.channel2
    return (
        "Frame("  # Keep output terse for console use
        f"ch1: count={ch1.count}, wait={ch1.wait_cycles}, freq={ch1.frequency_hz:.4f} Hz; "
        f"ch2: count={ch2.count}, wait={ch2.wait_cycles}, freq={ch2.frequency_hz:.4f} Hz)"
    )
