"""Low-level drivers for GRBL-compatible motors."""

from __future__ import annotations

import logging
import queue
import threading
import time
from itertools import count
from typing import Callable, Optional, Protocol

from .motor_types import MotorError

try:  # pragma: no cover - optional dependency
    import serial  # type: ignore[import]
    import serial.tools.list_ports  # type: ignore[import]
except Exception:  # pragma: no cover - simulator fallback
    serial = None  # type: ignore[assignment]

SERIAL_AVAILABLE = serial is not None
LineListener = Callable[[str], None]


def list_serial_ports() -> list[str]:
    """Return available serial port names (empty when pyserial is unavailable)."""

    if serial is None:
        return []
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]


class MotorDriver(Protocol):
    """Interface used by the high-level controller."""

    is_simulation: bool

    def connect(self, port: str, *, baudrate: int = 115200, timeout: float = 0.05) -> None: ...
    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
    def send_line(self, command: str, *, wait_for_ok: bool = True, timeout: float = 2.0) -> None: ...
    def send_raw(self, payload: bytes) -> None: ...
    def reset_input_buffer(self) -> None: ...
    def jog_increment(self, axis: str, distance: float, feed: float) -> None: ...
    def register_line_listener(self, callback: LineListener) -> Callable[[], None]: ...
    @property
    def port(self) -> Optional[str]: ...


class SimulatedMotorDriver:
    """Driver that mirrors the hardware API without serial I/O."""

    is_simulation: bool = True

    def __init__(self, logger: Optional[logging.Logger] = None) -> None:
        self._logger = logger or logging.getLogger("motor.driver.sim")
        self._connected = False
        self._port: Optional[str] = None

    @property
    def port(self) -> Optional[str]:
        return self._port

    def connect(self, port: str, *, baudrate: int = 115200, timeout: float = 0.05) -> None:  # noqa: ARG002 - parity with protocol
        self._connected = True
        self._port = port or "Simulated Motor"
        self._logger.info("Simulated motor connected (port=%s)", self._port)

    def disconnect(self) -> None:
        self._connected = False
        self._port = None
        self._logger.info("Simulated motor disconnected")

    def is_connected(self) -> bool:
        return self._connected

    def send_line(self, command: str, *, wait_for_ok: bool = True, timeout: float = 2.0) -> None:  # noqa: ARG002 - parity with protocol
        if not command:
            return
        self._logger.debug("[sim] -> %s", command)

    def send_raw(self, payload: bytes) -> None:
        if not payload:
            return
        self._logger.debug("[sim] -> raw %s", payload)

    def reset_input_buffer(self) -> None:  # pragma: no cover - no-op
        return

    def jog_increment(self, axis: str, distance: float, feed: float) -> None:  # noqa: ARG002 - parity with protocol
        self._logger.debug("[sim] jog %s %+0.3f @ %s", axis, distance, feed)

    def register_line_listener(self, callback: LineListener) -> Callable[[], None]:  # pragma: no cover - no listeners in sim
        def _unregister() -> None:
            return None

        return _unregister


class SerialMotorDriver:
    """GRBL serial driver handling connection, reads, and acknowledgements."""

    is_simulation: bool = False

    def __init__(self, logger: Optional[logging.Logger] = None) -> None:
        self._logger = logger or logging.getLogger("motor.driver.serial")
        self._serial_handle: Optional["serial.Serial"] = None  # type: ignore[name-defined]
        self._reader_thread: Optional[threading.Thread] = None
        self._reader_stop = threading.Event()
        self._response_queue: "queue.Queue[str]" = queue.Queue(maxsize=64)
        self._listener_counter = count(1)
        self._line_listeners: dict[int, LineListener] = {}
        self._listener_lock = threading.Lock()
        self._connected = False
        self._port: Optional[str] = None

    @property
    def port(self) -> Optional[str]:
        return self._port

    def connect(self, port: str, *, baudrate: int = 115200, timeout: float = 0.05) -> None:
        if serial is None:
            raise MotorError("pyserial is not installed; cannot connect to hardware.")
        if not port:
            raise MotorError("Port must be provided when using hardware mode.")

        if self._connected and port == self._port:
            self._logger.debug("Already connected to %s; skipping connect", port)
            return

        if self._connected:
            self.disconnect()

        try:
            handle = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, write_timeout=0.5)  # type: ignore[misc]
        except Exception as exc:  # pragma: no cover - hardware access
            raise MotorError(f"Could not open port {port}: {exc}") from exc

        self._serial_handle = handle
        self._connected = True
        self._port = port

        self._initialise_controller()
        self._start_reader()
        self._logger.info("Motor connected to %s (baud=%s)", port, baudrate)

    def disconnect(self) -> None:
        self._stop_reader()
        handle = self._serial_handle
        self._serial_handle = None
        self._connected = False
        self._port = None
        if handle is not None:
            try:  # pragma: no cover - hardware access
                handle.close()
            except Exception:
                self._logger.exception("Error while closing serial handle")
        self._logger.info("Motor disconnected")

    def is_connected(self) -> bool:
        return self._connected

    def send_line(self, command: str, *, wait_for_ok: bool = True, timeout: float = 2.0) -> None:
        if not command:
            return
        if not self._connected or self._serial_handle is None:
            raise MotorError("Motor is not connected.")
        self._write_line(command, wait_for_ok=wait_for_ok, timeout=timeout)

    def send_raw(self, payload: bytes) -> None:  # pragma: no cover - hardware helper
        if not payload:
            return
        if not self._connected or self._serial_handle is None:
            raise MotorError("Motor is not connected.")
        self._send_realtime(payload)

    def reset_input_buffer(self) -> None:
        handle = self._serial_handle
        if handle is None:
            return
        try:
            handle.reset_input_buffer()
        except Exception:
            self._logger.debug("Failed to reset input buffer", exc_info=True)

    def jog_increment(self, axis: str, distance: float, feed: float) -> None:
        cmd = f"$J=G91 {axis}{distance:+.3f} F{feed:.1f}"
        self.send_line(cmd)

    def register_line_listener(self, callback: LineListener) -> Callable[[], None]:
        if callback is None:
            raise ValueError("callback must not be None")
        token = next(self._listener_counter)
        with self._listener_lock:
            self._line_listeners[token] = callback

        def _unregister() -> None:
            with self._listener_lock:
                self._line_listeners.pop(token, None)

        return _unregister

    # -------------------- Internal helpers --------------------
    def _start_reader(self) -> None:
        if self._reader_thread and self._reader_thread.is_alive():
            return
        if self._serial_handle is None:
            return
        self._reader_stop.clear()
        self._response_queue = queue.Queue(maxsize=64)
        thread = threading.Thread(target=self._reader_loop, name="MotorSerialReader", daemon=True)
        self._reader_thread = thread
        thread.start()

    def _stop_reader(self) -> None:
        if self._reader_thread is not None:
            self._reader_stop.set()
            try:
                self._reader_thread.join(timeout=1.0)
            except Exception:
                pass
        self._reader_thread = None
        self._reader_stop.clear()
        self._response_queue = queue.Queue(maxsize=64)

    def _reader_loop(self) -> None:
        handle = self._serial_handle
        if handle is None:
            return
        while not self._reader_stop.is_set():
            try:
                line = handle.readline()
            except Exception as exc:  # pragma: no cover - hardware
                self._logger.error("Serial read failed: %s", exc)
                break
            if not line:
                time.sleep(0.02)
                continue
            try:
                decoded = line.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
            if not decoded:
                continue
            self._logger.debug("<- %s", decoded)
            self._notify_line_listeners(decoded)
            self._maybe_queue_ack(decoded)

    def _notify_line_listeners(self, line: str) -> None:
        if not line:
            return
        with self._listener_lock:
            listeners = list(self._line_listeners.values())
        for listener in listeners:
            try:
                listener(line)
            except Exception:
                self._logger.debug("Line listener raised", exc_info=True)

    def _maybe_queue_ack(self, decoded: str) -> None:
        lower = decoded.lower()
        if lower.startswith("ok") or lower.startswith("error") or lower.startswith("alarm"):
            try:
                self._response_queue.put_nowait(decoded)
            except queue.Full:
                pass

    def _await_ok(self, timeout: float = 2.0) -> None:
        try:
            response = self._response_queue.get(timeout=timeout)
        except queue.Empty:
            raise MotorError("Timeout waiting for controller acknowledgement")
        lower = response.lower()
        if lower.startswith("ok"):
            return
        raise MotorError(response)

    def _send_command_blocking(self, command: str, *, timeout: float = 5.0) -> None:
        handle = self._serial_handle
        if handle is None:
            raise MotorError("Serial handle not available.")
        try:
            handle.write((command + "\r\n").encode("ascii"))
            handle.flush()
        except Exception as exc:
            raise MotorError(f"Failed to send command {command}: {exc}") from exc

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = handle.readline()
            except Exception as exc:
                raise MotorError(f"Failed to read response for {command}: {exc}") from exc
            if not line:
                time.sleep(0.05)
                continue
            decoded = line.decode("ascii", errors="ignore").strip()
            if not decoded:
                continue
            self._logger.debug("<- %s", decoded)
            self._notify_line_listeners(decoded)
            lower = decoded.lower()
            if lower.startswith("ok"):
                return
            if lower.startswith("error") or lower.startswith("alarm"):
                raise MotorError(decoded)
        raise MotorError(f"Timeout waiting for response to {command}")

    def _initialise_controller(self) -> None:
        if self.is_simulation:
            return
        handle = self._serial_handle
        if handle is None:
            return
        try:
            handle.reset_input_buffer()
        except Exception:
            pass
        try:
            handle.write(b"\r\n\r\n")
            handle.flush()
        except Exception:
            self._logger.exception("Failed to send wake sequence")
        time.sleep(0.5)
        startup_deadline = time.monotonic() + 5.0
        while time.monotonic() < startup_deadline:
            try:
                line = handle.readline()
            except Exception as exc:
                self._logger.error("Serial read failed during startup: %s", exc)
                break
            if not line:
                time.sleep(0.05)
                continue
            decoded = line.decode("ascii", errors="ignore").strip()
            if not decoded:
                continue
            self._logger.debug("<- %s", decoded)
            self._notify_line_listeners(decoded)
            if decoded.lower().startswith("grbl"):
                break
        for cmd in ("$X", "G21", "G91"):
            try:
                self._send_command_blocking(cmd, timeout=5.0)
            except MotorError as exc:
                self._logger.warning("Initial command %s failed: %s", cmd, exc)
        try:
            handle.reset_input_buffer()
        except Exception:
            pass

    def _write_line(self, command: str, *, wait_for_ok: bool = True, timeout: float = 2.0) -> None:  # pragma: no cover - hardware
        handle = self._serial_handle
        if handle is None:
            raise MotorError("Serial handle not available.")
        if not command.endswith("\r\n"):
            command = f"{command}\r\n"
        self._logger.debug("-> %s", command.strip())
        out = command.encode("ascii")
        handle.write(out)
        handle.flush()
        if wait_for_ok and not self.is_simulation:
            self._await_ok(timeout)

    def _send_realtime(self, payload: bytes) -> None:  # pragma: no cover - hardware helper
        if not payload:
            return
        handle = self._serial_handle
        if handle is None:
            raise MotorError("Serial handle not available.")
        handle.write(payload)
        handle.flush()


__all__ = [
    "LineListener",
    "MotorDriver",
    "SERIAL_AVAILABLE",
    "SerialMotorDriver",
    "SimulatedMotorDriver",
    "list_serial_ports",
]
