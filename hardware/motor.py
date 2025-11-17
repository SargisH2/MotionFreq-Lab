"""High-level motor control abstraction used by the Measurements tab.

The real hardware can be driven via a GRBL-compatible serial interface.
For development and testing the class falls back to a deterministic
simulation so the GUI remains fully interactive without hardware.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass
from itertools import count
from typing import Callable, Dict, Iterable, List, Optional

try:  # pragma: no cover - optional dependency
    import serial  # type: ignore[import]
    import serial.tools.list_ports  # type: ignore[import]
except Exception:  # pragma: no cover - simulator fallback
    serial = None  # type: ignore[assignment]

from config.env import env_bool

LOGGER = logging.getLogger("measurements.motor")

# Default simulation behaviour can be overridden via .env (USE_SIMULATION=0/1).
USE_SIMULATION_DEFAULT = env_bool("USE_SIMULATION_MOTOR", serial is None)

# Supported axes in the measurement workflow.
AXES = ("X", "Y")


def list_serial_ports() -> List[str]:
    """Return the available serial port names.

    Returns an empty list when ``pyserial`` is not available.
    """

    if serial is None:
        return []
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]


class MotorError(RuntimeError):
    """Raised when a motor command cannot be completed."""


@dataclass
class MotorState:
    """Immutable snapshot of the controller state."""

    connected: bool
    port: Optional[str]
    axis: Optional[str]
    moving: bool
    direction: int
    speed: float
    positions: Dict[str, float]
    last_error: Optional[str]


class MotorController:
    """Thread-safe abstraction for homing and jogging the motor axes."""

    def __init__(
        self,
        *,
        use_simulation: Optional[bool] = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self._logger = logger or LOGGER
        if use_simulation is None:
            use_simulation = USE_SIMULATION_DEFAULT
        self._use_simulation = bool(use_simulation)
        self._lock = threading.Lock()
        self._positions: Dict[str, float] = {axis: 0.0 for axis in AXES}
        self._connected = False
        self._port: Optional[str] = None
        self._movement_thread: Optional[threading.Thread] = None
        self._movement_stop = threading.Event()
        self._movement_axis: Optional[str] = None
        self._movement_direction = 0
        self._movement_speed = 0.0
        self._last_error: Optional[str] = None
        self._serial_handle: Optional["serial.Serial"] = None  # type: ignore[name-defined]
        self._reader_thread: Optional[threading.Thread] = None
        self._reader_stop = threading.Event()
        self._response_queue: "queue.Queue[str]" = queue.Queue(maxsize=64)
        self._listener_counter = count(1)
        self._line_listeners: Dict[int, Callable[[str], None]] = {}
        mode = "simulation" if self._use_simulation else "hardware"
        self._logger.info("MotorController initialised in %s mode", mode)

    # ------------------------------------------------------------------ #
    # Lifecycle                                                          #
    # ------------------------------------------------------------------ #
    def connect(self, port: str, *, baudrate: int = 115200, timeout: float = 0.05) -> None:
        """Open the motor connection."""

        with self._lock:
            if self._connected:
                if port != self._port:
                    self._logger.info("Switching motor connection from %s to %s", self._port, port)
                    self.disconnect()
                else:
                    self._logger.debug("Already connected to %s; skipping connect", port)
                    return

        if self._use_simulation:
            with self._lock:
                self._connected = True
                self._port = port
            self._logger.info("Motor connected in simulation mode (port=%s)", port)
            return

        if serial is None:
            raise MotorError("pyserial is not installed; cannot connect to hardware.")

        if not port:
            raise MotorError("Port must be provided when using hardware mode.")

        if not self._use_simulation and port.lower().startswith("simulated"):
            raise MotorError("Simulation port placeholder cannot be used in hardware mode.")

        try:
            handle = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, write_timeout=0.5)  # type: ignore[misc]
        except Exception as exc:  # pragma: no cover - hardware access
            self._record_error(f"Could not open port {port}: {exc}")
            raise MotorError(str(exc)) from exc

        with self._lock:
            self._serial_handle = handle
            self._connected = True
            self._port = port

        self._initialise_controller()
        self._start_reader()
        self._logger.info("Motor connected to %s (baud=%s)", port, baudrate)

    def disconnect(self) -> None:
        """Close the motor connection."""

        self.stop()
        self._stop_reader()
        with self._lock:
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

    # ------------------------------------------------------------------ #
    # Raw serial helpers                                                  #
    # ------------------------------------------------------------------ #
    def send_line(self, command: str, *, wait_for_ok: bool = True, timeout: float = 2.0) -> None:
        """Send an arbitrary G-code line to the controller."""

        if not command:
            return
        self._ensure_connected()
        if self._use_simulation:
            self._logger.debug("Simulated send: %s", command)
            return
        self._write_line(command, wait_for_ok=wait_for_ok, timeout=timeout)

    def send_raw(self, payload: bytes) -> None:
        """Send raw realtime bytes (e.g. jog cancel)."""

        if not payload:
            return
        self._ensure_connected()
        if self._use_simulation:
            self._logger.debug("Simulated raw send: %s", payload)
            return
        self._send_realtime(payload)

    def reset_input_buffer(self) -> None:
        """Clear the serial input buffer if hardware is available."""

        if self._use_simulation:
            return
        handle = self._serial_handle
        if handle is None:
            return
        try:
            handle.reset_input_buffer()
        except Exception:
            self._logger.debug("Failed to reset input buffer", exc_info=True)

    # ------------------------------------------------------------------ #
    # Movement primitives                                                #
    # ------------------------------------------------------------------ #
    def home(self, axis: str) -> None:
        """Home the given axis and reset position to zero."""

        self._ensure_axis(axis)
        self._ensure_connected()
        self.stop()

        if self._use_simulation:
            with self._lock:
                self._positions[axis] = 0.0
            self._logger.info("Homed axis %s (simulation)", axis)
            return

        if serial is None:  # pragma: no cover - defensive
            raise MotorError("pyserial is not available; cannot home axis.")

        try:  # pragma: no cover - hardware access
            # Jog back to the reported origin and reset the work offset.
            self.jog_increment(axis, -self.get_position(axis), feed=300.0)
            self._write_line(f"G10 L20 P0 {axis}0")
        except Exception as exc:
            self._record_error(f"Failed to home axis {axis}: {exc}")
            raise MotorError(str(exc)) from exc

        with self._lock:
            self._positions[axis] = 0.0
        self._logger.info("Homed axis %s (hardware)", axis)

    def move_positive(self, axis: str, speed: float) -> None:
        """Start moving the axis in the positive direction."""

        self._start_motion(axis, speed, direction=+1)

    def move_negative(self, axis: str, speed: float) -> None:
        """Start moving the axis in the negative direction."""

        self._start_motion(axis, speed, direction=-1)

    def stop(self) -> None:
        """Stop any active motion."""

        thread: Optional[threading.Thread] = None
        had_motion = False
        with self._lock:
            if self._movement_thread is not None:
                self._movement_stop.set()
                thread = self._movement_thread
                had_motion = True
            self._movement_thread = None
            self._movement_axis = None
            self._movement_direction = 0
            self._movement_speed = 0.0

        if had_motion and not self._use_simulation:  # pragma: no cover - hardware side-effect
            try:
                self._send_realtime(b"\x85")  # Jog cancel to ensure GRBL stops immediately
                time.sleep(0.05)
            except Exception:
                self._logger.exception("Failed to send jog cancel command")

        if thread is not None:
            thread.join(timeout=1.5)

        if had_motion and not self._use_simulation:  # pragma: no cover - hardware side-effect
            try:
                self._send_realtime(b"~")  # Resume so future jogs are accepted
            except Exception:
                self._logger.exception("Failed to send cycle-start command")

        self._logger.info("Motor stop requested")

    # ------------------------------------------------------------------ #
    # State query helpers                                                #
    # ------------------------------------------------------------------ #
    def get_position(self, axis: str) -> float:
        """Return the current coordinate for ``axis``."""

        self._ensure_axis(axis)
        with self._lock:
            return float(self._positions[axis])

    def get_positions(self) -> Dict[str, float]:
        """Return a copy of all axis coordinates."""

        with self._lock:
            return dict(self._positions)

    def is_connected(self) -> bool:
        """Return whether the controller is currently connected."""

        with self._lock:
            return self._connected

    def is_moving(self) -> bool:
        """Return whether a motion thread is currently active."""

        with self._lock:
            return self._movement_direction != 0

    @property
    def is_simulation(self) -> bool:
        """Return whether the controller operates in simulation mode."""

        return self._use_simulation

    def last_error(self) -> Optional[str]:
        """Return the last captured error message."""

        with self._lock:
            return self._last_error

    def snapshot(self) -> MotorState:
        """Return a snapshot of the controller state."""

        with self._lock:
            return MotorState(
                connected=self._connected,
                port=self._port,
                axis=self._movement_axis,
                moving=self._movement_direction != 0,
                direction=self._movement_direction,
                speed=self._movement_speed,
                positions=dict(self._positions),
                last_error=self._last_error,
            )

    # ------------------------------------------------------------------ #
    # Internal helpers                                                   #
    # ------------------------------------------------------------------ #
    def _start_motion(self, axis: str, speed: float, *, direction: int) -> None:
        if speed <= 0:
            raise MotorError("Speed must be a positive value.")
        if direction not in (-1, 0, +1):
            raise ValueError("Direction must be -1, 0, or +1.")

        self._ensure_axis(axis)
        self._ensure_connected()
        self.stop()

        self._movement_stop.clear()

        if self._use_simulation:
            thread = threading.Thread(
                target=self._simulate_motion,
                args=(axis, speed, direction),
                name=f"MotorSim-{axis}",
                daemon=True,
            )
        else:
            thread = threading.Thread(
                target=self._hardware_motion,
                args=(axis, speed, direction),
                name=f"MotorHw-{axis}",
                daemon=True,
            )

        with self._lock:
            self._movement_thread = thread
            self._movement_axis = axis
            self._movement_direction = direction
            self._movement_speed = speed

        if not self._use_simulation:  # pragma: no cover - hardware
            # Ensure GRBL is ready to accept new jog commands (exit any hold state).
            try:
                self._send_realtime(b"~")
            except Exception:
                self._logger.debug("Failed to send cycle start before motion", exc_info=True)

        thread.start()
        self._logger.info("Motor motion started (axis=%s dir=%s speed=%s)", axis, direction, speed)

    def _start_reader(self) -> None:
        """Spawn the background thread that mirrors serial lines into queues."""

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
        """Continuously read controller output and notify listeners."""

        handle = self._serial_handle
        if handle is None:
            return
        while not self._reader_stop.is_set():
            try:
                line = handle.readline()
            except Exception as exc:  # pragma: no cover - hardware
                self._record_error(f"Serial read failed: {exc}")
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

    def _initialise_controller(self) -> None:
        """Perform the wake-up sequence expected by GRBL controllers."""

        if self._use_simulation:
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
                self._record_error(f"Serial read failed during startup: {exc}")
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

    def _drain_responses(self) -> None:
        """Clear any queued acknowledgement lines."""

        while not self._response_queue.empty():
            try:
                self._response_queue.get_nowait()
            except queue.Empty:
                break

    def _await_ok(self, timeout: float = 2.0) -> None:
        """Block until an 'ok' response is seen or timeout expires."""

        try:
            response = self._response_queue.get(timeout=timeout)
        except queue.Empty:
            raise MotorError("Timeout waiting for controller acknowledgement")
        lower = response.lower()
        if lower.startswith("ok"):
            return
        raise MotorError(response)

    def _send_command_blocking(self, command: str, *, timeout: float = 5.0) -> None:
        """Send ``command`` over serial and wait for an acknowledgement."""

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

    def _simulate_motion(self, axis: str, speed: float, direction: int) -> None:
        """Continuously update the simulated position until stop is requested."""

        tick = 0.05  # 20 Hz updates for smooth plotting.
        self._logger.debug("Simulation motion thread active for axis %s", axis)
        start = time.perf_counter()
        try:
            while not self._movement_stop.is_set():
                step = speed * direction * tick / 60.0
                self._apply_increment(axis, step)
                time.sleep(tick)
                if time.perf_counter() - start > 6 * 60:  # safety: stop after 6 minutes
                    self._logger.warning("Simulation motion exceeded safety timeout; stopping")
                    break
        finally:
            self._movement_stop.clear()
            with self._lock:
                self._movement_thread = None
                self._movement_axis = None
                self._movement_direction = 0
                self._movement_speed = 0.0
            self._logger.debug("Simulation motion thread finished for axis %s", axis)

    def _hardware_motion(self, axis: str, speed: float, direction: int) -> None:  # pragma: no cover - hardware
        """Issue repeated jogs over serial to approximate continuous motion."""

        try:
            step = max(speed / 120.0, 0.1) * (1 if direction > 0 else -1)
            feed = max(speed, 1.0)
            while not self._movement_stop.is_set():
                try:
                    self.jog_increment(axis, step, feed)
                except MotorError as exc:
                    self._record_error(f"Motion error ({axis}): {exc}")
                    break
        except Exception as exc:
            self._record_error(f"Motion error ({axis}): {exc}")
        finally:
            self._movement_stop.clear()
            with self._lock:
                self._movement_thread = None
                self._movement_axis = None
                self._movement_direction = 0
                self._movement_speed = 0.0
            self._logger.debug("Hardware motion thread finished for axis %s", axis)

    def jog_increment(self, axis: str, distance: float, feed: float) -> None:
        """Perform a single incremental jog."""

        self._ensure_axis(axis)
        self._ensure_connected()
        feed = max(feed, 1.0)

        if self._use_simulation:
            self._apply_increment(axis, distance)
            est = abs(distance) / feed * 60.0
            time.sleep(max(est, 0.02))
            return

        cmd = f"$J=G91 {axis}{distance:+.3f} F{feed:.1f}"
        self._write_line(cmd)
        self._apply_increment(axis, distance)
        est = abs(distance) / feed * 60.0
        time.sleep(max(est, 0.05))

    def _apply_increment(self, axis: str, delta: float) -> None:
        """Apply a simulated or estimated delta to the cached axis position."""

        with self._lock:
            self._positions[axis] += delta

    def _record_error(self, message: str) -> None:
        """Persist the last error and log it for diagnostics."""

        self._logger.error("%s", message)
        with self._lock:
            self._last_error = message

    def register_line_listener(self, callback: Callable[[str], None]) -> Callable[[], None]:
        """Register a callback that receives every raw controller line."""

        if callback is None:
            raise ValueError("callback must not be None")
        token = next(self._listener_counter)
        with self._lock:
            self._line_listeners[token] = callback

        def _unregister() -> None:
            with self._lock:
                self._line_listeners.pop(token, None)

        return _unregister

    def _notify_line_listeners(self, line: str) -> None:
        if not line:
            return
        with self._lock:
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

    def _ensure_connected(self) -> None:
        if not self.is_connected():
            raise MotorError("Motor is not connected.")

    def _ensure_axis(self, axis: str) -> None:
        if axis not in AXES:
            raise MotorError(f"Unsupported axis '{axis}'. Expected one of {AXES}.")

    # ------------------------------------------------------------------ #
    # Hardware helpers                                                   #
    # ------------------------------------------------------------------ #
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
        if wait_for_ok and not self._use_simulation:
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
    "AXES",
    "MotorController",
    "MotorError",
    "MotorState",
    "USE_SIMULATION_DEFAULT",
    "list_serial_ports",
]
