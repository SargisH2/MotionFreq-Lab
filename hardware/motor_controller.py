"""High-level motor controller built on top of low-level drivers."""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Dict, Optional, Tuple

from config.env import env_bool

from .motor_driver import (
    MotorDriver,
    SERIAL_AVAILABLE,
    SerialMotorDriver,
    SimulatedMotorDriver,
)
from .motor_types import AXES, MotorError, MotorState, normalise_axis

LOGGER = logging.getLogger("measurements.motor")
USE_SIMULATION_DEFAULT = env_bool("USE_SIMULATION_MOTOR", not SERIAL_AVAILABLE)


class MotorController:
    """Thread-safe abstraction that orchestrates motor movement."""

    def __init__(
        self,
        *,
        driver: Optional[MotorDriver] = None,
        use_simulation: Optional[bool] = None,
        default_feed: float = 1600.0,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self._logger = logger or LOGGER
        if driver is None:
            if use_simulation is None:
                use_simulation = USE_SIMULATION_DEFAULT
            self._driver = SimulatedMotorDriver(self._logger) if use_simulation else SerialMotorDriver(self._logger)
        else:
            self._driver = driver
            if use_simulation is None:
                use_simulation = driver.is_simulation

        self._use_simulation = bool(use_simulation)
        self._lock = threading.Lock()
        self._positions: Dict[str, float] = {axis: 0.0 for axis in AXES}
        self._soft_limits: Dict[str, Optional[Tuple[Optional[float], Optional[float]]]] = {
            axis: None for axis in AXES
        }
        self._movement_thread: Optional[threading.Thread] = None
        self._movement_stop = threading.Event()
        self._movement_axis: Optional[str] = None
        self._movement_direction = 0
        self._movement_speed = 0.0
        self._status_query_lock = threading.Lock()
        self._last_error: Optional[str] = None
        self._default_feed = max(1.0, float(default_feed))

        mode = "simulation" if self._use_simulation else "hardware"
        self._logger.info("MotorController initialised in %s mode", mode)

    # ------------------------------------------------------------------ #
    # Lifecycle                                                          #
    # ------------------------------------------------------------------ #
    def connect(self, port: str, *, baudrate: int = 115200, timeout: float = 0.05) -> None:
        """Open the motor connection."""

        self._driver.connect(port, baudrate=baudrate, timeout=timeout)

    def disconnect(self) -> None:
        """Close the motor connection."""

        self.stop()
        try:
            self._driver.disconnect()
        except Exception:
            self._logger.debug("Motor disconnect failed", exc_info=True)

    # ------------------------------------------------------------------ #
    # Raw helpers                                                        #
    # ------------------------------------------------------------------ #
    def send_line(self, command: str, *, wait_for_ok: bool = True, timeout: float = 2.0) -> None:
        """Send an arbitrary G-code line to the controller."""

        if not command:
            return
        self._ensure_connected()
        if self._use_simulation:
            self._logger.debug("Simulated send: %s", command)
            return
        self._driver.send_line(command, wait_for_ok=wait_for_ok, timeout=timeout)

    def send_raw(self, payload: bytes) -> None:
        """Send raw realtime bytes (e.g. jog cancel)."""

        if not payload:
            return
        self._ensure_connected()
        if self._use_simulation:
            self._logger.debug("Simulated raw send: %s", payload)
            return
        self._driver.send_raw(payload)

    def reset_input_buffer(self) -> None:
        """Clear the serial input buffer if hardware is available."""

        if self._use_simulation:
            return
        try:
            self._driver.reset_input_buffer()
        except Exception:
            self._logger.debug("Failed to reset input buffer", exc_info=True)

    # ------------------------------------------------------------------ #
    # Movement primitives                                                #
    # ------------------------------------------------------------------ #
    def home(self, axis: str) -> None:
        """Home the given axis and reset position to zero."""

        axis = normalise_axis(axis)
        self._ensure_connected()
        self.stop()

        if self._use_simulation:
            with self._lock:
                self._positions[axis] = 0.0
            self._logger.info("Homed axis %s (simulation)", axis)
            return

        try:  # pragma: no cover - hardware access
            # Jog back to the reported origin and reset the work offset.
            self.jog_increment(axis, -self.get_position(axis), feed=1600.0)
            cmd = f"G10 L20 P0 {axis}0"
            _, wpos = self.read_status_positions(timeout=0.3)
            if wpos:
                for other in AXES:
                    if other != axis and other in wpos:
                        cmd += f" {other}{wpos[other]:.3f}"
            self._driver.send_line(cmd)
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

    def move_by(self, axis: str, delta: float, *, feed: Optional[float] = None) -> None:
        """Move ``axis`` by the requested delta."""

        axis = normalise_axis(axis)
        self.stop()
        feed_value = self._coerce_feed(feed)
        self.jog_increment(axis, delta, feed_value)

    def move_to_coordinate(
        self,
        axis: str,
        coordinate: float,
        *,
        feed: Optional[float] = None,
        tolerance: float = 1e-3,
    ) -> None:
        """Move an axis to an absolute coordinate."""

        axis = normalise_axis(axis)
        self._ensure_connected()
        target = float(coordinate)
        feed_value = self._coerce_feed(feed)
        current = self.get_position(axis)
        if abs(target - current) < tolerance:
            self._logger.debug("Axis %s already at coordinate %.3f (tol=%.3f)", axis, target, tolerance)
            return
        self._logger.info("Moving axis %s to %.3f (current=%.3f)", axis, target, current)
        self.move_by(axis, target - current, feed=feed_value)

    def move_to_start(self, axis: str, *, feed: Optional[float] = None, start_position: Optional[float] = None) -> None:
        """Convenience helper to move to the configured start (lower) boundary."""

        axis = normalise_axis(axis)
        target = start_position
        limits = self._soft_limits.get(axis)
        if target is None and limits is not None:
            target = limits[0]
        if target is None:
            target = 0.0
        self.move_to_coordinate(axis, target, feed=feed)

    def move_to_end(self, axis: str, *, feed: Optional[float] = None, end_position: Optional[float] = None) -> None:
        """Move to the configured end (upper) boundary."""

        axis = normalise_axis(axis)
        target = end_position
        limits = self._soft_limits.get(axis)
        if target is None and limits is not None:
            target = limits[1]
        if target is None:
            raise MotorError("End position not configured for axis; provide end_position or set soft limits.")
        self.move_to_coordinate(axis, target, feed=feed)

    def jog_increment(self, axis: str, distance: float, feed: float) -> None:
        """Perform a single incremental jog."""

        axis = normalise_axis(axis)
        self._ensure_connected()
        feed = self._coerce_feed(feed)

        if self._use_simulation:
            self._apply_increment(axis, distance)
            est = abs(distance) / feed * 60.0
            time.sleep(max(est, 0.02))
            return

        self._driver.jog_increment(axis, distance, feed)
        self._apply_increment(axis, distance)
        est = abs(distance) / feed * 60.0
        time.sleep(max(est, 0.05))

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
                self._driver.send_raw(b"\x85")  # Jog cancel to ensure GRBL stops immediately
                time.sleep(0.05)
            except Exception:
                self._logger.exception("Failed to send jog cancel command")

        if thread is not None:
            thread.join(timeout=1.5)

        if had_motion and not self._use_simulation:  # pragma: no cover - hardware side-effect
            try:
                self._driver.send_raw(b"~")  # Resume so future jogs are accepted
            except Exception:
                self._logger.exception("Failed to send cycle-start command")

        self._logger.info("Motor stop requested")

    # ------------------------------------------------------------------ #
    # State query helpers                                                #
    # ------------------------------------------------------------------ #
    def get_position(self, axis: str) -> float:
        """Return the current coordinate for ``axis``."""

        axis = normalise_axis(axis)
        with self._lock:
            return float(self._positions[axis])

    def get_positions(self) -> Dict[str, float]:
        """Return a copy of all axis coordinates."""

        with self._lock:
            return dict(self._positions)

    def set_soft_limits(
        self,
        axis: str,
        *,
        minimum: Optional[float] = None,
        maximum: Optional[float] = None,
    ) -> None:
        """Persist optional soft limits used by high-level helpers."""

        axis = normalise_axis(axis)
        self._soft_limits[axis] = (minimum, maximum)

    def set_position(self, axis: str, value: float) -> None:
        """Manually override the cached position for an axis (e.g., after G92)."""

        axis = normalise_axis(axis)
        with self._lock:
            self._positions[axis] = float(value)

    def is_connected(self) -> bool:
        """Return whether the controller is currently connected."""

        try:
            return self._driver.is_connected()
        except Exception:
            return False

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
            axis = self._movement_axis
            direction = self._movement_direction
            speed = self._movement_speed
            positions = dict(self._positions)
            last_error = self._last_error
        return MotorState(
            connected=self.is_connected(),
            port=self._driver.port,
            axis=axis,
            moving=direction != 0,
            direction=direction,
            speed=speed,
            positions=positions,
            last_error=last_error,
        )

    def read_status_positions(
        self,
        *,
        timeout: float = 0.2,
    ) -> Tuple[Optional[Dict[str, float]], Optional[Dict[str, float]]]:
        """Query GRBL status and return (MPos, WPos) dicts when available."""

        if not self.is_connected():
            return None, None
        if self._use_simulation:
            positions = self.get_positions()
            return positions, positions

        line = self._query_status_line(timeout=timeout)
        if not line:
            return None, None
        return self._parse_status_positions(line)

    def read_status_position(
        self,
        axis: str,
        *,
        use_machine: bool = False,
        timeout: float = 0.2,
    ) -> Optional[float]:
        """Return the axis position from GRBL status (MPos/WPos) or None."""

        axis = normalise_axis(axis)
        mpos, wpos = self.read_status_positions(timeout=timeout)
        preferred = mpos if use_machine else wpos
        if preferred and axis in preferred:
            return float(preferred[axis])
        return None

    # ------------------------------------------------------------------ #
    # Internal helpers                                                   #
    # ------------------------------------------------------------------ #
    def _start_motion(self, axis: str, speed: float, *, direction: int) -> None:
        if speed <= 0:
            raise MotorError("Speed must be a positive value.")
        if direction not in (-1, 0, +1):
            raise ValueError("Direction must be -1, 0, or +1.")

        axis = normalise_axis(axis)
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
            try:
                self._driver.send_raw(b"~")
            except Exception:
                self._logger.debug("Failed to send cycle start before motion", exc_info=True)

        thread.start()
        self._logger.info("Motor motion started (axis=%s dir=%s speed=%s)", axis, direction, speed)

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

    def _query_status_line(self, *, timeout: float = 0.2) -> Optional[str]:
        if self._use_simulation:
            return None
        self._ensure_connected()
        status_line: Dict[str, Optional[str]] = {"line": None}
        event = threading.Event()

        def _listener(line: str) -> None:
            if line.startswith("<"):
                status_line["line"] = line
                event.set()

        with self._status_query_lock:
            unregister = self._driver.register_line_listener(_listener)
            try:
                self.send_raw(b"?")
                event.wait(timeout)
            finally:
                unregister()

        return status_line["line"]

    def _parse_status_positions(
        self, line: str
    ) -> Tuple[Optional[Dict[str, float]], Optional[Dict[str, float]]]:
        if not line or not line.startswith("<"):
            return None, None
        return (
            self._extract_status_positions(line, "MPos:"),
            self._extract_status_positions(line, "WPos:"),
        )

    def _extract_status_positions(self, line: str, token: str) -> Optional[Dict[str, float]]:
        if token not in line:
            return None
        segment = line.split(token, 1)[1].split("|", 1)[0]
        parts = segment.split(",")
        axes = ("X", "Y", "Z")
        positions: Dict[str, float] = {}
        for axis, value in zip(axes, parts):
            try:
                positions[axis] = float(value)
            except (TypeError, ValueError):
                continue
        return positions or None

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

        return self._driver.register_line_listener(callback)

    def _ensure_connected(self) -> None:
        if not self.is_connected():
            raise MotorError("Motor is not connected.")

    def _coerce_feed(self, feed: Optional[float]) -> float:
        try:
            value = float(feed) if feed is not None else self._default_feed
        except Exception:
            value = self._default_feed
        return max(1.0, value)


__all__ = [
    "AXES",
    "MotorController",
    "MotorError",
    "MotorState",
    "USE_SIMULATION_DEFAULT",
]
