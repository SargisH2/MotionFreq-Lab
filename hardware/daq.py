"""Data acquisition helpers for the Measurements tab."""

from __future__ import annotations

import logging
import math
import random
import threading
import time
from typing import TYPE_CHECKING, Callable, Iterator, Optional, Tuple

from config.env import env_bool

LOGGER = logging.getLogger("measurements.daq")

try:  # pragma: no cover - optional when hardware files missing
    from read_freq.controller import (
        ControllerError,
        DEFAULT_BAUDRATE,
        DEFAULT_CALIBRATION,
        DEFAULT_TIMER_MP,
        VWFrequencyController,
    )
except Exception as exc:  # pragma: no cover - fallback to simulation
    VWFrequencyController = None  # type: ignore[assignment]
    ControllerError = RuntimeError  # type: ignore[assignment]
    DEFAULT_BAUDRATE = 38400
    DEFAULT_CALIBRATION = 2
    DEFAULT_TIMER_MP = 0.008192
    _CONTROLLER_IMPORT_ERROR: Optional[Exception] = exc
else:
    _CONTROLLER_IMPORT_ERROR = None

if TYPE_CHECKING:  # pragma: no cover - typing aid
    from read_freq.controller import FrequencyFrame

# Deterministic pseudo random generator ensures reproducibility between runs.
_random = random.Random(42)

# Recommended minimum interval between hardware reads (seconds).
MIN_SAMPLE_INTERVAL = 0.0005  # 20 Hz upper bound

# Global default for DAQ simulation behaviour.
USE_SIMULATION_DEFAULT = env_bool("USE_SIMULATION_DAQ", True)

# Default handshake parameters used by the legacy controller.
DEFAULT_MODE = 3
DEFAULT_K_VALUE = 31


class DataAcquisitionError(RuntimeError):
    """Raised when the DAQ cannot produce a measurement."""


class FrequencyDAQ:
    """Non-blocking frequency reader with optional hardware integration."""

    def __init__(
        self,
        *,
        use_simulation: Optional[bool] = None,
        controller_factory: Optional[Callable[[], "VWFrequencyController"]] = None, # type: ignore[assignment]
        read_timeout: float = 3.0,
        mode: int = DEFAULT_MODE,
        k_value: int = DEFAULT_K_VALUE,
        baudrate: int = DEFAULT_BAUDRATE,
        calibration: int = DEFAULT_CALIBRATION,
        timer_mp: float = DEFAULT_TIMER_MP,
    ) -> None:
        if use_simulation is None:
            use_simulation = USE_SIMULATION_DEFAULT
        self._use_simulation = bool(use_simulation)
        self._logger = LOGGER
        self._controller_factory = controller_factory
        self._controller: Optional["VWFrequencyController"] = None # type: ignore[assignment]
        self._controller_port: Optional[str] = None
        self._connected_port: Optional[str] = None
        self._read_timeout = float(read_timeout)
        self._baudrate = int(baudrate)
        self._calibration = int(calibration)
        self._timer_mp = float(timer_mp)

        self._mode = int(mode)
        self._k_value = int(k_value)
        self._measurement_window = 0.0
        self._measurement_gap = MIN_SAMPLE_INTERVAL
        self._stream_lock = threading.Lock()
        self._acquire_lock = threading.Lock()
        self._controller_lock = threading.Lock()

        if not self._use_simulation and VWFrequencyController is None:
            self._logger.warning(
                "Hardware DAQ requested but controller dependency is unavailable (%s); "
                "falling back to simulation.",
                _CONTROLLER_IMPORT_ERROR,
            )
            self._use_simulation = True

        mode_name = "simulation" if self._use_simulation else "hardware"
        self._logger.info("FrequencyDAQ initialised in %s mode (mode=%s, k=%s)", mode_name, self._mode, self._k_value)

    # ------------------------------------------------------------------ #
    # Public API                                                         #
    # ------------------------------------------------------------------ #
    def connect(self, port: str, *, configure: bool = True) -> None:
        """Connect to the DAQ on the specified serial ``port``."""

        if not port:
            raise DataAcquisitionError("Serial port must be provided for DAQ connection.")

        if not self._use_simulation and port.lower().startswith("simulated"):
            raise DataAcquisitionError("Simulation port placeholder cannot be used in hardware mode.")

        if self._use_simulation:
            self._connected_port = port
            return

        factory = self._controller_factory
        if factory is None:
            if VWFrequencyController is None:
                raise DataAcquisitionError("DAQ controller unavailable; cannot enter hardware mode.")
            factory = VWFrequencyController

        controller = factory(
            baudrate=self._baudrate,
            read_timeout=self._read_timeout,
            calibration=self._calibration,
            timer_mp=self._timer_mp,
        )
        try:
            controller.connect(port)
            if configure:
                self._apply_measurement_configuration(controller)
        except ControllerError as exc:  # pragma: no cover - hardware path
            controller.close()
            raise DataAcquisitionError(f"Failed to connect DAQ on {port}: {exc}") from exc
        except DataAcquisitionError:
            controller.close()
            raise

        with self._controller_lock:
            previous = self._controller
            self._controller = controller
            self._controller_port = port
            self._connected_port = port
        if previous is not None:
            try:
                previous.close()
            except Exception:
                self._logger.exception("Error while closing previous DAQ controller")
        self._logger.info(
            "FrequencyDAQ connected on %s (gap~=%.3fs, window~=%.3fs)", port, self._measurement_gap, self._measurement_window
        )

    def disconnect(self) -> None:
        """Disconnect and release resources."""

        if self._use_simulation:
            self._connected_port = None
            return

        with self._controller_lock:
            controller = self._controller
            self._controller = None
            self._controller_port = None
            self._connected_port = None
        if controller is not None:
            try:  # pragma: no cover - hardware path
                controller.close()
            except Exception:
                self._logger.exception("Error while closing DAQ controller")
        self._logger.info("FrequencyDAQ disconnected")

    def is_connected(self) -> bool:
        """Return whether the DAQ is ready for measurements."""

        if self._use_simulation:
            return True
        with self._controller_lock:
            return self._controller is not None

    @property
    def connected_port(self) -> Optional[str]:
        """Return the last connected port (if any)."""

        return self._connected_port

    @property
    def is_simulation(self) -> bool:
        """Return whether the DAQ operates in simulation mode."""

        return self._use_simulation

    @property
    def measurement_configuration(self) -> Tuple[int, int]:
        """Return the mode/k configuration currently in use."""

        return self._mode, self._k_value

    @property
    def measurement_gap(self) -> float:
        """Return the enforced delay between hardware triggers (seconds)."""

        return self._measurement_gap

    @property
    def measurement_window(self) -> float:
        """Return the expected measurement window duration (seconds)."""

        return self._measurement_window

    def set_measurement_parameters(self, mode: int, k_value: int) -> None:
        """Update the handshake parameters used for subsequent measurements."""

        self._mode = int(mode)
        self._k_value = int(k_value)
        if self._use_simulation:
            return

        controller = self._get_controller(strict=False)
        if controller is None:
            return

        try:
            self._apply_measurement_configuration(controller)
        except ControllerError as exc:  # pragma: no cover - hardware path
            raise DataAcquisitionError(f"Failed to configure DAQ: {exc}") from exc
        self._logger.info(
            "Updated DAQ configuration (mode=%s, k=%s, gap~=%.3fs)",
            self._mode,
            self._k_value,
            self._measurement_gap,
        )


    def read_frequencies(self) -> Tuple[float, float]:
        """Return a single pair of channel frequencies."""

        if self._use_simulation:
            return self._simulate_sample(time.perf_counter())

        iterator = self.iter_frequencies(timeout=self._read_timeout)
        try:
            return next(iterator)
        except StopIteration as exc:
            raise DataAcquisitionError("DAQ did not return any data.") from exc
        finally:
            try:
                iterator.close()
            except Exception:
                pass

    def iter_frames(
        self,
        *,
        timeout: Optional[float] = None,
        interval: float = 0.0,
        auto_trigger: bool = True,
    ) -> Iterator["FrequencyFrame"]:
        """Yield raw frames from the hardware DAQ."""

        if self._use_simulation:
            raise DataAcquisitionError("Frame iterator unavailable when running in simulation mode.")

        if timeout is None:
            timeout = self._read_timeout

        self._stream_lock.acquire()

        controller = self._get_controller()

        def _generator() -> Iterator["FrequencyFrame"]:
            try:
                for frame in controller.iter_frames(
                    count=None,
                    timeout=timeout,
                    auto_trigger=auto_trigger,
                    interval=interval,
                ):
                    yield frame
            except ControllerError as exc:
                raise DataAcquisitionError(f"DAQ read failed, try to increase timeout: {exc}") from exc
            finally:
                self._stream_lock.release()

        return _generator()

    def iter_frequencies(
        self,
        *,
        timeout: Optional[float] = None,
        interval: float = 0.0,
        auto_trigger: bool = True,
    ) -> Iterator[Tuple[float, float]]:
        """Yield channel frequencies as they are produced."""

        if self._use_simulation:
            return self._iter_simulated(interval)

        frames = self.iter_frames(timeout=timeout, interval=interval, auto_trigger=auto_trigger)

        def _generator() -> Iterator[Tuple[float, float]]:
            try:
                for frame in frames:
                    yield float(frame.channel1.frequency_hz), float(frame.channel2.frequency_hz)
            finally:
                try:
                    frames.close()
                except Exception:
                    pass

        return _generator()

    @property
    def read_timeout(self) -> float:
        """Return the configured read timeout in seconds."""

        return self._read_timeout

    def set_transport_parameters(
        self,
        *,
        baudrate: int,
        read_timeout: float,
        calibration: int,
        timer_mp: float,
    ) -> None:
        """Update serial transport parameters used for the next connection."""

        self._baudrate = int(baudrate)
        self._read_timeout = float(read_timeout)
        self._calibration = int(calibration)
        self._timer_mp = float(timer_mp)
        if not self._use_simulation and self._controller is not None:
            self.disconnect()


    def _apply_measurement_configuration(self, controller: "VWFrequencyController") -> None: # type: ignore[assignment]
        """Apply mode/k settings and refresh timing metadata."""

        controller.configure_measurement(self._mode, self._k_value)
        try:
            window = float(controller.measurement_window_seconds())
        except Exception:
            window = 0.0
        try:
            gap = float(controller.measurement_gap_seconds())
        except Exception:
            gap = MIN_SAMPLE_INTERVAL
        if gap <= 0.0:
            gap = MIN_SAMPLE_INTERVAL
        self._measurement_window = window
        self._measurement_gap = max(MIN_SAMPLE_INTERVAL, gap)

    def _get_controller(self, *, strict: bool = True) -> Optional["VWFrequencyController"]: # type: ignore[assignment]
        """Return the active controller, optionally raising when disconnected."""

        with self._controller_lock:
            controller = self._controller
        if controller is None and strict:
            raise DataAcquisitionError("DAQ is not connected.")
        return controller

    def _iter_simulated(self, interval: float) -> Iterator[Tuple[float, float]]:
        """Yield simulated frequency pairs at roughly ``interval`` seconds."""

        interval = max(interval, MIN_SAMPLE_INTERVAL)

        def _generator() -> Iterator[Tuple[float, float]]:
            try:
                while True:
                    yield self._simulate_sample(time.perf_counter())
                    time.sleep(interval)
            except GeneratorExit:
                return

        return _generator()

    def _simulate_sample(self, ts: float) -> Tuple[float, float]:
        """Return a pseudo-random pair of channel frequencies for timestamp ``ts``."""

        # The simulator generates a smooth waveform with low-amplitude noise.
        base = math.sin(ts / 3.0) * 5.0 + 100.0
        drift = math.cos(ts / 6.0) * 2.0
        noise1 = _random.uniform(-0.5, 0.5)
        noise2 = _random.uniform(-0.5, 0.5)
        ch1 = base + drift + noise1
        ch2 = base - drift + noise2
        return max(ch1, 0.0), max(ch2, 0.0)


__all__ = [
    "DataAcquisitionError",
    "DEFAULT_BAUDRATE",
    "DEFAULT_CALIBRATION",
    "DEFAULT_K_VALUE",
    "DEFAULT_MODE",
    "DEFAULT_TIMER_MP",
    "FrequencyDAQ",
    "MIN_SAMPLE_INTERVAL",
    "USE_SIMULATION_DEFAULT",
]
