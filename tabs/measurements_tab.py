"""Measurements tab that coordinates motor movement and frequency capture."""

from __future__ import annotations

import logging
import threading
import time
import tkinter as tk
from contextlib import closing
from dataclasses import dataclass
from datetime import datetime
from tkinter import filedialog, messagebox, ttk
from typing import Callable, Dict, Iterator, List, Optional, Tuple

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from config.motor import MotorConfig, load_motor_config, save_motor_config
from export.excel import export_measurements_to_excel
from hardware.daq import (
    DataAcquisitionError,
    DEFAULT_BAUDRATE,
    DEFAULT_CALIBRATION,
    DEFAULT_K_VALUE,
    DEFAULT_MODE,
    DEFAULT_TIMER_MP,
    FrequencyDAQ,
    MIN_SAMPLE_INTERVAL,
)
from hardware.motor_controller import MotorController
from hardware.motor_driver import list_serial_ports
from hardware.motor_types import AXES, MotorError, MotorState
from ui.motor_shared import prepare_port_list, select_port_value
from models.data_buffers import MeasurementBuffer, MeasurementRow, ModeLiteral, PlotIdLiteral

LOGGER = logging.getLogger("measurements.tab")
logging.getLogger("matplotlib").setLevel(logging.WARNING)
logging.getLogger("matplotlib.font_manager").setLevel(logging.WARNING)

# Sampling cadence for acquisition workers (seconds).
SAMPLING_INTERVAL = max(MIN_SAMPLE_INTERVAL, 0.01)
# Timeout for GRBL status requests (seconds).
STATUS_QUERY_TIMEOUT = 0.2
# Poll cadence when waiting on sweep positioning (seconds).
SWEEP_POLL_INTERVAL = 0.05
# Position tolerance for sweep start/end checks (mm).
SWEEP_POSITION_TOLERANCE = 0.01

MODE_LABELS: Dict[ModeLiteral, str] = {
    "no_motor_time": "No Motor (Time)",
    "pos_from_zero": "Motor + Direction",
    "neg_from_zero": "Motor - Direction",
}

MODE_XLABELS: Dict[ModeLiteral, str] = {
    "no_motor_time": "Time (s)",
    "pos_from_zero": "Coordinate (mm)",
    "neg_from_zero": "Coordinate (mm)",
}

SIMULATED_PORT_NAME = "Simulated Motor"


@dataclass
class PlotRuntime:
    """Tracks runtime state for each plot."""

    plot_id: PlotIdLiteral
    card: "PlotCard"
    buffer: MeasurementBuffer
    mode: Optional[ModeLiteral] = None
    thread: Optional[threading.Thread] = None
    motor_thread: Optional[threading.Thread] = None
    stop_event: Optional[threading.Event] = None
    start_monotonic: float = 0.0
    start_time: Optional[datetime] = None
    freq_iter: Optional[Iterator[Tuple[float, float]]] = None
    last_measurement_finished_at: float = 0.0
    next_measurement_not_before: float = 0.0
    last_coordinate_mm: float = 0.0
    sweep_started_event: Optional[threading.Event] = None
    sweep_start_mm: float = 0.0
    sweep_end_mm: float = 0.0
    coord_system: str = "work"
    stop_motor_on_stop: bool = True
    stop_reason: Optional[str] = None


class PlotCard(ttk.LabelFrame):
    """Encapsulates the UI elements for a single measurement plot."""

    def __init__(
        self,
        parent: tk.Widget,
        plot_id: PlotIdLiteral,
        *,
        start_callback: Callable[[PlotIdLiteral, ModeLiteral], None],
    ) -> None:
        super().__init__(parent, text=f"Plot {plot_id}")
        self.plot_id = plot_id
        self._start_callback = start_callback

        self.columnconfigure(0, weight=1)

        self._figure = Figure(figsize=(6, 2.6), dpi=100)
        ax = self._figure.add_subplot(111)
        ax.set_title("Frequency vs Coordinate")
        ax.set_xlabel("Coordinate (mm)")
        ax.set_ylabel("Frequency (Hz)")
        ax.grid(True, linestyle="--", alpha=0.5)
        (self._line_ch1,) = ax.plot([], [], label="Channel 1")
        (self._line_ch2,) = ax.plot([], [], label="Channel 2")
        ax.legend(loc="upper right")
        self._axes = ax

        self._canvas = FigureCanvasTkAgg(self._figure, master=self)
        canvas_widget = self._canvas.get_tk_widget()
        canvas_widget.grid(row=0, column=0, sticky="nsew", padx=8, pady=(8, 4))

        controls = ttk.Frame(self)
        controls.grid(row=1, column=0, sticky="ew", padx=8)
        controls.columnconfigure((0, 1, 2), weight=1)

        self._mode_buttons: Dict[ModeLiteral, ttk.Button] = {}
        for idx, mode in enumerate(MODE_LABELS, start=0):
            btn = ttk.Button(controls, text=MODE_LABELS[mode], command=lambda m=mode: self._on_mode_clicked(m))
            btn.grid(row=0, column=idx, sticky="ew", padx=2, pady=2)
            self._mode_buttons[mode] = btn

        self._status_var = tk.StringVar(value="Points: 0 | Coord: 0.000 | Ch1: 0.00 Hz | Ch2: 0.00 Hz")
        status_lbl = ttk.Label(self, textvariable=self._status_var, anchor="w")
        status_lbl.grid(row=2, column=0, sticky="ew", padx=8, pady=(4, 8))

        self._current_display_mode: Optional[ModeLiteral] = None
        self._current_running_mode: Optional[ModeLiteral] = None
        self._x_data: List[float] = []
        self._ch1_data: List[float] = []
        self._ch2_data: List[float] = []

    # ------------------------------------------------------------------ #
    def _on_mode_clicked(self, mode: ModeLiteral) -> None:
        self._start_callback(self.plot_id, mode)

    def set_running(self, mode: Optional[ModeLiteral]) -> None:
        """Update button states to reflect the active mode."""

        self._current_running_mode = mode
        for m, btn in self._mode_buttons.items():
            if mode is None:
                btn.configure(text=MODE_LABELS[m], state=tk.NORMAL)
            elif m == mode:
                btn.configure(text="Stop", state=tk.NORMAL)
            else:
                btn.configure(state=tk.DISABLED)

    def reset_plot(self, mode: ModeLiteral) -> None:
        """Clear plotted data and set axis labels for the new mode."""

        self._current_display_mode = mode
        self._axes.set_xlabel(MODE_XLABELS[mode])
        self._x_data.clear()
        self._ch1_data.clear()
        self._ch2_data.clear()
        self._update_lines()

    def append_point(self, x_value: float, ch1: float, ch2: float, *, mode: ModeLiteral) -> None:
        """Append a new point to the plot."""

        if self._current_display_mode != mode:
            self.reset_plot(mode)
        self._x_data.append(x_value)
        self._ch1_data.append(ch1)
        self._ch2_data.append(ch2)
        self._update_lines()

    def _update_lines(self) -> None:
        self._line_ch1.set_data(self._x_data, self._ch1_data)
        self._line_ch2.set_data(self._x_data, self._ch2_data)
        self._axes.relim()
        self._axes.autoscale_view()
        self._canvas.draw_idle()

    def update_status(self, *, points: int, coordinate: float, ch1: float, ch2: float) -> None:
        """Update the textual status line."""

        self._status_var.set(
            f"Points: {points} | Coord: {coordinate:.3f} | Ch1: {ch1:.2f} Hz | Ch2: {ch2:.2f} Hz"
        )


class MeasurementsTab(ttk.Frame):
    """Tkinter tab that exposes frequency measurement workflows."""

    def __init__(self, parent: tk.Widget, *, daq: Optional[FrequencyDAQ] = None, motor: Optional[MotorController] = None) -> None:
        super().__init__(parent)
        self._logger = LOGGER
        self._daq = daq or FrequencyDAQ()
        self._motor = motor or MotorController()
        self._motor_config: MotorConfig = load_motor_config()
        self._last_motor_connected = self._motor.is_connected()
        self._plots: Dict[PlotIdLiteral, PlotRuntime] = {}
        self._status_poll_id: Optional[str] = None
        self._active_motor_plot: Optional[PlotIdLiteral] = None
        self._daq_interval_sec = 0.0
        self._daq_auto_trigger = True
        self._active_daq_plot: Optional[PlotIdLiteral] = None
        self._config_save_lock = threading.Lock()
        self._config_save_pending: Optional[MotorConfig] = None
        self._config_save_running = False

        self._build_ui()
        self._poll_status()
        self.after(200, self.refresh_ports)

    # ------------------------------------------------------------------ #
    # Unit helpers
    def _mm_to_mm(self, value_mm: float) -> float:
        try:
            return float(value_mm)
        except Exception:
            return 0.0

    # ------------------------------------------------------------------ #
    def _build_ui(self) -> None:
        """Create all static widgets for the measurements tab."""
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        top = ttk.Frame(self)
        top.grid(row=0, column=0, sticky="ew", padx=12, pady=8)
        top.columnconfigure(8, weight=1)
        top.columnconfigure(11, weight=1)

        cfg = self._motor_config
        speed_default = float(getattr(cfg, "jog_feed", 1600.0) or 1600.0)
        axis_choices = [axis for axis in AXES if bool(cfg.axis_enabled.get(axis, True))]
        if not axis_choices:
            axis_choices = list(AXES)
        axis_default = cfg.default_axis if cfg.default_axis in axis_choices else axis_choices[0]

        ttk.Label(top, text="Motor Feed (mm/min):").grid(row=0, column=0, sticky="w")
        self.speed_var = tk.DoubleVar(value=max(1.0, speed_default))
        self.speed_spin = ttk.Spinbox(
            top,
            from_=1.0,
            to=30000.0,
            increment=10.0,
            textvariable=self.speed_var,
            width=8,
            format="%.1f",
            command=self._on_speed_changed,
        )
        self.speed_spin.grid(row=0, column=1, padx=(4, 8))
        self.speed_spin.bind("<FocusOut>", lambda _: self._on_speed_changed())
        self.speed_spin.bind("<Return>", lambda _: self._on_speed_changed())

        ttk.Label(top, text="Axis:").grid(row=0, column=2, sticky="w")
        self.axis_var = tk.StringVar(value=axis_default)
        self.axis_combo = ttk.Combobox(top, values=axis_choices, textvariable=self.axis_var, state="readonly", width=3)
        self.axis_combo.grid(row=0, column=3, padx=(4, 12))

        ttk.Label(top, text="Port:").grid(row=0, column=4, sticky="w")
        self.port_var = tk.StringVar(value=cfg.default_port)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, state="readonly", width=14)
        self.port_combo.grid(row=0, column=5, padx=(4, 8))

        self.refresh_btn = ttk.Button(top, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=6, padx=(0, 8))

        self.connect_btn = ttk.Button(top, text="Connect", command=self.connect_motor)
        self.connect_btn.grid(row=0, column=7, padx=(0, 4))

        self.disconnect_btn = ttk.Button(top, text="Disconnect", command=self.disconnect_motor)
        self.disconnect_btn.grid(row=0, column=8, padx=(0, 4))

        self.home_btn = ttk.Button(top, text="Home to Zero", command=self.home_motor)
        self.home_btn.grid(row=0, column=9, padx=(8, 4))

        self.stop_btn = ttk.Button(top, text="Stop Motor", command=self.stop_motor)
        self.stop_btn.grid(row=0, column=10, padx=(4, 0))

        ttk.Label(top, text="DAQ Port:").grid(row=1, column=0, sticky="w", pady=(4, 0))
        self.daq_port_var = tk.StringVar()
        self.daq_port_combo = ttk.Combobox(top, textvariable=self.daq_port_var, state="readonly", width=14)
        self.daq_port_combo.grid(row=1, column=1, padx=(4, 8), pady=(4, 0))

        ttk.Label(top, text="DAQ Mode:").grid(row=1, column=2, sticky="w", pady=(4, 0))
        self.daq_mode_var = tk.IntVar(value=DEFAULT_MODE)
        self.daq_mode_spin = ttk.Spinbox(
            top,
            from_=0,
            to=255,
            textvariable=self.daq_mode_var,
            width=5,
            command=self._on_daq_params_changed,
        )
        self.daq_mode_spin.grid(row=1, column=3, padx=(4, 12), pady=(4, 0))
        self.daq_mode_spin.bind("<FocusOut>", lambda _: self._on_daq_params_changed())
        self.daq_mode_spin.bind("<Return>", lambda _: self._on_daq_params_changed())

        ttk.Label(top, text="DAQ K:").grid(row=1, column=4, sticky="w", pady=(4, 0))
        self.daq_k_var = tk.IntVar(value=DEFAULT_K_VALUE)
        self.daq_k_spin = ttk.Spinbox(
            top,
            from_=1,
            to=255,
            textvariable=self.daq_k_var,
            width=5,
            command=self._on_daq_params_changed,
        )
        self.daq_k_spin.grid(row=1, column=5, padx=(4, 0), pady=(4, 0))
        self.daq_k_spin.bind("<FocusOut>", lambda _: self._on_daq_params_changed())
        self.daq_k_spin.bind("<Return>", lambda _: self._on_daq_params_changed())

        ttk.Label(top, text="DAQ Baud:").grid(row=1, column=6, sticky="w", pady=(4, 0))
        self.daq_baud_var = tk.StringVar(value=str(DEFAULT_BAUDRATE))
        ttk.Entry(top, textvariable=self.daq_baud_var, width=8).grid(row=1, column=7, padx=(4, 0), pady=(4, 0))

        ttk.Label(top, text="Timeout (s):").grid(row=1, column=8, sticky="w", pady=(4, 0))
        self.daq_timeout_var = tk.StringVar(value=str(self._daq.read_timeout))
        ttk.Entry(top, textvariable=self.daq_timeout_var, width=8).grid(row=1, column=9, padx=(4, 0), pady=(4, 0))

        ttk.Label(top, text="Timer MP (s):").grid(row=1, column=10, sticky="w", pady=(4, 0))
        self.daq_timer_mp_var = tk.StringVar(value=str(DEFAULT_TIMER_MP))
        ttk.Entry(top, textvariable=self.daq_timer_mp_var, width=10).grid(row=1, column=11, padx=(4, 0), pady=(4, 0))

        ttk.Label(top, text="Calibration:").grid(row=1, column=12, sticky="w", pady=(4, 0))
        self.daq_calibration_var = tk.StringVar(value=str(DEFAULT_CALIBRATION))
        ttk.Entry(top, textvariable=self.daq_calibration_var, width=6).grid(row=1, column=13, padx=(4, 0), pady=(4, 0))

        ttk.Label(top, text="Interval (s):").grid(row=1, column=14, sticky="w", pady=(4, 0))
        self.daq_interval_var = tk.StringVar(value="0.0")
        ttk.Entry(top, textvariable=self.daq_interval_var, width=8).grid(row=1, column=15, padx=(4, 0), pady=(4, 0))

        self.daq_auto_trigger_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Auto trigger", variable=self.daq_auto_trigger_var).grid(
            row=1, column=16, sticky="w", pady=(4, 0)
        )
        self.daq_connect_btn = ttk.Button(top, text="Connect DAQ", command=self.connect_daq)
        self.daq_connect_btn.grid(row=1, column=17, padx=(4, 4), pady=(4, 0))
        self.daq_disconnect_btn = ttk.Button(
            top,
            text="Disconnect DAQ",
            command=self.disconnect_daq,
            state=tk.DISABLED,
        )
        self.daq_disconnect_btn.grid(row=1, column=18, padx=(0, 4), pady=(4, 0))

        ttk.Label(top, text="Start (mm):").grid(row=2, column=0, sticky="w", pady=(4, 0))
        self.start_pos_var = tk.StringVar(value="0.0")
        ttk.Entry(top, textvariable=self.start_pos_var, width=8).grid(row=2, column=1, padx=(4, 8), pady=(4, 0))

        ttk.Label(top, text="End (mm):").grid(row=2, column=2, sticky="w", pady=(4, 0))
        self.end_pos_var = tk.StringVar(value="10.0")
        ttk.Entry(top, textvariable=self.end_pos_var, width=8).grid(row=2, column=3, padx=(4, 8), pady=(4, 0))

        ttk.Label(top, text="Accel (mm/s^2):").grid(row=2, column=4, sticky="w", pady=(4, 0))
        self.accel_var = tk.StringVar(value="100.0")
        ttk.Entry(top, textvariable=self.accel_var, width=8).grid(row=2, column=5, padx=(4, 8), pady=(4, 0))

        ttk.Label(top, text="Coord:").grid(row=2, column=6, sticky="w", pady=(4, 0))
        self.coord_var = tk.StringVar(value="MPos")
        self.coord_combo = ttk.Combobox(
            top,
            values=("WPos", "MPos"),
            textvariable=self.coord_var,
            state="readonly",
            width=5,
        )
        self.coord_combo.grid(row=2, column=7, padx=(4, 8), pady=(4, 0))

        mode, k_val = self._daq.measurement_configuration
        self.daq_mode_var.set(mode)
        self.daq_k_var.set(k_val)

        self.connect_btn.configure(state=tk.DISABLED)
        self.disconnect_btn.configure(state=tk.DISABLED)

        top_status = ttk.Frame(self)
        top_status.grid(row=1, column=0, sticky="ew", padx=12)
        top_status.columnconfigure(0, weight=1)
        if self._daq.is_simulation:
            mode, k_val = self._daq.measurement_configuration
            default_status = f"Disconnected | Motor Idle | DAQ: Simulation (mode {mode}, k {k_val})"
        else:
            default_status = "Disconnected | Motor Idle | DAQ: Disconnected"
        self.status_var = tk.StringVar(value=default_status)
        self._last_status_line = default_status
        ttk.Label(top_status, textvariable=self.status_var, anchor="w").grid(row=0, column=0, sticky="ew")

        body_container = ttk.Frame(self)
        body_container.grid(row=2, column=0, sticky="nsew")
        body_container.rowconfigure(0, weight=1)
        body_container.columnconfigure(0, weight=1)

        self._body_canvas = tk.Canvas(body_container, highlightthickness=0)
        self._body_canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(body_container, orient="vertical", command=self._body_canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self._body_canvas.configure(yscrollcommand=scrollbar.set)

        self._body_frame = ttk.Frame(self._body_canvas)
        self._body_canvas_window = self._body_canvas.create_window((0, 0), window=self._body_frame, anchor="nw")
        self._body_frame.columnconfigure(0, weight=1)

        self._body_frame.bind("<Configure>", lambda _: self._body_canvas.configure(scrollregion=self._body_canvas.bbox("all")))
        self._body_canvas.bind(
            "<Configure>",
            lambda event: self._body_canvas.itemconfigure(self._body_canvas_window, width=event.width),
        )
        self._body_frame.bind("<Enter>", lambda _: self._bind_mousewheel(self._body_canvas))
        self._body_frame.bind("<Leave>", lambda _: self._unbind_mousewheel(self._body_canvas))

        for plot_id in (1, 2, 3):
            card = PlotCard(self._body_frame, plot_id, start_callback=self._toggle_measurement)
            card.grid(row=plot_id - 1, column=0, sticky="nsew", padx=12, pady=6)
            runtime = PlotRuntime(plot_id=plot_id, card=card, buffer=MeasurementBuffer(plot_id))
            self._plots[plot_id] = runtime

        footer = ttk.Frame(self)
        footer.grid(row=3, column=0, sticky="ew", padx=12, pady=(4, 12))
        footer.columnconfigure(0, weight=1)
        ttk.Separator(footer, orient=tk.HORIZONTAL).grid(row=0, column=0, sticky="ew", pady=(0, 8))

        self.export_btn = ttk.Button(footer, text="Save Results to Excel", command=self.export_results)
        self.export_btn.grid(row=1, column=0, sticky="e")

        self._on_daq_params_changed()
        self.refresh_ports()
        self.bind("<Visibility>", lambda _: self.after(0, self.refresh_ports))

    # ------------------------------------------------------------------ #
    # Port / motor lifecycle
    def refresh_ports(self) -> None:
        """Refresh the port list from the environment."""

        self._motor_config = load_motor_config()
        try:
            self._sync_motor_controls_from_config(self._motor_config)
        except Exception:
            LOGGER.debug("Failed to sync motor controls from config", exc_info=True)
        ports = list(list_serial_ports())
        LOGGER.debug("Available ports: %s", ports)

        motor_ports = prepare_port_list(ports, is_simulation=self._motor.is_simulation, simulated_label=SIMULATED_PORT_NAME)
        daq_ports = prepare_port_list(ports, is_simulation=self._daq.is_simulation, simulated_label="Simulated")

        preferred_port = self._motor_config.default_port or self.port_var.get()
        selected_motor = select_port_value(motor_ports, self.port_var.get(), preferred_port)
        self.port_combo.configure(values=tuple(motor_ports))
        self.port_var.set(selected_motor)

        selected_daq = select_port_value(daq_ports, self.daq_port_var.get())
        self.daq_port_combo.configure(values=tuple(daq_ports))
        self.daq_port_var.set(selected_daq)
        self._update_connection_buttons()
        self._update_daq_buttons()

    def _sync_motor_controls_from_config(self, cfg: MotorConfig) -> None:
        """Update motor controls from the shared MotorConfig (without interrupting active runs)."""
        any_running = any(runtime.mode is not None for runtime in self._plots.values())
        if any_running:
            return
        axis_choices = [axis for axis in AXES if bool(cfg.axis_enabled.get(axis, True))]
        if not axis_choices:
            axis_choices = list(AXES)
        self.axis_combo.configure(values=axis_choices)
        current_axis = (self.axis_var.get() or "").upper()
        if current_axis not in axis_choices:
            self.axis_var.set(axis_choices[0])
        feed = float(getattr(cfg, "jog_feed", 1600.0) or 1600.0)
        self.speed_var.set(max(1.0, feed))

    def connect_motor(self) -> None:
        """Connect the shared motor controller using the selected port."""
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Please select a port before connecting.")
            return
        if not self._motor.is_simulation and port.lower().startswith("simulated"):
            messagebox.showerror(
                "Invalid port",
                "Motor operates in hardware mode; select a real port before connecting.",
            )
            return
        try:
            cfg = load_motor_config()
            self._motor.connect(port, baudrate=int(getattr(cfg, "default_baud", 115200) or 115200))
        except MotorError as exc:
            LOGGER.exception("Motor connection failed")
            messagebox.showerror("Motor connection failed", f"Could not connect to {port}:\n{exc}")
            return

        self._logger.info("Motor connected on %s", port)
        self._persist_motor_config()
        self._update_connection_buttons()

    def disconnect_motor(self) -> None:
        """Disconnect the motor controller and reset UI state."""
        self._logger.info("Disconnecting motor")
        if self._active_motor_plot is not None:
            self._stop_measurement(self._active_motor_plot)
            self._active_motor_plot = None
        try:
            self._motor.stop()
        except MotorError:
            LOGGER.debug("Motor stop during disconnect raised error", exc_info=True)
        self._motor.disconnect()
        self._update_connection_buttons()

    def connect_daq(self) -> None:
        """Ensure the DAQ transport is configured and connected."""
        if self._ensure_daq_connected():
            self._update_daq_buttons()

    def disconnect_daq(self) -> None:
        """Disconnect the DAQ transport unless running in simulation."""
        self._active_daq_plot = None
        if self._daq.is_simulation:
            self._update_daq_buttons()
            return
        try:
            self._daq.disconnect()
        except DataAcquisitionError as exc:
            LOGGER.exception("DAQ disconnect failed")
            messagebox.showerror("DAQ disconnect failed", str(exc))
        self._update_daq_buttons()

    def _on_daq_params_changed(self) -> None:
        """Respond to UI edits that affect DAQ parameters."""
        self._apply_daq_params()

    def _apply_daq_params(self) -> bool:
        """Validate and forward DAQ mode/k settings to the controller."""
        mode = self._sanitize_int(self.daq_mode_var, min_value=0, max_value=255, fallback=DEFAULT_MODE)
        k_value = self._sanitize_int(self.daq_k_var, min_value=1, max_value=255, fallback=DEFAULT_K_VALUE)
        try:
            self._daq.set_measurement_parameters(mode, k_value)
        except DataAcquisitionError as exc:
            LOGGER.exception("Failed to apply DAQ parameters")
            messagebox.showerror("DAQ configuration failed", str(exc))
            return False
        return True

    def _sanitize_speed_value(self) -> float:
        """Ensure the feed value is a positive float (mm/min)."""
        try:
            value = float(self.speed_var.get())
        except (tk.TclError, ValueError):
            value = float(getattr(self._motor_config, "jog_feed", 1600.0) or 1600.0)
        return max(1.0, value)

    def _on_speed_changed(self) -> None:
        """Clamp the feed value and persist it for future sessions."""
        sanitized = self._sanitize_speed_value()
        try:
            current = float(self.speed_var.get())
        except (tk.TclError, ValueError):
            current = sanitized
        if abs(current - sanitized) > 1e-6:
            self.speed_var.set(sanitized)
        self._persist_motor_config()

    def _sanitize_int(self, var: tk.IntVar, *, min_value: int, max_value: int, fallback: int) -> int:
        """Read a Tk IntVar safely, clamping to the provided bounds."""
        try:
            value = int(var.get())
        except (tk.TclError, ValueError):
            value = fallback
        if value < min_value or value > max_value:
            value = max(min_value, min(max_value, value))
            var.set(value)
        return value

    def _get_entry_int(self, var: tk.StringVar, fallback: int, *, min_value: Optional[int] = None) -> int:
        """Parse an integer Entry field, applying fallbacks/clamps."""
        try:
            value = int(var.get())
        except (ValueError, TypeError):
            value = fallback
        if min_value is not None and value < min_value:
            value = min_value
        var.set(str(value))
        return value

    def _get_entry_float(self, var: tk.StringVar, fallback: float, *, min_value: Optional[float] = None) -> float:
        """Parse a float Entry field, applying fallbacks/clamps."""
        try:
            value = float(var.get())
        except (ValueError, TypeError):
            value = fallback
        if min_value is not None and value < min_value:
            value = min_value
        var.set(f"{value}")
        return value

    def _capture_motor_config(self, cfg: MotorConfig) -> MotorConfig:
        """Copy the current UI values into a MotorConfig instance."""
        cfg.jog_feed = self._sanitize_speed_value()
        # Backward-compat value used by older UIs.
        try:
            cfg.default_speed = int(round(cfg.jog_feed))
        except Exception:
            pass

        axis = (self.axis_var.get() or cfg.default_axis).upper()
        if axis not in AXES:
            axis = AXES[0]
        cfg.default_axis = axis

        port = (self.port_var.get() or "").strip()
        if port.lower().startswith("simulated"):
            port = SIMULATED_PORT_NAME
        cfg.default_port = port
        return cfg

    def _persist_motor_config(self) -> None:
        """Write the latest motor defaults to disk."""
        cfg = self._capture_motor_config(self._motor_config)
        self._motor_config = cfg
        with self._config_save_lock:
            self._config_save_pending = cfg
            if self._config_save_running:
                return
            self._config_save_running = True

        def worker() -> None:
            while True:
                with self._config_save_lock:
                    pending = self._config_save_pending
                    self._config_save_pending = None
                    if pending is None:
                        self._config_save_running = False
                        return
                try:
                    save_motor_config(pending)
                except Exception:
                    LOGGER.exception("Failed to save motor config", exc_info=True)

        threading.Thread(target=worker, daemon=True).start()

    def _get_combo_values(self, combo: ttk.Combobox) -> list[str]:
        """Return the list backing a readonly combobox."""
        raw = combo.cget("values")
        if isinstance(raw, (tuple, list)):
            return list(raw)
        if isinstance(raw, str):
            return [raw] if raw else []
        return []

    def _update_connection_buttons(self) -> None:
        """Enable/disable connect buttons based on availability."""
        motor_connected = self._motor.is_connected()
        motor_ports_available = bool(self._get_combo_values(self.port_combo))
        if self._motor.is_simulation and not motor_ports_available:
            motor_ports_available = True

        if motor_connected:
            self.connect_btn.configure(state=tk.DISABLED)
            self.disconnect_btn.configure(state=tk.NORMAL)
        else:
            self.connect_btn.configure(state=tk.NORMAL if motor_ports_available else tk.DISABLED)
            self.disconnect_btn.configure(state=tk.DISABLED)

    def _update_daq_buttons(self) -> None:
        """Enable/disable DAQ connect buttons based on availability."""

        if not hasattr(self, "daq_connect_btn"):
            return
        if self._daq.is_simulation:
            self.daq_connect_btn.configure(state=tk.DISABLED)
            self.daq_disconnect_btn.configure(state=tk.DISABLED)
            return
        connected = self._daq.is_connected()
        self.daq_connect_btn.configure(state=tk.DISABLED if connected else tk.NORMAL)
        self.daq_disconnect_btn.configure(state=tk.NORMAL if connected else tk.DISABLED)


    def _ensure_daq_connected(self) -> bool:
        """Validate DAQ settings and connect if needed."""
        baud = self._get_entry_int(self.daq_baud_var, DEFAULT_BAUDRATE, min_value=1200)
        timeout = self._get_entry_float(self.daq_timeout_var, 3.0, min_value=0.1)
        timer_mp = self._get_entry_float(self.daq_timer_mp_var, DEFAULT_TIMER_MP, min_value=1e-6)
        calibration = self._get_entry_int(self.daq_calibration_var, DEFAULT_CALIBRATION, min_value=0)
        interval = self._get_entry_float(self.daq_interval_var, 0.0, min_value=0.0)
        auto_trigger = bool(self.daq_auto_trigger_var.get())

        try:
            self._daq.set_transport_parameters(
                baudrate=baud,
                read_timeout=timeout,
                calibration=calibration,
                timer_mp=timer_mp,
            )
        except DataAcquisitionError as exc:
            LOGGER.exception("Failed to update DAQ parameters")
            messagebox.showerror("DAQ configuration failed", str(exc))
            return False

        if self._daq.is_simulation:
            if not self._daq.is_connected():
                try:
                    self._daq.connect(self.daq_port_var.get() or "Simulated")
                except DataAcquisitionError as exc:
                    LOGGER.exception("DAQ connection failed")
                    messagebox.showerror("DAQ connection failed", str(exc))
                    return False
        else:
            port = self.daq_port_var.get()
            if not port:
                messagebox.showwarning("No DAQ port", "Please select a DAQ port before starting a measurement.")
                return False
            if not self._daq.is_connected():
                try:
                    self._daq.connect(port)
                except DataAcquisitionError as exc:
                    LOGGER.exception("DAQ connection failed")
                    messagebox.showerror("DAQ connection failed", f"Could not connect to {port}:\n{exc}")
                    return False
                self._logger.info("DAQ connected on %s", port)

        if not self._apply_daq_params():
            return False

        self._daq_interval_sec = interval
        self._daq_auto_trigger = auto_trigger
        self._update_daq_buttons()
        return True
    def home_motor(self) -> None:
        """Home the selected motor axis back to zero."""
        axis = self.axis_var.get()
        if not self._motor.is_connected():
            messagebox.showwarning("Motor disconnected", "Connect the motor before homing.")
            return
        try:
            self._motor.home(axis)
        except MotorError as exc:
            LOGGER.exception("Failed to home motor")
            messagebox.showerror("Homing failed", str(exc))

    def stop_motor(self) -> None:
        """Stop the motor and terminate any active measurement."""
        self._logger.info("Stop motor requested by user")
        self._motor.stop()
        if self._active_motor_plot is not None:
            self._stop_measurement(self._active_motor_plot)
            self._active_motor_plot = None

    # ------------------------------------------------------------------ #
    # Measurement orchestration
    def _toggle_measurement(self, plot_id: PlotIdLiteral, mode: ModeLiteral) -> None:
        """Start or stop acquisition for the requested plot/mode."""
        runtime = self._plots[plot_id]
        if runtime.mode == mode:
            self._stop_measurement(plot_id)
            return
        if runtime.mode is not None:
            self._stop_measurement(plot_id)
        if not self._ensure_daq_connected():
            return
        if not self._daq.is_simulation and self._active_daq_plot not in (None, plot_id):
            messagebox.showwarning(
                "DAQ busy",
                "Another plot is currently acquiring data. Stop it before starting a new measurement.",
            )
            return
        self._start_measurement(plot_id, mode)

    def _start_measurement(self, plot_id: PlotIdLiteral, mode: ModeLiteral) -> None:
        """Spawn the acquisition thread for the given plot/mode."""
        try:
            self._motor_config = load_motor_config()
            self._sync_motor_controls_from_config(self._motor_config)
        except Exception:
            LOGGER.debug("Failed to sync motor controls before measurement start", exc_info=True)
        axis = self.axis_var.get()
        if mode != "no_motor_time":
            if not self._motor.is_connected():
                messagebox.showwarning("Motor disconnected", "Please connect the motor before starting this mode.")
                return
            if self._active_motor_plot not in (None, plot_id):
                messagebox.showwarning(
                    "Motor busy",
                    "Another plot is already using the motor. Stop it before starting a new motor-driven measurement.",
                )
                return

        axis = (self.axis_var.get() or "").upper()
        if axis not in AXES:
            axis = AXES[0]
        speed = self._sanitize_speed_value()
        start_mm = 0.0
        end_mm = 0.0
        accel = 0.0
        coord_system = "work"
        if mode != "no_motor_time":
            start_mm, end_mm, accel, coord_system = self._read_sweep_settings()
            if mode == "pos_from_zero" and end_mm < start_mm:
                messagebox.showwarning(
                    "Invalid sweep range",
                    "End position must be greater than or equal to start for + direction.",
                )
                return
            if mode == "neg_from_zero" and end_mm > start_mm:
                messagebox.showwarning(
                    "Invalid sweep range",
                    "End position must be less than or equal to start for - direction.",
                )
                return

        runtime = self._plots[plot_id]
        runtime.buffer.clear()
        runtime.card.reset_plot(mode)
        runtime.card.set_running(mode)
        runtime.mode = mode
        runtime.stop_event = threading.Event()
        runtime.sweep_started_event = threading.Event() if mode != "no_motor_time" else None
        runtime.stop_motor_on_stop = True
        runtime.stop_reason = None
        now = time.perf_counter()
        runtime.start_monotonic = now
        runtime.start_time = datetime.now()
        runtime.last_measurement_finished_at = now
        runtime.next_measurement_not_before = now
        runtime.last_coordinate_mm = start_mm if mode != "no_motor_time" else 0.0
        runtime.sweep_start_mm = start_mm
        runtime.sweep_end_mm = end_mm
        runtime.coord_system = coord_system
        runtime.motor_thread = None

        thread = threading.Thread(
            target=self._acquisition_loop,
            name=f"Acquisition-{plot_id}",
            args=(runtime, axis, mode),
            daemon=True,
        )
        runtime.thread = thread
        thread.start()

        if mode in {"pos_from_zero", "neg_from_zero"}:
            motor_thread = threading.Thread(
                target=self._motor_sweep,
                name=f"MotorSweep-{plot_id}",
                args=(runtime, axis, start_mm, end_mm, speed, accel, coord_system),
                daemon=True,
            )
            runtime.motor_thread = motor_thread
            motor_thread.start()

        if mode != "no_motor_time":
            self._active_motor_plot = plot_id
        if not self._daq.is_simulation:
            self._active_daq_plot = plot_id

    def _coord_system(self) -> str:
        value = (self.coord_var.get() or "MPos").strip().upper()
        return "machine" if value.startswith("M") else "work"

    def _read_sweep_settings(self) -> Tuple[float, float, float, str]:
        start_mm = self._get_entry_float(self.start_pos_var, 0.0)
        end_mm = self._get_entry_float(self.end_pos_var, 0.0)
        accel = self._get_entry_float(self.accel_var, 0.0, min_value=0.0)
        coord_system = self._coord_system()
        return start_mm, end_mm, accel, coord_system

    def _wait_for_sweep_start(self, runtime: PlotRuntime, stop_event: threading.Event) -> bool:
        event = runtime.sweep_started_event
        if event is None:
            return True
        while not stop_event.is_set():
            if event.wait(SWEEP_POLL_INTERVAL):
                return True
        return False

    def _read_axis_position(self, axis: str, coord_system: str) -> Optional[float]:
        use_machine = coord_system == "machine"
        try:
            return self._motor.read_status_position(axis, use_machine=use_machine, timeout=STATUS_QUERY_TIMEOUT)
        except MotorError:
            return None

    def _has_reached_end(self, runtime: PlotRuntime, coordinate_mm: float) -> bool:
        start_mm = runtime.sweep_start_mm
        end_mm = runtime.sweep_end_mm
        if end_mm >= start_mm:
            return coordinate_mm >= (end_mm - SWEEP_POSITION_TOLERANCE)
        return coordinate_mm <= (end_mm + SWEEP_POSITION_TOLERANCE)

    def _build_move_command(self, axis: str, target_mm: float, speed: float, coord_system: str) -> str:
        prefix = "G53 " if coord_system == "machine" else ""
        return f"{prefix}G1 {axis}{target_mm:.3f} F{speed:.1f}"

    def _apply_axis_settings(self, axis: str, *, speed: float, accel: float) -> None:
        axis = axis.upper()
        speed_setting = {"X": "$110", "Y": "$111"}.get(axis)
        accel_setting = {"X": "$120", "Y": "$121"}.get(axis)
        if accel_setting and accel > 0:
            self._motor.send_line(f"{accel_setting}={accel:.3f}")
        if speed_setting:
            self._motor.send_line(f"{speed_setting}={speed:.1f}")

    def _estimate_move_timeout(self, current: float, target: float, speed: float) -> float:
        if speed <= 0:
            return 5.0
        distance = abs(target - current)
        seconds = distance / (speed / 60.0) if speed > 0 else 0.0
        return max(2.0, min(120.0, seconds * 2.0 + 2.0))

    def _wait_for_target_position(
        self,
        runtime: PlotRuntime,
        axis: str,
        target_mm: float,
        coord_system: str,
        speed: float,
        stop_event: threading.Event,
    ) -> bool:
        current = self._read_axis_position(axis, coord_system)
        if current is None:
            for _ in range(3):
                if stop_event.wait(SWEEP_POLL_INTERVAL):
                    return False
                current = self._read_axis_position(axis, coord_system)
                if current is not None:
                    break
        if current is None:
            coord_label = "MPos" if coord_system == "machine" else "WPos"
            runtime.stop_reason = "status_missing"
            self._report_error(
                f"GRBL status does not include {coord_label}. "
                "Enable it in $10 or switch the coordinate selector."
            )
            return False
        timeout = self._estimate_move_timeout(current, target_mm, speed)
        deadline = time.monotonic() + timeout
        while not stop_event.is_set() and time.monotonic() < deadline:
            pos = self._read_axis_position(axis, coord_system)
            if pos is not None:
                runtime.last_coordinate_mm = pos
                if abs(pos - target_mm) <= SWEEP_POSITION_TOLERANCE:
                    return True
            stop_event.wait(SWEEP_POLL_INTERVAL)
        return False

    def _simulate_axis_move(
        self,
        axis: str,
        start_mm: float,
        end_mm: float,
        speed: float,
        stop_event: threading.Event,
    ) -> bool:
        delta = end_mm - start_mm
        if abs(delta) <= SWEEP_POSITION_TOLERANCE:
            self._motor.set_position(axis, end_mm)
            return True
        mm_per_sec = max(speed, 1.0) / 60.0
        duration = abs(delta) / mm_per_sec if mm_per_sec > 0 else 0.0
        tick = max(SWEEP_POLL_INTERVAL, 0.02)
        steps = max(1, int(duration / tick))
        for idx in range(steps):
            if stop_event.is_set():
                return False
            frac = (idx + 1) / steps
            pos = start_mm + (delta * frac)
            self._motor.set_position(axis, pos)
            stop_event.wait(tick)
        self._motor.set_position(axis, end_mm)
        return True

    def _motor_sweep(
        self,
        runtime: PlotRuntime,
        axis: str,
        start_mm: float,
        end_mm: float,
        speed: float,
        accel: float,
        coord_system: str,
    ) -> None:
        stop_event = runtime.stop_event
        if stop_event is None:
            return
        if not self._motor.is_connected():
            stop_event.set()
            return
        try:
            self._apply_axis_settings(axis, speed=speed, accel=accel)
        except MotorError as exc:
            self._report_error(f"Failed to apply axis settings: {exc}")
            stop_event.set()
            return

        if self._motor.is_simulation:
            current = runtime.last_coordinate_mm
            if not self._simulate_axis_move(axis, current, start_mm, speed, stop_event):
                return
            runtime.last_coordinate_mm = start_mm
            if stop_event.is_set():
                return
            runtime.start_monotonic = time.perf_counter()
            runtime.start_time = datetime.now()
            runtime.last_measurement_finished_at = time.perf_counter()
            if runtime.sweep_started_event:
                runtime.sweep_started_event.set()
            if not self._simulate_axis_move(axis, start_mm, end_mm, speed, stop_event):
                return
            return

        try:
            self._motor.send_line("G21")
            self._motor.send_line("G90")
        except MotorError as exc:
            self._report_error(f"Failed to set motion mode: {exc}")
            stop_event.set()
            return

        try:
            self._motor.send_line(self._build_move_command(axis, start_mm, speed, coord_system))
        except MotorError as exc:
            self._report_error(f"Failed to move to start: {exc}")
            stop_event.set()
            return

        if not self._wait_for_target_position(runtime, axis, start_mm, coord_system, speed, stop_event):
            if not stop_event.is_set():
                if runtime.stop_reason != "status_missing":
                    self._report_error("Timed out waiting for start position.")
            stop_event.set()
            return

        if stop_event.is_set():
            return

        try:
            self._motor.send_line(self._build_move_command(axis, end_mm, speed, coord_system))
        except MotorError as exc:
            self._report_error(f"Failed to start sweep: {exc}")
            stop_event.set()
            return

        runtime.start_monotonic = time.perf_counter()
        runtime.start_time = datetime.now()
        runtime.last_measurement_finished_at = time.perf_counter()
        if runtime.sweep_started_event:
            runtime.sweep_started_event.set()

    def _stop_measurement(self, plot_id: PlotIdLiteral, *, from_thread: bool = False) -> None:
        """Stop acquisition for a plot and reset its UI state."""
        runtime = self._plots[plot_id]
        previous_mode = runtime.mode
        self._logger.info("Stop requested for plot %s (mode=%s)", plot_id, previous_mode)
        stop_event = runtime.stop_event
        if stop_event is not None:
            stop_event.set()
        iterator = runtime.freq_iter
        runtime.freq_iter = None
        if iterator is not None:
            try:
                close = getattr(iterator, "close", None)
                if callable(close):
                    close()
            except Exception:
                LOGGER.debug("Error while closing DAQ iterator", exc_info=True)
        thread = runtime.thread
        if thread and thread.is_alive() and not from_thread:
            thread.join(timeout=2.0)
        motor_thread = runtime.motor_thread
        if motor_thread and motor_thread.is_alive() and not from_thread:
            motor_thread.join(timeout=2.0)
        stop_motor = runtime.stop_motor_on_stop
        if previous_mode in {"pos_from_zero", "neg_from_zero"} and self._motor.is_connected() and stop_motor:
            try:
                self._logger.debug("Issuing realtime jog cancel")
                cancel = getattr(self._motor, "send_raw", None)
                if callable(cancel):
                    cancel(b"\x85")
            except Exception:
                self._logger.exception("Failed to send realtime cancel during stop")
            try:
                self._motor.stop()
            except MotorError as exc:
                LOGGER.warning("Failed to stop motor during measurement shutdown: %s", exc)
        runtime.thread = None
        runtime.motor_thread = None
        runtime.stop_event = None
        runtime.sweep_started_event = None
        runtime.stop_motor_on_stop = True
        runtime.stop_reason = None
        runtime.mode = None
        runtime.card.set_running(None)
        if self._active_motor_plot == plot_id:
            self._active_motor_plot = None
        if self._active_daq_plot == plot_id:
            self._active_daq_plot = None

    def _acquisition_loop(
        self,
        runtime: PlotRuntime,
        axis: str,
        mode: ModeLiteral,
    ) -> None:
        """Background loop that streams data and updates the UI."""
        stop_event = runtime.stop_event
        if stop_event is None:
            return
        freq_iter: Optional[Iterator[Tuple[float, float]]] = None
        use_iter = mode == "no_motor_time"
        try:
            interval = max(0.0, self._daq_interval_sec)
            auto_trigger = bool(self._daq_auto_trigger)
            if mode != "no_motor_time":
                if not self._wait_for_sweep_start(runtime, stop_event):
                    return
                runtime.last_measurement_finished_at = time.perf_counter()
            def _read_single() -> Optional[Tuple[float, float]]:
                measurement_interval = interval
                if self._daq.is_simulation:
                    measurement_interval = max(measurement_interval, SAMPLING_INTERVAL)
                ready_at = runtime.last_measurement_finished_at + measurement_interval
                delay = ready_at - time.perf_counter()
                if delay > 0:
                    if stop_event.wait(delay):
                        return None
                return self._daq.read_frequencies()

            if use_iter:
                if self._daq.is_simulation:
                    freq_iter = self._daq.iter_frequencies(interval=max(SAMPLING_INTERVAL, interval))
                else:
                    freq_iter = self._daq.iter_frequencies(
                        timeout=self._daq.read_timeout,
                        interval=interval,
                        auto_trigger=auto_trigger,
                    )
                runtime.freq_iter = freq_iter
                freq_source = freq_iter
                with closing(freq_source):  # type: ignore[arg-type]
                    iterator = iter(freq_source)
                    while not stop_event.is_set():
                        try:
                            ch1, ch2 = next(iterator)
                        except StopIteration:
                            break
                        if stop_event.is_set():
                            break
                        timestamp = datetime.now()
                        coordinate = 0.0
                        elapsed = time.perf_counter() - runtime.start_monotonic
                        x_value = elapsed
                        self._record_measurement(runtime, axis, mode, ch1, ch2, coordinate, x_value, elapsed, timestamp)
            else:
                while not stop_event.is_set():
                    try:
                        sample = _read_single()
                    except DataAcquisitionError as exc:
                        self._report_error(f"DAQ read failed: {exc}")
                        break
                    if sample is None:
                        break
                    ch1, ch2 = sample
                    runtime.last_measurement_finished_at = time.perf_counter()
                    if stop_event.is_set():
                        break
                    timestamp = datetime.now()
                    coordinate_mm = runtime.last_coordinate_mm
                    if mode != "no_motor_time" and self._motor.is_connected():
                        pos = self._read_axis_position(axis, runtime.coord_system)
                        if pos is not None:
                            coordinate_mm = pos
                            runtime.last_coordinate_mm = pos

                    elapsed = time.perf_counter() - runtime.start_monotonic
                    if mode == "no_motor_time":
                        x_value = elapsed
                        coordinate = 0.0
                    else:
                        coordinate = self._mm_to_mm(coordinate_mm)
                        x_value = coordinate
                    self._record_measurement(runtime, axis, mode, ch1, ch2, coordinate, x_value, elapsed, timestamp)
                    if mode != "no_motor_time" and self._has_reached_end(runtime, coordinate_mm):
                        runtime.stop_reason = "end_reached"
                        runtime.stop_motor_on_stop = False
                        stop_event.set()
                        break
        except MotorError as exc:
            # Motor errors should not prevent DAQ-only capture.
            self._logger.warning("Motor error during acquisition; continuing: %s", exc)
        finally:
            if freq_iter is not None:
                try:
                    freq_iter.close()  # type: ignore[union-attr]
                except Exception:
                    pass
            runtime.freq_iter = None
            if mode != "no_motor_time" and runtime.stop_motor_on_stop:
                self._motor.stop()
            self.after(0, lambda pid=runtime.plot_id: self._stop_measurement(pid, from_thread=True))

    def _record_measurement(
        self,
        runtime: PlotRuntime,
        axis: str,
        mode: ModeLiteral,
        ch1: float,
        ch2: float,
        coordinate: float,
        x_value: float,
        elapsed_s: float,
        timestamp: datetime,
    ) -> None:
        elapsed_ms = int(round(elapsed_s * 1000.0))

        self._logger.debug(
            "Measurement recorded: plot=%s coord=%.3f elapsed=%.3fs (%d ms) ch1=%.2f ch2=%.2f mode=%s",
            runtime.plot_id,
            coordinate,
            elapsed_s,
            elapsed_ms,
            ch1,
            ch2,
            mode,
        )

        row = MeasurementRow(
            timestamp=timestamp,
            elapsed_s=elapsed_s,
            elapsed_ms=elapsed_ms,
            axis=axis if axis in AXES else "X",
            coordinate=coordinate,
            ch1_freq=ch1,
            ch2_freq=ch2,
            mode=mode,
            plot_id=runtime.plot_id,
        )
        runtime.buffer.append(row)

        callback = self._make_plot_update(
            plot_id=runtime.plot_id,
            x_value=x_value,
            coordinate=coordinate,
            ch1=ch1,
            ch2=ch2,
            mode=mode,
        )
        self.after(0, callback)

    def _update_plot(
        self,
        plot_id: PlotIdLiteral,
        x_value: float,
        coordinate: float,
        ch1: float,
        ch2: float,
        mode: ModeLiteral,
    ) -> None:
        """Append a new point to the UI plot and status label."""

        runtime = self._plots[plot_id]
        points = len(runtime.buffer)
        runtime.card.append_point(x_value, ch1, ch2, mode=mode)
        runtime.card.update_status(points=points, coordinate=coordinate, ch1=ch1, ch2=ch2)

    def _make_plot_update(
        self,
        *,
        plot_id: PlotIdLiteral,
        x_value: float,
        coordinate: float,
        ch1: float,
        ch2: float,
        mode: ModeLiteral,
    ) -> Callable[[], None]:
        """Return a closure used by the acquisition loop to update plots."""

        return lambda: self._update_plot(plot_id, x_value, coordinate, ch1, ch2, mode)

    def _report_error(self, message: str) -> None:
        """Log and surface acquisition errors to the operator."""
        LOGGER.error("%s", message)
        self.after(0, lambda msg=message: messagebox.showerror("Measurement error", msg))

    # ------------------------------------------------------------------ #
    # Status polling
    def _poll_status(self) -> None:
        """Refresh the footer status summarising motor/DAQ state."""
        snapshot: MotorState = self._motor.snapshot()
        connected = snapshot.connected
        if connected != self._last_motor_connected:
            self._last_motor_connected = connected
            self._update_connection_buttons()
        parts = []
        if snapshot.connected:
            if snapshot.port:
                parts.append(f"Connected ({snapshot.port})")
            else:
                parts.append("Connected")
        else:
            parts.append("Disconnected")
        parts.append("Motor Running" if snapshot.moving else "Motor Idle")
        if snapshot.last_error:
            parts.append(f"Error: {snapshot.last_error}")
        if self._daq.is_simulation:
            mode, k_value = self._daq.measurement_configuration
            daq_txt = f"DAQ: Simulation (mode {mode}, k {k_value})"
        elif self._daq.is_connected():
            daq_port = self._daq.connected_port or "Connected"
            mode, k_value = self._daq.measurement_configuration
            gap = self._daq.measurement_gap
            daq_txt = f"DAQ: {daq_port} (mode {mode}, k {k_value}, gap {gap:.2f}s)"
        else:
            daq_txt = "DAQ: Disconnected"
        parts.append(daq_txt)
        composed = " | ".join(parts)
        if composed != getattr(self, "_last_status_line", None):
            self.status_var.set(composed)
            self._last_status_line = composed
        self._status_poll_id = self.after(500, self._poll_status)

    # ------------------------------------------------------------------ #
    # Export helpers
    def export_results(self) -> None:
        """Persist buffered measurements to an Excel workbook."""
        buffers = {plot_id: runtime.buffer for plot_id, runtime in self._plots.items()}
        total_points = sum(len(runtime.buffer) for runtime in self._plots.values())
        if total_points == 0:
            messagebox.showinfo("No data", "There are no measurements to export yet.")
            return

        filename = filedialog.asksaveasfilename(
            title="Export measurements",
            defaultextension=".xlsx",
            filetypes=[("Excel files", "*.xlsx"), ("All files", "*.*")],
        )
        if not filename:
            return
        try:
            destination = export_measurements_to_excel(buffers, filename)
        except Exception as exc:
            LOGGER.exception("Export failed")
            messagebox.showerror("Export failed", f"Could not export measurements:\n{exc}")
            return

        messagebox.showinfo("Export complete", f"Saved {total_points} samples to:\n{destination}")

    # ------------------------------------------------------------------ #
    # Shutdown logic
    def shutdown(self) -> None:
        """Gracefully stop acquisition threads and release hardware."""
        self._logger.info("Shutting down measurements tab")
        try:
            self._persist_motor_config()
        except Exception:
            self._logger.exception("Failed to persist motor config during shutdown")
        for plot_id in list(self._plots.keys()):
            self._stop_measurement(plot_id)
        self._motor.stop()
        if self._status_poll_id is not None:
            try:
                self.after_cancel(self._status_poll_id)
            except Exception:
                LOGGER.debug("Status poll already cancelled")
            self._status_poll_id = None
        self._motor.disconnect()
        try:
            self._daq.disconnect()
        except DataAcquisitionError:
            LOGGER.debug("DAQ disconnect during shutdown raised error", exc_info=True)

    # ------------------------------------------------------------------ #
    # Mousewheel helpers
    def _bind_mousewheel(self, widget: tk.Widget) -> None:
        """Attach mousewheel scrolling to a canvas widget."""
        widget.bind_all("<MouseWheel>", self._on_mousewheel)
        widget.bind_all("<Button-4>", self._on_mousewheel)
        widget.bind_all("<Button-5>", self._on_mousewheel)

    def _unbind_mousewheel(self, widget: tk.Widget) -> None:
        """Detach mousewheel scrolling from a canvas widget."""
        widget.unbind_all("<MouseWheel>")
        widget.unbind_all("<Button-4>")
        widget.unbind_all("<Button-5>")

    def _on_mousewheel(self, event: tk.Event) -> None:
        """Scroll the body canvas vertically on mousewheel events."""
        num = getattr(event, "num", 0)
        delta = getattr(event, "delta", 0)
        if num == 4 or delta > 0:
            self._body_canvas.yview_scroll(-1, "units")
        else:
            self._body_canvas.yview_scroll(1, "units")


__all__ = ["MeasurementsTab"]
