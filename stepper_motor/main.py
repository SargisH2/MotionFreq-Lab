import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import re
import logging
import queue
from collections import deque
from typing import Callable, Optional

from config.env import env_bool
from config.motor import MotorConfig, load_motor_config, save_motor_config
from ui.motor_shared import select_port_value

from hardware.motor_controller import MotorController
from hardware.motor_driver import list_serial_ports as motor_list_serial_ports
from hardware.motor_types import MotorError

ALARM_PREFIX = "ALARM"
OK_PREFIX = "ok"
STATUS_RE = re.compile(r"<(?P<state>[^|>]+)(?:\|[^>]*)?>")
PN_RE = re.compile(r"Pn:([A-Z]+)")
LOGGER = logging.getLogger("stepper.panel")
USE_MOTOR_BACKEND_DEFAULT = env_bool("STEPPER_USE_MOTOR_BACKEND", True)
PERSIST_ON_FOCUS_OUT = env_bool("STEPPER_PERSIST_ON_FOCUS_OUT", False)

def now_ms():
    return int(time.time() * 1000)

class GRBLInterface:
    """Shared functionality for any Tk widget that talks to a GRBL controller."""
    def __init__(
        self,
        master,
        *,
        set_title: bool = True,
        motor: Optional["MotorController"] = None,
        motor_config: Optional[MotorConfig] = None,
        enable_motor_backend: Optional[bool] = None,
    ):
        LOGGER.info("Initialising GRBLInterface (set_title=%s, master=%s)", set_title, type(master).__name__)
        self.master = master
        self._ui_thread = threading.current_thread()
        self.ser = None
        self.reader_thread = None
        self.reader_alive = threading.Event()
        # Queues & locks
        self.line_queue = deque(maxlen=2000)   # all raw lines from GRBL
        self.tail_queue = deque(maxlen=50)     # short tail for quick pattern checks
        self.ui_log_queue = deque(maxlen=2000) # messages to append to GUI
        self.line_lock = threading.Lock()
        self.status_lock = threading.Lock()
        self.last_status = ""
        self._last_wco = None
        self._last_mpos = None
        self._last_wpos = None
        self.alarm_event = threading.Event()
        self.stop_event = threading.Event()
        self.original_status_mask = None  # $10 original value to restore (if changed)
        self._status_poll_stop = threading.Event()
        self._status_poll_thread = None
        self._status_poll_interval = 0.1
        self._homing_active = False
        self._motor_task_queue: "queue.Queue[tuple[Callable[..., None], tuple, dict, str]]" = queue.Queue(maxsize=20)
        self._motor_task_thread = None
        self._motor_task_stop = threading.Event()
        self.machine_state = tk.StringVar(value="Disconnected")
        self._motor_config: MotorConfig = motor_config or load_motor_config()
        enable_backend = USE_MOTOR_BACKEND_DEFAULT if enable_motor_backend is None else bool(enable_motor_backend)
        backend_motor = motor
        if backend_motor is None and enable_backend and MotorController is not None:
            try:
                backend_motor = MotorController()
                LOGGER.info("Created dedicated MotorController backend for panel")
            except Exception:
                LOGGER.exception("Failed to instantiate MotorController; falling back to direct serial")
                backend_motor = None
        elif backend_motor is None and not enable_backend:
            LOGGER.info("Motor backend explicitly disabled; using direct serial")
        self._motor: Optional["MotorController"] = backend_motor
        self._motor_unregister: Optional[Callable[[], None]] = None
        self._last_state_text: Optional[str] = None
        self._use_motor_backend = self._motor is not None and MotorController is not None
        if self._use_motor_backend:
            try:
                self._motor_unregister = self._motor.register_line_listener(self._handle_motor_line)
            except Exception:
                LOGGER.exception("Failed to register motor line listener")

        # Controller safety settings we want enforced after connection/recovery.
        self._pending_hard_limits_enable = False
        self._pending_hard_limits_last_attempt = 0.0
        self._suspend_hard_limits_enforcement = False
        self._persist_after_id: Optional[str] = None

        # cache for $$ parsing
        self.settings = {}

        self.home_x_mpos = None
        self.home_y_mpos = None
        # Per-axis last captured machine positions at edges
        self.last_home_mpos = {}
        self.last_finish_mpos = {}
        # Last computed midpoint in WCS (per axis), e.g., X mid as absolute G90 coordinate
        self.last_mid_wcs = {}
        # Span measurement (commanded distance) between +release and -release
        self.measure_active = False
        self.measure_axis = None
        self.measure_accum = 0.0
        self.last_span = {}

        cfg = self._motor_config

        # Steps/mm fallback for measurement flows (UI distances are entered in mm)
        self.default_steps_per_mm = 1.0
        self._steps_warned: set[str] = set()

        # === Top connection row ===
        top = ttk.Frame(master)
        top.grid(row=0, column=0, sticky="ew", padx=8, pady=(8, 4))
        top.columnconfigure(8, weight=1)

        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w")
        initial_ports = self._list_ports()
        self.port_var = tk.StringVar(value=cfg.default_port)
        self.port_cmb = ttk.Combobox(top, width=16, state="readonly", textvariable=self.port_var, values=initial_ports)
        self._apply_default_port_selection(force=True, ports=list(initial_ports))
        self.port_cmb.grid(row=0, column=1, padx=4)

        ttk.Label(top, text="Baud:").grid(row=0, column=2, sticky="w")
        self.baud_entry = ttk.Entry(top, width=8)
        self.baud_entry.insert(0, str(cfg.default_baud))
        self.baud_entry.grid(row=0, column=3, padx=4)
        self._bind_persist(self.baud_entry)

        ttk.Button(top, text="Connect", command=self.connect).grid(row=0, column=4, padx=4)
        ttk.Button(top, text="Disconnect", command=self.disconnect).grid(row=0, column=5, padx=4)
        ttk.Button(top, text="Reconnect", command=self.reconnect).grid(row=0, column=6, padx=4)
        ttk.Button(top, text="Refresh Ports", command=self.refresh_ports).grid(row=0, column=7, padx=4)
        ttk.Label(top, textvariable=self.machine_state, foreground="#555").grid(row=0, column=8, sticky="w")

        # === Mid controls container ===
        mid = ttk.Frame(master)
        mid.grid(row=1, column=0, sticky="ew", padx=8, pady=4)
        for c in range(4):
            mid.columnconfigure(c, weight=1)

        # Axis options & homing params
        axes = ttk.LabelFrame(mid, text="Axes / Homing Params")
        axes.grid(row=0, column=0, sticky="nsew", padx=(0,8))
        self.x_en = tk.BooleanVar(value=cfg.axis_enabled.get("X", True))
        self.y_en = tk.BooleanVar(value=cfg.axis_enabled.get("Y", False))
        ttk.Checkbutton(axes, text="X", variable=self.x_en, command=self._schedule_persist_motor_config).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(axes, text="Y", variable=self.y_en, command=self._schedule_persist_motor_config).grid(row=0, column=1, sticky="w")

        ttk.Label(axes, text="Homing Feed (mm/min):").grid(row=1, column=0, sticky="w")
        self.feed_entry = ttk.Entry(axes, width=10)
        self.feed_entry.insert(0, format(cfg.homing_feed, "g"))
        self.feed_entry.grid(row=1, column=1, sticky="w")
        self._bind_persist(self.feed_entry)

        ttk.Label(axes, text="Clear (mm):").grid(row=2, column=0, sticky="w")
        self.clear_entry = ttk.Entry(axes, width=10)
        self.clear_entry.insert(0, format(cfg.homing_clearance, "g"))
        self.clear_entry.grid(row=2, column=1, sticky="w")
        self._bind_persist(self.clear_entry)

        ttk.Label(axes, text="Long jog (mm):").grid(row=3, column=0, sticky="w")
        self.long_entry = ttk.Entry(axes, width=10)
        self.long_entry.insert(0, format(cfg.homing_long_jog, "g"))
        self.long_entry.grid(row=3, column=1, sticky="w")
        self._bind_persist(self.long_entry)

        # Quick command buttons
        quick = ttk.LabelFrame(mid, text="Quick Commands")
        quick.grid(row=0, column=1, sticky="nsew", padx=(0,8))
        ttk.Button(quick, text="Unlock ($X)", command=lambda: self.safe_send("$X")).grid(row=0, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(quick, text="Settings ($$)", command=self.read_settings).grid(row=1, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(quick, text="Absolute (G90)", command=lambda: self.safe_send("G90")).grid(row=2, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(quick, text="Relative (G91)", command=lambda: self.safe_send("G91")).grid(row=3, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(quick, text="Status (?)", command=self.query_status).grid(row=4, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(quick, text="Soft Reset (Ctrl-X)", command=self.soft_reset).grid(row=5, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(quick, text="Clear Logs", command=self.clear_logs).grid(row=6, column=0, sticky="ew", padx=4, pady=2)

        # Custom homing
        home = ttk.LabelFrame(mid, text="Custom Homing (hard-limit based)")
        home.grid(row=0, column=2, sticky="nsew", padx=(0,8))
        ttk.Button(home, text="Home X", command=self.start_home_x).grid(row=0, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(home, text="Home Y", command=self.start_home_y).grid(row=1, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(home, text="Home X+Y", command=self.start_home_xy).grid(row=2, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(home, text="Stop", command=self.request_stop).grid(row=3, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(home, text="Go X Mid", command=lambda: self.go_mid('X')).grid(row=4, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(home, text="Go Y Mid", command=lambda: self.go_mid('Y')).grid(row=5, column=0, sticky="ew", padx=4, pady=2)

        # Manual Jog panel
        jog = ttk.LabelFrame(mid, text="Manual Jog")
        jog.grid(row=0, column=3, sticky="nsew")
        ttk.Label(jog, text="Step (mm):").grid(row=0, column=0, sticky="w")
        self.jog_step = ttk.Entry(jog, width=8); self.jog_step.insert(0, format(cfg.jog_step, "g"))
        self.jog_step.grid(row=0, column=1, sticky="w", padx=(0,6))
        self._bind_persist(self.jog_step)
        ttk.Label(jog, text="Feed (mm/min):").grid(row=1, column=0, sticky="w")
        self.jog_feed = ttk.Entry(jog, width=8); self.jog_feed.insert(0, format(cfg.jog_feed, "g"))
        self.jog_feed.grid(row=1, column=1, sticky="w", padx=(0,6))
        self._bind_persist(self.jog_feed)
        row_btns = 2
        ttk.Button(jog, text="X +", command=lambda: self.manual_jog("X", +1)).grid(row=row_btns, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(jog, text="X −", command=lambda: self.manual_jog("X", -1)).grid(row=row_btns, column=1, sticky="ew", padx=4, pady=2)
        row_btns += 1
        ttk.Button(jog, text="Y +", command=lambda: self.manual_jog("Y", +1)).grid(row=row_btns, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(jog, text="Y −", command=lambda: self.manual_jog("Y", -1)).grid(row=row_btns, column=1, sticky="ew", padx=4, pady=2)
        row_btns += 1
        ttk.Button(jog, text="Stop Jog", command=self.request_stop).grid(row=row_btns, column=0, columnspan=2, sticky="ew", padx=4, pady=2)
        row_btns += 1
        ttk.Button(jog, text="Set X Home (MPos)", command=self.capture_x_home).grid(row=row_btns, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(jog, text="Go X Home", command=self.go_x_home).grid(row=row_btns, column=1, sticky="ew", padx=4, pady=2)
        row_btns += 1
        ttk.Button(jog, text="Set Y Home (MPos)", command=self.capture_y_home).grid(row=row_btns, column=0, sticky="ew", padx=4, pady=2)
        ttk.Button(jog, text="Go Y Home", command=self.go_y_home).grid(row=row_btns, column=1, sticky="ew", padx=4, pady=2)

        # Position display + Go To controls
        posctl = ttk.LabelFrame(mid, text="Position / Go To")
        posctl.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(6,0))
        for c in range(10):
            posctl.columnconfigure(c, weight=1)

        # Live status/position vars
        self.state_var = tk.StringVar(value="---")
        self.mpos_x = tk.StringVar(value="---")
        self.mpos_y = tk.StringVar(value="---")
        self.wpos_x = tk.StringVar(value="---")
        self.wpos_y = tk.StringVar(value="---")
        self.span_x = tk.StringVar(value="---")
        self.span_y = tk.StringVar(value="---")

        ttk.Label(posctl, text="State:").grid(row=0, column=0, sticky="w")
        ttk.Label(posctl, textvariable=self.state_var).grid(row=0, column=1, sticky="w")

        ttk.Label(posctl, text="MPos X:").grid(row=0, column=2, sticky="e")
        ttk.Label(posctl, textvariable=self.mpos_x).grid(row=0, column=3, sticky="w")
        ttk.Label(posctl, text="Y:").grid(row=0, column=4, sticky="e")
        ttk.Label(posctl, textvariable=self.mpos_y).grid(row=0, column=5, sticky="w")

        ttk.Label(posctl, text="WPos X:").grid(row=1, column=2, sticky="e")
        ttk.Label(posctl, textvariable=self.wpos_x).grid(row=1, column=3, sticky="w")
        ttk.Label(posctl, text="Y:").grid(row=1, column=4, sticky="e")
        ttk.Label(posctl, textvariable=self.wpos_y).grid(row=1, column=5, sticky="w")

        # Go To controls (coordinates)
        ttk.Label(posctl, text="Go To:").grid(row=2, column=0, sticky="w", pady=(4,0))
        ttk.Label(posctl, text="X").grid(row=2, column=1, sticky="e", pady=(4,0))
        self.goto_x = ttk.Entry(posctl, width=10)
        self.goto_x.grid(row=2, column=2, sticky="w", padx=(4,8), pady=(4,0))
        ttk.Label(posctl, text="Y").grid(row=2, column=3, sticky="e", pady=(4,0))
        self.goto_y = ttk.Entry(posctl, width=10)
        self.goto_y.grid(row=2, column=4, sticky="w", padx=(4,8), pady=(4,0))

        self.move_mode = tk.StringVar(value="abs")
        ttk.Radiobutton(posctl, text="Absolute (G90)", variable=self.move_mode, value="abs").grid(row=2, column=7, sticky="w", pady=(4,0))
        ttk.Radiobutton(posctl, text="Relative (G91)", variable=self.move_mode, value="rel").grid(row=2, column=8, sticky="w", pady=(4,0))

        ttk.Label(posctl, text="Feed:").grid(row=2, column=9, sticky="e", pady=(4,0))
        self.goto_feed = ttk.Entry(posctl, width=10)
        self.goto_feed.insert(0, format(cfg.goto_feed, "g"))
        self.goto_feed.grid(row=2, column=10, sticky="w", padx=(4,8), pady=(4,0))
        self._bind_persist(self.goto_feed)

        ttk.Button(posctl, text="Move", command=self.move_to_coords).grid(row=2, column=11, sticky="ew", padx=(8,0), pady=(4,0))

        ttk.Label(posctl, text="Length (mm):").grid(row=3, column=0, sticky="w", pady=(2,0))
        ttk.Label(posctl, text="X").grid(row=3, column=1, sticky="e", pady=(2,0))
        ttk.Label(posctl, textvariable=self.span_x).grid(row=3, column=2, sticky="w", pady=(2,0))
        ttk.Label(posctl, text="Y").grid(row=3, column=3, sticky="e", pady=(2,0))
        ttk.Label(posctl, textvariable=self.span_y).grid(row=3, column=4, sticky="w", pady=(2,0))


        # Speed / Acceleration config
        sa = ttk.LabelFrame(master, text="Speed / Acceleration ($110/$111, $120/$121)")
        sa.grid(row=2, column=0, sticky="ew", padx=8, pady=(2,0))
        for c in range(8):
            sa.columnconfigure(c, weight=1)

        ttk.Label(sa, text="$110 X max rate (mm/min):").grid(row=0, column=0, sticky="w")
        self.s110 = ttk.Entry(sa, width=10); self.s110.grid(row=0, column=1, sticky="w", padx=(0,12))
        ttk.Label(sa, text="$111 Y max rate (mm/min):").grid(row=0, column=2, sticky="w")
        self.s111 = ttk.Entry(sa, width=10); self.s111.grid(row=0, column=3, sticky="w", padx=(0,12))
        ttk.Label(sa, text="$120 X accel (mm/sec²):").grid(row=0, column=4, sticky="w")
        self.s120 = ttk.Entry(sa, width=10); self.s120.grid(row=0, column=5, sticky="w", padx=(0,12))
        ttk.Label(sa, text="$121 Y accel (mm/sec²):").grid(row=0, column=6, sticky="w")
        self.s121 = ttk.Entry(sa, width=10); self.s121.grid(row=0, column=7, sticky="w", padx=(0,12))
        ttk.Button(sa, text="Read", command=self.read_settings).grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=4)
        ttk.Button(sa, text="Apply", command=self.apply_speed_accel).grid(row=1, column=2, columnspan=2, sticky="ew", padx=4, pady=4)

        # G-code entry
        entry_frame = ttk.Frame(master)
        entry_frame.grid(row=3, column=0, sticky="ew", padx=8, pady=(4,0))
        entry_frame.columnconfigure(0, weight=1)
        self.entry = ttk.Entry(entry_frame); self.entry.grid(row=0, column=0, sticky="ew")
        ttk.Button(entry_frame, text="Send", command=self.send_from_entry).grid(row=0, column=1, padx=(6,0))
        
        # Log
        self.log_area = scrolledtext.ScrolledText(master, width=80, height=18, state="disabled")
        self.log_area.bind("<MouseWheel>", "break")
        self.log_area.grid(row=4, column=0, padx=8, pady=8, sticky="nsew")
        master.rowconfigure(4, weight=1)
        master.columnconfigure(0, weight=1)

        # kick off UI log flusher
        self.master.after(40, self._flush_ui_log)
        # kick off periodic status polling for position display
        self.master.after(100, self._poll_status_and_update_ui)
        LOGGER.info("GRBLInterface initialised")

    # -------------------- Connection helpers --------------------
    def is_connected(self) -> bool:
        backend = self._motor_backend()
        if backend is not None:
            try:
                return backend.is_connected()
            except Exception:
                return False
        return bool(self.ser and getattr(self.ser, "is_open", False))

    def _list_ports(self):
        backend = self._motor_backend()
        if backend is not None and motor_list_serial_ports:
            try:
                ports = motor_list_serial_ports()
            except Exception:
                LOGGER.exception("Failed to enumerate motor ports via motor_driver")
                ports = []
            if getattr(backend, "is_simulation", False) and "Simulated Motor" not in ports:
                ports.insert(0, "Simulated Motor")
            LOGGER.info("Detected %d GRBL port(s) via MotorController", len(ports))
            return ports
        try:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            LOGGER.info("Detected %d GRBL port(s)", len(ports))
            return ports
        except Exception:
            LOGGER.exception("Failed to enumerate serial ports")
            return []

    def refresh_ports(self):
        self._motor_config = load_motor_config()
        ports = self._list_ports()
        LOGGER.debug("Refreshing port combo with values: %s", ports)
        preferred_port = self._motor_config.default_port or self.port_var.get()
        selected = select_port_value(list(ports), self.port_var.get(), preferred_port)
        self.port_cmb.configure(values=tuple(ports))
        self.port_var.set(selected)

    def log(self, msg: str):
        # Always enqueue; actual widget updates happen on main thread
        self.ui_log_queue.append(msg)

    def _flush_ui_log(self):
        if self.ui_log_queue:
            self.log_area.config(state="normal")
            while self.ui_log_queue:
                self.log_area.insert(tk.END, self.ui_log_queue.popleft() + "\n")
            self.log_area.see(tk.END)
            self.log_area.config(state="disabled")
        self.master.after(40, self._flush_ui_log)

    def clear_logs(self):
        try:
            self.ui_log_queue.clear()
        except Exception:
            pass
        self.log_area.config(state="normal")
        self.log_area.delete("1.0", tk.END)
        self.log_area.config(state="disabled")

    def set_state(self, txt):
        self.machine_state.set(txt)

    # -------------------- Serial I/O --------------------
    def connect(self):
        port = self.port_var.get().strip()
        baud = int(self.baud_entry.get().strip() or "115200")
        backend = self._motor_backend()
        if backend is not None:
            if not port:
                messagebox.showerror("Connection error", "Please select a port.")
                return
            if not getattr(backend, "is_simulation", False) and port.lower().startswith("simulated"):
                messagebox.showerror("Connection error", "Simulation port is only available in simulation mode.")
                return
            try:
                # Ensure any existing session is closed before reconnecting.
                if self.is_connected():
                    self.disconnect()
            except Exception:
                pass
            try:
                backend.connect(port, baudrate=baud, timeout=0.05)
                self.set_state(f"Connected {port}@{baud}")
                self.log(f"Connected to GRBL on {port} @ {baud}")
                self.wait_for_startup(timeout=4.0)
                self._pending_hard_limits_enable = True
                self.master.after(250, self._maybe_enable_hard_limits)
                self._update_port_defaults(port, baud)
                self._start_status_polling()
                self._start_motor_worker()
            except MotorError as exc:
                messagebox.showerror("Connection error", str(exc))
                self.set_state("Disconnected")
            except Exception as exc:  # pragma: no cover - defensive
                messagebox.showerror("Connection error", str(exc))
                self.set_state("Disconnected")
            return
        try:
            self.disconnect()
            self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=0.5)
            # Start reader first, then wait for banner from queued lines
            self.start_reader()
            self.wait_for_startup(timeout=4.0)
            self.set_state(f"Connected {port}@{baud}")
            self.log(f"Connected to GRBL on {port} @ {baud}")
            self._pending_hard_limits_enable = True
            self.master.after(250, self._maybe_enable_hard_limits)
            self._update_port_defaults(port, baud)
            self._start_status_polling()
            self._start_motor_worker()
        except Exception as e:
            messagebox.showerror("Connection error", str(e))
            self.set_state("Disconnected")
            self.ser = None

    def reconnect(self):
        # Perform a full disconnect + connect to trigger a clean GRBL reset
        try:
            self.disconnect()
            time.sleep(0.2)
        except Exception:
            pass
        self.connect()

    def disconnect(self):
        self._stop_status_polling()
        self._stop_motor_worker()
        backend = self._motor_backend()
        if backend is not None:
            try:
                backend.disconnect()
            except Exception:
                LOGGER.debug("Motor disconnect failed", exc_info=True)
            self.set_state("Disconnected")
            return
        self.reader_alive.clear()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        self.reader_thread = None
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
        self.ser = None
        self.set_state("Disconnected")

    def start_reader(self):
        if self._use_motor_backend:
            return
        self.reader_alive.set()
        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()

    def reader_loop(self):
        if self._use_motor_backend:
            return
        while self.reader_alive.is_set() and self.is_connected():
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                break
            if not line:
                continue
            # Store lines (thread-safe)
            with self.line_lock:
                self.line_queue.append(line)
                self.tail_queue.append(line)
            # Parse light state
            if line.startswith("<"):
                with self.status_lock:
                    self.last_status = line
            if line.startswith(ALARM_PREFIX):
                self.alarm_event.set()
            # Defer UI write to main thread
            self.log(line)
        self.set_state("Disconnected")

    def _handle_motor_line(self, line: str) -> None:
        if line is None:
            return
        try:
            text = line.strip()
        except Exception:
            text = ""
        if not text:
            return
        with self.line_lock:
            self.line_queue.append(text)
            self.tail_queue.append(text)
        if text.startswith("<"):
            with self.status_lock:
                self.last_status = text
        if text.startswith(ALARM_PREFIX):
            self.alarm_event.set()
        self.log(text)

    def _release_motor_listener(self) -> None:
        if self._motor_unregister is None:
            return
        try:
            self._motor_unregister()
        except Exception:
            LOGGER.debug("Error while unregistering motor listener", exc_info=True)
        self._motor_unregister = None

    def _apply_default_port_selection(self, *, force: bool = False, ports: Optional[list[str]] = None) -> None:
        if not hasattr(self, "port_cmb"):
            return
        available = list(ports if ports is not None else self.port_cmb["values"])
        preferred = (self._motor_config.default_port or "").strip()
        current = self.port_var.get().strip()
        if not force and current and current in available:
            return
        selected = select_port_value(
            available,
            current if current in available else "",
            preferred,
        )
        self.port_var.set(selected)

    def _bind_persist(self, widget: tk.Widget) -> None:
        widget.bind("<Return>", lambda _: self._schedule_persist_motor_config())
        if PERSIST_ON_FOCUS_OUT:
            widget.bind("<FocusOut>", lambda _: self._schedule_persist_motor_config())

    def _read_float_entry(self, entry: tk.Entry, fallback: float) -> float:
        try:
            raw = entry.get().strip()
        except Exception:
            return fallback
        if not raw:
            return fallback
        try:
            return float(raw)
        except Exception:
            return fallback

    def _read_int_entry(self, entry: tk.Entry, fallback: int) -> int:
        try:
            raw = entry.get().strip()
        except Exception:
            return fallback
        if not raw:
            return fallback
        try:
            return int(float(raw))
        except Exception:
            return fallback

    def _capture_ui_into_config(self, cfg: MotorConfig) -> MotorConfig:
        cfg.axis_enabled["X"] = bool(self.x_en.get())
        cfg.axis_enabled["Y"] = bool(self.y_en.get())
        cfg.homing_feed = self._read_float_entry(self.feed_entry, cfg.homing_feed)
        cfg.homing_clearance = self._read_float_entry(self.clear_entry, cfg.homing_clearance)
        cfg.homing_long_jog = self._read_float_entry(self.long_entry, cfg.homing_long_jog)
        cfg.jog_step = self._read_float_entry(self.jog_step, cfg.jog_step)
        cfg.jog_feed = self._read_float_entry(self.jog_feed, cfg.jog_feed)
        cfg.goto_feed = self._read_float_entry(self.goto_feed, cfg.goto_feed)
        cfg.default_baud = self._read_int_entry(self.baud_entry, cfg.default_baud)
        try:
            cfg.default_port = self.port_var.get().strip()
        except Exception:
            pass
        return cfg

    def _persist_motor_config(self) -> None:
        cfg = self._capture_ui_into_config(load_motor_config())
        try:
            save_motor_config(cfg)
        except Exception:
            LOGGER.exception("Failed to save motor config", exc_info=True)
        else:
            self._motor_config = cfg

    def _schedule_persist_motor_config(self, delay_ms: int = 250) -> None:
        """Debounce config writes while the user is editing fields."""
        try:
            if self._persist_after_id is not None:
                self.master.after_cancel(self._persist_after_id)
        except Exception:
            pass
        try:
            self._persist_after_id = self.master.after(delay_ms, self._persist_motor_config)
        except Exception:
            self._persist_after_id = None

    def _motor_backend(self) -> Optional[MotorController]:
        if not self._use_motor_backend:
            return None
        return self._motor

    def _update_axis_length_display(self, axis: str, span: float) -> None:
        """Update the UI labels that show the homed travel length per axis."""

        label = self.span_x if axis.upper() == "X" else self.span_y
        try:
            label.set(f"{span:.3f}")
        except Exception:
            pass

    def _update_port_defaults(self, port: str, baud: int) -> None:
        cfg = load_motor_config()
        changed = False
        if port and port != cfg.default_port:
            cfg.default_port = port
            changed = True
        if baud and baud != cfg.default_baud:
            cfg.default_baud = baud
            changed = True
        if changed:
            try:
                save_motor_config(cfg)
            except Exception:
                LOGGER.exception("Failed to save motor config", exc_info=True)
            else:
                self._motor_config = cfg

    def _steps_per_mm(self, axis: str) -> float:
        key = "$100" if axis.upper() == "X" else "$101"
        try:
            raw = float(self.settings.get(key, self.default_steps_per_mm))
        except Exception:
            raw = self.default_steps_per_mm
        if raw <= 0:
            raw = self.default_steps_per_mm
        if key not in self._steps_warned and key not in self.settings:
            self._steps_warned.add(key)
            self.log(f"ℹ Using default steps/mm ({raw:g}) for {axis}; read $$ to load $100/$101.")
        return raw

    def _ui_distance_to_mm(self, _axis: str, distance_mm: float) -> float:
        """UI jog/move distances are already expressed in mm."""

        try:
            return float(distance_mm)
        except Exception:
            return 0.0

    def _ui_feed_to_mm(self, _axis: str, feed_mm_per_min: float) -> float:
        """UI feeds are already expressed in mm/min."""

        try:
            return float(feed_mm_per_min)
        except Exception:
            return 0.0

    def _wait_for_motion_settle(self, axis: str, distance: float, feed: float) -> None:
        """Poll status to confirm GRBL finished the commanded jog."""

        est = abs(distance) / max(feed, 1e-6) * 60.0
        timeout = min(3.0, est + 0.6)
        start = time.time()
        while time.time() - start < timeout and not self.stop_event.is_set():
            try:
                self.query_status()
            except Exception:
                pass
            time.sleep(0.1)
            with self.status_lock:
                status = self.last_status
            if status.startswith("<Idle"):
                return
            if status.startswith("ALARM"):
                self.alarm_event.set()
                return

    # -------------------- Safe send wrappers --------------------
    def safe_send_raw(self, data: bytes) -> bool:
        if not self.is_connected():
            self.log("❌ Not connected (skip raw send)")
            return False
        backend = self._motor_backend()
        if backend is not None:
            try:
                backend.send_raw(data)
                return True
            except Exception as exc:
                self.log(f"❌ Send failed: {exc}")
                return False
        try:
            self.ser.write(data)
            return True
        except Exception as e:
            self.log(f"❌ Send failed: {e}")
            return False

    def safe_send(self, cmd: str) -> bool:
        if not self.is_connected():
            self.log("❌ Not connected (skip send)")
            return False
        backend = self._motor_backend()
        if backend is not None:
            try:
                backend.send_line(cmd, wait_for_ok=False)
                self.log(f">>> {cmd}")
                return True
            except Exception as exc:
                self.log(f"❌ Send failed: {exc}")
                return False
        try:
            out = (cmd + "\r\n").encode("ascii")
            self.ser.write(out)
            self.log(f">>> {cmd}")
            return True
        except Exception as e:
            self.log(f"❌ Send failed: {e}")
            return False

    def _send_line_wait_ok(self, cmd: str, *, timeout: float = 2.0) -> bool:
        """Send a line and confirm an 'ok' response (best-effort for direct serial)."""
        if not self.is_connected():
            self.log("❌ Not connected (skip send)")
            return False
        backend = self._motor_backend()
        if backend is not None:
            try:
                backend.send_line(cmd, wait_for_ok=True, timeout=timeout)
                self.log(f">>> {cmd}")
                return True
            except Exception as exc:
                self.log(f"❌ Send failed: {exc}")
                return False

        with self.line_lock:
            self.tail_queue.clear()
        if not self.safe_send(cmd):
            return False
        t0 = time.time()
        while time.time() - t0 < timeout and not self.stop_event.is_set():
            with self.line_lock:
                tail = list(self.tail_queue)
            if any(ln.startswith("error:") for ln in tail):
                return False
            if any(ln.startswith(OK_PREFIX) for ln in tail):
                return True
            time.sleep(0.05)
        return False

    def _maybe_enable_hard_limits(self, *, force: bool = False) -> None:
        """Ensure GRBL hard limits are enabled ($21=1), with throttled retries."""
        backend = self._motor_backend()
        if backend is not None and getattr(backend, "is_simulation", False):
            self._pending_hard_limits_enable = False
            return
        if not self.is_connected():
            return
        if self._suspend_hard_limits_enforcement:
            return
        if not (force or self._pending_hard_limits_enable):
            return
        # Avoid writing settings while motion/jog is in progress. We rely on a fresh status query
        # because backend snapshots don't reflect incremental $J motions.
        try:
            self.query_status()
        except Exception:
            pass
        time.sleep(0.05)
        state = self._grbl_state()
        if state is not None and state != "Idle":
            return
        now = time.time()
        if not force and (now - self._pending_hard_limits_last_attempt) < 1.5:
            return
        self._pending_hard_limits_last_attempt = now

        ok = self._send_line_wait_ok("$21=1", timeout=2.5)
        if ok:
            self._pending_hard_limits_enable = False
            self.settings["$21"] = 1.0
            self.log("✓ Hard limits enabled ($21=1)")
        else:
            self._pending_hard_limits_enable = True
            if force:
                self.log("⚠ Could not enable hard limits ($21=1); will retry.")

    def send_from_entry(self):
        cmd = self.entry.get().strip()
        if cmd:
            self.safe_send(cmd)
            self.entry.delete(0, tk.END)

    # -------------------- Reset / status / waiters --------------------
    def get_mpos(self, tries=40, sleep_s=0.08):
        """
        Poll status until we can parse MPos. Returns dict like {'X': 12.345, 'Y': ..., 'Z': ...}
        Requires $10 mask that includes MPos in <status> (your $10=19 already does).
        """
        backend = self._motor_backend()
        if backend is not None:
            if not self.is_connected():
                return None
            try:
                mpos, _wpos = backend.read_status_positions(timeout=0.3)
            except Exception:
                return None
            if not mpos:
                return None
            return {
                "X": mpos.get("X", 0.0),
                "Y": mpos.get("Y", 0.0),
                "Z": mpos.get("Z", 0.0),
            }
        m = None
        for _ in range(tries):
            self.query_status()
            time.sleep(sleep_s)
            with self.status_lock:
                status = self.last_status  # e.g. <Idle|MPos:1.000,0.000,0.000|...>
            if not status or "MPos:" not in status:
                continue
            try:
                part = status.split("MPos:", 1)[1]
                nums = part.split("|", 1)[0].split(",")
                vals = [float(x) for x in nums[:3]]
                m = {
                    "X": vals[0],
                    "Y": vals[1] if len(vals) > 1 else 0.0,
                    "Z": vals[2] if len(vals) > 2 else 0.0
                }
                break
            except Exception:
                pass
        return m
    
    def _capture_home(self, axis: str) -> None:
        ax = axis.upper()
        if not self.is_connected():
            self.log("❌ Not connected")
            return
        m = self.get_mpos()
        if not m or ax not in m:
            self.log("⚠ Could not read MPos to capture home.")
            return
        pos = m[ax]
        if ax == "X":
            self.home_x_mpos = pos
        elif ax == "Y":
            self.home_y_mpos = pos
        self.last_home_mpos[ax] = pos
        self.log(f"🏠 Captured {ax} Home (MPos): {pos:.3f}")
        if self._set_work_zero_for_axis(ax):
            backend = self._motor_backend()
            if backend is not None:
                try:
                    backend.set_position(ax, 0.0)
                except Exception:
                    pass
            self.log(f"✓ Set {ax} work coordinate to 0.000 at current position")
        else:
            self.log(f"⚠ Failed to set {ax} work coordinate to 0.000")

    def capture_x_home(self):
        self._capture_home("X")

    def capture_y_home(self):
        self._capture_home("Y")

    def _get_home_target(self, axis: str) -> float | None:
        ax = axis.upper()
        if ax == "X":
            return self.home_x_mpos
        if ax == "Y":
            return self.home_y_mpos
        return None

    def _set_home_target(self, axis: str, value: float) -> None:
        ax = axis.upper()
        if ax == "X":
            self.home_x_mpos = value
        elif ax == "Y":
            self.home_y_mpos = value
        self.last_home_mpos[ax] = value

    def _go_home(self, axis: str, ensure_other_axes: bool = True):
        ax = axis.upper()
        if not self.is_connected():
            self.log("❌ Not connected")
            return
        if ensure_other_axes and not self._ensure_other_axes_ready({ax}, set()):
            return
        home_target = self._get_home_target(ax)
        backend = self._motor_backend()
        if backend is not None:
            try:
                base_feed = float(self.jog_feed.get() or self.feed_entry.get() or "1600")
            except Exception:
                base_feed = 1600.0
            gentle = min(1600.0, max(60.0, base_feed))
            try:
                current = backend.read_status_position(ax, use_machine=True, timeout=0.3)
                if current is None:
                    current = backend.get_position(ax)
            except Exception as exc:
                self.log(f"⚠ Could not read current position: {exc}")
                return
            target = home_target if home_target is not None else 0.0
            delta = target - current
            if abs(delta) < 1e-3:
                self.log(f"✅ Already at {ax} Home ({target:.3f}).")
                return
            try:
                backend.jog_increment(ax, delta, gentle)
                self.log(f"📍 Jogging to {ax} Home ({target:.3f})")
            except Exception as exc:
                self.log(f"❌ Failed to move to {ax} home: {exc}")
            return
        if home_target is None:
            try:
                base_feed = float(self.jog_feed.get() or self.feed_entry.get() or "1600")
            except Exception:
                base_feed = 1600.0
            gentle = min(1600.0, max(60.0, base_feed))
            if not self.safe_send("G90"):
                return
            self.wait_for_ok_or_idle(0.2)
            if not self.safe_send(f"G1 {ax}0 F{gentle:.1f}"):
                return
            time.sleep(0.3)
            self.log(f"📍 Moved to {ax} Home (WCS {ax}0)")
            return
        m = self.get_mpos()
        if not m or ax not in m:
            self.log("⚠ Could not read current MPos to go home.")
            return
        cur = m[ax]
        delta = home_target - cur
        if abs(delta) < 1e-3:
            self.log(f"✅ Already at {ax} Home.")
            return
        try:
            feed = float(self.jog_feed.get() or self.feed_entry.get() or "400")
        except Exception:
            feed = 1600.0
        feed = min(1600.0, max(60.0, feed))
        lo = self.last_finish_mpos.get(ax)
        hi = self.last_home_mpos.get(ax)
        if lo is not None and hi is not None:
            span = abs(hi - lo)
            if span >= 0.5:
                lo_, hi_ = (lo, hi) if lo <= hi else (hi, lo)
                margin = min(0.2, 0.1 * span)
                target = max(min(home_target, hi_ - margin), lo_ + margin)
                if abs(target - home_target) > 1e-6:
                    self.log(f"ℹ Clamped {ax} Home target within span [{lo_:.3f},{hi_:.3f}] → {target:.3f}")
                delta = target - cur
        self.stop_event.clear(); self.alarm_event.clear()
        if not self.safe_send("G91"):
            return
        self.wait_for_ok_or_idle(0.2)
        remaining = delta
        def pick_step(rem):
            mag = abs(rem)
            if mag > 2.0:
                s = 1.0
            elif mag > 1.0:
                s = 0.5
            elif mag > 0.5:
                s = 0.3
            else:
                s = 0.2
            return s if rem >= 0 else -s
        step = pick_step(remaining)
        moved = 0.0
        while abs(remaining) > 1e-3 and not self.alarm_event.is_set():
            d = step if abs(remaining) > abs(step) else remaining
            if not self.safe_send(f"$J=G91 {ax}{d:.3f} F{feed:.1f}"):
                break
            try:
                est = abs(d) / max(feed, 1e-3) * 60.0
                time.sleep(max(0.15, min(1.5, est)))
            except Exception:
                time.sleep(0.2)
            moved += d
            remaining = delta - moved
            step = pick_step(remaining)
            if self.stop_event.is_set():
                break
        self.wait_for_ok_or_idle(2.0)
        if self.alarm_event.is_set():
            self.log(f"⚠ Alarm while approaching {ax} Home; performing pull-off and capture.")
            try:
                self._recover_after_alarm("home approach")
            except Exception:
                self.reconnect()
                if not self.unlock_and_prepare():
                    self.log("❌ Could not unlock after home approach.")
                    return
            pull = 0.3
            self.safe_send("G91"); self.wait_for_ok_or_idle(0.2)
            self.safe_send(f"$J=G91 {ax}{-pull:.3f} F{feed:.1f}")
            time.sleep(max(0.3, min(1.0, pull / max(feed, 1e-3) * 60.0)))
            m2 = self.get_mpos() or {}
            if ax in m2:
                self._set_home_target(ax, m2[ax])
                self.log(f"📌 Captured {ax} Home after pull-off: {m2[ax]:.3f}")
            return
        m2 = self.get_mpos() or {}
        at = m2.get(ax, home_target)
        self.log(f"📍 Moved to {ax} Home at MPos {at:.3f} (target {home_target:.3f})")

    def go_x_home(self):
        self._go_home("X")

    def go_y_home(self):
        self._go_home("Y")

    def soft_reset(self):
        # Cancel jog, send Ctrl-X, drain via reader, wait for banner
        self.safe_send_raw(b"\x85")
        time.sleep(0.05)
        # Best-effort clear of input buffer so we catch fresh reset lines
        backend = self._motor_backend()
        if backend is not None:
            try:
                backend.reset_input_buffer()
            except Exception:
                LOGGER.debug("Failed to reset motor input buffer", exc_info=True)
        else:
            try:
                if self.ser:
                    self.ser.reset_input_buffer()
            except Exception:
                pass
        self.safe_send_raw(b"\x18")
        self.log(">>> [Soft Reset]")
        self.alarm_event.clear()
        time.sleep(0.10)
        # Do not wait here; higher-level recovery handles a robust wait

    def wait_for_startup(self, timeout=3.0):
        """
        Wait for clear GRBL reset/welcome indicators after a soft reset by scanning queued lines.
        Accepts any of the following as a successful reset indication:
        - line starts with "Grbl" (welcome banner)
        - contains "['$' for help]" (welcome banner variant)
        - MSG lines commonly printed on reset: "[MSG:Reset to continue]" or
          "[MSG:'$H'|'$X' to unlock]"
        """
        backend = self._motor_backend()
        if backend is not None and getattr(backend, "is_simulation", False):
            return True
        t0 = time.time()
        while time.time() - t0 < timeout and self.is_connected():
            with self.line_lock:
                recent = list(self.line_queue)[-200:]
            for ln in recent:
                if ln.startswith("Grbl"):
                    return True
                if "['$' for help]" in ln:
                    return True
                if ln.startswith("[MSG:") and (
                    "Reset to continue" in ln or
                    ("$H" in ln and "$X" in ln and "unlock" in ln)
                ):
                    return True
            time.sleep(0.02)
        return False

    def unlock_and_prepare(self, retries=3) -> bool:
        """Send $X and confirm 'ok'. Returns True only if unlocked."""
        for _ in range(retries):
            self.alarm_event.clear()
            with self.line_lock:
                self.tail_queue.clear()
            if not self.safe_send("$X"):
                return False
            ok = self.wait_for_ok_or_idle(timeout=1.5)
            if ok:
                return True
            time.sleep(0.2)
        return False

    def query_status(self):
        self.safe_send_raw(b"?")

    def _motor_task_loop(self) -> None:
        while not self._motor_task_stop.is_set():
            try:
                func, args, kwargs, label = self._motor_task_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                func(*args, **kwargs)
            except Exception as exc:
                self.log(f"❌ {label} failed: {exc}")

    def _start_motor_worker(self) -> None:
        if self._motor_task_thread and self._motor_task_thread.is_alive():
            return
        self._motor_task_stop.clear()
        thread = threading.Thread(
            target=self._motor_task_loop,
            name="MotorTaskWorker",
            daemon=True,
        )
        self._motor_task_thread = thread
        thread.start()

    def _stop_motor_worker(self) -> None:
        self._motor_task_stop.set()
        thread = self._motor_task_thread
        if thread and thread.is_alive():
            thread.join(timeout=0.5)
        self._motor_task_thread = None
        self._motor_task_stop.clear()
        try:
            while True:
                self._motor_task_queue.get_nowait()
        except queue.Empty:
            pass

    def _queue_motor_task(
        self,
        func: Callable[[], None],
        *args,
        label: str = "Motor task",
        **kwargs,
    ) -> bool:
        if self._motor_backend() is None:
            return False
        self._start_motor_worker()
        try:
            self._motor_task_queue.put_nowait((func, args, kwargs, label))
        except queue.Full:
            self.log("⚠ Motor busy; command dropped.")
            return False
        return True

    def _send_status_query(self) -> None:
        backend = self._motor_backend()
        if backend is not None:
            try:
                backend.send_raw(b"?")
            except Exception:
                pass
            return
        if self.ser is None:
            return
        try:
            self.ser.write(b"?")
        except Exception:
            pass

    def _status_poll_loop(self) -> None:
        while not self._status_poll_stop.is_set():
            if not self.is_connected():
                time.sleep(0.1)
                continue
            self._send_status_query()
            time.sleep(self._status_poll_interval)

    def _start_status_polling(self) -> None:
        if self._status_poll_thread and self._status_poll_thread.is_alive():
            return
        self._status_poll_stop.clear()
        thread = threading.Thread(
            target=self._status_poll_loop,
            name="GRBLStatusPoller",
            daemon=True,
        )
        self._status_poll_thread = thread
        thread.start()

    def _stop_status_polling(self) -> None:
        self._status_poll_stop.set()
        thread = self._status_poll_thread
        if thread and thread.is_alive():
            thread.join(timeout=0.5)
        self._status_poll_thread = None
        self._status_poll_stop.clear()

    # -------------------- Live position UI --------------------
    def _parse_status_line(self, line: str):
        """
        Parse a GRBL status line like:
        <Idle|MPos:1.000,2.000,0.000|WPos:1.000,2.000,0.000|...>
        Returns dict with keys: state, mpos (tuple), wpos (tuple), wco (tuple)
        Any missing field becomes None.
        """
        out = {"state": None, "mpos": None, "wpos": None, "wco": None}
        if not line or not line.startswith("<"):
            return out
        # State
        try:
            m = STATUS_RE.match(line)
            if m:
                out["state"] = m.group("state")
        except Exception:
            pass
        # MPos
        try:
            if "MPos:" in line:
                seg = line.split("MPos:", 1)[1]
                nums = seg.split("|", 1)[0].split(",")
                vals = [float(x) for x in nums[:3]]
                while len(vals) < 3:
                    vals.append(0.0)
                out["mpos"] = (vals[0], vals[1], vals[2])
        except Exception:
            pass
        # WPos
        try:
            if "WPos:" in line:
                seg = line.split("WPos:", 1)[1]
                nums = seg.split("|", 1)[0].split(",")
                vals = [float(x) for x in nums[:3]]
                while len(vals) < 3:
                    vals.append(0.0)
                out["wpos"] = (vals[0], vals[1], vals[2])
        except Exception:
            pass
        # WCO
        try:
            if "WCO:" in line:
                seg = line.split("WCO:", 1)[1]
                nums = seg.split("|", 1)[0].split(",")
                vals = [float(x) for x in nums[:3]]
                while len(vals) < 3:
                    vals.append(0.0)
                out["wco"] = (vals[0], vals[1], vals[2])
        except Exception:
            pass
        try:
            wco = out["wco"] or self._last_wco
            if out["mpos"] and wco and not out["wpos"]:
                out["wpos"] = tuple(m - o for m, o in zip(out["mpos"], wco))
            if out["wpos"] and wco and not out["mpos"]:
                out["mpos"] = tuple(w + o for w, o in zip(out["wpos"], wco))
        except Exception:
            pass
        return out

    def _grbl_state(self) -> str | None:
        try:
            with self.status_lock:
                line = self.last_status
        except Exception:
            line = ""
        if not line or not line.startswith("<"):
            return None
        try:
            m = STATUS_RE.match(line)
            if not m:
                return None
            state = (m.group("state") or "").strip()
            return state or None
        except Exception:
            return None

    def _wait_for_idle(self, timeout: float = 2.5) -> bool:
        """Poll GRBL status until it reports <Idle...> (best-effort)."""
        backend = self._motor_backend()
        if backend is not None and getattr(backend, "is_simulation", False):
            return True
        t0 = time.time()
        while time.time() - t0 < timeout and not self.stop_event.is_set():
            try:
                self.query_status()
            except Exception:
                pass
            time.sleep(0.08)
            if self._grbl_state() == "Idle":
                return True
        return False

    def _poll_status_and_update_ui(self):
        try:
            backend = self._motor_backend()
            if backend is not None:
                try:
                    snap = backend.snapshot()
                except Exception:
                    snap = None
                state_txt = "Disconnected"
                if snap and snap.connected:
                    port_label = snap.port or ("Simulated Motor" if getattr(backend, "is_simulation", False) else "Unknown")
                    state_txt = f"Connected {port_label}"
                    with self.status_lock:
                        ls = self.last_status
                    st = self._parse_status_line(ls)
                    if st.get("wco"):
                        self._last_wco = st["wco"]
                    if st.get("mpos"):
                        self._last_mpos = st["mpos"]
                    if st.get("wpos"):
                        self._last_wpos = st["wpos"]
                    mpos = st.get("mpos") or self._last_mpos
                    wpos = st.get("wpos") or self._last_wpos
                    if hasattr(self, "state_var"):
                        if st.get("state"):
                            self.state_var.set(st["state"])
                        else:
                            self.state_var.set("Moving" if snap.moving else "Idle")
                    if getattr(backend, "is_simulation", False) and snap:
                        sim_pos = snap.positions or {}
                        if not mpos:
                            mpos = (
                                sim_pos.get("X", 0.0),
                                sim_pos.get("Y", 0.0),
                                sim_pos.get("Z", 0.0),
                            )
                        if not wpos:
                            wpos = mpos
                    if hasattr(self, "mpos_x"):
                        if mpos:
                            self.mpos_x.set(f"{mpos[0]:.3f}")
                            self.mpos_y.set(f"{mpos[1]:.3f}")
                        else:
                            self.mpos_x.set("n/a")
                            self.mpos_y.set("n/a")
                    if hasattr(self, "wpos_x"):
                        if wpos:
                            self.wpos_x.set(f"{wpos[0]:.3f}")
                            self.wpos_y.set(f"{wpos[1]:.3f}")
                        else:
                            self.wpos_x.set("n/a")
                            self.wpos_y.set("n/a")
                if state_txt != self._last_state_text:
                    self._last_state_text = state_txt
                    self.machine_state.set(state_txt)
            elif self.is_connected():
                with self.status_lock:
                    ls = self.last_status
                st = self._parse_status_line(ls)
                if st.get("wco"):
                    self._last_wco = st["wco"]
                if st.get("mpos"):
                    self._last_mpos = st["mpos"]
                if st.get("wpos"):
                    self._last_wpos = st["wpos"]
                mpos = st.get("mpos") or self._last_mpos
                wpos = st.get("wpos") or self._last_wpos
                if hasattr(self, 'state_var') and st.get('state'):
                    self.state_var.set(st['state'])
                if mpos and hasattr(self, 'mpos_x'):
                    self.mpos_x.set(f"{mpos[0]:.3f}")
                    self.mpos_y.set(f"{mpos[1]:.3f}")
                if wpos and hasattr(self, 'wpos_x'):
                    self.wpos_x.set(f"{wpos[0]:.3f}")
                    self.wpos_y.set(f"{wpos[1]:.3f}")
            # Safety: if we disabled hard limits temporarily, retry enabling them in the background.
            self._maybe_enable_hard_limits()
        except Exception:
            pass
        # Reschedule
        try:
            self.master.after(100, self._poll_status_and_update_ui)
        except Exception:
            pass

    # -------------------- Coordinate move --------------------
    def move_to_coords(self):
        if not self.is_connected():
            self.log("❌ Not connected")
            return
        # Collect coordinates; allow blanks to be skipped
        def read_val(entry):
            txt = entry.get().strip() if entry else ""
            if txt == "":
                return None
            return float(txt)
        try:
            x_mm = read_val(getattr(self, 'goto_x', None))
            y_mm = read_val(getattr(self, 'goto_y', None))
            feed_txt = (self.goto_feed.get().strip() if hasattr(self, 'goto_feed') else '')
            if feed_txt == "":
                # Fall back to jog feed or homing feed
                try:
                    feed_mm = float(self.jog_feed.get() or self.feed_entry.get() or "400")
                except Exception:
                    feed_mm = 400.0
            else:
                feed_mm = float(feed_txt)
        except ValueError:
            messagebox.showerror("Invalid input", "Please enter numeric values for X/Y and Feed.")
            return

        mode = (self.move_mode.get() if hasattr(self, 'move_mode') else 'abs')

        # No coordinates provided
        if x_mm is None and y_mm is None:
            self.log("ℹ No coordinates provided.")
            return

        moving_axes = set()
        if x_mm is not None:
            moving_axes.add("X")
        if y_mm is not None:
            moving_axes.add("Y")
        require_on_positive = set()
        if mode == "rel":
            if x_mm is not None and x_mm > 0:
                require_on_positive.add("X")
            if y_mm is not None and y_mm > 0:
                require_on_positive.add("Y")
        else:
            if x_mm is not None:
                cur = self._read_wpos_axis("X")
                if cur is None:
                    if x_mm > 0:
                        require_on_positive.add("X")
                elif x_mm > cur:
                    require_on_positive.add("X")
            if y_mm is not None:
                cur = self._read_wpos_axis("Y")
                if cur is None:
                    if y_mm > 0:
                        require_on_positive.add("Y")
                elif y_mm > cur:
                    require_on_positive.add("Y")

        backend = self._motor_backend()
        if backend is not None:
            feed_value = max(1.0, feed_mm)
            def _do_move(
                backend=backend,
                mode=mode,
                x_mm=x_mm,
                y_mm=y_mm,
                feed_value=feed_value,
            ) -> None:
                if not self._ensure_other_axes_ready(moving_axes, require_on_positive):
                    return
                try:
                    if x_mm is not None:
                        if mode == 'abs':
                            backend.move_to_coordinate("X", x_mm, feed=feed_value)
                            self.log(f"→ Move: X -> {x_mm:.3f} mm @ F{feed_value:.1f}")
                        else:
                            backend.move_by("X", x_mm, feed=feed_value)
                            self.log(f"→ Move: X Δ{x_mm:.3f} mm @ F{feed_value:.1f}")
                    if y_mm is not None:
                        if mode == 'abs':
                            backend.move_to_coordinate("Y", y_mm, feed=feed_value)
                            self.log(f"→ Move: Y -> {y_mm:.3f} mm @ F{feed_value:.1f}")
                        else:
                            backend.move_by("Y", y_mm, feed=feed_value)
                            self.log(f"→ Move: Y Δ{y_mm:.3f} mm @ F{feed_value:.1f}")
                except Exception as exc:
                    self.log(f"❌ Motor move failed: {exc}")
            self._queue_motor_task(_do_move, label="Move command")
            return

        if not self._ensure_other_axes_ready(moving_axes, require_on_positive):
            return

        # Build move in requested mode
        if mode == 'abs':
            if not self.safe_send("G90"):
                return
        else:
            if not self.safe_send("G91"):
                return
        self.wait_for_ok_or_idle(0.3)

        parts = []
        if x_mm is not None:
            parts.append(f"X{x_mm:.3f}")
        if y_mm is not None:
            parts.append(f"Y{y_mm:.3f}")
        cmd = "G1 " + " ".join(parts) + f" F{float(feed_mm):.1f}"
        if self.safe_send(cmd):
            self.log(f"→ Move: {cmd}")
        else:
            self.log("❌ Failed to send move command.")

    def wait_for_alarm(self, timeout=20.0):
        self.alarm_event.clear()
        t0 = time.time()
        while time.time() - t0 < timeout and not self.stop_event.is_set():
            if self.alarm_event.is_set():
                return True
            time.sleep(0.02)
        return False

    def wait_for_ok_or_idle(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout and not self.stop_event.is_set():
            with self.line_lock:
                if any(ln.startswith(OK_PREFIX) for ln in self.tail_queue):
                    return True
                if any(ln.startswith("<Idle") for ln in self.tail_queue):
                    return True
            time.sleep(0.05)
        return False

    def get_pn_axes(self):
        self.query_status()
        time.sleep(0.1)
        m = PN_RE.search(self.last_status)
        if not m:
            return set()
        return set(m.group(1))

    def _pn_axes_from_last_status(self) -> set[str]:
        try:
            with self.status_lock:
                status = self.last_status
        except Exception:
            status = ""
        m = PN_RE.search(status or "")
        if not m:
            return set()
        return set(m.group(1))

    # -------------------- Jog / Move helpers --------------------
    def jog_inc(self, axis: str, distance: float, feed: float):
        """Use $J incremental jog: $J=G91 X... F..."""
        axis = axis.upper()
        backend = self._motor_backend()
        if backend is not None:
            if threading.current_thread() is self._ui_thread:
                def _do_jog(
                    backend=backend,
                    axis=axis,
                    distance=distance,
                    feed=feed,
                ) -> None:
                    require = {axis} if distance > 0 else set()
                    if not self._ensure_other_axes_ready({axis}, require):
                        return
                    try:
                        backend.jog_increment(axis, distance, feed)
                    except Exception as exc:
                        self.log(f"❌ Jog failed: {exc}")
                        return
                    try:
                        if self.measure_active and axis == (self.measure_axis or ""):
                            self.measure_accum += distance
                    except Exception:
                        pass
                self._queue_motor_task(_do_jog, label="Jog")
                return
            require = {axis} if distance > 0 else set()
            if not self._ensure_other_axes_ready({axis}, require):
                return
            try:
                backend.jog_increment(axis, distance, feed)
            except Exception as exc:
                self.log(f"❌ Jog failed: {exc}")
                return
            try:
                if self.measure_active and axis == (self.measure_axis or ""):
                    self.measure_accum += distance
            except Exception:
                pass
            return
        require = {axis} if distance > 0 else set()
        if not self._ensure_other_axes_ready({axis}, require):
            return
        cmd = f"$J=G91 {axis}{distance:.3f} F{feed:.1f}"
        if self.safe_send(cmd):
            # Wait for controller acknowledgement/idle before issuing another step
            try:
                est = abs(distance) / max(feed, 1e-6) * 60.0
            except Exception:
                est = 0.2
            self.wait_for_ok_or_idle(timeout=max(0.2, min(3.0, est + 0.2)))
        # Accumulate commanded distance if measuring span for this axis
        try:
            if self.measure_active and axis.upper() == (self.measure_axis or ""):
                self.measure_accum += distance
        except Exception:
            pass

    def manual_jog(self, axis: str, dir_sign: int):
        try:
            step_mm = float(self.jog_step.get())
            feed_mm = float(self.jog_feed.get())
            step_mm = abs(step_mm) * (1 if dir_sign > 0 else -1)
            self.stop_event.clear()
            self.jog_inc(axis.upper(), step_mm, feed_mm)
        except ValueError:
            messagebox.showerror("Invalid value", "Please enter numeric Step and Feed values.")

    def move_until_alarm(
        self,
        axis: str,
        direction: int,
        long_mm: float,
        feed: float,
        label: str,
        step_mm: float | None = None,
    ) -> None:
        """
        Repeatedly jog in small chunks until ALARM is raised.
        - direction: +1 or -1
        - long_mm: used to choose a reasonable chunk size and safety cap
        """
        axis = axis.upper()
        dir_sign = 1 if direction > 0 else -1
        feed = float(feed or 1600.0)
    
        # choose a chunk size (mm) that’s gentle but moves forward
        raw_chunk = step_mm if step_mm is not None else long_mm
        chunk = max(0.5, min(10.0, abs(raw_chunk)))  # 0.5..10 mm per step
        max_runtime = 60.0                          # seconds hard timeout
        max_steps = int(max(200, min(5000, (abs(long_mm) * 10) / chunk)))  # safety cap
    
        self.log(f"→ Moving {label}: {axis} {'+' if dir_sign>0 else '-'} in {chunk}mm steps until ALARM")
        self.alarm_event.clear()
        if self._pending_hard_limits_enable:
            if self._suspend_hard_limits_enforcement:
                raise RuntimeError("Hard limits are temporarily disabled; cannot seek an ALARM.")
            self._wait_for_idle(timeout=2.5)
            self._maybe_enable_hard_limits(force=True)
            if self._pending_hard_limits_enable:
                raise RuntimeError("Hard limits are disabled ($21=0). Send $21=1 and retry.")
        # Clear recent lines so lock/reset detection only sees fresh messages
        with self.line_lock:
            self.tail_queue.clear()

        t0 = time.time()
        steps = 0
    
        try:
            while not self.alarm_event.is_set() and not self.stop_event.is_set():
                # send one small incremental jog
                self.jog_inc(axis, dir_sign * chunk, feed)
                steps += 1
                # wait for planner confirmation/idle to avoid stacking tiny moves
                self.wait_for_ok_or_idle(timeout=max(0.2, min(1.5, abs(chunk) / max(feed, 1e-6) * 60.0)))
                self._wait_for_motion_settle(axis, chunk, feed)
                pressed = self._pn_axes_from_last_status()
                if axis in pressed:
                    # If hard limits are off, we still want to stop at the physical switch.
                    self.alarm_event.set()
                    self.log(f"✓ Limit detected via Pn:{''.join(sorted(pressed))} (no ALARM); stopping seek.")
                    break

                # Wait approximately for move duration to avoid stacking jogs
                est_s = max(0.15, min(2.0, (abs(chunk) / max(feed, 1e-6)) * 60.0))
                waited = 0.0
                while waited < est_s:
                    # Detect controller reset/lock conditions and abort cleanly
                    with self.line_lock:
                        recent = list(self.tail_queue)
                    if any((ln.startswith("Grbl") or
                            "['$' for help]" in ln or
                            ln.startswith("[MSG:Check Limits]") or
                            ("$H" in ln and "$X" in ln and "unlock" in ln) or
                            ln.startswith("error:8")) for ln in recent):
                        self.safe_send_raw(b"\x85")
                        # raise RuntimeError("Controller reset/locked; unlock with $X before jogging.")
                    if self.alarm_event.is_set() or self.stop_event.is_set():
                        break
                    # time.sleep(0.02)
                    waited += 0.02
    
                # safety guards
                if (time.time() - t0) > max_runtime:
                    raise RuntimeError(f"Timed out moving toward {label} without hitting a limit.")
                if steps >= max_steps:
                    raise RuntimeError(f"Safety cap reached while seeking {label} (no ALARM).")
    
            if self.stop_event.is_set():
                # cancel any ongoing jog if user pressed Stop (GRBL 1.1 jog cancel: 0x85)
                self.safe_send_raw(b"\x85")
                raise RuntimeError("Stopped by user")
    
            if not self.alarm_event.is_set():
                # no alarm and not stopped => cancel jog just in case and error out
                self.safe_send_raw(b"\x85")
                raise RuntimeError(f"No ALARM triggered while seeking {label}. Check $21=1 and limit wiring.")
    
            self.log(f"✓ ALARM hit on {axis} {label}")
    
        finally:
            # Best-effort jog cancel if we’re not in alarm (harmless if already alarmed)
            if not self.alarm_event.is_set():
                self.safe_send_raw(b"\x85")


    # Small, robust limit clear with retries (used by homing)
    def clear_limit_with_retries(self, axis: str, away_dir: int, feed_clear=120.0,
                                 step_mm=0.25, max_attempts=10) -> bool:
        axis = axis.upper()
        attempts = 0
        # Ensure relative mode for small jogs
        self.safe_send("G91"); self.wait_for_ok_or_idle(0.2)

        # Read original $21 and temporarily disable hard limits to avoid re-ALARM while clearing
        try:
            self.read_settings()
            orig21 = int(self.settings.get("$21", 1))
        except Exception:
            orig21 = 1
        changed_21 = False
        switch_cleared = False
        restore_ok = True
        try:
            if orig21 == 1:
                if self._send_line_wait_ok("$21=0", timeout=2.5):
                    changed_21 = True
                    restore_ok = False
                    self._pending_hard_limits_enable = True
                    self.settings["$21"] = 0.0
                    self._suspend_hard_limits_enforcement = True
                    self.log(f"ℹ Disabled hard limits while clearing {axis} switch")

            while attempts < max_attempts and not self.stop_event.is_set():
                attempts += 1
                pressed = self.get_pn_axes()
                if axis not in pressed:
                    switch_cleared = True
                    break
                try:
                    dist = step_mm if away_dir > 0 else -step_mm
                    self.jog_inc(axis, dist, feed_clear)
                    time.sleep(0.25)
                except Exception as e:
                    self.log(f"⚠ Jog error while clearing {axis}: {e}")
                pressed = self.get_pn_axes()
                if axis not in pressed:
                    switch_cleared = True
                    break
            if switch_cleared:
                self.log(f"✓ {axis} switch cleared")
            else:
                self.log(f"⚠ Gave up clearing {axis} after {max_attempts} attempts.")
        finally:
            if changed_21:
                self._suspend_hard_limits_enforcement = False
                # Restore $21 and wait a moment. If we fail, stop the flow (next ALARM seek would be unsafe).
                self._wait_for_idle(timeout=2.5)
                for _ in range(4):
                    if self._send_line_wait_ok("$21=1", timeout=2.5):
                        restore_ok = True
                        break
                    time.sleep(0.25)
                if restore_ok:
                    self._pending_hard_limits_enable = False
                    self.settings["$21"] = 1.0
                    self.log(f"ℹ Restored hard limits after clearing {axis}")
                else:
                    self._pending_hard_limits_enable = True
                    try:
                        self.master.after(400, self._maybe_enable_hard_limits)
                    except Exception:
                        pass

        if not switch_cleared:
            return False
        if changed_21 and not restore_ok:
            self.log("❌ Switch cleared, but failed to re-enable hard limits ($21=1). Aborting for safety.")
            return False
        return True

    # -------------------- $10 mask handling --------------------
    def ensure_status_mask_with_pn(self):
        self.read_settings()  # fills self.settings
        ten_val = self.settings.get("$10")
        if ten_val is not None:
            self.original_status_mask = ten_val
        if ten_val is None or int(ten_val) not in (19, 3, 7, 11, 15, 23, 27, 31):
            if self.safe_send("$10=19"):
                self.wait_for_ok_or_idle(1.0)
                self.log("✓ Set $10=19 to read Pn: switch pins")

    def restore_status_mask(self):
        if self.original_status_mask is not None and self.is_connected():
            if self.safe_send(f"$10={int(self.original_status_mask)}"):
                self.wait_for_ok_or_idle(1.0)
                self.log(f"✓ Restored $10={int(self.original_status_mask)}")
            self.original_status_mask = None

    # -------------------- Settings ($$) helpers --------------------
    def read_settings(self):
        """Read $$ and parse selected keys into self.settings + populate UI fields if present."""
        self.settings.clear()
        if not self.safe_send("$$"):
            return
        time.sleep(0.40)
        with self.line_lock:
            # deques don't support slicing; snapshot to list then slice
            tail = list(self.line_queue)[-300:]
        for ln in tail:
            if ln.startswith("$") and "=" in ln and ln[1].isdigit():
                k, v = ln.split("=", 1)
                k = k.strip()
                try:
                    val = float(v.strip())
                except Exception:
                    continue
                self.settings[k] = val

        # Fill fields if found
        def put(entry, key):
            if key in self.settings:
                entry.delete(0, tk.END)
                entry.insert(0, str(int(self.settings[key])))

        put(self.s110, "$110")
        put(self.s111, "$111")
        put(self.s120, "$120")
        put(self.s121, "$121")
        if "$10" in self.settings:
            self.log(f"Info: $10 (status mask) = {int(self.settings['$10'])}")

    def apply_speed_accel(self):
        """Apply $110/$111 (max rate, mm/min) and $120/$121 (accel, mm/sec^2)."""
        cmds = []
        def maybe(cmd, entry):
            val = entry.get().strip()
            if val != "":
                float(val)  # validate numeric
                cmds.append(f"{cmd}={val}")
        try:
            maybe("$110", self.s110)
            maybe("$111", self.s111)
            maybe("$120", self.s120)
            maybe("$121", self.s121)
        except ValueError:
            messagebox.showerror("Invalid input", "Please enter numeric values for speed/acceleration fields.")
            return
        if not cmds:
            self.log("No values to apply.")
            return
        for c in cmds:
            self.safe_send(c)
            self.wait_for_ok_or_idle(0.5)
        self.log("✓ Speed/acceleration updated. (Power cycle may be required on some builds)")

    # -------------------- Homing sequences --------------------
    def start_home_x(self):
        threading.Thread(target=self._home_sequence, args=(True, False), daemon=True).start()

    def start_home_y(self):
        threading.Thread(target=self._home_sequence, args=(False, True), daemon=True).start()

    def start_home_xy(self):
        threading.Thread(target=self._home_sequence, args=(True, True), daemon=True).start()

    def request_stop(self):
        self.stop_event.set()
        backend = self._motor_backend()
        if backend is not None:
            try:
                backend.stop()
            except Exception as exc:
                self.log(f"⚠ Stop request failed: {exc}")
            else:
                self.log("⏹ Stop requested")
            try:
                time.sleep(0.1)
                self.reconnect()
                self.log("🔌 Motor reconnected after stop")
            except Exception as exc:
                self.log(f"⚠ Reconnect after stop failed: {exc}")
        else:
            self.safe_send_raw(b"\x85")  # jog cancel
            self.stop_event.set()
            self.log("⏹ Stop requested")
            try:
                time.sleep(0.1)
                self.reconnect()
                self.log("🔌 Motor reconnected after stop")
            except Exception as exc:
                self.log(f"⚠ Reconnect after stop failed: {exc}")

    def _recover_after_alarm(self, note: str):
        if self.stop_event.is_set():
            raise RuntimeError("Stopped by user")
        self.log(f"↺ Recover after ALARM: {note}")
        # Simplify recovery per user workflow: full reconnect then $X
        self.safe_send_raw(b"\x85")
        time.sleep(0.05)
        self.reconnect()
        if self.stop_event.is_set():
            raise RuntimeError("Stopped by user")
        if not self.unlock_and_prepare(retries=5):
            self.log("ℹ Could not unlock with $X after reconnect. Ensure limit switch is released.")
            raise RuntimeError("Unlock failed after reconnect")
        if self._pending_hard_limits_enable:
            self._wait_for_idle(timeout=2.5)
            self._maybe_enable_hard_limits(force=True)
            if self._pending_hard_limits_enable:
                raise RuntimeError("Hard limits are disabled ($21=0) after reconnect; enable with $21=1.")
        # Relative mode for follow-up jogs
        self.safe_send("G91")
        self.wait_for_ok_or_idle(0.5)

    def _set_work_zero_for_axis(self, axis: str) -> bool:
        ax = axis.upper()
        wpos = None
        try:
            self.query_status()
            time.sleep(0.05)
            with self.status_lock:
                st = self._parse_status_line(self.last_status)
            wpos = st.get("wpos") or self._last_wpos
            if not wpos:
                mpos = st.get("mpos") or self._last_mpos
                wco = st.get("wco") or self._last_wco
                if mpos and wco:
                    wpos = tuple(m - o for m, o in zip(mpos, wco))
        except Exception:
            wpos = self._last_wpos

        cmd = f"G10 L20 P0 {ax}0"
        if wpos:
            try:
                vals = {"X": wpos[0], "Y": wpos[1], "Z": wpos[2]}
                for other in ("X", "Y", "Z"):
                    if other != ax:
                        cmd += f" {other}{vals[other]:.3f}"
            except Exception:
                pass
        if self._send_line_wait_ok(cmd, timeout=2.0):
            return True
        if self.safe_send(f"G92 {ax}0"):
            self.wait_for_ok_or_idle(0.5)
            return True
        return False

    def _read_wpos_axis(self, axis: str) -> Optional[float]:
        ax = axis.upper()
        try:
            with self.status_lock:
                st = self._parse_status_line(self.last_status)
            wpos = st.get("wpos") or self._last_wpos
            if wpos:
                return float({"X": wpos[0], "Y": wpos[1], "Z": wpos[2]}.get(ax, 0.0))
        except Exception:
            pass
        return None

    def _move_axis_to_zero(self, axis: str) -> bool:
        ax = axis.upper()
        if ax == "X" and hasattr(self, "x_en") and not self.x_en.get():
            return True
        if ax == "Y" and hasattr(self, "y_en") and not self.y_en.get():
            return True
        backend = self._motor_backend()
        try:
            base_feed = float(self.jog_feed.get() or self.feed_entry.get() or "1600")
        except Exception:
            base_feed = 1600.0
        gentle = min(1600.0, max(60.0, base_feed))
        if backend is not None:
            try:
                backend.move_to_coordinate(ax, 0.0, feed=gentle)
                self.log(f"↩ Move {ax} to 0.000 before + move")
                return True
            except Exception as exc:
                self.log(f"❌ Failed to move {ax} to 0: {exc}")
                return False
        if not self.safe_send("G90"):
            return False
        self.wait_for_ok_or_idle(0.2)
        if not self.safe_send(f"G1 {ax}0.000 F{gentle:.1f}"):
            return False
        self.log(f"↩ Move {ax} to 0.000 before + move")
        return True

    def _ensure_other_axes_ready(self, moving_axes: set[str], require_on_positive: set[str]) -> bool:
        if self._homing_active:
            return True
        if "X" in require_on_positive and "Y" not in moving_axes:
            if not self._move_axis_to_zero("Y"):
                return False
        if "Y" in require_on_positive and "X" not in moving_axes:
            if not self._move_axis_to_zero("X"):
                return False
        return True

    def _home_sequence(self, do_x: bool, do_y: bool):
        if not self.is_connected():
            self.log("❌ Not connected")
            return
        homed_ok = True
        try:
            self._homing_active = True
            feed = float(self.feed_entry.get() or "1600")
            clear_mm = float(self.clear_entry.get() or "3")
            long_mm = float(self.long_entry.get() or "1000")

            if not do_x and not do_y:
                self.log("Nothing to home (no axes selected)")
                return

            self.set_state("Homing...")
            self.stop_event.clear()

            # Ensure we can see Pn
            self.ensure_status_mask_with_pn()

            # Relative mode, try an initial unlock (okay if this doesn't ack; per-axis flow rechecks)
            self.safe_send("G91")
            time.sleep(0.05)
            self.safe_send("$X")
            self.wait_for_ok_or_idle(1.0)

            if do_x and self.x_en.get():
                homed_ok &= self._home_one_axis_release("X", long_mm, feed, clear_mm)
                if self.stop_event.is_set():
                    homed_ok = False
                if homed_ok and do_y and self.y_en.get():
                    # Return X to its captured home before starting Y if it drifted.
                    try:
                        home_x = self.last_home_mpos.get("X")
                        cur = self.get_mpos() or {}
                        cur_x = cur.get("X")
                        if home_x is not None and cur_x is not None:
                            if abs(cur_x - home_x) > 1e-2:
                                self.log("↩ Moving X back to home before starting Y homing")
                                self.go_x_home()
                        else:
                            self.log("↩ Moving X back to home before starting Y homing")
                            self.go_x_home()
                    except Exception as exc:
                        self.log(f"⚠ Failed to return X to home before Y: {exc}")
                        homed_ok = False

            if do_y and self.y_en.get():
                homed_ok &= self._home_one_axis_release("Y", long_mm, feed, clear_mm)
                if self.stop_event.is_set():
                    homed_ok = False

            if homed_ok and self.is_connected():
                self.safe_send("G90")
                self.log("✅ Homing complete. Current work zero set to axes homed.")
                self.set_state("Idle")
            else:
                self.log("⛔ Homing aborted/failed.")
                self.set_state("Error")

        except RuntimeError as e:
            self.log(f"❌ Homing error: {e}")
            self.set_state("Error")
        except Exception as e:
            self.log(f"❌ Exception: {e}")
            self.set_state("Error")
        finally:
            self._homing_active = False
            # Only restore if still connected
            if self.is_connected():
                try:
                    self.restore_status_mask()
                except Exception:
                    pass

    def go_mid(self, axis: str):
        ax = axis.upper()
        if not self.is_connected():
            self.log("❌ Not connected")
            return
        # Prefer saved midpoint in WCS if available
        if ax in getattr(self, 'last_mid_wcs', {}):
            try:
                base_feed = float(self.jog_feed.get() or self.feed_entry.get() or "1600")
            except Exception:
                base_feed = 1600.0
            gentle = min(1600.0, max(60.0, base_feed))
            target = self.last_mid_wcs[ax]
            if not self.safe_send("G90"):
                return
            self.wait_for_ok_or_idle(0.2)
            if not self.safe_send(f"G1 {ax}{target:.3f} F{gentle:.1f}"):
                return
            time.sleep(0.3)
            self.log(f"📍 Moved to {ax} midpoint at WCS {ax}{target:.3f}")
            return
        home = self.last_home_mpos.get(ax)
        finish = self.last_finish_mpos.get(ax)
        if home is None or finish is None:
            self.log(f"ℹ Midpoint unknown for {ax}. Run homing to capture + and - limits.")
            return
        m = self.get_mpos()
        if not m or ax not in m:
            self.log("⚠ Could not read current MPos.")
            return
        cur = m[ax]
        # Compute midpoint and handle degenerate/short span
        span = abs(home - finish)
        center = (home + finish) / 2.0
        if span < 0.5:
            self.log(f"ℹ {ax} span too small ({span:.3f}mm). Midpoint may be unreliable; aborting.")
            return
        # Clamp center inside span with safety margin proportional to span
        lo_, hi_ = (home, finish) if home <= finish else (finish, home)
        margin = min(0.2, 0.1 * span)
        center = max(min(center, hi_ - margin), lo_ + margin)
        delta = center - cur
        self.log(f"ℹ {ax} mid debug — cur:{cur:.3f} home:{home:.3f} finish:{finish:.3f} center:{center:.3f} delta:{delta:.3f}")
        if abs(delta) < 1e-3:
            self.log(f"✅ Already at {ax} midpoint (MPos {center:.3f}).")
            return
        try:
            feed_in = self.jog_feed.get() if hasattr(self, 'jog_feed') else ''
            base_feed = float(feed_in or self.feed_entry.get() or "1600")
        except Exception:
            base_feed = 1600.0
        gentle = min(1600.0, max(60.0, base_feed))
        # Ensure fresh start
        self.stop_event.clear(); self.alarm_event.clear()
        if not self.safe_send("G91"):
            return
        self.wait_for_ok_or_idle(0.3)
        # Chunked jog to avoid planner stacking and to be gentle near limits
        remaining = delta
        def pick_step(rem):
            mag = abs(rem)
            if mag > 2.0:
                s = 1.0
            elif mag > 1.0:
                s = 0.5
            elif mag > 0.5:
                s = 0.3
            else:
                s = 0.2
            return s if rem >= 0 else -s
        step = pick_step(remaining)
        moved = 0.0
        while abs(remaining) > 1e-3 and not self.alarm_event.is_set():
            d = step if abs(remaining) > abs(step) else remaining
            if not self.safe_send(f"$J=G91 {ax}{d:.3f} F{gentle:.1f}"):
                break
            try:
                est = abs(d) / max(gentle, 1e-3) * 60.0
                time.sleep(max(0.15, min(1.5, est)))
            except Exception:
                time.sleep(0.2)
            moved += d
            remaining = delta - moved
            step = pick_step(remaining)
            if self.stop_event.is_set():
                break
        self.wait_for_ok_or_idle(2.0)
        if self.alarm_event.is_set():
            self.log("⚠ Alarm while moving to midpoint; stopped early.")
            return
        # Verify and report after Idle
        m2 = self.get_mpos() or {}
        at = m2.get(ax, center)
        self.log(f"📍 Moved to {ax} midpoint at MPos {at:.3f} (target {center:.3f})")

    def _home_one_axis_release(self, axis: str, long_mm: float, feed: float, clear_mm: float) -> bool:
        ax = axis.upper()
        self.log(f"=== Homing {ax} (release-based) ===")
        gentle = min(1600.0, max(100.0, float(feed or 1600)))
        pull = max(0.2, min(2.0, abs(clear_mm)))

        plus_release = None
        minus_release = None

        try:
            if self.stop_event.is_set():
                raise RuntimeError("Stopped by user")
            # 1) Seek +limit until ALARM
            self.move_until_alarm(ax, +1, long_mm, feed, label="+limit")
            if self.stop_event.is_set():
                raise RuntimeError("Stopped by user")
            # Recover and jog off until switch releases; capture plus_release
            self._recover_after_alarm("+limit")
            if self.stop_event.is_set():
                raise RuntimeError("Stopped by user")
            if not self.clear_limit_with_retries(ax, away_dir=-1, feed_clear=gentle, step_mm=0.25, max_attempts=40):
                self.log(f"⚠ Could not clear {ax}+ switch reliably.")
            m = self.get_mpos()
            if m and ax in m:
                plus_release = m[ax]
                self.log(f"📌 {ax}+ release MPos: {plus_release:.3f}")
                self.last_finish_mpos[ax] = plus_release
            # Start span measurement from +release to -release
            self.measure_active = True
            self.measure_axis = ax
            self.measure_accum = 0.0

            # 2) Seek -limit until ALARM using small steps for accuracy
            self.move_until_alarm(ax, -1, long_mm, gentle, label="-limit", step_mm=3.0)
            if self.stop_event.is_set():
                raise RuntimeError("Stopped by user")
            # Recover and jog off until switch releases; capture minus_release
            self._recover_after_alarm("-limit")
            if self.stop_event.is_set():
                raise RuntimeError("Stopped by user")
            if not self.clear_limit_with_retries(ax, away_dir=+1, feed_clear=gentle, step_mm=0.25, max_attempts=40):
                self.log(f"⚠ Could not clear {ax}- switch reliably.")
            m = self.get_mpos()
            if m and ax in m:
                minus_release = m[ax]
                self.log(f"📌 {ax}- release MPos: {minus_release:.3f}")
                # Persist home reference at -release (machine coords)
                self.last_home_mpos[ax] = minus_release
                if ax == 'X':
                    self.home_x_mpos = minus_release
                elif ax == 'Y':
                    self.home_y_mpos = minus_release
            # Stop span measurement and record absolute span
            self.measure_active = False
            span = abs(self.measure_accum)
            self.last_span[ax] = span
            self._update_axis_length_display(ax, span)
            self.log(f"📏 {ax} span by command distance = {span:.3f} mm")

            # 3) Set zero at -release (work zero) and stay there
            if not self._set_work_zero_for_axis(ax):
                return False
            backend = self._motor_backend()
            if backend is not None:
                try:
                    backend.set_position(ax, 0.0)
                except Exception:
                    self.log(f"⚠ Failed to sync {ax} position to 0 in backend")
            self.log(f"🎯 Set {ax}0 at -release; homing complete for {ax}")
            # Save midpoint for future 'Go mid' but keep position at home
            try:
                self.last_mid_wcs[ax] = span / 2.0
            except Exception:
                pass
            return True

        except RuntimeError as e:
            self.log(f"❌ Homing error: {e}")
            return False

# -------------------- Embeddable panel --------------------
class StepperPanel(ttk.Frame, GRBLInterface):
    """Embeddable Tk frame exposing the full GRBL/Stepper tooling."""
    def __init__(
        self,
        parent,
        *,
        set_title: bool = False,
        motor: Optional["MotorController"] = None,
        enable_motor_backend: Optional[bool] = None,
        **kwargs,
    ):
        LOGGER.info("Creating StepperPanel (set_title=%s)", set_title)
        ttk.Frame.__init__(self, parent, **kwargs)
        content = ttk.Frame(self)
        content.pack(fill="both", expand=True, pady=(16, 0))
        GRBLInterface.__init__(
            self,
            content,
            set_title=set_title,
            motor=motor,
            enable_motor_backend=enable_motor_backend,
        )
        LOGGER.info("StepperPanel initialised")

    def shutdown(self):
        LOGGER.info("StepperPanel shutdown invoked")
        try:
            # Ensure jog/measure loops are notified before disconnect.
            self.request_stop()
        except Exception:
            LOGGER.exception("Error while requesting stop", exc_info=True)
            pass
        try:
            self._persist_motor_config()
        except Exception:
            LOGGER.exception("Error while persisting motor config", exc_info=True)
        self.disconnect()
        self._release_motor_listener()

    def destroy(self):
        LOGGER.info("StepperPanel destroy called")
        try:
            self.shutdown()
        finally:
            super().destroy()


# -------------------- App entry --------------------
def main():
    root = tk.Tk()
    # Use ttk theme for nicer look
    try:
        style = ttk.Style()
        if "vista" in style.theme_names():
            style.theme_use("vista")
        elif "clam" in style.theme_names():
            style.theme_use("clam")
    except Exception:
        pass

    panel = StepperPanel(root, set_title=True)
    panel.grid(row=0, column=0, sticky="nsew")
    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)
    root.mainloop()

if __name__ == "__main__":
    main()
