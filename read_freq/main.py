"""Tkinter GUI for the VW frequency controller Python port."""

from __future__ import annotations

import csv
import logging
import queue
import threading
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, messagebox, ttk
from typing import Any, Callable, Optional

CONTROLLER_IMPORT_ERROR: Optional[BaseException] = None

try:  # pragma: no cover - package relative import
    from .controller import ControllerError, VWFrequencyController, format_frame
except ImportError as exc:  # pragma: no cover - fallback for script execution
    try:
        from controller import ControllerError, VWFrequencyController, format_frame
    except ImportError as fallback_exc:  # pragma: no cover - dependency missing
        CONTROLLER_IMPORT_ERROR = fallback_exc

        class ControllerError(RuntimeError):
            """Raised when the frequency controller cannot be used."""

        VWFrequencyController = None  # type: ignore[assignment]

        def format_frame(_: Any) -> str:  # type: ignore[override]
            return "Controller unavailable; cannot render frame."
    else:
        CONTROLLER_IMPORT_ERROR = None
else:
    CONTROLLER_IMPORT_ERROR = None


LOGGER = logging.getLogger("frequency.panel")


class FrequencyPanel(ttk.Frame):
    """Embeddable Tk frame that drives ``VWFrequencyController``."""

    def __init__(
        self,
        parent: tk.Misc,
        *,
        controller_factory: Optional[Callable[..., Any]] = None,
        set_title: bool = False,
        **kwargs: Any,
    ) -> None:
        super().__init__(parent, **kwargs)

        if controller_factory is None and VWFrequencyController is not None:
            controller_factory = VWFrequencyController  # type: ignore[assignment]

        self._controller_factory: Optional[Callable[..., Any]] = controller_factory
        if self._controller_factory:
            ports_provider = getattr(self._controller_factory, "available_ports", None)
        else:
            ports_provider = None
        if not callable(ports_provider):
            if VWFrequencyController is not None:
                ports_provider = VWFrequencyController.available_ports
            else:
                ports_provider = lambda: []
        self._ports_provider: Callable[[], list[str]] = ports_provider  # type: ignore[assignment]
        self._import_error = CONTROLLER_IMPORT_ERROR

        self._toplevel = self.winfo_toplevel()
        self._owns_window = False
        LOGGER.info("Initialising FrequencyPanel (controller available=%s)", self._controller_factory is not None)
        if set_title and hasattr(self._toplevel, "title"):
            try:
                self._toplevel.title("VW Frequency Controller")
            except Exception:
                pass
            if hasattr(self._toplevel, "geometry"):
                try:
                    self._toplevel.geometry("720x480")
                except Exception:
                    pass
            if hasattr(self._toplevel, "protocol"):
                try:
                    self._toplevel.protocol("WM_DELETE_WINDOW", self.on_close)
                    self._owns_window = True
                except Exception:
                    pass

        self.controller: Optional[VWFrequencyController] = None # type: ignore[assignment]
        self.connected_port: Optional[str] = None
        self.measure_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.queue: queue.Queue[tuple[str, Any]] = queue.Queue()

        self._build_ui()
        LOGGER.debug("UI built; checking dependencies")
        self._handle_missing_dependencies()
        self.after(100, self._poll_queue)

    # ------------------------------------------------------------------
    def shutdown(self) -> None:
        """Stop threads and close the controller."""

        LOGGER.info("FrequencyPanel shutdown requested")
        self.stop_event.set()
        try:
            self.stop_measurement()
        except Exception:
            LOGGER.exception("Error while stopping measurement during shutdown")
            pass
        if self.controller:
            try:
                self.controller.close()
            except Exception:
                pass
        self.controller = None

    def destroy(self) -> None:  # type: ignore[override]
        try:
            self.shutdown()
        finally:
            super().destroy()

    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        """Construct widgets for the frequency controller panel."""

        self.status_var = tk.StringVar(value="Disconnected")
        self.port_var = tk.StringVar()
        self.mode_var = tk.StringVar(value="3")
        self.k_var = tk.StringVar(value="31")
        self.baud_var = tk.StringVar(value="38400")
        self.timeout_var = tk.StringVar(value="3.0")
        self.timer_var = tk.StringVar(value="0.008192")
        self.calib_var = tk.StringVar(value="2")
        self.interval_var = tk.StringVar(value="0.0")
        self.auto_trigger_var = tk.BooleanVar(value=True)
        self.show_raw_var = tk.BooleanVar(value=False)

        header = ttk.Frame(self, padding=10)
        header.pack(fill=tk.X)

        ttk.Label(header, text="Port:").grid(row=0, column=0, sticky=tk.W)
        self.port_combo = ttk.Combobox(header, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(4, 10))
        ttk.Button(header, text="Refresh", command=self.refresh_ports).grid(row=0, column=2)

        ttk.Label(header, text="Mode:").grid(row=0, column=3, sticky=tk.W)
        ttk.Entry(header, textvariable=self.mode_var, width=5).grid(row=0, column=4, padx=4)

        ttk.Label(header, text="K:").grid(row=0, column=5, sticky=tk.W)
        ttk.Entry(header, textvariable=self.k_var, width=5).grid(row=0, column=6, padx=4)

        ttk.Label(header, text="Baud:").grid(row=0, column=7, sticky=tk.W)
        ttk.Entry(header, textvariable=self.baud_var, width=8).grid(row=0, column=8, padx=4)

        ttk.Label(header, text="Timeout (s):").grid(row=1, column=0, sticky=tk.W, pady=(6, 0))
        ttk.Entry(header, textvariable=self.timeout_var, width=8).grid(row=1, column=1, pady=(6, 0))

        ttk.Label(header, text="Timer MP (s):").grid(row=1, column=2, sticky=tk.W, pady=(6, 0))
        ttk.Entry(header, textvariable=self.timer_var, width=10).grid(row=1, column=3, pady=(6, 0))

        ttk.Label(header, text="Calibration:").grid(row=1, column=4, sticky=tk.W, pady=(6, 0))
        ttk.Entry(header, textvariable=self.calib_var, width=6).grid(row=1, column=5, pady=(6, 0))

        ttk.Label(header, text="Interval (s):").grid(row=1, column=6, sticky=tk.W, pady=(6, 0))
        ttk.Entry(header, textvariable=self.interval_var, width=8).grid(row=1, column=7, pady=(6, 0))

        ttk.Checkbutton(header, text="Auto trigger", variable=self.auto_trigger_var).grid(
            row=2, column=0, columnspan=2, sticky=tk.W, pady=(6, 0)
        )
        ttk.Checkbutton(header, text="Show raw payload", variable=self.show_raw_var).grid(
            row=2, column=2, columnspan=3, sticky=tk.W, pady=(6, 0)
        )

        for col in range(9):
            header.grid_columnconfigure(col, weight=1)

        controls = ttk.Frame(self, padding=(10, 0, 10, 10))
        controls.pack(fill=tk.X)

        self.connect_btn = ttk.Button(controls, text="Connect", command=self.connect)
        self.connect_btn.pack(side=tk.LEFT)
        self.disconnect_btn = ttk.Button(controls, text="Disconnect", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=6)
        self.start_btn = ttk.Button(controls, text="Start", command=self.start_measurement, state=tk.DISABLED)
        self.start_btn.pack(side=tk.LEFT)
        self.stop_btn = ttk.Button(controls, text="Stop", command=self.stop_measurement_command, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=6)
        self.export_btn = ttk.Button(controls, text="Export CSV", command=self.export_csv, state=tk.DISABLED)
        self.export_btn.pack(side=tk.LEFT, padx=6)

        ttk.Label(controls, textvariable=self.status_var).pack(side=tk.RIGHT)

        display = ttk.Frame(self, padding=10)
        display.pack(fill=tk.BOTH, expand=True)

        self.ch1_var = tk.StringVar(value="Channel 1: -")
        self.ch2_var = tk.StringVar(value="Channel 2: -")
        ttk.Label(display, textvariable=self.ch1_var, font=("Segoe UI", 11, "bold")).pack(anchor=tk.W)
        ttk.Label(display, textvariable=self.ch2_var, font=("Segoe UI", 11, "bold")).pack(anchor=tk.W, pady=(0, 10))

        self.log = tk.Text(display, height=12, state=tk.DISABLED)
        self.log.pack(fill=tk.BOTH, expand=True)

        self.frame_history: list[dict[str, Any]] = []
        self._frame_counter = 0

        self.refresh_ports()

    def _handle_missing_dependencies(self) -> None:
        """Disable interactive controls when the controller cannot be loaded."""

        if self._controller_factory is not None:
            return

        message = self._controller_unavailable_message()
        LOGGER.error("Frequency controller unavailable: %s", message)
        self.status_var.set(message)
        self.connect_btn.configure(state=tk.DISABLED)
        self.disconnect_btn.configure(state=tk.DISABLED)
        self.start_btn.configure(state=tk.DISABLED)
        self.stop_btn.configure(state=tk.DISABLED)
        self.export_btn.configure(state=tk.DISABLED)
        self.port_combo.configure(state="disabled")
        self._append_log(message + "\n")

        # Surface to user once, but avoid messagebox recursion if running headless.
        try:
            self.after(200, lambda: messagebox.showerror("Frequency controller unavailable", message))
        except Exception:
            pass

    def _controller_unavailable_message(self) -> str:
        """Return a human-friendly error string for missing dependencies."""

        if self._import_error:
            return (
                "Controller unavailable: "
                f"{self._import_error}. Install 'pyserial' and reconnect."
            )
        return "Controller unavailable: missing dependency."

    # ------------------------------------------------------------------
    def refresh_ports(self) -> None:
        """Populate the port dropdown via the controller factory provider."""

        ports = []
        try:
            LOGGER.debug("Refreshing available serial ports")
            ports = self._ports_provider()
        except Exception:
            LOGGER.exception("Failed to refresh ports")
            ports = []
        self.port_combo["values"] = ports
        LOGGER.info("Detected %d port(s)", len(ports))
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def connect(self) -> None:
        """Instantiate and connect the controller with current settings."""

        if self._controller_factory is None:
            messagebox.showerror("Controller unavailable", self._controller_unavailable_message())
            return
        if self.controller:
            self.disconnect()

        try:
            baudrate = int(self.baud_var.get())
            read_timeout = float(self.timeout_var.get())
            timer_mp = float(self.timer_var.get())
            calibration = int(float(self.calib_var.get()))
            LOGGER.info(
                "Connecting controller (baud=%s timeout=%s timer_mp=%s calibration=%s port=%s)",
                baudrate,
                read_timeout,
                timer_mp,
                calibration,
                self.port_var.get(),
            )
        except ValueError:
            messagebox.showerror(
                "Invalid settings",
                "Check baudrate, timeout, timer MP, and calibration values.",
            )
            return

        try:
            factory = self._controller_factory
            assert factory is not None
            self.controller = factory(
                baudrate=baudrate,
                read_timeout=read_timeout,
                calibration=calibration,
                timer_mp=timer_mp,
            )
        except ControllerError as exc:
            LOGGER.exception("Failed to create controller instance")
            self._handle_error(exc)
            self.controller = None
            return

        def _bytes_logger(direction: str, data: bytes) -> None:
            try:
                self._append_log(f"{direction} {self._format_bytes_for_log(data)}\n")
            except Exception:
                pass

        self.controller.set_bytes_logger(_bytes_logger)

        target_port = self.port_var.get()
        handshake_window: Optional[float] = None
        handshake_cycle: Optional[float] = None
        try:
            if target_port:
                self.controller.connect(target_port)
            else:
                target_port = self.controller.auto_connect()
            self.connected_port = target_port
            LOGGER.info("Controller connected on %s", target_port)

            mode = int(self.mode_var.get())
            k_value = int(self.k_var.get())
            self.controller.configure_measurement(mode, k_value)
            handshake_window = self.controller.measurement_window_seconds()
            handshake_cycle = self.controller.measurement_gap_seconds()
        except (ValueError, ControllerError) as exc:
            LOGGER.exception("Handshake with controller failed")
            self._handle_error(exc)
            if self.controller:
                try:
                    self.controller.close()
                except Exception:
                    pass
            self.controller = None
            self.connected_port = None
            return

        self.status_var.set(f"Connected on {target_port}")
        self.connect_btn.configure(state=tk.DISABLED)
        self.disconnect_btn.configure(state=tk.NORMAL)
        self.start_btn.configure(state=tk.NORMAL)
        self._append_log(f"Connected on {target_port}\n")
        if handshake_window is not None:
            message = (
                f"Handshake sent: mode={self.controller.mode}, "
                f"k={self.controller.k_value}"
            )
            requested_k = self.controller.requested_k_value
            if requested_k != self.controller.k_value:
                extra_wait = self.controller.extra_delay_seconds()
                message += f" (requested {requested_k}, extra wait~= {extra_wait:.6f}s)"
            message += f". window~= {handshake_window:.6f}s"
            if handshake_cycle is not None:
                message += f", cycle~= {handshake_cycle:.6f}s"
            self._append_log(message + "\n")
        self.start_measurement()

    def disconnect(self) -> None:
        """Close the controller connection and reset the UI."""

        LOGGER.info("Disconnect requested")
        self.stop_measurement()
        if self.controller:
            try:
                self.controller.close()
            except Exception:
                LOGGER.exception("Error while closing controller")
                pass
        self.controller = None
        self.connected_port = None
        self.status_var.set("Disconnected")
        self.connect_btn.configure(state=tk.NORMAL if self._controller_factory else tk.DISABLED)
        self.disconnect_btn.configure(state=tk.DISABLED)
        self.start_btn.configure(state=tk.DISABLED)
        self.export_btn.configure(state=tk.DISABLED)

    # ------------------------------------------------------------------
    def start_measurement(self) -> None:
        """Spawn the background measurement thread using current interval settings."""

        if not self.controller:
            LOGGER.warning("Attempted to start measurement without controller")
            messagebox.showwarning("Not connected", "Connect to the controller first.")
            return
        if self.measure_thread and self.measure_thread.is_alive():
            LOGGER.info("Measurement thread already running; ignoring start request")
            return

        try:
            interval = float(self.interval_var.get())
            timeout = float(self.timeout_var.get())
        except ValueError:
            messagebox.showerror("Invalid interval", "Interval and timeout must be numeric.")
            return

        LOGGER.info("Starting measurement (interval=%s timeout=%s)", interval, timeout)
        self.frame_history.clear()
        self._frame_counter = 0
        self.export_btn.configure(state=tk.DISABLED)

        self.stop_event.clear()
        self.measure_thread = threading.Thread(
            target=self._measurement_loop,
            args=(timeout, interval, self.auto_trigger_var.get()),
            daemon=True,
        )
        self.measure_thread.start()

        self.start_btn.configure(state=tk.DISABLED)
        self.stop_btn.configure(state=tk.NORMAL)
        self.status_var.set(f"Measuring on {self.connected_port}")
        self._append_log("Started measurement\n")

    def stop_measurement(self) -> None:
        """Stop the reader thread and restore button states."""

        LOGGER.info("Stopping measurement thread")
        self.stop_event.set()
        if self.measure_thread and self.measure_thread.is_alive():
            self.measure_thread.join(timeout=1.0)
        self.measure_thread = None
        self.stop_btn.configure(state=tk.DISABLED)
        if self.controller:
            self.start_btn.configure(state=tk.NORMAL)
            if self.connected_port:
                self.status_var.set(f"Connected on {self.connected_port}")
        else:
            self.start_btn.configure(state=tk.DISABLED)

    def stop_measurement_command(self) -> None:
        """UI callback that stops measurement and offers CSV export."""

        had_frames = bool(self.frame_history)
        self.stop_measurement()
        if had_frames and self.frame_history:
            self.export_csv()

    # ------------------------------------------------------------------
    def _measurement_loop(self, timeout: float, interval: float, auto_trigger: bool) -> None:
        """Iterate frames on a worker thread and queue them for the UI."""

        controller = self.controller
        if controller is None:
            LOGGER.warning("Measurement loop started without controller")
            return
        LOGGER.debug("Measurement loop running (timeout=%s, interval=%s, auto_trigger=%s)", timeout, interval, auto_trigger)
        try:
            for frame in controller.iter_frames(
                count=None,
                timeout=timeout,
                auto_trigger=auto_trigger,
                interval=interval,
            ):
                if self.stop_event.is_set():
                    LOGGER.debug("Measurement loop stopping due to stop_event")
                    break
                self.queue.put(("frame", frame))
        except ControllerError as exc:
            LOGGER.exception("Controller error inside measurement loop")
            self.queue.put(("error", exc))
        finally:
            self.stop_event.set()

    def _poll_queue(self) -> None:
        """Drain queued frame/error events and reschedule the next poll."""

        drained = 0
        while True:
            try:
                item = self.queue.get_nowait()
            except queue.Empty:
                break
            kind, payload = item
            if kind == "frame":
                self._handle_frame(payload)
            elif kind == "error":
                self._handle_error(payload)
            drained += 1
        if drained:
            LOGGER.debug("Processed %d queued event(s)", drained)
        self.after(100, self._poll_queue)

    def _handle_error(self, exc: Exception) -> None:
        """Display controller errors to the operator and log them."""

        LOGGER.error("Controller error delivered to UI: %s", exc)
        messagebox.showerror("Controller error", str(exc))
        self.status_var.set("Error")
        self._append_log(f"Error: {exc}\n")

    def _handle_frame(self, frame: Any) -> None:
        """Update the UI/log/history for a received frame."""

        LOGGER.debug("Handling measurement frame #%d", self._frame_counter + 1)
        ch1 = frame.channel1
        ch2 = frame.channel2
        self.ch1_var.set(
            f"Channel 1: {ch1.frequency_hz:.4f} Hz (count={ch1.count}, wait={ch1.wait_cycles})"
        )
        self.ch2_var.set(
            f"Channel 2: {ch2.frequency_hz:.4f} Hz (count={ch2.count}, wait={ch2.wait_cycles})"
        )
        self._append_log(format_frame(frame) + "\n")
        if self.show_raw_var.get():
            raw_hex = " ".join(f"{byte:02X}" for byte in frame.raw_payload)
            self._append_log(f"    raw: {raw_hex}\n")

        gate1 = frame.channel1.gate_bytes
        gate2 = frame.channel2.gate_bytes
        high1_ticks = (gate1[0] << 8) | gate1[1]
        low1_ticks = (gate1[2] << 8) | gate1[3]
        high2_ticks = (gate2[0] << 8) | gate2[1]
        low2_ticks = (gate2[2] << 8) | gate2[3]

        self._frame_counter += 1
        record = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "frame_index": self._frame_counter,
            "ch1_count": ch1.count,
            "ch1_wait": ch1.wait_cycles,
            "ch1_frequency_hz": ch1.frequency_hz,
            "ch1_gate_high_us": high1_ticks * 0.625,
            "ch1_gate_low_us": low1_ticks * 0.625,
            "ch2_count": ch2.count,
            "ch2_wait": ch2.wait_cycles,
            "ch2_frequency_hz": ch2.frequency_hz,
            "ch2_gate_high_us": high2_ticks * 0.625,
            "ch2_gate_low_us": low2_ticks * 0.625,
            "raw_payload_hex": " ".join(f"{byte:02X}" for byte in frame.raw_payload),
        }
        self.frame_history.append(record)
        if self.export_btn["state"] == tk.DISABLED:
            self.export_btn.configure(state=tk.NORMAL)

    def _append_log(self, text: str) -> None:
        """Append a line of text to the scrolling log widget."""

        LOGGER.debug("Appending log text: %s", text.strip())
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, text)
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def _format_bytes_for_log(self, data: bytes) -> str:
        """Return a hex + printable representation of serial bytes."""

        hex_part = " ".join(f"{b:02X}" for b in data)
        try:
            decoded = data.decode("utf-8")
        except UnicodeDecodeError:
            decoded = data.decode("latin-1", errors="replace")
        printable = "".join(ch if 32 <= ord(ch) <= 126 else "." for ch in decoded)
        raw_bytes = repr(data)[2:-1]  # strips the leading b''
        return f"{hex_part} | {printable} || {raw_bytes}"

    def export_csv(self) -> None:
        """Persist captured frames to a CSV file chosen by the user."""

        if not self.frame_history:
            messagebox.showinfo("No data", "No frames available to export.")
            return

        filename = filedialog.asksaveasfilename(
            title="Export measurement data",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not filename:
            return

        fieldnames = [
            "timestamp",
            "frame_index",
            "ch1_count",
            "ch1_wait",
            "ch1_frequency_hz",
            "ch1_gate_high_us",
            "ch1_gate_low_us",
            "ch2_count",
            "ch2_wait",
            "ch2_frequency_hz",
            "ch2_gate_high_us",
            "ch2_gate_low_us",
            "raw_payload_hex",
        ]

        try:
            with open(filename, "w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.frame_history)
        except OSError as exc:
            messagebox.showerror("Export failed", f"Could not write file:\n{exc}")
            return

        messagebox.showinfo(
            "Export complete",
            f"Saved {len(self.frame_history)} frame(s) to:\n{filename}",
        )

    def on_close(self) -> None:
        """Handle WM_DELETE_WINDOW by shutting down the panel cleanly."""

        self.shutdown()
        if self._owns_window and hasattr(self._toplevel, "destroy"):
            try:
                self._toplevel.destroy()
            except Exception:
                pass


class ControllerApp(FrequencyPanel):
    """Backward-compatible wrapper that owns a standalone root window."""

    def __init__(self) -> None:
        """Initialise the standalone wrapper and embed the panel."""

        self._root = tk.Tk()
        super().__init__(self._root, set_title=True)
        self.pack(fill=tk.BOTH, expand=True)

    def mainloop(self, *args: Any, **kwargs: Any) -> Any:
        """Delegate directly to the underlying Tk root mainloop."""

        return self._root.mainloop(*args, **kwargs)

    def destroy(self) -> None:  # type: ignore[override]
        """Destroy both the panel and the privately-owned root window."""

        try:
            super().destroy()
        finally:
            try:
                self._root.destroy()
            except Exception:
                pass


def main() -> None:
    app = ControllerApp()
    app.mainloop()


if __name__ == "__main__":
    main()
