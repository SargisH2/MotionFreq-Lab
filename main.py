"""Combined control panel for stepper motor GRBL interface and frequency reader."""

from __future__ import annotations

import logging
import sys
import tkinter as tk
from tkinter import ttk
from typing import Any, Callable, Mapping, Optional, Tuple, TypeVar

logging.getLogger("matplotlib").setLevel(logging.WARNING)
logging.getLogger("matplotlib.font_manager").setLevel(logging.WARNING)

try:
    from stepper_motor.main import StepperPanel
except Exception as exc:  # pragma: no cover
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        stream=sys.stderr,
    )
    logging.getLogger(__name__).exception("Failed to import StepperPanel", exc_info=exc)
    raise  

try:
    from read_freq.main import FrequencyPanel
except Exception as exc:  # pragma: no cover
    logging.getLogger(__name__).exception("Failed to import FrequencyPanel", exc_info=exc)
    raise

try:
    from tabs.measurements_tab import MeasurementsTab
except Exception as exc:  # pragma: no cover
    logging.getLogger(__name__).exception("Failed to import MeasurementsTab", exc_info=exc)
    raise

from hardware.motor_controller import MotorController

LOGGER = logging.getLogger("combined.app")


def _configure_logging() -> None:
    """Initialise the root logger once so child modules inherit the formatter."""

    if logging.getLogger().handlers:
        return
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        stream=sys.stdout,
    )
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    LOGGER.debug("Root logging configured")


class MainApp(tk.Tk):
    """Top-level Tk application coordinating every panel."""

    def __init__(self) -> None:
        _configure_logging()
        LOGGER.info("Creating MainApp root window")
        super().__init__()
        self.title("Stepper + Frequency Control")
        self.geometry("1024x768")

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=0)

        self._motor_controller = self._create_motor_controller()

        LOGGER.info("Building notebook and panels")
        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=0, column=0, sticky="nsew")

        motor_kwargs = {"motor": self._motor_controller} if self._motor_controller else None

        self.stepper = self._create_panel(
            attr_name="stepper",
            label="Stepper Motor",
            factory=StepperPanel,
            factory_kwargs=motor_kwargs,
        )
        self.reader = self._create_panel(
            attr_name="reader",
            label="Frequency Reader",
            factory=FrequencyPanel,
        )
        self.measurements = self._create_panel(
            attr_name="measurements",
            label="Measurements",
            factory=MeasurementsTab,
            factory_kwargs=motor_kwargs,
        )

        self.status_var = tk.StringVar(value="Ready")
        status_bar = ttk.Label(self, textvariable=self.status_var, anchor="w", padding=(8, 4))
        status_bar.grid(row=1, column=0, sticky="ew")

        LOGGER.info("Attaching window close handler")
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self._trace_tokens: list[Tuple[tk.Variable, str]] = []
        self._wire_status_updates()

    def _create_motor_controller(self) -> Optional["MotorController"]:
        """Instantiate a shared MotorController when the backend is available."""

        if MotorController is None:
            LOGGER.warning("MotorController backend unavailable; panels will use direct serial paths")
            return None
        try:
            controller = MotorController()
        except Exception:
            LOGGER.exception("Failed to create shared MotorController; panels will fallback")
            return None
        LOGGER.info("Shared MotorController initialised")
        return controller

    TWidget = TypeVar("TWidget", bound=tk.Widget)

    def _create_panel(
        self,
        *,
        attr_name: str,
        label: str,
        factory: Callable[[tk.Misc], "TWidget"],
        factory_kwargs: Optional[Mapping[str, Any]] = None,
    ) -> "TWidget":
        """Instantiate a panel, add it to the notebook, and return the widget."""

        params = dict(factory_kwargs or {})
        LOGGER.info("Initialising %s", label)
        try:
            panel = factory(self.notebook, **params)
        except Exception:
            LOGGER.exception("%s initialisation failed", label)
            raise
        self.notebook.add(panel, text=label)
        setattr(self, attr_name, panel)
        LOGGER.info("%s initialised", label)
        return panel

    def on_close(self) -> None:
        """Shut down every panel before destroying the Tk root window."""

        LOGGER.info("Main window closing requested")
        try:
            self.status_var.set("Shutting down...")
        except Exception:
            LOGGER.warning("Failed to set shutdown status", exc_info=True)
        for name, component in ("stepper", self.stepper), ("frequency", self.reader):
            try:
                LOGGER.info("Shutting down %s panel", name)
                component.shutdown()
            except Exception:
                LOGGER.exception("Error while shutting down %s panel", name)
        try:
            LOGGER.info("Shutting down measurements panel")
            self.measurements.shutdown()
        except Exception:
            LOGGER.exception("Error while shutting down measurements panel")
        LOGGER.info("Destroying root window")
        self.destroy()

    def _wire_status_updates(self) -> None:
        """Link panel status variables to the global status bar."""

        LOGGER.debug("Wiring status variable traces")

        def watch(var: tk.Variable, label: str) -> None:
            LOGGER.debug("Binding status watcher for %s", label)
            token = var.trace_add("write", lambda *_: self._update_status())
            self._trace_tokens.append((var, token))

        if hasattr(self.stepper, "machine_state"):
            watch(self.stepper.machine_state, "stepper")
        if hasattr(self.reader, "status_var"):
            watch(self.reader.status_var, "frequency")
        if hasattr(self.measurements, "status_var"):
            watch(self.measurements.status_var, "measurements")
        self.notebook.bind("<<NotebookTabChanged>>", lambda event: self._update_status())
        self.after(200, self._update_status)

    def _update_status(self) -> None:
        """Compose the status bar text from each panel's status variable."""

        try:
            stepper_txt = (
                self.stepper.machine_state.get()
                if hasattr(self.stepper, "machine_state")
                else "n/a"
            )
        except Exception:
            LOGGER.exception("Failed to read stepper status")
            stepper_txt = "n/a"
        try:
            reader_txt = (
                self.reader.status_var.get()
                if hasattr(self.reader, "status_var")
                else "n/a"
            )
        except Exception:
            LOGGER.exception("Failed to read frequency status")
            reader_txt = "n/a"
        try:
            measurements_txt = (
                self.measurements.status_var.get()
                if hasattr(self.measurements, "status_var")
                else "n/a"
            )
        except Exception:
            LOGGER.exception("Failed to read measurements status")
            measurements_txt = "n/a"

        active_tab = ""
        try:
            current = self.notebook.select()
            tab_text = self.notebook.tab(current, "text")
            active_tab = f" | Active: {tab_text}"
        except Exception:
            LOGGER.debug("Could not determine active tab", exc_info=True)
        composed = (
            f"Stepper: {stepper_txt} | Reader: {reader_txt} | Measurements: {measurements_txt}{active_tab}"
        )
        if composed != getattr(self, "_last_status_text", None):
            LOGGER.debug("Status bar text -> %s", composed)
            self.status_var.set(composed)
            self._last_status_text = composed


def main() -> None:
    """Entrypoint used by both CLI execution and packaging scripts."""

    LOGGER.info("Starting combined application")
    app = MainApp()
    app.mainloop()
    LOGGER.info("Application closed")


if __name__ == "__main__":
    main()
