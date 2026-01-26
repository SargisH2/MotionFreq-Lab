# Technical Documentation

## Overview
- Combined Tkinter application with three tabs: Stepper Motor (GRBL), Frequency Reader (VW controller), and Measurements (motor sweep plus DAQ capture).
- Two hardware integrations: GRBL-compatible stepper motor over serial and VW frequency controller over serial.
- Simulation modes exist for both motor and DAQ to allow UI testing without hardware.
- Motor UI defaults persist to `config/motor.json`; environment flags are read from `.env`.
- Measurements can be exported to Excel; frequency frames can be exported to CSV.

## Module Reference

### `main.py`
Purpose: Entry point for the combined GUI and top-level window orchestration.

Functions:
- `_configure_logging`: Configure the root logger once and set noise filters for matplotlib.
- `main`: Create `MainApp` and start the Tk mainloop.

Classes:
- `MainApp` (tk.Tk): Top-level Tk root that owns the notebook and status bar.
  - `__init__`: Configure logging, create the shared motor controller, build the tab notebook, create panels, create a status bar, and wire status updates.
  - `_create_motor_controller`: Try to instantiate `MotorController`; fall back to `None` on failure.
  - `_create_panel`: Build a tab widget, insert it into the notebook, and store it on `self`.
  - `on_close`: Request shutdown on each panel and destroy the window.
  - `_wire_status_updates`: Attach variable traces and tab change listeners to refresh the status bar.
  - `_update_status`: Compose status text from each panel and show the active tab.

### `config/env.py`
Purpose: Load `.env` once and expose helper to read booleans.

Functions:
- `ensure_env_loaded`: Load environment variables from `.env` if present, only once per process.
- `env_bool`: Interpret an environment variable as a boolean with a fallback default.

### `config/motor.py`
Purpose: Persisted defaults for motor-related UI and settings.

Functions:
- `_config_path`: Resolve the filesystem path to `config/motor.json`.
- `load_motor_config`: Load JSON config or return defaults if missing or invalid.
- `save_motor_config`: Write config to disk using a temporary file and atomic replace.

Classes:
- `MotorConfig`: Dataclass for motor UI defaults.
  - `from_mapping`: Normalize a dict into a `MotorConfig`, coercing types.
  - `to_mapping`: Convert to a dict with normalized axis keys.

### `config/__init__.py`
Purpose: Re-export configuration helpers (`env_bool`, `MotorConfig`, etc).

### `ui/motor_shared.py`
Purpose: Shared UI helpers for port lists and selection.

Functions:
- `dedupe_preserve`: Remove duplicates while preserving the original order.
- `prepare_port_list`: Normalize port display names, optionally adding a simulated label.
- `select_port_value`: Choose the best available port given current and preferred values.

Constants:
- `SIMULATED_ALIAS`: Default display label for simulated motor ports.

### `ui/__init__.py`
Purpose: Re-export UI helpers from `ui/motor_shared.py`.

### `models/data_buffers.py`
Purpose: Thread-safe buffers for storing measurement rows.

Classes:
- `MeasurementRow`: Dataclass describing a single measurement sample.
- `MeasurementBuffer`: Thread-safe list wrapper for `MeasurementRow`.
  - `__init__`: Store the plot id and initialize a lock-protected list.
  - `append`: Enforce plot id match and append a row.
  - `clear`: Remove all rows.
  - `__len__`: Return the number of buffered rows.
  - `snapshot`: Return a copy of all rows.
  - `to_dataframe`: Convert to pandas DataFrame (requires pandas).

Types:
- `ModeLiteral`: Measurement mode enum for the UI.
- `PlotIdLiteral`: Plot id enum (1 to 3).

### `models/__init__.py`
Purpose: Re-export data buffer models.

### `export/excel.py`
Purpose: Export measurement buffers to Excel workbooks.

Functions:
- `export_measurements_to_excel`: Build a three-sheet workbook (`Plot1` to `Plot3`) from buffers, ordering and filtering columns for export.

Types:
- `SheetData`: Union of `MeasurementBuffer` or list of `MeasurementRow`.

### `export/__init__.py`
Purpose: Re-export Excel export helper.

### `hardware/motor_types.py`
Purpose: Shared types and validation for motor logic.

Functions:
- `normalise_axis`: Validate and normalize axis labels.

Classes:
- `MotorError`: Exception for motor failures.
- `MotorState`: Dataclass capturing a controller snapshot.

Constants:
- `AXES`: Supported axes tuple.
- `AxisLiteral`: Type alias for axis values.

### `hardware/motor_driver.py`
Purpose: Low-level driver layer that talks to GRBL or simulates it.

Functions:
- `list_serial_ports`: Return available serial port names (empty if pyserial missing).

Classes:
- `MotorDriver` (Protocol): Interface required by higher-level controller.
- `SimulatedMotorDriver`: In-memory motor implementation for testing.
  - `connect`: Mark simulated connection as active.
  - `disconnect`: Reset connection state.
  - `is_connected`: Report connection state.
  - `send_line`: Log commands without serial I/O.
  - `send_raw`: Log raw payloads without serial I/O.
  - `reset_input_buffer`: No-op.
  - `jog_increment`: Log jogs.
  - `register_line_listener`: Return a no-op unregister function.
- `SerialMotorDriver`: Actual GRBL serial implementation.
  - `connect`: Open port, run startup sequence, spawn reader thread.
  - `disconnect`: Stop reader, close port, reset state.
  - `is_connected`: Report connection state.
  - `send_line`: Send a G-code line and optionally wait for "ok".
  - `send_raw`: Send realtime bytes (jog cancel, status query).
  - `reset_input_buffer`: Clear serial input buffer.
  - `jog_increment`: Send incremental jog command.
  - `register_line_listener`: Register callbacks for every line read.
  - `_start_reader`: Create a background thread to read lines.
  - `_stop_reader`: Stop the reader thread and clear queues.
  - `_reader_loop`: Read from serial, emit listeners, and capture "ok/error/alarm" acknowledgements.
  - `_notify_line_listeners`: Call registered callbacks for a line.
  - `_maybe_queue_ack`: Save acknowledgement lines for synchronous waits.
  - `_await_ok`: Block until "ok" or error is received.
  - `_drain_response_queue`: Clear old acknowledgements before a blocking send.
  - `_send_command_blocking`: Send a command and read the response inline.
  - `_initialise_controller`: Send wake sequence and baseline commands ($X, G21, G91).
  - `_write_line`: Serial write wrapper with optional ack wait.
  - `_send_realtime`: Serial write wrapper for realtime bytes.

Constants:
- `SERIAL_AVAILABLE`: True if pyserial import succeeded.

### `hardware/motor_controller.py`
Purpose: Thread-safe, high-level motor control abstraction.

Classes:
- `MotorController`: Orchestrates driver, motion threads, and cached positions.
  - `__init__`: Choose driver, set defaults, initialize state caches.
  - `connect`: Connect the driver to a port.
  - `disconnect`: Stop motion and disconnect driver.
  - `send_line`: Send a G-code line, no-op in simulation.
  - `send_raw`: Send realtime bytes, no-op in simulation.
  - `reset_input_buffer`: Clear serial buffer when available.
  - `home`: Jog to origin and set work zero.
  - `move_positive`: Start continuous motion in positive direction.
  - `move_negative`: Start continuous motion in negative direction.
  - `move_by`: Jog by a delta using incremental movement.
  - `move_to_coordinate`: Move to an absolute coordinate.
  - `move_to_start`: Move to configured soft-limit minimum.
  - `move_to_end`: Move to configured soft-limit maximum.
  - `jog_increment`: One-shot incremental jog with timing estimates.
  - `stop`: Stop any active motion and send jog cancel if needed.
  - `get_position`: Return cached axis position.
  - `get_positions`: Return all cached positions.
  - `set_soft_limits`: Persist optional min and max limits.
  - `set_position`: Override cached position.
  - `is_connected`: Report driver connection state.
  - `is_moving`: Report whether motion thread is active.
  - `is_simulation`: Property exposing simulation mode.
  - `last_error`: Return last recorded error string.
  - `snapshot`: Return a `MotorState` snapshot.
  - `read_status_positions`: Query GRBL status for MPos and WPos.
  - `read_status_position`: Read a single axis from MPos or WPos.
  - `_start_motion`: Start a motion thread (simulation or hardware).
  - `_simulate_motion`: Simulated motion loop updating positions.
  - `_hardware_motion`: Repeated jogs to approximate continuous motion.
  - `_query_status_line`: Send "?" and wait for a status line.
  - `_parse_status_positions`: Extract MPos and WPos from status line.
  - `_extract_status_positions`: Parse axis values from a token.
  - `_apply_increment`: Update cached position for a delta.
  - `_record_error`: Persist and log error messages.
  - `register_line_listener`: Register a callback on raw line input.
  - `_ensure_connected`: Raise if not connected.
  - `_coerce_feed`: Normalize feed values to valid numeric defaults.

Constants:
- `USE_SIMULATION_DEFAULT`: Default simulation mode based on pyserial availability.

### `hardware/daq.py`
Purpose: Data acquisition for the Measurements tab, with simulation fallback.

Classes:
- `DataAcquisitionError`: Exception for DAQ failures.
- `FrequencyDAQ`: Non-blocking frequency reader.
  - `__init__`: Choose simulation/hardware mode and set defaults.
  - `connect`: Connect to DAQ hardware or set simulated port.
  - `disconnect`: Disconnect or reset simulation state.
  - `is_connected`: Report DAQ readiness.
  - `connected_port`: Property with last connected port.
  - `is_simulation`: Property exposing simulation mode.
  - `measurement_configuration`: Return mode and k value.
  - `measurement_gap`: Return minimum delay between triggers.
  - `measurement_window`: Return measurement window duration.
  - `set_measurement_parameters`: Update handshake settings.
  - `read_frequencies`: Return a single pair of channel frequencies.
  - `iter_frames`: Stream raw frames from hardware.
  - `iter_frequencies`: Stream frequency pairs from hardware or simulator.
  - `read_timeout`: Return read timeout in seconds.
  - `set_transport_parameters`: Update baudrate, timeout, calibration, timer MP.
  - `_apply_measurement_configuration`: Push mode and k to controller and update timing.
  - `_get_controller`: Return active controller or raise if missing.
  - `_iter_simulated`: Generator for simulated frequency pairs.
  - `_simulate_sample`: Deterministic pseudo-random sample generator.

Constants:
- `DEFAULT_MODE`, `DEFAULT_K_VALUE`, `DEFAULT_BAUDRATE`, `DEFAULT_CALIBRATION`, `DEFAULT_TIMER_MP`
- `MIN_SAMPLE_INTERVAL`, `USE_SIMULATION_DEFAULT`

### `hardware/__init__.py`
Purpose: Re-export motor and DAQ helpers.

### `read_freq/controller.py`
Purpose: Serial protocol implementation for VW frequency controller.

Classes:
- `ControllerError`: Exception for controller failures.
- `ChannelMeasurement`: Dataclass for a channel payload.
- `FrequencyFrame`: Dataclass containing two channel measurements.
- `VWFrequencyController`: Implements protocol and decoding logic.
  - `__init__`: Store connection and timing parameters.
  - `_log_bytes`: Dispatch raw bytes to optional logger.
  - `_read_available`: Read buffered bytes without blocking.
  - `_reset_like_vb`: Toggle DTR to match legacy VB behavior.
  - `_drain_echo`: Discard small status bursts from the device.
  - `available_ports`: Discover COM ports.
  - `auto_connect`: Try ports until a connection succeeds.
  - `connect`: Open serial port and initialize buffers.
  - `set_bytes_logger`: Register a logger callback for raw bytes.
  - `close`: Close the serial connection.
  - `configure_measurement`: Send mode and k value handshake.
  - `trigger_measurement`: Send single-byte trigger.
  - `iter_frames`: Yield frames continuously or for a count.
  - `read_frame`: Read one frame, searching for header and payload.
  - `_decode_frame`: Parse payload into channel measurements.
  - `_compute_frequency`: Compute frequency using timing formula.
  - `_measurement_window`: Compute time window for capture.
  - `measurement_window_seconds`: Expose measurement window.
  - `measurement_gap_seconds`: Window plus guard time.
  - `extra_delay_seconds`: Additional delay when k is clamped.
  - `requested_k_value`: Property of requested (unclamped) k.
  - `__enter__` / `__exit__`: Context manager for auto-close.

Functions:
- `format_frame`: Human-friendly string for a `FrequencyFrame`.

### `read_freq/main.py`
Purpose: Standalone Tkinter panel to drive the frequency controller.

Classes:
- `FrequencyPanel`: Embeddable panel for DAQ capture.
  - `__init__`: Create UI, verify dependencies, start queue polling.
  - `shutdown`: Stop threads and close controller.
  - `destroy`: Ensure shutdown before widget destruction.
  - `_build_ui`: Build connection, settings, and display widgets.
  - `_handle_missing_dependencies`: Disable UI if controller missing.
  - `_controller_unavailable_message`: Compose missing dependency message.
  - `refresh_ports`: Populate serial port list.
  - `connect`: Create controller, connect, configure, and auto-start.
  - `disconnect`: Stop measurement and reset UI states.
  - `start_measurement`: Spawn worker thread and update UI state.
  - `stop_measurement`: Stop worker thread and restore controls.
  - `stop_measurement_command`: Stop and optionally export CSV.
  - `_measurement_loop`: Iterate frames and enqueue for UI thread.
  - `_poll_queue`: Drain queued events and reschedule polling.
  - `_handle_error`: Show controller errors and log them.
  - `_handle_frame`: Update readouts and history for a frame.
  - `_append_log`: Append log text to the scrolling widget.
  - `_format_bytes_for_log`: Format bytes into hex and printable strings.
  - `export_csv`: Save frame history to CSV.
  - `on_close`: Clean shutdown on window close.
- `ControllerApp`: Wrapper that owns a Tk root for standalone execution.
  - `__init__`: Create root, embed `FrequencyPanel`.
  - `mainloop`: Delegate to root mainloop.
  - `destroy`: Destroy panel and root window.

Functions:
- `main`: Run `ControllerApp` as a standalone program.

### `read_freq/__init__.py`
Purpose: Re-export controller types and provide fallbacks when pyserial is missing.

### `tabs/measurements_tab.py`
Purpose: Measurements tab that coordinates motor movement and DAQ capture.

Classes:
- `PlotRuntime`: Dataclass capturing per-plot runtime state and threads.
- `PlotCard`: UI widget for one plot.
  - `__init__`: Build plot and control buttons.
  - `_on_mode_clicked`: Dispatch mode selection callback.
  - `set_running`: Update button states based on current mode.
  - `reset_plot`: Clear plot data and set axis labels.
  - `append_point`: Append a plotted point and refresh.
  - `_update_lines`: Autoscale and redraw the plot.
  - `update_status`: Update plot status line text.
- `MeasurementsTab`: Main Measurements tab.
  - `__init__`: Initialize DAQ, motor, buffers, UI, and status polling.
  - `_mm_to_mm`: Coerce UI distance value to float.
  - `_build_ui`: Construct all controls, plots, and footer.
  - `refresh_ports`: Refresh motor and DAQ port lists.
  - `_sync_motor_controls_from_config`: Update axis and feed defaults from config.
  - `connect_motor`: Connect motor using selected port and config baud.
  - `disconnect_motor`: Stop motor and disconnect, clearing state.
  - `connect_daq`: Connect DAQ using current settings.
  - `disconnect_daq`: Disconnect DAQ unless in simulation mode.
  - `_on_daq_params_changed`: Apply DAQ parameter edits.
  - `_apply_daq_params`: Validate and send mode/k settings to DAQ.
  - `_sanitize_speed_value`: Clamp motor feed to a positive value.
  - `_on_speed_changed`: Normalize and persist feed to config.
  - `_sanitize_int`: Clamp IntVar to specified bounds.
  - `_get_entry_int`: Parse an integer entry field with fallback.
  - `_get_entry_float`: Parse a float entry field with fallback.
  - `_capture_motor_config`: Copy UI values into `MotorConfig`.
  - `_persist_motor_config`: Debounced persistence of motor config to disk.
  - `_get_combo_values`: Return list values from a combobox.
  - `_update_connection_buttons`: Enable or disable motor connect buttons.
  - `_update_daq_buttons`: Enable or disable DAQ connect buttons.
  - `_ensure_daq_connected`: Validate transport settings and connect DAQ.
  - `home_motor`: Home the selected axis.
  - `stop_motor`: Stop motor and any active measurement.
  - `_toggle_measurement`: Start or stop a measurement for a plot and mode.
  - `_start_measurement`: Initialize runtime state and spawn acquisition threads.
  - `_coord_system`: Return "machine" or "work" based on UI selection.
  - `_read_sweep_settings`: Parse sweep start, end, accel, and coord system.
  - `_wait_for_sweep_start`: Block until sweep start signal or stop event.
  - `_read_axis_position`: Read axis position from GRBL status.
  - `_has_reached_end`: Determine if sweep has reached end position.
  - `_build_move_command`: Build G1 move command for machine or work coords.
  - `_apply_axis_settings`: Send $110/$111 and $120/$121 for axis tuning.
  - `_estimate_move_timeout`: Estimate timeout based on distance and feed.
  - `_wait_for_target_position`: Poll until axis reaches target or timeout.
  - `_simulate_axis_move`: Update simulated positions over time.
  - `_motor_sweep`: Perform sweep movement and signal acquisition start.
  - `_stop_measurement`: Stop threads and reset runtime state.
  - `_acquisition_loop`: Read DAQ values and record measurements in background.
  - `_record_measurement`: Store measurement row and schedule UI update.
  - `_update_plot`: Update plot display and status text.
  - `_make_plot_update`: Create a closure to update the plot on UI thread.
  - `_report_error`: Show measurement errors to the operator.
  - `_poll_status`: Update the status banner and buttons on a timer.
  - `export_results`: Export all buffers to Excel.
  - `shutdown`: Stop measurements and release hardware on close.
  - `_bind_mousewheel`: Attach mousewheel handlers for canvas scrolling.
  - `_unbind_mousewheel`: Detach mousewheel handlers.
  - `_on_mousewheel`: Scroll the canvas.

### `tabs/__init__.py`
Purpose: Re-export `MeasurementsTab`.

### `stepper_motor/main.py`
Purpose: GRBL stepper motor control panel and UI logic.

Functions:
- `now_ms`: Return current time in milliseconds.

Classes:
- `GRBLInterface`: Shared UI and control logic for GRBL interactions.
  - `__init__`: Build the UI, configure backend, initialize queues, and start pollers.
  - `is_connected`: Report connection state for backend or serial.
  - `_list_ports`: Enumerate serial ports via backend or pyserial.
  - `refresh_ports`: Update the UI combobox with available ports.
  - `log`: Queue a line for UI logging.
  - `_flush_ui_log`: Drain queued log messages into the text widget.
  - `clear_logs`: Clear log view and queued messages.
  - `set_state`: Update the visible connection status.
  - `connect`: Connect to motor via backend or direct serial, then initialize.
  - `reconnect`: Disconnect and reconnect to reset GRBL state.
  - `disconnect`: Stop workers and close serial or backend connection.
  - `start_reader`: Start the serial reader thread (direct serial mode).
  - `reader_loop`: Read serial lines and update queues/state.
  - `_handle_motor_line`: Handle raw line callbacks from backend driver.
  - `_release_motor_listener`: Unregister backend line listener.
  - `_apply_default_port_selection`: Choose best port from defaults and UI.
  - `_bind_persist`: Attach config persistence to UI edit events.
  - `_read_float_entry`: Parse float from an entry with fallback.
  - `_read_int_entry`: Parse int from an entry with fallback.
  - `_capture_ui_into_config`: Copy UI values into MotorConfig.
  - `_persist_motor_config`: Save MotorConfig to disk.
  - `_schedule_persist_motor_config`: Debounce config writes.
  - `_motor_backend`: Return backend instance or None.
  - `_update_axis_length_display`: Update UI for measured axis length.
  - `_update_port_defaults`: Persist detected port and baud to config.
  - `_steps_per_mm`: Read $100/$101 steps per mm or fallback.
  - `_ui_distance_to_mm`: Convert UI distances to mm.
  - `_ui_feed_to_mm`: Convert UI feed to mm/min.
  - `_wait_for_motion_settle`: Poll status until motion is idle.
  - `safe_send_raw`: Send realtime bytes with error handling.
  - `safe_send`: Send a G-code line without waiting for "ok".
  - `_send_line_wait_ok`: Send and wait for "ok" using backend or tail queue.
  - `_maybe_enable_hard_limits`: Ensure $21=1 when safe to do so.
  - `send_from_entry`: Send the G-code line in the entry widget.
  - `get_mpos`: Read MPos from status or backend.
  - `_capture_home`: Store current MPos as home for axis and set WCS zero.
  - `capture_x_home`: Capture X home position.
  - `capture_y_home`: Capture Y home position.
  - `_get_home_target`: Return stored home target for axis.
  - `_set_home_target`: Store home target for axis.
  - `_go_home`: Move to stored home position using jogs.
  - `go_x_home`: Move to X home.
  - `go_y_home`: Move to Y home.
  - `soft_reset`: Send jog cancel and Ctrl-X, reset buffers.
  - `wait_for_startup`: Wait for GRBL banner or reset messages.
  - `unlock_and_prepare`: Send $X and wait for ok/idle.
  - `query_status`: Send realtime status query ("?").
  - `_motor_task_loop`: Worker thread to run backend tasks off the UI thread.
  - `_start_motor_worker`: Start the worker thread.
  - `_stop_motor_worker`: Stop the worker thread and clear tasks.
  - `_queue_motor_task`: Queue a backend action to the worker thread.
  - `_send_status_query`: Send a status query via backend or serial.
  - `_status_poll_loop`: Poll status on a fixed interval.
  - `_start_status_polling`: Start the status polling thread.
  - `_stop_status_polling`: Stop the status polling thread.
  - `_parse_status_line`: Parse GRBL status fields (state, MPos, WPos, WCO).
  - `_grbl_state`: Return the parsed GRBL state string.
  - `_wait_for_idle`: Wait until GRBL reports Idle.
  - `_poll_status_and_update_ui`: Update UI position labels and state.
  - `move_to_coords`: Build and send a move based on UI values.
  - `wait_for_alarm`: Wait for ALARM event to trigger.
  - `wait_for_ok_or_idle`: Wait for ok or idle in recent lines.
  - `get_pn_axes`: Read Pn: switch state from status.
  - `_pn_axes_from_last_status`: Parse Pn: from last status line.
  - `jog_inc`: Issue incremental jog and update measurement spans.
  - `manual_jog`: Read UI jog params and call `jog_inc`.
  - `move_until_alarm`: Repeatedly jog until a limit alarm or switch is hit.
  - `clear_limit_with_retries`: Step away from a limit switch and re-enable $21.
  - `ensure_status_mask_with_pn`: Ensure $10 mask includes Pn bits.
  - `restore_status_mask`: Restore previous $10 mask value.
  - `read_settings`: Send $$, parse responses, and populate UI fields.
  - `apply_speed_accel`: Apply $110/$111/$120/$121 from UI fields.
  - `start_home_x`: Spawn homing thread for X.
  - `start_home_y`: Spawn homing thread for Y.
  - `start_home_xy`: Spawn homing thread for X and Y.
  - `request_stop`: Stop jogs or backend motion immediately.
  - `_recover_after_alarm`: Reconnect and unlock after an ALARM.
  - `_home_sequence`: Execute the full homing flow for selected axes.
  - `go_mid`: Move to the measured midpoint of the axis span.
  - `_home_one_axis_release`: Seek both limits and compute span using releases.
- `StepperPanel`: Embeddable wrapper around `GRBLInterface`.
  - `__init__`: Build the panel and initialize `GRBLInterface`.
  - `shutdown`: Stop motion, persist config, disconnect, release listener.
  - `destroy`: Shutdown and destroy widget.

Functions:
- `main`: Launch a standalone motor panel with its own Tk root.

### `stepper_motor/__init__.py`
Purpose: No module in this repository (package only contains `main.py`).

### `read_freq/test` files
Purpose: CLI and duplicate controller used for testing.

`test/test.py` functions:
- `hexdump`: Convert bytes to hex string.
- `build_parser`: Build CLI argument parser for testing.
- `main`: Connect to controller, configure handshake, and print frames.

`test/controller.py`:
- Duplicate of `read_freq/controller.py` used for CLI test isolation.
- Functions and classes match the controller module above.

### Data and config files
- `config/motor.json`: Persisted motor defaults (ports, feed, homing values).
- `.env`: Environment overrides (`USE_SIMULATION_DAQ`, `USE_SIMULATION_MOTOR`, etc).
- `requirements.txt`: Python dependencies needed to run the app.
- `grbl_help.json`: Reference JSON for GRBL-related help content.
- `tst.xlsx`: Example Excel output file.
