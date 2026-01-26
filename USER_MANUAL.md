# User Manual

## What this app is
This project is a desktop GUI that combines three tools in one window:
- Stepper Motor panel for GRBL-based motion control.
- Frequency Reader panel for the VW frequency controller.
- Measurements tab that sweeps the motor and captures frequencies over time or position.

It is designed to work with real hardware or with built-in simulators when hardware is not available.

## Where to find things
Entry points:
- `main.py`: Combined app with all three tabs.
- `stepper_motor/main.py`: Standalone motor control panel.
- `read_freq/main.py`: Standalone frequency reader panel.
- `test/test.py`: CLI tester for the frequency controller protocol.

Configuration and data:
- `.env`: Simulation flags and environment settings.
- `config/motor.json`: Saved motor defaults (ports, feed, homing values).
- `requirements.txt`: Python dependencies.
- `tst.xlsx`: Example export file (if present).
- `grbl_help.json`: Reference information for GRBL commands.

Exports:
- Frequency Reader tab: CSV export from the log of frames.
- Measurements tab: Excel export with Plot1, Plot2, Plot3 sheets.

## How to run
1. Install Python 3.10 or newer.
2. Install dependencies:
   - `pip install -r requirements.txt`
3. Run the combined app:
   - `python main.py`

Optional: run a single panel
- Motor only: `python stepper_motor/main.py`
- Frequency only: `python read_freq/main.py`

## Quick start workflow
### Stepper Motor tab
1. Select a serial port and baud rate.
2. Click Connect.
3. Use the jog buttons for X and Y.
4. Use the homing buttons to capture and return to home.
5. Use Move to go to specific coordinates.

### Frequency Reader tab
1. Select a serial port and set Mode and K.
2. Click Connect.
3. The panel starts reading frames automatically.
4. Click Stop and Export CSV to save data.

### Measurements tab
1. Connect the motor (top row).
2. Connect the DAQ (second row).
3. Set sweep Start, End, and Accel values.
4. Pick a plot and mode:
   - No Motor (Time): capture frequencies over time only.
   - Motor + Direction: sweep from start to end.
   - Motor - Direction: sweep from start to end in the negative direction.
5. Click Save Results to Excel when done.

## How the script works (high level)
- The Stepper Motor tab sends G-code to GRBL using either a direct serial connection or the `MotorController` backend.
- The Frequency Reader tab uses the VW protocol, sends a handshake (mode and k), then reads frames continuously.
- The Measurements tab coordinates a motor sweep and DAQ reads in background threads, then updates plots and buffers. It can run with real hardware or a simulated data source.

## Simulation and environment settings
The app can run in simulation mode to help with UI testing:
- `.env` contains:
  - `USE_SIMULATION_DAQ=1` to simulate frequency readings.
  - `USE_SIMULATION_MOTOR=0` to use real motor hardware.

If simulation is enabled:
- The motor port list includes "Simulated Motor".
- The DAQ port list includes "Simulated".

## Tips and troubleshooting
- If the Frequency Reader says "Controller unavailable", install `pyserial`.
- If the motor will not move, check the selected port and baud rate.
- If homing or status values look wrong, ensure GRBL $10 includes the fields you need (MPos/WPos/Pn).
- Use the Stop button to cancel motion immediately.
