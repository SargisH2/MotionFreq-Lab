# MotionFreq Lab

Desktop Tkinter application that combines GRBL stepper motor control with VW
frequency acquisition and measurement sweeps. Designed for laboratory use with
real hardware or simulation.

## Features
- Stepper Motor panel for GRBL control (ports, jog, homing, move, status).
- Frequency Reader panel for VW controller handshake and frame capture.
- Measurements tab to sweep the motor while logging frequencies.
- Excel export for measurements and CSV export for frequency frames.
- Simulation modes for DAQ and motor to support UI testing.

## Quick start
1. Install Python 3.10+.
2. Install dependencies:
   - `pip install -r requirements.txt`
3. Run the combined app:
   - `python main.py`

Optional standalone panels:
- `python stepper_motor/main.py`
- `python read_freq/main.py`

## Configuration
- `.env` controls simulation flags:
  - `USE_SIMULATION_DAQ=1`
  - `USE_SIMULATION_MOTOR=0`
- `config/motor.json` stores motor defaults (ports, feeds, homing values).

## Project layout
- `main.py`: Combined UI entry point.
- `stepper_motor/`: GRBL motor panel.
- `read_freq/`: VW frequency controller panel and protocol.
- `tabs/`: Measurements tab with DAQ and motor sweep logic.
- `hardware/`: Motor and DAQ backends.
- `models/`: In-memory measurement buffers.
- `export/`: Excel export helper.

## License
This project is proprietary and requires prior written approval for any use.
See `LICENSE`.
