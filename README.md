# LegoTars

Pybricks EV3 programs for infrared remote driving and PC controlled measurement and motor control.

## Programs

- `tars_ir_control.py` runs on the EV3 and maps the EV3 infrared remote's four channels to motors A-D. Edit `IR_PORT`, `CHANNELS`, and `POWER` near the top to match the robot.
- `tars_control_brick.py` runs on the EV3. It discovers motors and sensors, reports motor angle and speed and gyro angle and speed on a 10 ms sampling tick, and accepts leased motor duty commands.
- `tars_control_pc.py` runs on Windows. It connects over Bluetooth Classic, saves measurements as a NumPy array, and sends motor commands on a configurable PC control tick. The `tars_bluetooth.py` module provides its Bluetooth transport.
- `describe.py` summarizes the columns in the newest capture, including means, standard deviations, ranges, and the first and last nonzero values.

## Setup

Install Python 3.13 and [uv](https://docs.astral.sh/uv/). From this directory, sync the PC environment:

```powershell
uv sync
```

Upload `tars_control_brick.py` to the EV3 with Pybricks Code and start it. Pair the EV3 with Windows, then run:

```powershell
uv run tars_control_pc.py
```

The PC asks for manual motor duties in the terminal. For example, enter `A=20,B=-10`; negative duty reverses direction. Omitted motors are left alone, and `0` coasts the selected motor. Nonzero commands are renewed on each PC tick and expire to a stop after 250 ms if renewal stops. Press Ctrl+C to stop the session and save the capture.

Change the PC command tick with `--control-ms` (10 to 1000 ms), and optionally provide a controller module defining `control(sample) -> {port: duty}`:

```powershell
uv run tars_control_pc.py --control-ms 50 --controller controller.py
```

The controller receives the newest sample, with readings such as `sample["A"]["angle_deg"]` and `sample["S3"]["speed_deg_s"]`. The function runs once per PC tick and should return only the connected motor ports it wants to command.

## Captures

By default, captures are saved under `results/tars_control/<timestamp>/`. `measurements.npy` is a float64 matrix. `columns.json` names the columns: sample index, brick elapsed time, then angle and speed for each discovered motor and gyro. The index makes skipped sampling sequence numbers visible. Additional CSV files record PC control tick timing and command acknowledgments.

Describe the newest capture with:

```powershell
uv run describe.py
```

Pass a specific capture directory using `--capture`, or change the search root using `--measurement`.

The brick's 10 ms sampler and PC controller are best effort, not hard real time; Bluetooth transfer, sensor refresh, firmware scheduling, and motor mechanics affect observed timing. The motor watchdog stops a powered motor when its 250 ms command lease expires.
