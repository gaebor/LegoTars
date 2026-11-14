# LegoTars

Pybricks EV3 programs for infrared remote driving and PC controlled measurement and motor control.

## Programs

- `tars_ir_control.py` runs on the EV3 and maps the EV3 infrared remote's four channels to motors A-D. Edit `IR_PORT`, `CHANNELS`, and `POWER` near the top to match the robot.
- `tars_control_brick.py` runs on the EV3. It discovers motors and sensors, reports motor angle and speed and gyro angle and speed on a 10 ms sampling tick, and accepts leased motor duty commands.
- `tars_control_pc.py` runs on Windows. It connects over Bluetooth Classic, saves measurements as a NumPy array, and sends motor commands on a configurable PC control tick. The `tars_bluetooth.py` module provides its Bluetooth transport.
- `tars_motion_pc.py` provides manual motor jog calibration and bounded motor sweeps.
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

The PC asks for manual motor duties in the terminal. For example, enter `A=20,B=-10`; negative duty reverses direction. Duty signs use the calibrated direction convention, including A/D polarity. Omitted motors are left alone, and `0` coasts the selected motor. Nonzero commands are renewed on each PC tick and expire to a stop after 100 ms if renewal stops. If calibrated motor limits exist, the brick enforces them for this controller too. Press Ctrl+C to stop the session and save the capture.

Change the PC command tick with `--control-ms` (10 to 1000 ms), and optionally provide a controller module defining `control(sample) -> {port: duty}`:

```powershell
uv run tars_control_pc.py --control-ms 50 --controller controller.py
```

The controller receives the newest sample, with readings such as `sample["A"]["angle_deg"]` and `sample["S3"]["speed_deg_s"]`. The function runs once per PC tick and should return only the connected motor ports it wants to command.

## Captures

By default, captures are saved under `results/tars_control/<timestamp>/`. `measurements.npy` is an int32 matrix. Its first four columns are `arrival_time_pc_ms` (PC monotonic time relative to the first packet), `packet_index`, brick sample `index`, and `brick_time_ms`; angle and speed columns for each discovered motor and gyro follow. Rows from the same telemetry packet share the PC arrival timestamp and packet index. Missing sensor values use the int32 minimum as a sentinel. The sample index makes skipped sampling ticks visible. Additional CSV files record PC control tick timing and command acknowledgments.

Describe the newest capture with:

```powershell
uv run describe.py
```

Pass a specific capture directory using `--capture`, or change the search root using `--measurement`.

The brick's 10 ms sampler and PC controller are best effort, not hard real time; Bluetooth transfer, sensor refresh, firmware scheduling, and motor mechanics affect observed timing. The motor watchdog stops a powered motor when its 100 ms command lease expires.

## Calibration and bounded sweeps

Upload the current `tars_control_brick.py` to the EV3, then start the combined calibration and sweep program:

```powershell
uv run tars_motion_pc.py
```

Starting or restarting the programs does not reset motor encoders. Saved A/D limits are inactive during manual calibration, so old bounds cannot block recalibration; fresh marks arm the limits for sweep. Jog B/C to the intended turntable zero pose, wait for the motor to stop, then enter `ZERO B` or `ZERO C`; only that explicit command resets the selected motor. It also enables that motor's ±630 motor-degree limit (±90° at the 8:56 gear ratio). A/D encoder positions are left untouched. `MIN`/`MAX` record the current A/D angles without resetting them. Jog with commands such as `A+` or `B-`; combine motors with spaces, such as `A+ C-`. A+ and A− (likewise D+/D−) have the same calibrated rod-direction meaning in jogs, sweeps, and PC controllers. Each jog is a 100% pulse that expires after 100 ms and brakes. Re-enter a jog command for another pulse, or press Enter on a blank line to repeat the last jog. Recording or clearing a limit and changing modes clears the repeat command.

For each of A/D, jog to a safe position at the rod's physically lower-angle endpoint, wait for it to stop, then enter `MIN A` (or `A MIN`, likewise D). Jog to the physically higher-angle endpoint and enter `MAX A` (or `A MAX`). The script translates these labels to encoder limits using the calibrated A/D direction; you do not need to reason about encoder sign. Mark positions inside the mechanical hard stops with room for coasting. B/C need no endpoint marks. The active calibration is saved in `results/tars_motion/motor_limits.json` and copied into each capture directory. It is loaded by both motion and general PC control sessions; recalibrate if the brick resets encoder angles. To jog freely while recalibrating, enter `CLEAR A` or `CLEAR D` to remove that motor's saved limits, then record both endpoints again. To clear only one saved endpoint, use `CLEAR MIN A` or `CLEAR MAX A`.

Enter `SWEEP` after recording both A/D endpoints and zeroing each connected B/C during the current session; saved calibration alone does not qualify. The program pulses toward each endpoint, brakes between 100 ms pulses, and waits for momentum to settle before reversing toward a target. It sweeps one motor from minimum to maximum and back before moving to the next. `STOP` halts in place and returns to calibration; `STOP` during calibration exits. Ctrl+C stops in place.

The brick independently enforces each recorded encoder limit: each command and the 5 ms motor task check the active direction and brake a motor if it is at or beyond a limit and still being driven outward. All PC motor duties use the same calibrated sign convention; A/D polarity is translated on the brick. The limits describe the safe positions you recorded, not the mechanical hard stops, so record them with enough clearance for the motor's remaining momentum. The motor lease is 100 ms.

Calibration and sweep captures, including the PC arrival timestamp and packet index columns, are saved under `results/tars_motion/`.
