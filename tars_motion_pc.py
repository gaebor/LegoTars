"""Manual motor jogs and bounded sweeps for the EV3 robot."""

import argparse
import asyncio
import json
import queue
import re
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from tars_bluetooth import BluetoothHub, known_devices
from tars_control_pc import Client

MOTOR_POWER = 100  # Constant duty magnitude (%) for every jog and sweep pulse.
MOTOR_LEASE_MS = 100
TURN_LIMIT_MOTOR_DEG = 90 * 56 / 8
SETTLE_SPEED_DEG_S = 3
LIMITS_VERSION = 2


def serialize_limits(limits):
    return {"_version": LIMITS_VERSION, **limits}


def command_duties(directions):
    return {port: direction * MOTOR_POWER for port, direction in directions.items() if direction}


def parse_jog(line, motor_ports):
    command = line.strip().upper()
    if command in ("SWEEP", "STOP"):
        return command.lower(), {}
    zero = re.fullmatch(r"(?:ZERO\s+([BC])|([BC])\s+ZERO)", command)
    if zero:
        port = zero.group(1) or zero.group(2)
        if port not in motor_ports:
            raise ValueError(f"No connected motor at {port}")
        return "zero_port", {port: None}
    mark = re.fullmatch(r"(?:(MIN|MAX)\s+([ABCD])|([ABCD])\s+(MIN|MAX))", command)
    if mark:
        first_kind, first_port, second_port, second_kind = mark.groups()
        kind = first_kind or second_kind
        port = first_port or second_port
        if port not in motor_ports:
            raise ValueError(f"No connected motor at {port}")
        if port in "BC":
            raise ValueError("B/C use the fixed turntable range; only mark A/D limits")
        return kind.lower(), {port: None}
    clear_port = re.fullmatch(r"CLEAR\s+([AD])", command)
    if clear_port:
        port = clear_port.group(1)
        if port not in motor_ports:
            raise ValueError(f"No connected motor at {port}")
        return "clear_port", {port: None}
    clear = re.fullmatch(r"CLEAR\s+(MIN|MAX)\s+([ABCD])", command)
    if clear:
        kind, port = clear.groups()
        if port not in motor_ports:
            raise ValueError(f"No connected motor at {port}")
        if port in "BC":
            raise ValueError("B/C use the fixed turntable range")
        return "clear_" + kind.lower(), {port: None}
    directions = {}
    for item in re.split(r"[\s,]+", command):
        if len(item) != 2 or item[0] not in "ABCD" or item[1] not in "+-":
            raise ValueError("Use A+ or A-; combine motors with spaces, such as A+ C-")
        port, value = item[0], item[1]
        if port not in motor_ports:
            raise ValueError(f"No connected motor at {port}")
        if value == "+":
            directions[port] = 1
        else:
            directions[port] = -1
    return "jog", directions


def read_commands(command_queue):
    while True:
        try:
            command_queue.put(input("> "))
        except EOFError:
            command_queue.put("STOP")
            return


async def run_program(client, motor_ports, args, output):
    while client.latest is None:
        await asyncio.sleep(0.01)
        if client.failure:
            raise client.failure

    command_queue = queue.Queue()
    threading.Thread(target=read_commands, args=(command_queue,), daemon=True).start()
    phase = "calibrate"
    limits_path = output.parent / "motor_limits.json"
    # Old saved A/D bounds must not block manual recalibration jogs.
    limits = {port: {"min": None, "max": None} for port in motor_ports}
    marked_this_session = {port: set() for port in motor_ports.intersection("AD")}
    turn_ports = sorted(motor_ports.intersection("BC"))
    zeroed_this_session = set()
    turn_limit = int(TURN_LIMIT_MOTOR_DEG)
    configured = {
        port: bounds
        for port, bounds in limits.items()
        if port in "AD" and (bounds["min"] is not None or bounds["max"] is not None)
    }
    if configured:
        await client.request("set_motor_limits", limits=configured)
    (output / "motor_limits.json").write_text(
        json.dumps(serialize_limits(limits), indent=2), encoding="utf-8"
    )
    order = []
    active_index = 0
    leg = 0
    target = 0
    boundary_wait = False
    pulse_off = False
    last_jog_directions = None
    print(
        f"Jog with A+/A- etc. at {MOTOR_POWER}% for at most {MOTOR_LEASE_MS} ms. "
        "A/D jog signs follow the rod direction."
    )
    if motor_ports.intersection("AD"):
        print(
            "For A/D, MIN means the rod's lower-angle endpoint; MAX means its higher-angle endpoint."
        )
        print("Record them with MIN A / MAX A (likewise D), regardless of encoder sign.")
    print("Jog B/C to their intended turntable zero pose, then enter ZERO B or ZERO C.")
    print("Each ZERO command sets that motor's encoder to 0 and enables its +/-90 degree limits.")
    print("Saved A/D limits are inactive during calibration; fresh marks arm them for sweep.")
    print("Press Enter on an empty line to repeat the last jog; marks and mode changes clear it.")
    print("Enter SWEEP when A/D endpoints are calibrated. STOP halts in place.")

    loop = asyncio.get_running_loop()
    period = args.control_ms / 1000
    next_tick = loop.time() + period
    while True:
        await asyncio.sleep(max(0, next_tick - loop.time()))
        now = loop.time()
        late = max(0, int((now - next_tick) // period))
        next_tick += (late + 1) * period
        if client.failure:
            raise client.failure
        if client.latest_received_ns and (
            time.perf_counter_ns() - client.latest_received_ns > 250_000_000
        ):
            raise TimeoutError("Telemetry became stale; motor commands will stop")

        sample = client.latest
        if phase == "calibrate":
            while True:
                try:
                    line = command_queue.get_nowait()
                except queue.Empty:
                    break
                try:
                    if not line.strip():
                        if last_jog_directions is None:
                            continue
                        action, values = "jog", last_jog_directions.copy()
                    else:
                        action, values = parse_jog(line, motor_ports)
                        if action != "jog":
                            last_jog_directions = None
                    if action == "jog":
                        result = await client.request(
                            "replace_motors", duties=command_duties(values)
                        )
                        last_jog_directions = values.copy()
                        limited = result.get("limited", [])
                        blocked = sorted(set(values).intersection(limited))
                        if blocked:
                            print(
                                "Blocked by saved encoder limits on "
                                + ", ".join(blocked)
                                + ". Use CLEAR A or CLEAR D to jog freely for recalibration."
                            )
                        else:
                            print(f"Applied {values} for up to {MOTOR_LEASE_MS} ms.")
                    elif action == "zero_port":
                        port = next(iter(values))
                        if abs(sample[port]["speed_deg_s"]) > SETTLE_SPEED_DEG_S:
                            raise ValueError("Wait for the motor to stop before zeroing it")
                        await client.request("replace_motors", duties={})
                        zero_result = await client.request("zero", ports=[port])
                        while client.latest["index"] <= zero_result["sequence"]:
                            await asyncio.sleep(0.01)
                            if client.failure:
                                raise client.failure
                        sample = client.latest
                        limits[port] = {"min": -turn_limit, "max": turn_limit}
                        await client.request("set_motor_limits", limits={port: limits[port]})
                        limits_path.write_text(
                            json.dumps(serialize_limits(limits), indent=2), encoding="utf-8"
                        )
                        (output / "motor_limits.json").write_text(
                            json.dumps(serialize_limits(limits), indent=2), encoding="utf-8"
                        )
                        zeroed_this_session.add(port)
                        print(
                            f"Zeroed {port}; limits are +/-{turn_limit} motor degrees "
                            "(+/-90 degrees at the turntable)."
                        )
                    elif action in ("min", "max"):
                        port = next(iter(values))
                        if abs(sample[port]["speed_deg_s"]) > SETTLE_SPEED_DEG_S:
                            raise ValueError("Wait for the motor to stop before recording a limit")
                        await client.request("replace_motors", duties={})
                        encoder_bound = "max" if action == "min" else "min"
                        limits[port][encoder_bound] = sample[port]["angle_deg"]
                        marked_this_session[port].add(action)
                        await client.request("set_motor_limits", limits={port: limits[port]})
                        limits_path.write_text(
                            json.dumps(serialize_limits(limits), indent=2), encoding="utf-8"
                        )
                        (output / "motor_limits.json").write_text(
                            json.dumps(serialize_limits(limits), indent=2), encoding="utf-8"
                        )
                        print(
                            f"Recorded physical {port} {action.upper()} at "
                            f"{sample[port]['angle_deg']:.0f} encoder degrees."
                        )
                    elif action in ("clear_min", "clear_max"):
                        port = next(iter(values))
                        physical_key = action.removeprefix("clear_")
                        encoder_key = "max" if physical_key == "min" else "min"
                        await client.request("replace_motors", duties={})
                        limits[port][encoder_key] = None
                        marked_this_session[port].discard(physical_key)
                        await client.request("set_motor_limits", limits={port: limits[port]})
                        limits_path.write_text(
                            json.dumps(serialize_limits(limits), indent=2), encoding="utf-8"
                        )
                        (output / "motor_limits.json").write_text(
                            json.dumps(serialize_limits(limits), indent=2), encoding="utf-8"
                        )
                        print(f"Cleared physical {port} {physical_key.upper()} limit.")
                    elif action == "clear_port":
                        port = next(iter(values))
                        await client.request("replace_motors", duties={})
                        limits[port] = {"min": None, "max": None}
                        marked_this_session[port].clear()
                        await client.request("set_motor_limits", limits={port: limits[port]})
                        limits_path.write_text(
                            json.dumps(serialize_limits(limits), indent=2), encoding="utf-8"
                        )
                        (output / "motor_limits.json").write_text(
                            json.dumps(serialize_limits(limits), indent=2), encoding="utf-8"
                        )
                        print(f"Cleared both {port} limits; jog it to recalibrate.")
                    elif action == "sweep":
                        missing = [
                            port
                            for port, marks in marked_this_session.items()
                            if marks != {"min", "max"}
                        ]
                        missing_zero = sorted(set(turn_ports) - zeroed_this_session)
                        if missing:
                            raise ValueError(
                                "Record both MIN and MAX during this session for: "
                                + ", ".join(missing)
                            )
                        if missing_zero:
                            raise ValueError(
                                "Set the turntable zero during this session with ZERO "
                                + " and ZERO ".join(missing_zero)
                            )
                        await client.request("set_motor_limits", limits=limits)
                        order = sorted(motor_ports)
                        active_index = 0
                        leg = 0
                        target = limits[order[0]]["min"]
                        boundary_wait = False
                        pulse_off = False
                        phase = "sweep"
                        print("Starting endpoint sweep; limits are enforced on the brick.")
                        for port, bounds in limits.items():
                            print(
                                f"{port}: encoder {bounds['min']:.0f}.."
                                f"{bounds['max']:.0f} degrees"
                            )
                        break
                    elif action == "stop":
                        await client.request("replace_motors", duties={})
                        return
                except Exception as error:
                    print(f"Invalid command: {error}")
            if phase == "calibrate":
                continue

        if phase == "sweep":
            while True:
                try:
                    line = command_queue.get_nowait()
                except queue.Empty:
                    break
                if line.strip().upper() == "STOP":
                    await client.request("replace_motors", duties={})
                    last_jog_directions = None
                    phase = "calibrate"
                    boundary_wait = False
                    pulse_off = False
                    print("Sweep halted in place. Calibration jog mode.")
                    break
            if phase != "sweep":
                continue

            port = order[active_index]
            position = sample[port]["angle_deg"]
            speed = sample[port]["speed_deg_s"]
            directions = {}
            if boundary_wait:
                if abs(speed) <= SETTLE_SPEED_DEG_S:
                    boundary_wait = False
                    leg += 1
                    if leg >= 3:
                        active_index = (active_index + 1) % len(order)
                        leg = 0
                    port = order[active_index]
                    target = limits[port][("min", "max", "min")[leg]]
                    print(f"Boundary reached; driving toward {port} {target:.0f}.")
            else:
                low, high = limits[port]["min"], limits[port]["max"]
                if leg in (0, 2):
                    encoder_direction = 1 if position < low else -1
                else:
                    encoder_direction = -1 if position > high else 1
                directions[port] = -encoder_direction if port in "AD" else encoder_direction

            if boundary_wait or not directions:
                await client.request("replace_motors", duties={})
                pulse_off = False
            elif pulse_off:
                await client.request("replace_motors", duties={})
                pulse_off = False
            else:
                result = await client.request("replace_motors", duties=command_duties(directions))
                pulse_off = True
                if port in result.get("limited", []):
                    boundary_wait = True
                    await client.request("replace_motors", duties={})


async def session(args):
    address = args.address
    if not address:
        matches = [d for d in known_devices() if "EV3" in d["name"].upper() and d["paired"]]
        if len(matches) != 1:
            raise RuntimeError("Pair one EV3 or pass --address")
        address = matches[0]["address"]
    hub = BluetoothHub(address)
    client = Client(hub)
    receive_task = None
    output = Path(args.output) / datetime.now().strftime("%Y%m%dT%H%M%S.%f")
    output.mkdir(parents=True, exist_ok=False)
    try:
        await hub.connect()
        await hub.run()
        ready = json.loads(await asyncio.wait_for(hub.read_line(), 60))
        if ready != {"ready": "ev3-tars-control-7"}:
            raise RuntimeError("Upload the current tars_control_brick.py and run it on the EV3")
        receive_task = asyncio.create_task(client.receive())
        devices = (await client.request("hello"))["devices"]
        client.configure_columns(devices)
        motor_ports = {p for p in "ABCD" if devices.get(p, {}).get("type") == "motor"}
        if not motor_ports:
            raise RuntimeError("No connected motors found")
        (output / "devices.json").write_text(json.dumps(devices, indent=2), encoding="utf-8")
        await client.request("stream")
        await run_program(client, motor_ports, args, output)
    except KeyboardInterrupt:
        print("Stopped in place. Start calibration again before sweeping.")
    finally:
        try:
            await asyncio.wait_for(client.request("stop"), 2)
        except Exception:
            pass
        try:
            await asyncio.wait_for(hub.stop_user_program(), 2)
        except Exception:
            pass
        if receive_task:
            receive_task.cancel()
            try:
                await receive_task
            except asyncio.CancelledError:
                pass
        await hub.disconnect()
        array = np.asarray(client.matrix, dtype=np.int32).reshape((-1, len(client.columns)))
        np.save(output / "measurements.npy", array)
        (output / "columns.json").write_text(
            json.dumps(client.columns, indent=2), encoding="utf-8"
        )
        print(f"Saved {len(array)} measurements to {output / 'measurements.npy'}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address")
    parser.add_argument("--output", default="results/tars_motion")
    parser.add_argument("--control-ms", type=int, default=100)
    args = parser.parse_args()
    if not 10 <= args.control_ms <= 1000:
        parser.error("--control-ms must be from 10 to 1000")
    try:
        asyncio.run(session(args))
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
