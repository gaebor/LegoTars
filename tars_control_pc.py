"""Tick-based EV3 measurement capture and PC-side control loop."""

import argparse
import asyncio
import csv
import importlib.util
import json
import math
import sys
import threading
import queue
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from pc_agent import BluetoothHub, known_devices


class Client:
    def __init__(self, hub):
        self.hub = hub
        self.pending = {}
        self.next_id = 0
        self.columns = ["index", "time"]
        self.column_specs = []
        self.matrix = []
        self.failure = None
        self.latest = None
        self.latest_received_ns = None
        self.command_log = []
        self.control_log = []
        self.running = True

    async def receive(self):
        try:
            while True:
                msg = json.loads(await self.hub.read_line())
                received_ns = time.perf_counter_ns()
                if "id" in msg:
                    f = self.pending.get(msg["id"])
                    if f and not f.done():
                        f.set_result(msg)
                elif "samples" in msg:
                    for seq, t_ms, values in msg["samples"]:
                        row = [seq, t_ms / 1000.0]
                        latest = {"index": seq, "time": t_ms / 1000.0}
                        for port, components in values.items():
                            if len(components) >= 2:
                                latest[port] = {
                                    "angle_deg": components[0],
                                    "speed_deg_s": components[1],
                                }
                        for port, component in self.column_specs:
                            vals = values.get(port, [])
                            try:
                                row.append(float(vals[int(component)]))
                            except (IndexError, TypeError, ValueError):
                                row.append(math.nan)
                        self.matrix.append(row)
                        self.latest = latest
                        self.latest_received_ns = received_ns
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.failure = e

    async def request(self, op, **args):
        self.next_id += 1
        ident = self.next_id
        fut = asyncio.get_running_loop().create_future()
        self.pending[ident] = fut
        start_ns = time.perf_counter_ns()
        try:
            await self.hub.write_line(
                json.dumps({"id": ident, "op": op, **args}, separators=(",", ":"))
            )
            reply = await asyncio.wait_for(fut, 10)
            if "error" in reply:
                raise RuntimeError(reply["error"])
            if op == "set_motors":
                end_ns = time.perf_counter_ns()
                self.command_log.append(
                    [ident, start_ns, end_ns, json.dumps(args["duties"]), "ack"]
                )
            return reply["result"]
        except Exception as exc:
            if op == "set_motors":
                end_ns = time.perf_counter_ns()
                self.command_log.append(
                    [ident, start_ns, end_ns, json.dumps(args.get("duties", {})), repr(exc)]
                )
            raise
        finally:
            self.pending.pop(ident, None)

    def configure_columns(self, devices):
        self.columns = ["index", "time"]
        self.column_specs = []

        def add_pair(port):
            self.columns.extend([f"{port}.angle_deg", f"{port}.speed_deg_s"])
            self.column_specs.extend([(port, 0), (port, 1)])

        for port in "ABCD":
            if devices.get(port, {}).get("type") == "motor":
                add_pair(port)
        for port, device in devices.items():
            if device.get("family") == "gyro":
                add_pair(port)


def load_controller(path):
    if not path:
        return None
    controller_path = Path(path).resolve()
    spec = importlib.util.spec_from_file_location("user_controller", controller_path)
    if spec is None or spec.loader is None:
        raise ValueError(f"Cannot load controller module: {controller_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    callback = getattr(module, "control", None)
    if not callable(callback):
        raise ValueError("Controller module must define control(sample) -> {port: duty}")
    return callback


def validate_duties(duties, motor_ports):
    if not isinstance(duties, dict):
        raise ValueError("Controller must return a dictionary of motor duties")
    normalized = {}
    for port, duty in duties.items():
        port = str(port).upper()
        if port not in motor_ports:
            raise ValueError(f"Controller requested unavailable motor {port}")
        if type(duty) not in (int, float) or not math.isfinite(duty) or abs(duty) > 100:
            raise ValueError("Motor duties must be finite numbers from -100 to 100")
        normalized[port] = duty
    return normalized


async def control_loop(client, args, motor_ports, controller):
    loop = asyncio.get_running_loop()
    period = args.control_ms / 1000
    next_tick = loop.time() + period
    manual_duties = {}
    command_lines = queue.Queue()

    if controller is None:
        print(
            f"PC control tick: {args.control_ms} ms. Enter A=20,B=-10 to set duty; "
            "values are renewed each tick. Omitted motors are untouched; 0 coasts.",
            flush=True,
        )

        def read_commands():
            while True:
                try:
                    command_lines.put(input())
                except EOFError:
                    command_lines.put(None)
                    return

        threading.Thread(target=read_commands, daemon=True).start()
    else:
        print(f"PC controller: {args.controller} ({args.control_ms} ms tick)", flush=True)

    while True:
        await asyncio.sleep(max(0, next_tick - loop.time()))
        actual = loop.time()
        late_periods = max(0, int((actual - next_tick) // period))
        scheduled = next_tick + late_periods * period
        next_tick += (late_periods + 1) * period

        if client.failure:
            raise client.failure
        if controller is None:
            while True:
                try:
                    line = command_lines.get_nowait()
                except queue.Empty:
                    break
                if line is None:
                    return
                if line.strip():
                    for item in line.split(","):
                        port, duty = item.split("=", 1)
                        manual_duties.update(
                            validate_duties(
                                {port.strip().upper(): float(duty)}, motor_ports
                            )
                        )

        sample = client.latest
        sample_age_ms = (
            (time.perf_counter_ns() - client.latest_received_ns) / 1e6
            if client.latest_received_ns is not None
            else None
        )
        if sample is None:
            duties, status = {}, "waiting_for_sample"
        elif sample_age_ms is not None and sample_age_ms > 250:
            duties, status = {}, "stale_sample"
        else:
            state = dict(sample)
            state["age_ms"] = sample_age_ms
            raw_duties = controller(state) if controller else manual_duties
            duties = validate_duties(raw_duties, motor_ports)
            status = "sent" if duties else "no_motor_duties"

        started_ns = time.perf_counter_ns()
        if duties:
            await client.request("set_motors", duties=duties)
        finished_ns = time.perf_counter_ns()
        client.control_log.append(
            [
                len(client.control_log),
                int(scheduled * 1e9),
                int(actual * 1e9),
                started_ns,
                finished_ns,
                late_periods,
                sample.get("index") if sample else "",
                sample_age_ms if sample_age_ms is not None else "",
                status,
                json.dumps(duties, separators=(",", ":")),
            ]
        )


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
    out = Path(args.output) / datetime.now().strftime("%Y%m%dT%H%M%S.%f")
    out.mkdir(parents=True, exist_ok=False)
    try:
        controller = load_controller(args.controller)
        await hub.connect()
        await hub.run()
        ready = json.loads(await asyncio.wait_for(hub.read_line(), 60))
        if ready != {"ready": "ev3-tars-control-2"}:
            raise RuntimeError("Upload tars_control_brick.py and run it on the EV3")
        receive_task = asyncio.create_task(client.receive())
        devices = (await client.request("hello"))["devices"]
        client.configure_columns(devices)
        motor_ports = {
            port for port in "ABCD" if devices.get(port, {}).get("type") == "motor"
        }
        (out / "columns.json").write_text(json.dumps(client.columns, indent=2), encoding="utf-8")
        (out / "devices.json").write_text(json.dumps(devices, indent=2), encoding="utf-8")
        await client.request("stream")
        await control_loop(client, args, motor_ports, controller)
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
        array = np.asarray(client.matrix, dtype=np.float64).reshape((-1, len(client.columns)))
        np.save(out / "measurements.npy", array)
        (out / "columns.json").write_text(json.dumps(client.columns, indent=2), encoding="utf-8")
        with (out / "commands.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["id", "pc_start_ns", "pc_end_ns", "duties_json", "result"])
            writer.writerows(client.command_log)
        with (out / "control_ticks.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "tick",
                    "scheduled_pc_ns",
                    "actual_pc_ns",
                    "command_start_ns",
                    "command_end_ns",
                    "missed_ticks",
                    "sample_index",
                    "sample_age_ms",
                    "status",
                    "duties_json",
                ]
            )
            writer.writerows(client.control_log)
        print(f"Saved {len(array)} rows to {out / 'measurements.npy'}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--address")
    parser.add_argument("--output", default="results/tars_control")
    parser.add_argument("--control-ms", type=int, default=100, help="PC controller tick period")
    parser.add_argument("--controller", help="Python file defining control(sample) -> {port: duty}")
    args = parser.parse_args()
    if not 10 <= args.control_ms <= 1000:
        parser.error("--control-ms must be from 10 to 1000")
    try:
        asyncio.run(session(args))
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"{type(e).__name__}: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
