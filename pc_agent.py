"""Windows Bluetooth Classic + Pybricks 4.1 framed serial protocol.

Protocol reference: pybricks-micropython v4.1.0, lib/pbio/{src/serial.c,
src/cobs.c,include/pbio/protocol.h}. No BLE or virtual COM port required.
"""
import asyncio
import ctypes as ct
from ctypes import wintypes as wt
import select
import socket
import struct
import sys
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time
from datetime import datetime, timezone


def encode(data):
    """LEGO COBS variant: escape 0/1/2, XOR 3, end with 2."""
    out = bytearray([255])
    index, count = 0, 1
    for byte in data:
        if byte > 2:
            out.append(byte)
            count += 1
        if byte <= 2 or count > 84:
            if byte <= 2:
                out[index] = byte * 84 + count + 2
            index = len(out)
            out.append(255)
            count = 1
    out[index] = count + 2
    return bytes(b ^ 3 for b in out) + b"\x02"


def decode(frame):
    if frame.startswith(b"\x01"):
        frame = frame[1:]
    data = bytes(b ^ 3 for b in frame)
    out = bytearray()
    pos = 0
    while pos < len(data):
        code = data[pos]
        pos += 1
        if code < 3:
            raise ValueError("Invalid COBS code")
        if code == 255:
            count, value = 84, None
        else:
            value, remainder = divmod(code - 3, 84)
            count = remainder
        if pos + count > len(data):
            raise ValueError("Truncated COBS block")
        out.extend(data[pos:pos + count])
        pos += count
        if pos < len(data) and value is not None:
            out.append(value)
    if not out:
        raise ValueError("Empty frame")
    return bytes(out)


def known_devices():
    """Read Windows' known classic devices; does not pair or change settings."""
    if sys.platform != "win32":
        raise RuntimeError("This Bluetooth adapter currently targets Windows")

    class Info(ct.Structure):
        _fields_ = [("size", wt.DWORD), ("address", ct.c_uint64),
                    ("device_class", wt.ULONG), ("connected", wt.BOOL),
                    ("remembered", wt.BOOL), ("authenticated", wt.BOOL),
                    ("last_seen", wt.WORD * 8), ("last_used", wt.WORD * 8),
                    ("name", wt.WCHAR * 248)]

    class Search(ct.Structure):
        _fields_ = [("size", wt.DWORD), ("authenticated", wt.BOOL),
                    ("remembered", wt.BOOL), ("unknown", wt.BOOL),
                    ("connected", wt.BOOL), ("inquiry", wt.BOOL),
                    ("timeout", ct.c_ubyte), ("radio", wt.HANDLE)]

    api = ct.WinDLL("BluetoothApis.dll", use_last_error=True)
    api.BluetoothFindFirstDevice.argtypes = [ct.POINTER(Search), ct.POINTER(Info)]
    api.BluetoothFindFirstDevice.restype = wt.HANDLE
    api.BluetoothFindNextDevice.argtypes = [wt.HANDLE, ct.POINTER(Info)]
    api.BluetoothFindNextDevice.restype = wt.BOOL
    api.BluetoothFindDeviceClose.argtypes = [wt.HANDLE]
    api.BluetoothFindDeviceClose.restype = wt.BOOL
    query = Search(ct.sizeof(Search), True, True, False, True, False, 0, None)
    info = Info()
    info.size = ct.sizeof(Info)
    handle = api.BluetoothFindFirstDevice(ct.byref(query), ct.byref(info))
    if not handle:
        error = ct.get_last_error()
        if error == 259:  # ERROR_NO_MORE_ITEMS
            return []
        raise ct.WinError(error)
    result = []
    try:
        while True:
            addr = f"{info.address:012X}"
            result.append({"name": info.name, "address": ":".join(addr[i:i+2] for i in range(0, 12, 2)),
                           "paired": bool(info.authenticated), "connected": bool(info.connected)})
            if not api.BluetoothFindNextDevice(handle, ct.byref(info)):
                error = ct.get_last_error()
                if error != 259:
                    raise ct.WinError(error)
                break
    finally:
        api.BluetoothFindDeviceClose(handle)
    return result


def open_socket(address, timeout=10):
    if sys.platform != "win32":
        raise RuntimeError("Bluetooth Classic adapter requires Windows")
    # ws2bth.h uses one-byte packing (SOCKADDR_BTH is 30 bytes).
    sockaddr = struct.pack("<HQ16sI", 32, int(address.replace(":", "").replace("-", ""), 16), bytes(16), 1)
    ws = ct.WinDLL("Ws2_32.dll")
    ws.connect.argtypes = [ct.c_size_t, ct.c_void_p, ct.c_int]
    ws.connect.restype = ct.c_int
    ws.WSAGetLastError.restype = ct.c_int
    sock = socket.socket(32, socket.SOCK_STREAM, 3)
    try:
        sock.setblocking(False)
        if ws.connect(sock.fileno(), sockaddr, len(sockaddr)) != 0:
            error = ws.WSAGetLastError()
            if error != 10035:  # WSAEWOULDBLOCK
                raise OSError(error, "Bluetooth RFCOMM connect failed")
            _, writable, exceptional = select.select([], [sock], [sock], timeout)
            if not writable and not exceptional:
                raise TimeoutError("Bluetooth connection timed out: power on, pair, and disconnect website")
            error = sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
            if error:
                raise OSError(error, "Bluetooth RFCOMM connection failed")
        sock.settimeout(.2)
        return sock
    except BaseException:
        sock.close()
        raise


class BluetoothHub:
    def __init__(self, address):
        self.address = address
        self.sock = None
        self.reader = None
        self.frames = bytearray()
        self.stdout = bytearray()
        self.lines = asyncio.Queue()
        self.responses = asyncio.Queue()
        self.reads = asyncio.Queue()
        self.lock = asyncio.Lock()
        self.tag = 0
        self.max_write = 20
        self.info = {}

    async def connect(self):
        self.sock = await asyncio.to_thread(open_socket, self.address)
        self.reader = asyncio.create_task(self._reader())
        try:
            for key, char in (("name", 0x2A00), ("firmware", 0x2A26), ("protocol", 0x2A28)):
                self.info[key] = (await self.read_characteristic(1, char)).decode()
            if not self.info["protocol"].startswith("1.7."):
                raise RuntimeError("Expected Pybricks protocol 1.7.x, got " + self.info["protocol"])
            caps = await self.read_characteristic(2, 3)
            if len(caps) < 10:
                raise RuntimeError("Invalid hub capabilities")
            self.max_write = int.from_bytes(caps[:2], "little")
            if not 2 <= self.max_write <= 512:
                raise RuntimeError("Invalid maximum command size")
            await self._send(b"\x01\x01")  # subscribe (no response)
        except BaseException:
            await self.disconnect()
            raise

    async def _send(self, payload):
        await asyncio.to_thread(self.sock.sendall, encode(payload))

    def _dispatch(self, message):
        kind, payload = message[0], message[1:]
        if kind == 1:
            self.responses.put_nowait(payload)
        elif kind == 3:
            self.reads.put_nowait(payload)
        elif kind == 2 and payload and (payload[0] == 1 or payload[:2] == b"\x02\xff"):
            # stdout remains supported for firmware tracebacks; AppData carries agent JSON.
            self.stdout.extend(payload[2:] if payload[0] == 2 else payload[1:])
            if len(self.stdout) > 65536:
                raise RuntimeError("Agent output line exceeds limit")
            while b"\n" in self.stdout:
                line, _, rest = self.stdout.partition(b"\n")
                self.stdout = bytearray(rest)
                self.lines.put_nowait(line.rstrip(b"\r").decode("utf-8"))

    async def _reader(self):
        try:
            while True:
                try:
                    chunk = await asyncio.to_thread(self.sock.recv, 2048)
                except socket.timeout:
                    continue
                if not chunk:
                    raise ConnectionError("Bluetooth disconnected")
                self.frames.extend(chunk)
                while b"\x02" in self.frames:
                    frame, _, rest = self.frames.partition(b"\x02")
                    self.frames = bytearray(rest)
                    if frame:
                        self._dispatch(decode(frame))
                if len(self.frames) > 2048:
                    raise RuntimeError("Oversized Bluetooth frame")
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            for queue in (self.lines, self.responses, self.reads):
                queue.put_nowait(exc)

    async def _get(self, queue):
        item = await asyncio.wait_for(queue.get(), 5)
        if isinstance(item, Exception):
            raise item
        return item

    async def read_characteristic(self, service, char):
        async with self.lock:
            selector = struct.pack("<BH", service, char)
            await self._send(b"\x03" + selector)
            reply = await self._get(self.reads)
            if reply[:3] != selector:
                raise RuntimeError("Characteristic reply mismatch")
            return reply[3:]

    async def command(self, payload):
        async with self.lock:
            self.tag = (self.tag + 1) % 256
            await self._send(bytes([2, self.tag]) + payload)
            response = await self._get(self.responses)
            if len(response) != 5 or response[0] != self.tag:
                raise RuntimeError("Bluetooth command response mismatch")
            status = int.from_bytes(response[1:], "little")
            if status:
                raise RuntimeError(f"Pybricks command rejected: 0x{status:02x}")

    async def run(self, path=None, wait=False, print_output=False):
        if path is not None:
            raise ValueError("Deploy the agent with Pybricks Code")
        await self.command(b"\x01")

    async def stop_user_program(self):
        await self.command(b"\x00")

    async def write_line(self, line):
        data = (line + "\n").encode()
        for pos in range(0, len(data), self.max_write - 1):
            await self.command(b"\x06" + data[pos:pos + self.max_write - 1])

    async def read_line(self):
        # Client's timeout includes the whole operation, including local bursts.
        item = await self.lines.get()
        if isinstance(item, Exception):
            raise item
        return item

    async def disconnect(self):
        if self.sock:
            self.sock.close()
        if self.reader:
            self.reader.cancel()
            try:
                await self.reader
            except asyncio.CancelledError:
                pass
        self.sock = None


def summarize(values):
    values = sorted(values)
    if not values:
        return {"count": 0}
    return {"count": len(values), "median": statistics.median(values),
            "p95": values[math.ceil(len(values) * .95) - 1],
            "p99": values[math.ceil(len(values) * .99) - 1],
            "max": values[-1]}


class AgentClient:
    def __init__(self, hub, log, motor_port="B"):
        self.hub, self.log = hub, log
        self.motor_port = motor_port
        self.dropped_samples = 0
        self.next_id = 0
        self.pending = {}
        self.latest = None
        self.previous = None
        self.intervals = []
        self.packet_intervals = []
        self.sample_ages = []
        self.last_packet_ns = None
        self.last_sample_ns = None
        self.clock_offset_ms = None
        self.watchdog_stops = 0
        self.sample_gaps = 0
        self.rtts = []
        self.failure = None
        self.by_measurement = {}
        self.sample_times = {}
        self.b_previous = None
        self.b_received_ns = None
        self.limit_seen = False
        self.display_ns = 0
        self.values = {}
        self.origins = {}
        self.motor_test_samples = None

    async def receive(self):
        try:
            while True:
                line = await self.hub.read_line()
                received_ns = time.perf_counter_ns()
                message = json.loads(line)
                self.log({"event": "receive", "pc_ns": received_ns, "message": message})
                if "id" in message:
                    future = self.pending.get(message["id"])
                    if future is not None and not future.done():
                        future.set_result((message, received_ns))
                elif "samples" in message:
                    self.watchdog_stops = message["watchdog_stops"]
                    self.dropped_samples = message.get("dropped_samples", 0)
                    if self.last_packet_ns is not None:
                        self.packet_intervals.append((received_ns - self.last_packet_ns) / 1e6)
                    self.last_packet_ns = received_ns
                    for row in message["samples"]:
                        if self.motor_test_samples is not None and row[4] == "encoder":
                            self.motor_test_samples.append((received_ns, row))
                        key = row[3] + "/" + row[4]
                        if row[4].endswith("_error"):
                            self.values.pop(key[:-6], None)
                        else:
                            self.values.pop(key + "_error", None)
                        self.values[key] = row[5]
                        if row[4] == "encoder":
                            self.origins.setdefault(row[3], row[5]["angle_deg"])
                        if key in self.sample_times:
                            interval = row[1] - self.sample_times[key]
                            self.intervals.append(interval)
                            self.by_measurement.setdefault(key, []).append(interval)
                        self.sample_times[key] = row[1]
                        if self.previous:
                            self.sample_gaps += max(0, row[0] - self.previous[0] - 1)
                        if row[3:5] == [self.motor_port, "encoder"]:
                            self.b_previous = row
                            self.b_received_ns = received_ns
                            if self.motor_test_samples is None and abs(row[5]["angle_deg"] - self.origins[self.motor_port]) > 90 and not self.limit_seen:
                                print("Motor", self.motor_port, "exceeded the expected +/-90 degree range (logged, not a hard stop).")
                                self.limit_seen = True
                        self.previous = row
                        self.latest = row
                        self.last_sample_ns = received_ns
                        age = None
                        if self.clock_offset_ms is not None:
                            age = received_ns / 1e6 - (row[1] + self.clock_offset_ms)
                            self.sample_ages.append(age)
                        self.log({"event": "sample", "pc_ns": received_ns, "row": row,
                                  "estimated_age_ms": age})
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.failure = exc
            for future in self.pending.values():
                if not future.done():
                    future.set_exception(exc)

    async def request(self, op, **fields):
        if self.failure:
            raise self.failure
        self.next_id += 1
        ident = self.next_id
        future = asyncio.get_running_loop().create_future()
        self.pending[ident] = future
        start = time.perf_counter_ns()
        try:
            async def exchange():
                await self.hub.write_line(json.dumps({"id": ident, "op": op, **fields}, separators=(",", ":")))
                return await future
            reply, end = await asyncio.wait_for(exchange(), 60 if op == "stream" else 5)
            if "error" in reply:
                raise RuntimeError("Brick: " + reply["error"])
            rtt = (end - start) / 1e6
            self.log({"event": "request", "op": op, "id": ident, "fields": fields,
                      "pc_start_ns": start, "pc_end_ns": end, "rtt_ms": rtt})
            return reply, rtt, start, end
        finally:
            self.pending.pop(ident, None)


def position_duty(row, origin, received_ns, now_ns, target, kp):
    """PC-side proportional position control; tolerate overshoot and drive back toward the target."""
    if row is None or origin is None or received_ns is None or now_ns - received_ns > 250_000_000:
        return 0
    error = target - (row[5]["angle_deg"] - origin)
    return max(-15, min(15, kp * error))


async def motor_capabilities(client, devices, log, resolution=False):
    """Short, low-duty pulses; all limits are checked on the PC."""
    results = {}
    client.motor_test_samples = []
    await client.request("stream", report_ms=0, sensor_mode="none")
    try:
        for port in sorted(p for p, d in devices.items() if d.get("type") == "motor" and d.get("status") == "connected"):
            print("Full-power resolution test: motor" if resolution else "Small-motion test: motor", port, flush=True)
            def reading():
                key = port + "/encoder"
                if client.failure:
                    raise client.failure
                samples = client.motor_test_samples
                latest = next(((t, r) for t, r in reversed(samples) if r[3] == port), None)
                if latest is None or time.perf_counter_ns() - latest[0] > 150_000_000:
                    raise RuntimeError("Missing or stale encoder for motor " + port)
                if client.watchdog_stops:
                    raise RuntimeError("Motor watchdog fired")
                return latest[1][5]["angle_deg"]

            await asyncio.sleep(.2)
            origin = reading()
            phases = []
            results[port] = {"origin_deg": origin, "phases": phases}
            limit = (315 if port in ("B", "C") else 900) if resolution else 20
            # Stop well before the requested range; reserve room for latency/coasting.
            threshold = limit / 2 if resolution else limit
            pulse_ms = 100 if resolution else 350
            pulse_degrees = 40 if resolution else 8
            results[port].update(range_limit_deg=limit, stop_threshold_deg=threshold)
            for duty in ((100, -100) * 3 if resolution else (5, -5, 10, -10, 15, -15)):
                start_angle = reading()
                if abs(start_angle - origin) >= threshold:
                    results[port]["stopped_at_travel_limit"] = True
                    break
                first = len(client.motor_test_samples)
                start_ns = time.perf_counter_ns()
                reason = str(pulse_ms) + " ms pulse completed"
                log({"event": "motor_test_phase", "port": port, "duty": duty, "pc_ns": start_ns})
                rtts = []
                try:
                    while time.perf_counter_ns() - start_ns < pulse_ms * 1_000_000:
                        angle = reading()
                        if abs(angle - start_angle) >= pulse_degrees or abs(angle - origin) >= threshold:
                            reason = "travel threshold reached"
                            break
                        _, rtt, _, _ = await client.request("dc", port=port, duty=duty)
                        rtts.append(rtt)
                finally:
                    await client.request("brake", port=port)
                powered_end = len(client.motor_test_samples)
                await asyncio.sleep(.2)
                end_angle = reading()
                samples = [(t, r) for t, r in client.motor_test_samples[first:powered_end] if r[3] == port]
                rows = [r for _, r in samples]
                changed = [(t, r) for t, r in samples if r[5]["angle_deg"] != start_angle]
                steps = [abs(b[5]["angle_deg"] - a[5]["angle_deg"]) for a, b in zip(rows, rows[1:])]
                one_degree_pairs = [[a[1], b[1], a[5]["angle_deg"], b[5]["angle_deg"]]
                                    for a,b in zip(rows, rows[1:])
                                    if abs(b[5]["angle_deg"] - a[5]["angle_deg"]) == 1]
                phases.append({"one_degree_pairs": one_degree_pairs, "duty_percent": duty, "start_deg": start_angle, "settled_deg": end_angle,
                               "displacement_deg": end_angle - start_angle, "stop_reason": reason,
                               "samples": len(rows), "adjacent_pairs": len(steps),
                               "identical_adjacent": steps.count(0),
                               "nonzero_angle_step_deg": summarize([s for s in steps if s]),
                               "sample_interval_ms": summarize([b[1]-a[1] for a,b in zip(rows,rows[1:])]),
                               "first_change_observed_ms": (changed[0][0]-start_ns)/1e6 if changed else None,
                               "command_round_trip_ms": summarize(rtts)})
                print(" ", duty, "%:", end_angle - start_angle, "degrees;", reason, flush=True)
            results[port]["final_offset_deg"] = reading() - origin
        return results
    finally:
        log({"event": "motor_test_results", "results": results})
        await client.request("stop")
        client.motor_test_samples = None


async def session(args):
    address = args.address
    if not address:
        candidates = [d for d in known_devices() if "EV3" in d["name"].upper() and d["paired"]]
        if len(candidates) != 1:
            raise RuntimeError("Pair one EV3 in Windows, or pass --address AA:BB:CC:DD:EE:FF")
        address = candidates[0]["address"]
    hub = BluetoothHub(address)
    task = None
    started = False
    folder = Path(args.output) / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    folder.mkdir(parents=True)
    summary = {"status": "started", "transport": "Bluetooth Classic RFCOMM", "address": address,
               "python": sys.version, "settings": vars(args)}
    print("Results:", folder)
    try:
        await hub.connect()
        summary["hub"] = hub.info
        print("Connected:", hub.info)
        if args.probe:
            summary["status"] = "handshake_verified"
            print("Handshake verified. No program started.")
            return
        with (folder / "events.jsonl").open("w", encoding="utf-8") as raw, (folder / "samples.csv").open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(["sequence", "read_start_ms", "read_end_ms", "port", "measurement",
                             "value_json", "pc_receive_ns", "estimated_sample_age_ms"])
            def log(event):
                raw.write(json.dumps(event) + "\n")
                raw.flush()
                if event["event"] == "sample":
                    writer.writerow(event["row"][:5] + [json.dumps(event["row"][5]), event["pc_ns"], event["estimated_age_ms"]])

            # Only run the manually stored agent. Never upload from the PC.
            await hub.run()
            started = True
            ready = json.loads(await asyncio.wait_for(hub.read_line(), 60))
            if ready != {"ready": "ev3-general-8"}:
                raise RuntimeError("Wrong stored program. Upload brick_agent.py and stop it before connecting.")
            client = AgentClient(hub, log, args.motor)
            task = asyncio.create_task(client.receive())
            hello, _, _, _ = await client.request("hello")
            summary["devices"] = hello["result"]["devices"]
            (folder / "devices.json").write_text(json.dumps(summary["devices"], indent=2), encoding="utf-8")
            print("Connected devices:", json.dumps(summary["devices"], indent=2))
            if args.move and summary["devices"].get(args.motor, {}).get("type") != "motor":
                raise RuntimeError("Requested test motor is not connected: " + args.motor)
            if args.benchmark:
                offsets = []
                for i in range(args.roundtrips):
                    reply, rtt, start, end = await client.request("ping")
                    client.rtts.append(rtt)
                    offsets.append((rtt, (start + end) / 2e6 - (reply["received_ms"] + reply["done_ms"]) / 2))
                    if (i + 1) % 100 == 0:
                        print("Round trips:", i + 1, "/", args.roundtrips)
                client.clock_offset_ms = min(offsets)[1]
                summary["round_trip_ms"] = summarize(client.rtts)
                summary["rtt_jitter_abs_successive_difference_ms"] = summarize([abs(b-a) for a,b in zip(client.rtts, client.rtts[1:])])
                summary["clock_offset_estimate_ms"] = client.clock_offset_ms
                summary["clock_offset_method"] = "midpoint of lowest-RTT ping; assumes symmetric paths; not synchronized clocks"
            if args.motor_test or args.resolution_test:
                summary["motor_capabilities"] = await motor_capabilities(client, summary["devices"], log, args.resolution_test)
                summary.update(dropped_samples=client.dropped_samples, sequence_gaps=client.sample_gaps,
                               watchdog_stops=client.watchdog_stops)
            if args.stream:
                reply, _, _, _ = await client.request("stream", report_ms=1000 // args.report_hz if args.report_hz else 0,
                                                     sensor_mode=args.sensor_mode)
                motion_start = time.perf_counter()
                finish = motion_start + args.seconds
                control_rtts = []
                while time.perf_counter() < finish:
                    if client.failure:
                        raise client.failure
                    if client.watchdog_stops:
                        raise RuntimeError("Brick watchdog fired; powered run invalid")
                    if args.move:
                        target = args.amplitude * math.sin(2 * math.pi * (time.perf_counter() - motion_start) / args.cycle_seconds)
                        duty = position_duty(client.b_previous, client.origins.get(args.motor), client.b_received_ns,
                                             time.perf_counter_ns(), target, args.kp)
                        log({"event": "control", "target_deg": target, "duty": duty})
                        _, rtt, _, _ = await client.request("dc", port=args.motor, duty=duty)
                    else:
                        _, rtt, _, _ = await client.request("ping")
                    if client.latest is not None and time.perf_counter_ns() - client.display_ns >= 1_000_000_000:
                        print("Raw inputs:", json.dumps(client.values))
                        client.display_ns = time.perf_counter_ns()
                    control_rtts.append(rtt)
                    await asyncio.sleep(1 / args.report_hz if args.report_hz else 0)
                await client.request("stop")
                summary.update(measurement_interval_ms={k: summarize(v) for k, v in client.by_measurement.items()},
                               motor_exceeded_expected_range=client.limit_seen,
                               dropped_samples=client.dropped_samples,
                               packet_interarrival_ms=summarize(client.packet_intervals),
                               estimated_sample_age_ms=summarize(client.sample_ages),
                               streaming_command_round_trip_ms=summarize(control_rtts),
                               sequence_gaps=client.sample_gaps, watchdog_stops=client.watchdog_stops)
                inventory, _, _, _ = await client.request("devices")
                summary["devices"] = inventory["result"]["devices"]
                (folder / "devices.json").write_text(json.dumps(summary["devices"], indent=2), encoding="utf-8")
                if client.latest is None:
                    raise RuntimeError("No sensor samples received")
            summary["status"] = "complete"
            print(json.dumps(summary, indent=2))
    except BaseException as exc:
        summary.update(status="failed", error=repr(exc))
        raise
    finally:
        if started:
            try:
                await asyncio.wait_for(hub.stop_user_program(), 2)
            except Exception as exc:
                print("Firmware stop failed:", exc, file=sys.stderr)
        if task:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        await hub.disconnect()
        (folder / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Windows PC to Pybricks 4.1 EV3 over Bluetooth Classic. Default: hello/ack only.")
    parser.add_argument("--list", action="store_true", help="list known EV3 Bluetooth devices")
    parser.add_argument("--address", help="Bluetooth MAC; defaults to the single paired EV3")
    parser.add_argument("--probe", action="store_true", help="handshake only; do not start agent")
    parser.add_argument("--benchmark", action="store_true", help="measure application ping/ack round trips")
    parser.add_argument("--roundtrips", type=int, default=1000)
    parser.add_argument("--stream", action="store_true", help="stream all discovered raw inputs; unpowered unless --move")
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--report-hz", type=int, choices=[0, 20, 40, 50], default=0,
                        help="0: no deliberate pacing (default); otherwise limit report/control rates")
    parser.add_argument("--move", action="store_true", help="enable selected motor position sweep within expected +/-90 degree range")
    parser.add_argument("--resolution-test", action="store_true", help="short 100 percent pulses on all motors; measure observed 1-degree transitions")
    parser.add_argument("--motor-test", action="store_true", help="test every connected motor separately with short 5/10/15 percent pulses")
    parser.add_argument("--motor", choices=list("ABCD"), default="B", help="motor for optional powered sweep")
    parser.add_argument("--sensor-mode", choices=["all", "none", "0", "1", "2", "3", "4"],
                        help="all sensor modes by default; powered sweep defaults to none")
    parser.add_argument("--amplitude", type=float, default=90, help="position sweep amplitude in degrees (0..90)")
    parser.add_argument("--cycle-seconds", type=float, default=8, help="duration of one sinusoidal sweep")
    parser.add_argument("--kp", type=float, default=.2, help="percent duty per degree of position error")
    parser.add_argument("--output", default="results")
    args = parser.parse_args()
    if args.sensor_mode is None:
        args.sensor_mode = "none" if args.move else "all"
    if args.roundtrips < 1000:
        parser.error("Benchmark needs at least 1000 round trips")
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    if any(not math.isfinite(v) or v < 0 for v in (args.kp, args.amplitude)) or args.amplitude > 90:
        parser.error("Gains must be finite and nonnegative; amplitude must be 0..90 degrees")
    if not math.isfinite(args.cycle_seconds) or args.cycle_seconds < 2:
        parser.error("--cycle-seconds must be at least 2")
    if args.move and not args.stream:
        parser.error("--move requires --stream")
    if args.resolution_test and args.motor_test:
        parser.error("Choose --resolution-test or --motor-test")
    if (args.motor_test or args.resolution_test) and (args.stream or args.move or args.probe):
        parser.error("--motor-test cannot be combined with --stream, --move or --probe")
    if args.probe and (args.stream or args.benchmark or args.move):
        parser.error("--probe cannot be combined with experiments")
    try:
        if args.list:
            for dev in known_devices():
                if "EV3" in dev["name"].upper():
                    print(json.dumps(dev, ensure_ascii=False))
        else:
            asyncio.run(session(args))
        return 0
    except KeyboardInterrupt:
        print("Interrupted; attempted firmware stop.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
