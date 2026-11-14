"""Tick-based EV3 measurements and leased motor commands for tars_control_pc.py."""

import ujson as json
from pybricks.messaging import AppData
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, GyroSensor
from pybricks.iodevices import LUMPDevice
from pybricks.parameters import Port
from pybricks.tools import StopWatch, wait, read_input_byte, run_task, multitask

clock = StopWatch()
hub = EV3Brick()
app = AppData([])
motors, sensors, devices = {}, [], {}
replies, rows = [], []
motor_deadlines = {}
motor_duties = {}
motor_limits = {}
streaming = False
sequence = 0
SAMPLE_TICK_MS = 10
LEASE_TICK_MS = 5
MOTOR_LEASE_MS = 100
MOTOR_POLARITY = {"A": -1, "D": -1}


def discover():
    for name in "ABCD":
        try:
            # Preserve accumulated encoder positions between program sessions.
            m = Motor(getattr(Port, name), reset_angle=False)
            m.stop()
            motors[name] = m
            devices[name] = {"type": "motor", "status": "connected"}
        except Exception as e:
            devices[name] = {"status": "empty_or_unsupported", "error": str(e)}
    for name in ("S1", "S2", "S3", "S4"):
        port = getattr(Port, name)
        try:
            d = LUMPDevice(port)
            info = d.info()
            advertised_modes = [
                {"index": i, "name": mode[0], "metadata": list(mode[1:])}
                for i, mode in enumerate(info.get("modes", ()))
            ]
            if info.get("id") == 32:
                d, kind = GyroSensor(port), "gyro"
                sensors.append((name, d))
                devices[name] = {
                    "type": "sensor",
                    "status": "connected",
                    "info": info,
                    "family": kind,
                    "available_modes": advertised_modes,
                }
            else:
                kind = "infrared" if info.get("id") == 33 else "other"
                devices[name] = {
                    "type": "sensor",
                    "status": "connected",
                    "info": info,
                    "family": kind,
                    "available_modes": advertised_modes,
                }
        except Exception as uart_error:
            try:
                TouchSensor(port)
                devices[name] = {
                    "type": "touch",
                    "status": "connected",
                    "available_modes": [],
                }
            except Exception as e:
                devices[name] = {
                    "status": "empty_or_unsupported",
                    "UART_error": str(uart_error),
                    "error": str(e),
                }


def motor(req):
    port = req.get("port")
    duty = req.get("duty")
    if port not in motors:
        raise ValueError("No connected motor at requested port")
    if type(duty) not in (int, float) or not -100 <= duty <= 100:
        raise ValueError("duty must be within -100..100 percent")
    if duty:
        physical_duty = duty * MOTOR_POLARITY.get(port, 1)
        limited = at_motor_limit(port, physical_duty)
        if limited:
            motors[port].brake()
            motor_deadlines.pop(port, None)
            motor_duties.pop(port, None)
        else:
            motors[port].dc(physical_duty)
            motor_deadlines[port] = clock.time() + MOTOR_LEASE_MS
            motor_duties[port] = physical_duty
    else:
        motors[port].stop()
        motor_deadlines.pop(port, None)
        motor_duties.pop(port, None)
        limited = False
    return {"port": port, "duty": duty, "limited": limited}


def at_motor_limit(port, duty):
    bounds = motor_limits.get(port)
    if not bounds:
        return False
    angle = motors[port].angle()
    low, high = bounds
    return (duty < 0 and low is not None and angle <= low) or (
        duty > 0 and high is not None and angle >= high
    )


def set_motor_limits(limits):
    if not isinstance(limits, dict):
        raise ValueError("limits must map motor ports to min/max encoder angles")
    normalized = {}
    for port, bounds in limits.items():
        if port not in motors:
            raise ValueError("No connected motor at requested port: " + str(port))
        if not isinstance(bounds, dict):
            raise ValueError("Each motor limit must be an object with min and/or max")
        low, high = bounds.get("min"), bounds.get("max")
        if low is not None and type(low) not in (int, float):
            raise ValueError("min limits must be numeric")
        if high is not None and type(high) not in (int, float):
            raise ValueError("max limits must be numeric")
        if low is not None and high is not None and low >= high:
            raise ValueError("Each min limit must be less than its max limit")
        normalized[port] = (low, high)
    motor_limits.update(normalized)
    return {
        "limits": {
            port: {"min": bounds[0], "max": bounds[1]} for port, bounds in motor_limits.items()
        }
    }


def set_motors(duties):
    if not isinstance(duties, dict):
        raise ValueError("duties must be an object mapping motor ports to duty")
    for port, duty in duties.items():
        if port not in motors:
            raise ValueError("No connected motor at requested port: " + str(port))
        if type(duty) not in (int, float) or not -100 <= duty <= 100:
            raise ValueError("Every duty must be within -100..100 percent")
    for port, duty in duties.items():
        motor({"port": port, "duty": duty})
    return {"duties": duties, "lease_ms": MOTOR_LEASE_MS}


def replace_motors(duties):
    if not isinstance(duties, dict):
        raise ValueError("duties must be an object mapping motor ports to duty")
    for port, duty in duties.items():
        if port not in motors:
            raise ValueError("No connected motor at requested port: " + str(port))
        if type(duty) not in (int, float) or not -100 <= duty <= 100:
            raise ValueError("Every duty must be within -100..100 percent")
    for port in tuple(motor_deadlines):
        if port not in duties or not duties[port]:
            motors[port].brake()
            motor_deadlines.pop(port, None)
            motor_duties.pop(port, None)
    for port, duty in duties.items():
        if duty:
            physical_duty = duty * MOTOR_POLARITY.get(port, 1)
            if at_motor_limit(port, physical_duty):
                motors[port].brake()
                motor_deadlines.pop(port, None)
                motor_duties.pop(port, None)
            else:
                motors[port].dc(physical_duty)
                motor_deadlines[port] = clock.time() + MOTOR_LEASE_MS
                motor_duties[port] = physical_duty
        else:
            motors[port].stop()
            motor_deadlines.pop(port, None)
            motor_duties.pop(port, None)
    limited = [
        port
        for port, duty in duties.items()
        if duty and at_motor_limit(port, duty * MOTOR_POLARITY.get(port, 1))
    ]
    return {
        "duties": duties,
        "lease_ms": MOTOR_LEASE_MS,
        "replaced": True,
        "limited": limited,
    }


async def commands():
    buf = ""
    replies.append({"ready": "ev3-tars-control-7"})
    while True:
        for _ in range(128):
            b = read_input_byte()
            if b is None:
                break
            if b != 10:
                buf += chr(b)
                if len(buf) > 1024:
                    buf = ""
                continue
            req = {}
            try:
                req = json.loads(buf)
                op = req["op"]
                if op == "hello":
                    result = {"devices": devices}
                elif op == "positions":
                    result = {
                        port: {"angle_deg": m.angle(), "speed_deg_s": m.speed()}
                        for port, m in motors.items()
                    }
                elif op == "zero":
                    ports = req.get("ports", list(motors))
                    if not isinstance(ports, list) or any(port not in motors for port in ports):
                        raise ValueError("ports must list connected motors")
                    for port in ports:
                        motors[port].reset_angle(0)
                    result = {
                        "sequence": sequence,
                        "positions": {port: motors[port].angle() for port in ports},
                    }
                elif op == "stream":
                    global streaming
                    streaming = True
                    result = {"devices": devices}
                elif op == "dc":
                    result = motor(req)
                elif op == "set_motors":
                    result = set_motors(req.get("duties"))
                elif op == "replace_motors":
                    result = replace_motors(req.get("duties"))
                elif op == "set_motor_limits":
                    result = set_motor_limits(req.get("limits"))
                elif op == "stop":
                    for m in motors.values():
                        m.stop()
                    motor_deadlines.clear()
                    motor_duties.clear()
                    streaming = False
                    result = {"stopped": True}
                else:
                    raise ValueError("Unknown operation")
                replies.append({"id": req.get("id"), "result": result})
            except Exception as e:
                replies.append({"id": req.get("id"), "error": str(e)})
            buf = ""
        await wait(1)


async def sampler():
    global sequence
    next_tick = clock.time() + SAMPLE_TICK_MS
    while True:
        now = clock.time()
        if now < next_tick:
            await wait(next_tick - now)
            continue
        missed = (now - next_tick) // SAMPLE_TICK_MS
        next_tick += (missed + 1) * SAMPLE_TICK_MS
        if streaming:
            sequence += missed
            values = {}
            for port, m in tuple(motors.items()):
                try:
                    values[port] = [m.angle(), m.speed()]
                except Exception:
                    values[port] = [None]
                await wait(0)
            for port, d in sensors:
                try:
                    value = [d.angle(), d.speed()]
                    values[port] = value
                except Exception:
                    values[port] = []
                await wait(0)
            rows.append([sequence, now, values])
            sequence += 1
            if len(rows) > 64:
                rows.pop(0)


async def motor_lease_task():
    next_tick = clock.time() + LEASE_TICK_MS
    while True:
        now = clock.time()
        if now < next_tick:
            await wait(next_tick - now)
            continue
        missed = (now - next_tick) // LEASE_TICK_MS
        next_tick += (missed + 1) * LEASE_TICK_MS
        for port, deadline in tuple(motor_deadlines.items()):
            duty = motor_duties.get(port, 0)
            if at_motor_limit(port, duty):
                try:
                    motors[port].brake()
                except Exception:
                    pass
                motor_deadlines.pop(port, None)
                motor_duties.pop(port, None)
            elif now >= deadline:
                try:
                    motors[port].brake()
                except Exception:
                    pass
                motor_deadlines.pop(port, None)
                motor_duties.pop(port, None)


async def send(message):
    data = bytes(json.dumps(message) + "\n", "utf-8")
    for i in range(0, len(data), 128):
        await app.write_bytes(b"\xff" + data[i : i + 128])


async def reporter():
    while True:
        if replies:
            await send(replies.pop(0))
        elif streaming and rows:
            batch = rows[:8]
            del rows[:8]
            await send({"samples": batch})
            await wait(20)
        else:
            await wait(10)


if __name__ == "__main__":
    try:
        discover()
        run_task(multitask(commands(), sampler(), reporter(), motor_lease_task()))
    finally:
        for m in motors.values():
            try:
                m.stop()
            except Exception:
                pass
