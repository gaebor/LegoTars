from pybricks.hubs import EV3Brick
from pybricks.ev3devices import InfraredSensor, Motor
from pybricks.parameters import Button, Port
from pybricks.tools import run_task, wait


IR_PORT = Port.S4
POWER = 100  # Percent duty cycle.

# channel, left rocker outputs, right rocker outputs (port, direction)
CHANNELS = (
    (1, ((Port.B, 1), (Port.C, 1)), ((Port.A, 1), (Port.D, 1))),
    (2, ((Port.B, 1), (Port.C, -1)), ((Port.A, 1), (Port.D, -1))),
    (3, ((Port.B, 1),), ((Port.C, 1),)),
    (4, ((Port.A, 1),), ((Port.D, 1),)),
)
MOTOR_SIGNS = {Port.A: 1, Port.B: 1, Port.C: 1, Port.D: 1}


def rocker_power(buttons, up, down):
    """Opposite buttons cancel; a released rocker stops its motor."""
    return POWER * (int(up in buttons) - int(down in buttons))


def apply_power(motor, power):
    if power:
        motor.dc(power)
    else:
        motor.brake()


async def control_loop(sensor, motors):
    while True:
        # Combine all channels before applying output: inactive channels must
        # not stop motors being controlled by another channel.
        targets = {port: 0 for port in motors}
        for channel, left_outputs, right_outputs in CHANNELS:
            buttons = await sensor.buttons(channel)
            left = rocker_power(buttons, Button.LEFT_UP, Button.LEFT_DOWN)
            right = rocker_power(buttons, Button.RIGHT_UP, Button.RIGHT_DOWN)
            for outputs, power in ((left_outputs, left), (right_outputs, right)):
                for port, direction in outputs:
                    targets[port] += power * direction
        for port, motor in motors.items():
            power = max(-POWER, min(POWER, targets[port]))
            apply_power(motor, power * MOTOR_SIGNS[port])
        await wait(20)


def main():
    # Device constructors must run before run_task starts multitasking.
    brick = EV3Brick()
    motors = {}
    try:
        sensor = InfraredSensor(IR_PORT)
        for port in MOTOR_SIGNS:
            motors[port] = Motor(port)

        brick.screen.clear()
        brick.screen.print("IR remote ready")
        brick.screen.print("1: BC / AD same")
        brick.screen.print("2: BC / AD opposite")
        brick.screen.print("3: B / C")
        brick.screen.print("4: A / D")
        brick.screen.print("Power:", POWER, "%")

        run_task(control_loop(sensor, motors))
    finally:
        for motor in motors.values():
            motor.brake()


main()
