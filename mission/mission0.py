import color_sensor, motor_pair, motor, runloop
from hub import motion_sensor, port

#Inicializace
motor_pair.unpair(motor_pair.PAIR_1)
motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)
integral_error, prev_error = 0, 0
Kp, Ki, Kd = 5.0, 0.0, 3


async def main():
    # 2. od první černé zprava
    pid_reset()
    pid_degrees(540, 300)
    await runloop.sleep_ms(200)
    
    motor.run_for_degrees(port.A, -100, 200)
    await motor.run_for_degrees(port.B, -100, 200)

    pid_reset()
    await runloop.sleep_ms(200)  
    pid_degrees(350, 400)
    await motor.run_for_time(port.C, 900, -500)
    await runloop.sleep_ms(700)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -80, 0)
    await motor.run_for_time(port.C, 300, -500)

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 40, 90, velocity=90)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 160, 90, velocity=180)

    motor.run_for_degrees(port.C, 480, 1000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 230, 90)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -700, 0, velocity=1000)

    raise SystemExit("konec")

def gyro_reset():
    motion_sensor.reset_tilt_angles()

def gyro_turn(degrees, speed):
    motion_sensor.reset_tilt_angles()
    while motion_sensor.tilt_angles()[0] > -degrees:
        motor_pair.move(motor_pair.PAIR_1, 90, velocity=speed)
    motor_pair.stop(motor_pair.PAIR_1)


def pid_reset(*,kp=Kp,ki=Ki,kd=Kd):
    global integral_error, prev_error, Kp, Ki, Kp
    motion_sensor.reset_yaw_angle(0)
    integral_error, prev_error = 0, 0
    Kp, Ki, Kp = kp, ki, kd


def pid_control(speed):
    global integral_error, prev_error
    gyro_angle = motion_sensor.tilt_angles()[0]

    integral_error += (0 - gyro_angle)
    output = Kp * (0 - gyro_angle) + Ki * integral_error + Kd * ((0 - gyro_angle) - prev_error)
    prev_error = (0 - gyro_angle)
    
    motor.run(port.A, int(speed + output))
    motor.run(port.B, -int(speed - output))

def pid_degrees(degrees, speed):
    motor.reset_relative_position(port.A, 0)
    while motor.relative_position(port.A) < degrees:
        pid_control(speed)
    motor.stop(port.A)
    motor.stop(port.B)


runloop.run(main())