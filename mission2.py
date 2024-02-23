import color_sensor, motor_pair, motor, runloop
from hub import motion_sensor, port, button

#Inicializace
motor_pair.unpair(motor_pair.PAIR_1)
motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)
integral_error, prev_error = 0, 0
Kp, Ki, Kd = 5.0, 0.0, 3


async def main():

    #Druhá čára zprava
    await motor.run_for_degrees(port.D, -60, 300)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 880, -2, velocity=-400)
    await runloop.sleep_ms(500)
    await motor.run_for_degrees(port.D, 70, 300, stop=motor.HOLD)
    await runloop.sleep_ms(500)

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 200, 0, velocity=200)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 100, -100, velocity=200)

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 35, 100, velocity=200)
    await motor.run_for_degrees(port.D, -110, 200)

    
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 1000, 10, velocity=600)
   
    raise SystemExit("konec")

def gyro_reset():
    motion_sensor.reset_tilt_angles()


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