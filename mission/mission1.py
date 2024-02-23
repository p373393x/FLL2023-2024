import color_sensor, motor_pair, motor, runloop
from hub import motion_sensor, port, button

#Inicializace
motor_pair.unpair(motor_pair.PAIR_1)
motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)
integral_error, prev_error = 0, 0
Kp, Ki, Kd = 5.0, 0.0, 2.8


async def main():
    # 3. čára od černé zprava
    motor.run_for_degrees(port.C, -600, 500)
    
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -420, -35, velocity=400)
    await runloop.sleep_ms(150)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -410, 35, velocity=400)
    await runloop.sleep_ms(150)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 470, 0, velocity=400)
    await runloop.sleep_ms(150)
    #await motor_pair.move_for_degrees(motor_pair.PAIR_1, -420, 0, velocity=400)
    pid_reset()
    pid_degrees(420, 400)

    
    pid_reset()
    pid_degrees(800, 400)
    await runloop.sleep_ms(500)
    motor.run_for_degrees(port.D, -420, 1000)

    await runloop.sleep_ms(900)
    while color_sensor.reflection(port.F) > 50:
        motor_pair.move(motor_pair.PAIR_1, 0, velocity=-300)
    motor_pair.stop(motor_pair.PAIR_1)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -60, 0)

    #tam zpátky
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -130, 90, velocity=200)

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -90, 0, velocity=300)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 100, 0, velocity=300)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, -100, 0, velocity=300)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 300, 15, velocity=300)

    await runloop.sleep_ms(300)
    while color_sensor.reflection(port.F) > 50:
        motor_pair.move(motor_pair.PAIR_1, 0, velocity=-90)
    motor_pair.stop(motor_pair.PAIR_1)
    while color_sensor.reflection(port.E) > 50:
        motor_pair.move(motor_pair.PAIR_1, 70, velocity=-90)
    motor_pair.stop(motor_pair.PAIR_1)
    await runloop.sleep_ms(300)
    #Srovnani hotovo

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 330, -90, velocity=-200)
    await runloop.sleep_ms(300)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 550, 0, velocity=-400)
    await motor.run_for_degrees(port.D, -300, 1000)

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 95, 0, velocity=-400)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 110, 90, velocity=-100)
    await motor.run_for_degrees(port.C, 510, 500)
    await motor.run_for_degrees(port.C, -510, 500)

    motor.run_for_degrees(port.C, 620, 100)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 420, 30, velocity=400)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 380, 0, velocity=-400)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 210, -90, velocity=-600)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 400, -8, velocity=-1000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 980, -15, velocity=-1000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 550, -3, velocity=-1000)

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