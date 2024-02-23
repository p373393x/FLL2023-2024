import color_sensor, motor_pair, motor, runloop
from hub import motion_sensor, port, button

#Inicializace
motor_pair.unpair(motor_pair.PAIR_1)
motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)
integral_error, prev_error = 0, 0
Kp, Ki, Kd = 5.0, 0.0, 3

async def main():
    
    # 2. čára zprava
    pid_reset()
    motor.run_for_degrees(port.D, 180, 200)
    pid_degrees(1150, 400)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 100, -100, velocity=200)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 450, 0, velocity=-300)
    
    #Otoceni na gyro
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 220, 90, velocity=200)
    
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 235, 0, velocity=200)

    pid_reset()
    await runloop.sleep_ms(100)
    pid_degrees(520, 200)

    await motor.run_for_degrees(port.D, 330, 400)
    pid_reset()
    
    motor.reset_relative_position(port.A, 0)
    while motor.relative_position(port.A) > -100:
        pid_control(-350)
    motor.stop(port.A)
    motor.stop(port.B)

    await motor.run_for_degrees(port.A, 320, -250)
    await motor.run_for_degrees(port.C, 240, -275) #panáci
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 210, 0, velocity=300)
    await motor.run_for_degrees(port.A, 100, 300) #seřizeni na tisk

    #netrefit tisk
    pid_reset()
    motor.reset_relative_position(port.A, 0)
    while motor.relative_position(port.A) > -400:
        pid_control(-290)
    motor.stop(port.A)
    motor.stop(port.B)

    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 900, 28, velocity=1000)
    
    # vyzkumak
    motor.run_for_time(port.C, 1500, 1000)
    await runloop.sleep_ms(1000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 100, 30, velocity=1000)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, 200, -28, velocity=1000)
    await runloop.sleep_ms(200)
    motor.stop(port.C)

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