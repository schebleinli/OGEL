# code fror 2.x
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

hub = PrimeHub()
motor_pair = MotorPair('A', 'E')
motor_pair.set_default_speed(40)
motor_pair.set_motor_rotation(27.64, 'cm')
motor_pair.set_stop_action('hold')
front_attachment = Motor('D') 
motor = Motor('E') 
left_color_sensor = ColorSensor('B')
right_color_sensor = ColorSensor('F')

# robot specific, change as necessary
gyro_correction = -0.9
gyro_integral = 0.001
gyro_derivative = 0
color_correction = .1
color_integral = 0.001
color_derivative = 0.5

def main():
  # put main code here, will be executed after all function definitions

def gyro_turn(angle, reset =0):
    currentAngle = hub.motion_sensor.get_yaw_angle() 
    if (reset == 0):
        hub.motion_sensor.reset_yaw_angle() 
        currentAngle = 0 
    lower = angle - 3 
    upper = angle + 3 
    while(currentAngle <= lower or currentAngle >= upper): 
        while (hub.motion_sensor.get_yaw_angle() >= angle ):
            motor_pair.start(-100,10) 
            currentAngle = hub.motion_sensor.get_yaw_angle() 
        while (hub.motion_sensor.get_yaw_angle() <= angle ): 
            motor_pair.start(100,10) 
            currentAngle = hub.motion_sensor.get_yaw_angle() 
        motor_pair.stop() 
      
def straight_by_gyro(distance, speed, bearing =0): 
    integral = 0 
    lastError = 0 
    degrees = distance * 360 / 27.63 
    motor.set_degrees_counted(0) 
    hub.motion_sensor.reset_yaw_angle() 
    while abs(motor.get_degrees_counted()) < degrees: 
        error = hub.motion_sensor.get_yaw_angle() - bearing 
        P_fix = error * gyro_correction 
        integral = integral + error 
        I_fix = integral * gyro_integral 
        derivative = error - lastError 
        lastError = error 
        D_fix = derivative * gyro_derivative 
        correction = I_fix + P_fix + D_fix 
        motor_pair.start_tank(int(speed+correction), int(speed-correction)) 
    motor_pair.stop() 
  
def follow_line_until(targetcolor, speed): 
    integral = 0
    lastError = 0 
    while (right_color_sensor.get_color() != targetcolor):
        error = left_color_sensor.get_reflected_light() - 50 
        P_fix = error * color_correction 
        integral = integral + error 
        I_fix = integral * color_integral 
        derivative = error - lastError 
        lastError = error 
        D_fix = derivative * color_derivative 
        correction = P_fix + I_fix + D_fix 
        motor_pair.start_tank_at_power(int(speed+correction), int(speed-correction)) 
    motor_pair.stop()
  
def go_until_color(targetcolor, sensor, speed, turn =0): 
    motor_pair.start(turn,speed) 
    while True: 
        if (sensor == "right"): 
            color = right_color_sensor.wait_for_new_color() 
            if (color == targetcolor): 
                motor_pair.stop() 
                break 
        if (sensor == "left"): 
            color = left_color_sensor.wait_for_new_color() 
            if (color == targetcolor): 
                motor_pair.stop() 
                break 

main()
raise SystemExit 
