#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import Font
from time import sleep


# Get sensor instances
colour_sensor = ColorSensor(Port.S2)
bumper_sensor = TouchSensor(Port.S3)
infrared_sensor = InfraredSensor(Port.S4)

# Get motor instances
left_wheel = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right_wheel = Motor(Port.C, Direction.COUNTERCLOCKWISE)
steer = Motor(Port.D, Direction.COUNTERCLOCKWISE)

# Get brick instance
brick = EV3Brick()

def home_steer(steering_motor, duty_limit = 60, speed = 100):
    """
    Home the steering wheel.
    This will recalibrate the steering wheel to make sure that it is centered on 0.
    """
    steering_motor.run_until_stalled(-speed, Stop.COAST, duty_limit) # Rotate the steering motor as far left as possible, until it stalls.
    negative_limit = steering_motor.angle() # Record the current angle as the leftmost angle

    steering_motor.run_until_stalled(speed, Stop.COAST, duty_limit) # Rotate the steering motor as far left as possible, until it stalls.
    positive_limit = steering_motor.angle() # Record the current angle as the rightmost angle

    center_angle = (negative_limit + positive_limit) // 2 # The center angle is the average of the leftmost and rightmost limits.

    steering_motor.run_target(150, center_angle, then = Stop.COAST) # Straighten the wheels
    steering_motor.reset_angle(0) # Set the curent angle as the 0-point

    max_left_angle = int((negative_limit - center_angle) * .9) # Set the leftmost angle, from our new centered point, as the distance between the limit and the center points. Keep a little margin (10%) for error.
    max_right_angle = int((positive_limit - center_angle) * .9) # Set the rightmost angle, from our new centered point, as the distance between the limit and the center points. Keep a little margin (10%) for error.

    return max_left_angle, max_right_angle

def output_text(text):
    """
    Writes text to the LCD screen and debug output.
    """
    brick.screen.print(text)
    print(text)

def main():
    """
    Code body that will be executed on startup.
    """
    # Prepare the LCD-screen
    brick.screen.clear()
    brick.screen.set_font(Font(size=12))

    # Some vehicle settings
    wheel_diameter_mm = 40
    wheel_track_mm = 120
    engine_acceleration = 250
    engine_speed = 200
    steer_speed = 250

    # Print some text to the LCD screen and the debug output
    output_text("RDW Lego")
    output_text("Self Driving Challenge")

    # Prepare the steering of the vehicle by calibrating/homing the steering motor
    output_text("Homing steering wheels...")
    max_left_angle, max_right_angle = home_steer(steer)
    output_text("Done!")

    # Prepare the engine by combining both motors
    output_text("Configuring drivetrain...")
    engine = DriveBase(left_wheel, right_wheel, wheel_diameter_mm, wheel_track_mm)
    engine.settings(straight_speed=engine_speed, straight_acceleration=engine_acceleration, turn_rate=0, turn_acceleration=0)
    output_text("Done!")

    # Make some room by driving backwards until the distance sensor is reporting atleast "50"
    while (infrared_sensor.distance() < 50):
        output_text("Too little space, moving backwards...")
        engine.straight(-100)

    # Wait for bumper touch sensor press
    output_text("Ready.... Set....")
    output_text("(waiting for bumper press)")
    while (not bumper_sensor.pressed()):
        wait(100)
    # Sound a beep, which is required for the competition to allow accurate lap-timing.
    brick.speaker.beep(500, 250)
    # Drive one centimeter
    output_text("GO!")
    engine.straight(10)

    # Dance sequence for demo purposes
    output_text("Starting dancing...")
    brick.speaker.say("Dancing the Self Driving Challenge Dance!")
    brick.speaker.play_notes(['E4/4', 'C4/4', 'D4/4', 'E4/4', 'D4/4', 'C4/4', 'C4/4', 'E4/4', 'B4/4', 'B4/4', 'A4/4', 'A4/4'], tempo=240)

def drive():
    pv = 55
    kp = 0.55
    ki = 0.0003
    kd = 2.5
    dt = 0.1
    integral = 0.0
    pre_error = 0.0

    brick.screen.clear()
    brick.screen.set_font(Font(size=12))

    # Some vehicle settings
    wheel_diameter_mm = 40
    wheel_track_mm = 120
    engine_acceleration = 250
    engine_speed = 200
    steer_speed = 250

    output_text("Homing steering wheels...")
    max_left_angle, max_right_angle = home_steer(steer)
    output_text("Done!")

    # Prepare the engine by combining both motors
    output_text("Configuring drivetrain...")
    engine = DriveBase(left_wheel, right_wheel, wheel_diameter_mm, wheel_track_mm)
    engine.settings(straight_speed=engine_speed, straight_acceleration=engine_acceleration, turn_rate=0, turn_acceleration=0)
    output_text("Done!")

    while (not bumper_sensor.pressed()):
        wait(100)
        
    # Sound a beep, which is required for the competition to allow accurate lap-timing.
    brick.speaker.beep(500, 250)
    # Drive one centimeter
    output_text("GO!")

    left_wheel.run(180)
    right_wheel.run(180)

    while True:
        #engine.straight(60)
        set_point = infrared_sensor.distance()
        #output_text(str(set_point))
        error = set_point - pv

        proportional_out = error * kp
        
        integral += error * dt
        integral_out = integral * ki

        derivative = (error - pre_error) / dt
        derivative_out = derivative * kd

        output = proportional_out + integral_out + derivative_out
        pre_error = error

        angle = max_left_angle + (max_right_angle - max_left_angle) * output / 100.0

        steer.track_target(output)
        wait(dt * 1000)

        output_text(integral)

        #if set_point < 10:
            #engine.straight(-40)

def drive2():
    brick.screen.clear()
    brick.screen.set_font(Font(size=12))

    # Some vehicle settings
    wheel_diameter_mm = 40
    wheel_track_mm = 120
    engine_acceleration = 250
    engine_speed = 200
    steer_speed = 250

    output_text("Homing steering wheels...")
    max_left_angle, max_right_angle = home_steer(steer)
    output_text("Done!")

    # Prepare the engine by combining both motors
    output_text("Configuring drivetrain...")
    engine = DriveBase(left_wheel, right_wheel, wheel_diameter_mm, wheel_track_mm)
    engine.settings(straight_speed=engine_speed, straight_acceleration=engine_acceleration, turn_rate=0, turn_acceleration=0)
    output_text("Done!")

    while (not bumper_sensor.pressed()):
        wait(100)
        
    # Sound a beep, which is required for the competition to allow accurate lap-timing.
    brick.speaker.beep(500, 250)
    # Drive one centimeter
    output_text("GO!")

    last_distance_2 = 0 
    while True: 
        engine.straight(40)
        distance_2 = infrared_sensor.distance()
        
        if distance_2 > last_distance_2:
            1+1
        else:
            if angle == 60:
                angle = -60
            if angle == -60:
                angle = 60

        steer.track_target(angle)
        last_distance_2 = distance_2

        if distance_2 < 30:
            engine.straight(-40)

def play_ride_of_the_valkyries():
    output_text("Starting to ride...")
    brick.speaker.say("Time to ride")
    brick.speaker.play_notes(["E4/4", "E4/4", "E4/4", "C4/4", "G4/4", "G4/4", "G4/4", "E4/4", "E4/4", "E4/4", "B3/4", "D4/4", "E4/4",
                              "E4/4", "E4/4", "C4/4", "G4/4", "G4/4", "G4/4", "E4/4", "E4/4", "E4/4", "B3/4", "D4/4", "E4/4",
                              "E4/4", "E4/4", "C4/4", "G4/4", "G4/4", "G4/4", "E4/4", "E4/4", "E4/4", "B3/4", "D4/4", "E4/4",
                              "F4/4", "F4/4", "F4/4", "A4/4", "C5/4", "A4/4", "F4/4", "E4/4", "E4/4", "E4/4", "D4/4", "D4/4", "D4/4",
                              "D4/4", "D4/4", "E4/4", "F4/4", "E4/4", "F4/4", "G4/4", "A4/4", "B4/4", "C5/4", "D5/4", "E5/4", "F5/4",
                              "G5/4", "A5/4", "G5/4", "F5/4", "E5/4", "D5/4", "C5/4", "B4/4", "A4/4", "G4/4", "F4/4", "E4/4", "D4/4",
                              "C4/4", "B3/4", "A3/4", "G3/4", "F3/4", "E3/4", "D3/4", "C3/4", "B2/4", "A2/4"], tempo=480)
            # sound.tonenot/], 400)
        # sleep(note[1] * 0.4)

        
if __name__ == '__main__':
    play_ride_of_the_valkyries()
    drive2()