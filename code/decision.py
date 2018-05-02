import random

import numpy as np
from perception import RoverMode

MAX_STUCK_EFFORTS = 100
STUCK_MODE_THREHOLD = 100


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    print("rover nav_angles len: {}".format(len(Rover.nav_angles)))
    if Rover.nav_angles is not None:
        # If rover in FORWARD mode
        if Rover.mode == RoverMode.FORWARD:
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else:  # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                # Add a small percentage of perturbation to prevent rover from circling around the same region
                Rover.steer += Rover.steer * random.randint(-5, 5) / 100
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = RoverMode.STOP

        # If rover in STOP mode
        elif Rover.mode == RoverMode.STOP:
            # If rover in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15  # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                    Rover.mode = RoverMode.FORWARD
                # If we're already in "stop" mode then make different decisions

        # If sample is in sight
        elif Rover.mode == RoverMode.APPROACH_SAMPLE:
            # Slowly approach rock sample and stop if near it.
            if len(Rover.nav_angles) >= 2:
                if Rover.vel < 0.5:
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                else:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set / 5
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
            # Slowly approach rock sample and stop if near it, even if insufficient navigable terrain.
            else:
                if not Rover.near_sample:
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                    Rover.steer = 0
                # Stop rover if near rock sample
                else:
                    # Stop at the rock sample
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0

        # If rover is stuck
        elif Rover.mode == RoverMode.STUCK:
            # If not enough efforts are made, continue to do it to get rover out of stuck.
            if Rover.stuck_effort_counter >= 0:
                Rover.stuck_effort_counter -= 1
                # Back up to get out of stuck
                Rover.throttle = -Rover.throttle_set
                Rover.brake = 0
                # Turn to the opposite direction while backing up
                if len(Rover.nav_angles) > 2:
                    Rover.steer = -np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                else:
                    Rover.steer = -15
            # Enough efforts made, go back to FORWARD mode and give anther try.
            else:
                Rover.stuck_effort_counter = MAX_STUCK_EFFORTS
                Rover.throttle = Rover.throttle_set
                Rover.brake = 0
                Rover.steer = 0
                Rover.mode = RoverMode.FORWARD

    # Just to make the rover do something even if no modifications have been made to the code
    else:
        print("doing nothing")
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If rover is near a sample and ready to pick it up
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        print("picking up sample...")
    # If a sample has just been collected
    elif Rover.prev_samples_collected != Rover.samples_collected:
        Rover.prev_samples_collected = Rover.samples_collected
        print("picking up done!")
        Rover.mode = RoverMode.STOP
    # If rover is not near a sample and not picking up anything
    else:
        print("not picking up anything...")

    # Prevent rover from getting stuck
    if Rover.mode != RoverMode.STOP:
        if Rover.vel <= 0.1 and Rover.throttle > 0:
            # If rover got stuck for the first time
            if not Rover.stuck_in_prev_step and Rover.stuck_counter == 0:
                print("rover gets stuck for the first time")
                Rover.stuck_counter += 1
                Rover.stuck_in_prev_step = True
            # If rover got stuck in prior step
            else:
                Rover.stuck_counter += 1
                print("rover still gets stuck, stuck counter: {}".format(Rover.stuck_counter))
                # Change rover mode if got stuck for many steps
                if Rover.stuck_counter > STUCK_MODE_THREHOLD:
                    print("rover gets stuck too many steps, changing mode to STUCK")
                    Rover.mode = RoverMode.STUCK
                    Rover.stuck_counter = 0
                    Rover.stuck_in_prev_step = False
        else:
            # Reset rover status if not stuck in this step
            Rover.stuck_counter = 0
            Rover.stuck_in_prev_step = False

    return Rover
