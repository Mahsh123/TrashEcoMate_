#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import firebase_admin
from firebase_admin import credentials, db
from std_msgs.msg import Int32
import time
import os
import numpy as np
import signal
import sys
import math


# GPIO Pins for L293D and Servo
LEFT_IN1, LEFT_IN2 = 17, 18
RIGHT_IN3, RIGHT_IN4 = 27, 22
TRIG, ECHO = 23, 24
ENABLE_1, ENABLE_2 = 19, 12
SERVO_PIN = 13  # Servo motor pin (Physical Pin 33)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4, TRIG, ENABLE_1, ENABLE_2, SERVO_PIN], GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output([ENABLE_1, ENABLE_2], GPIO.HIGH)

# Servo setup using PWM
servo = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz PWM frequency for servo
servo.start(0)  # Initialize servo at 0% duty cycle

# Firebase setup
cred_path = os.getenv("FIREBASE_CREDENTIALS", "/home/sam/waste_management_robot/serviceAccountKey.json")
firebase_url = os.getenv("FIREBASE_URL", "https://trashecomate-default-rtdb.asia-southeast1.firebasedatabase.app/")
try:
    cred = credentials.Certificate(cred_path)
    firebase_admin.initialize_app(cred, {"databaseURL": firebase_url})
    rospy.loginfo("Firebase initialized successfully")
except Exception as e:
    rospy.logerr(f"Failed to initialize Firebase: {e}")
    exit(1)

status_ref = db.reference("robot/status")

# Q-Learning setup
actions = ["forward", "backward", "turn_right", "turn_left", "stop"]
num_actions = len(actions)
distance_bins = [0, 10, 20, 50, 999]  # Discretize distance into bins
num_states = len(distance_bins) - 1
q_table = np.zeros((num_states, num_actions))  # Q-table
alpha = 0.1  # Learning rate
gamma = 0.9  # Discount factor
epsilon = 0.1  # Exploration rate

# Robot position and orientation
robot_x = 0.0  # Starting position (x-coordinate in meters)
robot_y = 0.0  # Starting position (y-coordinate in meters)
robot_angle = 0.0  # Orientation in degrees (0 = facing positive x-axis, 90 = positive y-axis)

# Target location (e.g., bin location in the room)
target_x = 2.0  # meters
target_y = 1.0  # meters

# Robot movement parameters
SPEED = 0.5  # meters per second (adjust based on your robot's actual speed)
TURN_TIME = 0.5  # seconds to turn 90 degrees

def discretize_distance(distance):
    for i in range(len(distance_bins) - 1):
        if distance_bins[i] <= distance < distance_bins[i + 1]:
            return i
    return len(distance_bins) - 2

def choose_action(state):
    if np.random.uniform(0, 1) < epsilon:
        return np.random.randint(num_actions)  # Explore
    return np.argmax(q_table[state])  # Exploit

def set_servo_angle(angle):
    duty = 2.5 + (angle / 18.0)  # Linear mapping for 0-180 degrees
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)  # Allow time for servo to move
    servo.ChangeDutyCycle(0)  # Stop sending signal to prevent jitter

def move_forward(distance=None):
    global robot_x, robot_y
    if distance is None:
        # Default behavior for Q-learning (move for 1 second)
        duration = 1.0
    else:
        # Move a specific distance for targeted navigation
        duration = distance / SPEED
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN3, GPIO.HIGH)
    GPIO.output(RIGHT_IN4, GPIO.LOW)
    try:
        status_ref.set("Moving Forward")
    except Exception as e:
        rospy.logerr(f"Firebase status update failed: {e}")
    rospy.loginfo(f"Moving Forward for {duration:.2f} seconds")
    time.sleep(duration)
    # Update robot position based on orientation
    move_distance = SPEED * duration
    robot_x += move_distance * math.cos(math.radians(robot_angle))
    robot_y += move_distance * math.sin(math.radians(robot_angle))
    stop()

def move_backward(distance=None):
    global robot_x, robot_y
    if distance is None:
        # Default behavior for Q-learning (move for 1 second)
        duration = 1.0
    else:
        # Move a specific distance for targeted navigation
        duration = distance / SPEED
    GPIO.output(LEFT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN2, GPIO.HIGH)
    GPIO.output(RIGHT_IN3, GPIO.LOW)
    GPIO.output(RIGHT_IN4, GPIO.HIGH)
    try:
        status_ref.set("Moving Backward")
    except Exception as e:
        rospy.logerr(f"Firebase status update failed: {e}")
    rospy.loginfo(f"Moving Backward for {duration:.2f} seconds")
    time.sleep(duration)
    # Update robot position based on orientation
    move_distance = SPEED * duration
    robot_x -= move_distance * math.cos(math.radians(robot_angle))
    robot_y -= move_distance * math.sin(math.radians(robot_angle))
    stop()

def turn_right():
    global robot_angle
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN3, GPIO.LOW)
    GPIO.output(RIGHT_IN4, GPIO.LOW)
    try:
        status_ref.set("Turning Right")
    except Exception as e:
        rospy.logerr(f"Firebase status update failed: {e}")
    rospy.loginfo("Turning Right")
    time.sleep(TURN_TIME)  # Turn for 0.5 seconds (90 degrees)
    robot_angle = (robot_angle - 90) % 360  # Update orientation
    stop()

def turn_left():
    global robot_angle
    GPIO.output(LEFT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN3, GPIO.HIGH)
    GPIO.output(RIGHT_IN4, GPIO.LOW)
    try:
        status_ref.set("Turning Left")
    except Exception as e:
        rospy.logerr(f"Firebase status update failed: {e}")
    rospy.loginfo("Turning Left")
    time.sleep(TURN_TIME)  # Turn for 0.5 seconds (90 degrees)
    robot_angle = (robot_angle + 90) % 360  # Update orientation
    stop()

def stop():
    GPIO.output([LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4], GPIO.LOW)
    try:
        status_ref.set("Off")
    except Exception as e:
        rospy.logerr(f"Firebase status update failed: {e}")
    rospy.loginfo("Stopping")

def get_distance():
    try:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        start_time = time.time()
        stop_time = time.time()

        # Wait for ECHO to go high (start of echo pulse)
        while GPIO.input(ECHO) == 0 and time.time() - start_time < 0.1:
            start_time = time.time()

        # Wait for ECHO to go low (end of echo pulse)
        while GPIO.input(ECHO) == 1 and time.time() - start_time < 0.1:
            stop_time = time.time()

        elapsed = stop_time - start_time
        if elapsed <= 0 or time.time() - start_time >= 0.1:
            rospy.logwarn("Ultrasonic sensor timeout or invalid reading, returning default distance")
            return 999  # Default distance if sensor fails

        distance = (elapsed * 34300) / 2
        rospy.loginfo(f"Measured distance: {distance:.2f} cm")
        return distance
    except Exception as e:
        rospy.logerr(f"Error in get_distance: {e}")
        return 999  # Return default distance on error

def look_left():
    set_servo_angle(170)  # Look left
    distance = get_distance()
    set_servo_angle(90)  # Return to center
    return distance

def look_right():
    set_servo_angle(10)  # Look right
    distance = get_distance()
    set_servo_angle(90)  # Return to center
    return distance

def look_front():
    set_servo_angle(90)  # Look front
    distance = get_distance()
    return distance

def check_bin_status():
    try:
        waste_level = rospy.wait_for_message("/bin_level", Int32, timeout=5).data
        rospy.loginfo(f"Checked bin status: {waste_level}%")
        return waste_level
    except rospy.ROSException as e:
        rospy.logwarn(f"Failed to receive bin level: {e}")
        return None


def navigate_to_target(target_x, target_y):
    global robot_x, robot_y, robot_angle
    # Calculate the distance and angle to the target
    dx = target_x - robot_x
    dy = target_y - robot_y
    distance = math.sqrt(dx**2 + dy**2)
    target_angle = math.degrees(math.atan2(dy, dx))  # Angle to target in degrees

    # Calculate the angle to turn
    angle_diff = (target_angle - robot_angle) % 360
    if angle_diff > 180:
        angle_diff -= 360

    # Turn to face the target
    if abs(angle_diff) > 5:  # Allow small tolerance
        if angle_diff > 0:
            rospy.loginfo(f"Turning left by {angle_diff:.2f} degrees")
            while angle_diff > 0:
                turn_left()
                angle_diff -= 90
        else:
            rospy.loginfo(f"Turning right by {-angle_diff:.2f} degrees")
            while angle_diff < 0:
                turn_right()
                angle_diff += 90

    return distance  # Return the remaining distance to the target



def cleanup():
    rospy.loginfo("Cleaning up resources")
    stop()
    servo.stop()
    GPIO.cleanup()

def signal_handler(sig, frame):
    rospy.loginfo("Termination signal received, cleaning up")
    cleanup()
    sys.exit(0)

def motor_control():
    rospy.init_node("motor_control", anonymous=True)
    check_interval = 30  # Check bin status every 30 seconds
    last_check_time = time.time()
    collecting = False
    navigating_to_target = False  # Flag to switch between Q-learning and targeted navigation

    # Initialize servo position
    set_servo_angle(90)  # Start facing forward

    while not rospy.is_shutdown():
        # Check bin status every 30 seconds
        current_time = time.time()
        if current_time - last_check_time >= check_interval:
            waste_level = check_bin_status()
            if waste_level is None:
                rospy.logwarn("Unable to determine bin level, stopping robot")
                stop()
                try:
                    status_ref.set("Error: No bin level data")
                except Exception as e:
                    rospy.logerr(f"Firebase status update failed: {e}")
                continue

            if waste_level > 90:
                collecting = True
                navigating_to_target = True  # Start targeted navigation to the bin
                try:
                    status_ref.set("Active")
                except Exception as e:
                    rospy.logerr(f"Firebase status update failed: {e}")
                rospy.loginfo("Waste level > 90%, starting collection")
            else:
                collecting = False
                navigating_to_target = False
                stop()
                try:
                    status_ref.set("Idle")
                except Exception as e:
                    rospy.logerr(f"Firebase status update failed: {e}")
                rospy.loginfo("Waste level <= 90%, robot idle")
            last_check_time = current_time

        # If collecting, navigate with servo-based scanning
        if collecting:
            if navigating_to_target:
                # Navigate to the target location
                remaining_distance = navigate_to_target(target_x, target_y)
                if remaining_distance < 0.1:  # Stop when within 10 cm of target
                    navigating_to_target = False
                    collecting = False
                    try:
                        status_ref.set("Idle")
                    except Exception as e:
                        rospy.logerr(f"Firebase status update failed: {e}")
                    rospy.loginfo(f"Reached target location ({target_x}, {target_y}), stopping collection")
                    continue

            # After turning (or if not navigating to target), use Q-learning for navigation and obstacle avoidance
            rospy.loginfo("Entering navigation loop")
            distance_front = look_front()
            state = discretize_distance(distance_front)
            action_idx = None  # Initialize action_idx
            reward = 0  # Initialize reward

            if distance_front < 20 and distance_front != 999:
                # Move backward to avoid hitting the object
                stop()
                move_backward(0.5)  # Move backward 0.5 meters
                stop()
                rospy.loginfo("Moved backward to avoid obstacle, now scanning surroundings")

                # Scan surroundings after moving back
                distance_left = look_left()
                distance_right = look_right()
                distance_front = look_front()

                # Determine safest direction
                distances = {
                    "left": distance_left,
                    "right": distance_right,
                    "front": distance_front
                }
                safest_direction = max(distances, key=distances.get)
                safest_distance = distances[safest_direction]
                rospy.loginfo(f"Safest direction: {safest_direction} with distance {safest_distance:.2f} cm")

                if safest_distance < 20:
                    # No safe direction, move backward again
                    move_backward(0.5)
                    stop()
                    action_idx = actions.index("backward")
                    reward = -1
                else:
                    if safest_direction == "left":
                        turn_left()
                        action_idx = actions.index("turn_left")
                        reward = 1
                    elif safest_direction == "right":
                        turn_right()
                        action_idx = actions.index("turn_right")
                        reward = 1
                    else:
                        move_forward(0.5)  # Move forward 0.5 meters
                        action_idx = actions.index("forward")
                        reward = 1
            else:
                # No obstacle, use Q-learning for navigation
                action_idx = choose_action(state)
                action = actions[action_idx]
                rospy.loginfo(f"Q-learning chose action: {action}")

                if action == "forward":
                    move_forward(0.5)  # Move forward 0.5 meters
                    reward = 1 if distance_front >= 20 else -1
                elif action == "backward":
                    move_backward(0.5)
                    reward = 0 if distance_front < 20 else -1
                elif action == "turn_right":
                    turn_right()
                    reward = 0 if distance_front < 20 else -1
                elif action == "turn_left":
                    turn_left()
                    reward = 0 if distance_front < 20 else -1
                else:
                    stop()
                    reward = -0.5

            # Update Q-table
            next_distance = look_front()
            next_state = discretize_distance(next_distance)
            if action_idx is not None:
                q_table[state, action_idx] = q_table[state, action_idx] + alpha * (reward + gamma * np.max(q_table[next_state]) - q_table[state, action_idx])
            else:
                rospy.logwarn("action_idx is None, skipping Q-table update")

        else:
            stop()

        rospy.sleep(0.5)  # Check every 0.5 seconds for smooth navigation


if __name__ == "__main__":
    # Handle termination signals (e.g., Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        motor_control()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted, cleaning up")
        cleanup()
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        cleanup()
    finally:
        cleanup()