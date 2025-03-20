#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import firebase_admin
from firebase_admin import credentials, db
from std_msgs.msg import Int32
import time
import os
import numpy as np

# GPIO Pins for L293D
LEFT_IN1, LEFT_IN2 = 17, 18
RIGHT_IN3, RIGHT_IN4 = 27, 22
TRIG, ECHO = 23, 24
ENABLE_1, ENABLE_2 = 19, 12

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4, TRIG, ENABLE_1, ENABLE_2], GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output([ENABLE_1, ENABLE_2], GPIO.HIGH)

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
actions = ["forward", "backward", "turn_right", "stop"]
num_actions = len(actions)
distance_bins = [0, 10, 20, 50, 999]  # Discretize distance into bins
num_states = len(distance_bins) - 1
q_table = np.zeros((num_states, num_actions))  # Q-table
alpha = 0.1  # Learning rate
gamma = 0.9  # Discount factor
epsilon = 0.1  # Exploration rate

def discretize_distance(distance):
    for i in range(len(distance_bins) - 1):
        if distance_bins[i] <= distance < distance_bins[i + 1]:
            return i
    return len(distance_bins) - 2

def choose_action(state):
    if np.random.uniform(0, 1) < epsilon:
        return np.random.randint(num_actions)  # Explore
    return np.argmax(q_table[state])  # Exploit

def move_forward():
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN3, GPIO.HIGH)
    GPIO.output(RIGHT_IN4, GPIO.LOW)
    try:
        status_ref.set("Moving Forward")
    except Exception as e:
        rospy.logerr(f"Firebase status update failed: {e}")
    rospy.loginfo("Moving Forward")

def move_backward():
    GPIO.output(LEFT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN2, GPIO.HIGH)
    GPIO.output(RIGHT_IN3, GPIO.LOW)
    GPIO.output(RIGHT_IN4, GPIO.HIGH)
    try:
        status_ref.set("Moving Backward")
    except Exception as e:
        rospy.logerr(f"Firebase status update failed: {e}")
    rospy.loginfo("Moving Backward")

def turn_right():
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN3, GPIO.LOW)
    GPIO.output(RIGHT_IN4, GPIO.LOW)
    try:
        status_ref.set("Turning Right")
    except Exception as e:
        rospy.logerr(f"Firebase status update failed: {e}")
    rospy.loginfo("Turning Right")

def stop():
    GPIO.output([LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4], GPIO.LOW)
    try:
        status_ref.set("Off")
    except Exception as e:
        rospy.logerr(f"Firebase status update failed: {e}")
    rospy.loginfo("Stopping")

def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start_time = time.time()
    stop_time = time.time()
    while GPIO.input(ECHO) == 0 and time.time() - start_time < 0.1:
        start_time = time.time()
    while GPIO.input(ECHO) == 1 and time.time() - start_time < 0.1:
        stop_time = time.time()
    elapsed = stop_time - start_time
    distance = (elapsed * 34300) / 2 if elapsed > 0 else 999
    rospy.loginfo(f"Measured distance: {distance:.2f} cm")
    return distance

def check_bin_status():
    try:
        waste_level = rospy.wait_for_message("/bin_level", Int32, timeout=5).data
        rospy.loginfo(f"Checked bin status: {waste_level}%")
        return waste_level
    except rospy.ROSException as e:
        rospy.logwarn(f"Failed to receive bin level: {e}")
        return None

def motor_control():
    rospy.init_node("motor_control", anonymous=True)
    check_interval = 30  # Check bin status every 30 seconds
    last_check_time = time.time()
    collecting = False

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
                try:
                    status_ref.set("Active")
                except Exception as e:
                    rospy.logerr(f"Firebase status update failed: {e}")
                rospy.loginfo("Waste level > 90%, starting collection")
            else:
                collecting = False
                stop()
                try:
                    status_ref.set("Idle")
                except Exception as e:
                    rospy.logerr(f"Firebase status update failed: {e}")
                rospy.loginfo("Waste level <= 90%, robot idle")
            last_check_time = current_time

        # If collecting, navigate using Q-learning
        if collecting:
            distance = get_distance()
            state = discretize_distance(distance)
            action_idx = choose_action(state)
            action = actions[action_idx]

            # Execute action
            if action == "forward":
                move_forward()
                reward = 1 if distance >= 20 else -1
            elif action == "backward":
                move_backward()
                reward = 0 if distance < 20 else -1
            elif action == "turn_right":
                turn_right()
                reward = 0 if distance < 20 else -1
            else:
                stop()
                reward = -0.5

            # Update Q-table
            next_distance = get_distance()
            next_state = discretize_distance(next_distance)
            q_table[state, action_idx] = q_table[state, action_idx] + alpha * (reward + gamma * np.max(q_table[next_state]) - q_table[state, action_idx])
        else:
            stop()

        rospy.sleep(0.5)  # Check every 0.5 seconds for smooth navigation

    stop()

if __name__ == "__main__":
    try:
        motor_control()
    except rospy.ROSInterruptException:
        stop()
        GPIO.cleanup()