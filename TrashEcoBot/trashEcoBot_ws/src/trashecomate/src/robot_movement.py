#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import firebase_admin
from firebase_admin import credentials, db
from std_msgs.msg import Int32
import time

# GPIO Pins for L293D (BCM numbers)
LEFT_IN1, LEFT_IN2 = 17, 18  # Left motor (Pins 11, 12)
RIGHT_IN3, RIGHT_IN4 = 27, 22  # Right motor (Pins 13, 15)
TRIG, ECHO = 23, 24  # Ultrasonic sensor (Pins 16, 18)
ENABLE_1, ENABLE_2 = 19, 12  # Enable 1 (Pin 35), Enable 2 (Pin 32)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([LEFT_IN1, LEFT_IN2, RIGHT_IN3, RIGHT_IN4, TRIG, ENABLE_1, ENABLE_2], GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Set Enable pins HIGH to activate motors
GPIO.output([ENABLE_1, ENABLE_2], GPIO.HIGH)
rospy.loginfo("Enable pins set HIGH (Enable 1: GPIO 19, Enable 2: GPIO 12)")

# Initialize Firebase
try:
    cred = credentials.Certificate("/home/sam/waste_management_robot/serviceAccountKey.json")
    firebase_admin.initialize_app(cred, {
        "databaseURL": "https://trashecomate-default-rtdb.asia-southeast1.firebasedatabase.app/"
    })
    rospy.loginfo("Firebase initialized successfully")
except Exception as e:
    rospy.logerr(f"Failed to initialize Firebase: {e}")
    exit(1)

# Firebase reference for robot status (updated path)
status_ref = db.reference("robot/status")

# Test Firebase connectivity
try:
    status_ref.set("Off")  # Initial status set
    rospy.loginfo("Firebase status path verified and set to Off")
except Exception as e:
    rospy.logerr(f"Failed to set initial Firebase status: {e}")

def move_forward():
    rospy.loginfo("Setting forward: LEFT_IN1=1, LEFT_IN2=0, RIGHT_IN3=1, RIGHT_IN4=0")
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN3, GPIO.HIGH)
    GPIO.output(RIGHT_IN4, GPIO.LOW)
    try:
        status_ref.set("Moving")
    except Exception as e:
        rospy.logerr(f"Firebase status update failed: {e}")
    rospy.loginfo("Moving Forward")

def move_backward():
    rospy.loginfo("Setting backward: LEFT_IN1=0, LEFT_IN2=1, RIGHT_IN3=0, RIGHT_IN4=1")
    GPIO.output(LEFT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN2, GPIO.HIGH)
    GPIO.output(RIGHT_IN3, GPIO.LOW)
    GPIO.output(RIGHT_IN4, GPIO.HIGH)
    try:
        status_ref.set("Moving")
    except Exception as e:
        rospy.logerr(f"Firebase status update failed: {e}")
    rospy.loginfo("Moving Backward")

def turn_right():
    rospy.loginfo("Setting turn right: LEFT_IN1=1, LEFT_IN2=0, RIGHT_IN3=0, RIGHT_IN4=0")
    GPIO.output(LEFT_IN1, GPIO.HIGH)
    GPIO.output(LEFT_IN2, GPIO.LOW)
    GPIO.output(RIGHT_IN3, GPIO.LOW)
    GPIO.output(RIGHT_IN4, GPIO.LOW)
    try:
        status_ref.set("Moving")
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

def bin_callback(data):
    waste_level = data.data
    rospy.loginfo(f"Received waste level: {waste_level}%")
    if waste_level > 80:
        try:
            status_ref.set("Active")
        except Exception as e:
            rospy.logerr(f"Firebase status update failed: {e}")
        rospy.loginfo("Waste level > 80%, starting collection")
        while not rospy.is_shutdown() and waste_level > 80:
            distance = get_distance()
            if distance < 20 and distance != 999:
                stop()
                move_backward()
                time.sleep(1)
                turn_right()
                time.sleep(0.5)
                stop()
            else:
                move_forward()
            rospy.sleep(0.1)
            waste_level = rospy.wait_for_message("/bin_level", Int32).data
        stop()
        rospy.loginfo("Collection paused or complete")
    else:
        stop()
        rospy.loginfo("Waste level <= 80%, no action taken")

def motor_control():
    rospy.init_node("motor_control", anonymous=True)
    rospy.Subscriber("/bin_level", Int32, bin_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        motor_control()
    except rospy.ROSInterruptException:
        stop()
        GPIO.cleanup()