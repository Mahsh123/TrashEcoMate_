#!/usr/bin/env python3
import rospy
import firebase_admin
from firebase_admin import credentials, db
from std_msgs.msg import Int32
import os
import time

# Force a fresh Firebase app initialization
if firebase_admin._apps:
    firebase_admin.delete_app(firebase_admin.get_app())

# Firebase setup
cred_path = os.getenv("FIREBASE_CREDENTIALS", "/home/sam/waste_management_robot/serviceAccountKey.json")
firebase_url = os.getenv("FIREBASE_URL", "https://trashecomate-default-rtdb.asia-southeast1.firebasedatabase.app/")

# Retry Firebase initialization with delay
max_retries = 3
retry_delay = 5  # seconds
for attempt in range(max_retries):
    try:
        cred = credentials.Certificate(cred_path)
        firebase_admin.initialize_app(cred, {"databaseURL": firebase_url})
        rospy.loginfo("Firebase initialized successfully")
        break
    except Exception as e:
        rospy.logerr(f"Failed to initialize Firebase (attempt {attempt + 1}/{max_retries}): {e}")
        if attempt < max_retries - 1:
            rospy.loginfo(f"Retrying in {retry_delay} seconds...")
            time.sleep(retry_delay)
        else:
            rospy.logerr("Max retries reached, exiting")
            exit(1)

# Firebase references
waste_level_ref = db.reference("bins/sensor1/wasteLevel")
gps_ref = db.reference("bins/sensor1/gps")

def firebase_listener():
    rospy.init_node("firebase_listener", anonymous=True)
    pub = rospy.Publisher("/bin_level", Int32, queue_size=10)
    rate = rospy.Rate(1)  # Publish every second

    while not rospy.is_shutdown():
        try:
            # Read waste level
            waste_level = waste_level_ref.get()
            if waste_level is not None:
                waste_level = int(waste_level)
                rospy.loginfo(f"Publishing waste level: {waste_level}%")
                pub.publish(waste_level)
            else:
                rospy.logwarn("No waste level data in Firebase")

            # Read and print GPS location
            gps_data = gps_ref.get()
            if gps_data is not None:
                latitude = gps_data.get("latitude", "N/A")
                longitude = gps_data.get("longitude", "N/A")
                rospy.loginfo(f"Bin GPS Location - Latitude: {latitude}, Longitude: {longitude}")
            else:
                rospy.logwarn("No GPS data in Firebase")
        except Exception as e:
            rospy.logerr(f"Failed to read from Firebase: {e}")
            time.sleep(1)  # Avoid spamming logs on repeated failures
        rate.sleep()

if __name__ == "__main__":
    try:
        firebase_listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupted, shutting down")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
    finally:
        if firebase_admin._apps:
            firebase_admin.delete_app(firebase_admin.get_app())