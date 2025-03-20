#!/usr/bin/env python3
import rospy
import firebase_admin
from firebase_admin import credentials, db
from std_msgs.msg import Int32
import os

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

# Firebase reference
waste_level_ref = db.reference("bins/sensor1/wasteLevel")

def firebase_listener():
    rospy.init_node("firebase_listener", anonymous=True)
    pub = rospy.Publisher("/bin_level", Int32, queue_size=10)
    rate = rospy.Rate(1)  # Publish every second

    while not rospy.is_shutdown():
        try:
            waste_level = waste_level_ref.get()
            if waste_level is not None:
                rospy.loginfo(f"Publishing waste level: {waste_level}%")
                pub.publish(waste_level)
            else:
                rospy.logwarn("No waste level data in Firebase")
        except Exception as e:
            rospy.logerr(f"Failed to read from Firebase: {e}")
        rate.sleep()

if __name__ == "__main__":
    try:
        firebase_listener()
    except rospy.ROSInterruptException:
        pass