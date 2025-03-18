#!/usr/bin/env python3
import rospy
import firebase_admin
from firebase_admin import credentials, db
from std_msgs.msg import Int32

# Initialize Firebase with explicit error handling
try:
    cred = credentials.Certificate("/home/sam/waste_management_robot/serviceAccountKey.json")
    firebase_admin.initialize_app(cred, {
        "databaseURL": "https://trashecomate-default-rtdb.asia-southeast1.firebasedatabase.app/"  # Replace with your actual URL
    })
    rospy.loginfo("Firebase initialized successfully")
except Exception as e:
    rospy.logerr(f"Failed to initialize Firebase: {e}")
    exit(1)

def check_bin_status():
    # Initialize ROS node
    rospy.init_node("firebase_listener", anonymous=True)
    pub = rospy.Publisher("/bin_level", Int32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        try:
            # Get bin status from Firebase
            ref = db.reference("bins/sensor1")
            bin_data = ref.get()
            if bin_data and "wasteLevel" in bin_data:
                bin_level = bin_data["wasteLevel"]
                rospy.loginfo(f"Bin Level: {bin_level}%")
                pub.publish(bin_level)
            else:
                rospy.logwarn("No wasteLevel data found in Firebase")
        except Exception as e:
            rospy.logerr(f"Firebase error: {e}")
        rate.sleep()

if __name__ == "__main__":
    try:
        check_bin_status()
    except rospy.ROSInterruptException:
        pass
