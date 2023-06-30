import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge


def main():
    # Initialize the whisper ROS node and a publisher for the AI4HRIutterance topic
    rospy.init_node("camera_node", anonymous=True)
    rospy.Subscriber("/ai4hri/take_photo", String, callback)
    rospy.loginfo("Camera node initialized. Listening...")
    rospy.spin()


def callback(msg):

    # Initialize the camera
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    # Check if the camera is opened correctly
    if not cap.isOpened():
        raise IOError("Cannot open camera")

    # Capture and discard a few frames
    for _ in range(5):  # Adjust this number as needed
        ret, frame = cap.read()
        if not ret:
            rospy.loginfo("Cannot capture frame")
            return  # Exit the function if no frame was captured
    
    # Capture one frame
    ret, frame = cap.read()

    # Check if the frame was captured correctly
    if not ret:
        rospy.loginfo("Cannot capture frame")
        return  # Exit the function if no frame was captured

    # Save the frame as a .png file
    cv2.imwrite('photo.png', frame)

    # Load an image from file
    cv_image = cv2.imread("photo.png")

    # Convert the image to a ROS image message
    bridge = CvBridge()
    ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")

    pub = rospy.Publisher('/ai4hri/image', Image, queue_size= 1)
    
    # Publish the image.
    rospy.sleep(1)
    pub.publish(ros_image)
    rospy.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
