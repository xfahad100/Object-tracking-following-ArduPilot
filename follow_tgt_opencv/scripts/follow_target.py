#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
import cv2
import numpy as np
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget
import math

# Initialize ROS node
rospy.init_node('camera_tracker_node')

print(cv2.__version__)
selected_point = None
tracker = None
kalman_filter = cv2.KalmanFilter(6, 2)  # 6 state variables (position and velocity in x, y, and z), 2 measurement variables
kalman_filter.measurementMatrix = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]], np.float32)
kalman_filter.transitionMatrix = np.array([[1, 0, 1, 0, 0.5, 0], [0, 1, 0, 1, 0, 0.5], [0, 0, 1, 0, 1, 0], [0, 0, 0, 1, 0, 1], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]], np.float32)
kalman_filter.processNoiseCov = 0.1 * np.eye(6, dtype=np.float32)
kalman_filter.measurementNoiseCov = 1 * np.eye(2, dtype=np.float32)
predicted_state = None

# Function to handle mouse events
def mouse_callback(event, x, y, flags, param):
    global selected_point, tracker, predicted_state

    if event == cv2.EVENT_LBUTTONDOWN:
        selected_point = (x, y)
        tracker = cv2.TrackerCSRT_create()

        # Initialize the tracker with the current frame
        ok = True  # Set to True so that the tracker doesn't fail immediately
        frame = current_frame.copy()  # Get the current frame from the callback

        if ok:
            box_size = 40
            bbox = (x - box_size, y - box_size, 2 * box_size, 2 * box_size)
            tracker.init(frame, bbox)
            kalman_filter.statePost = np.array([[x], [y], [0], [0], [0], [0]], dtype=np.float32)
            predicted_state = kalman_filter.statePost[:4]

def pid_controller(kp, ki, kd, current_value, setpoint):
    prev_error = 0
    integral = 0

    error = setpoint - current_value
    integral += error
    derivative = error - prev_error

    output = (kp * error) + (ki * integral) + (kd * derivative)
    prev_error = error
    output = -output
        # Apply the control output to your system (e.g., adjust a motor, valve, etc.)
    return output
        # Sleep for a short duration before the next iteration to control update rate

def yaw_callback(data):
    #rospy.loginfo("Compass Heading: %.2f degrees", data.data)
    global copter_yaw
    copter_yaw = data.data*(math.pi/180)

# Create a bridge between ROS and OpenCV
bridge = CvBridge()

# Initialize the current frame variable
current_frame = None
center_x_bbox = 0.0
center_y_bbox = 0.0
center_x_cam = 0.0
center_y_cam = 0.0

# Function to handle incoming image messages
def image_callback(msg):
    global current_frame, center_x_cam, center_y_cam, center_x_bbox, center_y_bbox, send_vel_cmd
    send_vel_cmd = False
    try:
        # Convert the ROS Image message to an OpenCV image
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        current_frame = frame  # Store the current frame for the mouse_callback

        if selected_point is not None:
            ok, bbox = tracker.update(frame)

            if ok:
                x, y, w, h = [int(v) for v in bbox]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

                center_x = int(x + w / 2)
                center_y = int(y + h / 2)
                center_x_bbox = center_x
                center_y_bbox = center_y
                send_vel_cmd = True
                predicted_state = kalman_filter.predict()[:4]
                predicted_x, predicted_y = predicted_state[0], predicted_state[1]
                cv2.circle(frame, (int(predicted_x), int(predicted_y)), 5, (0, 255, 0), -1)
                measurement = np.array([[center_x], [center_y]], dtype=np.float32)
                kalman_filter.correct(measurement)

                cv2.putText(frame, f'Target: ({center_x}, {center_y})', (center_x, center_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                # Convert velocity from pixels per frame to meters per second
                predicted_vx, predicted_vy = predicted_state[2], predicted_state[3]
                # Convert velocity from pixels per frame to meters per second
                pixels_per_meter = 100  # Adjust this value based on your specific setup
                velocity_scale = 30  # Adjust this value to scale the displayed velocity
                velocity_mps = (predicted_vx / pixels_per_meter, predicted_vy / pixels_per_meter)
                velocity_text = f'Velocity: ({velocity_mps[0][0]:.2f} m/s, {velocity_mps[1][0]:.2f} m/s)'

                # Display the velocity
                text_size, _ = cv2.getTextSize(velocity_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                text_x = 10
                text_y = frame.shape[0] - 10
                cv2.putText(frame, velocity_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Get the dimensions of the frame
        frame_height, frame_width, _ = frame.shape

        # Calculate the coordinates of the center point of the frame
        center_x = int(frame_width / 2)
        center_y = int(frame_height / 2)

        center_x_cam = center_x
        center_y_cam = center_y
        if send_vel_cmd == True:
            velocity_msg.header.stamp = rospy.Time.now()
            pos_x_out = pid_controller(0.00095,0.0001,0.00005, center_x_cam, center_x_bbox)
            #print("pos_error x:", pos_x_out)
            velocity_msg.twist.angular.z = pos_x_out
            pos_y_out = pid_controller(0.001,0.0002,0.01, center_y_cam, center_y_bbox)
            velocity_msg.twist.linear.z = pos_y_out  # Set desired angular velocity in the Z-axis (yaw rate, adjust as needed)
            #print("pos_error y:", pos_y_out)
            #if pos_x_out < 0.1 and pos_y_out < 0.1:
                # velocity_msg.twist.linear.x = 0.5*math.cos(copter_yaw)  # Set desired linear velocity in the X-axis (adjust as needed)
            velocity_msg.twist.linear.x = 0.5*math.sin(copter_yaw)
            velocity_msg.twist.linear.y = 0.5*math.cos(copter_yaw)
                #velocity_msg.twist.linear.y = 0.5
                #velocity_msg.twist.linear.y = 0.3*math.sin(copter_yaw)
            print("linear y:", velocity_msg.twist.linear.y)
                #print("linear x:", velocity_msg.twist.linear.x)
        else:
            velocity_msg.header.stamp = rospy.Time.now()
            velocity_msg.twist.linear.x = 0.0  # Set desired linear velocity in the X-axis (adjust as needed)
            velocity_msg.twist.linear.y = 0.0
            velocity_msg.twist.linear.y = 0.0
            velocity_msg.twist.angular.z = 0.0

        velocity_pub.publish(velocity_msg)


        # Display the center point of the frame
        cv2.circle(frame, (center_x, center_y), 3, (255, 255, 255), -1)
        cv2.putText(frame, f'({center_x}, {center_y})', (center_x, center_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Set the desired window size
        window_width = 700
        window_height = 500

        # Resize the display window
        cv2.namedWindow('Frame', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Frame', window_width, window_height)

        # Create a named window and set mouse callback
        cv2.namedWindow('Frame')
        cv2.setMouseCallback('Frame', mouse_callback)
        # Display the frame
        cv2.imshow('Frame', frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rospy.signal_shutdown("Shutting down")

    except CvBridgeError as e:
        print(e)

# Subscribe to the camera topic
rospy.Subscriber("rrbot/camera1/image_raw", Image, image_callback)
rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, yaw_callback)

# Create a TwistStamped message to set linear and angular velocities
velocity_msg = TwistStamped()
velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)


# Publish the velocity command


# Keep the ROS node running
rospy.spin()
