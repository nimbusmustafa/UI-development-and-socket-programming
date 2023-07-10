

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QPushButton ,QHBoxLayout
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import QTimer
from sensor_msgs.msg import Imu, NavSatFix, Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import LaserScan
import pyqtgraph as pg
import pandas as pd
import math
from tf.transformations import quaternion_from_euler
import tf.transformations as tf


class MainWindow(QMainWindow):
    counter =1
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle('Socket GUI Task')
        self.setGeometry(100, 100, 800, 600)

        self.imu_data = None
        self.imu_rover_data = None
        self.obstacle_detected = False
        self.display_quaternion = False
        self.video_frame = None
        self.recording = False
        self.recorded_data = []  

        self.init_ui()
        self.init_ros()

    def init_ui(self):
        self.central_widget = QWidget()

        self.setCentralWidget(self.central_widget)
        vertical_layout = QVBoxLayout(self.central_widget)

        self.imu_label = QLabel('IMU Data:')
        vertical_layout.addWidget(self.imu_label)
        
        self.plot = pg.PlotWidget(title="IMU Orientation")
        self.plot.setMinimumSize(400, 400)
        self.plot.setLabel('left', 'Angle (degrees)')
        self.plot.setLabel('bottom', 'Time')

        self.curve_roll = self.plot.plot(pen='r', name='Roll')
        self.curve_pitch = self.plot.plot(pen='g', name='Pitch')
        self.curve_yaw = self.plot.plot(pen='b', name='Yaw')

        vertical_layout.addWidget(self.plot)
        
        # self.layout.addLayout(self.imu_layout)

        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []
        
        
        self.test_label = QLabel('Test Feed:')
        vertical_layout.addWidget(self.test_label)

        self.video_label = QLabel('Video Feed:')
        self.video_label.setFixedSize(400, 300)
        vertical_layout.addWidget(self.video_label)

        self.gps_label = QLabel('GPS Data:')
        vertical_layout.addWidget(self.gps_label)

        self.obstacle_label = QLabel('Obstacle Status: No')
        vertical_layout.addWidget(self.obstacle_label)

        horizontal1_layout = QHBoxLayout()
        self.left_box = QLabel('LEFT')
        self.left_box.setStyleSheet("background-color: green")
        horizontal1_layout.addWidget(self.left_box)

        self.front_box = QLabel('FRONT')
        self.front_box.setStyleSheet("background-color: green")
        horizontal1_layout.addWidget(self.front_box)

        self.right_box = QLabel('RIGHT')
        self.right_box.setStyleSheet("background-color: green")
        horizontal1_layout.addWidget(self.right_box)

        vertical_layout.addLayout(horizontal1_layout)

        horizontal_layout = QHBoxLayout()

        self.imu_button = QPushButton('Toggle IMU orientation')
        self.imu_button.clicked.connect(self.toggle_imu_orientation)
        horizontal_layout.addWidget(self.imu_button)

        self.screenshot_button = QPushButton('Capture Screenshot')
        self.screenshot_button.clicked.connect(self.capture_screenshot)
        horizontal_layout.addWidget(self.screenshot_button)

        self.record_button = QPushButton('Record Video')
        self.record_button.clicked.connect(self.record_video)
        horizontal_layout.addWidget(self.record_button)

        vertical_layout.addLayout(horizontal_layout)

    def init_ros(self):
        rospy.init_node('gui')

        rospy.Subscriber('/gui_imu', Imu, self.imu_callback)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.video_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/imu'  , Imu ,self.imu_rover_callback)
        
    def imu_rover_callback(self,msg):
        self.imu_rover_data = msg  
        roll, pitch, yaw = self.quaternion_to_euler(
        self.imu_rover_data.orientation.x,
        self.imu_rover_data.orientation.y,
        self.imu_rover_data.orientation.z,
        self.imu_rover_data.orientation.w      )
        self.roll_data.append(roll)
        self.pitch_data.append(pitch)
        self.yaw_data.append(yaw)                    
        self.curve_roll.setData(self.roll_data)
        self.curve_pitch.setData(self.pitch_data)
        self.curve_yaw.setData(self.yaw_data)

        
    def imu_callback(self, msg):
        self.imu_data = msg
        self.update_imu_label()
        self.recorded_data.append({'Acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                           'Velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                           'Orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]})


    def lidar_callback(self, msg):
        lidar_ranges = msg.ranges

        right_ranges = lidar_ranges[0:240]
        front_ranges = lidar_ranges[241:480]
        left_ranges = lidar_ranges[481:720]

        max_range = 2.0

        right_obstacle = min(right_ranges) < max_range
        front_obstacle = min(front_ranges) < max_range
        left_obstacle = min(left_ranges) < max_range

        if right_obstacle and front_obstacle and left_obstacle:
            self.obstacle_label.setText('Obstacle Status: Detected on the Right, Front, and Left')
        elif right_obstacle and front_obstacle:
            self.obstacle_label.setText('Obstacle Status: Detected on the Right and Front')
        elif right_obstacle and left_obstacle:
            self.obstacle_label.setText('Obstacle Status: Detected on the Right and Left')
        elif front_obstacle and left_obstacle:
            self.obstacle_label.setText('Obstacle Status: Detected in Front and Left')
        elif right_obstacle:
            self.obstacle_label.setText('Obstacle Status: Detected on the Right')
        elif front_obstacle:
            self.obstacle_label.setText('Obstacle Status: Detected in Front')
        elif left_obstacle:
            self.obstacle_label.setText('Obstacle Status: Detected on the Left')
        else:
            self.obstacle_label.setText('Obstacle Status: No Obstacle Detected')

        self.front_box.setStyleSheet("background-color: red" if front_obstacle else "background-color: green")
        self.right_box.setStyleSheet("background-color: red" if right_obstacle else "background-color: green")
        self.left_box.setStyleSheet("background-color: red" if left_obstacle else "background-color: green")
        self.recorded_data.append({'Obstacle': [ left_obstacle,front_obstacle, right_obstacle]})


    def gps_callback(self, msg):
        current_latitude = msg.latitude
        current_longitude = msg.longitude
        self.gps_label.setText(f'GPS Data:\nLatitude: {current_latitude}\nLongitude: {current_longitude}')

        self.recorded_data.append({'GPS': [current_latitude, current_longitude]})


    def video_callback(self, msg):
        try:
            self.video_msg = msg
            self.test_label.setText('Video Feed:')

            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            height, width, channel = rgb_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            self.video_label.setPixmap(pixmap.scaledToWidth(400))
            self.video_frame = cv_image  
            
            if self.recording:
                self.video_writer.write(cv_image) 

        except Exception as e:
            rospy.logerr(e)

    def toggle_imu_orientation(self):
        self.display_quaternion = not self.display_quaternion
        self.update_imu_label()

    def quaternion_to_euler(self, x, y, z, w):
        quaternion = (x, y, z, w)
        euler = tf.euler_from_quaternion(quaternion)
        return euler

    def update_imu_label(self):
        if self.imu_data:
            imu_data = f"Linear Acceleration: {self.imu_data.linear_acceleration}\n" \
                       f"Angular Velocity: {self.imu_data.angular_velocity}\n"

            if self.display_quaternion:
                imu_data += f"Orientation (Quaternion): X={self.imu_data.orientation.x}, " \
                            f"Y={self.imu_data.orientation.y}, Z={self.imu_data.orientation.z}, " \
                            f"W={self.imu_data.orientation.w}"
            else:
                roll, pitch, yaw = self.quaternion_to_euler(
                    self.imu_data.orientation.x,
                    self.imu_data.orientation.y,
                    self.imu_data.orientation.z,
                    self.imu_data.orientation.w
                )
                imu_data += f"Orientation (Euler Angles): Roll={math.degrees(roll)}, " \
                            f"Pitch={math.degrees(pitch)}, Yaw={math.degrees(yaw)}"

            self.imu_label.setText(f'IMU Data:\n{imu_data}')

    def capture_screenshot(self):
        try:
            if self.video_frame is not None:
                filename = f"screenshot_{self.counter}.png"
                cv2.imwrite(filename, self.video_frame)
                self.test_label.setText(f'Screenshot captured: {filename}')
                self.counter += 1
            else:
                self.test_label.setText('No video frame available')
        except Exception as e:
            rospy.logerr(e)

    def record_video(self):
     if not self.recording:
        self.recording = True
        self.record_button.setText('Stop Recording')

        video_file = 'recorded_video.mp4'
        video_codec = cv2.VideoWriter_fourcc(*'mp4v')
        video_fps = 30.0
        video_frame_size = (self.video_frame.shape[1], self.video_frame.shape[0])

        
        self.video_writer = cv2.VideoWriter(video_file, video_codec, video_fps, video_frame_size)

     else:
        self.recording = False
        self.record_button.setText('Record Video')

        self.video_writer.release()

    def save_recorded_data(self):
        if len(self.recorded_data) > 0:
            df = pd.DataFrame(self.recorded_data)
            df.to_csv('recorded_data.csv', index=False)
            self.test_label.setText('Recorded data saved successfully!')
        else:
            self.test_label.setText('No recorded data to save')

    def closeEvent(self, event):
       
        self.save_recorded_data()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())