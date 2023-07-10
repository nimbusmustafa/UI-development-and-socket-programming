# UI-development-and-socket-programming
## Socket Pragramming
Using pre-made applications(HyperIMU) to send your phones IMU data to your laptop using socket programming. The data is published onto a rostopic and is visualised on RViz.
### Code Breakdown - server.py
1) These lines import the necessary modules for socket communication, ROS functionality, mathematical operations, and message types.
```sh
import socket
import rospy
import math
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
```
2) These lines define the host IP address and port number that will be used for socket communication. Your IP address  and port number in your computer should be same in the HyperIMU settings, and both devices should be connected on same wi-fi
```sh
HOST = '  '
PORT = 8000
```
3) This function euler_to_quaternion converts Euler angles (roll, pitch, yaw) into quaternion representation using the quaternion_from_euler function from the tf.transformations module.
```sh
def euler_to_quaternion(roll, pitch, yaw):
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    return quaternion
```
4) This function socket_callback initializes a ROS node with the name 'imu_pub'. The anonymous=True flag ensures that the node has a unique name by appending a unique identifier to the node name.
```sh
def socket_callback():
    rospy.init_node('imu_pub', anonymous=True)
```
5) These lines create a socket object using socket.socket() with the AF_INET family and SOCK_STREAM type. Then, the server socket is bound to the specified host and port using bind().
```sh
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
```
   
6) Here, the server socket starts listening for incoming connections with a backlog of 1 (maximum number of queued connections). It also prints a message indicating that the server is listening on the specified host and port.
```sh
server_socket.listen(1)
print("Server listening on {}:{}".format(HOST, PORT))
```
7) The accept() method blocks until a client establishes a connection. It returns a new socket object representing the connection and the client's address. Here, the server socket accepts the connection and prints a message indicating the client's address
```sh
client_socket, client_address = server_socket.accept()
print("Connected to client:", client_address)
```
8) These lines decode the received data from bytes to a string using UTF-8 encoding. It removes any carriage return ("\r") and newline ("\n") characters. Then, it splits the string into a list of values using comma (',') as the delimiter. Any empty strings in the list are removed.
```sh
data1 = data.decode("utf-8")
data1 = data1.replace("\r", "").replace("\n", "")
values = data1.split(',')

try:
 values.remove('')
except ValueError:
 pass
```
9) Finally, after the loop exits, the client and server sockets are closed to release the resources.
```sh
client_socket.close()
server_socket.close()
```
### IMU visualisation on RVIZ
![Screenshot from 2023-07-10 22-50-27](https://github.com/nimbusmustafa/UI-development-and-socket-programming/assets/117943931/9cb0f045-7e11-4abb-8c90-3ff90a52f1e2)

## UI development
Created a desktop GUI application capable of displaying real-time dynamic data and displayed the following data from ROS info streams -

*IMU data retrieved from the phone
```sh
  def imu_callback(self, msg):
        self.imu_data = msg
        self.update_imu_label()
        
```

* IMU Grpah retrieved from the gazebo bot launched from other terminal
  ![Screenshot from 2023-07-10 23-56-46](https://github.com/nimbusmustafa/UI-development-and-socket-programming/assets/117943931/a33bbc03-4dec-43d2-8684-8cec0af4ae56)

* Video feed from the 4 wheeled gazebo bot
Here, CvBridge is used to convert the ROS image message (msg) to a format compatible with OpenCV. imgmsg_to_cv2() converts the ROS image message to a NumPy array (cv_image). Then, cv2.cvtColor() converts the image from the BGR color space to the RGB color space (rgb_image).
```sh
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
```
* GPS data from the gazebo bot
```sh
    def gps_callback(self, msg):
        current_latitude = msg.latitude
        current_longitude = msg.longitude
        self.gps_label.setText(f'GPS Data:\nLatitude: {current_latitude}\nLongitude: {current_longitude}')

        self.recorded_data.append({'GPS': [current_latitude, current_longitude]})
```
* Visualised a way to display whether an obstacle has been detected or if the path is clear
![Screenshot from 2023-07-11 00-03-11](https://github.com/nimbusmustafa/UI-development-and-socket-programming/assets/117943931/7768741f-1105-4ddc-ab57-ea8c62a8ad48)
Here, i have subscribed to the lidar topic of the gazebo bot and divided it into three ranges i.e Right Front and Left. The respective buttons turn red if theres an obstacle detected in that range.
```sh
 def lidar_callback(self, msg):
        lidar_ranges = msg.ranges

        right_ranges = lidar_ranges[0:240]
        front_ranges = lidar_ranges[241:480]
        left_ranges = lidar_ranges[481:720]

        max_range = 2.0

        right_obstacle = min(right_ranges) < max_range
        front_obstacle = min(front_ranges) < max_range
        left_obstacle = min(left_ranges) < max_range
```
* Created a toggle button which will change the IMU data representation switch between Euler angle and quaternions .
```sh
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
```
* Created a screenshot button to save an image from the videostream and a record button to record a video and save it.
  
 If self.recording is initially False, indicating that video recording is not in progress, the code enters this block. It sets self.recording to True, indicating that recording is now active. It also updates the text of a button, represented by self.record_button, to display "Stop Recording."
 The code then specifies the details for the output video file. It sets the video_file variable to the name of the output video file ('recorded_video.mp4'). The video_codec is set to 'mp4v', which corresponds to the MP4 video format. The video_fps represents the desired frames per second for the output video, set to 30.0 in this case. Finally, video_frame_size is set to the dimensions of self.video_frame, which represents the size of each video frame.
 A cv2.VideoWriter object is created using the specified video file, codec, frame rate, and frame size. This object, self.video_writer, will be used to write video frames to the output file.
 If self.recording is True, indicating that video recording is already in progress, the code enters this block. It sets self.recording to False to stop the recording. It also updates the text of self.record_button to display "Record Video."
 Finally, the release() method is called on the self.video_writer object. This releases any resources held by the VideoWriter and finalizes the output video file
```sh
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
```

* All data recorded should be stored in either CSV format
 ![Screenshot from 2023-07-11 00-07-43](https://github.com/nimbusmustafa/UI-development-and-socket-programming/assets/117943931/93f5bff6-a5c4-4692-a9d6-62a5a21c62de)

## GUI Application
  ![Screenshot from 2023-07-08 22-34-03](https://github.com/nimbusmustafa/UI-development-and-socket-programming/assets/117943931/b3a9b2be-7e99-4d43-8b70-dec0c1110d68)

