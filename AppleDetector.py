import cv2 # OpenCV is a library of programming functions mainly aimed at real-time computer vision
import numpy as np
import pyrealsense2 as rs #Intel RealSense SDK 2.0 is a cross-platform library for Intel RealSense depth cameras (D400 series and the SR300)
import csv #
from ultralytics import YOLO #Ultralytics YOLO is a state-of-the-art, real-time object detection system that is open-source and free to use
from pydobot import Dobot #pydobot is a Python library for controlling Dobot Magician robotic arm
import pythoncom # Import pythoncom for COM initialization
from serial.tools import list_ports

class AppleDetector:
    """
    Apple detector class
    """
    def __init__(self):
        # Configure RealSense pipeline
        #rs.pipeline() is used to manages the streaming of frames from the cameraq
        #rs.config() is used to create a configuration object that defines the settings of the pipeline
        self.camera_offset = [200,-268, 70] # unit: mm
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) #640 is the width, 480 is the height, z16 is the format, 30 is the FPS
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(config) 
        self.align = rs.align(rs.stream.color) # align the depth and color frames

        #check is there any available port for the Dobot Magician
        available_ports = list_ports.comports() #list_ports.comports() is used to list all the available ports
        if not available_ports:
            print("No available ports found.")
            return
        else: 
            print(f"Available ports: {[x.device for x in available_ports]}")
            port = available_ports[0].device #get the first available port
            print(f"Using port: {port}")
        # Get depth scale
        #get_depth_scale() is used to get the depth scale of the depth sensor
        # first_depth_sensor() is used to get the first depth sensor of the device
        # depth_scale is used to get the depth scale of the depth sensor
        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()

        #DOBOT initialization
        pythoncom.CoInitialize() #Initialize the COM library
        try:
            print("Connecting to Dobot Magician")
            self.dobot = Dobot(port= port, verbose=True)  #verbose=True is used to print the debug information
            print(" Successfully connected to Dobot Magician")

        except Exception as e:
            print("Failed to connect to Dobot Magician")
            self.dobot = None

        if self.dobot:    
            self.dobot.speed(70, 70)
            self.home_position = (70, 264, 65, 0)  # x, y, z, r unit: mm
            self.dobot.move_to(*self.home_position) # move to the home position

        # Load YOLO11 model
        self.model = YOLO("yolo11x.pt")
        self.target_class = "apple"

        # CSV output file
        self.csv_file_1 = "detected_apples.csv"
        self.csv_file_2 = "picked_apples.csv"
        #Wite the head line of the CSV file
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Index", "X (m)", "Y (m)", "Z (m)", "Distance (m)"])

    def cam_to_robot(self, x_cam, y_cam, z_cam):
        """Convert camera coordinates to robot coordinates (in mm)"""
        x_robot = x_cam + self.camera_offset[0]
        y_robot = z_cam + self.camera_offset[1]  # rotate 90 degrees along the X axis(pitch angle)
        z_robot = -y_cam + self.camera_offset[2]  # the y-axis of the camera is the negative z-axis of the robot
        return x_robot, y_robot, z_robot
    
    def move_to_apple(self, x, y, z):
        """Move the arm to the apple position"""
        x_mm = x * 1000  # convert to mm
        y_mm = y * 1000
        z_mm = z * 1000
        target_position = self.cam_to_robot(x_mm, y_mm, z_mm)
        # Move to the target's x and y coordinates first, keeping the current z (height)
        #self.dobot.pose() is used to get the current pose of the robot
        self.dobot.move_to(target_position[0], target_position[1], self.dobot.pose()[2], 0)
        self.dobot.wait(500)  # wait 500ms
        # Then move to the target's z coordinate (height)
        self.dobot.move_to(target_position[0], target_position[1], target_position[2], 0)
        self.dobot.wait(500)  # wait 500ms
    
    def pick_apple(self):
        """Pick up the apple"""
        self.dobot.suck(True)  # turn on the gripper
        self.dobot.wait(1000)  # wait
        # Move to the hight of the home position
        self.dobot.move_to(self.dobot.pose()[0],self.dobot.pose()[1], self.home_position[2],0)  
        self.dobot.wait(500)  # wait for 0.5s
        self.dobot.move_to(*self.home_position)  # move to the home position
        self.dobot.wait(500)  # wait for 0.5s
        self.dobot.suck(False)  # close the gripper

    def get_3d_coordinates(self, depth_frame, x, y):
        """Calculate 3D coordinates from depth data"""
        # intrinsics means the camera parameters, which are used to convert the pixel coordinates to 3D coordinates
        # rs2_deproject_pixel_to_point() is used to convert the pixel coordinates to 3D coordinates
        # intrinsics is a structure that contains the camera parameters, such as focal length, principal point, and distortion coefficients
        # depth_frame.profile is used to get the profile of the depth frame, which contains the camera parameters
        depth = depth_frame.get_distance(x, y)
        if depth == 0:
            return None
        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        return rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)

    def save_to_csv_detected(self, apple_data):
        """Append apple data to CSV"""
        with open(self.csv_file_1, mode='a', newline='') as file:
            writer = csv.writer(file)
            for item in apple_data:
                writer.writerow((item[0], item[5], item[6], item[7], item[8]))
    
    def save_to_csv_picked(self, apple_data):
        """Save picked apple data to a CSV file"""
        with open(self.csv_file_2, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Index", "X (m)", "Y (m)", "Z (m)", "Distance (m)"])
            for item in apple_data:
                writer.writerow((item[0], item[5], item[6], item[7], item[8]))


    def run(self):
        """Main loop for apple detection and picking"""
        # counter
        counter = 0
        fps = 30 # camera FPS                          
        cycle_time = 15 # unit: seconds, the time interval for each cycle of detection and picking
        detection_interval = int(fps * cycle_time)  

        try:
            while True:
                # Wait for frames
                frames = self.pipeline.wait_for_frames() #wait_for_frames() is used to wait for the next set of frames from the camera
                aligned_frames = self.align.process(frames) #align.process() is used to align the depth and color frames
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                # Continue if any of the frames is missing
                if not depth_frame or not color_frame:
                    continue
                # Convert to numpy arrays
                #image is a 3D array of shape (height, width, channels) where channels = 3 for RGB image.
                # asanyarray() is used to convert the input to an array
                color_image = np.asanyarray(color_frame.get_data())

                # only when the counter is a multiple of the detection interval, perform detection
                if counter % detection_interval == 0:
                    # Perform detection
                    # conf is the confidence threshold, 0.5 means that the model is 50% confident that the detected object is an apple
                    # lower the value of conf, more the number of false positives, higher the value of conf, more the number of false negatives
                    results = self.model.predict(color_image, conf=0.5, verbose=False)

                # Store detected apples
                detected_apples = []

                for result in results:
                    #result.boxes is a list of detected objects, each object has a [bounding box, class id, and confidence score]
                    for box_idx, box in enumerate(result.boxes):
                        # Get class ID and check if it's the target class
                        cls_id = int(box.cls[0])
                        if result.names[cls_id] == self.target_class:
                            # Get bounding box coordinates
                            #xyxy[0] is the bounding box coordinates in the format [x1, y1, x2, y2], xyxy[1] is the confidence score
                            #xyxy[2] is the class id, xyxy[3] is the class name
                            x1, y1, x2, y2 = map(int, box.xyxy[0])

                            # Calculate center point
                            cx = (x1 + x2) // 2
                            cy = (y1 + y2) // 2

                            # Get 3D coordinates from depth data, coodinate Z is the distance from the camera to the object
                            coords = self.get_3d_coordinates(depth_frame, cx, cy)

                            if coords:
                                #np.linalg.norm() is used to calculate the Euclidean distance between two points in 3D space
                                #coords[0] = x, coords[1] = y, coords[2] = z
                                distance = np.linalg.norm(coords)
            
                                detected_apples.append((box_idx, x1, y1, x2, y2,coords[0], coords[1], coords[2], distance))

                # Sort and get the closest three apples
                detected_apples.sort(key=lambda x: x[4])  # Sort by distance
                picked = [] # List to store picked apples
        
                if len(detected_apples) == 0:
                    print("No apples detected")
                    continue
                else:
                    for apple in detected_apples:
                        # key is a tuple of the apple's coordinates (x, y, z) rounded to 3 decimal places
                        key = (round(apple[5], 3), round(apple[6], 3), round(apple[7], 3))
                        if key not in self.picked_apples:
                            self.move_to_apple(apple[5], apple[6], apple[7])
                            self.pick_apple()
                            self.picked_apples.add(key)  # Add the apple to the set of picked apples
                            picked.append(apple)  # Add the apple to the list of picked apples
                            print(f"Picked apple at: X: {apple[5]:.2f}m Y: {apple[6]:.2f}m Z: {apple[7]:.2f}m Distance: {apple[8]:.2f}m")

                # Save to CSV 
                self.save_to_csv_detected(detected_apples)
                self.save_to_csv_picked(picked)
                # Visualization
                for apple in detected_apples:
                    # Unpack data for each individual apple
                    index, x1, y1, x2, y2,x, y, z, distance = apple
                    print(f"Apple {index} detected at: X: {x:.2f}m Y: {y:.2f}m Z: {z:.2f}m Distance: {distance:.2f}m")

                    # Draw bounding box
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Display distance
                    label = f"{distance:.2f}m"
                    cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Draw coordinate axes
                height, width = color_image.shape[:2]
                cx_img = width // 2
                cy_img = height // 2
                axis_length = 50

                # Draw X axis (red)
                cv2.arrowedLine(color_image, (cx_img, cy_img), (cx_img + axis_length, cy_img), (0, 0, 255), 2)
                # Draw Y axis (green)
                cv2.arrowedLine(color_image, (cx_img, cy_img), (cx_img, cy_img + axis_length), (0, 255, 0), 2)
                # Draw Z axis (blue)
                cv2.arrowedLine(color_image, (cx_img, cy_img), (cx_img - axis_length, cy_img - axis_length), (255, 0, 0), 2)
                # Add labels
                cv2.putText(color_image, "X", (cx_img + axis_length + 5, cy_img), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.putText(color_image, "Y", (cx_img, cy_img + axis_length + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(color_image, "Z", (cx_img - axis_length - 20, cy_img - axis_length - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)    

                cv2.namedWindow('Apple Detection', cv2.WINDOW_NORMAL)  # Create a window
                cv2.resizeWindow('Apple Detection', 640, 480)  # Resize the window
                cv2.imshow('Apple Detection', color_image)
                # Press 'q' to break the loop
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                # Increment the counter
                counter += 1
        # Stop the pipeline and close all windows
        finally:
            self.pipeline.stop()
            if self.dobot:
                self.dobot.close()
            cv2.destroyAllWindows() 
    

if __name__ == "__main__":
    detector = AppleDetector() #create an instance of the AppleDetector class
    detector.run()