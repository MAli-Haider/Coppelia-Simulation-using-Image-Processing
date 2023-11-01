import sim
import time
import sys
import keyboard
import dlib
import cv2
import math

# Load the pre-trained face detection and landmark detection models
detector = dlib.get_frontal_face_detector()  # Create a face detector object
predictor = dlib.shape_predictor("D://Projects//Coppelia Adam//model//model2.dat")  # Load facial landmark predictor

# Open the webcam
cap = cv2.VideoCapture(0)

# Define tilt angle threshold and tilt sensitivity
angle_threshold = 10  # Angle in degrees to determine tilt
sensitivity = 1.5     # Adjust this value to make tilt detection more or less sensitive

print('System Started')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print('Connection Successful')
else:
    sys.exit('Failed to Connect')

time.sleep(1)

error_code, left_motor_handle = sim.simxGetObjectHandle(clientID, '/RobotnikSummitXL/front_left_wheel', sim.simx_opmode_oneshot_wait)
error_code, right_motor_handle = sim.simxGetObjectHandle(clientID, '/RobotnikSummitXL/front_right_wheel', sim.simx_opmode_oneshot_wait)
error_code, left2_motor_handle = sim.simxGetObjectHandle(clientID, '/RobotnikSummitXL/back_right_wheel', sim.simx_opmode_oneshot_wait)
error_code, right2_motor_handle = sim.simxGetObjectHandle(clientID, '/RobotnikSummitXL/back_left_wheel', sim.simx_opmode_oneshot_wait)

# Initial motor speeds
left_motor_speed = 0.0
right_motor_speed = 0.0
left2_motor_speed = 0.0
right2_motor_speed = 0.0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame from webcam.")
        break
    
    # Convert the frame to grayscale for face detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect faces in the grayscale frame
    faces = detector(gray)
    
    for face in faces:
        # Get the coordinates of the face bounding box
        x, y, w, h = face.left(), face.top(), face.width(), face.height()
        
        # Draw a bounding box around the face
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Detect facial landmarks
        landmarks = predictor(gray, face)
        
        # Get coordinates of left and right eye landmarks
        left_eye = (landmarks.part(36).x, landmarks.part(36).y)
        right_eye = (landmarks.part(45).x, landmarks.part(45).y)
        
        # Calculate the angle using the arctangent function
        angle = math.atan2(right_eye[1] - left_eye[1], right_eye[0] - left_eye[0])
        angle_degrees = angle * 180 / math.pi
        
        # Adjust sensitivity for tilt detection
        sensitivity_angle = angle_threshold * sensitivity
        
        # Determine if the face is tilted left, right, or straight
        if angle_degrees > sensitivity_angle:
            tilt_direction = "Right"
            left_motor_speed = 0.5
            right_motor_speed = -0.5
            left2_motor_speed = 0.5
            right2_motor_speed = -0.5
        
        elif angle_degrees < -sensitivity_angle:
            tilt_direction = "Left"
            left_motor_speed = -0.5
            right_motor_speed = 0.5
            left2_motor_speed = -0.5
            right2_motor_speed = 0.5
            
        else:
            tilt_direction = "Straight"
            left_motor_speed = 0.5
            right_motor_speed = 0.5
            left2_motor_speed = 0.5
            right2_motor_speed = 0.5
    
        # Set the motor speeds
        error_code = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, left_motor_speed, sim.simx_opmode_oneshot)
        error_code = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, right_motor_speed, sim.simx_opmode_oneshot)
        error_code = sim.simxSetJointTargetVelocity(clientID, left2_motor_handle, left2_motor_speed, sim.simx_opmode_oneshot)
        error_code = sim.simxSetJointTargetVelocity(clientID, right2_motor_handle, right2_motor_speed, sim.simx_opmode_oneshot)
        
        # Draw the angle information on the frame
        cv2.putText(frame, f"Tilt: {tilt_direction} ({angle_degrees:.2f} degrees)", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw a line beneath the eyes to indicate tilt direction
        line_color = (0, 0, 255) if tilt_direction == "Left" else (255, 0, 0) if tilt_direction == "Right" else (0, 255, 0)
        cv2.line(frame, left_eye, right_eye, line_color, 2)
        cv2.imshow("Face Tilt Detection", frame)
        time.sleep(0.2)

    
    
    # Display the frame with detected tilt information
    
    
    time.sleep(0.1)  # Adjust the sleep duration as needed