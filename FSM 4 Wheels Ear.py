import sim
import time
import sys
import dlib
import cv2
import math

# Load the pre-trained face detection and landmark detection models
detector = dlib.get_frontal_face_detector()  # Create a face detector object
predictor = dlib.shape_predictor("model//newmodel.dat")  # Load facial landmark predictor

# Open the webcam
cap = cv2.VideoCapture(0)

# Define tilt angle threshold, tilt sensitivity, and eye position threshold for looking down
angle_threshold = 10  # Angle in degrees to determine tilt
sensitivity = 1.5     # Adjust this value to make tilt detection more or less sensitive
eye_position_threshold = 10  # Adjust this value for looking down sensitivity

print('System Started')

# Initialize the simulation client
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print('Connection Successful')
else:
    sys.exit('Failed to Connect')

time.sleep(1)

# Get object handles for the wheels
error_code, left_motor_handle = sim.simxGetObjectHandle(clientID, '/RobotnikSummitXL/front_left_wheel', sim.simx_opmode_oneshot_wait)
error_code, right_motor_handle = sim.simxGetObjectHandle(clientID, '/RobotnikSummitXL/front_right_wheel', sim.simx_opmode_oneshot_wait)
error_code, left2_motor_handle = sim.simxGetObjectHandle(clientID, '/RobotnikSummitXL/back_right_wheel', sim.simx_opmode_oneshot_wait)
error_code, right2_motor_handle = sim.simxGetObjectHandle(clientID, '/RobotnikSummitXL/back_left_wheel', sim.simx_opmode_oneshot_wait)

# Check for errors in obtaining object handles
if error_code != 0:
    sys.exit(f'Failed to get object handles with error code: {error_code}')

# Initial motor speeds
left_motor_speed = 0.0
right_motor_speed = 0.0
left2_motor_speed = 0.0
right2_motor_speed = 0.0

# Define states
states = {
    "Left": "Turning Left",
    "Right": "Turning Right",
    "Straight": "Going Straight",
    "Down": "Stopped"
}

current_state = None  # Initialize the current state to None

while True:
    # Capture a frame from the webcam
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame from webcam.")
        break

    # Convert the frame to grayscale for face detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale frame
    faces = detector(gray)

    for face in faces:
        x, y, w, h = face.left(), face.top(), face.width(), face.height()
        
        # Draw a bounding box around the face
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Detect facial landmarks
        landmarks = predictor(gray, face)
        
        # Get coordinates of left and right eye landmarks
        left_ear = (landmarks.part(0).x, landmarks.part(0).y)
        right_ear = (landmarks.part(16).x, landmarks.part(16).y)
        
        # Get coordinates of chin landmark
        chin = (landmarks.part(8).x, landmarks.part(8).y)

        ear1 = (landmarks.part(36).x, landmarks.part(36).y)
        ear2 = (landmarks.part(45).x, landmarks.part(45).y)
        
        # Calculate the angle using the arctangent function
        angle = math.atan2(ear2[1] - ear1[1], ear2[0] - ear1[0])
        angle_degrees = angle * 180 / math.pi
        ear_y = (ear1[1] + ear2[1]) / 2
        
        # Adjust sensitivity for tilt detection
        sensitivity_angle = angle_threshold * sensitivity
        # ... (your existing code)

        # Determine the new state based on the tilt direction
        if angle_degrees > sensitivity_angle:
            new_state = "Right"
        elif angle_degrees < -sensitivity_angle:
            new_state = "Left"
        elif ear_y > h + eye_position_threshold:
            new_state = "Down"
        else:
            new_state = "Straight"

        # If the state has changed, update the current state
        if new_state != current_state:
            current_state = new_state
            print(f"Transitioning to state: {states[current_state]}")

    # Apply actions based on the current state
    if current_state == "Right":
        tilt_direction = "Right"
        left_motor_speed = 0.5
        right_motor_speed = -0.5
        left2_motor_speed = 0.5
        right2_motor_speed = -0.5
    elif current_state == "Left":
        tilt_direction = "Left"
        left_motor_speed = -0.5
        right_motor_speed = 0.5
        left2_motor_speed = -0.5
        right2_motor_speed = 0.5
    elif current_state == "Down":
        tilt_direction = "Down"
        left_motor_speed = 0.0
        right_motor_speed = 0.0
        left2_motor_speed = 0.0
        right2_motor_speed = 0.0
    elif current_state == "Straight":
        tilt_direction = "Straight"
        left_motor_speed = 0.5
        right_motor_speed = 0.5
        left2_motor_speed = 0.5
        right2_motor_speed = 0.5

    # Update the wheel velocities in the simulation
    error_code = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, left_motor_speed, sim.simx_opmode_oneshot)
    error_code = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, right_motor_speed, sim.simx_opmode_oneshot)
    error_code = sim.simxSetJointTargetVelocity(clientID, left2_motor_handle, left2_motor_speed, sim.simx_opmode_oneshot)
    error_code = sim.simxSetJointTargetVelocity(clientID, right2_motor_handle, right2_motor_speed, sim.simx_opmode_oneshot)

    cv2.putText(frame, f"Tilt: {tilt_direction} ({angle_degrees:.2f} degrees)", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw a line beneath the eyes to indicate tilt direction
    circle_color = (0, 255, 255)  # Yellow color
    cv2.circle(frame, left_ear, 5, circle_color, -1)
    cv2.circle(frame, right_ear, 5, circle_color, -1)
    cv2.circle(frame, chin, 5, circle_color, -1)
    
    # Draw a red triangle between the circles (ears and chin)
    triangle_color = (0, 0, 255)  # Red color
    cv2.line(frame, left_ear, right_ear, triangle_color, 2)
    cv2.line(frame, right_ear, chin, triangle_color, 2)
    cv2.line(frame, chin, left_ear, triangle_color, 2)
     # Display the frame with detected tilt information
    cv2.imshow("Face Tilt Detection", frame)
    
    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all windows
cap.release()
cv2.destroyAllWindows()
