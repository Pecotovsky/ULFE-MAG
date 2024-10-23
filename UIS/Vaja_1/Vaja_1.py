import cv2
import numpy as np
from mtcnn import MTCNN
from mtcnn.utils.images import load_image



# Definition of Haar cascade for detection of frontal faces and profile faces
#face_cascade_ff = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
face_cascade_pf = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_profileface.xml")

# Create a MTCNN detector instance
detector = MTCNN(device="GPU:0")


# Function takes camera frame, converts it to grayscale and applies filters
def frame_preproces(frame):

    # Convert RGB to Grayscale
    frame_GRAY = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Improve contrast of grayscale image - EH
    frame_EH = cv2.equalizeHist(frame_GRAY)

    # Apply low pass filter to preprocessed image - GaussianBlur
    frame_GB = cv2.GaussianBlur(frame_EH, (7,7), 0)

    # Apply low pass filter to preprocessed image - BilateralFilter
    frame_BF = cv2.bilateralFilter(frame_EH, 9, 75, 75)

    return frame_GB, frame_BF


# Function takes preprocessed frame and finds edges using Sobel and Canny method
def find_edges(frame):

    # Find edges - Sobel
    frame_sobel_x = cv2.Sobel(frame, cv2.CV_64F, 1, 0, ksize = 5)
    frame_sobel_y = cv2.Sobel(frame, cv2.CV_64F, 0, 1, ksize = 5)
    frame_abs_x = cv2.convertScaleAbs(frame_sobel_x)
    frame_abs_y = cv2.convertScaleAbs(frame_sobel_y)
    frame_sobel = cv2.addWeighted(frame_abs_x, 0.5, frame_abs_y, 0.5, 0)

    # Find edges - Canny
    frame_canny = cv2.Canny(frame, 50, 150)

    return frame_sobel, frame_canny


# Function takes frame and thresholds it
def threshold(frame):

    _, frame_thresh = cv2.threshold(frame, 122, 255, cv2.THRESH_BINARY)
    _, frame_thresh_inv = cv2.threshold(frame, 122, 255, cv2.THRESH_BINARY_INV)

    return frame_thresh, frame_thresh_inv


# Function to resize frames
def frame_resize(frame, scale=0.3):

    return cv2.resize(frame, (0, 0), fx=scale, fy=scale)



# Lab work part 1
def Lab_one(frame):

    # Preproces frame
    frame_GB, frame_BF = frame_preproces(frame_resize(frame))

    # Find edges of preprocesed frame
    frame_GB_sobel, frame_GB_canny = find_edges(frame_GB)
    frame_BF_sobel, frame_BF_canny = find_edges(frame_BF)

    # Threshold frame
    frame_GB_sobel_thresh, frame_GB_sobel_thresh_inv = threshold(frame_GB_sobel)
    frame_GB_canny_thresh, frame_GB_canny_thresh_inv = threshold(frame_GB_canny)
    frame_BF_sobel_thresh, frame_BF_sobel_thresh_inv = threshold(frame_BF_sobel)
    frame_BF_canny_thresh, frame_BF_canny_thresh_inv = threshold(frame_BF_canny)

    # Combine different frames for imshow
    frame_combined_h_1 = np.hstack((frame_GB, frame_GB_sobel, frame_GB_canny, frame_GB_sobel_thresh, frame_GB_sobel_thresh_inv, frame_GB_canny_thresh, frame_GB_canny_thresh_inv))
    frame_combined_h_2 = np.hstack((frame_BF, frame_BF_sobel, frame_BF_canny, frame_BF_sobel_thresh, frame_BF_sobel_thresh_inv, frame_BF_canny_thresh, frame_BF_canny_thresh_inv))
    frame_combined = np.vstack((frame_combined_h_1, frame_combined_h_2))

    return frame_combined


# Lab work part 2
def Lab_two(frame):

    frame_GB_ff = frame
    #frame_GB_ff, _ = frame_preproces(frame)
    frame_GB_pf = frame_GB_ff

    # face_ff = face_cascade_ff.detectMultiScale(frame, scaleFactor = 1.1, minNeighbors = 5, minSize = (40, 40))
    # for (x, y, w, h) in face_ff:
    #     cv2.rectangle(frame_GB_ff, (x, y), (x + w, y + h), (128, 0, 128), 2)
        
    face_pf = face_cascade_pf.detectMultiScale(frame, scaleFactor = 1.1, minNeighbors = 5, minSize = (40, 40))
    for (x, y, w, h) in face_pf:
        cv2.rectangle(frame_GB_pf, (x, y), (x + w, y + h), (128, 0, 128), 2)

    #frame_combined = np.hstack((frame_GB_ff, frame_GB_pf))
    
    # return frame_combined
    return frame_GB_pf


def Lab_three(frame):
    # Detect faces in the image
    result = detector.detect_faces(frame)

    # Check if any faces are detected
    if len(result) > 0:
        # Get the bounding box of the first detected face
        bbox = result[0]["box"]
        print(bbox)
        # Draw a rectangle around the face
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), (128, 0, 128), 2)
    else:
        print("No face detected")

    return frame


# Open the camera at the ID 0 
capture = cv2.VideoCapture(0)

# Check if camera is activated
if not (capture.isOpened()):
    print("Error: Could not access the camera")

# Set resolution of video frames
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("Press any key to exit...")



# Open oscar.jpg image
image_oscar = cv2.imread('oscar.jpg')



# Main loop
while(True):

    # Read current frame
    ret, frame = capture.read() # ret is a bool equal to True, when frame is read correctly
    
    # Toggle between camera and oscar.jpg
    #frame = image_oscar


    # Lab exercise part 1
    #frame_izhod = Lab_one(frame)

    # Lab exercise part 2
    frame_izhod = Lab_two(frame)

    # Lab exercise part 3
    #frame_izhod = Lab_three(frame)

    # Display current camera frame - RGB
    cv2.imshow("Camera is live :)", frame_izhod)

    # Exit the loop when user presses any key
    if cv2.waitKey(1) != -1:
        break



# Release the camera and close all active windows before exiting the program
capture.release()
cv2.destroyAllWindows()