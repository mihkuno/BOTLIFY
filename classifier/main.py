
# 240 below small
# 240 above is medium 25 minutes
# 280 above is large 45 minutes


import cv2
import numpy as np
import mediapipe as mp


model_path = "exported_model/model.tflite"

BaseOptions = mp.tasks.BaseOptions
ImageClassifier = mp.tasks.vision.ImageClassifier
ImageClassifierOptions = mp.tasks.vision.ImageClassifierOptions
VisionRunningMode = mp.tasks.vision.RunningMode
ImageFormat = mp.ImageFormat

options = ImageClassifierOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    max_results=5,
    running_mode=VisionRunningMode.IMAGE
)



cap = cv2.VideoCapture(1)

def nothing(x):
    pass
    
cv2.namedWindow('Parameters')
cv2.resizeWindow('Parameters', 640, 240)
cv2.createTrackbar('Threshold1', 'Parameters', 0, 255, nothing)
cv2.createTrackbar('Threshold2', 'Parameters', 30, 255, nothing)
# cv2.createTrackbar('Threshold3', 'Parameters', 150, 255, nothing)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()


# Create an instance of ImageClassifier and classify the image
with ImageClassifier.create_from_options(options) as classifier:
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Failed to capture image.")
            break
        

        cropped_image = frame[60:355, 260:440]
        
        # Convert the BGR image to RGB (MediaPipe expects RGB format)
        rgb_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB)

        # Convert the image to a MediaPipe Image object
        mp_image = mp.Image(image_format=ImageFormat.SRGB, data=rgb_image)

        # Perform image classification on the provided single image.
        classification_result = classifier.classify(mp_image)

        # Print classification results
        object_name = classification_result.classifications[0].categories[0].category_name
        print(object_name)

        
        blurred_image = cv2.GaussianBlur(cropped_image, (13, 13), 0)
        grayed_image  = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)

        threshold1  = cv2.getTrackbarPos('Threshold1', 'Parameters')
        threshold2  = cv2.getTrackbarPos('Threshold2', 'Parameters')
        canny_image = cv2.Canny(grayed_image, threshold1, threshold2)

        # dilation function
        kernel = np.ones((5, 5), np.uint8)
        dilate_image = cv2.dilate(canny_image, kernel, iterations=1)

        contours, hierarchy = cv2.findContours(dilate_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        max_area = 0
        max_cnt = 0
        for index, cnt in enumerate(contours):
            
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                max_cnt = cnt
        
        contour_image = cv2.drawContours(cropped_image, max_cnt, -1, (0, 255, 0), 3)
        
        
        peri = cv2.arcLength(max_cnt, True)
        approx = cv2.approxPolyDP(max_cnt, 0.02 * peri, True)
        x, y, w, h = cv2.boundingRect(approx)
        diagonal = int(np.sqrt(w**2 + h**2))
        print(diagonal)
        
                
        cv2.line(contour_image, (x, y), (x+w, y+h), (0, 0, 255), 2)        
        cv2.rectangle(contour_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.putText(contour_image, f'Area: {int(max_area)}', (x, y+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(contour_image, f'Diagonal: {diagonal}', (x, y+60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Display the image
        cv2.imshow('Loaded Image', contour_image)  
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
