import cv2
import numpy as np

# Specify the path to the image file
image_path = 'test5.jpg'
# Read the image
image = cv2.imread(image_path)

def nothing(x):
    pass
    

cv2.namedWindow('Parameters')
cv2.resizeWindow('Parameters', 640, 240)
cv2.createTrackbar('Threshold1', 'Parameters', 85, 255, nothing)
cv2.createTrackbar('Threshold2', 'Parameters', 65, 255, nothing)

# Check if the image was successfully loaded
if image is None:
    print("Error: Could not read the image.")
else:
    # cropped_image = image[150:805, 800:1245]
    cropped_image = image[180:1070, 1050:1650]
    blurred_image = cv2.GaussianBlur(cropped_image, (27, 27), 0)
    grayed_image  = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)
    
    while True:
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
        print(max_area)
        
        peri = cv2.arcLength(max_cnt, True)
        approx = cv2.approxPolyDP(max_cnt, 0.02 * peri, True)
        x, y, w, h = cv2.boundingRect(approx)
        diagonal = int(np.sqrt(w**2 + h**2))
                
        cv2.line(contour_image, (x, y), (x+w, y+h), (0, 0, 255), 2)        
        cv2.rectangle(contour_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.putText(contour_image, f'Area: {int(max_area)}', (x, y+30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(contour_image, f'Diagonal: {diagonal}', (x, y+60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        # cv2.putText(contour_image, f'Points: {len(approx)}', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Display the image
        cv2.imshow('Loaded Image', contour_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
