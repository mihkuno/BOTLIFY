import cv2
import mediapipe as mp
import numpy as np

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

# Specify the path to the image file
image_path = 'test.jpg'
# Read the image
image = cv2.imread(image_path)

# Check if the image was successfully loaded
if image is None:
    print("Error: Could not read the image.")
else:
    # Crop the image
    cropped_image = image[150:815, 800:1250]

    # Convert the BGR image to RGB (MediaPipe expects RGB format)
    rgb_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB)

    # Convert the image to a MediaPipe Image object
    mp_image = mp.Image(image_format=ImageFormat.SRGB, data=rgb_image)

    # Create an instance of ImageClassifier and classify the image
    with ImageClassifier.create_from_options(options) as classifier:
        # Perform image classification on the provided single image.
        classification_result = classifier.classify(mp_image)

        # Print classification results
        object_name = classification_result.classifications[0].categories[0].category_name
        print(object_name)

    # Display the image
    cv2.imshow('Loaded Image', cropped_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
