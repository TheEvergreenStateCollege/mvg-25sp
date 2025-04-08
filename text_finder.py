# This script is used to find text in images using the Tesseract OCR engine.
# It captures images from a webcam and processes them to extract text.
# It requires the OpenCV and pytesseract libraries.

import cv2
from PIL import Image
import pytesseract
import os
#from ocrmac import OCRMac


def save_frame_camera(device_num, dir_path, basename, ext="jpg", delay = 1, window_name='frame'):
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    os.makedirs(dir_path, exist_ok=True)  # Ensure the directory exists
    base_path = os.path.join(dir_path, basename)

    n = 0
    while True:
        ret, frame = cap.read()
        cv2.imshow(window_name, frame)
        key = cv2.waitKey(delay) & 0xFF
        if key == ord('c'):
            filename = ('{}_{}_{}'.format(base_path, n, ext))
            print(filename)
            cv2.imwrite(f"{base_path}_{n}.{ext}", frame)
            n += 1
        elif key == ord('q'):
            break

    cv2.destroyAllWindows(window_name)

def find_text_in_image(image_path):
    # Load the image using OpenCV
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error could not read the image at {image_path}")
        return
    config = '--oem 3 --psm 6'  # Example configuration: OCR Engine Mode 3 and Page Segmentation Mode 6
    # Convert the image to RGB format
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    annotations = pytesseract.image_to_string(rgb_image, config=config)
    print(annotations)
    return annotations

save_frame_camera(1, "UDGR/images", "frame", ext="jpg", delay=1, window_name='frame')
find_text_in_image("UDGR/images/frame_0.jpg")
