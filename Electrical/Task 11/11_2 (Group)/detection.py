import cv2 as cv
import numpy as np
import os

# Loop through all images in "cropped" folder
for image_name in os.listdir("cropped"):

    # ==== 1. Define absolute path to image ====
    image_path = os.path.join("cropped", image_name)

    # ==== 2. Check if file exists ====
    if not os.path.exists(image_path):
        print("❌ Image not found:", image_path)
        continue  # skip instead of exit, so it processes other images

    # ==== 3. Load the image ====
    img = cv.imread(image_path)
    img = cv.blur(img , (4,4))
    

    if img is None:
        print("❌ Failed to load image. Check file format/integrity:", image_name)
        continue

    # ==== 4. Resize image for easier processing ====
    img = cv.resize(img, (1000, 1000))

    # ==== 5. Convert to grayscale ====
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # ==== 6. Apply threshold ====
    _, threshold = cv.threshold(gray, 110, 255, cv.THRESH_BINARY)

    # ==== 7. Find contours ====
    contours, _ = cv.findContours(threshold, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # ==== 8. Process each contour ====
    for i, contour in enumerate(contours):
        if i == 0:
            continue  # skip the outer contour

        # ✅ Skip very small contours (noise)
        if cv.contourArea(contour) < 700:
            continue

        # Approximate contour
        approx = cv.approxPolyDP(contour, 0.02 * cv.arcLength(contour, True), True)

        # Draw contour
        cv.drawContours(img, [contour], -1, (0, 0, 255), 3)

        # Find center of shape
        M = cv.moments(contour)
        if M['m00'] != 0:
            x = int(M['m10'] / M['m00'])
            y = int(M['m01'] / M['m00'])
        else:
            x, y = 0, 0

        # Detect shape by number of sides
        sides = len(approx)
        if sides == 3:
            label = 'Triangle'
        elif sides == 4:
            label = 'Quadrilateral'
        elif sides == 5:
            label = 'Pentagon'
        elif sides == 6:
            label = 'Hexagon'
        elif sides == 12:
            label = 'Cross'
        else:
            label = 'Circle'

        # Label the shape
        cv.putText(img, label, (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # ==== 9. Save result ====
    os.makedirs("detected" , exist_ok= True)
    save_path = os.path.join("detected" , f'detecetd_{image_name}')

    cv.imwrite(save_path , img)




cv.destroyAllWindows()
