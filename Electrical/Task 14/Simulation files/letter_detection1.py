#!/usr/bin/env python3
from ultralytics import YOLO
import os
import cv2
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#appending 
current_word = ""
last_append_time = 0  
append_interval = 10

#node initial
rospy.init_node("letter_detection")
pub = rospy.Publisher("/camera" , Image , queue_size= 10)

#model loading
script_dir = os.path.dirname(__file__)
yolo_path  = os.path.join(script_dir, "alphabet.pt")
#print(yolo_path)
model = YOLO(yolo_path)

#replace cap with ros integration depending on model camera and topics
cap = cv2.VideoCapture(2) 

def check_background_colour(frame ,x1,y1,x2,y2 , inner = 0.55 , margin_frac = 0.15,
                            green_ratio_threshold = 0.45,
                            red_ratio_threshold = 0.45 ,
                            min_ring_pixles = 100):
    
    #compute box dims
    w = max(1 , x2 - x1)
    h = max(1 , y2-y1)
    margin = int(max(8, margin_frac * max(w, h)))   #margin around box for background sampling

    #outer box
    ox1 = max(0, x1 - margin)
    oy1 = max(0, y1 - margin)
    ox2 = min(frame.shape[1], x2 + margin)
    oy2 = min(frame.shape[0], y2 + margin)
    if ox2 <= ox1 or oy2 <= oy1:           #ensuring valid crop,not an empty rectangle
        return None, 0.0, 0.0 
    
    #computing inner box
    crop_w = ox2 - ox1
    crop_h = oy2 - oy1
    inner_w = int(crop_w * inner)
    inner_h = int(crop_h * inner)
    # center inner box inside the crop
    cx = ox1 + crop_w // 2
    cy = oy1 + crop_h // 2
    ix1 = max(0, cx - inner_w // 2)
    iy1 = max(0, cy - inner_h // 2)
    ix2 = min(frame.shape[1], ix1 + inner_w)
    iy2 = min(frame.shape[0], iy1 + inner_h)

    #outer region cropping
    crop = frame[oy1:oy2, ox1:ox2]
    if crop.size == 0:
        return None, 0.0, 0.0
    
    #mask creating
    mask = np.zeros((crop.shape[0], crop.shape[1]), dtype=np.uint8) 
    cv2.rectangle(mask, (0, 0), (crop.shape[1]-1, crop.shape[0]-1), 255, -1)  #fill_outer
    
    #inner rect relative to crop
    rel_ix1 = max(0, ix1 - ox1)
    rel_iy1 = max(0, iy1 - oy1)
    rel_ix2 = min(crop.shape[1], ix2 - ox1)
    rel_iy2 = min(crop.shape[0], iy2 - oy1)
    cv2.rectangle(mask, (rel_ix1, rel_iy1), (rel_ix2-1, rel_iy2-1), 0, -1) #cutting out inner

    #convert crop to HSV for masking
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

    """
    #HSV ranges -> changeable depending on model camera
    """
    green_lower = np.array([35, 60, 40])   
    green_upper = np.array([90, 255, 255])  
    red_lower1 = np.array([0, 60, 40])     
    red_upper1 = np.array([10, 255, 255])   
    red_lower2 = np.array([170, 60, 40])    
    red_upper2 = np.array([180, 255, 255]) 

    #create colour masks
    mask_green = cv2.inRange(hsv, green_lower, green_upper)
    mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
    mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    #apply ring mask so only ring pixeles count
    mask_green = cv2.bitwise_and(mask_green , mask)
    mask_red   = cv2.bitwise_and(mask_red , mask)

    #Noise reduction
    #https://www.geeksforgeeks.org/python/python-opencv-morphological-operations/ T-T didn't understand it enough tbh
    kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE , (6,6))
    mask_green = cv2.morphologyEx(mask_green , cv2.MORPH_OPEN , kernal ,iterations= 1)
    mask_green = cv2.morphologyEx(mask_green , cv2.MORPH_DILATE, kernal, iterations=1)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernal, iterations=1)
    mask_red = cv2.morphologyEx(mask_red , cv2.MORPH_DILATE, kernal, iterations=1)

    #pixles_counting
    green_px = int(cv2.countNonZero(mask_green))
    red_px   = int(cv2.countNonZero(mask_red))
    ring_px  = int(cv2.countNonZero(mask))

    #avoiding tiny rings
    if ring_px < min_ring_pixles :
        return None, 0 ,0 
    
    #compute ratios to decide background
    green_ratio = green_px / ring_px
    red_ratio  = red_px / ring_px

    if green_ratio >= green_ratio_threshold and green_px > 10 :
        return True , green_ratio , red_ratio
    elif red_ratio >= red_ratio_threshold and red_px > 20 :
        return False , green_ratio , red_ratio
    
    return None , green_ratio , red_ratio


#setting
bridge = CvBridge()

#main_loop to test
while True :
    ret, frame = cap.read()
    ##publishing
    published_frame = bridge.cv2_to_imgmsg(frame , encoding='bgr8')
    pub.publish(published_frame)

    results = model(frame)  
    for r in results:
        for box in r.boxes:                   # iterate detected boxes
            # get xyxy coordinates as integers
            x1, y1, x2, y2 = map(int, box.xyxy[0])   # left, top, right, bottom
            class_id = int(box.cls[0])         
            name = model.names[class_id] if hasattr(model, "names") and class_id in model.names else str(class_id)

            # run ring-color check
            ok_green, g_ratio, r_ratio = check_background_colour(frame, x1, y1, x2, y2,
                                                       inner=0.55,
                                                       margin_frac=0.15,
                                                       green_ratio_threshold=0.3,
                                                       red_ratio_threshold=0.3,
                                                       min_ring_pixles=200)

            # draw results based on decision
            if ok_green is True:
                # green: show detection
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)   # green box
                cv2.putText(frame, f"{name} (BG green {g_ratio:.2f})", (x1, y1 - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                #letter_appending
                current_time = time.time()
                if current_time - last_append_time >= append_interval :
                    with open("detected_letters.txt", "a") as f:
                         f.write(name)
                         f.write("\n")
                    current_word += name
                    last_append_time = current_time

            elif ok_green is False:
                # red background: ignore detection (draw faint red box if you want)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)   # red outline
                cv2.putText(frame, f"IGNORED (BG red {r_ratio:.2f})", (x1, y1 - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            else:
                # ambiguous: don't show label (yellow = ambiguous)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 1)
                cv2.putText(frame, f"ambig G:{g_ratio:.2f} R:{r_ratio:.2f}", (x1, y1 - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

    # show final frame
    cv2.imshow("Alphabet Detection (press q to quit)", frame)

    # press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
# cleanup
cap.release()
cv2.destroyAllWindows()

print (current_word)