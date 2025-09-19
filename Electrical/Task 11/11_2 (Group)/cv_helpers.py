import cv2

def print_img (name ,img) :
    cv2.imshow(name , cv2.resize(img , (1000,1000)))
    
