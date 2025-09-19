'''
load object/ 
detect object features/

load pictures from dataset -scenes/
detect picture features/

match between object and picture features/
get good matches /

gettitng largest cluster -page-/


cropping

saving
'''
import cv2
import os
from cv_helpers import *
from sklearn.cluster import DBSCAN
import numpy as np



#object loading and featuers
object = cv2.imread("object.jpg" , 0)
orb = cv2.ORB_create(nfeatures = 2500)
kp_object , des_object = orb.detectAndCompute(object , None)


img = cv2.drawKeypoints(object , kp_object , None , flags= None)

#Bf initializing
bf = cv2.BFMatcher()


for image_name in os.listdir("dataset") :
    #loading images from folder
    image_path = os.path.join("dataset" , image_name)
    image = cv2.imread(image_path , 0)
    # image = cv2.blur(img ,(7,7) )

    kp_scene , des_scene = orb.detectAndCompute(image , None)

    #matching
    matches = bf.knnMatch(des_object , des_scene , k=2)
    #print (matches)

    good = [] 
    for m ,n in matches :
        if m.distance < 0.75 * n.distance :
            good.append([m])
    

    # img_match = cv2.drawMatchesKnn(object , kp_object , image , kp_scene , good , None , flags= 2)
    # print_img("match" , img_match)

    #Cluster
    dst_points = np.float32([kp_scene[m[0].trainIdx].pt for m in good])
    # print (dst_points)
    # print("--------------")

    clustering = DBSCAN(eps = 700 , min_samples = 4).fit(dst_points)
    labels = clustering.labels_

    # print (clustering)
    #print(labels)
    # 

    #clusters and their count
    unique , counts = np.unique(labels[labels >= 0] , return_counts= True)

    # print(unique ,"---" , counts)
    

    #finding largest cluster
    largest_cluster = unique[np.argmax(counts)]
    # print (largest_cluster)
    # print('----------------------------------')

    cluster_pts = dst_points[labels == largest_cluster]

    #cropping
    x, y, w, h = cv2.boundingRect(np.int32(cluster_pts))
    cropped = image[y:y+h, x:x+w]

    os.makedirs("cropped" , exist_ok= True)
    save_path = os.path.join("cropped" , f'cropped_{image_name}')

    cv2.imwrite(save_path , cropped)










cv2.waitKey(0)
cv2.destroyAllWindows()

