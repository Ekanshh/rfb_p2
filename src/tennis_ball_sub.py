#!/usr/bin/env python 

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

#all_opencv_functions_in_one
def call_all_function(frame):
    get_masked_frame= detect_yellow(frame)
    get_contours= getContour(get_masked_frame)
    process_contour= processContours(get_masked_frame,get_contours,frame)

#function_for_mask_color_of_ball
def detect_yellow(frame):  
    hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)#converting_BGR2HSV
    cv2.imshow("HSV Image",hsv) #showing_HSV_image
    yellow_lower=(30,50,100) #upper_and_lower_bound_for_yellow_color(tennis_ball)
    yellow_upper=(60,255,255)
    mask =cv2.inRange(hsv, yellow_lower, yellow_upper) #selecting_yellow_color_of_tennis_ball
    mask_show=cv2.imshow("Mask Image",mask) #showing_masked_image
    return mask

#defining_contours_for_ball
def getContour(get_masked_frame):
    contours = cv2.findContours(get_masked_frame, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[1] #I_have_used_[1]_beacuse_the_output_is_three_arrays_and_I_have_to_use_1
    return contours

#processing_contours
def processContours(get_masked_frame,get_contours,frame):
    black_background_frame= np.zeros([get_masked_frame.shape[0], get_masked_frame.shape[1],3],'uint8')
    for c in get_contours: #loop_for_defining_contour_parameters_at_any_instance
        area = cv2.contourArea(c) 
        perimeter = cv2.arcLength(c, True)
        ((x,y), radius)= cv2.minEnclosingCircle(c) #defining_min_circle_at_any_instanct
        if(area>500):  #draw_contours_if_Area>500
            cv2.drawContours(frame,[c],-1,(150,250,150),1) #draw_contours_rgb_image
            cv2.drawContours(black_background_frame, [c], -1,(150,250,150),1) #draw_contour_black_image
            cx,cy= get_Contour_center(c) #getting_contour_center 
            cv2.circle(frame, (cx,cy),(int)(radius), (255,255,255),1) #defining_a_circle_around_drawn_contour
            cv2.circle(frame, (cx,cy),5, (255,255,255),-1) #marking_centriod_using_small_radius_and_-1
            cv2.circle(black_background_frame, (cx,cy),(int)(radius),(255,255,255),1) #defining_a_circle_around_drawn_contour
            cv2.circle(black_background_frame, (cx,cy),5, (255,255,255),-1) #marking_centriod_using_small_radius_and_-1
    RGB_processed_frame=cv2.imshow("RGB Background Contours", frame) #RGB_frame_with_processcontours
    BB_processed_frame=cv2.imshow("Black Background Contours", black_background_frame) #bc_frame_with_processcontours
    
#To_find_center_of_every_contour
def get_Contour_center(get_contours): 
    M=cv2.moments(get_contours)
    cx=-1
    cy=-1
    if(M['m00']!=0): 
        cx= int (M['m10']/M['m00'])
        cy= int (M['m01']/M['m00'])
    return cx, cy

#callback_function_from_subscriber
def ImageCallback(ros_image):
    print ("got an image") #to_check_control_flow
    global bridge
    bridge= CvBridge() #cvbrigde_object_for_ros_to_cv2_imageconversion
    try: 
        frame= bridge.imgmsg_to_cv2(ros_image,"bgr8") #converting_captured_ros_image_to_cv2_format
        call_all_function(frame) #calling_opencv_function_defined_in_callallfunctions_and_passing_frame
        cv2.waitKey(100)

    except CvBridgeError as e:
        print(e)


def main():

    #create_a_subscriber_and_intialise_node
    sub= rospy.Subscriber("tennis_ball_image",Image,ImageCallback)
    rospy.init_node('tennis_ball_detection', anonymous=True)
    rospy.spin() #loop

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException():
        rospy.loginfo("master  node was terminated")