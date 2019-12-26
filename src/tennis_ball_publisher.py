#!/usr/bin/env python 


import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg

def rescale_frame(*frames):

    for frame in frames:
     width = int(frame.shape[1] * 40/ 100)
     height = int(frame.shape[0] * 40/ 100)
     dim = (width, height)
     return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

def main():

    #initialising_node_and_publisher
    rospy.init_node('tennis_ball_detection', anonymous=True) 
    pub= rospy.Publisher("tennis_ball_image", Image)

    #create_CvBridge()_object_for_cv2_to_ros_img_conversion
    bridge= CvBridge()

    #rospkg_object_for_defining_path_for_video_file
    rospack = rospkg.RosPack() #object_created
    video_path = rospack.get_path('project_10')+'/video/' #first_directory_of_project_then_video_folder
    
    #Reading_video_stream_using_cv2_library
    cap = cv2.VideoCapture(video_path + 'tennis-ball-video.mp4') #defined_path_&_filename
    if (cap.isOpened()== False): #condition_if_video_file_not_opened
        print("Error opening video stream or file")

    while not rospy.is_shutdown():
        if(cap.isOpened()==True):
            ret,get_frame= cap.read() #reading_video_stream_frame_by_frame
            get_frame= rescale_frame(get_frame) #rescaling_original_frame_windows_to_fit_on_screen
            print(ret) #return_true_if_frame_is_received
            pub.publish(bridge.cv2_to_imgmsg(get_frame,"bgr8")) #publishing_frame_after_cv2_to_ros_conversion
            cv2.imshow("frame",get_frame) #to_show_the_video 
            if cv2.waitKey(100) & 0xFF== ord('q'):
              break
    cap.release()    
cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException():
        rospy.loginfo("master  node was terminated")