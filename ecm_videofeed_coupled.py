'''
Title: ECM + External Camera Video Streamer
Date: July 4th, 2024
Authors: Alexandre Banks & Randy Moore
Description: Takes in the left/right ECM from the dVRK and the left/right video feed 
from an external camera and superimposes the external camera below the ECM frame

Steps to Setup ROS for Cameras:

1. Connect green ethernet to USB-to-Ethernet adapter and connect to USB port, then:
    1.1 press 'Ethernet Connection' and under that select 'wired connection'
    1.2 $ssh pi@192.168.0.12
    1.3 $sudo ifconfig eth0 192.168.1.12 netmask 255.255.255.0 up

    Don't close terminal!

2. Connect white ethernet to wired ethernet
3. In connection settings (top right of ubuntu):
    - Under 'PCI Ethernet Connected' or 'wired ethernet':
        set it to 'Wired Ethernet' 
    - Under 'Ethernet Connected' or 'USB Ethernet' set it to 'USB Ethernet'
4. (this step should already be done) $ifconfig should show 'eno1' with inet: 192.168.0.11
  and 'enx00e05a001959' with inet: 192.168.1.13 (note the names for these two ports may change)
  If these are not the IP addresses showing, configure these IP address with:
  $sudo ifconfig <ethernet port name> <desired IP from above> netmask 255.255.255.0 up
  Note: these should match what the wired ethertnet GUI has for 'USB Ethernet' and 'Wired Ethernet' profiles

5. $roscore on dVRK-pc2
6. $roslaunch dvrk_robot jhu_daVinci_video.launch rig_name:=ubc_dVRK_ECM
    - Gets the left/right ECM feeds with topics:
    'ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
    'ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'
7. ssh pi@192.168.0.10 => Enters left pi
 7.2 export ROS_IP=192.168.0.10
 7.3 export ROS_MASTER_URI=http://192.168.0.11:11311
 7.4 roslaunch raspicam_node camerav2_1280x960.launch
    - Starts left external camera video feed with ros topic:
    'raspicam_node_left/image/compressed'

8. ssh pi@192.168.1.12 => Enters right pi
 8.2 export ROS_IP=192.168.1.12
 8.3 export ROS_MASTER_URI=http://192.168.1.13:11311
 8.4 roslaunch raspicam_node camerav2_1280x960.launch
    - Starts right external camera video feed with ros topic:
    'raspicam_node_right/image/compressed'

Note: Password for both pi's is mech464
'''


import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

#ROS Topics
RightECM_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftECM_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'
RightExternal_Topic='raspicam_node_right/image/compressed'
LeftExternal_Topic='raspicam_node_left/image/compressed'
SIZE_REDUCTION_PERCENTAGE_ECM=0.365 #Amount that external frame is reduced from original
SIZE_REDUCTION_PERCENTAGE_EXTERNAL=0.38
EXTERNAL_WIDTH=int(round(SIZE_REDUCTION_PERCENTAGE_EXTERNAL*1280))
EXTERNAL_HEIGHT=int(round(SIZE_REDUCTION_PERCENTAGE_EXTERNAL*960))

ECM_WIDTH=int(round(SIZE_REDUCTION_PERCENTAGE_ECM*1400))
ECM_HEIGHT=int(round(SIZE_REDUCTION_PERCENTAGE_ECM*986))

VIEW_WINDOW_WIDTH=1024
VIEW_WINDOW_HEIGHT=768

EXTERNAL_SEPARATION=10 #Pixel offset from center line for left (neg offset) and right (pos offset)

ZEROS_ARRAY=np.zeros((EXTERNAL_HEIGHT,EXTERNAL_WIDTH,3))
MAX_ARRAY=255*np.ones((EXTERNAL_HEIGHT,EXTERNAL_WIDTH,3))


class ECMVideoStreamer:
    def __init__(self):


        self.bridge=CvBridge()

        #TKinter Window Setup

        #Subscribing to ROS Topics
        rospy.Subscriber(name=RightECM_Topic,data_class=CompressedImage,callback=self.rightECMCallback,queue_size=1,buff_size=2**20)
        rospy.Subscriber(name=LeftECM_Topic,data_class=CompressedImage,callback=self.leftECMCallback,queue_size=1,buff_size=2**20)
      
        
        rospy.Subscriber(name=RightExternal_Topic,data_class=CompressedImage,callback=self.rightExternalCallback,queue_size=1,buff_size=2**20)
        rospy.Subscriber(name=LeftExternal_Topic,data_class=CompressedImage,callback=self.leftExternalCallback,queue_size=1,buff_size=2**20)

        #Frames Init
        self.external_frame_left = None
        self.external_frame_right = None 
        self.ecm_frame_left = None 
        self.ecm_frame_right = None


        rospy.spin()
        


    def showFramesCallback(self):
        
        if self.ecm_frame_left is not None:

            superimposed_left=self.superimposeView(self.ecm_frame_left,self.external_frame_left,'left')
            superimposed_right=self.superimposeView(self.ecm_frame_right,self.external_frame_right,'right')
            #Showing Frames
            cv2.imshow('left_eye',superimposed_left)
            cv2.imshow('right_eye',superimposed_right)
            cv2.imshow('left_eye_desktop_view',superimposed_left)
            cv2.imshow('right_eye_desktop_view',superimposed_right)
            keys=cv2.waitKey(1) & 0xFF
            if keys==ord('q'):
                print("Entered")

    #Callbacks for video feeds:
    def rightECMCallback(self,data):
        self.ecm_frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        self.showFramesCallback()

    def leftECMCallback(self,data):
        self.ecm_frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')        

    def leftExternalCallback(self,data):
        self.external_frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    def rightExternalCallback(self,data):
        self.external_frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    #Superimpose the external view on bottom right corner of each ecm frame
    def superimposeView(self,ecm_frame,external_frame,left_right):
        #Input: ecm frame (ecm_frame); external camera frame: external_frame
        #Output: ecm frame with external camera frame on bottom right below ecm (superimposed_frame)
        if external_frame is not None:
            ecm_frame_new=cv2.resize(ecm_frame,(ECM_WIDTH,ECM_HEIGHT),interpolation=cv2.INTER_LINEAR)
            external_frame_new=cv2.resize(external_frame,(EXTERNAL_WIDTH,EXTERNAL_HEIGHT),interpolation=cv2.INTER_LINEAR)
            external_frame_new=self.unsharp_mask(external_frame_new,kernel_size=(7,7),sigma=2.0,amount=3,threshold=0)
            
            superimposed_frame=np.zeros((VIEW_WINDOW_HEIGHT,VIEW_WINDOW_WIDTH,3),np.uint8)
            superimposed_frame[0:ECM_HEIGHT,int(round((VIEW_WINDOW_WIDTH-ECM_WIDTH)/2)):int(round((VIEW_WINDOW_WIDTH-ECM_WIDTH)/2))+ECM_WIDTH]=ecm_frame_new

            if left_right=='left': #We add a slight offset for the left/right to account for disparity
                superimposed_frame[-EXTERNAL_HEIGHT:,int(round((VIEW_WINDOW_WIDTH-EXTERNAL_WIDTH)/2)-EXTERNAL_SEPARATION):int(round((VIEW_WINDOW_WIDTH-EXTERNAL_WIDTH)/2)-EXTERNAL_SEPARATION)+EXTERNAL_WIDTH]=external_frame_new
            else: #right frame
                superimposed_frame[-EXTERNAL_HEIGHT:,int(round((VIEW_WINDOW_WIDTH-EXTERNAL_WIDTH)/2)+EXTERNAL_SEPARATION):int(round((VIEW_WINDOW_WIDTH-EXTERNAL_WIDTH)/2)+EXTERNAL_SEPARATION)+EXTERNAL_WIDTH]=external_frame_new

            
            #superimposed_frame[ECM_HEIGHT+1:,-EXTERNAL_WIDTH-((ECM_WIDTH-EXTERNAL_WIDTH)/2):]=external_frame

            # h_external=h*SIZE_REDUCTION_PERCENTAGE
            # w_external=w*SIZE_REDUCTION_PERCENTAGE
            # h_external=int(round(h_external))
            # w_external=int(round(w_external))
            # external_frame_new=cv2.resize(external_frame,(w_external,h_external),interpolation=cv2.INTER_LINEAR)
            # superimposed_frame=ecm_frame
            # superimposed_frame[-h_external:,-w_external:,:]=external_frame_new
        else:
            ecm_frame_new=cv2.resize(ecm_frame,(ECM_WIDTH,ECM_HEIGHT),interpolation=cv2.INTER_LINEAR)
            superimposed_frame=np.zeros((VIEW_WINDOW_HEIGHT,VIEW_WINDOW_WIDTH,3),np.uint8)
            superimposed_frame[0:ECM_HEIGHT,int(round((VIEW_WINDOW_WIDTH-ECM_WIDTH)/2)):int(round((VIEW_WINDOW_WIDTH-ECM_WIDTH)/2))+ECM_WIDTH]=ecm_frame_new

        return superimposed_frame
       
        # if external_frame is not None:
        #     h,w,_=external_frame.shape  #height/width of external camera frame

        #     h_external=h*SIZE_REDUCTION_PERCENTAGE
        #     w_external=w*SIZE_REDUCTION_PERCENTAGE
        #     h_external=int(round(h_external))
        #     w_external=int(round(w_external))
        #     external_frame_new=cv2.resize(external_frame,(w_external,h_external),interpolation=cv2.INTER_LINEAR)
        #     superimposed_frame=ecm_frame
        #     superimposed_frame[-h_external:,-w_external:,:]=external_frame_new
        # else:
        #     superimposed_frame=ecm_frame

        # return superimposed_frame   
    
    def unsharp_mask(self,image,kernel_size=(5,5),sigma=1.0,amount=1.0,threshold=0):
        blurred=cv2.GaussianBlur(image,kernel_size,sigma)

        sharpened=float(amount+1)*image-float(amount)*blurred
        sharpened=np.maximum(sharpened,ZEROS_ARRAY)
        sharpened=np.minimum(sharpened,MAX_ARRAY)
        #sharpened=sharpened.round().astype(np.uint8)
        if threshold>0:
            low_contrast_mask=np.absolute(image-blurred)<threshold
            np.copyto(sharpened,image,where=low_contrast_mask)
        
        return sharpened




if __name__=='__main__':
    rospy.init_node('ECM_VideoStreamer',anonymous=True)
    #rospy.Rate(10000)
    video_streamer=ECMVideoStreamer()
    
