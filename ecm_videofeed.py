import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge


#ROS Topics
RightECM_Topic='ubc_dVRK_ECM/right/decklink/camera/image_raw/compressed'
LeftECM_Topic='ubc_dVRK_ECM/left/decklink/camera/image_raw/compressed'

SIZE_REDUCTION_PERCENTAGE=0.2 #Amount that external frame is reduced from original

class ECMVideoStreamer:
    def __init__(self):
        self.bridge=CvBridge()
        #Subscribing to ROS Topics
        rospy.Subscriber(name=RightECM_Topic,data_class=CompressedImage,callback=self.rightECMCallback,queue_size=1,buff_size=2**18)
        rospy.Subscriber(name=LeftECM_Topic,data_class=CompressedImage,callback=self.leftECMCallback,queue_size=1,buff_size=2**18)
        print("Subscribed")
        
        #Frames Init
        self.ecm_frame_right = None
        self.ecm_frame_left = None 
        self.external_frame_left = None
        self.external_frame_right = None 

        #Initializing video frames
        cv2.namedWindow('left_eye',cv2.WINDOW_NORMAL)
        cv2.namedWindow('right_eye',cv2.WINDOW_NORMAL)
        print("Named Window")
        rospy.spin()
        

    #Callbacks for video feeds:
    def leftECMCallback(self,data):
        print("Left Callback")
        self.ecm_frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        #superimposed_frame=self.superimposeView(self.ecm_frame_left,self.external_frame_left)
        #cv2.imshow('left_eye',superimposed_frame)
    def rightECMCallback(self,data):
        print("Right Callback")
        self.ecm_frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
        #superimposed_frame=self.superimposeView(self.ecm_frame_right,self.external_frame_right)
        #cv2.imshow('right_eye',superimposed_frame)

    def leftExternalCallback(self,data):
        self.external_frame_left=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')
    def rightExternalCallback(self,data):
        self.external_frame_right=self.bridge.compressed_imgmsg_to_cv2(data,'passthrough')

    #Superimpose the external view on bottom right corner of each ecm frame
    def superimposeView(seld,ecm_frame,external_frame):
        #Input: ecm frame (ecm_frame); external camera frame: external_frame
        #Output: ecm frame with external camera frame on bottom right corner (superimposed_frame)
        if external_frame is not None:
            h,w,_=external_frame.shape  #height/width of external camera frame

            h_external=h*SIZE_REDUCTION_PERCENTAGE
            w_external=w*SIZE_REDUCTION_PERCENTAGE
            h_external=int(round(h_external))
            w_external=int(round(w_external))
            external_frame_new=cv2.resize(external_frame,(w_external,h_external),interpolation=cv2.INTER_LINEAR)
            superimposed_frame=ecm_frame
            superimposed_frame[-h_external:,-w_external:,:]=external_frame_new
        else:
            superimposed_frame=ecm_frame

        return superimposed_frame   




if __name__=='__main__':
    rospy.init_node('ECM_VideoStreamer')
    rospy.Rate(10000)
    video_streamer=ECMVideoStreamer()
    cv2.destroyAllWindows()
