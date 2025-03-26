import cv2 
import rospy
from sensor_msgs.msg import Image
import cv_bridge
import os
import sys
import time


class image_capture():
    def __init__(self):
        rospy.init_node('checker_capture',anonymous=True)

        ## folder path 
        self.images_folder = fr'pkg_1/checker_images'

        if not os.path.exists(self.images_folder):
            os.mkdir(self.images_folder)

        ## subscribing to the image topic 
        rospy.Subscriber('/camera/color/image_raw',Image,self.camera_callback,queue_size=15)

        ## creating a cv_bridge class
        self.cv_bridge_obj = cv_bridge.CvBridge()

        ## initializing image count 
        self.image_count = 1

        ## setting the capture frequency
        self.time_interval = 5

        ## displaying the image 
        self.cv_image = None
        self.display()

        
    def camera_callback(self,data:Image):
        
        try:
            self.cv_image = self.cv_bridge_obj.imgmsg_to_cv2(data,desired_encoding='bgr8')
            # print('image is converted \n')

        except cv_bridge.CvBridgeError as e:
            print(e)   
            print('im here in exception')  
                   

    def display(self):
        flag = 'p'
        while True:
            if self.cv_image is not None:
                cv2.imshow('capture_image',self.cv_image)
                k = cv2.waitKey(1)

                ## updating the current time 
                current_time = time.time()
                
                ## key handlings begin here 

                if k & 0xff == ord('c'):
                    self.capture(self.cv_image)                   
                    print('image captured')
                
                if k & 0xff == ord('r') or flag =='r':
                    
                    if flag !='r':
                        print('recording the data ')
                        flag = 'r'
                        last_time = time.time()

                    time_diff  =  current_time - last_time

                    ## checking if the time interval has reached to capture 
                    if time_diff>=self.time_interval:
                        self.capture(self.cv_image)
                        
                        ## updating the last time when image captured 
                        last_time = time.time()


                if k & 0xff == ord('p'):
                    flag = 'p'
                    print('flag changed to pause state ')

                if k & 0xff == ord('q'):
                    cv2.destroyAllWindows()
                    print('exited the display the display')
                    break;

   
    def capture(self,image):
        image_path = os.path.join(self.images_folder,f'img_{self.image_count}.png')
        image_path = os.path.abspath(image_path)
        cv2.imwrite(image_path,image)
        print(f'image no. {self.image_count} captured')
        self.image_count +=1
        




if __name__ == '__main__':    

    camera_obj = image_capture()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')
    



    
        