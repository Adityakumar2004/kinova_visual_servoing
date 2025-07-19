import rospy
import tf2_ros
from sensor_msgs.msg import Image
from std_msgs.msg import String
import kortex_driver.msg as msg

import cv_bridge
import cv2
import numpy as np

import threading

import json
from typing import Union
import copy

class camera_interaction():
    def __init__(self,cam_config:str):
        rospy.init_node('camera_interaction',anonymous=True)
        ## subscribing the camera topic
        self.img_subscriber = rospy.Subscriber('/camera/color/image_raw',Image,self.camera_callback,queue_size=15) 
        self.bridge = cv_bridge.CvBridge()
        self.frame = None
        self.current_points = []
        self.desired_points = []

        ## loading the camera parameters
        self.cam_mtx,self.cam_dist,_,_ = load_calibration_json(cam_config)
        self.sx = 1
        self.sy = 1 ## these are also uncertain need to think about it 

        ## initializing tf_buffer for getting transformation matrices
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        ## subscribing to the keyboard node 
        self.keyboard_sub = rospy.Subscriber('/keyboard_commands', String, queue_size=10, callback=self.keyboard_callback)

        ## visual servoing stuff
        self.vs_flag = 'p' ## flag to pause (p) or continue (c)
        self.vs_thread = None

        ## velocity publisher                 
        self.pub_arm = rospy.Publisher(
            name = '/my_gen3/in/cartesian_velocity', 
            data_class= msg.TwistCommand, 
            queue_size= 10
            )

        self.twist = msg.TwistCommand()
        self.twist.duration = 0
        self.twist.reference_frame = 2
        
        self.recoding_state = "t"
        self.display_img()
     
    def camera_callback(self,data:Image):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
            # print('converted')    
            # cv2.imshow('frame',self.frame)   
            # cv2.waitKey(1)     

            # print('image is converted\n',type(frame))
                
        except cv_bridge.CvBridgeError as e:
            print(e)   
            print('im here in exception') 
        
    def display_img(self):
        flag = 's'
        while True:
            if self.frame is not None:
                cv2.imshow('image',self.frame)
                key = cv2.waitKey(1)

                if key & 0xff == ord('f'):
                    self.feature_selection()
                
                if key & 0xff == ord('t') or flag == 't':

                    if flag != 't':
                        self.old_gray = cv2.cvtColor(self.frame,cv2.COLOR_BGR2GRAY)  
                        flag = 't'

                    new_pts, current_gray = self.tracking(old_pts = self.current_points,
                                                          old_gray= self.old_gray, 
                                                          new_frame= self.frame)

                    self.old_gray = current_gray
                    self.current_points = new_pts


                if key & 0xff == ord('q'):
                    print('exiting')
                    cv2.destroyAllWindows()
                    break;
    
                if key & 0xff == ord('i'):
                    self.interaction_matrix(r = self.current_points[:,0], c = self.current_points[0:1])

                if len(self.current_points) != 0:
                    frame2 = self.frame.copy()
                    # print(type(self.current_points))
                    draw_points(frame2,self.current_points,(0, 255, 0))
                    draw_points(frame2,self.desired_points,(255, 0, 0))
                    cv2.imshow('features_frame2',frame2)
                    # self.frame2 = frame2        

                    if self.recoding_state == "c":
                        self.out.write(frame2)     
                        print("recording")                        

                    # print(self.current_points, ' \n these are the current pts \n')
                    # print(self.desired_points, '\n these are the desired pts \n ')
  
    def tracking(self,old_pts,old_gray,new_frame): ## old gray newrgb might save some computation
        
        if type(old_pts) == list:
            old_pts = np.array(old_pts,dtype=np.float32) # required for tracking

        current_gray = cv2.cvtColor(new_frame,cv2.COLOR_BGR2GRAY)
        # cv2.imshow('gray',current_gray)
        # cv2.waitKey(1)
        # frame = new_frame.copy()
        new_pts,sttus,err = cv2.calcOpticalFlowPyrLK(old_gray, current_gray,  old_pts, 
                            None, maxLevel=1,
                            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                                                                15, 0.08))
        # print(new_pts,'\n',old_pts)
        # print('status \n',sttus)
        return(new_pts, current_gray)

    def feature_selection(self):
        self.current_points = []
        cv2.namedWindow('features_frame')
        while 1:
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            img = self.frame.copy()

            ## drawing the points 
            if len(self.current_points) != 0 :               
                img = draw_points(img,self.current_points,(0,0,255))

            cv2.imshow('features_frame', img)
            key = cv2.waitKey(1)

            if key & 0xff == ord('c'):
                print('clearing the feature points')
                self.current_points.clear()
            
            if key & 0xff == ord('m'):
                cv2.setMouseCallback('features_frame',self.mouse_callback)
            
            if key & 0xff == ord('q'):
                self.desired_points = np.array(self.current_points);
                cv2.destroyWindow('features_frame')
                break;

            
            print(self.current_points, ' these are the current pts \n')
    
    def mouse_callback(self,event,x,y,flags, param):

        # frame,state = param
        if event == cv2.EVENT_LBUTTONDOWN:
            self.current_points.append([x,y])
            print(x,y)  

            # cv2.circle(frame,(x,y),5,(0, 0, 255), -1)
            # cv2.imshow(f'feature_selection_{state}',frame)

    def interaction_matrix(self,r:np.ndarray,c:np.ndarray):  ## r,c are pixel coordinates of the point arrays

        ## note the dimension of the r and c should be (n,1) and (n,1) np arrays
        
        '''
        the camera frame is taken same as textbook which needs to be taken care as per the urdf 
        in this calculation for intercation matrix its not taken care 

        '''
        r = r.reshape((-1,1))
        c = c.reshape((-1,1))
        ox = self.cam_mtx[0,2]
        oy = self.cam_mtx[1,2]

        fx = self.cam_mtx[0,0]
        fy = self.cam_mtx[1,1]

        u = -self.sx * (r - ox)
        v = -self.sy * (c - oy)
        
        ## defining the z coordinate to be one for all the points 
        n = r.shape[0]
        z = 0.70 * np.ones((n,1))#0.74*np.ones((n,1))

        f = fx
        zeros = np.zeros((n,1))
        # print('this is r \n', r)
        # print('this is u/z \n', u/z)
        # print('this is f/z \n ', f/z)
        
        u_int_mat = np.block([-f/z, zeros, u/z, u*v/f, -((f**2 + u**2)/f), v])
        v_int_mat = np.block([zeros, -f/z, v/z, (f**2 + v**2)/f, -u*v/f, -u])

        # print(f'this is the u_int size {u_int_mat.shape} \n', u_int_mat)
        # print(f'this si the v_int size {v_int_mat.shape} \n', v_int_mat)

        interaction_matrix = np.zeros((2*n,6))
        interaction_matrix[0::2,:] = np.round(u_int_mat, 4)
        interaction_matrix[1::2,:] = np.round(v_int_mat, 4)

        # print(f'this is the interaction matrix of size {interaction_matrix.shape} \n',interaction_matrix)
        return(interaction_matrix)

    ## transformation stuff 
    def get_transformation(self, from_frame, to_frame): 
        '''
            this transformation is "from_frame" to "to_frame" which 
            takes a point defined in "to_frame" to "from_frame" 
        '''
        try:
            # Wait for transform to be available
            trans = self.tfBuffer.lookup_transform(from_frame, to_frame, rospy.Time(0), rospy.Duration(1.0))

            # Extract translation
            translation = trans.transform.translation
            t = np.array([translation.x, translation.y, translation.z])

            # Extract rotation as quaternion
            rotation = trans.transform.rotation
            q = np.array([rotation.x, rotation.y, rotation.z, rotation.w])

            # Convert quaternion to rotation matrix
            R = quaternion_to_rotation_matrix(q)

            # Construct 4x4 homogeneous transformation matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t

            return T
        
        except Exception as e:
            rospy.logerr(f"Could not get transform: {e}")
            return None

    def twist_frame_transform(self): #,twist_c_c):
        '''
            this function is defined for transforming :-
            twist of camera link with respect to camera link frame 
                                ||
                            ....||....
                             ...||...
                              ..||..
                                ..
            twist of end effector link with respect to end effector link frame 
        ''' 
        eye = np.eye(3)
        zeros = np.zeros((3,3))
        tf_efl_cl = self.get_transformation('end_effector_link', 'camera_link')
        dist_skew = skew_matrix(tf_efl_cl[:,3])

        matrix = np.block([[eye  , dist_skew],
                           [zeros, eye    ]])
        # print(matrix)
        return(matrix)
    
    ## visual servoing
    def visual_servoing(self):

        if (len(self.current_points) == 0 or len(self.desired_points) == 0):
            print('desired or current points not available ')
            return 
        
        '''
            assuming the self.desired_points and self.current_points are of 
            dim (2,n)
        '''

        Lo = 0
        L_limit = 2.3*(10**(-4))
        U_limit = 2*(10**(-2))

        k = 0.5 ## proportional constant
        twist_tf_matrix = self.twist_frame_transform()
        while (self.vs_flag == 'c'):
            # e_dot:np.ndarray
            e_dot = k*(self.current_points - self.desired_points)
            
            error = e_dot*e_dot/(k**2)
            # error = np.sum(error[:,0] + error[:,1])
            error = round(np.sqrt(np.sum(error[:,0] + error[:,1])),4)
            print("this the error ", error)

            if error<18:
                self.movement(twist_list=[0,0,0,0,0,0])
                continue


            inter_matrix = self.interaction_matrix(r = self.current_points[:,0], c = self.current_points[:,1])

            ## inverse not sure if its correct 
            # ill conditioned, rank deficient
            inv_inter_matrix = compute_interaction_matrix_inverse(inter_matrix) ## dim (6,2n)

            e_dot = e_dot.reshape((-1,1))
            ## twist is expressed as [[lin_vel], [angular_vel]]

            # print('this is e_dot \n ', e_dot)
            # print('this is self.current and self.desired \n',self.current_points,'\n',self.desired_points)
            # print(f'this is the inv_interaction matrix of size {inv_inter_matrix.shape} \n',inv_inter_matrix,)
            twist_c_c =  inv_inter_matrix @ e_dot; 

            twist_e_e = twist_tf_matrix @ twist_c_c

            # print('this is the camera twist \n',twist_c_c)
            # print('this is the ef twist \n',twist_e_e)

            # twist_e_e = np.where(np.abs(twist_e_e) <= L_limit, np.sign(twist_e_e) * Lo, 
            #                      np.where(np.abs(twist_e_e) > U_limit, np.sign(twist_e_e)* U_limit, twist_e_e))
            
            twist_e_e = np.where(np.abs(twist_e_e) <= L_limit, np.sign(twist_e_e) * Lo, 
                                 np.where(np.abs(twist_e_e) > U_limit, np.sign(twist_e_e)* U_limit, twist_e_e))
            
            twist_e_e = np.round(twist_e_e, 6)
            
            # print('this is the changed ef twist \n',twist_e_e)

            

            twist_command = list(twist_e_e.squeeze())
            self.movement(twist_list = [twist_command[0], twist_command[1], twist_command[2], twist_command[3], twist_command[4], twist_command[5]])
            rospy.sleep(0.8)
             
    def uw_fun(self):
        while (self.vs_flag == 'c'):
            print(self.vs_flag)

    def keyboard_callback(self,msg:String):

        key = msg.data

        if key == 't':
            self.twist_frame_transform()

        if key == 'v': ## for pausing / continuing visual_servoing
            if self.vs_flag == 'p':
                self.vs_flag = 'c'
                if self.vs_thread is None or not self.vs_thread.is_alive():
                    self.vs_thread = threading.Thread(target = self.visual_servoing)
                    self.vs_thread.daemon = True ## to stop the thread with the main program
                    self.vs_thread.start()
            else:
                self.vs_flag = 'p'
                print(self.vs_flag,' thread ended ')
        
        if key == '.':
            twist_list = [0, 0, 0, 0, 0, 0]
            self.movement(twist_list)

        if key == 'r': ## recording
            if self.recoding_state == "t":
                self.recoding_state = "c"
                path_vid = rf"/home/aditya/btp_kortex_ros/src/pkg_1/vid/vid_2.mp4"
                fps = 40
                height,width,channels = self.frame.shape
                size = (width,height)
                cv2_fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                self.out = cv2.VideoWriter(path_vid,cv2_fourcc,fps,size)
                print("recording started")

            if self.recoding_state == "p":
                self.recoding_state = "c"
                print("recording continuing")
            
            # if self.recoding_state == "c":
            #     self.out.write(self.frame2)     
            #     print("recording") 



        if key == "p":
            self.recoding_state = "p"
            print("recording paused")


        if key == "t":
            self.recoding_state = "t"
            self.out.release() 
            print("recording terminated ")


    ## function for sending velocities to arm 
    def movement(self,twist_list:list):  ## [angular,linear]

        self.twist.twist.angular_x = twist_list[3]
        self.twist.twist.angular_y = twist_list[4]
        self.twist.twist.angular_z = twist_list[5]

        self.twist.twist.linear_x  = twist_list[0]
        self.twist.twist.linear_y  = twist_list[1]
        self.twist.twist.linear_z  = twist_list[2]

        self.pub_arm.publish(self.twist)

## helper functions for visualization          
def draw_points(frame, points_array:Union[np.ndarray,list], color:tuple):
    if len(points_array) == 0:
        return(frame)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    text_color = (0, 0, 0) 
    i=0;
    for pt in points_array:
        i+=1
        text = str(i)
        coord = (int(pt[0]), int(pt[1]))
        cv2.putText(frame,text,coord,font,font_scale,text_color,1,cv2.LINE_AA)
        cv2.circle(frame,coord,5,color,-1)

    return(frame)

## helper functions for utils 
def load_calibration_json(filename):
    with open(filename, "r") as f:
        data = json.load(f)
    
    mtx = np.array(data["mtx"])
    dist = np.array(data["dist"])
    rvecs = [np.array(r) for r in data["rvecs"]]
    tvecs = [np.array(t) for t in data["tvecs"]]
    
    return mtx, dist, rvecs, tvecs

## helper functions for transformations
def quaternion_to_rotation_matrix(q):
    """Convert a quaternion [x, y, z, w] to a 3x3 rotation matrix."""
    x, y, z, w = q
    R = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])
    return R

def skew_matrix(vector:Union[np.ndarray,list]):
    
    if vector is np.ndarray:
        vector = vector.squeeze()

    if len(vector) == 4:
            x,y,z,_ = vector[:]
    else:
        x,y,z = vector[:]

    matrix = np.array([[0, -z, y],
                        [z, 0, -x],
                        [-y, x, 0]])
    return(matrix)

## helper function for visual servoing 
def compute_interaction_matrix_inverse(L, lambda_=0.01, rank_threshold=1e-6, cond_threshold=1e3):
    """
    Compute the inverse of the interaction matrix L for visual servoing.

    Parameters:
    - L: np.ndarray -> The interaction matrix.
    - lambda_: float -> Regularization factor for damping (default: 0.01).
    - rank_threshold: float -> Threshold for determining rank deficiency.
    - cond_threshold: float -> Condition number threshold for ill-conditioning.

    Returns:
    - L_inv: np.ndarray -> The best inverse of L based on its properties.
    """
    ## need to check this inverse calculation

    rank = np.linalg.matrix_rank(L)
    min_dim = min(L.shape)
    U, S, Vt = np.linalg.svd(L, full_matrices=False)

    # Compute condition number (max singular value / min singular value)
    cond_number = S[0] / (S[-1] if S[-1] > 1e-12 else 1e-12)

    print(f"Rank: {rank}, Condition Number: {cond_number}")

    # 1️ Full-rank case: Use Moore-Penrose pseudo-inverse
    if rank == min_dim and cond_number < cond_threshold:
        print("Using Moore-Penrose pseudo-inverse")
        return np.linalg.pinv(L)

    # 2️ Rank-deficient case: Use SVD-based inverse with thresholding
    if rank < min_dim:
        print("Rank-deficient: Using SVD-based pseudo-inverse")
        S_inv = np.diag([1/s if s > rank_threshold else 0 for s in S])
        return Vt.T @ S_inv @ U.T

    # 3️ Ill-conditioned case: Use Damped Least Squares (Tikhonov Regularization)
    if cond_number >= cond_threshold:
        print("Ill-conditioned: Using Damped Least Squares")
        return np.linalg.inv(L.T @ L + lambda_ * np.eye(L.shape[1])) @ L.T

    # Default case (fallback)
    print("Using default pseudo-inverse as fallback")
    return np.linalg.pinv(L)


if __name__ == "__main__":
    camera_config_path = fr'/home/aditya/btp_kortex_ros/src/pkg_1/checker_images/cam_300.json'
    camera_obj = camera_interaction(camera_config_path)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')
    
