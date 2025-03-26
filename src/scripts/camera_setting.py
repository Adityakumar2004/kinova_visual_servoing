import rospy
from std_msgs.msg import String
import rosservice
from kortex_driver.srv import DoSensorFocusAction, GetIntrinsicParameters
from kortex_driver.msg import SensorFocusAction, FocusAction, Sensor, SensorIdentifier, IntrinsicParameters, Resolution, ManualFocus 


class camera_settings():
    def __init__(self):
        rospy.init_node('camera_settings',anonymous=True)
        

        ## subscribing to keyboard topic 
        rospy.Subscriber('/keyboard_commands',String,queue_size=10,callback=self.keyboard_callback)

        ## waiting for autofocus service 
        autofocus_srv_name = "/my_gen3/vision_config/do_sensor_focus_action"
        rospy.wait_for_service(autofocus_srv_name)

        ## defining the autfocus service object 
        self.autofocus_srv = rospy.ServiceProxy(autofocus_srv_name,DoSensorFocusAction)

        ## waiting for getintrinsic parameters
        getintrin_param_srv_name = "/my_gen3/vision_config/get_intrinsic_parameters"
        rospy.wait_for_service(getintrin_param_srv_name)
        self.get_intrinsic_param_srv = rospy.ServiceProxy(getintrin_param_srv_name,GetIntrinsicParameters)

        print('im here')
        # self.get_intrinsic_parameters()


    def set_autofocus(self,state:str='off'):
               
        msg = SensorFocusAction()
        msg.sensor = Sensor.SENSOR_COLOR
       
        if state == 'off':
            msg.focus_action = FocusAction.FOCUSACTION_DISABLE_FOCUS

        if state == 'on':
            msg.focus_action = FocusAction.FOCUSACTION_START_CONTINUOUS_FOCUS

        if state == 'far_focus':  ## 300 is chosen
            # focus = int(input("give the focus "))
            self.set_autofocus(state='off')
            # print("auto focus is off ")
            focus = 300
            msg.focus_action = FocusAction.FOCUSACTION_SET_MANUAL_FOCUS
            msg.oneof_action_parameters.manual_focus = [ManualFocus(focus)]
        self.autofocus_srv(msg)

        print(f"auto focus is {state} ", f"focus is {focus}" if state=="far_focus" else "")

    def get_intrinsic_parameters(self):

        dicti_resolution = {
                0: "RESOLUTION_UNSPECIFIED",
                1: "RESOLUTION_320x240",
                2: "RESOLUTION_424x240",
                3: "RESOLUTION_480x270",
                4: "RESOLUTION_640x480",
                5: "RESOLUTION_1280x720",
                6: "RESOLUTION_1920x1080"
        }

        msg = SensorIdentifier()
        msg.sensor = Sensor.SENSOR_COLOR

        params = IntrinsicParameters()
        params = self.get_intrinsic_param_srv(msg)
        print('resolution : ',dicti_resolution[params.output.resolution])
        
        print(params)

    # def set_intrinsic_parameters(self):

    # def get_intrinsic_parameter_profile(self):



    def keyboard_callback(self,msg:String):
        key = msg.data

        if key == '1':
            self.set_autofocus('off')

        if key == '2':
            self.set_autofocus('on')

        if key == '3':
            self.get_intrinsic_parameters()

        if key == '4':
            self.set_autofocus('far_focus')


 

if __name__ == "__main__":

    camera_obj = camera_settings()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')
        

