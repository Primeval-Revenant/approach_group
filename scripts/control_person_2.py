#!/usr/bin/env python 

## Control a person in simulation. For the best ever exeperience use a gamepad with the joy package
## Alternatively, you can use WASD teleop on Rviz, publishing to /person_cmdvel

import rospy
from gazebo_msgs.msg import ModelState 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Int16


class PersonController:

    def cmd_vel_cb(self, rosdata):

        #The person's reference frame is rotated...
        self.state_msg.twist.linear.y = -1.5*rosdata.linear.x
        self.state_msg.twist.linear.x = 1.5*rosdata.linear.y
        self.state_msg.twist.angular.z = 1.5*rosdata.angular.z

    
    def joy_cb(self, joydata):

        #Convert joy to twist data
        #The person's reference frame is rotated...
        self.state_msg.twist.linear.y = -2*joydata.axes[1]
        self.state_msg.twist.linear.x = 2*joydata.axes[2]
        self.state_msg.twist.angular.z = 2*joydata.axes[0]

        #Enable/disable gaze (button 0 - triangle on Playstation-like gamepads)
        if joydata.buttons[0] > 0:
            rospy.wait_for_service('/kendon_signal_analysis/set_simgaze')
            serv = rospy.ServiceProxy('/kendon_signal_analysis/set_simgaze', Trigger)
            resp = serv.call(TriggerRequest())

        #Perform a wave as a distance salutation (button 1 - circle on Playstation-like gamepads)
        if joydata.buttons[1] > 0:
            self.gestures_int = 0
            self.salutations = 1

        #Perform a wave as a close salutation (button 2 - X on Playstation-like gamepads)
        elif joydata.buttons[2] > 0:
            self.gestures_int = 0
            self.salutations = 2

        #Perform a handshake as a close salutation (button 3 - square on Playstation-like gamepads)
        elif joydata.buttons[3] > 0:
            self.gestures_int = 1
            self.salutations = 2
        
        else:
            self.gestures_int = -1
            self.salutations = 0


    def __init__(self):

        self.state_msg = ModelState()
        self.state_msg.model_name = 'pessoa2'
        self.state_msg.reference_frame = 'pessoa2'

        self.gestures_int = -1
        self.salutations = 0
        
        #Gazebo state publisher
        self.person_publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        #cmd_vel subscriber
        self.sub = rospy.Subscriber("/person_cmdvel", Twist, self.cmd_vel_cb)


        #joy subscriber
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb)


        #Gestures and salutations publisher
        self.gest_int = rospy.Publisher("/gestures_int", Int16, queue_size=1)
        self.sal_pub = rospy.Publisher("/salutations", Int16, queue_size=1)

        #gestures and gaze subscriber for rviz
        # Eventually... maybe one day. But with a cheap gamepad = less work and more fun.

        rt = rospy.Rate(10)

        while not rospy.is_shutdown():
            
            #I need to keep publishing the data to be sure the model does not stop moving.
            self.person_publisher.publish(self.state_msg)

            self.gest_int.publish(self.gestures_int)
            self.sal_pub.publish(self.salutations)

            rt.sleep()


if __name__ == '__main__':
    rospy.init_node('PersonController')

    try:
        controller = PersonController()
    except Exception as e:
        print("Shutting down " + rospy.get_name() +": " + str(e))
    rospy.spin()