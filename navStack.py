#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist,Vector3
from sensor_msgs.msg import LaserScan
global regions
class controller:
    def __init__(self):
        """Initializes the parameters for ROS.
        """
        # Creating a velocity publisher which publishes to the the topic '/cmd_vel' topic.
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # This is the Linear velocity value.
        self.lin_speed = 0
        # This is the Angular velocity value.
        self.ang_speed = 0

    def move(self,lin_speed,ang_speed):

        """Function to publish velocities.
        Args:
            lin_speed (float): The desired linear velocity.
            ang_speed (float): The desired Angular velocity.
        """
        while not rospy.is_shutdown():
            # Creating a Twist message.
            vel = Twist()
            # Set Linear and Angular velocities.
            vel.linear = Vector3(lin_speed, 0, 0)
            vel.angular = Vector3(0, 0, ang_speed)
            for _ in range(5):
                self.pub.publish(vel)
                rospy.sleep(.1)


    def stop(self):
        """Function which stops the robot.
        """
        # Creating a Twist message.
        vel = Twist()

        # Set Linear and Angular velocities.
        vel.linear = Vector3(0, 0, 0)
        vel.angular = Vector3(0, 0, 0)
        # Publishing the message.
        for _ in range(5):
            self.pub.publish(vel)
            rospy.sleep(.1)

def laser_callback(msg):
    global regions, range_max
    range_max=msg.range_max 
    regions = {
        'bright': min(min(msg.ranges[0:143]), range_max),   # msg.ranges[0:144] ,
        'fright': min(min(msg.ranges[144:288]), range_max), # msg.ranges[144:288] ,
        'front': min(min(msg.ranges[289:432]), range_max),  # msg.ranges[288:432] ,
        'fleft': min(min(msg.ranges[433:576]), range_max),  # msg.ranges[432:576] ,
        'bleft': min(min(msg.ranges[577:720]), range_max),  # msg.ranges[576:720] ,
    }
    print(regions)

def ebot_nav():
    """Navigating through racks 
    """
    # Accessing the global variable ctrl.
    global ctrl, rate,regions
    while not rospy.is_shutdown():
        #
        # Your algorithm to navigate
        #
        if(regions['bleft'] >= 0.2 and regions['front'] >= 1 and regions['bright'] >= 0.2):
            ctrl.move(1,0)
        else:
            ctrl.stop()
        

        
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()




def main():
    """Main function.
    """
    global ctrl,regions,rate

    # Initializing node.
    rospy.init_node("ebot_controller")
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rate = rospy.Rate(10) 
    ctrl = controller()

    ebot_nav()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass