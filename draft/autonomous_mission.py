import sys 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3

from ros2_rover.GNSS import GNSS
import ros2_rover.AR_Tag as AR_Tag
# import ros2_rover.AR_Tag_webcam as AR_Tag
import ros2_rover.Indicator as Indicator
import ros2_rover.ManualControl as ManualControl



class MyPublisher(Node):


    def __init__(self, gnss_point):

        super().__init__('autonomous_mission')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)
        self.gnss_point = gnss_point  # value from the command-line argument
        self.last_gnss = 7 # last index of gnss point
        self.indicator_publisher_node = Indicator.IndicatorNode()  # instance of the IndicatorNode
        self.GPS = GNSS()
        self.AR_posts = [3, 4, 5]



    def publish_message(self):

        # check exit status
        if self.gnss_point >  self.last_gnss:
            self.gnss_point = 1 # Reset course
            self.publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
            self.get_logger().info('Course Done !!! Use Teleop to Operate the Rover')
            self.indicator_publisher_node.manualMode()
            sys.exit(0)

        self.indicator_publisher_node.autonomousMode()
        self.GPS.navigate(publisher=self, target=self.gnss_point)
        
        if self.gnss_point in self.AR_posts:
            AR_Tag.traverse(self)

        elif self.gnss_point == 6:
            # Do not import this in Global Scope | Makes OpenCV ArUco dectection laggy
            import ros2_rover.OrangeRubberMallet as OrangeRubberMallet
            OrangeRubberMallet.traverse(self)
            
        elif self.gnss_point == 7:
            # Do not import this in Global Scope | Makes OpenCV ArUco dectection laggy
            import ros2_rover.MouthedPlasticWaterBottle as MouthedPlasticWaterBottle
            MouthedPlasticWaterBottle.traverse(self)
   
        self.indicator_publisher_node.pointReached(self) # send point number
        self.gnss_point += 1
        ManualControl.sleep_while_teleop_is_ongoing(self)
         

def wait_for_user_confirmation():
    while True:
        user_input = input('Press Enter to CONTINUE ')
        if user_input.strip() == '':
            break


def main(args=None):

    rclpy.init(args=args)    
    try: # pass the gps coordinate index 
        gnss_point = int(sys.argv[1]) if len(sys.argv) > 1 else 1  # default gnss_point = 1 | first waypoint
    except:
        gnss_point = 1

    my_publisher = MyPublisher(gnss_point)
    my_publisher.get_logger().info('WayPoint Coordinates List: \n')
    points = my_publisher.GPS.targetList
    for i in points:
        my_publisher.get_logger().info(f'{i}')
    my_publisher.get_logger().info(f'Starting with Coordinate Index: {gnss_point}') 
    my_publisher.get_logger().info('#######################################\n\n')
    wait_for_user_confirmation()


    # spinning the publisher
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
