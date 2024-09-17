from ros2node.api import get_node_names
from geometry_msgs.msg import Twist, Vector3

node_1 = 'teleop_twist_keyboard'
node_2 = 'teleop_is_on'
node_3 = 'avoiding_obstacle_is_on'

def get_ros2_node_list(publisher):

    return get_node_names(node=publisher, include_hidden_nodes=False)


def teleop_nodes_are_alive(publisher):
    
    nodes = get_ros2_node_list(publisher)
    return any(node_1 in name for name in nodes) or any(node_2 in name for name in nodes)

def avoiding_obstacle_is_on(publisher):

    nodes = get_ros2_node_list(publisher)
    return any(node_3 in name for name in nodes)


def sleep_while_teleop_is_ongoing(publisher):

    flag = True
    while teleop_nodes_are_alive(publisher):
        if flag:
            publisher.publisher.publish(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0))) # stop
            publisher.indicator_publisher_node.manualMode()
            flag = False
            publisher.get_logger().info(str("Manual Control"))
    
    if not flag:
        publisher.indicator_publisher_node.autonomousMode()


def sleep_while_avoiding_obstacle_is_on(publisher):

    flag = True
    while avoiding_obstacle_is_on(publisher):
        if flag:
            publisher.get_logger().info(str("Avoiding Obstacle"))
            flag = False

