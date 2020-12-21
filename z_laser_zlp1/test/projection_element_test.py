#!/usr/bin/env python3

import unittest
import rospy
import socket
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from z_laser_msgs.msg import Figure
from z_laser_msgs.srv import ProjectionElement, ProjectionElementRequest

def ip_open(ip,port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2)
    try:
        s.connect((ip, int(port)))
        success = True
    except:
        success = False
    s.close()
    return success

class TestServiceCall(unittest.TestCase):

    def setUp(self):
        # check projector is on network
        if not ip_open("192.168.10.11",9090):
            self.skipTest("projector is not found on network")

    # test service answer correctly
    def test1_add_proj_elem(self):
        rospy.init_node("projection_element_test")
        rospy.wait_for_service("projection_start")
        # at this point we asume projector is connected
        try:
            # send projection element 
            topic = rospy.resolve_name("add_projection_element")
            pub = rospy.Publisher(topic, Figure, queue_size=1)
            rospy.sleep(1)

            # create circle
            circle = Figure()
            circle.figure_type = Figure.CIRCLE
            circle.projection_group = "figure_test"
            circle.figure_name = "circle_test"
            circle.position.x = 0      
            circle.position.y = 0        
            circle.position.z = 0        
            circle.angle.append(0)
            circle.size.append(50) # radius

            pub.publish(circle)
            rospy.sleep(1)
            
        except Exception as e:
            self.fail("%s" %e)

    # test service answer correctly
    def test2_manage_proj_elem(self):
        rospy.wait_for_service("hide_projection_element")
        rospy.wait_for_service("unhide_projection_element")
        rospy.wait_for_service("remove_projection_element")
        h = rospy.ServiceProxy('hide_projection_element', ProjectionElement)
        uh = rospy.ServiceProxy('unhide_projection_element', ProjectionElement)
        rem = rospy.ServiceProxy('remove_projection_element', ProjectionElement)
        
        req = ProjectionElementRequest()
        req.figure_type = Figure.CIRCLE
        req.projection_group = "figure_test"
        req.figure_name = "circle_test"

        try:
            resp = h(req)
            if not resp.success:
                self.fail("service should have responded successfully")
            
            resp = uh(req)
            if not resp.success:
                self.fail("service should have responded successfully")

            resp = rem(req)
            if not resp.success:
                self.fail("service should have responded successfully")
        except Exception as e:
            self.fail("service should have responded successfully")


if __name__ == '__main__':
    import rostest
    rostest.rosrun("z_laser_zlp1", 'projection_element_test', TestServiceCall)