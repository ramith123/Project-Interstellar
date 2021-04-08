import rospy
from std_msgs.msg import String
import unittest 



class Tester(unittest.TestCase):

    def test_cubeLocations(self):
        rospy.init_node('location_checker')
        print("Waiting for location from topic...")
        data = rospy.wait_for_message("/locations", String)
        curr_location = data.data
        self.assertEqual(curr_location, "box1,-0.476636,-0.000026,0.017914,0.001403,-0.000240,-0.000051,1.000000\n")
        

if __name__ == '__main__':
    unittest.main()