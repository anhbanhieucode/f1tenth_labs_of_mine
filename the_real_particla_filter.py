import rospy
from sensor_msgs.msg import LaserScan

class particle_filter:
    def __init__(self):
        rospy.Subscriber('/scan' , LaserScan , self.scan_callback() )
    def scan_callback(self,scan_msg):
        scan_msg.ranges
    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


        
def main():
    rospy.init_node('particle_filter')
    pf = particle_filter()
    rospy.spin()

if __name__ =='__main__':
    main()