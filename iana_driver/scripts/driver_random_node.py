#!/usr/bin/env python
import rospy
from driver_random.driver_random import DriverRandom

if __name__ == '__main__':
    try:
        rospy.init_node('iana_driver_random', anonymous=True)
        driver = DriverRandom()
        rate = rospy.Rate(10)
        last_update = rospy.get_time()
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            driver.update(current_time - last_update)
            last_update = current_time
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
