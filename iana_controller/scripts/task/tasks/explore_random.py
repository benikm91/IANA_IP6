import threading
import rospy
import time

from task.task import Task

import std_msgs.msg


class ExploreRandomTask(Task):

    def __init__(self, msg):
        super(ExploreRandomTask, self).__init__()
        self.driver_explore_enable_publisher = rospy.Publisher('/iana/driver_explore/enable', std_msgs.msg.Empty, queue_size=10)
        self.driver_explore_disable_publisher = rospy.Publisher('/iana/driver_explore/disable', std_msgs.msg.Empty, queue_size=10)
        self.running = threading.Event()
        self.until = msg.until

    @property
    def name(self):
        return "Explore Task (Only Random)"

    def update(self, elapsed):
        if time.time() > self.until:
            self.terminated.set()

    def on_start(self):
        rospy.loginfo('start random exploring')
        self.running.set()
        self.driver_explore_enable_publisher.publish()

    def on_resume(self):
        rospy.loginfo('resume random exploring')
        self.on_start()

    def on_interrupt(self):
        rospy.loginfo('interrupt random exploring')
        self.running.clear()
        self.driver_explore_disable_publisher.publish()

    def on_shutdown(self):
        rospy.loginfo('shutdown random exploring')
        self.running.clear()
        self.driver_explore_disable_publisher.publish()
        self.terminated.set()

    def interruptable_by(self, task):
        return True
